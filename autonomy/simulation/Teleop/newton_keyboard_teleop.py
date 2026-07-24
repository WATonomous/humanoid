"""
Newton / NVIDIA Warp keyboard teleoperation script for the bimanual WATO humanoid arm.

Physics engine : NVIDIA Warp + Newton (newton.solvers.SolverXPBD on CPU)
Viewer         : viser (WebSocket + Three.js browser UI at http://localhost:8080)
IK solver      : MuJoCo mj_jacBody — kept as a lightweight math dependency only

The original mjlabs_keyboard_teleop.py (MuJoCo-based) is left untouched as a fallback.

Controls:
  I/K : Move right arm along X-axis (+/-)
  J/L : Move right arm along Y-axis (+/-)
  U/O : Move right arm along Z-axis (+/-)
  G   : Toggle right finger gripper (Open/Closed)
  R   : Reset right arm to default pose

Setup (run inside the Docker container):
  pip install warp-lang newton viser trimesh
  python3 /root/ament_ws/src/simulation/Teleop/newton_keyboard_teleop.py
  → Open http://localhost:8080 in any browser
"""

import os
import sys
import time
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

# ── NVIDIA Warp & Newton (physics engine) ─────────────────────────────────────
try:
    import warp as wp
    import newton
    _WARP_AVAILABLE = True
except ImportError:
    _WARP_AVAILABLE = False
    sys.exit(
        "[ERROR] warp-lang or newton is not installed.\n"
        "Run:  pip install warp-lang newton\n"
        "Then retry."
    )

# ── Viser (standalone browser viewer) ─────────────────────────────────────────
try:
    import viser
    import viser.transforms as vtf
except ImportError:
    sys.exit(
        "[ERROR] viser is not installed.\n"
        "Run:  pip install viser\n"
        "Then retry."
    )

# ── MuJoCo — used ONLY for IK Jacobian computation ───────────────────────────
try:
    import mujoco
except ImportError:
    sys.exit(
        "[ERROR] mujoco is not installed (needed for IK Jacobians).\n"
        "Run:  pip install mujoco\n"
        "Then retry."
    )

# ── Mesh loading (optional) ───────────────────────────────────────────────────
try:
    import trimesh
    _TRIMESH_AVAILABLE = True
except ImportError:
    _TRIMESH_AVAILABLE = False
    print("[WARN] trimesh not installed — robot link meshes will not be visible. Run: pip install trimesh")

# ─────────────────────────────────────────────────────────────────────────────
# PATHS
# ─────────────────────────────────────────────────────────────────────────────
_SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH   = _SCRIPT_DIR.parent / "Humanoid_Wato" / "wato_bimanual_arm" / "urdf" / "bimanual_arm.urdf"
if not URDF_PATH.exists():
    URDF_PATH = Path("/root/ament_ws/src/simulation/Humanoid_Wato/wato_bimanual_arm/urdf/bimanual_arm.urdf")
if not URDF_PATH.exists():
    sys.exit(f"[ERROR] URDF not found at {URDF_PATH}")

# ─────────────────────────────────────────────────────────────────────────────
# SCENE CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
BLOCK_W    = 0.04        # 4 cm
BLOCK_D    = 0.04
BLOCK_H    = 0.04
BLOCK_MASS = 0.15        # 150 g

BLOCK_Ixx  = BLOCK_MASS * (BLOCK_D**2 + BLOCK_H**2) / 12
BLOCK_Iyy  = BLOCK_MASS * (BLOCK_W**2 + BLOCK_H**2) / 12
BLOCK_Izz  = BLOCK_MASS * (BLOCK_W**2 + BLOCK_D**2) / 12

PLATFORM_W  = 0.20
PLATFORM_D  = 0.20
PLATFORM_H  = 0.05

BALL_XY       = (0.44, 0.20)
BALL_INIT_POS = (BALL_XY[0], BALL_XY[1], PLATFORM_H + BLOCK_H / 2)   # 0.08 m

ADD_BLOCK = True
GRIPPER_OFFSET = np.array([0.0345, 0.10361, 0.004349])

# ─────────────────────────────────────────────────────────────────────────────
# PHYSICS CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
RENDER_HZ   = 60
SUBSTEPS    = 4
SIM_DT      = 1.0 / (RENDER_HZ * SUBSTEPS)    # 0.00417 s

HIGH_FRICTION   = 10.0          # sliding friction coefficient
CONTACT_KE      = 8_000.0       # contact spring stiffness
CONTACT_KD      = 200.0         # contact damping
BLOCK_LINEAR_DAMPING  = 2.5     # virtual air drag on free block
BLOCK_ANGULAR_DAMPING = 2.5
FINGER_FRAMES   = 4            # steps for full gripper travel
STEP_SIZE       = 0.015         # 1.5 cm per keypress

# ─────────────────────────────────────────────────────────────────────────────
# DEBUG
# ─────────────────────────────────────────────────────────────────────────────
DEBUG_JOINTS   = True   # print joint/DOF config once at startup
DEBUG_RUNTIME  = True   # print periodic runtime joint/finger state
DEBUG_INTERVAL = 60     # frames between runtime debug prints (~1x/sec at 60Hz)

# Use coordinate-layout targets in Newton to make target setting 1-to-1 with joint indices
newton.use_coord_layout_targets = True

# ─────────────────────────────────────────────────────────────────────────────
# DEFAULT ARM POSES
# ─────────────────────────────────────────────────────────────────────────────
def get_bimanual_defaults():
    right_defaults = {
        "joint1":  np.deg2rad(-139.2),
        "joint2":  np.deg2rad(66.1),
        "joint3":  np.deg2rad(-147.9),
        "joint4":  np.deg2rad(76.5),
        "joint5":  np.deg2rad(76.5),
        "joint6":  np.deg2rad(22.6),
        "joint7":  -0.05,
        "joint8":  0.05,
    }
    left_defaults = {
        "joint1L": np.deg2rad(140.8),
        "joint2l": np.deg2rad(55.7),
        "joint3l": np.deg2rad(66.0),
        "joint4l": np.deg2rad(-111.4),
        "joint5l": np.deg2rad(-34.8),
        "joint6l": np.deg2rad(-3.5),
        "joint7l": -0.05,
        "joint8l": 0.05,
    }
    return right_defaults, left_defaults


# ─────────────────────────────────────────────────────────────────────────────
# MUJOCO IK MODEL  (physics-free — Jacobian math only)
# ─────────────────────────────────────────────────────────────────────────────
def build_mujoco_ik_model():
    path = Path(URDF_PATH).resolve()
    mesh_dir = path.parent.parent / "meshes"
    robot_xml = path.read_text().replace("../meshes", str(mesh_dir))
    robot_root = ET.fromstring(robot_xml)

    # Add a fixed floor so mj_forward works
    fl = ET.SubElement(robot_root, "link", {"name": "ground_ik"})
    fi = ET.SubElement(fl, "inertial")
    ET.SubElement(fi, "mass", {"value": "1.0"})
    ET.SubElement(fi, "origin", {"xyz": "0 0 0"})
    ET.SubElement(fi, "inertia", {"ixx": "1", "ixy": "0", "ixz": "0",
                                  "iyy": "1", "iyz": "0", "izz": "1"})
    fj = ET.SubElement(robot_root, "joint", {"name": "floor_ik_joint", "type": "fixed"})
    ET.SubElement(fj, "parent", {"link": "base_link"})
    ET.SubElement(fj, "child", {"link": "ground_ik"})
    ET.SubElement(fj, "origin", {"xyz": "0 0 -0.01", "rpy": "0 0 0"})

    mj_elem = ET.SubElement(robot_root, "mujoco")
    ET.SubElement(mj_elem, "compiler", {"balanceInertia": "true", "discardvisual": "false"})

    xml_str = ET.tostring(robot_root, encoding="utf-8").decode("utf-8")
    return mujoco.MjModel.from_xml_string(xml_str)


def solve_palm_ik(mj_model, mj_data, target_pos, wrist_angle,
                  damping=1e-3, step=0.5, max_iter=150, tol=1e-4):
    """Solve translation IK for the RIGHT arm end effector (link6).

    The right arm's 6 joints (joint1..joint6) occupy qpos[0:6] in the
    URDF-compiled MuJoCo model, since the URDF declares the right-arm
    chain before the left-arm chain.
    """
    body_id = mj_model.body("link6").id
    nv      = mj_model.nv
    for _ in range(max_iter):
        mj_data.qpos[5] = wrist_angle
        mujoco.mj_forward(mj_model, mj_data)
        pos = mj_data.xpos[body_id].copy() + mj_data.xmat[body_id].reshape(3, 3) @ GRIPPER_OFFSET
        err = target_pos - pos
        if np.max(np.abs(err)) < tol:
            break
        jacp = np.zeros((3, nv))
        mujoco.mj_jac(mj_model, mj_data, jacp, None, pos, body_id)
        J   = jacp[:, 0:5]
        dq  = np.linalg.solve(J.T @ J + damping * np.eye(5), J.T @ err)
        mj_data.qpos[0:5] += step * dq
    mj_data.qpos[5] = wrist_angle


# ─────────────────────────────────────────────────────────────────────────────
# NEWTON PHYSICS MODEL
# ─────────────────────────────────────────────────────────────────────────────
def build_warp_model():
    wp.init()
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)

    # Configure default shape physics (friction, stiffness, damping)
    shape_cfg = newton.ModelBuilder.ShapeConfig(
        mu=HIGH_FRICTION,
        ke=CONTACT_KE,
        kd=CONTACT_KD,
    )
    builder.default_shape_cfg = shape_cfg

    # ── Parse robot URDF ─────────────────────────────────────────────────────
    path = Path(URDF_PATH).resolve()
    builder.add_urdf(
        str(path),
        xform=wp.transform([0.0, 0.0, 0.0], wp.quat_identity()),
        floating=False,          # fixed base
        enable_self_collisions=False,
    )

    # ------------------------------------------------------------------
    # Fix bogus joint limits/effort/velocity baked into the SolidWorks-
    # exported URDF.
    #
    # bimanual_arm.urdf puts a placeholder
    #   <limit lower="0" upper="0" effort="0" velocity="0"/>
    # on EVERY joint, real limit or not (standard SolidWorks URDF export
    # behavior). MuJoCo's URDF importer treats lower==upper==0 as "no limit
    # specified" and ALSO ignores effort=0/velocity=0 the same way, so those
    # joints end up fully free and drivable -- which is why the MuJoCo
    # teleop script moves. Newton's URDF importer takes every one of these
    # values literally:
    #   - limit_lower=0/limit_upper=0  -> locks the joint to exactly 0 rad
    #     via a stiff joint-limit constraint.
    #   - effort_limit=0               -> clamps the PD drive's max torque
    #     to ZERO, so even with the position limits widened the actuator
    #     can produce no corrective force at all. This is why the arm
    #     didn't track the target and instead sagged under gravity at
    #     whichever joint had the least resistance (not a CPU/GPU issue).
    #   - velocity_limit=0             -> would also cap max joint speed
    #     to zero if left as-is.
    #
    # Fix: give the 6 arm joints per side a wide-open rotation range with
    # real effort/velocity limits, and give the 2 finger joints per side
    # their real (small) travel range + modest effort/velocity limits,
    # instead of the degenerate URDF placeholders. Joint index -> DOF index
    # is resolved via joint_qd_start since add_urdf() also inserts a 0-DOF
    # fixed base joint that would otherwise offset a naive index mapping.
    FINGER_LIMITS = {
        "joint7":  (-0.05, 0.0),
        "joint8":  (0.0, 0.05),
        "joint7l": (-0.05, 0.0),
        "joint8l": (0.0, 0.05),
    }
    WIDE_LIMIT      = 4.0 * np.pi   # generous rotation range for arm joints
    ARM_EFFORT_LIMIT     = 200.0    # N·m, generous torque cap for arm joints
    ARM_VELOCITY_LIMIT   = 4.0      # rad/s
    FINGER_EFFORT_LIMIT   = 50.0    # N, linear force cap for prismatic fingers
    FINGER_VELOCITY_LIMIT = 0.5     # m/s
    for i, lbl in enumerate(builder.joint_label):
        name = lbl.split("/")[-1]
        if name in FINGER_LIMITS:
            lo, hi = FINGER_LIMITS[name]
            effort, vel = FINGER_EFFORT_LIMIT, FINGER_VELOCITY_LIMIT
        elif name.startswith("joint"):
            lo, hi = -WIDE_LIMIT, WIDE_LIMIT
            effort, vel = ARM_EFFORT_LIMIT, ARM_VELOCITY_LIMIT
        else:
            continue
        dof_idx = builder.joint_qd_start[i]
        builder.joint_limit_lower[dof_idx]   = lo
        builder.joint_limit_upper[dof_idx]   = hi
        builder.joint_effort_limit[dof_idx]  = effort
        builder.joint_velocity_limit[dof_idx] = vel

    # Set PD drive gains and armature directly on the builder.
    #
    # NOTE: joint_target_ke MUST be nonzero. These joints are driven by a real
    # PD position drive (ke = stiffness, kd = damping) toward `ctrl.joint_target_q`
    # every substep. With ke=0 there is no restoring force at all, so the only
    # thing holding a joint at its target is velocity damping — gravity and
    # contact forces are free to push it away.
    # IMPORTANT: add_urdf() already computed joint_target_mode PER-DOF at
    # import time, based on the ke/kd gains that existed then (both 0.0,
    # since URDF only carries <dynamics damping>, never stiffness). That
    # means every DOF was assigned JointTargetMode.NONE. Overwriting
    # joint_target_ke/kd afterwards (below) does NOT retroactively change
    # joint_target_mode -- with mode=NONE the solver ignores the drive
    # gains entirely, no matter their value. This was the actual reason
    # q_actual/qd_actual never moved despite q_target updating every frame:
    # there was no active drive at all, not a threading/index/IK issue.
    ARM_JOINT_KE = 4000.0
    ARM_JOINT_KD = 120.0
    for i in range(len(builder.joint_target_ke)):
        builder.joint_target_ke[i] = ARM_JOINT_KE
        builder.joint_target_kd[i] = ARM_JOINT_KD
        builder.joint_armature[i] = 0.05
        builder.joint_target_mode[i] = int(newton.JointTargetMode.POSITION)

    # Robot body/joint names
    robot_body_names  = list(builder.body_label)
    robot_joint_names = list(builder.joint_label)

    # ── Free-floating grab block ──────────────────────────────────────────────
    block_body_idx = None
    if ADD_BLOCK:
        block_body_idx = builder.body_count
        builder.add_body(
            xform=wp.transform(list(BALL_INIT_POS), wp.quat_identity()),
            mass=BLOCK_MASS,
            inertia=wp.mat33(np.diag([BLOCK_Ixx, BLOCK_Iyy, BLOCK_Izz]).tolist()),
            label="tennis_ball"
        )
        builder.add_shape_box(
            body=block_body_idx,
            hx=BLOCK_W / 2, hy=BLOCK_D / 2, hz=BLOCK_H / 2,
            cfg=shape_cfg
        )

    # ── Platform (attached to static world) ───────────────────────────────────
    builder.add_shape_box(
        body=-1,
        xform=wp.transform([BALL_XY[0], BALL_XY[1], PLATFORM_H / 2], wp.quat_identity()),
        hx=PLATFORM_W / 2, hy=PLATFORM_D / 2, hz=PLATFORM_H / 2,
        cfg=shape_cfg
    )

    # ── Ground Plane (attached to static world) ───────────────────────────────
    ground_cfg = newton.ModelBuilder.ShapeConfig(mu=2.0, ke=CONTACT_KE, kd=CONTACT_KD)
    builder.add_shape_box(
        body=-1,
        xform=wp.transform([BALL_XY[0], BALL_XY[1], -0.05], wp.quat_identity()),
        hx=1.0, hy=1.0, hz=0.01,
        cfg=ground_cfg
    )

    # Finalize on CPU
    model = builder.finalize()


    extra_names = []
    if ADD_BLOCK:
        extra_names.append("tennis_ball")
    extra_names += ["ball_platform", "ground_plane"]
    all_body_names  = robot_body_names + extra_names
    all_joint_names = robot_joint_names

    return model, all_joint_names, all_body_names, block_body_idx


# ─────────────────────────────────────────────────────────────────────────────
# VISER MESH LOADING
# ─────────────────────────────────────────────────────────────────────────────
def load_meshes_into_viser(server, urdf_path):
    if not _TRIMESH_AVAILABLE:
        return {}

    path     = Path(urdf_path).resolve()
    mesh_dir = path.parent.parent / "meshes"
    xml_str  = path.read_text().replace("../meshes", str(mesh_dir))
    root_el  = ET.fromstring(xml_str)

    handles = {}
    for link in root_el.findall("link"):
        link_name = link.get("name", "")
        for visual in link.findall("visual"):
            geom = visual.find("geometry")
            if geom is None:
                continue
            mesh_el = geom.find("mesh")
            if mesh_el is None:
                continue

            mesh_file = mesh_el.get("filename", "")
            mesh_path = Path(mesh_file)
            if not mesh_path.is_absolute():
                mesh_path = path.parent / mesh_file
            if not mesh_path.exists():
                continue

            try:
                raw = trimesh.load(str(mesh_path), force="mesh", process=False)
                if isinstance(raw, trimesh.Scene):
                    parts = raw.dump(concatenate=True)
                    raw   = parts if isinstance(parts, trimesh.Trimesh) else None
                if raw is None or len(raw.vertices) == 0:
                    continue

                scale_str = mesh_el.get("scale", "1 1 1")
                scale = [float(v) for v in scale_str.split()]
                raw.apply_scale(scale)

                origin_el = visual.find("origin")
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]
                if origin_el is not None:
                    xyz = [float(v) for v in origin_el.get("xyz", "0 0 0").split()]
                    rpy = [float(v) for v in origin_el.get("rpy", "0 0 0").split()]

                T = np.eye(4)
                T[:3, :3] = vtf.SO3.from_rpy_radians(*rpy).as_matrix()
                T[:3, 3]  = xyz
                raw.apply_transform(T)

                handle = server.scene.add_mesh_trimesh(
                    f"/bodies/{link_name}/mesh",
                    raw,
                    wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
                    position=np.array([0.0, 0.0, 0.0]),
                )
                handles[link_name] = handle
            except Exception as exc:
                print(f"[WARN] Mesh load failed for {link_name}: {exc}")

    return handles


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("[INFO] Building Newton physics model...")
    model, joint_names, body_names, block_body_idx = build_warp_model()

    state_cur = model.state()
    state_nxt = model.state()

    print("[INFO] Initializing Newton XPBD CPU Solver...")
    # More iterations so the fairly stiff contact model (CONTACT_KE=8000) has
    # room to converge within a substep instead of overshooting/jittering.
    solver = newton.solvers.SolverXPBD(model, iterations=20)
    pipeline = newton.CollisionPipeline(model)
    contacts = newton.Contacts(1000, 0)
    ctrl = model.control()

    # ── MuJoCo IK model ──────────────────────────────────────────────────────
    print("[INFO] Building MuJoCo IK model...")
    mj_model = build_mujoco_ik_model()
    mj_data  = mujoco.MjData(mj_model)

    right_defaults, left_defaults = get_bimanual_defaults()

    def apply_mj_defaults(d):
        for name, val in {**right_defaults, **left_defaults}.items():
            try:
                d.qpos[mj_model.joint(name).qposadr[0]] = val
            except KeyError:
                pass

    apply_mj_defaults(mj_data)
    mujoco.mj_forward(mj_model, mj_data)
    palm_body_id     = mj_model.body("link6").id
    initial_palm_pos = mj_data.xpos[palm_body_id].copy() + mj_data.xmat[palm_body_id].reshape(3, 3) @ GRIPPER_OFFSET

    # Dedicated MuJoCo scratch data used ONLY to cross-check Newton's FK.
    # Given the SAME right-arm joint angles that Newton is actually holding,
    # this lets us compute "what MuJoCo thinks link1..link6 should be at"
    # and diff it directly against Newton's simulated body_q for the same
    # links, to localize exactly which joint's kinematics diverge.
    mj_debug_data = mujoco.MjData(mj_model)
    apply_mj_defaults(mj_debug_data)
    RIGHT_CHAIN_LINKS = ["link1", "link2", "link3", "link4", "link5", "link6"]
    mj_chain_body_ids = {name: mj_model.body(name).id for name in RIGHT_CHAIN_LINKS}

    # Joint index mapping
    #
    # IMPORTANT: model.joint_label[i] is indexed by JOINT index, not by
    # coordinate (joint_q) index. add_urdf() inserts a 0-DOF FIXED base
    # joint at index 0 (to anchor the robot to the world) before joint1,
    # so joint index != coordinate index once you're past that base joint
    # (everything is shifted by however many joints precede it that
    # contribute 0 coordinates). Since `newton.use_coord_layout_targets =
    # True`, ctrl.joint_target_q and state_cur.joint_q are COORDINATE-
    # layout arrays, so we must map joint name -> coordinate start index
    # via model.joint_q_start, not the raw joint index. Using the raw
    # joint index here was writing each joint's target into the wrong
    # coordinate slot (off by one, cascading through the rest of the
    # chain), which is why only one joint appeared to move.
    joint_q_start_np = model.joint_q_start.numpy()
    jname_to_qidx = {}
    for i in range(model.joint_count):
        lbl = model.joint_label[i]
        # Clean label (removes world path prefix if present)
        name = lbl.split("/")[-1]
        jname_to_qidx[name] = int(joint_q_start_np[i])

    RIGHT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    LEFT_ARM_JOINTS  = list(left_defaults.keys())
    FINGER_R_JOINTS  = ["joint7", "joint8"]

    right_arm_nidxs = [jname_to_qidx[n] for n in RIGHT_ARM_JOINTS if n in jname_to_qidx]
    left_arm_nidxs  = [jname_to_qidx[n] for n in LEFT_ARM_JOINTS  if n in jname_to_qidx]
    left_arm_vals   = np.array([left_defaults[n] for n in LEFT_ARM_JOINTS if n in jname_to_qidx])
    finger_r_nidxs  = [jname_to_qidx[n] for n in FINGER_R_JOINTS  if n in jname_to_qidx]

    # Seed initial joint targets and state (coordinate-layout, see note above)
    q0 = state_cur.joint_q.numpy()
    for name, val in {**right_defaults, **left_defaults}.items():
        idx = jname_to_qidx.get(name)
        if idx is not None:
            q0[idx] = val

    # ── DEBUG: dump joint/DOF/coordinate index mapping ──────────────────────
    if DEBUG_JOINTS:
        joint_qd_start_np = model.joint_qd_start.numpy()
        limit_lower_np    = model.joint_limit_lower.numpy()
        limit_upper_np    = model.joint_limit_upper.numpy()
        effort_limit_np   = model.joint_effort_limit.numpy()
        target_ke_np      = model.joint_target_ke.numpy()
        target_kd_np      = model.joint_target_kd.numpy()
        print("\n" + "=" * 100)
        print("[DEBUG] Joint / coordinate / DOF index mapping (model.joint_count = "
              f"{model.joint_count}, joint_coord_count = {len(q0)}, "
              f"joint_dof_count = {len(limit_lower_np)})")
        target_mode_np_dbg = model.joint_target_mode.numpy()
        print(f"{'j_idx':>5} {'name':<12} {'q_idx':>6} {'qd_idx':>7} "
              f"{'limit_lo':>10} {'limit_hi':>10} {'effort':>8} {'ke':>8} {'kd':>8} {'mode':>5}")
        for i in range(model.joint_count):
            name = model.joint_label[i].split("/")[-1]
            q_idx  = int(joint_q_start_np[i])
            qd_idx = int(joint_qd_start_np[i])
            # Guard against 0-DOF joints (e.g. the fixed base joint) when
            # indexing the per-DOF arrays.
            if qd_idx < len(limit_lower_np):
                lo, hi, eff = limit_lower_np[qd_idx], limit_upper_np[qd_idx], effort_limit_np[qd_idx]
                ke, kd = target_ke_np[qd_idx], target_kd_np[qd_idx]
                mode = int(target_mode_np_dbg[qd_idx])
            else:
                lo, hi, eff, ke, kd, mode = float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), -1
            print(f"{i:>5} {name:<12} {q_idx:>6} {qd_idx:>7} "
                  f"{lo:>10.3f} {hi:>10.3f} {eff:>8.1f} {ke:>8.1f} {kd:>8.1f} {mode:>5}")
        print("[DEBUG] joint_target_mode legend: check newton.JointTargetMode enum "
              "(commonly NONE=0). If arm joints show mode=0 (NONE) despite nonzero ke/kd, "
              "the drive is not actually enabled and must be set explicitly.")
        print("-" * 100)
        print(f"[DEBUG] right_arm_nidxs (coord idx) = {right_arm_nidxs}  for {RIGHT_ARM_JOINTS}")
        print(f"[DEBUG] left_arm_nidxs  (coord idx) = {left_arm_nidxs}  for {LEFT_ARM_JOINTS}")
        print(f"[DEBUG] finger_r_nidxs  (coord idx) = {finger_r_nidxs}  for {FINGER_R_JOINTS}")
        print(f"[DEBUG] initial_palm_pos (link6, IK model) = {initial_palm_pos}")

        # ── DEBUG: check for BodyFlags.KINEMATIC on robot links ──────────
        # SolverXPBD's copy_kinematic_body_state_kernel forcibly overwrites
        # body_q/body_qd for any body flagged KINEMATIC with the PREVIOUS
        # step's values, completely bypassing any joint-drive correction.
        # If robot links got flagged KINEMATIC instead of DYNAMIC (e.g. via
        # add_urdf defaults, or inherited from the fixed base joint), this
        # would exactly explain q_actual/qd_actual never changing despite
        # correct target_ke/kd and target values -- independent of indexing,
        # drive mode, or threading.
        body_flags_np = model.body_flags.numpy()
        body_mass_np  = model.body_mass.numpy()
        KINEMATIC_BIT = 1 << 1  # BodyFlags.KINEMATIC (verify against printed value below)
        try:
            import newton as _newton_mod
            KINEMATIC_BIT = int(_newton_mod.BodyFlags.KINEMATIC)
        except Exception as exc:
            print(f"[DEBUG] Could not import newton.BodyFlags directly: {exc}")
        print(f"[DEBUG] BodyFlags.KINEMATIC bit value = {KINEMATIC_BIT}")
        print(f"{'b_idx':>5} {'name':<12} {'flags':>6} {'is_kinematic':>13} {'mass':>10}")
        for i in range(model.body_count):
            bname = model.body_label[i].split("/")[-1]
            if bname in ("link1", "link2", "link3", "link4", "link5", "link6",
                         "link7", "link8", "base_link"):
                flags = int(body_flags_np[i])
                is_kin = bool(flags & KINEMATIC_BIT)
                print(f"{i:>5} {bname:<12} {flags:>6} {is_kin!s:>13} {body_mass_np[i]:>10.4f}")
        print("=" * 100 + "\n")

    # Run Forward Kinematics to populate body positions
    newton.eval_fk(model, state_cur.joint_q, state_cur.joint_qd, state_cur)
    newton.eval_fk(model, state_nxt.joint_q, state_nxt.joint_qd, state_nxt)

    # ── DEBUG: isolated, synchronous solver self-test ───────────────────────
    # Runs BEFORE any threads/viser start. Sets an extreme target on joint1
    # only, steps the solver directly N times with no other code involved,
    # and prints joint_q before/after each chunk. This isolates whether
    # solver.step() ever applies ANY drive force to move a joint toward
    # ctrl.joint_target_q -- independent of threading, viser, or the IK loop.
    if DEBUG_JOINTS:
        print("[DEBUG] Running isolated solver self-test on joint1 "
              "(no threads, no viser, direct solver.step calls)...")
        test_q_idx = jname_to_qidx["joint1"]
        q_before = state_cur.joint_q.numpy()[test_q_idx]
        target_mode_np = model.joint_target_mode.numpy()
        target_ke_np2  = model.joint_target_ke.numpy()
        test_qd_idx = None
        joint_qd_start_np2 = model.joint_qd_start.numpy()
        for jj in range(model.joint_count):
            if model.joint_label[jj].split("/")[-1] == "joint1":
                test_qd_idx = int(joint_qd_start_np2[jj])
                break
        print(f"[DEBUG] joint1: q_idx={test_q_idx}, qd_idx={test_qd_idx}, "
              f"target_mode={target_mode_np[test_qd_idx] if test_qd_idx is not None else 'N/A'}, "
              f"target_ke={target_ke_np2[test_qd_idx] if test_qd_idx is not None else 'N/A'}, "
              f"q_before={q_before:.4f}")
        test_target = q_before + 1.0  # force a large 1 rad step
        for chunk in range(5):
            q_target_arr = ctrl.joint_target_q.numpy()
            q_target_arr[test_q_idx] = test_target
            for _ in range(60):  # 60 substeps ~ 0.25s of sim time at SIM_DT
                newton.eval_fk(model, state_cur.joint_q, state_cur.joint_qd, state_cur)
                pipeline.collide(state_cur, contacts)
                solver.step(state_cur, state_nxt, ctrl, contacts, SIM_DT)
                state_cur, state_nxt = state_nxt, state_cur
                newton.eval_ik(model, state_cur, state_cur.joint_q, state_cur.joint_qd)
            q_now  = state_cur.joint_q.numpy()[test_q_idx]
            qd_now = state_cur.joint_qd.numpy()[test_qd_idx] if test_qd_idx is not None else float("nan")
            print(f"[DEBUG] self-test chunk {chunk}: target={test_target:.4f}  "
                  f"q_actual={q_now:.4f}  qd_actual={qd_now:.4f}")
        print("[DEBUG] Self-test complete. If q_actual never moved toward "
              f"target={test_target:.4f} from q_before={q_before:.4f}, "
              "the solver/drive setup itself is broken (not a threading/IK issue).\n")
        # Restore default pose before starting the real sim loop.
        q0_restore = state_cur.joint_q.numpy()
        for name, val in {**right_defaults, **left_defaults}.items():
            idx = jname_to_qidx.get(name)
            if idx is not None:
                q0_restore[idx] = val
        state_cur.joint_qd.numpy()[:] = 0.0
        newton.eval_fk(model, state_cur.joint_q, state_cur.joint_qd, state_cur)
        newton.eval_fk(model, state_nxt.joint_q, state_nxt.joint_qd, state_nxt)

    # ── Viser server ──────────────────────────────────────────────────────────
    print("[INFO] Starting viser server on port 8080...")
    server = viser.ViserServer(port=8080, verbose=False)

    print("[INFO] Loading robot meshes into viewer...")
    _mesh_handles = load_meshes_into_viser(server, URDF_PATH)
    n_robot_meshes = len(_mesh_handles)
    print(f"[INFO] Loaded {n_robot_meshes} link meshes.")

    # Initiser body visual frames
    body_frame_handles = {}
    body_q_np = state_cur.body_q.numpy()
    for i in range(model.body_count):
        lbl = model.body_label[i]
        bname = lbl.split("/")[-1]
        t = body_q_np[i]
        pos = np.array(t[:3])
        wxyz = np.array([float(t[6]), float(t[3]), float(t[4]), float(t[5])])
        body_frame_handles[bname] = server.scene.add_frame(
            f"/bodies/{bname}",
            position=pos,
            wxyz=wxyz,
            show_axes=False,
        )

    if ADD_BLOCK:
        block_vis = server.scene.add_box(
            "/scene/tennis_ball",
            dimensions=(BLOCK_W, BLOCK_D, BLOCK_H),
            color=(38, 115, 230),
            position=np.array(BALL_INIT_POS),
            wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
    platform_vis = server.scene.add_box(
        "/scene/ball_platform",
        dimensions=(PLATFORM_W, PLATFORM_D, PLATFORM_H),
        color=(217, 107, 25),
        position=np.array([BALL_XY[0], BALL_XY[1], PLATFORM_H / 2]),
        wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )

    target_frame_handle = server.scene.add_frame(
        "/target_palm",
        axes_length=0.15,
        axes_radius=0.008,
        position=np.array(initial_palm_pos),
    )

    # ── DEBUG: resolve Newton body indices for the right palm/fingers ───────
    # Used at runtime to print the ACTUAL simulated position of the palm and
    # fingertips (from Newton's body_q), so it can be compared directly
    # against the IK target and the MuJoCo IK model's palm estimate.
    newton_body_name_to_idx = {}
    for i in range(model.body_count):
        bname = model.body_label[i].split("/")[-1]
        newton_body_name_to_idx[bname] = i
    right_palm_body_idx   = newton_body_name_to_idx.get("link6")
    right_finger7_body_idx = newton_body_name_to_idx.get("link7")
    right_finger8_body_idx = newton_body_name_to_idx.get("link8")
    if DEBUG_JOINTS:
        print(f"[DEBUG] Newton body indices -> link6: {right_palm_body_idx}, "
              f"link7: {right_finger7_body_idx}, link8: {right_finger8_body_idx}")

    # ── Shared state ──────────────────────────────────────────────────────────
    lock = threading.Lock()
    sim_state = {
        "target_pos":         initial_palm_pos.copy(),
        "current_ik_target":  initial_palm_pos.copy(),
        "is_curled":          False,
        "current_curl":       0.0,
        "arm_joint_targets":  np.array([right_defaults[n] for n in RIGHT_ARM_JOINTS]),
        "ik_mj_data":         mujoco.MjData(mj_model),
        "wrist_angle":        right_defaults["joint6"],
    }
    apply_mj_defaults(sim_state["ik_mj_data"])

    # ── Physics and Render Loop ───────────────────────────────────────────────
    def physics_and_render_loop():
        nonlocal state_cur, state_nxt
        frame_count = 0
        while True:
            t0 = time.perf_counter()
            frame_count += 1

            with lock:
                curl        = sim_state["current_curl"]
                target_curl = 1.0 if sim_state["is_curled"] else 0.0
                if curl < target_curl:
                    sim_state["current_curl"] = min(target_curl, curl + 1.0 / FINGER_FRAMES)
                elif curl > target_curl:
                    sim_state["current_curl"] = max(target_curl, curl - 1.0 / FINGER_FRAMES)
                curl         = sim_state["current_curl"]
                arm_targets  = sim_state["arm_joint_targets"].copy()

            f1_target = -0.05 * (1.0 - curl)
            f2_target =  0.05 * (1.0 - curl)

            for _ in range(SUBSTEPS):
                q_target = ctrl.joint_target_q.numpy()

                # Drive the arm/fingers via the PD position targets only.
                # Do NOT hard-write joint_q/joint_qd here — that fights the
                # solver's own contact/dynamics resolution and is what caused
                # the visible jitter/snapping. The PD drive (joint_target_ke/kd,
                # set in build_warp_model) is now strong enough to hold the arm
                # at its target while still letting the solver integrate it
                # smoothly and react to contacts.
                for idx, val in zip(right_arm_nidxs, arm_targets):
                    q_target[idx] = val
                for idx, val in zip(left_arm_nidxs, left_arm_vals):
                    q_target[idx] = val
                if len(finger_r_nidxs) >= 2:
                    q_target[finger_r_nidxs[0]] = f1_target
                    q_target[finger_r_nidxs[1]] = f2_target

                # Run Forward Kinematics to refresh body_q for collision.
                newton.eval_fk(model, state_cur.joint_q, state_cur.joint_qd, state_cur)

                # Collide & Step
                pipeline.collide(state_cur, contacts)
                solver.step(state_cur, state_nxt, ctrl, contacts, SIM_DT)
                state_cur, state_nxt = state_nxt, state_cur
                newton.eval_ik(model, state_cur, state_cur.joint_q, state_cur.joint_qd)

            # Update visualiser positions
            bq = state_cur.body_q.numpy()
            for i in range(model.body_count):
                lbl = model.body_label[i]
                bname = lbl.split("/")[-1]
                if bname in body_frame_handles:
                    t = bq[i]
                    body_frame_handles[bname].position = np.array(t[:3])
                    body_frame_handles[bname].wxyz     = np.array([float(t[6]), float(t[3]), float(t[4]), float(t[5])])

            if ADD_BLOCK and block_body_idx is not None and block_body_idx < len(bq):
                t = bq[block_body_idx]
                block_vis.position = np.array(t[:3])
                block_vis.wxyz     = np.array([float(t[6]), float(t[3]), float(t[4]), float(t[5])])

            # ── DEBUG: periodic runtime dump of target vs actual joint state ──
            if DEBUG_RUNTIME and frame_count % DEBUG_INTERVAL == 0:
                with lock:
                    dbg_target_pos = sim_state["target_pos"].copy()
                    dbg_ik_target  = sim_state["current_ik_target"].copy()
                    dbg_arm_targets = arm_targets.copy()
                q_actual  = state_cur.joint_q.numpy()
                qd_actual = state_cur.joint_qd.numpy()
                q_tgt_now = ctrl.joint_target_q.numpy()
                print(f"\n[RUNTIME t={frame_count / RENDER_HZ:6.1f}s] "
                      f"RGB target_pos={np.round(dbg_target_pos, 4)}  "
                      f"ik_target={np.round(dbg_ik_target, 4)}")
                print("  right arm: name   q_target(ik)  q_target(ctrl)   q_actual   qd_actual")
                for j_name, c_idx, ik_val in zip(RIGHT_ARM_JOINTS, right_arm_nidxs, dbg_arm_targets):
                    print(f"    {j_name:<8} {ik_val:>10.4f}   {q_tgt_now[c_idx]:>10.4f}   "
                          f"{q_actual[c_idx]:>10.4f}   {qd_actual[c_idx]:>10.4f}")
                if right_palm_body_idx is not None:
                    palm_t = bq[right_palm_body_idx]
                    p_wrist = np.array(palm_t[:3])
                    q_wrist = np.array(palm_t[3:7])  # [qx, qy, qz, qw]
                    # Rotate offset by quaternion
                    q_xyz = q_wrist[:3]
                    q_w = q_wrist[3]
                    cross1 = np.cross(q_xyz, GRIPPER_OFFSET) + q_w * GRIPPER_OFFSET
                    p_tip = p_wrist + 2.0 * np.cross(q_xyz, cross1)
                    print(f"  Newton actual gripper tip pos = {np.round(p_tip, 4)}  "
                           f"vs RGB target_pos = {np.round(dbg_target_pos, 4)}  "
                           f"dist = {np.linalg.norm(p_tip - dbg_target_pos):.4f} m")
                if right_finger7_body_idx is not None and right_finger8_body_idx is not None:
                    f7_t = bq[right_finger7_body_idx]
                    f8_t = bq[right_finger8_body_idx]
                    print(f"  Newton link7 (finger) pos = {np.round(np.array(f7_t[:3]), 4)}   "
                          f"link8 (finger) pos = {np.round(np.array(f8_t[:3]), 4)}")

                # ── DEBUG: cross-check FK link-by-link, MuJoCo vs Newton,
                # using the EXACT SAME right-arm joint angles Newton is
                # currently holding (q_actual). This localizes exactly which
                # link/joint diverges between the two kinematic chains.
                mj_debug_data.qpos[0:6] = [q_actual[idx] for idx in right_arm_nidxs]
                mujoco.mj_forward(mj_model, mj_debug_data)
                print("  --- FK cross-check (same joint angles) ---")
                print(f"    {'link':<10} {'MuJoCo xpos':<28} {'Newton body_q pos':<28} {'diff':>8}")
                for lname in RIGHT_CHAIN_LINKS:
                    mj_pos = mj_debug_data.xpos[mj_chain_body_ids[lname]].copy()
                    n_idx  = newton_body_name_to_idx.get(lname)
                    if n_idx is not None:
                        n_pos = np.array(bq[n_idx][:3])
                        diff  = np.linalg.norm(mj_pos - n_pos)
                        print(f"    {lname:<10} {np.round(mj_pos, 4)!s:<28} "
                              f"{np.round(n_pos, 4)!s:<28} {diff:>8.4f}")
                    else:
                        print(f"    {lname:<10} {np.round(mj_pos, 4)!s:<28} "
                              f"{'<no matching Newton body>':<28}")

            elapsed = time.perf_counter() - t0
            time.sleep(max(0.0, 1.0 / RENDER_HZ - elapsed))

    # ── IK Loop ───────────────────────────────────────────────────────────────
    def ik_loop():
        rate = 1.0 / 60.0
        ik   = sim_state["ik_mj_data"]
        ik_frame_count = 0
        while True:
            ik_frame_count += 1
            with lock:
                diff = sim_state["target_pos"] - sim_state["current_ik_target"]
                if np.linalg.norm(diff) > 1e-5:
                    sim_state["current_ik_target"] += diff * 0.12

                solve_palm_ik(mj_model, ik, sim_state["current_ik_target"], sim_state["wrist_angle"])
                sim_state["arm_joint_targets"] = ik.qpos[0:6].copy()

                mujoco.mj_forward(mj_model, ik)
                palm_pos = ik.xpos[palm_body_id].copy() + ik.xmat[palm_body_id].reshape(3, 3) @ GRIPPER_OFFSET

                offset = sim_state["target_pos"] - palm_pos
                dist   = np.linalg.norm(offset)
                if dist > 0.10:
                    sim_state["target_pos"]        = palm_pos + (offset / dist) * 0.10
                    sim_state["current_ik_target"] = sim_state["target_pos"].copy()

                target_frame_handle.position = np.array(sim_state["target_pos"])

                # ── DEBUG: verify the MuJoCo IK model itself is converging ──
                if DEBUG_RUNTIME and ik_frame_count % DEBUG_INTERVAL == 0:
                    print(f"[IK t={ik_frame_count / 60.0:6.1f}s] "
                          f"ik_target={np.round(sim_state['current_ik_target'], 4)}  "
                          f"mj_palm_estimate={np.round(palm_pos, 4)}  "
                          f"err={np.round(np.linalg.norm(sim_state['current_ik_target'] - palm_pos), 5)}  "
                          f"qpos[0:6]={np.round(ik.qpos[0:6], 4)}")

            time.sleep(rate)

    # ── Teleop updates ────────────────────────────────────────────────────────
    def update_teleop(delta=None, toggle_gripper=False, reset=False, wrist_delta=0.0):
        with lock:
            if reset:
                sim_state["target_pos"]        = initial_palm_pos.copy()
                sim_state["current_ik_target"] = initial_palm_pos.copy()
                sim_state["is_curled"]         = False
                sim_state["current_curl"]      = 0.0
                sim_state["wrist_angle"]       = right_defaults["joint6"]
                sim_state["arm_joint_targets"] = np.array([right_defaults[n] for n in RIGHT_ARM_JOINTS])
                apply_mj_defaults(sim_state["ik_mj_data"])
                target_frame_handle.position = np.array(sim_state["target_pos"])
                return
            if delta is not None:
                sim_state["target_pos"] += delta
            if toggle_gripper:
                sim_state["is_curled"] = not sim_state["is_curled"]
                status = "Closing" if sim_state["is_curled"] else "Opening"
                print(f"[GRASP] Gripper → {status}")
            if wrist_delta != 0.0:
                old_val = sim_state["wrist_angle"]
                new_val = np.clip(old_val + wrist_delta, -np.pi, np.pi)
                if new_val != old_val:
                    sim_state["wrist_angle"] = new_val
                    print(f"[WRIST] Angle → {np.rad2deg(new_val):.1f}°")
                else:
                    print("[WRIST] Limit reached!")

    # ── Viser GUI ─────────────────────────────────────────────────────────────
    with server.gui.add_folder("Keyboard Teleoperation"):
        server.gui.add_markdown(
            "**Physics:** NVIDIA Warp / Newton XPBD\n\n"
            "**Instructions:** Focus the browser window and use hotkeys, "
            "or click the buttons below."
        )
        btn_xp   = server.gui.add_button("Move X+ (I)")
        btn_xm   = server.gui.add_button("Move X- (K)")
        btn_yp   = server.gui.add_button("Move Y+ (J)")
        btn_ym   = server.gui.add_button("Move Y- (L)")
        btn_zp   = server.gui.add_button("Move Z+ (U)")
        btn_zm   = server.gui.add_button("Move Z- (O)")
        btn_grip = server.gui.add_button("Toggle Gripper (G)")
        btn_w_cw   = server.gui.add_button("Rotate Wrist CW (C)")
        btn_w_ccw  = server.gui.add_button("Rotate Wrist CCW (X)")
        btn_rst  = server.gui.add_button("Reset (R)")

    @btn_xp.on_click
    def _(_): update_teleop(delta=np.array([STEP_SIZE, 0.0, 0.0]))
    @btn_xm.on_click
    def _(_): update_teleop(delta=np.array([-STEP_SIZE, 0.0, 0.0]))
    @btn_yp.on_click
    def _(_): update_teleop(delta=np.array([0.0, STEP_SIZE, 0.0]))
    @btn_ym.on_click
    def _(_): update_teleop(delta=np.array([0.0, -STEP_SIZE, 0.0]))
    @btn_zp.on_click
    def _(_): update_teleop(delta=np.array([0.0, 0.0, STEP_SIZE]))
    @btn_zm.on_click
    def _(_): update_teleop(delta=np.array([0.0, 0.0, -STEP_SIZE]))
    @btn_grip.on_click
    def _(_): update_teleop(toggle_gripper=True)
    @btn_w_cw.on_click
    def _(_): update_teleop(wrist_delta=np.deg2rad(5.0))
    @btn_w_ccw.on_click
    def _(_): update_teleop(wrist_delta=np.deg2rad(-5.0))
    @btn_rst.on_click
    def _(_): update_teleop(reset=True)

    cmd_xp   = server.gui.add_command("Move X+",        hotkey="i")
    cmd_xm   = server.gui.add_command("Move X-",        hotkey="k")
    cmd_yp   = server.gui.add_command("Move Y+",        hotkey="j")
    cmd_ym   = server.gui.add_command("Move Y-",        hotkey="l")
    cmd_zp   = server.gui.add_command("Move Z+",        hotkey="u")
    cmd_zm   = server.gui.add_command("Move Z-",        hotkey="o")
    cmd_w_cw   = server.gui.add_command("Rotate Wrist CW",   hotkey="c")
    cmd_w_ccw  = server.gui.add_command("Rotate Wrist CCW",  hotkey="x")
    cmd_grip = server.gui.add_command("Toggle Gripper", hotkey="g")
    cmd_rst  = server.gui.add_command("Reset",          hotkey="r")

    @cmd_xp.on_trigger
    def _(_): update_teleop(delta=np.array([STEP_SIZE, 0.0, 0.0]))
    @cmd_xm.on_trigger
    def _(_): update_teleop(delta=np.array([-STEP_SIZE, 0.0, 0.0]))
    @cmd_yp.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, STEP_SIZE, 0.0]))
    @cmd_ym.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, -STEP_SIZE, 0.0]))
    @cmd_zp.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, 0.0, STEP_SIZE]))
    @cmd_zm.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, 0.0, -STEP_SIZE]))
    @cmd_grip.on_trigger
    def _(_): update_teleop(toggle_gripper=True)
    @cmd_w_cw.on_trigger
    def _(_): update_teleop(wrist_delta=np.deg2rad(5.0))
    @cmd_w_ccw.on_trigger
    def _(_): update_teleop(wrist_delta=np.deg2rad(-5.0))
    @cmd_rst.on_trigger
    def _(_): update_teleop(reset=True)

    # Start threads
    threading.Thread(target=ik_loop,                daemon=True).start()
    threading.Thread(target=physics_and_render_loop, daemon=True).start()

    print("\n" + "=" * 54)
    print(" Newton Bimanual Arm Teleop (Physics: SolverXPBD)")
    print(" Controlling: RIGHT arm")
    print("=" * 54)
    print(" Open:  http://localhost:8080")
    print(" Keys:  I/K → X    J/L → Y    U/O → Z")
    print("        C/X → Wrist Roll ([-180°, 180°])")
    print("        G → Gripper    R → Reset")
    print("=" * 54 + "\n")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("[INFO] Shutting down.")


if __name__ == "__main__":
    main()
