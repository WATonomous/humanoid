# Place for location to be decided
"""
Keyboard teleoperation script for the bimanual WATO humanoid arm using MuJoCo and mjviser.
Provides Inverse Kinematics (IK) positioning of the left arm's end effector and left gripper open/close.
Loads the bimanual arm with a tennis ball on the ground for pick-and-place teleop demonstration.

Controls:
  I/K : Move left arm along X-axis (+/-)
  J/L : Move left arm along Y-axis (+/-)
  U/O : Move left arm along Z-axis (+/-)
  G   : Toggle left finger gripper (Open/Closed)
  R   : Reset left arm to default pose
"""

import os
import sys
import time
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco
from mjviser import Viewer

# Define paths
_SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH = _SCRIPT_DIR.parent.parent / "Humanoid_Wato" / "wato_bimanual_arm" / "urdf" / "bimanual_arm.urdf"

# Fallback to container absolute path if local workspace path does not resolve
if not URDF_PATH.exists():
    URDF_PATH = Path("/root/ament_ws/src/simulation/Humanoid_Wato/wato_bimanual_arm/urdf/bimanual_arm.urdf")

# Tennis ball physical properties
# Grab block dimensions and physics (small cube, stable on flat surface)
BLOCK_W     = 0.04        # 4 cm wide
BLOCK_D     = 0.04        # 4 cm deep
BLOCK_H     = 0.04        # 4 cm tall
BLOCK_MASS  = 0.15        # 150 g — light enough to lift, heavy enough to stay put
# Box inertia: I = m*(a^2+b^2)/12 for each axis
BLOCK_Ixx   = BLOCK_MASS * (BLOCK_D**2 + BLOCK_H**2) / 12
BLOCK_Iyy   = BLOCK_MASS * (BLOCK_W**2 + BLOCK_H**2) / 12
BLOCK_Izz   = BLOCK_MASS * (BLOCK_W**2 + BLOCK_D**2) / 12

# Platform dimensions
PLATFORM_W  = 0.20        # 20 cm wide
PLATFORM_D  = 0.20        # 20 cm deep
PLATFORM_H  = 0.05        # 5 cm tall

# Block XY position (centred on the platform)
BALL_XY     = (0.44, -0.20)

# Initial block Z: base of block sits on top of platform
BALL_INIT_POS = (BALL_XY[0], BALL_XY[1], PLATFORM_H + BLOCK_H / 2)  # = 0.08 m

# Temporary flag to enable/disable the blue block
ADD_BLOCK = True


def load_model(urdf_path=URDF_PATH):
    path = Path(urdf_path).resolve()
    if not path.exists():
        raise FileNotFoundError(f"URDF file not found at {path}")

    mesh_dir = path.parent.parent / "meshes"
    if not mesh_dir.is_dir():
        raise FileNotFoundError(f"Mesh directory not found at {mesh_dir}")

    robot_xml = path.read_text().replace("../meshes", str(mesh_dir))
    robot_root = ET.fromstring(robot_xml)

    # ---------------------------------------------------------------
    # Add a low platform (5 cm tall box, fixed) for the ball to rest on.
    # Using a box geom guarantees solid, reliable collision detection.
    # ---------------------------------------------------------------
    plat_link = ET.SubElement(robot_root, "link", {"name": "ball_platform"})

    p_visual = ET.SubElement(plat_link, "visual")
    ET.SubElement(
        ET.SubElement(p_visual, "geometry"),
        "box", {"size": f"{PLATFORM_W} {PLATFORM_D} {PLATFORM_H}"}
    )
    ET.SubElement(p_visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    p_mat = ET.SubElement(p_visual, "material", {"name": "platform_orange"})
    ET.SubElement(p_mat, "color", {"rgba": "0.85 0.42 0.10 1.0"})

    p_coll = ET.SubElement(plat_link, "collision", {"friction": "2.5 0.01 0.0002"})
    ET.SubElement(
        ET.SubElement(p_coll, "geometry"),
        "box", {"size": f"{PLATFORM_W} {PLATFORM_D} {PLATFORM_H}"}
    )
    ET.SubElement(p_coll, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

    p_iner = ET.SubElement(plat_link, "inertial")
    ET.SubElement(p_iner, "mass", {"value": "10000.0"})   # very heavy = effectively fixed
    ET.SubElement(p_iner, "origin", {"xyz": "0 0 0"})
    ET.SubElement(p_iner, "inertia", {
        "ixx": "1000", "ixy": "0", "ixz": "0",
        "iyy": "1000", "iyz": "0", "izz": "1000"
    })

    plat_joint = ET.SubElement(robot_root, "joint", {
        "name": "ball_platform_joint", "type": "fixed"
    })
    ET.SubElement(plat_joint, "parent", {"link": "base_link"})
    ET.SubElement(plat_joint, "child", {"link": "ball_platform"})
    # Centre of the platform: XY = ball position, Z = half-height above floor
    ET.SubElement(plat_joint, "origin", {
        "xyz": f"{BALL_XY[0]} {BALL_XY[1]} {PLATFORM_H / 2}",
        "rpy": "0 0 0"
    })

    if ADD_BLOCK:
        # ---------------------------------------------------------------
        # Add a small grab block (cube box) as a free-floating rigid body.
        # A box geom has exact flat-face collision detection — won't roll away.
        # ---------------------------------------------------------------
        block_link = ET.SubElement(robot_root, "link", {"name": "tennis_ball"})

        visual = ET.SubElement(block_link, "visual")
        ET.SubElement(
            ET.SubElement(visual, "geometry"),
            "box", {"size": f"{BLOCK_W} {BLOCK_D} {BLOCK_H}"}
        )
        ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        mat = ET.SubElement(visual, "material", {"name": "block_blue"})
        ET.SubElement(mat, "color", {"rgba": "0.15 0.45 0.90 1.0"})

        collision = ET.SubElement(block_link, "collision", {"friction": "1.5 0.005 0.0001"})
        ET.SubElement(
            ET.SubElement(collision, "geometry"),
            "box", {"size": f"{BLOCK_W} {BLOCK_D} {BLOCK_H}"}
        )
        ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

        inertial = ET.SubElement(block_link, "inertial")
        ET.SubElement(inertial, "mass", {"value": str(BLOCK_MASS)})
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0"})
        ET.SubElement(inertial, "inertia", {
            "ixx": str(BLOCK_Ixx), "ixy": "0", "ixz": "0",
            "iyy": str(BLOCK_Iyy), "iyz": "0", "izz": str(BLOCK_Izz)
        })

        # Floating joint so the block is a free rigid body in the world
        ball_joint = ET.SubElement(robot_root, "joint", {
            "name": "tennis_ball_joint", "type": "floating"
        })
        ET.SubElement(ball_joint, "parent", {"link": "base_link"})
        ET.SubElement(ball_joint, "child", {"link": "tennis_ball"})
        ET.SubElement(ball_joint, "origin", {
            "xyz": f"{BALL_INIT_POS[0]} {BALL_INIT_POS[1]} {BALL_INIT_POS[2]}",
            "rpy": "0 0 0"
        })

    # ---------------------------------------------------------------
    # Add a ground plane so the ball rests on something solid.
    # A large, heavy, fixed box acts as the floor.
    # ---------------------------------------------------------------
    floor_link = ET.SubElement(robot_root, "link", {"name": "ground_plane"})

    f_visual = ET.SubElement(floor_link, "visual")
    ET.SubElement(ET.SubElement(f_visual, "geometry"), "box", {"size": "3.0 3.0 0.02"})
    ET.SubElement(f_visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    f_mat = ET.SubElement(f_visual, "material", {"name": "floor_grey"})
    ET.SubElement(f_mat, "color", {"rgba": "0.55 0.55 0.55 1.0"})

    f_coll = ET.SubElement(floor_link, "collision")
    ET.SubElement(ET.SubElement(f_coll, "geometry"), "box", {"size": "3.0 3.0 0.02"})
    ET.SubElement(f_coll, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

    f_inertial = ET.SubElement(floor_link, "inertial")
    ET.SubElement(f_inertial, "mass", {"value": "10000.0"})
    ET.SubElement(f_inertial, "origin", {"xyz": "0 0 0"})
    ET.SubElement(f_inertial, "inertia", {
        "ixx": "1000", "ixy": "0", "ixz": "0",
        "iyy": "1000", "iyz": "0", "izz": "1000"
    })

    floor_joint = ET.SubElement(robot_root, "joint", {
        "name": "floor_joint", "type": "fixed"
    })
    ET.SubElement(floor_joint, "parent", {"link": "base_link"})
    ET.SubElement(floor_joint, "child", {"link": "ground_plane"})
    ET.SubElement(floor_joint, "origin", {"xyz": "0 0 -0.01", "rpy": "0 0 0"})

    # Update finger joint limits to unlock motion range for physics/actuators
    for j_name, (lo, hi) in {
        "joint7l": ("-0.05", "0.0"),
        "joint8l": ("0.0", "0.05"),
        "joint7":  ("-0.05", "0.0"),
        "joint8":  ("0.0", "0.05"),
    }.items():
        joint_elem = robot_root.find(f".//joint[@name='{j_name}']")
        if joint_elem is not None:
            limit = joint_elem.find("limit")
            if limit is not None:
                limit.set("lower", lo)
                limit.set("upper", hi)
                limit.set("effort", "100.0")
                limit.set("velocity", "2.0")

    # Add compiler/option XML configurations
    mujoco_elem = ET.SubElement(robot_root, "mujoco")
    ET.SubElement(mujoco_elem, "compiler", {"balanceInertia": "true", "discardvisual": "false"})
    ET.SubElement(mujoco_elem, "option", {
        "cone": "elliptic",
        "jacobian": "dense"
    })

    # 1. Compile the URDF structure to an intermediate MjModel
    urdf_xml = ET.tostring(robot_root, encoding="utf-8").decode("utf-8")
    temp_model = mujoco.MjModel.from_xml_string(urdf_xml)

    # 2. Save compiled model to standard MJCF XML format (temporary file)
    temp_path = "/tmp/temp_bimanual_mjcf.xml"
    mujoco.mj_saveLastXML(temp_path, temp_model)

    # 3. Parse the MJCF XML structure using ElementTree
    tree = ET.parse(temp_path)
    root = tree.getroot()

    # 4. Inject position actuators directly into the MJCF root element
    actuator_root = ET.SubElement(root, "actuator")
    ET.SubElement(actuator_root, "position", {
        "name": "left_finger_1_actuator",
        "joint": "joint7l",
        "kp": "200.0",
        "ctrlrange": "-0.05 0.0"
    })
    ET.SubElement(actuator_root, "position", {
        "name": "left_finger_2_actuator",
        "joint": "joint8l",
        "kp": "200.0",
        "ctrlrange": "0.0 0.05"
    })

    # 5. Compile the final MJCF XML to the returned MjModel
    final_xml = ET.tostring(root, encoding="utf-8").decode("utf-8")
    return mujoco.MjModel.from_xml_string(final_xml)


def clip_to_joint_limits(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Clamp joint values to limits specified in the URDF."""
    for i in range(model.njnt):
        jtype = model.jnt_type[i]
        if jtype not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        pos_idx = model.jnt_qposadr[i]
        lo = model.jnt_range[i, 0]
        hi = model.jnt_range[i, 1]
        if lo >= hi:
            continue
        if np.isfinite(lo) or np.isfinite(hi):
            data.qpos[pos_idx] = np.clip(data.qpos[pos_idx], lo, hi)


def solve_palm_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    target_pos: np.ndarray,
    *,
    damping: float = 1e-3,
    step: float = 0.5,
    max_iter: int = 150,
    tol: float = 1e-4,
) -> None:
    """Solve translation IK for the left arm end effector link6l."""
    body_name = "link6l"
    body_id = model.body(body_name).id
    nv = model.nv

    for _ in range(max_iter):
        mujoco.mj_forward(model, data)
        pos = data.xpos[body_id].copy()
        err = target_pos - pos
        if np.max(np.abs(err)) < tol:
            break

        jacp = np.zeros((3, nv))
        jacr = np.zeros((3, nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)

        J_arm = jacp[:, 8:14]
        A = J_arm.T @ J_arm + damping * np.eye(6)
        dq_arm = np.linalg.solve(A, J_arm.T @ err)

        data.qpos[8:14] += step * dq_arm
        clip_to_joint_limits(model, data)


def set_finger_curl(model: mujoco.MjModel, data: mujoco.MjData, curl_factor: float):
    """Set finger joints position based on curl factor (0.0=open, 1.0=closed)."""
    data.qpos[14] = -0.05 * (1.0 - curl_factor)
    data.qpos[15] = 0.05 * (1.0 - curl_factor)


def get_bimanual_defaults():
    """Default poses (in radians) from bimanual configuration files."""
    right_defaults = {
        "joint1": np.deg2rad(-140.8),
        "joint2": np.deg2rad(55.7),
        "joint3": np.deg2rad(-66.0),
        "joint4": np.deg2rad(111.4),
        "joint5": np.deg2rad(34.8),
        "joint6": np.deg2rad(3.5),
        "joint7": -0.05,
        "joint8": 0.05
    }
    left_defaults = {
        "joint1L": np.deg2rad(139.2),
        "joint2l": np.deg2rad(66.1),
        "joint3l": np.deg2rad(147.9),
        "joint4l": np.deg2rad(-76.5),
        "joint5l": np.deg2rad(-76.5),
        "joint6l": np.deg2rad(-22.6),
        "joint7l": -0.05,
        "joint8l": 0.05
    }
    return right_defaults, left_defaults


def main():
    print("[INFO] Loading robot URDF model with grab block...")
    model = load_model()
    data = mujoco.MjData(model)

    # Enable gravity so the block rests on the platform
    model.opt.gravity[:] = [0.0, 0.0, -9.81]

    # ---------------------------------------------------------------
    # Soft contact model & high friction tuning.
    # We assign elliptic cone physics and high torsional/rolling friction
    # parameters to the fingertips and block to enable solid grabs.
    # ---------------------------------------------------------------
    SOFT_SOLREF = np.array([0.10, 1.0])
    SOFT_SOLIMP = np.array([0.5, 0.9, 0.01, 0.5, 2.0])
    HIGH_FRICTION = np.array([10.0, 0.5, 0.05])
    ARM_BODIES  = {"link1","link2","link3","link4","link5","link6",
                   "link7","link8","link1L","link2l","link3l",
                   "link4l","link5l","link6l","link7l","link8l"}
    for i in range(model.ngeom):
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[i]) or ""
        # Apply soft contacts only to main arm links (1-6) to handle kinematic snaps,
        # but keep fingers and block under default rigid dynamics so they stop on impact.
        if bname in ARM_BODIES and bname not in {"link7", "link8", "link7l", "link8l"}:
            model.geom_solref[i] = SOFT_SOLREF
            model.geom_solimp[i] = SOFT_SOLIMP
        # Apply high friction to fingertips, block, platform, and floor geoms
        if bname in {"link7l", "link8l", "link7", "link8", "tennis_ball", "world", "ground_plane"}:
            model.geom_friction[i] = HIGH_FRICTION

    right_defaults, left_defaults = get_bimanual_defaults()

    def _apply_arm_defaults(d):
        for name, val in right_defaults.items():
            d.qpos[model.joint(name).qposadr[0]] = val
        for name, val in left_defaults.items():
            d.qpos[model.joint(name).qposadr[0]] = val

    _apply_arm_defaults(data)

    # Identify the tennis ball free joint if it exists
    has_block = False
    try:
        ball_joint_id = model.joint("tennis_ball_joint").id
        ball_qpos_adr = model.joint("tennis_ball_joint").qposadr[0]
        ball_dof_adr  = model.joint("tennis_ball_joint").dofadr[0]
        block_id      = model.body("tennis_ball").id
        has_block = True
    except KeyError:
        ball_joint_id = None
        ball_qpos_adr = None
        ball_dof_adr  = None
        block_id      = None

    # Set initial ball pose if block exists
    if has_block:
        data.qpos[ball_qpos_adr:ball_qpos_adr+3] = list(BALL_INIT_POS)
        data.qpos[ball_qpos_adr+3] = 1.0   # quaternion w=1 (no rotation)
        data.qpos[ball_qpos_adr+4:ball_qpos_adr+7] = 0.0

    mujoco.mj_forward(model, data)

    # Configure joint damping to stabilize physics step and prevent contact bouncing
    # Revolute arm joints get high damping, prismatic finger joints get moderate damping
    for i in range(model.njnt):
        jname = model.joint(i).name
        if jname != "tennis_ball_joint":
            dof_idx = model.joint(jname).dofadr[0]
            model.dof_damping[dof_idx] = 5.0 if ("joint7" in jname or "joint8" in jname) else 15.0

    # Add virtual drag damping to the free floating block to absorb impact energy
    if has_block:
        model.dof_damping[ball_dof_adr:ball_dof_adr+6] = 2.5

    palm_body_name = "link6l"
    palm_id = model.body(palm_body_name).id
    initial_palm_pos = data.xpos[palm_id].copy()

    state = {
        "target_pos": initial_palm_pos.copy(),
        "current_ik_target": initial_palm_pos.copy(),
        "is_curled": False,
        "current_curl": 0.0,
        "left_arm_qpos": np.array([left_defaults[n] for n in [
            "joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"
        ]]),
        "ik_data": mujoco.MjData(model),
    }
    _apply_arm_defaults(state["ik_data"])
    if has_block:
        state["ik_data"].qpos[ball_qpos_adr:ball_qpos_adr+3] = list(BALL_INIT_POS)
        state["ik_data"].qpos[ball_qpos_adr+3] = 1.0
    mujoco.mj_forward(model, state["ik_data"])

    step_size = 0.015  # 1.5 cm per key press
    FINGER_FRAMES = 1200  # steps for finger curl animation (~20 s)

    # -----------------------------------------------------------------------
    # Physics step with position actuators and kinematic locking.
    # We assign targets to position actuators before stepping to let
    # MuJoCo's solver compute real friction and normal forces on the block.
    # -----------------------------------------------------------------------
    def step_fn(model_obj, data_obj):
        # ---- 1. Apply slow finger target curl to actuators ----
        target_curl = 1.0 if state["is_curled"] else 0.0
        if state["current_curl"] < target_curl:
            state["current_curl"] = min(target_curl, state["current_curl"] + 1.0 / FINGER_FRAMES)
        elif state["current_curl"] > target_curl:
            state["current_curl"] = max(target_curl, state["current_curl"] - 1.0 / FINGER_FRAMES)

        # Map current_curl to actuator ranges:
        # joint7l: -0.05 (open) to 0.0 (closed)
        # joint8l: 0.05 (open) to 0.0 (closed)
        ctrl_f1 = -0.05 * (1.0 - state["current_curl"])
        ctrl_f2 =  0.05 * (1.0 - state["current_curl"])
        data_obj.ctrl[0] = ctrl_f1
        data_obj.ctrl[1] = ctrl_f2

        # ---- 2. Step physics engine ----
        mujoco.mj_step(model_obj, data_obj)

        # ---- 3. Kinematic Override for Arm Joints (keeps arm solid at target) ----
        left_arm_targets = state["left_arm_qpos"]
        for i in range(6):
            data_obj.qpos[8 + i] = left_arm_targets[i]
            data_obj.qvel[8 + i] = 0.0

        for name, val in right_defaults.items():
            adr = model_obj.joint(name).qposadr[0]
            dof = model_obj.joint(name).dofadr[0]
            data_obj.qpos[adr] = val
            data_obj.qvel[dof] = 0.0

        # Update collision broadphase positions
        mujoco.mj_fwdPosition(model_obj, data_obj)

    print("[INFO] Starting visualizer on port 8080...")
    viewer = Viewer(model, data, step_fn=step_fn)
    viewer._paused = False
    server = viewer._server

    viewer._setup_joint_sliders = lambda: None
    viewer._setup_actuator_sliders = lambda: None

    target_frame = server.scene.add_frame(
        "/target_palm",
        position=state["target_pos"],
        axes_length=0.15,
        axes_radius=0.008
    )
    def update_teleop(delta=None, toggle_gripper=False, reset=False):
        with viewer._lock:
            if reset:
                state["target_pos"] = initial_palm_pos.copy()
                state["current_ik_target"] = initial_palm_pos.copy()
                _apply_arm_defaults(data)
                _apply_arm_defaults(state["ik_data"])
                data.qvel[:] = 0.0
                state["is_curled"] = False
                state["current_curl"] = 0.0
                # Reset ball to initial position
                if has_block:
                    data.qpos[ball_qpos_adr:ball_qpos_adr+3] = list(BALL_INIT_POS)
                    data.qpos[ball_qpos_adr+3] = 1.0
                    data.qpos[ball_qpos_adr+4:ball_qpos_adr+7] = 0.0
                    data.qvel[model.joint("tennis_ball_joint").dofadr[0]:
                              model.joint("tennis_ball_joint").dofadr[0]+6] = 0.0
                state["left_arm_qpos"] = np.array([left_defaults[n] for n in [
                    "joint1L", "joint2l", "joint3l", "joint4l", "joint5l", "joint6l"
                ]])
                target_frame.position = state["target_pos"]
                mujoco.mj_forward(model, data)
                viewer._render()
                return

            if delta is not None:
                state["target_pos"] += delta

            if toggle_gripper:
                state["is_curled"] = not state["is_curled"]
                status = "Closing" if state["is_curled"] else "Opening"
                print(f"[GRASP] Gripper physical target set to: {status}")

    # 60 Hz IK solver thread
    def control_loop():
        rate = 1.0 / 60.0
        ik = state["ik_data"]
        while True:
            with viewer._lock:
                ik.qpos[8:16] = data.qpos[8:16]

                # Smoothly interpolate current_ik_target towards target_pos (discrete input).
                # This translates discrete keypress jumps into a continuous Cartesian trajectory,
                # preventing the arm from teleporting into the block and avoiding huge contact impulses.
                diff = state["target_pos"] - state["current_ik_target"]
                if np.linalg.norm(diff) > 1e-5:
                    state["current_ik_target"] += diff * 0.12  # smooth exponential convergence

                solve_palm_ik(model, ik, state["current_ik_target"])
                state["left_arm_qpos"] = ik.qpos[8:14].copy()

                mujoco.mj_forward(model, ik)
                palm_pos = ik.xpos[palm_id].copy()
                
                # Only clamp if the target wanders too far from the physical palm (e.g., >10cm).
                # This prevents the target frame from running away while ensuring absolutely
                # zero drift when the user is idle.
                offset = state["target_pos"] - palm_pos
                dist = np.linalg.norm(offset)
                if dist > 0.10:
                    state["target_pos"] = palm_pos + (offset / dist) * 0.10
                    state["current_ik_target"] = state["target_pos"].copy()

                target_frame.position = state["target_pos"]
                viewer._dirty = True
            time.sleep(rate)

    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    # GUI
    with server.gui.add_folder("Keyboard Teleoperation"):
        server.gui.add_markdown("**Instructions:** Click buttons below or focus the browser page and use hotkeys.")
        btn_xp = server.gui.add_button("Move X+ (I)")
        btn_xm = server.gui.add_button("Move X- (K)")
        btn_yp = server.gui.add_button("Move Y+ (J)")
        btn_ym = server.gui.add_button("Move Y- (L)")
        btn_zp = server.gui.add_button("Move Z+ (U)")
        btn_zm = server.gui.add_button("Move Z- (O)")
        btn_grip = server.gui.add_button("Toggle Gripper (G)")
        btn_reset = server.gui.add_button("Reset (R)")

    @btn_xp.on_click
    def _(_): update_teleop(delta=np.array([step_size, 0.0, 0.0]))
    @btn_xm.on_click
    def _(_): update_teleop(delta=np.array([-step_size, 0.0, 0.0]))
    @btn_yp.on_click
    def _(_): update_teleop(delta=np.array([0.0, step_size, 0.0]))
    @btn_ym.on_click
    def _(_): update_teleop(delta=np.array([0.0, -step_size, 0.0]))
    @btn_zp.on_click
    def _(_): update_teleop(delta=np.array([0.0, 0.0, step_size]))
    @btn_zm.on_click
    def _(_): update_teleop(delta=np.array([0.0, 0.0, -step_size]))
    @btn_grip.on_click
    def _(_): update_teleop(toggle_gripper=True)
    @btn_reset.on_click
    def _(_): update_teleop(reset=True)

    cmd_xp = server.gui.add_command("Move X+", hotkey="i")
    cmd_xm = server.gui.add_command("Move X-", hotkey="k")
    cmd_yp = server.gui.add_command("Move Y+", hotkey="j")
    cmd_ym = server.gui.add_command("Move Y-", hotkey="l")
    cmd_zp = server.gui.add_command("Move Z+", hotkey="u")
    cmd_zm = server.gui.add_command("Move Z-", hotkey="o")
    cmd_grip = server.gui.add_command("Toggle Gripper", hotkey="g")
    cmd_reset = server.gui.add_command("Reset", hotkey="r")

    @cmd_xp.on_trigger
    def _(_): update_teleop(delta=np.array([step_size, 0.0, 0.0]))
    @cmd_xm.on_trigger
    def _(_): update_teleop(delta=np.array([-step_size, 0.0, 0.0]))
    @cmd_yp.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, step_size, 0.0]))
    @cmd_ym.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, -step_size, 0.0]))
    @cmd_zp.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, 0.0, step_size]))
    @cmd_zm.on_trigger
    def _(_): update_teleop(delta=np.array([0.0, 0.0, -step_size]))
    @cmd_grip.on_trigger
    def _(_): update_teleop(toggle_gripper=True)
    @cmd_reset.on_trigger
    def _(_): update_teleop(reset=True)

    print("\n" + "="*50)
    print("Bimanual Arm + Tennis Ball Pick-and-Place Teleop")
    print("Usage: Open http://localhost:8080 or http://127.0.0.1:8080")
    print("Focus the web page and use keyboard controls (Left Arm):")
    print("  I/K : Move Left Arm X-axis (Forward/Backward)")
    print("  J/L : Move Left Arm Y-axis (Left/Right)")
    print("  U/O : Move Left Arm Z-axis (Up/Down)")
    print("  G   : Open/Close Gripper (Toggle)")
    print("  R   : Reset everything")
    print(f"Tennis ball at: X={BALL_INIT_POS[0]:.2f}, Y={BALL_INIT_POS[1]:.2f}")
    print(f"  → Press I about 7 times to reach the ball, then G to grip")
    print("="*50 + "\n")

    viewer.run()


if __name__ == "__main__":
    main()
