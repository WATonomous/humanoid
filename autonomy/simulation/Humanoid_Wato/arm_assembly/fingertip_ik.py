"""
Standalone gradient-based IK for arm_assembly (6-DOF arm + 15-DOF hand).
Solves for joint angles so that 5 fingertips reach desired world positions
using damped least-squares (no dependencies on other scripts in this repo).
"""
import os
import mujoco
import numpy as np
from pathlib import Path

# Path to URDF (script next to arm_assembly folder or repo root)
_SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH = _SCRIPT_DIR / "arm_assembly.urdf"

# Fingertip body names (distal link of each finger in arm_assembly.urdf)
FINGERTIP_BODIES = [
    "IP_THUMB_v1_1",   # thumb
    "DIP_INDEX_v1_1",  # index
    "DIP_MIDDLE_v1_1", # middle
    "DIP_RING_v1_1",   # ring
    "DIP_PINKY_v1_1",  # pinky
]

# First N DOFs are arm (shoulder, elbow, wrist); rest are hand. Used for weighted IK.
NUM_ARM_DOFS = 6


def load_model(urdf_path=None):
    path = Path(urdf_path or URDF_PATH).resolve()
    if not path.exists():
        raise FileNotFoundError(f"URDF not found: {path}")
    mesh_dir = path.parent / "meshes"
    if not mesh_dir.is_dir():
        raise FileNotFoundError(f"Mesh directory not found: {mesh_dir}")
    # MuJoCo's URDF importer keeps only mesh basename and opens from cwd.
    # Use mesh filenames without "meshes/" and chdir to meshes so they are found.
    xml = path.read_text()
    xml = xml.replace('filename="meshes/', 'filename="')
    old_cwd = os.getcwd()
    try:
        os.chdir(mesh_dir)
        return mujoco.MjModel.from_xml_string(xml)
    finally:
        os.chdir(old_cwd)


def body_name_to_id(model: mujoco.MjModel, name: str) -> int:
    try:
        return model.body(name).id
    except KeyError:
        # List bodies for debugging
        names = [model.body(i).name for i in range(model.nbody)]
        raise KeyError(f"Body '{name}' not in model. Available: {names}")


def print_joint_limits_urdf(model: mujoco.MjModel) -> None:
    """Print joint position limits from the MuJoCo model (from URDF).
    Use this to compare with USD joint limits (e.g. in Isaac Sim).
    """
    print("[joint limits] From URDF (MuJoCo model.jnt_range):")
    for i in range(model.njnt):
        jnt_type = model.jnt_type[i]
        if jnt_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or f"joint_{i}"
        qposadr = model.jnt_qposadr[i]
        if qposadr < 0:
            continue
        lo, hi = model.jnt_range[i, 0], model.jnt_range[i, 1]
        finite = np.isfinite(lo) or np.isfinite(hi)
        marker = "  <-- joint 0 (shoulder_flexion)" if qposadr == 0 else ""
        if finite:
            print(f"  [qpos {qposadr}] {name}: lower={lo:.4f}, upper={hi:.4f}{marker}")
            if qposadr == 0 and lo == hi:
                print("       ^ LOCKED at one value (lower=upper). MuJoCo URDF import may have set continuous joints to [0,0]. Fix URDF or USD so this joint has a real range.")
        else:
            print(f"  [qpos {qposadr}] {name}: (no limits / continuous){marker}")


def clip_qpos_to_limits(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Clip data.qpos to joint limits (only for limited joints).
    Bypass: if lower == upper (e.g. MuJoCo imported continuous as [0,0]), skip clipping
    so the IK can move that joint instead of locking it.
    """
    for i in range(model.njnt):
        jnt_type = model.jnt_type[i]
        if jnt_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        qposadr = model.jnt_qposadr[i]
        if qposadr < 0:
            continue
        lo = model.jnt_range[i, 0]
        hi = model.jnt_range[i, 1]
        if lo == hi:
            continue  # bypass: treat as unlimited (e.g. continuous imported as [0,0])
        if np.isfinite(lo) or np.isfinite(hi):
            data.qpos[qposadr] = np.clip(data.qpos[qposadr], lo, hi)


def solve_fingertip_ik(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    targets: dict[str, np.ndarray],
    *,
    damping: float = 1e-4,
    step: float = 0.5,
    max_iter: int = 200,
    tol: float = 1e-4,
    clip_limits: bool = True,
    arm_dofs: int = NUM_ARM_DOFS,
    finger_weight: float = 4.0,
    joint0_weight: float = 0.4,
) -> tuple[np.ndarray, bool, float]:
    """
    Run damped least-squares IK so that each fingertip reaches its target.

    Weighted regularization: arm DOFs (first arm_dofs) are penalized less than
    finger DOFs (finger_weight > 1), so the solver prefers moving the arm to reach.
    joint0_weight < 1 (default 0.4) further prefers joint 0 (shoulder_flexion) when
    error has a Y/Z component, so the arm uses it to reduce large Y (or Y+Z) error.

    Joint 0 rotates about X so it cannot help with pure X error; for Y/Z error it can.
    targets: dict mapping body name -> (3,) world position.
    Returns: (qpos, converged, final_max_error).
    """
    # Build list of (body_name, target_pos) in consistent order
    tip_names = [b for b in FINGERTIP_BODIES if b in targets]
    if not tip_names:
        raise ValueError("targets must contain at least one of %s" % FINGERTIP_BODIES)
    target_list = [targets[b] for b in tip_names]

    # Diagonal regularization: joint 0 preferred when < 1, other arm 1, fingers finger_weight.
    nv = model.nv
    arm_dofs = min(arm_dofs, nv)
    W = np.ones(nv, dtype=np.float64)
    W[0] = max(1e-3, float(joint0_weight))
    W[arm_dofs:] = finger_weight

    for _ in range(max_iter):
        mujoco.mj_forward(model, data)

        J_list = []
        err_list = []

        for body_name, target in zip(tip_names, target_list):
            body_id = body_name_to_id(model, body_name)
            pos = data.xpos[body_id].copy()
            err = target - pos
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
            J_list.append(jacp)
            err_list.append(err)

        J = np.vstack(J_list)
        err = np.concatenate(err_list)
        max_err = np.max(np.abs(err))

        if max_err < tol:
            return data.qpos.copy(), True, float(max_err)

        # Weighted damped least squares
        A = J.T @ J + damping * np.diag(W)
        rhs = J.T @ err
        dq = np.linalg.solve(A, rhs)

        # All joints in this model are 1-DOF; qpos layout matches dofs.
        data.qpos[:nv] += step * dq

        if clip_limits:
            clip_qpos_to_limits(model, data)

    return data.qpos.copy(), False, float(max_err)


def get_fingertip_positions(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, np.ndarray]:
    """Return current world positions of all 5 fingertips."""
    mujoco.mj_forward(model, data)
    out = {}
    for name in FINGERTIP_BODIES:
        try:
            bid = body_name_to_id(model, name)
            out[name] = data.xpos[bid].copy()
        except KeyError:
            pass
    return out


def diagnose_jacobian_joint0(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    err: np.ndarray | None = None,
) -> None:
    """
    At current qpos, build the 15x21 task Jacobian J and explain why joint 0
    (shoulder_flexion_extension) often gets dq[0] ≈ 0. Joint 0 rotates about X,
    so J's column 0 has zero X components; pure-X position error contributes
    nothing to (J.T @ err)[0].

    If err is provided (15-D task error: [ex,ey,ez] x 5 fingertips), also prints
    error by axis and (J.T @ err) for first 6 joints so you can see if joint 0
    gets a non-zero rhs when error is in Y.
    """
    mujoco.mj_forward(model, data)
    J_list = []
    for name in FINGERTIP_BODIES:
        bid = body_name_to_id(model, name)
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, bid)
        J_list.append(jacp)
    J = np.vstack(J_list)
    j0 = J[:, 0]
    x_rows = [0, 3, 6, 9, 12]
    j0_x = j0[x_rows]
    j0_yz = np.delete(j0, x_rows)
    print("Joint 0 (shoulder_flexion_extension) Jacobian column (15-D task):")
    print("  ||J[:,0]|| =", np.linalg.norm(j0))
    print("  X components (rows 0,3,6,9,12):", j0_x, "  -> should be ~0 (joint rotates about X)")
    print("  Y,Z components norm:", np.linalg.norm(j0_yz))
    err_x = np.zeros(15)
    err_x[0::3] = 1.0
    rhs0_x = (J.T @ err_x)[0]
    print("  (J.T @ err_pure_X)[0] =", rhs0_x, "  -> 0 means joint 0 gets no update for forward (X) reach")
    err_y = np.zeros(15)
    err_y[1::3] = 1.0
    rhs0_y = (J.T @ err_y)[0]
    rhs1_y = (J.T @ err_y)[1]
    print("  (J.T @ err_pure_Y)[0] =", rhs0_y, "  [1] =", rhs1_y, "  -> joint 0 can help for Y error")
    print("  So dq[0] stays 0 when error is mostly in world X.")

    if err is not None and err.size == 15:
        rhs = J.T @ err
        err_x_sum = np.sum(np.abs(err[0::3]))
        err_y_sum = np.sum(np.abs(err[1::3]))
        err_z_sum = np.sum(np.abs(err[2::3]))
        print("  --- With current task error ---")
        print("  |error| by axis (sum over 5 tips):  X =", err_x_sum, "  Y =", err_y_sum, "  Z =", err_z_sum)
        print("  (J.T @ err)[0:6] (arm joints):", rhs[:6])
        print("  -> If Y error is large but rhs[0] ~ 0, other joints are covering Y; if rhs[0] != 0, joint 0 could step.")


def main():
    model = load_model()
    data = mujoco.MjData(model)

    # --- Joint limits from URDF (MuJoCo) ---
    print("========== Joint limits (URDF / MuJoCo) ==========")
    print_joint_limits_urdf(model)
    print()

    # --- Diagnose at qpos = 0 ---
    data.qpos[:] = 0.0
    print("========== DIAGNOSE at qpos=0 ==========")
    diagnose_jacobian_joint0(model, data)
    print()

    default_positions = get_fingertip_positions(model, data)
    if len(default_positions) != 5:
        raise RuntimeError("Could not get all 5 fingertip positions from model.")

    # Test 1: small reach (X+Z offset)
    targets_small = {
        name: pos + np.array([0.02, 0.0, 0.02])
        for name, pos in default_positions.items()
    }
    # Test 2: large Y offset to reproduce "huge Y error" case (joint 0 can help in Y)
    targets_large_y = {
        name: pos + np.array([0.0, 0.1, 0.0])
        for name, pos in default_positions.items()
    }

    for label, targets in [("small reach (0.02,0,0.02)", targets_small), ("large Y offset +0.1", targets_large_y)]:
        data.qpos[:] = 0.0
        print("========== IK test:", label, "==========")
        qpos, converged, max_err = solve_fingertip_ik(
            model, data, targets,
            damping=5e-4,
            step=0.25,
            max_iter=3000,
            tol=5e-3,
        )
        print("Converged:", converged, "  max_err (m):", max_err, "  qpos[0] (joint 0):", data.qpos[0])

        # Build current task error and diagnose at this pose
        tip_names = [b for b in FINGERTIP_BODIES if b in targets]
        err_list = []
        for name in tip_names:
            bid = body_name_to_id(model, name)
            err_list.append(targets[name] - data.xpos[bid])
        err = np.concatenate(err_list)
        print("--- DIAGNOSE at final pose (with current error) ---")
        diagnose_jacobian_joint0(model, data, err=err)
        print()

    # Final summary: achieved vs target for last test
    mujoco.mj_forward(model, data)
    print("Achieved vs target (last test):")
    for name in FINGERTIP_BODIES:
        try:
            bid = body_name_to_id(model, name)
            achieved = data.xpos[bid]
            desired = targets_large_y.get(name, np.full(3, np.nan))
            e = desired - achieved
            print(f"  {name}: err = ({e[0]:.4f}, {e[1]:.4f}, {e[2]:.4f}) m")
        except KeyError:
            pass


if __name__ == "__main__":
    main()
