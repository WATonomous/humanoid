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


def clip_qpos_to_limits(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Clip data.qpos to joint limits (only for limited joints)."""
    for i in range(model.njnt):
        jnt_type = model.jnt_type[i]
        if jnt_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        qposadr = model.jnt_qposadr[i]
        if qposadr < 0:
            continue
        lo = model.jnt_range[i, 0]
        hi = model.jnt_range[i, 1]
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
) -> tuple[np.ndarray, bool, float]:
    """
    Run damped least-squares IK so that each fingertip reaches its target.

    targets: dict mapping body name -> (3,) world position.
    Returns: (qpos, converged, final_max_error).
    """
    # Build list of (body_name, target_pos) in consistent order
    tip_names = [b for b in FINGERTIP_BODIES if b in targets]
    if not tip_names:
        raise ValueError("targets must contain at least one of %s" % FINGERTIP_BODIES)
    target_list = [targets[b] for b in tip_names]

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

        # Damped least squares: dq = (J'J + d*I)^{-1} J' err
        A = J.T @ J + damping * np.eye(model.nv)
        dq = np.linalg.solve(A, J.T @ err)
        # All joints in this model are 1-DOF; qpos layout matches dofs.
        data.qpos[: model.nv] += step * dq

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


def main():
    model = load_model()
    data = mujoco.MjData(model)

    # Start from default qpos (zeros); get reachable fingertip positions via FK.
    data.qpos[:] = 0.0
    default_positions = get_fingertip_positions(model, data)
    if len(default_positions) != 5:
        raise RuntimeError("Could not get all 5 fingertip positions from model.")

    # Use reachable targets: small offset from default pose so IK has a solution.
    # User can replace this dict with any desired world-frame targets (meters).
    targets = {
        name: pos + np.array([0.02, 0.0, 0.02])  # small reach test
        for name, pos in default_positions.items()
    }

    qpos, converged, max_err = solve_fingertip_ik(
        model, data, targets,
        damping=1e-3,
        step=0.35,
        max_iter=1000,
        tol=1.2e-2,  # ~12 mm; residual often limited by joint limits
    )

    print("Converged:", converged)
    print("Final max position error (m):", max_err)
    print("Joint configuration (qpos):", qpos)

    mujoco.mj_forward(model, data)
    print("\nAchieved fingertip positions (world):")
    for name in FINGERTIP_BODIES:
        try:
            bid = body_name_to_id(model, name)
            achieved = data.xpos[bid]
            desired = targets.get(name, np.full(3, np.nan))
            print(f"  {name}: achieved {achieved}  target {desired}")
        except KeyError:
            pass


if __name__ == "__main__":
    main()
