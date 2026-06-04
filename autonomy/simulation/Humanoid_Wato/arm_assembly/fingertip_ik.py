"""
fingertip_ik.py
===============
Provides a MuJoCo-based damped least-squares Inverse Kinematics solver for the Wato robot's fingers.
It calculates the exact joint configuration (21-DOF qpos) required to bring the 5 fingertips
to desired 3D coordinates.

Process:
  1. Load the arm_assembly.urdf file and mesh geometry into a MuJoCo physics model
  2. Clamp joint positions within their limits, skipping continuous joints that have range [0, 0]
  3. Run an iterative solver loop:
     a. Compute current fingertip positions and 3D positioning errors
     b. Compute the body Jacobian matrix for each fingertip
     c. Resolve the joint velocity deltas using damped least-squares (Levenberg-Marquardt style)
     d. Update joint positions and clamp them
     e. Stop and return if error is below the tolerance threshold
"""
import os
import mujoco
import numpy as np
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH = _SCRIPT_DIR / "right_arm_assembly.urdf"

FINGERTIP_BODIES = [
    "IP_THUMB_v1_1",   # thumb
    "DIP_INDEX_v1_1",  # index
    "DIP_MIDDLE_v1_1",  # middle
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

    xml = path.read_text()
    # MuJoCo's URDF importer keeps only mesh basenames; chdir to the side subfolder.
    if 'filename="meshes/left_arm/' in xml:
        mesh_subdir = mesh_dir / "left_arm"
        xml = xml.replace('filename="meshes/left_arm/', 'filename="')
    elif 'filename="meshes/right_arm/' in xml:
        mesh_subdir = mesh_dir / "right_arm"
        xml = xml.replace('filename="meshes/right_arm/', 'filename="')
    else:
        mesh_subdir = mesh_dir
        xml = xml.replace('filename="meshes/', 'filename="')
    if not mesh_subdir.is_dir():
        raise FileNotFoundError(f"Mesh directory not found: {mesh_subdir}")
    old_cwd = os.getcwd()
    try:
        os.chdir(mesh_subdir)
        return mujoco.MjModel.from_xml_string(xml)
    finally:
        os.chdir(old_cwd)


def clip_to_joint_limits(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Clamp joint values to URDF limits."""
    for i in range(model.njnt):
        jnt_type = model.jnt_type[i]
        if jnt_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        pos_idx = model.jnt_qposadr[i]
        lo = model.jnt_range[i, 0]
        hi = model.jnt_range[i, 1]
        if lo >= hi:
            continue
        if np.isfinite(lo) or np.isfinite(hi):
            data.qpos[pos_idx] = np.clip(data.qpos[pos_idx], lo, hi)


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
    tip_names = [b for b in FINGERTIP_BODIES if b in targets]
    if not tip_names:
        raise ValueError("targets must contain at least one of %s" % FINGERTIP_BODIES)
    target_list = [targets[b] for b in tip_names]

    for _ in range(max_iter):
        mujoco.mj_forward(model, data)

        J_list = []
        err_list = []
        nv = model.nv

        for body_name, target in zip(tip_names, target_list):
            body_id = model.body(body_name).id
            pos = data.xpos[body_id].copy()
            err = target - pos
            jacp = np.zeros((3, nv))
            jacr = np.zeros((3, nv))
            mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
            J_list.append(jacp)
            err_list.append(err)

        J = np.vstack(J_list)
        err = np.concatenate(err_list)
        max_err = np.max(np.abs(err))

        if max_err < tol:
            return data.qpos.copy(), True, float(max_err)

        A = J.T @ J + damping * np.eye(nv)
        rhs = J.T @ err
        dq = np.linalg.solve(A, rhs)

        data.qpos[:nv] += step * dq

        if clip_limits:
            clip_to_joint_limits(model, data)

    return data.qpos.copy(), False, float(max_err)


def get_fingertip_positions(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, np.ndarray]:
    mujoco.mj_forward(model, data)
    out = {}
    for name in FINGERTIP_BODIES:
        id = model.body(name).id
        out[name] = data.xpos[id].copy()
    return out


def main():
    model = load_model()
    data = mujoco.MjData(model)

    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    default_positions = get_fingertip_positions(model, data)
    print(f"[sanity] found {len(default_positions)}/5 fingertip bodies in MuJoCo model")


if __name__ == "__main__":
    main()
