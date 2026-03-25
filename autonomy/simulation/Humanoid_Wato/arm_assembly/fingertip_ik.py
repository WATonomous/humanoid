"""
Standalone gradient-based IK for arm_assembly (6-DOF arm + 15-DOF hand).
Solves for joint angles so that 5 fingertips reach desired world positions
using damped least-squares

Joint 0 (shoulder_flexion_extension) issue:
  Here we found that MuJoCo's URDF importer can set continuous joints to range [0,0], which would
  lock joint 0 at zero. We bypass that in clip_to_joint_limits(): when
  lower == upper we skip clipping so the IK can move that joint.
"""
import os
import mujoco
import numpy as np
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH = _SCRIPT_DIR / "arm_assembly.urdf"

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

    xml = path.read_text()
    xml = xml.replace('filename="meshes/', 'filename="')
    old_cwd = os.getcwd()
    try:
        os.chdir(mesh_dir)
        return mujoco.MjModel.from_xml_string(xml)
    finally:
        os.chdir(old_cwd)


def clip_to_joint_limits(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Clamp each hinge/slide joint's scalar position to ``model.jnt_range``.

    Bypass: if lower == upper (e.g. MuJoCo imported continuous as [0,0]), skip clipping
    so the IK can move that joint instead of locking it.
    """
    for i in range(model.njnt):
        jnt_type = model.jnt_type[i]
        if jnt_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        pos_idx = model.jnt_qposadr[i]
        if pos_idx < 0:
            continue
        lo = model.jnt_range[i, 0]
        hi = model.jnt_range[i, 1]
        if lo == hi:
            continue  # bypass: treat as unlimited revolute joint
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
    """
    Run damped least-squares IK so that each fingertip reaches its target.

    targets: dict mapping body name -> (3,) world position.
    Returns: (generalized_positions, converged, final_max_error) — a copy of ``data.qpos``.
    """
    # Build list of (body_name, target_pos) in consistent order
    tip_names = [b for b in FINGERTIP_BODIES if b in targets]
    if not tip_names:
        raise ValueError("targets must contain at least one of %s" % FINGERTIP_BODIES)
    target_list = [targets[b] for b in tip_names]

    nv = model.nv

    for _ in range(max_iter):
        mujoco.mj_forward(model, data)

        J_list = []
        err_list = []

        for body_name, target in zip(tip_names, target_list):
            body_id = model.body(body_name).id
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

        A = J.T @ J + damping * np.eye(nv)
        rhs = J.T @ err
        dq = np.linalg.solve(A, rhs)

        # All joints in this model are 1-DOF; qpos layout matches dofs.
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
