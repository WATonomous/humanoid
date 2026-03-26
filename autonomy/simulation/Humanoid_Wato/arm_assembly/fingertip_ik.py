"""
IK for the 21 DOF humanoid arm-hand, with iterative jacobian least damped square method

Details in IK.md
"""
import time
import os
import mujoco
import mujoco.viewer
import numpy as np
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
URDF_PATH = _SCRIPT_DIR / "arm_assembly.urdf"

FINGERTIP_BODIES = [
    "IP_THUMB_v1_1",   
    "DIP_INDEX_v1_1",
    "DIP_MIDDLE_v1_1",
    "DIP_RING_v1_1",   
    "DIP_PINKY_v1_1",  
]

FINGERTIP_OFFSETS = {
    "IP_THUMB_v1_1":   np.array([-0.008, 0.005, 0.018]),
    "DIP_INDEX_v1_1":  np.array([0.006, 0.002, 0.02]),
    "DIP_MIDDLE_v1_1": np.array([0.005, -0.018, 0.0]),
    "DIP_RING_v1_1":   np.array([-0.005, 0.0, 0.02]),
    "DIP_PINKY_v1_1":  np.array([0.004, -0.019, 0.004]),
}


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
    """Clamp joint value to be within joint limit

    MuJoCo's URDF importer can set continuous joints to range [0,0] which would
    lock joint 0 at zero, here we bypass that by making if lower == upper,
    it doesn't enforce clipping
    """
    for i in range(model.njnt):
        jnt_type = model.jnt_type[i]
        if jnt_type not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            continue
        pos_idx = model.jnt_qposadr[i]
        lo = model.jnt_range[i, 0]
        hi = model.jnt_range[i, 1]
        if lo == hi:
            continue
        if np.isfinite(lo) or np.isfinite(hi):
            data.qpos[pos_idx] = np.clip(data.qpos[pos_idx], lo, hi)

def fingertip_world_pos(data, body_id, local_offset):
    R = data.xmat[body_id].reshape(3, 3)
    return data.xpos[body_id] + R @ local_offset


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
            local_offset = FINGERTIP_OFFSETS[body_name]
            pos = fingertip_world_pos(data, body_id, local_offset)
            err = target - pos
            jacp = np.zeros((3, nv))
            jacr = np.zeros((3, nv))
            mujoco.mj_jac(model, data, jacp, jacr, pos, body_id)
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
        out[name] = fingertip_world_pos(data, id, FINGERTIP_OFFSETS[name]).copy()
    return out


def main():
    model = load_model()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)

    def add_sphere(scene, pos, rgba, radius=0.008):
        g = scene.geoms[scene.ngeom]
        scene.ngeom += 1
        mujoco.mjv_initGeom(
            g,
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([radius, radius, radius]),
            pos=np.array(pos),
            mat=np.eye(3).reshape(-1),
            rgba=np.array(rgba),
        )
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.user_scn.ngeom = 0

            for name in FINGERTIP_BODIES:
                body_id = model.body(name).id
                body_pos = data.xpos[body_id].copy()
                pos = data.xpos[body_id].copy()
                tip_pos = fingertip_world_pos(data, body_id, FINGERTIP_OFFSETS[name])
                add_sphere(viewer.user_scn, tip_pos, [1, 0, 0, 1])

            viewer.sync()
            time.sleep(0.01) 

    default_positions = get_fingertip_positions(model, data)
    print(f"default is {default_positions}")
    print(f"[sanity] found {len(default_positions)}/5 fingertip bodies in MuJoCo model")



if __name__ == "__main__":
    main()
