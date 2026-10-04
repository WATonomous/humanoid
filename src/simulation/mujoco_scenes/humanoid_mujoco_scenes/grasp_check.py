"""Headless grasp check: the left gripper picks up a box object, lifts and carries it, and reports how
well the contacts hold it. No viewer, no leader -- a scripted IK operator stands in for the leader.

    python -m humanoid_mujoco_scenes.grasp_check --scene peg_insert --object peg

Drives the arm like the leader teleop does (position targets every CONTROL_DT, the teleop's lowered
wrist damping) and fails (exit 1) if the object slips or rotates in the jaws, the fingers chatter,
or contacts sink in deeper than the limits below.
"""
from __future__ import annotations

import argparse
import sys

import mujoco
import numpy as np

from humanoid_mujoco_scenes import list_scenes, make_model

CONTROL_DT = 0.01        # leader teleop control period (leader_mapping.CONTROL_DT)
WRIST_DAMPING = 2.5      # leader teleop wrist kv (leader_mapping.WRIST_DAMPING)
MAX_SLIP_MM = 5.0        # object travel in the gripper frame while carried
MAX_ROT_DEG = 2.0        # object rotation in the gripper frame while carried
MAX_PENETRATION_MM = 1.5
MAX_FINGER_SPEED = 0.02  # m/s, finger joint speed while holding (chatter)


class _Arm:
    """Left arm IK on the point midway between the open jaws, gripper orientation held at home."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        from pioneer_humanoid.arm_params import LEFT_ARM_JOINTS, LEFT_GRIPPER_JOINTS

        self.m, self.d, self.kin = model, data, mujoco.MjData(model)
        self.hand = model.body("link6l").id
        self.qadr = [model.joint(j).qposadr[0] for j in LEFT_ARM_JOINTS]
        self.vadr = [model.joint(j).dofadr[0] for j in LEFT_ARM_JOINTS]
        self.acts = [model.actuator(j).id for j in LEFT_ARM_JOINTS]
        self.grip_acts = [model.actuator(j).id for j in LEFT_GRIPPER_JOINTS]
        self.grip_vadr = [model.joint(j).dofadr[0] for j in LEFT_GRIPPER_JOINTS]
        self.lim = np.array([model.jnt_range[model.joint(j).id] for j in LEFT_ARM_JOINTS])
        jaws = [
            next(g for g in range(model.ngeom) if model.geom_bodyid[g] == model.body(b).id and model.geom_contype[g])
            for b in ("link7l", "link8l")
        ]
        rot = data.xmat[self.hand].reshape(3, 3)
        self.offset = rot.T @ (data.geom_xpos[jaws].mean(axis=0) - data.xpos[self.hand])
        self.rot0 = rot.copy()
        self.q = data.qpos[self.qadr].copy()

    def tcp(self, d: mujoco.MjData | None = None) -> np.ndarray:
        d = d or self.d
        return d.xpos[self.hand] + d.xmat[self.hand].reshape(3, 3) @ self.offset

    def ik(self, target: np.ndarray) -> np.ndarray:
        m, kin, q = self.m, self.kin, self.q.copy()
        kin.qpos[:] = self.d.qpos
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for _ in range(100):
            kin.qpos[self.qadr] = q
            mujoco.mj_kinematics(m, kin)
            mujoco.mj_comPos(m, kin)
            rot = kin.xmat[self.hand].reshape(3, 3)
            err = np.concatenate([target - self.tcp(kin), 0.5 * sum(np.cross(rot[:, i], self.rot0[:, i]) for i in range(3))])
            if np.linalg.norm(err) < 1e-6:
                break
            mujoco.mj_jac(m, kin, jp, jr, self.tcp(kin), self.hand)
            jac = np.vstack([jp[:, self.vadr], jr[:, self.vadr]])
            q = np.clip(q + jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), err), *self.lim.T)
        self.q = q
        return q


def run(scene: str, obj: str, verbose: bool = False) -> dict:
    from pioneer_humanoid.arm_params import LEFT_GRIPPER_CLOSED, LEFT_GRIPPER_JOINTS, LEFT_GRIPPER_OPEN
    from pioneer_humanoid.mujoco_bimanual_arm import set_home

    model = make_model(scene)
    data = mujoco.MjData(model)
    model.actuator_biasprm[model.actuator("joint6l").id, 2] = -WRIST_DAMPING
    set_home(model, data)
    mujoco.mj_forward(model, data)
    arm = _Arm(model, data)
    body = model.body(obj).id
    geom = next(g for g in range(model.ngeom) if model.geom_bodyid[g] == body)
    if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_BOX:
        raise SystemExit(f"--object {obj!r}: first geom must be a box")
    substeps = max(1, round(CONTROL_DT / model.opt.timestep))
    grip_open = np.array([LEFT_GRIPPER_OPEN[j] for j in LEFT_GRIPPER_JOINTS])
    grip_closed = np.array([LEFT_GRIPPER_CLOSED[j] for j in LEFT_GRIPPER_JOINTS])

    # Side grasp just below the object's top face, gripper as at home (jaws closing along world Y).
    xy = data.xpos[body][:2].copy()
    z_grasp = data.xpos[body][2] + model.geom_size[geom][2] - 0.005
    waypoints = [  # (tcp xyz, gripper closure, seconds, phase)
        ((*xy, z_grasp + 0.10), 0.0, 1.0, "approach"),
        ((*xy, z_grasp), 0.0, 1.0, "descend"),
        ((*xy, z_grasp), 1.0, 0.6, "close"),
        ((*xy, z_grasp), 1.0, 0.5, "settle"),
        ((*xy, z_grasp + 0.12), 1.0, 1.0, "lift"),
        ((*(xy + [0.11, 0.06]), z_grasp + 0.12), 1.0, 1.5, "carry"),
        ((*(xy + [0.11, 0.06]), z_grasp + 0.12), 1.0, 1.0, "hold"),
    ]
    tcp, closure = arm.tcp().copy(), 0.0
    slip = rot = finger_speed = penetration = 0.0
    ref = None
    for goal, goal_closure, seconds, phase in waypoints:
        start, closure0, n = tcp.copy(), closure, int(seconds / CONTROL_DT)
        for i in range(n):
            s = (i + 1) / n
            s = s * s * (3 - 2 * s)
            tcp, closure = start + (np.array(goal) - start) * s, closure0 + (goal_closure - closure0) * s
            data.ctrl[arm.acts] = arm.ik(tcp)
            data.ctrl[arm.grip_acts] = grip_open + closure * (grip_closed - grip_open)
            mujoco.mj_step(model, data, nstep=substeps)
            con = data.contact
            touching = (model.geom_bodyid[con.geom1] == body) | (model.geom_bodyid[con.geom2] == body)
            if touching.any():
                penetration = max(penetration, -con.dist[touching].min())
            hand_rot = data.xmat[arm.hand].reshape(3, 3)
            rel_pos = hand_rot.T @ (data.xpos[body] - arm.tcp())
            rel_rot = hand_rot.T @ data.xmat[body].reshape(3, 3)
            if phase == "settle":
                ref = (rel_pos, rel_rot)
            elif ref is not None:
                slip = max(slip, np.linalg.norm(rel_pos - ref[0]))
                cos = (np.trace(ref[1].T @ rel_rot) - 1) / 2
                rot = max(rot, np.degrees(np.arccos(np.clip(cos, -1, 1))))
                finger_speed = max(finger_speed, np.abs(data.qvel[arm.grip_vadr]).max())
        if verbose:
            print(f"  {phase:8s} object at {np.round(data.xpos[body], 3)}")
    lifted = data.xpos[body][2] - (z_grasp - model.geom_size[geom][2] + 0.005)
    return dict(slip_mm=slip * 1e3, rot_deg=rot, finger_speed=finger_speed,
                penetration_mm=penetration * 1e3, lifted_mm=lifted * 1e3)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scene", default="peg_insert", help="scene name (an unknown name lists them)")
    parser.add_argument("--object", default="peg", help="free body to grasp (its first geom must be a box)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()
    if args.scene not in list_scenes():
        raise SystemExit(f"unknown --scene {args.scene!r}; available: {list_scenes()}")

    r = run(args.scene, args.object, args.verbose)
    checks = [
        ("lifted", r["lifted_mm"], ">", 100.0, "mm"),
        ("slip in grip", r["slip_mm"], "<", MAX_SLIP_MM, "mm"),
        ("rotation in grip", r["rot_deg"], "<", MAX_ROT_DEG, "deg"),
        ("finger speed", r["finger_speed"], "<", MAX_FINGER_SPEED, "m/s"),
        ("max penetration", r["penetration_mm"], "<", MAX_PENETRATION_MM, "mm"),
    ]
    failed = False
    for name, value, op, limit, unit in checks:
        ok = value > limit if op == ">" else value < limit
        failed |= not ok
        print(f"{'ok  ' if ok else 'FAIL'} {name:17s} {value:8.3f} {unit:3s} (want {op} {limit})")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
