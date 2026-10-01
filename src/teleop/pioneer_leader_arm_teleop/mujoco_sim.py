"""MuJoCo backend of pioneer_leader_arm_teleop.py: the leader arm drives the Pioneer left arm in plain MuJoCo (CPU).

Same leader mapping as the Isaac backend (leader_mapping.py). Scenes: humanoid_mujoco_scenes.

  R   re-zero the leader, reset the arm and every object in the scene

Needs a display for the MuJoCo viewer (on macOS run with ``mjpython``). Recording is not supported yet.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
# pioneer_humanoid and humanoid_mujoco_scenes; this fallback keeps an uninstalled checkout working.
sys.path.insert(0, str(_SRC / "pioneer_humanoid"))
sys.path.insert(0, str(_SRC / "simulation" / "mujoco_scenes"))

from leader_mapping import (  # noqa: E402
    CONTROL_DT,
    WRIST_DAMPING,
    LeaderInput,
    LeaderMapping,
    WallClock,
    add_leader_args,
    check_leader_args,
)

_KEY_R = 82  # GLFW key code


def run() -> None:
    parser = argparse.ArgumentParser(description="7-servo leader teleoperation of the Pioneer left arm in MuJoCo.")
    add_leader_args(parser, scene_help="scene registered in humanoid_mujoco_scenes (an unknown name lists them)")
    parser.add_argument("--record", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    check_leader_args(parser, args)
    if args.record:
        parser.error("--record is not supported with --sim mujoco yet")

    import mujoco
    import mujoco.viewer
    from humanoid_mujoco_scenes import list_scenes, make_model, scene_camera
    from pioneer_humanoid.arm_params import (
        LEFT_ARM_JOINTS,
        LEFT_GRIPPER_CLOSED,
        LEFT_GRIPPER_JOINTS,
        LEFT_GRIPPER_OPEN,
    )
    from pioneer_humanoid.mujoco_arm import set_home

    if args.scene not in list_scenes():
        raise SystemExit(f"unknown --scene {args.scene!r}; available: {list_scenes()}")
    model = make_model(args.scene)
    data = mujoco.MjData(model)
    # Position actuator bias is [0, -kp, -kv]: lower the wrist's kv (see WRIST_DAMPING).
    model.actuator_biasprm[model.actuator("joint6l").id, 2] = -WRIST_DAMPING

    arm_acts = [model.actuator(j).id for j in LEFT_ARM_JOINTS]
    grip_acts = [model.actuator(j).id for j in LEFT_GRIPPER_JOINTS]
    grip_open = [LEFT_GRIPPER_OPEN[j] for j in LEFT_GRIPPER_JOINTS]
    grip_closed = [LEFT_GRIPPER_CLOSED[j] for j in LEFT_GRIPPER_JOINTS]
    set_home(model, data)
    mapping = LeaderMapping(
        args,
        home_rad=[data.ctrl[a] for a in arm_acts],
        limits_rad=[tuple(model.jnt_range[model.joint(j).id]) for j in LEFT_ARM_JOINTS],
    )
    print(mapping.describe(LEFT_ARM_JOINTS), flush=True)

    reset_requested = [False]

    def on_key(key: int) -> None:
        if key == _KEY_R:
            reset_requested[0] = True

    leader = LeaderInput(args)
    substeps = max(1, round(CONTROL_DT / model.opt.timestep))
    clock = WallClock(substeps * model.opt.timestep)
    print("[INFO] Hold the leader in the home pose (elbow bent, gripper open). R = re-zero + reset.", flush=True)
    try:
        with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
            for key, value in (scene_camera(args.scene) or {}).items():
                setattr(viewer.cam, key, value)
            while viewer.is_running():
                if reset_requested[0]:
                    reset_requested[0] = False
                    mujoco.mj_resetData(model, data)
                    set_home(model, data)
                    # Re-zero with the arm, so the next read does not command a jump.
                    leader.rezero()
                    mapping.reset()
                    print("\n[LEADER] Re-zeroed; arm and scene reset.", flush=True)

                target, grip = mapping.update(leader.read())
                data.ctrl[arm_acts] = target
                data.ctrl[grip_acts] = [o + grip * (c - o) for o, c in zip(grip_open, grip_closed)]
                mujoco.mj_step(model, data, nstep=substeps)
                viewer.sync()

                leader.report(grip)
                clock.wait()
    finally:
        leader.close()
        print("\n[INFO] Stopped. Leader torque is OFF.", flush=True)
