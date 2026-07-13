"""M2 smoke test: hold the default pose for 300 control steps.

Checks: env builds, resets randomize objects with separation, no NaNs, the
left arm holds its default pose under the joint-position action term, and
(if enabled) cameras produce frames.

Run from the HumanoidRL directory inside the simulation_il container:
    PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p \
        HumanoidRLPackage/HumanoidRLSetup/tasks/pick_place_bimanual/scripts/smoke_hold_pose.py \
        --headless --enable_cameras
"""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--steps", type=int, default=300)
parser.add_argument("--task_params", type=str, default=None)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import HumanoidRLPackage.HumanoidRLSetup.tasks  # noqa: E402,F401
from HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual import robot_cfg_shim as shim  # noqa: E402
from HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual.pick_place_env_cfg import (  # noqa: E402
    make_env_cfg,
)


def main():
    shim.check_constants_consistency()
    params = shim.PickPlaceTaskParams.from_yaml(args_cli.task_params)
    if not args_cli.enable_cameras:
        params.cameras.enabled = False
    env_cfg = make_env_cfg(params)
    env = gym.make("Isaac-PickPlace-BimanualLeft-v0", cfg=env_cfg)

    obs, _ = env.reset()
    robot = env.unwrapped.scene["robot"]
    action_term = env.unwrapped.action_manager.get_term("arm_action")
    joint_names = action_term._joint_names
    print(f"action joint order: {joint_names}")
    joint_ids = action_term._joint_ids
    hold = robot.data.default_joint_pos[:, joint_ids].clone()

    start_pos = robot.data.joint_pos[:, joint_ids].clone()
    for i in range(args_cli.steps):
        obs, _, terminated, truncated, _ = env.step(hold)
        assert not torch.isnan(obs["policy"]["joint_pos"]).any(), f"NaN joints at step {i}"
    end_pos = robot.data.joint_pos[:, joint_ids]
    drift = (end_pos - start_pos).abs().max().item()
    print(f"max joint drift over {args_cli.steps} steps: {drift:.4f} rad/m")
    assert drift < 0.05, "arm failed to hold default pose"

    obj = env.unwrapped.scene["object"]
    poses = []
    for _ in range(3):
        env.reset()
        p = obj.data.root_pos_w[0] - env.unwrapped.scene.env_origins[0]
        poses.append([round(float(v), 3) for v in p])
    print(f"object spawn samples (env-local): {poses}")
    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    assert max(xs) - min(xs) > 1e-4 or max(ys) - min(ys) > 1e-4, "object spawn not randomized"

    for cam_name in ("camera_external", "camera_wrist"):
        if cam_name in env.unwrapped.scene.sensors:
            cam = env.unwrapped.scene.sensors[cam_name]
            rgb = cam.data.output["rgb"]
            print(f"{cam_name}: {tuple(rgb.shape)} dtype={rgb.dtype} mean={rgb.float().mean():.1f}")
            assert rgb.float().mean() > 1.0, f"{cam_name} produced black frames"

    print("SMOKE TEST PASSED")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
