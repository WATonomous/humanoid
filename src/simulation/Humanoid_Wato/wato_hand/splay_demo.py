"""Sweep MCP_A abduction joints (4 fingers + thumb) so spread motion is easy to see.

Run from this directory (omit --headless to watch in the GUI):

    /home/hy/IsaacLab/isaaclab.sh -p splay_demo.py

Finger splay (MCP_A_1..4): ±0.15 rad (±8.594 deg).
Thumb abduction (MCP_A_thumb): 0 to 2.0 rad (~114.6 deg).
"""

from __future__ import annotations

import argparse
import math
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Wato hand MCP_A splay demo.")
parser.add_argument("--freq_hz", type=float, default=1.0, help="Splay sweep frequency in Hz.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass

from wato_hand_cfg import JOINT_POS_LIMITS, apply_joint_limits

# Same hand asset/physics as in-hand RL (self-collision off, palm-up, etc.).
_HUMANOID_RL_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "HumanoidRL"))
if _HUMANOID_RL_ROOT not in sys.path:
    sys.path.insert(0, _HUMANOID_RL_ROOT)
from HumanoidRLPackage.HumanoidRLSetup.modelCfg.wato_hand import INHAND_WATO_HAND_CFG  # noqa: E402

# Finger splay ±8.594 deg; thumb abduction 0..114.592 deg (each uses its own limits).
SPREAD_JOINTS = ["MCP_A_1", "MCP_A_2", "MCP_A_3", "MCP_A_4", "MCP_A_thumb"]

# Neutral demo pose: only MCP_A joints move; others pinned at 0.
_MCP_A_THUMB_MID = 0.5 * (JOINT_POS_LIMITS["MCP_A_thumb"][0] + JOINT_POS_LIMITS["MCP_A_thumb"][1])
_NEUTRAL_JOINT_POS = {name: 0.0 for name in [
    "circumduction", "MCP_A_thumb", "PIP_thumb", "DIP_thumb",
    "MCP_A_1", "MCP_A_2", "MCP_A_3", "MCP_A_4",
    "MCP_1", "PIP_1", "DIP_1", "MCP_2", "PIP_2", "DIP_2",
    "MCP_3", "PIP_3", "DIP_3", "MCP_4", "PIP_4", "DIP_4",
]}
_NEUTRAL_JOINT_POS["MCP_A_thumb"] = _MCP_A_THUMB_MID


@configclass
class SplayDemoSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=2500.0, color=(0.85, 0.85, 0.85)),
    )
    robot = INHAND_WATO_HAND_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=INHAND_WATO_HAND_CFG.init_state.pos,
            rot=INHAND_WATO_HAND_CFG.init_state.rot,
            joint_pos=_NEUTRAL_JOINT_POS,
        ),
    )


def _sweep_angle(sim_time_s: float, freq_hz: float, lo: float, hi: float) -> float:
    """Sine sweep between joint-specific limits."""
    mid = 0.5 * (lo + hi)
    amp = 0.5 * (hi - lo)
    return mid + amp * math.sin(2.0 * math.pi * freq_hz * sim_time_s)


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene, freq_hz: float) -> None:
    robot = scene["robot"]
    sim_dt = sim.get_physics_dt()
    sim_time_s = 0.0
    print_every = int(1.0 / sim_dt)

    spread_ids = [robot.find_joints(name)[0][0] for name in SPREAD_JOINTS]
    spread_limits = [JOINT_POS_LIMITS[name] for name in SPREAD_JOINTS]
    fixed_ids = [i for i in range(robot.num_joints) if i not in spread_ids]

    # Pin every non-spread joint at 0 rad.
    fixed_pos = torch.zeros((robot.num_instances, len(fixed_ids)), device=robot.device)
    spread_target = torch.zeros((robot.num_instances, len(spread_ids)), device=robot.device)

    print(f"[INFO] Moving only: {SPREAD_JOINTS}")
    print("[INFO] All other joints pinned at 0 rad")
    for name, (lo, hi) in zip(SPREAD_JOINTS, spread_limits):
        print(f"[INFO]   {name}: {math.degrees(lo):.1f} deg to {math.degrees(hi):.1f} deg")
    print(f"[INFO] Sweep frequency: {freq_hz:.2f} Hz")

    step = 0
    while simulation_app.is_running():
        for i, (lo, hi) in enumerate(spread_limits):
            spread_target[:, i] = _sweep_angle(sim_time_s, freq_hz, lo, hi)

        robot.set_joint_position_target(fixed_pos, joint_ids=fixed_ids)
        robot.set_joint_position_target(spread_target, joint_ids=spread_ids)
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim_dt)

        if step % print_every == 0:
            finger = spread_target[0, 0].item()
            thumb = spread_target[0, -1].item()
            print(
                f"[splay] t={sim_time_s:5.1f}s  MCP_A_1={math.degrees(finger):+.1f} deg"
                f"  MCP_A_thumb={math.degrees(thumb):+.1f} deg"
            )

        sim_time_s += sim_dt
        step += 1


def main() -> None:
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 120.0, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=(0.35, 0.35, 0.75), target=(0.0, 0.08, 0.54))

    scene = InteractiveScene(SplayDemoSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    apply_joint_limits(scene["robot"])
    print("[INFO] Splay demo running. MCP_A_1..4 + MCP_A_thumb move; all other joints stay at 0.")
    run_simulator(sim, scene, args_cli.freq_hz)


if __name__ == "__main__":
    main()
    simulation_app.close()
