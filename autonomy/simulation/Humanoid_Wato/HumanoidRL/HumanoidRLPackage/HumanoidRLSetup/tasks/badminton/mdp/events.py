from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.envs.mdp.events import reset_joints_by_scale
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

ARM_JOINT_NAMES = [
    "shoulder_flexion_extension",
    "shoulder_abduction_adduction",
    "shoulder_rotation",
    "elbow_flexion_extension",
    "forearm_rotation",
    "wrist_extension",
]

RACKET_GRIP_JOINT_POS = {
    "mcp_index": 0.8,
    "pip_index": 0.9,
    "dip_index": 0.7,
    "mcp_middle": 0.8,
    "pip_middle": 0.9,
    "dip_middle": 0.7,
    "mcp_ring": 0.75,
    "pip_ring": 0.85,
    "dip_ring": 0.65,
    "mcp_pinky": 0.7,
    "pip_pinky": 0.8,
    "dip_pinky": 0.6,
    "cmc_thumb": 0.5,
    "mcp_thumb": 1.2,
    "ip_thumb": 0.6,
}


def reset_arm_and_racket_grip(
    env: ManagerBasedEnv,
    env_ids: torch.Tensor,
    position_range: tuple[float, float],
    velocity_range: tuple[float, float],
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
):
    """Randomize arm joints and restore a fixed racket-grip pose on the fingers."""
    asset: Articulation = env.scene[asset_cfg.name]

    reset_joints_by_scale(
        env,
        env_ids,
        position_range=position_range,
        velocity_range=velocity_range,
        asset_cfg=asset_cfg,
    )

    joint_pos = asset.data.joint_pos[env_ids].clone()
    joint_vel = asset.data.joint_vel[env_ids].clone()
    for joint_name, grip_pos in RACKET_GRIP_JOINT_POS.items():
        joint_ids, _ = asset.find_joints(joint_name)
        for joint_idx in joint_ids:
            joint_pos[:, joint_idx] = grip_pos
            joint_vel[:, joint_idx] = 0.0

    asset.write_joint_state_to_sim(joint_pos, joint_vel, env_ids=env_ids)
