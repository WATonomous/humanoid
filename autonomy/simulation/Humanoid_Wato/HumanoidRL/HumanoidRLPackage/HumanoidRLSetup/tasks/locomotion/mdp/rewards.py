from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
from isaaclab.utils.math import quat_rotate_inverse, yaw_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


<<<<<<< HEAD
def _phase(env: ManagerBasedRLEnv, period: float) -> torch.Tensor:
    return torch.remainder(env.episode_length_buf.float() * env.step_dt, period) / period


def gait_phase(env: ManagerBasedRLEnv, period: float) -> torch.Tensor:
    """Return a two-channel gait clock observation: sin/cos phase."""
    phase = _phase(env, period)
    return torch.stack((torch.sin(2.0 * torch.pi * phase), torch.cos(2.0 * torch.pi * phase)), dim=-1)


def _body_pos_yaw_frame(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    body_pos_w = asset.data.body_pos_w[:, asset_cfg.body_ids, :]
    rel_pos_w = body_pos_w - asset.data.root_pos_w[:, None, :]
    num_bodies = len(asset_cfg.body_ids)
    yaw_quat_w = yaw_quat(asset.data.root_quat_w).repeat_interleave(num_bodies, dim=0)
    return quat_rotate_inverse(yaw_quat_w, rel_pos_w.reshape(-1, 3)).reshape(rel_pos_w.shape)


def _body_rel_lin_vel_yaw_frame(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset = env.scene[asset_cfg.name]
    body_vel_w = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :]
    rel_vel_w = body_vel_w - asset.data.root_lin_vel_w[:, None, :]
    num_bodies = len(asset_cfg.body_ids)
    yaw_quat_w = yaw_quat(asset.data.root_quat_w).repeat_interleave(num_bodies, dim=0)
    return quat_rotate_inverse(yaw_quat_w, rel_vel_w.reshape(-1, 3)).reshape(rel_vel_w.shape)


=======
>>>>>>> f3902ba0 (modify-locomotion-rl)
def feet_air_time(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg, threshold: float
) -> torch.Tensor:
    """Reward long steps taken by the feet using L2-kernel.

    This function rewards the agent for taking steps that are longer than a threshold. This helps ensure
    that the robot lifts its feet off the ground and takes steps. The reward is computed as the sum of
    the time for which the feet are in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    first_contact = contact_sensor.compute_first_contact(env.step_dt)[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def feet_air_time_positive_biped(env, command_name: str, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Reward long steps taken by the feet for bipeds.

    This function rewards the agent for taking steps up to a specified threshold and also keep one foot at
    a time in the air.

    If the commands are small (i.e. the agent is not supposed to take a step), then the reward is zero.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    # compute the reward
    air_time = contact_sensor.data.current_air_time[:, sensor_cfg.body_ids]
    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0
    in_mode_time = torch.where(in_contact, contact_time, air_time)
    single_stance = torch.sum(in_contact.int(), dim=1) == 1
    reward = torch.min(torch.where(single_stance.unsqueeze(-1), in_mode_time, 0.0), dim=1)[0]
    reward = torch.clamp(reward, max=threshold)
    # no reward for zero command
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def feet_slide(env, sensor_cfg: SceneEntityCfg, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize feet sliding.

    This function penalizes the agent for sliding its feet on the ground. The reward is computed as the
    norm of the linear velocity of the feet multiplied by a binary contact sensor. This ensures that the
    agent is penalized only when the feet are in contact with the ground.
    """
    # Penalize feet sliding
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    contacts = contact_sensor.data.net_forces_w_history[:, :, sensor_cfg.body_ids, :].norm(dim=-1).max(dim=1)[0] > 1.0
    asset = env.scene[asset_cfg.name]

    body_vel = asset.data.body_lin_vel_w[:, asset_cfg.body_ids, :2]
    reward = torch.sum(body_vel.norm(dim=-1) * contacts, dim=1)
    return reward


<<<<<<< HEAD
def feet_clearance_biped(
    env,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    target_height: float = 0.04,
) -> torch.Tensor:
    """Reward the swing foot lifting above the stance foot during single-stance stepping."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    asset = env.scene[asset_cfg.name]

    contact_time = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids]
    in_contact = contact_time > 0.0
    single_stance = torch.sum(in_contact.int(), dim=1) == 1

    foot_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]
    height_above_lowest_foot = foot_z - torch.min(foot_z, dim=1, keepdim=True)[0]
    swing = ~in_contact
    clearance = torch.clamp(height_above_lowest_foot / target_height, min=0.0, max=1.0)
    reward = torch.sum(clearance * swing.float(), dim=1) * single_stance.float()
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.1
    return reward


def phase_alternating_gait(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    period: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Reward matching a simple left-stance/right-stance alternating contact pattern."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    left_contact = in_contact[:, 0]
    right_contact = in_contact[:, 1]

    left_stance_phase = _phase(env, period) < left_stance_ratio
    desired_stance_contact = torch.where(left_stance_phase, left_contact, right_contact)
    desired_swing_air = torch.where(left_stance_phase, ~right_contact, ~left_contact)

    reward = (desired_stance_contact & desired_swing_air).float()
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return reward


def phase_contact_mismatch_penalty(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    period: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Penalize contact patterns that ignore the scheduled left/right swing phase."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    left_contact = in_contact[:, 0]
    right_contact = in_contact[:, 1]

    left_stance_phase = _phase(env, period) < left_stance_ratio
    stance_contact = torch.where(left_stance_phase, left_contact, right_contact)
    swing_contact = torch.where(left_stance_phase, right_contact, left_contact)

    penalty = (~stance_contact).float() + swing_contact.float()
    penalty *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return penalty


def phase_feet_clearance_biped(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    period: float,
    target_height: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Reward the scheduled swing foot lifting above the scheduled stance foot."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    asset = env.scene[asset_cfg.name]

    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    foot_z = asset.data.body_pos_w[:, asset_cfg.body_ids, 2]
    left_stance_phase = _phase(env, period) < left_stance_ratio

    stance_contact = torch.where(left_stance_phase, in_contact[:, 0], in_contact[:, 1])
    swing_air = torch.where(left_stance_phase, ~in_contact[:, 1], ~in_contact[:, 0])
    swing_clearance = torch.where(left_stance_phase, foot_z[:, 1] - foot_z[:, 0], foot_z[:, 0] - foot_z[:, 1])

    reward = torch.clamp(swing_clearance / target_height, min=0.0, max=1.0)
    reward *= (stance_contact & swing_air).float()
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return reward


def phase_swing_foot_forward(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    period: float,
    target_step_length: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Reward the scheduled swing foot moving forward of the scheduled stance foot."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    foot_x = _body_pos_yaw_frame(env, asset_cfg)[:, :, 0]

    left_stance_phase = _phase(env, period) < left_stance_ratio
    stance_contact = torch.where(left_stance_phase, in_contact[:, 0], in_contact[:, 1])
    swing_air = torch.where(left_stance_phase, ~in_contact[:, 1], ~in_contact[:, 0])
    swing_forward = torch.where(left_stance_phase, foot_x[:, 1] - foot_x[:, 0], foot_x[:, 0] - foot_x[:, 1])

    reward = torch.clamp(swing_forward / target_step_length, min=0.0, max=1.0)
    reward *= (stance_contact & swing_air).float()
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return reward


def phase_swing_foot_forward_velocity(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    period: float,
    target_velocity: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Reward the scheduled swing foot moving forward relative to the base."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    foot_vel_x = _body_rel_lin_vel_yaw_frame(env, asset_cfg)[:, :, 0]

    left_stance_phase = _phase(env, period) < left_stance_ratio
    stance_contact = torch.where(left_stance_phase, in_contact[:, 0], in_contact[:, 1])
    swing_air = torch.where(left_stance_phase, ~in_contact[:, 1], ~in_contact[:, 0])
    swing_vel_x = torch.where(left_stance_phase, foot_vel_x[:, 1], foot_vel_x[:, 0])

    reward = torch.clamp(swing_vel_x / target_velocity, min=0.0, max=1.0)
    reward *= (stance_contact & swing_air).float()
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return reward


def phase_swing_foot_behind_l2(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    period: float,
    min_step_length: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Penalize the scheduled swing foot lagging behind the scheduled stance foot."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    foot_x = _body_pos_yaw_frame(env, asset_cfg)[:, :, 0]

    left_stance_phase = _phase(env, period) < left_stance_ratio
    stance_contact = torch.where(left_stance_phase, in_contact[:, 0], in_contact[:, 1])
    swing_air = torch.where(left_stance_phase, ~in_contact[:, 1], ~in_contact[:, 0])
    swing_forward = torch.where(left_stance_phase, foot_x[:, 1] - foot_x[:, 0], foot_x[:, 0] - foot_x[:, 1])

    shortfall = torch.clamp((min_step_length - swing_forward) / min_step_length, min=0.0, max=2.0)
    penalty = torch.square(shortfall)
    penalty *= (stance_contact & swing_air).float()
    penalty *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return penalty


def phase_swing_foot_inward_l2(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    period: float,
    min_side_y: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Penalize the scheduled swing foot drifting inward toward the centerline."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    foot_y = _body_pos_yaw_frame(env, asset_cfg)[:, :, 1]

    left_stance_phase = _phase(env, period) < left_stance_ratio
    stance_contact = torch.where(left_stance_phase, in_contact[:, 0], in_contact[:, 1])
    swing_air = torch.where(left_stance_phase, ~in_contact[:, 1], ~in_contact[:, 0])
    swing_side_y = torch.where(left_stance_phase, -foot_y[:, 1], foot_y[:, 0])

    shortfall = torch.clamp((min_side_y - swing_side_y) / min_side_y, min=0.0, max=2.0)
    penalty = torch.square(shortfall)
    penalty *= (stance_contact & swing_air).float()
    penalty *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return penalty


def phase_swing_lateral_velocity_l2(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    period: float,
    free_velocity: float,
    target_velocity: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Penalize the scheduled swing foot swiping sideways too quickly."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    foot_vel_y = _body_rel_lin_vel_yaw_frame(env, asset_cfg)[:, :, 1]

    left_stance_phase = _phase(env, period) < left_stance_ratio
    stance_contact = torch.where(left_stance_phase, in_contact[:, 0], in_contact[:, 1])
    swing_air = torch.where(left_stance_phase, ~in_contact[:, 1], ~in_contact[:, 0])
    swing_lateral_speed = torch.where(left_stance_phase, foot_vel_y[:, 1].abs(), foot_vel_y[:, 0].abs())

    excess_speed = torch.clamp(swing_lateral_speed - free_velocity, min=0.0)
    penalty = torch.square(excess_speed / target_velocity)
    penalty *= (stance_contact & swing_air).float()
    penalty *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return penalty


def phase_swing_knee_bend(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    asset_cfg: SceneEntityCfg,
    period: float,
    target_bend: float,
    left_stance_ratio: float = 0.5,
) -> torch.Tensor:
    """Reward the scheduled swing knee bending more than the scheduled stance knee."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    asset = env.scene[asset_cfg.name]

    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    knee_pos = asset.data.joint_pos[:, asset_cfg.joint_ids]
    left_stance_phase = _phase(env, period) < left_stance_ratio

    stance_contact = torch.where(left_stance_phase, in_contact[:, 0], in_contact[:, 1])
    swing_air = torch.where(left_stance_phase, ~in_contact[:, 1], ~in_contact[:, 0])
    stance_knee = torch.where(left_stance_phase, knee_pos[:, 0], knee_pos[:, 1])
    swing_knee = torch.where(left_stance_phase, knee_pos[:, 1], knee_pos[:, 0])

    reward = torch.clamp((swing_knee - stance_knee) / target_bend, min=0.0, max=1.0)
    reward *= (stance_contact & swing_air).float()
    reward *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return reward


def paired_body_lateral_placement_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_width: float,
) -> torch.Tensor:
    """Penalize paired bodies drifting away from a centered nominal lateral width."""
    body_y = _body_pos_yaw_frame(env, asset_cfg)[:, :, 1]
    width = torch.abs(body_y[:, 0] - body_y[:, 1])
    center = torch.mean(body_y, dim=1)
    return torch.square(center) + torch.square(width - target_width)


def foot_lateral_placement_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    target_width: float,
) -> torch.Tensor:
    """Penalize drifting the feet sideways away from a centered nominal stance width."""
    return paired_body_lateral_placement_l2(env, asset_cfg, target_width)


def bodies_too_close_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    min_width: float,
) -> torch.Tensor:
    """Penalize paired body spacing that collapses below a minimum lateral width."""
    body_y = _body_pos_yaw_frame(env, asset_cfg)[:, :, 1]
    width = torch.abs(body_y[:, 0] - body_y[:, 1])
    return torch.square(torch.clamp((min_width - width) / min_width, min=0.0))


def feet_too_close_l2(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    min_width: float,
) -> torch.Tensor:
    """Penalize foot spacing that collapses below a minimum lateral width."""
    return bodies_too_close_l2(env, asset_cfg, min_width)


def double_support_penalty(
    env: ManagerBasedRLEnv, command_name: str, sensor_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Penalize both feet remaining planted while a forward command is active."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    in_contact = contact_sensor.data.current_contact_time[:, sensor_cfg.body_ids] > 0.0
    both_feet_contact = torch.all(in_contact, dim=1)
    penalty = both_feet_contact.float()
    penalty *= torch.norm(env.command_manager.get_command(command_name)[:, :2], dim=1) > 0.03
    return penalty


=======
>>>>>>> f3902ba0 (modify-locomotion-rl)
def track_lin_vel_xy_yaw_frame_exp(
    env, std: float, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of linear velocity commands (xy axes) in the gravity aligned robot frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_rotate_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1
    )
    return torch.exp(-lin_vel_error / std**2)


<<<<<<< HEAD
def track_lin_vel_xy_yaw_frame_upright_exp(
    env,
    std: float,
    command_name: str,
    tilt_std: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward linear velocity tracking only when the base remains upright."""
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_rotate_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    lin_vel_error = torch.sum(
        torch.square(env.command_manager.get_command(command_name)[:, :2] - vel_yaw[:, :2]), dim=1
    )
    tilt_error = torch.sum(torch.square(asset.data.projected_gravity_b[:, :2]), dim=1)
    return torch.exp(-lin_vel_error / std**2) * torch.exp(-tilt_error / tilt_std**2)


def forward_velocity_reward(
    env, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward forward base velocity in the robot yaw frame, saturated by the commanded speed."""
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_rotate_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    command_x = torch.clamp(env.command_manager.get_command(command_name)[:, 0], min=0.1)
    forward_speed = torch.clamp(vel_yaw[:, 0], min=0.0)
    return torch.minimum(forward_speed, command_x) / command_x


def world_forward_velocity_reward(
    env, command_name: str, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward forward base velocity in the world x direction, saturated by the commanded speed."""
    asset = env.scene[asset_cfg.name]
    command_x = torch.clamp(env.command_manager.get_command(command_name)[:, 0], min=0.1)
    forward_speed = torch.clamp(asset.data.root_lin_vel_w[:, 0], min=0.0)
    return torch.minimum(forward_speed, command_x) / command_x


def lateral_velocity_l2(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize sideways base velocity in the robot yaw frame."""
    asset = env.scene[asset_cfg.name]
    vel_yaw = quat_rotate_inverse(yaw_quat(asset.data.root_quat_w), asset.data.root_lin_vel_w[:, :3])
    return torch.square(vel_yaw[:, 1])


def yaw_rate_l2(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize yaw angular velocity in the world frame."""
    asset = env.scene[asset_cfg.name]
    return torch.square(asset.data.root_ang_vel_w[:, 2])


def heading_yaw_l2(env, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    """Penalize drifting away from the zero-yaw heading."""
    asset = env.scene[asset_cfg.name]
    quat = asset.data.root_quat_w
    yaw = torch.atan2(
        2.0 * (quat[:, 0] * quat[:, 3] + quat[:, 1] * quat[:, 2]),
        1.0 - 2.0 * (torch.square(quat[:, 2]) + torch.square(quat[:, 3])),
    )
    return torch.square(yaw)


=======
>>>>>>> f3902ba0 (modify-locomotion-rl)
def track_ang_vel_z_world_exp(
    env, command_name: str, std: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Reward tracking of angular velocity commands (yaw) in world frame using exponential kernel."""
    # extract the used quantities (to enable type-hinting)
    asset = env.scene[asset_cfg.name]
    ang_vel_error = torch.square(env.command_manager.get_command(command_name)[:, 2] - asset.data.root_ang_vel_w[:, 2])
    return torch.exp(-ang_vel_error / std**2)
