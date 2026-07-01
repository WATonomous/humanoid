from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.assets import RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor, FrameTransformer
from isaaclab.utils.math import combine_frame_transforms, quat_apply

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def object_is_lifted(
    env: ManagerBasedRLEnv, minimal_height: float, object_cfg: SceneEntityCfg = SceneEntityCfg("cube")
) -> torch.Tensor:
    """Reward the agent for lifting the object above the minimal height."""
    object: RigidObject = env.scene[object_cfg.name]

    # above return 1.0, else 0.0
    return torch.where(object.data.root_pos_w[:, 2] > minimal_height, 1.0, 0.0)


def object_ee_distance(
    env: ManagerBasedRLEnv,
    std: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """End-effector to cube reward using tanh-kernel (TCP via FrameTransformer)."""
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    cube_pos_w = cube.data.root_pos_w
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    distance = torch.norm(cube_pos_w - ee_w, dim=1)
    return 1 - torch.tanh(distance / std)


def object_goal_distance(
    env: ManagerBasedRLEnv,
    std: float,
    minimal_height: float,
    command_name: str,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """ Cube to target pos reward using tanh-kernel, only awarded if above object_is_lifted() threshold """
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)

    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(
        robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b)

    distance = torch.norm(des_pos_w - object.data.root_pos_w, dim=1)
    # rewarded if the object is lifted above the threshold
    return (object.data.root_pos_w[:, 2] > minimal_height) * (1 - torch.tanh(distance / std))


def _gripper_opening_normalized(robot, asset_cfg: SceneEntityCfg, open_width: float) -> torch.Tensor:
    """Return 0 when closed and 1 when fully open for a two-finger prismatic gripper."""
    joint_pos = robot.data.joint_pos[:, asset_cfg.joint_ids]
    if joint_pos.shape[1] < 2:
        return torch.zeros(joint_pos.shape[0], device=joint_pos.device)
    opening = torch.abs(joint_pos[:, 0] - joint_pos[:, 1])
    return torch.clamp(opening / open_width, 0.0, 1.0)


def _gripper_opening(robot, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    joint_pos = robot.data.joint_pos[:, asset_cfg.joint_ids]
    if joint_pos.shape[1] < 2:
        return torch.zeros(joint_pos.shape[0], device=joint_pos.device)
    return torch.abs(joint_pos[:, 0] - joint_pos[:, 1])


def _gripper_width_score(
    robot,
    asset_cfg: SceneEntityCfg,
    target_width: float,
    width_std: float,
) -> torch.Tensor:
    opening = _gripper_opening(robot, asset_cfg)
    return torch.exp(-torch.square(opening - target_width) / (width_std * width_std))


def _contact_force_norms(env: ManagerBasedRLEnv, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
    """Net contact-force magnitude for selected fingertip bodies."""
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    forces = contact_sensor.data.net_forces_w[:, sensor_cfg.body_ids, :]  # type: ignore[index]
    return torch.linalg.vector_norm(forces, dim=-1)


def _finger_tip_positions_w(robot, fingertip_cfg: SceneEntityCfg, tip_local_offsets: list[tuple[float, float, float]]):
    """Return the two distal fingertip positions in world frame."""
    body_pos = robot.data.body_pos_w[:, fingertip_cfg.body_ids, :]
    body_quat = robot.data.body_quat_w[:, fingertip_cfg.body_ids, :]
    offsets = torch.tensor(tip_local_offsets, device=body_pos.device, dtype=body_pos.dtype)
    offsets = offsets.unsqueeze(0).expand(body_pos.shape[0], -1, -1)
    tips = body_pos + quat_apply(body_quat.reshape(-1, 4), offsets.reshape(-1, 3)).reshape(body_pos.shape)
    return tips[:, 0, :], tips[:, 1, :]


def _object_between_fingertips_score(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    robot = env.scene[fingertip_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    tip_a, tip_b = _finger_tip_positions_w(robot, fingertip_cfg, tip_local_offsets)

    cube_pos = cube.data.root_pos_w
    center = 0.5 * (tip_a + tip_b)
    finger_vec = tip_b - tip_a
    finger_len_sq = torch.sum(finger_vec * finger_vec, dim=1).clamp_min(1.0e-8)

    # Project cube center onto the line between the fingertips. The best grasp has
    # the cube close to the midpoint and close to the finger gap line.
    t = torch.sum((cube_pos - tip_a) * finger_vec, dim=1) / finger_len_sq
    between_score = torch.exp(-torch.square(t - 0.5) / 0.18)
    closest = tip_a + torch.clamp(t, 0.0, 1.0).unsqueeze(1) * finger_vec

    center_distance = torch.norm(cube_pos - center, dim=1)
    perpendicular_distance = torch.norm(cube_pos - closest, dim=1)
    center_score = 1.0 - torch.tanh(center_distance / center_std)
    perpendicular_score = 1.0 - torch.tanh(perpendicular_distance / perpendicular_std)
    return between_score * center_score * perpendicular_score


def _two_finger_contact_grasp_score(
    env: ManagerBasedRLEnv,
    contact_force_std: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    sensor_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Score an actual physical pinch: both fingertips in contact around the cube."""
    robot = env.scene[gripper_cfg.name]
    force_norms = _contact_force_norms(env, sensor_cfg)
    contact_score_per_tip = 1.0 - torch.exp(-force_norms / max(contact_force_std, 1.0e-6))
    two_tip_contact = torch.prod(contact_score_per_tip, dim=1)

    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    return two_tip_contact * closed * centered


def assisted_grasp_follow(
    env: ManagerBasedRLEnv,
    closed_width: float,
    center_threshold: float,
    perpendicular_threshold: float,
    projection_margin: float,
    release_center_threshold: float,
    release_perpendicular_threshold: float,
    height_start: float,
    height_target: float,
    lift_speed: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Score a plausible closed pinch without moving the cube.

    This used to write the cube pose as a deadline demo helper. That made the
    GUI jittery and visually dishonest, so it is now a pure reward helper.
    """
    robot = env.scene[gripper_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    cube_pos = cube.data.root_pos_w

    tip_a, tip_b = _finger_tip_positions_w(robot, fingertip_cfg, tip_local_offsets)
    center = 0.5 * (tip_a + tip_b)
    finger_vec = tip_b - tip_a
    finger_len_sq = torch.sum(finger_vec * finger_vec, dim=1).clamp_min(1.0e-8)
    t = torch.sum((cube_pos - tip_a) * finger_vec, dim=1) / finger_len_sq
    closest = tip_a + torch.clamp(t, 0.0, 1.0).unsqueeze(1) * finger_vec
    center_distance = torch.norm(cube_pos - center, dim=1)
    perpendicular_distance = torch.norm(cube_pos - closest, dim=1)

    opening = _gripper_opening(robot, gripper_cfg)
    closed = opening <= closed_width
    in_slot = (
        closed
        & (t > -projection_margin)
        & (t < 1.0 + projection_margin)
        & (center_distance < center_threshold)
        & (perpendicular_distance < perpendicular_threshold)
    )

    height_progress = (cube_pos[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return in_slot.float() * (0.5 + height_progress)


def object_between_fingertips(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward placing the cube in the open slot between the two fingertips."""
    return _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )


def ee_above_object(
    env: ManagerBasedRLEnv,
    xy_std: float,
    z_offset: float,
    z_std: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward placing the TCP above the cube instead of side-swiping it."""
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    delta = ee_frame.data.target_pos_w[..., 0, :] - cube.data.root_pos_w[:, :3]
    xy_score = 1.0 - torch.tanh(torch.norm(delta[:, :2], dim=1) / xy_std)
    z_score = torch.exp(-torch.square(delta[:, 2] - z_offset) / (z_std * z_std))
    return xy_score * z_score


def _ee_above_object_score(
    env: ManagerBasedRLEnv,
    xy_std: float,
    z_offset: float,
    z_std: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    delta = ee_frame.data.target_pos_w[..., 0, :] - cube.data.root_pos_w[:, :3]
    xy_score = 1.0 - torch.tanh(torch.norm(delta[:, :2], dim=1) / xy_std)
    z_score = torch.exp(-torch.square(delta[:, 2] - z_offset) / (z_std * z_std))
    return xy_score * z_score


def _fingertip_center_pose_score(
    env: ManagerBasedRLEnv,
    xy_std: float,
    z_offset: float,
    z_std: float,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Score the midpoint between fingertip tips landing at the cube pinch height."""
    robot = env.scene[fingertip_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    tip_a, tip_b = _finger_tip_positions_w(robot, fingertip_cfg, tip_local_offsets)
    center = 0.5 * (tip_a + tip_b)
    delta = center - cube.data.root_pos_w[:, :3]
    xy_score = 1.0 - torch.tanh(torch.norm(delta[:, :2], dim=1) / xy_std)
    z_score = torch.exp(-torch.square(delta[:, 2] - z_offset) / (z_std * z_std))
    return xy_score * z_score


def fingertip_center_on_object(
    env: ManagerBasedRLEnv,
    xy_std: float,
    z_offset: float,
    z_std: float,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward putting the actual fingertip midpoint at the cube side-grasp height."""
    return _fingertip_center_pose_score(env, xy_std, z_offset, z_std, fingertip_cfg, tip_local_offsets, object_cfg)


def gripper_closed_on_object(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    target_width: float,
    width_std: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward a closed command only when the finger gap matches the cube."""
    robot = env.scene[gripper_cfg.name]
    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    width_score = _gripper_width_score(robot, gripper_cfg, target_width, width_std)
    return closed * centered * width_score


def gripper_width_on_object(
    env: ManagerBasedRLEnv,
    target_width: float,
    width_std: float,
    center_std: float,
    perpendicular_std: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward a stable pinch gap around the cube while it is centered between fingertips."""
    robot = env.scene[gripper_cfg.name]
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    width_score = _gripper_width_score(robot, gripper_cfg, target_width, width_std)
    return centered * width_score


def gripper_open_when_not_centered(
    env: ManagerBasedRLEnv,
    centered_threshold: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward keeping the gripper open until the cube is centered between the fingertips."""
    robot = env.scene[gripper_cfg.name]
    open_amount = _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    return (centered < centered_threshold).float() * open_amount


def object_height_when_grasped(
    env: ManagerBasedRLEnv,
    height_start: float,
    height_target: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward cube height only when the cube is centered between closed fingertips."""
    robot = env.scene[gripper_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    height_progress = (cube.data.root_pos_w[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return closed * centered * height_progress


def two_finger_contact_on_object(
    env: ManagerBasedRLEnv,
    contact_force_std: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    sensor_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward real two-finger contact while the cube is centered between the fingertips."""
    return _two_finger_contact_grasp_score(
        env,
        contact_force_std,
        center_std,
        perpendicular_std,
        open_width,
        sensor_cfg,
        gripper_cfg,
        fingertip_cfg,
        tip_local_offsets,
        object_cfg,
    )


def object_height_when_contact_grasped(
    env: ManagerBasedRLEnv,
    height_start: float,
    height_target: float,
    contact_force_std: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    sensor_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward cube lift only when the fingertips are truly pinching it."""
    cube: RigidObject = env.scene[object_cfg.name]
    grasp_score = _two_finger_contact_grasp_score(
        env,
        contact_force_std,
        center_std,
        perpendicular_std,
        open_width,
        sensor_cfg,
        gripper_cfg,
        fingertip_cfg,
        tip_local_offsets,
        object_cfg,
    )
    height_progress = (cube.data.root_pos_w[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return grasp_score * height_progress


def fingertip_lift_when_contact_grasped(
    env: ManagerBasedRLEnv,
    height_start: float,
    height_target: float,
    contact_force_std: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    sensor_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward lifting the fingertip pair only while it has a real two-finger contact grasp."""
    robot = env.scene[gripper_cfg.name]
    tip_a, tip_b = _finger_tip_positions_w(robot, fingertip_cfg, tip_local_offsets)
    fingertip_center = 0.5 * (tip_a + tip_b)
    grasp_score = _two_finger_contact_grasp_score(
        env,
        contact_force_std,
        center_std,
        perpendicular_std,
        open_width,
        sensor_cfg,
        gripper_cfg,
        fingertip_cfg,
        tip_local_offsets,
        object_cfg,
    )
    height_progress = (fingertip_center[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return grasp_score * height_progress


def fingertip_up_velocity_when_contact_grasped(
    env: ManagerBasedRLEnv,
    speed_target: float,
    contact_force_std: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    sensor_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward upward fingertip motion only while both fingertips have real contact."""
    robot = env.scene[gripper_cfg.name]
    fingertip_vel_z = robot.data.body_lin_vel_w[:, fingertip_cfg.body_ids, 2].mean(dim=1)
    upward_motion = torch.clamp(fingertip_vel_z / max(speed_target, 1.0e-6), 0.0, 1.0)
    grasp_score = _two_finger_contact_grasp_score(
        env,
        contact_force_std,
        center_std,
        perpendicular_std,
        open_width,
        sensor_cfg,
        gripper_cfg,
        fingertip_cfg,
        tip_local_offsets,
        object_cfg,
    )
    return grasp_score * upward_motion


def object_height_without_grasp(
    env: ManagerBasedRLEnv,
    height_start: float,
    height_target: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Penalty helper for lifting the cube without a centered closed grasp."""
    robot = env.scene[gripper_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    grasped = torch.clamp(closed * centered, 0.0, 1.0)
    height_progress = (cube.data.root_pos_w[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return (1.0 - grasped) * height_progress


def object_xy_displacement_without_grasp(
    env: ManagerBasedRLEnv,
    start_pos: tuple[float, float],
    max_distance: float,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Penalty helper for pushing the cube around before securing a centered closed grasp."""
    robot = env.scene[gripper_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    cube_pos_e = cube.data.root_pos_w - env.scene.env_origins

    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    grasped = torch.clamp(closed * centered, 0.0, 1.0)

    start_xy = torch.tensor(start_pos, device=cube_pos_e.device, dtype=cube_pos_e.dtype).unsqueeze(0)
    displacement = torch.norm(cube_pos_e[:, :2] - start_xy, dim=1)
    displacement_score = torch.clamp(displacement / max_distance, 0.0, 1.0)
    return (1.0 - grasped) * displacement_score


def object_velocity_without_grasp(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Penalty helper for slapping or buzzing the cube before a real grasp."""
    robot = env.scene[gripper_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]

    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    grasped = torch.clamp(closed * centered, 0.0, 1.0)

    xy_speed = torch.linalg.vector_norm(cube.data.root_lin_vel_w[:, :2], dim=1)
    angular_speed = torch.linalg.vector_norm(cube.data.root_ang_vel_w, dim=1)
    motion = torch.clamp(xy_speed, 0.0, 2.0) + 0.03 * torch.clamp(angular_speed, 0.0, 20.0)
    return (1.0 - grasped) * motion


def object_goal_distance_when_grasped(
    env: ManagerBasedRLEnv,
    std: float,
    minimal_height: float,
    command_name: str,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward moving the cube to the goal only while it is lifted in a centered closed grasp."""
    robot = env.scene[robot_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)

    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(robot.data.root_pos_w, robot.data.root_quat_w, des_pos_b)
    distance = torch.norm(des_pos_w - cube.data.root_pos_w, dim=1)

    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    lifted = (cube.data.root_pos_w[:, 2] > minimal_height).float()
    return closed * centered * lifted * (1.0 - torch.tanh(distance / std))


def gripper_closed_when_near_object(
    env: ManagerBasedRLEnv,
    threshold: float,
    open_width: float,
    asset_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward closing the gripper only after the TCP is near the cube."""
    robot = env.scene[asset_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    distance = torch.norm(cube.data.root_pos_w - ee_frame.data.target_pos_w[..., 0, :], dim=1)
    closed = 1.0 - _gripper_opening_normalized(robot, asset_cfg, open_width)
    return (distance < threshold).float() * closed


def close_action_near_object(
    env: ManagerBasedRLEnv,
    threshold: float,
    action_name: str = "gripper_action",
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward commanding gripper close once the TCP is close enough to the cube."""
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    distance = torch.norm(cube.data.root_pos_w - ee_frame.data.target_pos_w[..., 0, :], dim=1)
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    close_command = torch.clamp(-gripper_action, 0.0, 1.0)
    return (distance < threshold).float() * close_command


def close_action_when_centered(
    env: ManagerBasedRLEnv,
    centered_threshold: float,
    center_std: float,
    perpendicular_std: float,
    action_name: str,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward negative gripper action when the cube is centered between the fingertips."""
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    close_command = torch.clamp(-gripper_action, 0.0, 1.0)
    return (centered > centered_threshold).float() * close_command


def close_action_when_grasp_ready(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    xy_std: float,
    z_offset: float,
    z_std: float,
    action_name: str,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward close command only when the claw is centered and landed above the cube."""
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    above = _ee_above_object_score(env, xy_std, z_offset, z_std, object_cfg, ee_frame_cfg)
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    close_command = torch.clamp(-gripper_action, 0.0, 1.0)
    return centered * above * close_command


def close_action_when_not_grasp_ready(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    xy_std: float,
    z_offset: float,
    z_std: float,
    action_name: str,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Penalty helper for commanding close before the claw is centered and above the cube."""
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    above = _ee_above_object_score(env, xy_std, z_offset, z_std, object_cfg, ee_frame_cfg)
    readiness = torch.clamp(centered * above, 0.0, 1.0)
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    close_command = torch.clamp(-gripper_action, 0.0, 1.0)
    return close_command * (1.0 - readiness)


def open_action_when_grasp_ready(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    xy_std: float,
    z_offset: float,
    z_std: float,
    action_name: str,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Penalty helper for keeping the gripper open once the claw is ready to grasp."""
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    above = _ee_above_object_score(env, xy_std, z_offset, z_std, object_cfg, ee_frame_cfg)
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    open_command = torch.clamp(gripper_action, 0.0, 1.0)
    return centered * above * open_command


def close_action_when_pinch_ready(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    xy_std: float,
    z_offset: float,
    z_std: float,
    action_name: str,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward close command when the fingertip midpoint is at cube pinch height."""
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    landed = _fingertip_center_pose_score(env, xy_std, z_offset, z_std, fingertip_cfg, tip_local_offsets, object_cfg)
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    close_command = torch.clamp(-gripper_action, 0.0, 1.0)
    return centered * landed * close_command


def close_action_when_not_pinch_ready(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    xy_std: float,
    z_offset: float,
    z_std: float,
    action_name: str,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Penalty helper for close command before the fingertip midpoint has landed."""
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    landed = _fingertip_center_pose_score(env, xy_std, z_offset, z_std, fingertip_cfg, tip_local_offsets, object_cfg)
    readiness = torch.clamp(centered * landed, 0.0, 1.0)
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    close_command = torch.clamp(-gripper_action, 0.0, 1.0)
    return close_command * (1.0 - readiness)


def open_action_when_pinch_ready(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    xy_std: float,
    z_offset: float,
    z_std: float,
    action_name: str,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Penalty helper for staying open once the fingertip midpoint has landed."""
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    landed = _fingertip_center_pose_score(env, xy_std, z_offset, z_std, fingertip_cfg, tip_local_offsets, object_cfg)
    gripper_action = env.action_manager.get_term(action_name).raw_actions[:, 0]
    open_command = torch.clamp(gripper_action, 0.0, 1.0)
    return centered * landed * open_command


def cube_follows_gripper_when_grasped(
    env: ManagerBasedRLEnv,
    center_std: float,
    perpendicular_std: float,
    open_width: float,
    height_start: float,
    height_target: float,
    vertical_offset: float,
    vertical_std: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward the lifted cube staying under the gripper after a closed centered grasp."""
    robot = env.scene[gripper_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    closed = 1.0 - _gripper_opening_normalized(robot, gripper_cfg, open_width)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    ee_pos = ee_frame.data.target_pos_w[..., 0, :]
    vertical_score = torch.exp(-torch.square((ee_pos[:, 2] - cube.data.root_pos_w[:, 2]) - vertical_offset) / (vertical_std * vertical_std))
    height_progress = (cube.data.root_pos_w[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return closed * centered * vertical_score * height_progress


def gripper_open_when_far_object(
    env: ManagerBasedRLEnv,
    threshold: float,
    open_width: float,
    asset_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward keeping the gripper open while approaching the cube."""
    robot = env.scene[asset_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    distance = torch.norm(cube.data.root_pos_w - ee_frame.data.target_pos_w[..., 0, :], dim=1)
    open_amount = _gripper_opening_normalized(robot, asset_cfg, open_width)
    return (distance > threshold).float() * open_amount

def object_height_above_initial(
    env: ManagerBasedRLEnv,
    height_start: float,
    height_target: float,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Continuous reward for any upward cube motion before the sparse lifted bonus is reached."""
    object: RigidObject = env.scene[object_cfg.name]
    height_progress = (object.data.root_pos_w[:, 2] - height_start) / (height_target - height_start)
    return torch.clamp(height_progress, 0.0, 1.0)

def ee_lift_when_closed_near_object(
    env: ManagerBasedRLEnv,
    xy_threshold: float,
    height_start: float,
    height_target: float,
    open_width: float,
    asset_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
) -> torch.Tensor:
    """Reward moving the closed gripper upward while horizontally above the cube."""
    robot = env.scene[asset_cfg.name]
    cube: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_pos_w = ee_frame.data.target_pos_w[..., 0, :]
    xy_distance = torch.norm(cube.data.root_pos_w[:, :2] - ee_pos_w[:, :2], dim=1)
    closed = 1.0 - _gripper_opening_normalized(robot, asset_cfg, open_width)
    height_progress = (ee_pos_w[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return (xy_distance < xy_threshold).float() * closed * height_progress


def fingertip_lift_when_pinch_ready(
    env: ManagerBasedRLEnv,
    height_start: float,
    height_target: float,
    center_std: float,
    perpendicular_std: float,
    target_width: float,
    width_std: float,
    gripper_cfg: SceneEntityCfg,
    fingertip_cfg: SceneEntityCfg,
    tip_local_offsets: list[tuple[float, float, float]],
    object_cfg: SceneEntityCfg = SceneEntityCfg("cube"),
) -> torch.Tensor:
    """Reward lifting the fingertip midpoint only while holding a plausible pinch.

    If the hand lifts away and the cube stays on the table, the centered score
    drops, so this cannot pay for a free hand lift.
    """
    robot = env.scene[gripper_cfg.name]
    tip_a, tip_b = _finger_tip_positions_w(robot, fingertip_cfg, tip_local_offsets)
    fingertip_center = 0.5 * (tip_a + tip_b)
    centered = _object_between_fingertips_score(
        env, center_std, perpendicular_std, fingertip_cfg, tip_local_offsets, object_cfg
    )
    width_score = _gripper_width_score(robot, gripper_cfg, target_width, width_std)
    height_progress = (fingertip_center[:, 2] - height_start) / (height_target - height_start)
    height_progress = torch.clamp(height_progress, 0.0, 1.0)
    return centered * width_score * height_progress
