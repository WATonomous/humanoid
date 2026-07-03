from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import matrix_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _robot_ee_pose(env: ManagerBasedRLEnv):
    """(ee_tcp_pos, ee_tcp_quat, lfinger_pos, rfinger_pos) from robot articulation bodies."""
    robot = env.scene["robot"]
    # For bimanual arm, the right hand fingers are link7 and link8
    ee_ids, _ = robot.find_bodies("link7", preserve_order=True)
    thumb_ids, _ = robot.find_bodies("link8", preserve_order=True)
    if not ee_ids or not thumb_ids:
        return None
    lfinger_pos = robot.data.body_pos_w[:, ee_ids[0], :]
    rfinger_pos = robot.data.body_pos_w[:, thumb_ids[0], :]
    # The true End-Effector TCP is exactly halfway between the two fingers!
    ee_tcp_pos = (lfinger_pos + rfinger_pos) / 2.0
    ee_tcp_quat = robot.data.body_quat_w[:, ee_ids[0], :]
    return (ee_tcp_pos, ee_tcp_quat, lfinger_pos, rfinger_pos)


def approach_ee_handle(env: ManagerBasedRLEnv, threshold: float) -> torch.Tensor:
    r"""Reward the robot for reaching the drawer handle using inverse-square law.

    It uses a piecewise function to reward the robot for reaching the handle.

    .. math::

        reward = \begin{cases}
            2 * (1 / (1 + distance^2))^2 & \text{if } distance \leq threshold \\
            (1 / (1 + distance^2))^2 & \text{otherwise}
        \end{cases}

    """
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
    ee_tcp_pos, _, _, _ = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]

    distance = torch.norm(handle_pos - ee_tcp_pos, dim=-1, p=2)
    reward = 1.0 / (1.0 + distance**2)
    reward = torch.pow(reward, 2)
    return torch.where(distance <= threshold, 2 * reward, reward)


def align_ee_handle(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Reward for aligning the end-effector with the handle."""
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
    _, ee_tcp_quat, _, _ = pose
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]

    ee_tcp_rot_mat = matrix_from_quat(ee_tcp_quat)
    handle_mat = matrix_from_quat(handle_quat)

    handle_x, handle_y = handle_mat[..., 0], handle_mat[..., 1]
    ee_tcp_x, ee_tcp_z = ee_tcp_rot_mat[..., 0], ee_tcp_rot_mat[..., 2]

    align_z = torch.bmm(ee_tcp_z.unsqueeze(1), -handle_x.unsqueeze(-1)).squeeze(-1).squeeze(-1)
    align_x = torch.bmm(ee_tcp_x.unsqueeze(1), -handle_y.unsqueeze(-1)).squeeze(-1).squeeze(-1)
    return 0.5 * (torch.sign(align_z) * align_z**2 + torch.sign(align_x) * align_x**2)


def align_grasp_around_handle(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Bonus for correct hand orientation around the handle."""
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device, dtype=torch.bool)
    _, _, lfinger_pos, rfinger_pos = pose

    is_graspable = (rfinger_pos[:, 2] < handle_pos[:, 2]) & (lfinger_pos[:, 2] > handle_pos[:, 2])
    return is_graspable


def approach_gripper_handle(env: ManagerBasedRLEnv, offset: float = 0.04) -> torch.Tensor:
    """Reward the robot's gripper reaching the drawer handle with the right pose."""
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
    _, _, lfinger_pos, rfinger_pos = pose

    lfinger_dist = torch.abs(lfinger_pos[:, 2] - handle_pos[:, 2])
    rfinger_dist = torch.abs(rfinger_pos[:, 2] - handle_pos[:, 2])
    is_graspable = (rfinger_pos[:, 2] < handle_pos[:, 2]) & (lfinger_pos[:, 2] > handle_pos[:, 2])

    return is_graspable * ((offset - lfinger_dist) + (offset - rfinger_dist))


def grasp_handle(
    env: ManagerBasedRLEnv, threshold: float, open_joint_pos: float, asset_cfg: SceneEntityCfg
) -> torch.Tensor:
    """Reward for closing the fingers when being close to the handle."""
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
    ee_tcp_pos, _, _, _ = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    gripper_joint_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids]

    distance = torch.norm(handle_pos - ee_tcp_pos, dim=-1, p=2)
    is_close = distance <= threshold

    # The physics engine will now naturally clamp gripper_joint_pos to 0.015!
    return is_close * torch.sum(open_joint_pos - torch.abs(gripper_joint_pos), dim=-1)


def _claw_distances(env: ManagerBasedRLEnv):
    """Return (d_link7, d_link8, lfinger_pos, rfinger_pos, handle_pos) or None."""
    pose = _robot_ee_pose(env)
    if pose is None:
        return None
    _, _, lfinger_pos, rfinger_pos = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    d_link7 = torch.norm(handle_pos - lfinger_pos, dim=-1, p=2)
    d_link8 = torch.norm(handle_pos - rfinger_pos, dim=-1, p=2)
    return d_link7, d_link8, lfinger_pos, rfinger_pos, handle_pos


# Module-level tracking: closest each claw has gotten within the current PPO iteration.
_last_claw_print_step: int = -1
_min_d7_this_iter: float = float("inf")
_min_d8_this_iter: float = float("inf")


def single_claw_proximity(env: ManagerBasedRLEnv, contact_radius: float = 0.06) -> torch.Tensor:
    """Breadcrumb reward: points for each individual claw being near the handle (OR logic).

    Gives a soft Gaussian signal for link7 and link8 independently so the arm
    learns to bring EITHER finger close before we demand both.
    Prints the closest each claw got during the iteration, once per PPO iteration.
    """
    global _last_claw_print_step, _min_d7_this_iter, _min_d8_this_iter

    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)

    d_link7, d_link8, _, _, _ = result

    k = 3.0 / contact_radius
    score_link7 = torch.exp(-k * d_link7)
    score_link8 = torch.exp(-k * d_link8)

    # Track the closest any env got this iteration
    _min_d7_this_iter = min(_min_d7_this_iter, d_link7.min().item())
    _min_d8_this_iter = min(_min_d8_this_iter, d_link8.min().item())

    # One print per PPO iteration.
    # common_step_counter increments by 1 per env.step() call (not by num_envs).
    # RSL-RL collects num_steps_per_env=96 steps per iteration, so counter advances by 96.
    log_interval = 96
    if env.common_step_counter > 0 and env.common_step_counter - _last_claw_print_step >= log_interval:
        print(
            f"[Claw best] iter_end={env.common_step_counter} | "
            f"link7 closest: {_min_d7_this_iter:.3f}m  "
            f"link8 closest: {_min_d8_this_iter:.3f}m",
            flush=True,
        )
        _last_claw_print_step = env.common_step_counter
        _min_d7_this_iter = float("inf")
        _min_d8_this_iter = float("inf")

    # Sum (not product) so either finger getting close earns points
    return score_link7 + score_link8


def dual_claw_straddle(env: ManagerBasedRLEnv, contact_radius: float = 0.04) -> torch.Tensor:
    """Reward both claws being close AND on OPPOSITE sides of the handle center.

    Uses a direction-agnostic dot-product check:
      - Compute the unit vector from the handle to each finger: u7, u8
      - If dot(u7, u8) < 0, the fingers are on opposite sides of the handle
        along WHATEVER axis they happen to be on (any line through the origin).
      - This is correct regardless of which direction the robot approaches from.

    Gate 1 (proximity): both fingers within contact_radius of handle center.
    Gate 2 (opposite-side): dot(u7, u8) < 0  →  angle between them > 90°.
    Perfect opposite sides → cos = -1 → score = 1.0
    Perpendicular          → cos =  0 → score = 0.5
    Same side              → cos = +1 → score ≈ 0.0
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)

    d_link7, d_link8, lfinger_pos, rfinger_pos, handle_pos = result

    # ── Gate 1: proximity ─────────────────────────────────────────────────────
    k = 3.0 / contact_radius
    prox_link7 = torch.exp(-k * d_link7)
    prox_link8 = torch.exp(-k * d_link8)
    dual_proximity = prox_link7 * prox_link8  # AND gate — both must be close

    # ── Gate 2: direction-agnostic opposite-side check ────────────────────────
    # Vectors from handle center to each finger
    vec7 = lfinger_pos - handle_pos  # (N, 3)
    vec8 = rfinger_pos - handle_pos  # (N, 3)

    # Normalize (add small epsilon to avoid division by zero)
    u7 = vec7 / (torch.norm(vec7, dim=-1, keepdim=True) + 1e-6)  # (N, 3)
    u8 = vec8 / (torch.norm(vec8, dim=-1, keepdim=True) + 1e-6)  # (N, 3)

    # Cosine of the angle between the two finger directions
    cos_angle = (u7 * u8).sum(dim=-1)  # (N,) — range [-1, +1]

    # Sigmoid gate: scores near 1.0 when cos < 0 (opposite sides), 0.0 when same side.
    # Sharpness factor 8 gives a clear threshold around cos = 0.
    opposite_side_score = torch.sigmoid(-8.0 * cos_angle)

    return dual_proximity * opposite_side_score


def dual_contact_pull(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, contact_radius: float = 0.04) -> torch.Tensor:
    """Maximum points: both claws on opposite sides of the handle AND the drawer is moving.

    Reuses dual_claw_straddle as the contact gate, then scales by drawer_pos.
    This is the top-of-pyramid reward — it only fires when everything else is correct.
    """
    straddle_score = dual_claw_straddle(env, contact_radius=contact_radius)
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    return straddle_score * drawer_pos


def open_drawer_bonus(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Bonus for opening the drawer given by the joint position of the drawer.
    Gives a MASSIVE multiplier if the robot's hand is physically on the handle while it opens!
    """
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]

    pose = _robot_ee_pose(env)
    if pose is None:
        return drawer_pos

    ee_tcp_pos, _, lfinger_pos, rfinger_pos = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]

    # Use the exact Z-coordinate of the offset point!
    target_z = handle_pos[..., 2]
    dz = torch.abs(target_z - ee_tcp_pos[..., 2])

    # X/Y distance
    dx_dy = torch.norm(handle_pos[..., :2] - ee_tcp_pos[..., :2], dim=-1, p=2)

    # ASYMMETRIC GRADIENT: ban the top lip
    is_too_high = ee_tcp_pos[..., 2] > (target_z + 0.02)
    z_score = torch.where(is_too_high, torch.zeros_like(dz), torch.exp(-50.0 * dz))

    # Wide breadcrumb trail to get the hand generally near the handle
    xy_score = torch.where(dx_dy <= 0.15, torch.exp(-20.0 * dx_dy), torch.zeros_like(dx_dy))

    # BULLSEYE MULTIPLIER: up to 5x for dead-center
    bullseye_multiplier = 1.0 + torch.where(dx_dy <= 0.04, 4.0 * torch.exp(-150.0 * dx_dy), torch.zeros_like(dx_dy))

    is_close = z_score * xy_score * bullseye_multiplier

    # Claw closed multiplier (10x)
    finger_dist = torch.norm(lfinger_pos - rfinger_pos, dim=-1, p=2)
    is_claw_closed = (finger_dist < 0.06).float()
    claw_multiplier = 1.0 + (is_claw_closed * 9.0)

    return is_close * drawer_pos * claw_multiplier


def straddle_handle(env: ManagerBasedRLEnv, threshold: float) -> torch.Tensor:
    """Reward the robot for threading its fingers on strictly opposite sides of the handle."""
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)

    ee_tcp_pos, _, lfinger_pos, rfinger_pos = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]

    # Only reward straddling if the hand is physically near the handle
    distance_to_handle = torch.norm(handle_pos - ee_tcp_pos, dim=-1, p=2)
    is_close = (distance_to_handle <= threshold).float()

    dist_thumb_to_handle = torch.norm(handle_pos - rfinger_pos, dim=-1, p=2)
    dist_index_to_handle = torch.norm(handle_pos - lfinger_pos, dim=-1, p=2)
    dist_thumb_to_index = torch.norm(lfinger_pos - rfinger_pos, dim=-1, p=2)

    linearity_deviation = (dist_thumb_to_handle + dist_index_to_handle) - dist_thumb_to_index
    straddle_score = torch.exp(-50.0 * linearity_deviation)

    return is_close * straddle_score


def multi_stage_open_drawer(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Multi-stage bonus for opening the drawer."""
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]

    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)

    ee_tcp_pos, _, lfinger_pos, rfinger_pos = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]

    target_z = handle_pos[..., 2]
    dz = torch.abs(target_z - ee_tcp_pos[..., 2])
    dx_dy = torch.norm(handle_pos[..., :2] - ee_tcp_pos[..., :2], dim=-1, p=2)

    is_too_high = ee_tcp_pos[..., 2] > (target_z + 0.02)
    z_score = torch.where(is_too_high, torch.zeros_like(dz), torch.exp(-50.0 * dz))
    xy_score = torch.where(dx_dy <= 0.15, torch.exp(-20.0 * dx_dy), torch.zeros_like(dx_dy))
    bullseye_multiplier = 1.0 + torch.where(dx_dy <= 0.04, 4.0 * torch.exp(-150.0 * dx_dy), torch.zeros_like(dx_dy))
    is_close = z_score * xy_score * bullseye_multiplier

    finger_dist = torch.norm(lfinger_pos - rfinger_pos, dim=-1, p=2)
    is_claw_closed = (finger_dist < 0.06).float()
    claw_multiplier = 1.0 + (is_claw_closed * 9.0)

    open_easy = (drawer_pos > 0.01) * 0.5 * is_close * claw_multiplier
    open_medium = (drawer_pos > 0.2) * is_close * claw_multiplier
    open_hard = (drawer_pos > 0.3) * is_close * claw_multiplier

    return open_easy + open_medium + open_hard


def hook_grip_pull_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    gripper_cfg: SceneEntityCfg,
    hook_aperture: float = 0.025,
    aperture_sigma: float = 0.01,
    contact_radius: float = 0.05,
) -> torch.Tensor:
    """Maximum reward: hook grip + opposite-side straddle + positive drawer velocity.

    Three conditions must all be satisfied simultaneously:
      1. Hook aperture: each finger is at ~hook_aperture from center (Gaussian bell peak).
         Fingers a bit open, not fully closed, perfectly wrapping the handle bar.
      2. Opposite-side straddle: link7 and link8 on opposite sides of the handle bar Y-axis.
      3. Positive drawer velocity: actively pulling, not just sitting there.

    This is the highest-priority reward, acting as the pyramid capstone.
    """
    # ── 1. Hook aperture gate ─────────────────────────────────────────────────
    # joint7 target is -hook_aperture, joint8 is +hook_aperture.
    # Reward peaks when joints are AT the hook position; falls off as a Gaussian.
    gripper_pos = env.scene[gripper_cfg.name].data.joint_pos[:, gripper_cfg.joint_ids]  # (N, 2)
    j7_pos = gripper_pos[:, 0]  # joint7 (negative side)
    j8_pos = gripper_pos[:, 1]  # joint8 (positive side)

    j7_error = (j7_pos - (-hook_aperture)) ** 2
    j8_error = (j8_pos - hook_aperture) ** 2
    aperture_score = torch.exp(-(j7_error + j8_error) / (2 * aperture_sigma ** 2))  # (N,)

    # ── 2. Opposite-side straddle gate ────────────────────────────────────────
    # Reuse dual_claw_straddle for proximity + opposite-side check
    straddle_score = dual_claw_straddle(env, contact_radius=contact_radius)  # (N,)

    # ── 3. Positive drawer velocity ───────────────────────────────────────────
    drawer_vel = env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids[0]]
    pulling_vel = torch.clamp(drawer_vel, min=0.0)

    return aperture_score * straddle_score * pulling_vel


def drawer_vel_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Reward positive drawer joint velocity (i.e. actually pulling the drawer open).

    This fires even before the drawer position changes, teaching the agent to apply
    sustained outward force once it is straddling the handle.
    Scaled by the dual_claw_straddle score so it only activates when grip is correct.
    """
    drawer_vel = env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids[0]]
    # Only reward POSITIVE velocity (pulling outward), ignore pushing back
    pulling_vel = torch.clamp(drawer_vel, min=0.0)
    straddle_score = dual_claw_straddle(env, contact_radius=0.05)
    return pulling_vel * straddle_score


def conditional_action_rate_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Action rate penalty that scales down as the drawer opens."""
    action_rate = torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1)
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    multiplier = torch.clamp(1.0 - (drawer_pos / 0.35), min=0.1, max=1.0)
    return action_rate * multiplier


def conditional_joint_vel_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, robot_cfg: SceneEntityCfg) -> torch.Tensor:
    """Joint velocity penalty that scales down as the drawer opens."""
    joint_vel = torch.sum(torch.square(env.scene[robot_cfg.name].data.joint_vel), dim=1)
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    multiplier = torch.clamp(1.0 - (drawer_pos / 0.35), min=0.1, max=1.0)
    return joint_vel * multiplier


def print_stage_curriculum(env: ManagerBasedRLEnv, env_ids: torch.Tensor, term_name: str, weight: float, num_steps: int) -> float:
    """A wrapper for modify_reward_weight that prints a message exactly when the stage transitions."""
    from isaaclab.envs.mdp import modify_reward_weight

    if env.common_step_counter >= num_steps and env.common_step_counter < num_steps + env.num_envs * 2:
        if len(env_ids) > 0 and env_ids[0] == 0:
            print(f"\n=======================================================")
            print(f"CURRICULUM UNLOCKED: STAGE 2")
            print(f"=======================================================")
            print(f"The AI has mastered reaching! Now forcing it to pull...")
            print(f"Updating reward term '{term_name}' to {weight}")
            print(f"=======================================================\n")

    return modify_reward_weight(env, env_ids, term_name, weight, num_steps)


def debug_link_distances(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Weight-0 term: redundant debug view (link distances tracked in single_claw_proximity).

    Returns zeros — zero effect on training.
    """
    return torch.zeros(env.num_envs, device=env.device)
