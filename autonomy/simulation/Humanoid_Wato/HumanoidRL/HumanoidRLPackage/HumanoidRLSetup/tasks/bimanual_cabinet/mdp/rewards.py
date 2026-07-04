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


def approach_angle_reward(env: ManagerBasedRLEnv, proximity_radius: float = 0.15) -> torch.Tensor:
    """Continuous gradient that rewards increasing the spread angle between the two fingers.

    Fires whenever fingers are within proximity_radius of the handle, even when they
    are still on the SAME side. This fills the gradient dead-zone of dual_claw_straddle
    (which gives ~0 gradient when cos_angle > 0).

    Reward = prox_gate * (1 - cos_angle) / 2
      - cos_angle = -1 (perfect opposite)  →  reward = 1.0
      - cos_angle =  0 (perpendicular)     →  reward = 0.5
      - cos_angle = +1 (same side)         →  reward = 0.0

    The gradient always pushes the policy to rotate the approach so fingers move
    toward opposite sides of the handle, regardless of current configuration.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)

    d_link7, d_link8, lfinger_pos, rfinger_pos, handle_pos = result

    # Soft proximity gate — fires when AVERAGE finger distance is within proximity_radius
    avg_dist = (d_link7 + d_link8) * 0.5
    prox_gate = torch.exp(-3.0 * avg_dist / proximity_radius)

    # Direction vectors from handle center to each finger
    vec7 = lfinger_pos - handle_pos
    vec8 = rfinger_pos - handle_pos

    # Project onto plane perpendicular to handle bar length (handle local Y-axis)
    # so hovering over top at opposite ends of the bar does not count as opposite sides
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
    handle_mat = matrix_from_quat(handle_quat)
    handle_y = handle_mat[..., 1]  # (N, 3)

    vec7_perp = vec7 - (vec7 * handle_y).sum(dim=-1, keepdim=True) * handle_y
    vec8_perp = vec8 - (vec8 * handle_y).sum(dim=-1, keepdim=True) * handle_y

    u7 = vec7_perp / (torch.norm(vec7_perp, dim=-1, keepdim=True) + 1e-6)
    u8 = vec8_perp / (torch.norm(vec8_perp, dim=-1, keepdim=True) + 1e-6)

    cos_angle = (u7 * u8).sum(dim=-1)  # (N,) in [-1, +1]

    # Map to [0, 1]: 1.0 = perfect opposite, 0.0 = same direction
    angle_reward = (1.0 - cos_angle) * 0.5

    return prox_gate * angle_reward


def dual_claw_straddle(env: ManagerBasedRLEnv, contact_radius: float = 0.04) -> torch.Tensor:
    """Reward both claws being close AND on OPPOSITE sides of the handle center.

    Uses a direction-agnostic dot-product check:
      - Compute the unit vector from the handle to each finger: u7, u8
      - If dot(u7, u8) < 0, the fingers are on opposite sides of the handle
        along WHATEVER axis they happen to be on (any line through the origin).
      - This is correct regardless of which direction the robot approaches from.

    Gate 1 (proximity): exp(-k * average finger distance), k = 3.0 / contact_radius.
    Gate 2 (opposite-side): dot(u7, u8) < 0  →  angle between them > 90°.
      Perfect opposite sides → cos = -1 → score = 1.0
      Perpendicular          → cos =  0 → score = 0.5
      Same side              → cos = +1 → score ≈ 0.0
    Veto: if BOTH fingers are above the bar top (top-drape), the score is zeroed.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)

    d_link7, d_link8, lfinger_pos, rfinger_pos, handle_pos = result

    # ── Gate 1: proximity — use LINEAR gate instead of product-of-exponentials ──
    # Product gate collapses near zero when both fingers are at 5cm (score ≈ 0.003).
    # Instead: reward = exp(-k * AVERAGE distance), so both fingers at 5cm gives
    # exp(-3.0 * 0.048 / 0.10) = exp(-1.44) = 0.24 — a usable gradient.
    avg_dist = (d_link7 + d_link8) * 0.5
    k = 3.0 / contact_radius
    dual_proximity = torch.exp(-k * avg_dist)

    # ── Gate 2: direction-agnostic opposite-side check in handle cross-section ──
    # Vectors from handle center to each finger
    vec7 = lfinger_pos - handle_pos  # (N, 3)
    vec8 = rfinger_pos - handle_pos  # (N, 3)

    # Project out handle bar length direction (handle local Y-axis)
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
    handle_mat = matrix_from_quat(handle_quat)
    handle_y = handle_mat[..., 1]  # (N, 3)

    vec7_perp = vec7 - (vec7 * handle_y).sum(dim=-1, keepdim=True) * handle_y
    vec8_perp = vec8 - (vec8 * handle_y).sum(dim=-1, keepdim=True) * handle_y

    # Normalize (add small epsilon to avoid division by zero)
    u7 = vec7_perp / (torch.norm(vec7_perp, dim=-1, keepdim=True) + 1e-6)  # (N, 3)
    u8 = vec8_perp / (torch.norm(vec8_perp, dim=-1, keepdim=True) + 1e-6)  # (N, 3)

    # Cosine of the angle between the two finger directions in cross-section plane
    cos_angle = (u7 * u8).sum(dim=-1)  # (N,) — range [-1, +1]

    # Sigmoid gate: scores near 1.0 when cos < 0 (opposite sides), 0.0 when same side.
    # Sharpness factor 8 gives a clear threshold around cos = 0.
    opposite_side_score = torch.sigmoid(-8.0 * cos_angle)

    # ── Top-drape veto ───────────────────────────────────────────────────────
    # If BOTH fingers are above the bar, it's a top-drape — zero the straddle score.
    # One finger above + one below is a legitimate vertical straddle and is allowed.
    both_above = (lfinger_pos[:, 2] > handle_pos[:, 2] + 0.01) & \
                 (rfinger_pos[:, 2] > handle_pos[:, 2] + 0.01)
    anti_cheat = (~both_above).float()

    return dual_proximity * opposite_side_score * anti_cheat


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

    # Open claw hook multiplier (10x for fingers < 20cm apart)
    finger_dist = torch.norm(lfinger_pos - rfinger_pos, dim=-1, p=2)
    is_claw_hooked = (finger_dist < 0.20).float()
    claw_multiplier = 1.0 + (is_claw_hooked * 9.0)

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
    is_claw_hooked = (finger_dist < 0.20).float()
    claw_multiplier = 1.0 + (is_claw_hooked * 9.0)

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


def _finger_handle_contact(env: ManagerBasedRLEnv, force_threshold: float = 1.0):
    """Return (contact7, contact8) boolean tensors: is each finger touching the handle?

    Reads the ContactSensor force_matrix_w for link7 and link8 (filtered to the handle
    body only), so contact with anything OTHER than the handle does not count.
    A finger is 'in contact' when the contact force magnitude with the handle exceeds
    force_threshold newtons.
    """
    def _contact_mag(sensor_name: str) -> torch.Tensor:
        sensor = env.scene.sensors[sensor_name]
        fmat = sensor.data.force_matrix_w  # (N, num_bodies, num_filters, 3) or None
        if fmat is None:
            return torch.zeros(env.num_envs, device=env.device)
        # Sum force magnitude across all bodies/filters for this sensor
        return torch.norm(fmat, dim=-1).reshape(env.num_envs, -1).sum(dim=-1)

    mag7 = _contact_mag("contact_link7")
    mag8 = _contact_mag("contact_link8")
    return mag7 > force_threshold, mag8 > force_threshold


def contact_pull_gate(env: ManagerBasedRLEnv, force_threshold: float = 1.0) -> torch.Tensor:
    """Gate = 1.0 when EITHER link7 or link8 is physically touching the handle.

    This is the single source of truth for "the claw is gripping the handle." It uses
    real contact-sensor forces, so a finger merely near the handle (but not touching)
    scores 0. Contact with any surface other than the handle also scores 0.
    """
    contact7, contact8 = _finger_handle_contact(env, force_threshold)
    return (contact7 | contact8).float()


def claw_contact_reward(env: ManagerBasedRLEnv, force_threshold: float = 1.0, both_bonus: float = 2.0) -> torch.Tensor:
    """Reward the inner claws TOUCHING the handle (pure contact, no pulling required).

    Uses the finger contact sensors directly:
      +1.0 for each finger (link7 / link8) in contact with the handle, PLUS
      +both_bonus extra when BOTH fingers touch at the same time.

    So the score is:
      - 0.0  : no finger touching
      - 1.0  : one finger touching
      - 2.0 + both_bonus : both fingers touching (e.g. 4.0 with both_bonus=2.0)

    This is a prerequisite-shaping reward: it pays the robot to establish and hold
    contact, which is required before any pull reward can fire.
    """
    contact7, contact8 = _finger_handle_contact(env, force_threshold)
    c7 = contact7.float()
    c8 = contact8.float()
    return c7 + c8 + (c7 * c8 * both_bonus)


def drawer_vel_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Reward positive drawer velocity ONLY while a finger is TOUCHING the handle.

    Fires before the drawer position changes, teaching sustained outward force.
    Gated by real contact — no touch, no reward.
    """
    drawer_vel = env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids[0]]
    pulling_vel = torch.clamp(drawer_vel, min=0.0)
    return pulling_vel * contact_pull_gate(env)


def first_pull_bonus(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    threshold: float = 0.01,
) -> torch.Tensor:
    """FLAT intercept bonus: constant value the instant a finger TOUCHES + pull begins.

    Returns a FLAT 1.0 (scaled by weight) whenever BOTH are true, else 0.0:
      1. A finger (link7 or link8) is in contact with the handle
      2. Drawer pulled past `threshold` (1cm)

    Does NOT scale — it is the y-intercept step of the pull reward line. The 'how much
    you pulled' growth is handled by pull_distance_reward.
    """
    contact = contact_pull_gate(env) > 0.5
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    opened = drawer_pos > threshold
    return (contact & opened).float()


def upright_pull_bonus(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, threshold: float = 0.01) -> torch.Tensor:
    """Bonus for UPRIGHT wrist orientation WHILE touching the handle and pulling.

    Fires only when: a finger is in contact AND the drawer has moved past threshold.
    Then it scales with how upright the end-effector is (fingers arranged vertically
    rather than a flat horizontal hand). Rewards a clean, natural pulling posture.
    """
    contact = contact_pull_gate(env)
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    opened = (drawer_pos > threshold).float()

    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
    _, ee_tcp_quat, _, _ = pose
    ee_rot = matrix_from_quat(ee_tcp_quat)
    ee_z = ee_rot[..., 2]
    world_z = torch.zeros_like(ee_z)
    world_z[:, 2] = 1.0
    upright_score = (ee_z * world_z).sum(dim=-1) ** 2  # peak 1.0 when vertical

    return contact * opened * upright_score


def pull_distance_reward(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, contact_radius: float = 0.05) -> torch.Tensor:
    """SLOPE reward: grows with HOW MUCH the drawer is pulled, gated by real contact.

    This is the SLOPE of the pull reward line. Returns contact_gate × drawer_pos,
    so reward increases linearly the further the drawer opens. Combined with the flat
    first_pull_bonus intercept, total pull reward = intercept + slope × distance.

    Gated by real contact — no touch, no reward.
    """
    grip_score = contact_pull_gate(env)
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    return grip_score * drawer_pos


def open_claw_approach_reward(
    env: ManagerBasedRLEnv,
    gripper_cfg: SceneEntityCfg,
    open_target: float = 0.05,
    aperture_sigma: float = 0.02,
    proximity_radius: float = 0.12,
) -> torch.Tensor:
    """Reward keeping the claw OPEN while approaching the handle.

    Replaces reliance on hard joint limits to keep the gripper open. Rewards the
    gripper joints sitting near their open targets (joint7 ≈ -open_target,
    joint8 ≈ +open_target), gated by proximity to the handle so it only matters
    when the robot is actually going in for the grasp.

    A wide-open claw lets the handle bar pass BETWEEN the fingers instead of the
    robot draping a closed/pinched hand over the top.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)
    d7, d8, _, _, _ = result

    # Proximity gate — fires as the average fingertip nears the handle
    avg_dist = (d7 + d8) * 0.5
    prox_gate = torch.exp(-3.0 * avg_dist / proximity_radius)

    # Aperture score — Gaussian bell peaking when joints are at the open targets
    gripper_pos = env.scene[gripper_cfg.name].data.joint_pos[:, gripper_cfg.joint_ids]  # (N, 2)
    j7, j8 = gripper_pos[:, 0], gripper_pos[:, 1]
    err = (j7 - (-open_target)) ** 2 + (j8 - open_target) ** 2
    open_score = torch.exp(-err / (2.0 * aperture_sigma ** 2))

    return prox_gate * open_score


def finger_in_gap_reward(
    env: ManagerBasedRLEnv,
    gap_depth: float = 0.03,
    proximity_radius: float = 0.08,
    gap_sign: float = 1.0,
) -> torch.Tensor:
    """Reward threading a finger INTO the gap behind the handle bar (anti top-drape).

    The drawer opens along world -X (toward the robot), so the hole between the bar
    and the drawer face is on the +X side of the bar center. To hook and pull, a
    fingertip must get PAST the bar into that gap (world X greater than handle X).

    Reward = proximity_gate * clamp(deepest_finger_penetration / gap_depth, 0, 1)

    NOTE: if the geometry turns out mirrored (finger should go to -X), set
    gap_sign=-1.0 in the RewTerm params to flip the rewarded direction.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)
    d7, d8, lfinger_pos, rfinger_pos, handle_pos = result

    # Only reward penetration when the hand is actually near the handle
    avg_dist = (d7 + d8) * 0.5
    prox_gate = torch.exp(-3.0 * avg_dist / proximity_radius)

    # Signed penetration of each finger past the bar along the pull axis (world X)
    depth7 = gap_sign * (lfinger_pos[:, 0] - handle_pos[:, 0])
    depth8 = gap_sign * (rfinger_pos[:, 0] - handle_pos[:, 0])
    deepest = torch.maximum(depth7, depth8)

    # Saturating reward for positive penetration up to gap_depth
    depth_score = torch.clamp(torch.clamp(deepest, min=0.0) / gap_depth, max=1.0)

    return prox_gate * depth_score


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


def finger_behind_handle(
    env: ManagerBasedRLEnv,
    behind_offset: float = 0.03,
    below_offset: float = 0.0,
    sigma: float = 0.03,
    proximity_radius: float = 0.12,
    gap_sign: float = 1.0,
) -> torch.Tensor:
    """Reward the closest fingertip reaching a target point INSIDE the gap behind the bar.

    This is the key signal for HOOKING (as opposed to top-draping). It builds an explicit
    3D hook target:
        target = handle_center + gap_sign * behind_offset (along world +X, into the gap)
                                - below_offset            (down, below the bar's top edge)

    It then rewards a Gaussian bell on the closest fingertip's distance to that point,
    gated by overall hand proximity. Unlike distance-to-center rewards, a finger draped
    on TOP of the bar scores poorly here because the target is behind AND below the bar,
    so the only way to score high is to thread a fingertip into the actual gap.

    NOTE: mirror geometry by setting gap_sign=-1.0 if the gap is on the -X side.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)
    d7, d8, lfinger_pos, rfinger_pos, handle_pos = result

    # Build the hook target point: behind the bar (world +X) and below its top edge
    target = handle_pos.clone()
    target[:, 0] = target[:, 0] + gap_sign * behind_offset
    target[:, 2] = target[:, 2] - below_offset

    dist7 = torch.norm(lfinger_pos - target, dim=-1, p=2)
    dist8 = torch.norm(rfinger_pos - target, dim=-1, p=2)
    nearest = torch.minimum(dist7, dist8)

    # Overall hand proximity gate so this only matters once the hand is at the handle
    avg_dist = (d7 + d8) * 0.5
    prox_gate = torch.exp(-3.0 * avg_dist / proximity_radius)

    hook_score = torch.exp(-(nearest ** 2) / (2.0 * sigma ** 2))
    return prox_gate * hook_score


def descend_into_gap(
    env: ManagerBasedRLEnv,
    proximity_radius: float = 0.10,
    gap_sign: float = 1.0,
) -> torch.Tensor:
    """Reward positioning a finger BEHIND the bar and AT/BELOW its top (anti top-drape).

    A hooking finger should be behind the bar (+X) and at or below the bar's top edge so
    the fingertip is inside the gap, not draped over the top. This term rewards the
    finger that is furthest into the gap for being both behind and low. Combined with
    finger_behind_handle it shapes the full 'come in above the gap, then drop into it' motion.

    FIX: proximity gate changed from tight product-gate to average-distance gate so
    fingers at 5cm still receive a gradient. Also rewards approaching from the correct
    side (reducing behind_score falloff) so the gradient fires BEFORE the finger
    actually crosses the bar.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)
    d7, d8, lfinger_pos, rfinger_pos, handle_pos = result

    # Proximity gate on average distance — fires from ~15cm out
    avg_dist = (d7 + d8) * 0.5
    prox_gate = torch.exp(-3.0 * avg_dist / proximity_radius)

    # How far behind the bar each fingertip is (positive = on the gap side)
    behind7 = gap_sign * (lfinger_pos[:, 0] - handle_pos[:, 0])
    behind8 = gap_sign * (rfinger_pos[:, 0] - handle_pos[:, 0])

    # Pick the finger that is furthest into the gap
    use7 = behind7 >= behind8
    behind = torch.where(use7, behind7, behind8)
    finger_z = torch.where(use7, lfinger_pos[:, 2], rfinger_pos[:, 2])

    # Soft behind score — tanh so it starts giving gradient even before crossing 0
    # Saturates at 1.0 when 3cm behind, gives 0.46 at the bar center, ~0 when far in front
    behind_score = (torch.tanh(behind / 0.015) + 1.0) * 0.5

    # Reward being at or below the bar top (not draped above it)
    dz = finger_z - handle_pos[:, 2]  # >0 means above the bar
    below_score = torch.sigmoid(-40.0 * dz)  # ~1 when at/below, ~0 when above

    return prox_gate * behind_score * below_score


def hook_configuration_reward(
    env: ManagerBasedRLEnv,
    behind_depth: float = 0.02,
    front_depth: float = 0.02,
    sigma_behind: float = 0.025,
    sigma_front: float = 0.025,
    proximity_radius: float = 0.12,
    gap_sign: float = 1.0,
) -> torch.Tensor:
    """The core hook reward: one finger BEHIND the bar, the other IN FRONT.

    This replaces dual_approach_bonus and asymmetry_penalty for hook shaping.
    The correct hook configuration is inherently asymmetric:
      - hook finger: gap_sign * (finger_x - handle_x) > behind_depth  (in the gap)
      - front finger: gap_sign * (finger_x - handle_x) < -front_depth (in front of bar)

    Reward = behind_score * front_score
      - if behind_score = 0, reward = 0 (no gradient cheat via front only)
      - each score is a soft Gaussian so there's always a gradient

    The robot must discover that the two fingers serve different roles — this reward
    makes that asymmetry explicit rather than accidentally emergent.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)
    d7, d8, lfinger_pos, rfinger_pos, handle_pos = result

    # Signed X offset of each finger relative to bar center (+X = behind/in gap)
    x7 = gap_sign * (lfinger_pos[:, 0] - handle_pos[:, 0])  # link7
    x8 = gap_sign * (rfinger_pos[:, 0] - handle_pos[:, 0])  # link8

    # ── Behind score: reward whichever finger is furthest into the gap ──────────
    # Target: finger_x = +behind_depth (inside the gap)
    best_behind = torch.maximum(x7, x8)
    behind_err = (best_behind - behind_depth) ** 2
    behind_score = torch.exp(-behind_err / (2.0 * sigma_behind ** 2))
    # Hard zero if no finger has crossed into the gap at all (no cheat via front alone)
    behind_score = behind_score * (best_behind > 0).float()

    # ── Front score: reward whichever finger is furthest in front of the bar ────
    # Target: finger_x = -front_depth (in front of, facing the robot)
    front7 = -x7  # positive when finger is in front
    front8 = -x8
    best_front = torch.maximum(front7, front8)
    front_err = (best_front - front_depth) ** 2
    front_score = torch.exp(-front_err / (2.0 * sigma_front ** 2))

    # ── Proximity gate — only matters when both fingers are near the bar ─────────
    avg_dist = (d7 + d8) * 0.5
    prox_gate = torch.exp(-3.0 * avg_dist / proximity_radius)

    # ── Multiplicative: BOTH must be nonzero for full reward ────────────────────
    return prox_gate * behind_score * front_score


def upright_wrist_reward(
    env: ManagerBasedRLEnv,
    proximity_radius: float = 0.15,
) -> torch.Tensor:
    """Reward the end-effector being in an upright orientation near the handle.

    'Upright' here means the EE Z-axis (finger-spread axis) points roughly along
    world Z (vertical), so the fingers are arranged top-bottom rather than
    side-side or horizontal. This directly fights the 'horizontal flat hand'
    local optimum the robot falls into.

    Reward = prox_gate * dot(ee_z, world_z)^2  (squared so it's always positive,
    peak at 1.0 when perfectly vertical, 0.0 when horizontal)
    """
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
    ee_tcp_pos, ee_tcp_quat, _, _ = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]

    # Distance gate
    dist = torch.norm(handle_pos - ee_tcp_pos, dim=-1, p=2)
    prox_gate = torch.exp(-3.0 * dist / proximity_radius)

    # EE local Z-axis in world frame
    ee_rot = matrix_from_quat(ee_tcp_quat)          # (N, 3, 3)
    ee_z = ee_rot[..., 2]                            # (N, 3) — finger-spread axis

    # World up vector
    world_z = torch.zeros_like(ee_z)
    world_z[:, 2] = 1.0

    # Dot product: 1 when upright, 0 when horizontal, -1 when upside-down
    dot = (ee_z * world_z).sum(dim=-1)               # (N,)
    upright_score = dot ** 2                          # Squared: symmetric, peak at |dot|=1

    return prox_gate * upright_score


def dual_approach_bonus(env: ManagerBasedRLEnv, near_threshold: float = 0.06) -> torch.Tensor:
    """Stepping-stone bonus: both fingers simultaneously within near_threshold of the handle.

    Fires as a discrete bonus when BOTH claws cross the threshold at the same time.
    This bridges the gap between single_claw_proximity (OR logic, ~5cm) and
    dual_claw_straddle (requires both close + opposite sides, ~2cm).
    Without this, the agent has no gradient for the 5cm→2cm push with both fingers.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)
    d7, d8, _, _, _ = result

    # Soft AND: both must be within threshold — product of two Gaussian gates
    k = 3.0 / near_threshold
    gate7 = torch.exp(-k * d7)
    gate8 = torch.exp(-k * d8)
    return gate7 * gate8  # Only near 1.0 when BOTH fingers are close


def asymmetry_penalty(env: ManagerBasedRLEnv, contact_radius: float = 0.08, penalty_threshold: float = 0.05) -> torch.Tensor:
    """Penalty for asymmetric finger distances — discourages single-finger solutions.

    When one finger is much closer to the handle than the other, apply a penalty.
    This specifically targets the local optimum where link8 gets close (0.004m)
    while link7 stays far (0.115m).

    Penalty = prox_gate * max(0, |d7 - d8| - penalty_threshold)^2
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)

    d_link7, d_link8, _, _, _ = result

    # Only apply penalty when at least one finger is approaching
    min_distance = torch.minimum(d_link7, d_link8)
    prox_gate = torch.exp(-3.0 * min_distance / contact_radius)

    # Penalty increases quadratically with distance asymmetry
    distance_diff = torch.abs(d_link7 - d_link8)
    asymmetry_excess = torch.clamp(distance_diff - penalty_threshold, min=0.0)

    return prox_gate * (asymmetry_excess ** 2)


def debug_link_distances(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Weight-0 term: redundant debug view (link distances tracked in single_claw_proximity).

    Returns zeros — zero effect on training.
    """
    return torch.zeros(env.num_envs, device=env.device)