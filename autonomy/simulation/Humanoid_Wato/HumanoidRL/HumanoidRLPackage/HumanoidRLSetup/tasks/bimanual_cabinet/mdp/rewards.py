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


def _finger_contact_force(env: ManagerBasedRLEnv):
    """Return (f7, f8) net contact force VECTORS (N,3) on link7/link8 from the handle."""
    def _force(name: str) -> torch.Tensor:
        sensor = env.scene.sensors[name]
        fmat = sensor.data.force_matrix_w  # (N, bodies, filters, 3) or None
        if fmat is None:
            return torch.zeros(env.num_envs, 3, device=env.device)
        return fmat.reshape(env.num_envs, -1, 3).sum(dim=1)
    return _force("contact_link7"), _force("contact_link8")


def _inner_edge_contact(env: ManagerBasedRLEnv, force_threshold: float = 1.0, dir_margin: float = 0.2):
    """Return (inner7, inner8) booleans: is each finger's INNER edge touching the handle?

    Contact sensors alone cannot tell WHICH face touched, so we use the contact FORCE
    DIRECTION to be certain:
      - The inner faces of link7 and link8 face EACH OTHER.
      - When the handle presses a finger's INNER face, the reaction force on that finger
        points OUTWARD — away from the other finger (direction p_self - p_other).
      - When it presses the OUTER (back) face, the force points INWARD (toward the other
        finger) → REJECTED.
    A contact counts as inner ONLY when force magnitude > force_threshold AND the force
    direction projects onto the outward axis by more than dir_margin. This guarantees we
    never mistake an outer-edge press for a grip.
    """
    pose = _robot_ee_pose(env)
    if pose is None:
        z = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
        return z, z
    _, _, p7, p8 = pose
    f7, f8 = _finger_contact_force(env)

    mag7 = torch.norm(f7, dim=-1)
    mag8 = torch.norm(f8, dim=-1)

    # Outward axis for each finger (away from the other finger)
    out7 = p7 - p8
    out8 = p8 - p7
    out7 = out7 / (torch.norm(out7, dim=-1, keepdim=True) + 1e-6)
    out8 = out8 / (torch.norm(out8, dim=-1, keepdim=True) + 1e-6)

    f7u = f7 / (mag7.unsqueeze(-1) + 1e-6)
    f8u = f8 / (mag8.unsqueeze(-1) + 1e-6)

    # >0 means the contact force points outward → the INNER face is what touched
    proj7 = (f7u * out7).sum(dim=-1)
    proj8 = (f8u * out8).sum(dim=-1)

    inner7 = (mag7 > force_threshold) & (proj7 > dir_margin)
    inner8 = (mag8 > force_threshold) & (proj8 > dir_margin)
    return inner7, inner8


def good_grip_gate(env: ManagerBasedRLEnv, force_threshold: float = 1.0) -> torch.Tensor:
    """1.0 when BOTH inner edges contact the handle (a good grip), else 0.0.

    Over/under, left/right — ANY configuration counts as long as BOTH inner faces touch.
    Outer-edge contact never qualifies.
    """
    inner7, inner8 = _inner_edge_contact(env, force_threshold)
    return (inner7 & inner8).float()


def inner_edge_grip_reward(
    env: ManagerBasedRLEnv,
    force_threshold: float = 1.0,
    both_bonus: float = 4.0,
    center_sigma: float = 0.04,
) -> torch.Tensor:
    """Reward INNER-edge contact only, with a CENTER-GRIP multiplier.

    +1 per inner edge touching (outer-edge earns NOTHING).
    When BOTH inner edges touch, the both_bonus fires AND is scaled by a Gaussian
    centered on the handle's midpoint along its bar axis (local Y). Gripping at the
    exact center of the bar earns full center multiplier (×2); gripping at the end
    cap earns ×1 (no bonus). This steers the claw from the image — hooking on the
    end edge — to gripping mid-bar where the pull force is most stable.
    """
    inner7, inner8 = _inner_edge_contact(env, force_threshold)
    c7 = inner7.float()
    c8 = inner8.float()
    both = c7 * c8  # 1.0 when both inner edges touch

    # ── Center-grip multiplier ────────────────────────────────────────────────
    # Project the TCP midpoint onto the handle's local Y-axis (the bar length axis).
    # Y=0 in handle-local coords is the bar center; ±half_length are the end caps.
    # A Gaussian on this projection peaks at center (Y=0) and falls to ~0.5 at ±σ.
    pose = _robot_ee_pose(env)
    if pose is None:
        center_mul = torch.ones(env.num_envs, device=env.device)
    else:
        _, _, p7, p8 = pose
        tcp = (p7 + p8) * 0.5  # midpoint between the two fingers (N, 3)
        handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]  # (N, 3)
        handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
        handle_mat = matrix_from_quat(handle_quat)
        handle_y = handle_mat[..., 1]  # (N, 3) — bar long axis

        # Scalar projection of (tcp - handle_center) onto bar axis
        off_center = ((tcp - handle_pos) * handle_y).sum(dim=-1)  # (N,) signed offset

        # Gaussian: 1.0 at center, falls with sigma=center_sigma
        center_score = torch.exp(-(off_center ** 2) / (2.0 * center_sigma ** 2))
        # Multiplier: 1.0 (no bonus at end) → 2.0 (full bonus at center)
        center_mul = 1.0 + center_score  # range [1, 2]

    return c7 + c8 + both_bonus * both * center_mul


def first_pull_bonus(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    threshold: float = 0.01,
    force_threshold: float = 1.0,
) -> torch.Tensor:
    """FLAT bonus: constant the instant a GOOD GRIP (both inner edges) + pull begins.

    Returns a flat 1.0 (× weight) when both inner edges grip AND the drawer moved past
    `threshold`. Does not scale — the y-intercept of the pull reward line.
    """
    grip = good_grip_gate(env, force_threshold) > 0.5
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    opened = drawer_pos > threshold
    return (grip & opened).float()


def pull_distance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    max_open: float = 0.39,
    force_threshold: float = 1.0,
) -> torch.Tensor:
    """MAXIMUM points for pulling with a GOOD GRIP — superlinear in open fraction.

    Gated by good_grip_gate (BOTH inner edges touching). Returns grip * (f + 3 f^2),
    f = drawer_pos / max_open. Fully open with a good grip is worth ~120× a 1cm nudge.
    Only a genuine two-inner-edge grip earns any of this.
    """
    grip = good_grip_gate(env, force_threshold)
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    f = torch.clamp(drawer_pos / max_open, min=0.0, max=1.0)
    return grip * (f + 3.0 * f * f)


def continuous_pull_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    force_threshold: float = 1.0,
) -> torch.Tensor:
    """Reward SUSTAINED pulling in ONE swing: good grip × positive drawer velocity.

    A single continuous swing keeps drawer velocity high for many steps → large integral.
    Jerky stop-start pulls have low velocity with pauses → little reward. This makes one
    smooth pull far more valuable than many small tugs.
    """
    grip = good_grip_gate(env, force_threshold)
    drawer_vel = env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids[0]]
    pulling_vel = torch.clamp(drawer_vel, min=0.0)
    return grip * pulling_vel


def upright_pull_bonus(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    threshold: float = 0.01,
    force_threshold: float = 1.0,
) -> torch.Tensor:
    """Bonus for upright wrist orientation while holding a GOOD GRIP and pulling."""
    grip = good_grip_gate(env, force_threshold)
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

    return grip * opened * upright_score


# Debug print state for inner-edge contact monitoring
_last_grip_print_step: int = -1


def debug_inner_edge(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Weight-0 debug: prints INNER vs OUTER edge contact counts once per PPO iteration.

    Lets you confirm the inner-edge identification is correct — outer contacts are
    reported separately so you can be sure a grip is genuinely inner-edge.
    """
    global _last_grip_print_step
    inner7, inner8 = _inner_edge_contact(env)
    any7, any8 = _finger_handle_contact(env)  # ANY contact (inner OR outer)
    outer7 = any7 & (~inner7)
    outer8 = any8 & (~inner8)

    log_interval = 96
    if env.common_step_counter - _last_grip_print_step >= log_interval:
        print(
            f"[Grip] inner7={int(inner7.sum())} inner8={int(inner8.sum())} "
            f"BOTH_inner={int((inner7 & inner8).sum())} | "
            f"outer7={int(outer7.sum())} outer8={int(outer8.sum())} "
            f"(of {env.num_envs} envs)",
            flush=True,
        )
        _last_grip_print_step = env.common_step_counter
    return torch.zeros(env.num_envs, device=env.device)


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


# (debug_link_distances removed — replaced by debug_inner_edge, which prints inner vs
#  outer edge contact counts per iteration.)