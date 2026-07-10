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
_last_f_print_step: int = -1
_max_f_this_iter: float = 0.0
_mag7_at_max_f: float = 0.0
_mag8_at_max_f: float = 0.0
_sum_f_this_iter: float = 0.0
_count_f_this_iter: int = 0


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


def _inner_edge_contact(
    env: ManagerBasedRLEnv,
    force_threshold: float = 1.0,
    dir_margin: float = 0.2,
    top_margin: float = 0.02,
):
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

    TOP-DRAPE GUARD: a finger draped over the TOP of the bar and pulled backward presses
    its inner face against the BACK of the bar, which produces an outward-pointing force
    and would otherwise pass the direction test — this is the "hook the top" cheat seen
    in training. We additionally require the contacting finger to sit at or below the
    bar top (finger_z <= handle_z + top_margin). A finger clearly above the bar top is
    draping on top, not hooking behind/under the bar, so its contact is REJECTED.
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

    # Top-drape guard: reject contacts from a finger sitting above the bar top.
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    below_top7 = p7[:, 2] <= handle_pos[:, 2] + top_margin
    below_top8 = p8[:, 2] <= handle_pos[:, 2] + top_margin

    inner7 = (mag7 > force_threshold) & (proj7 > dir_margin) & below_top7
    inner8 = (mag8 > force_threshold) & (proj8 > dir_margin) & below_top8
    return inner7, inner8


def good_grip_gate(env: ManagerBasedRLEnv, force_threshold: float = 1.0) -> torch.Tensor:
    """1.0 when EITHER inner edge contacts the handle (relaxed grip), else 0.0.

    RELAXED (H2): only ONE inner edge needs to touch the handle to count as a good
    enough grip to start pulling. Requiring BOTH inner edges simultaneously was too
    strict — the robot could reliably get one finger's inner face on the bar but not
    both at once, so the pull rewards never unlocked. With one inner edge the robot
    can hook and pull from there. Outer-edge contact still never qualifies (the
    force-direction test in _inner_edge_contact rejects it).
    """
    inner7, inner8 = _inner_edge_contact(env, force_threshold)
    return (inner7 | inner8).float()


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

    IMPORTANT: The weight on this reward must stay small (≤10). It is a GUIDING signal
    only. If it grows large, the robot will farm this reward by holding a perfect grip
    all episode without ever pulling. All real reward should come from the pull tier.
    """
    inner7, inner8 = _inner_edge_contact(env, force_threshold)
    c7 = inner7.float()
    c8 = inner8.float()
    both = c7 * c8  # 1.0 when both inner edges touch

    # ── Center-grip multiplier ────────────────────────────────────────────────
    pose = _robot_ee_pose(env)
    if pose is None:
        center_mul = torch.ones(env.num_envs, device=env.device)
    else:
        _, _, p7, p8 = pose
        tcp = (p7 + p8) * 0.5
        handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
        handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
        handle_mat = matrix_from_quat(handle_quat)
        handle_y = handle_mat[..., 1]
        off_center = ((tcp - handle_pos) * handle_y).sum(dim=-1)
        center_score = torch.exp(-(off_center ** 2) / (2.0 * center_sigma ** 2))
    # Center multiplier — LOGARITHMIC (diminishing returns after a certain point).
    # At the center (off_center=0): center_score=1.0 → log(1 + 2×1.0) = log(3) ≈ 1.1
    # At one sigma off: center_score=0.61 → log(1 + 2×0.61) = log(2.22) ≈ 0.8
    # At two sigmas off: center_score=0.14 → log(1 + 2×0.14) = log(1.28) ≈ 0.25
    # After the inflection, each extra improvement earns less — no need to obsess over
    # perfect center alignment.
    center_mul = torch.log(1.0 + 2.0 * center_score) / torch.log(torch.tensor(3.0, device=env.device))

    # Grip reward is CAPPED — it is a constant recognition of a good grip, not a
    # per-step income stream. Once established, the robot earns no more from holding.
    # Pull rewards are 100-1000× larger, so the only way to earn significantly more
    # is to actually pull the drawer open.
    single = torch.clamp(c7 + c8, max=2.0)       # max 2.0: both fingers near
    both_reward = both_bonus * both * center_mul   # max = 4 × 2 = 8.0 at center

    # Hard cap per step so this reward cannot exceed 10.0 regardless of configuration
    return torch.clamp(single + both_reward, max=10.0)


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


def inner_grip_strength(env: ManagerBasedRLEnv, force_threshold: float = 1.0, force_ceiling: float = 30.0) -> torch.Tensor:
    """Saturating grip multiplier in [0, 1] based on INNER-EDGE contact FORCE.

    GREATER EMPHASIS: the pull reward now scales with how firmly the inner edge grips —
    a light touch earns little, a firm inner-edge grip earns the full multiplier.
    CAPPED: saturates at 1.0 once the inner-edge contact force reaches `force_ceiling`,
    so beyond a solid grip the multiplier stops increasing (no runaway).
    Only forces from edges that pass the inner-edge force-direction test are counted,
    so pressing the top/outer face contributes nothing.
    """
    inner7, inner8 = _inner_edge_contact(env, force_threshold)
    f7, f8 = _finger_contact_force(env)
    mag7 = torch.norm(f7, dim=-1)
    mag8 = torch.norm(f8, dim=-1)
    inner_force = mag7 * inner7.float() + mag8 * inner8.float()
    return torch.clamp(inner_force / force_ceiling, max=1.0)


def pull_distance_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    max_open: float = 0.39,
    force_threshold: float = 1.0,
    dual_grip_bonus: float = 1.0,
) -> torch.Tensor:
    """Dense + disproportionately large pull reward using exp(5f) - 1.

    Shape: exp(5f) - 1 where f = drawer_pos / max_open
      - 0.0001cm (f=2.6e-6): 0.000013 — real gradient immediately, guides from first micron
      - 0.01cm   (f=0.00026): 0.0013  — continuous slope building early
      - 1cm      (f=0.026):   0.139   — noticeable reward for early pull
      - 10cm     (f=0.256):   2.54    — 18× the 1cm value
      - full open (f=1.0):    147.4   — ~1060× the 1cm value, ~100× any current reward

    The grip multiplier is log(1 + grip_score) — logarithmic diminishing returns so
    the robot only needs a good-enough grip, not a perfect one.
    """
    # Grip multiplier scales with inner-edge contact FORCE, saturating at force_ceiling.
    # Firmer inner-edge grip → more pull reward, capped once the grip is solid.
    grip_mul = inner_grip_strength(env, force_threshold, force_ceiling=30.0)

    # ── DUAL-GRIP MULTIPLIER ──────────────────────────────────────────────────
    # Pulling with BOTH inner edges gripping earns the MOST points. A single inner
    # edge still earns the full base pull reward (one-finger hook remains valid), but
    # when both inner edges are in contact the whole pull reward is multiplied by
    # (1 + dual_grip_bonus). With dual_grip_bonus=3.0 this gives 4× the reward for a
    # proper two-finger grip, making it the unambiguously best strategy.
    inner7, inner8 = _inner_edge_contact(env, force_threshold)
    both_inner = (inner7 & inner8).float()
    dual_mul = 1.0 + dual_grip_bonus * both_inner

    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]

    global _last_f_print_step, _max_f_this_iter, _mag7_at_max_f, _mag8_at_max_f, _sum_f_this_iter, _count_f_this_iter

    f = torch.clamp(drawer_pos / max_open, min=0.0, max=1.0)
    # Scale factor A=4840, weight=0.01 in config → product=48.4.
    # This gives: 0.00001m held 480 steps = Episode_Reward of exactly 1.0 point.
    # The exponential exp(5f)-1 then gives these Episode_Reward milestones:
    #   0.00001m (0.001cm): 1.0      pts  (by design — the reference point)
    #   0.0001m  (0.01cm):  10.0     pts
    #   0.001m   (0.1cm):   102      pts
    #   0.01m    (1cm):     1,100    pts
    #   0.05m    (5cm):     17,200   pts
    #   0.1m     (10cm):    116,000  pts
    #   0.2m     (20cm):    4,930,000 pts
    #   0.39m    (full):    1.19B    pts  → disproportionately massive as requested
    A = 4840.0
    base_score = A * (torch.exp(5.0 * f) - 1.0)

    # TARGETED GRADIENT BOOST: amplify the gradient in the 3cm → 20cm band.
    # The exp(5f) curve is nearly linear here — weak gradient. This concave ramp adds
    # extra incentive in exactly that range: steep early (3→10cm), flatter near 20cm.
    # At 3cm: +0, at 10cm: +~9680 (≈2A), at 20cm: +9680 (saturates to boost_scale×A)
    f_3cm  = 0.03 / max_open   # 3cm  ≈ f=0.077
    f_20cm = 0.20 / max_open   # 20cm ≈ f=0.513
    band = torch.clamp((f - f_3cm) / (f_20cm - f_3cm), min=0.0, max=1.0)
    boost_scale = 2.0
    band_boost = A * boost_scale * torch.sqrt(band)

    distance_score = base_score + band_boost

    # Track for debug print — use the actual grip gate (grip_mul > 0 means an inner
    # edge is in contact). Previously referenced the removed `grip_score` variable.
    gripped_f = f * (grip_mul > 0.0).float()
    best_gripped_idx = gripped_f.argmax().item()
    best_gripped_f = gripped_f[best_gripped_idx].item()
    if best_gripped_f > _max_f_this_iter:
        _max_f_this_iter = best_gripped_f
        f7, f8 = _finger_contact_force(env)
        _mag7_at_max_f = torch.norm(f7[best_gripped_idx]).item()
        _mag8_at_max_f = torch.norm(f8[best_gripped_idx]).item()

    _sum_f_this_iter += f.mean().item()
    _count_f_this_iter += 1

    log_interval = 96
    if env.common_step_counter > 0 and env.common_step_counter - _last_f_print_step >= log_interval:
        avg_f = _sum_f_this_iter / max(1, _count_f_this_iter)
        print(
            f"[Pull best] iter_end={env.common_step_counter} | "
            f"max f: {_max_f_this_iter:.4f} ({_max_f_this_iter * max_open * 100:.1f}cm) "
            f"[F7={_mag7_at_max_f:.1f}N F8={_mag8_at_max_f:.1f}N] | "
            f"avg f: {avg_f:.4f} ({avg_f * max_open * 100:.2f}cm)",
            flush=True,
        )
        _last_f_print_step = env.common_step_counter
        _max_f_this_iter = 0.0
        _mag7_at_max_f = 0.0
        _mag8_at_max_f = 0.0
        _sum_f_this_iter = 0.0
        _count_f_this_iter = 0

    return grip_mul * distance_score * dual_mul


def continuous_pull_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    force_threshold: float = 1.0,
    momentum_bonus: float = 3.0,
    velocity_threshold: float = 0.02,
) -> torch.Tensor:
    """Reward SUSTAINED pulling in ONE swing: good grip × drawer velocity × momentum multiplier.

    The momentum multiplier rewards keeping velocity above `velocity_threshold` without
    dropping. It accumulates per-env — envs that maintain continuous pulling get a
    progressively larger multiplier, while envs that pause and restart reset to 1.0.
    This makes one smooth long pull worth far more than many short tugs.

    multiplier per env = 1.0 + momentum_bonus × (sustained_steps / max_steps_per_iter)
    where sustained_steps increments each step velocity is above threshold while gripping.
    """
    grip = good_grip_gate(env, force_threshold)
    drawer_vel = env.scene[asset_cfg.name].data.joint_vel[:, asset_cfg.joint_ids[0]]
    pulling_vel = torch.clamp(drawer_vel, min=0.0)

    # Track how many consecutive steps each env has been pulling above threshold
    if not hasattr(env, '_sustained_pull_steps'):
        env._sustained_pull_steps = torch.zeros(env.num_envs, device=env.device)

    # Increment where gripping + pulling above threshold, reset where not
    active = (grip > 0.5) & (pulling_vel > velocity_threshold)
    env._sustained_pull_steps = torch.where(
        active,
        env._sustained_pull_steps + 1.0,
        torch.zeros_like(env._sustained_pull_steps)
    )

    # Momentum multiplier: grows with sustained pull, capped at 1 + momentum_bonus
    max_steps = 192.0  # one PPO iteration worth of steps (num_steps_per_env)
    momentum_mul = 1.0 + momentum_bonus * torch.clamp(env._sustained_pull_steps / max_steps, max=1.0)

    return grip * pulling_vel * momentum_mul


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