from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import matrix_from_quat

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def _robot_ee_pose(env: ManagerBasedRLEnv):
    """(ee_tcp_pos, ee_tcp_quat, lfinger_pos, rfinger_pos) from robot articulation bodies. Uses body name regex like reach."""
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
    """Reward for closing the fingers when being close to the handle.

    The :attr:`threshold` is the distance from the handle at which the fingers should be closed.
    The :attr:`open_joint_pos` is the joint position when the fingers are open.

    Note:
        It is assumed that zero joint position corresponds to the fingers being closed.
    """
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

    # One print per PPO iteration. num_steps_per_env=96, so each iteration advances
    # common_step_counter by num_envs * 96.
    log_interval = env.num_envs * 96
    if env.common_step_counter - _last_claw_print_step >= log_interval:
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
    """Reward both claws being close AND on OPPOSITE sides of the handle bar.

    The handle bar runs along its local Y-axis. We project each finger's offset
    from the handle centre onto that axis to get a signed side value:
      - link7 should be on the +Y side  (one inner face against the bar)
      - link8 should be on the -Y side  (other inner face against the bar)
    Both proximity AND opposite-side conditions must be satisfied simultaneously.
    """
    result = _claw_distances(env)
    if result is None:
        return torch.zeros(env.num_envs, device=env.device)

    d_link7, d_link8, lfinger_pos, rfinger_pos, handle_pos = result

    # Proximity gates — both must be within contact_radius
    k = 3.0 / contact_radius
    prox_link7 = torch.exp(-k * d_link7)
    prox_link8 = torch.exp(-k * d_link8)
    dual_proximity = prox_link7 * prox_link8  # AND gate

    # Handle local Y-axis in world frame (bar direction)
    handle_quat = env.scene["cabinet_frame"].data.target_quat_w[..., 0, :]
    handle_mat = matrix_from_quat(handle_quat)
    handle_y = handle_mat[..., 1]  # (N, 3)

    # Signed projection of each finger onto the handle bar axis
    vec_to_link7 = lfinger_pos - handle_pos  # (N, 3)
    vec_to_link8 = rfinger_pos - handle_pos  # (N, 3)

    side_link7 = (vec_to_link7 * handle_y).sum(dim=-1)  # (N,)
    side_link8 = (vec_to_link8 * handle_y).sum(dim=-1)  # (N,)

    # Opposite sides: link7 > 0 and link8 < 0 (or vice versa)
    # Use a soft gate: tanh of the product being negative means opposite sides
    # product < 0  →  opposite sides  →  gate near 1.0
    # product > 0  →  same side       →  gate near 0.0
    opposite_side_score = torch.sigmoid(-20.0 * side_link7 * side_link8)

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
    
    # ASYMMETRIC GRADIENT: The top lip of the drawer is physically higher than the handle.
    # If the robot's hand goes higher than target_z + 2cm, it gets EXACTLY 0 points!
    # If it is below or at the handle, it gets the continuous breadcrumb trail.
    is_too_high = ee_tcp_pos[..., 2] > (target_z + 0.02)
    z_score = torch.where(is_too_high, torch.zeros_like(dz), torch.exp(-50.0 * dz))
    
    # Wide breadcrumb trail to get the hand generally near the handle
    xy_score = torch.where(dx_dy <= 0.15, torch.exp(-20.0 * dx_dy), torch.zeros_like(dx_dy))
    
    # BULLSEYE MULTIPLIER: Gives up to 5x MORE points for pulling from the exact middle!
    # Creates an ultra-sharp gravitational well directly in the center of the handle.
    bullseye_multiplier = 1.0 + torch.where(dx_dy <= 0.04, 4.0 * torch.exp(-150.0 * dx_dy), torch.zeros_like(dx_dy))
    
    is_close = z_score * xy_score * bullseye_multiplier
    
    # Check if the claw is actually closed!
    # Fully open is ~10cm apart. Fully closed is ~3cm apart. 
    finger_dist = torch.norm(lfinger_pos - rfinger_pos, dim=-1, p=2)
    is_claw_closed = (finger_dist < 0.06).float()
    
    # 1x points for pulling with handle, 10x AS MANY points if claw is closed!
    claw_multiplier = 1.0 + (is_claw_closed * 9.0)

    # EXACTLY 0 points if it is pulling from the top of the shelf!
    return is_close * drawer_pos * claw_multiplier


def straddle_handle(env: ManagerBasedRLEnv, threshold: float) -> torch.Tensor:
    """Reward the robot for threading its fingers on strictly opposite sides of the handle.
    This mathematically guarantees the handle is trapped INSIDE the grip, rather than pinched from outside.
    """
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
        
    ee_tcp_pos, _, lfinger_pos, rfinger_pos = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    
    # Only reward straddling if the hand is physically near the handle
    distance_to_handle = torch.norm(handle_pos - ee_tcp_pos, dim=-1, p=2)
    is_close = (distance_to_handle <= threshold).float()
    
    # Calculate Euclidean distances
    dist_thumb_to_handle = torch.norm(handle_pos - rfinger_pos, dim=-1, p=2)
    dist_index_to_handle = torch.norm(handle_pos - lfinger_pos, dim=-1, p=2)
    dist_thumb_to_index = torch.norm(lfinger_pos - rfinger_pos, dim=-1, p=2)
    
    # If the handle is PERFECTLY between the two fingers, then:
    # dist(thumb, handle) + dist(index, handle) == dist(thumb, index)
    # We penalize any deviation from this perfect straight line!
    linearity_deviation = (dist_thumb_to_handle + dist_index_to_handle) - dist_thumb_to_index
    
    # Convert deviation to a 0-to-1 score (1.0 means perfectly straddled)
    # We use a sharp exponential curve so it only gets points if it's highly accurate
    straddle_score = torch.exp(-50.0 * linearity_deviation)
    
    return is_close * straddle_score


def multi_stage_open_drawer(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Multi-stage bonus for opening the drawer.

    Depending on the drawer's position, the reward is given in three stages: easy, medium, and hard.
    This helps the agent to learn to open the drawer in a controlled manner.
    """
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    
    pose = _robot_ee_pose(env)
    if pose is None:
        return torch.zeros(env.num_envs, device=env.device)
        
    ee_tcp_pos, _, lfinger_pos, rfinger_pos = pose
    handle_pos = env.scene["cabinet_frame"].data.target_pos_w[..., 0, :]
    
    # Use the exact Z-coordinate of the offset point!
    target_z = handle_pos[..., 2]
    dz = torch.abs(target_z - ee_tcp_pos[..., 2])
    
    # X/Y distance
    dx_dy = torch.norm(handle_pos[..., :2] - ee_tcp_pos[..., :2], dim=-1, p=2)
    
    # ASYMMETRIC GRADIENT to ban the top lip
    is_too_high = ee_tcp_pos[..., 2] > (target_z + 0.02)
    z_score = torch.where(is_too_high, torch.zeros_like(dz), torch.exp(-50.0 * dz))
    
    # Wide breadcrumb trail
    xy_score = torch.where(dx_dy <= 0.15, torch.exp(-20.0 * dx_dy), torch.zeros_like(dx_dy))
    
    # BULLSEYE MULTIPLIER: 5x points for exact middle
    bullseye_multiplier = 1.0 + torch.where(dx_dy <= 0.04, 4.0 * torch.exp(-150.0 * dx_dy), torch.zeros_like(dx_dy))
    
    is_close = z_score * xy_score * bullseye_multiplier
    
    # Claw closed multiplier (10x)
    finger_dist = torch.norm(lfinger_pos - rfinger_pos, dim=-1, p=2)
    is_claw_closed = (finger_dist < 0.06).float()
    claw_multiplier = 1.0 + (is_claw_closed * 9.0)

    # ALL milestones require is_close. If it pulls the shelf, it gets exactly 0!
    open_easy = (drawer_pos > 0.01) * 0.5 * is_close * claw_multiplier
    open_medium = (drawer_pos > 0.2) * is_close * claw_multiplier
    open_hard = (drawer_pos > 0.3) * is_close * claw_multiplier

    return open_easy + open_medium + open_hard


def conditional_action_rate_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Action rate penalty that scales down as the drawer opens.
    
    If the drawer is completely closed, the penalty multiplier is 1.0.
    As the drawer opens, the penalty multiplier scales down to 0.1 (so it's allowed to be slightly jerky while pulling).
    """
    # Base action rate penalty (squared difference of actions)
    action_rate = torch.sum(torch.square(env.action_manager.action - env.action_manager.prev_action), dim=1)
    
    # Get drawer position
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    
    # Scale multiplier from 1.0 (at 0m) down to 0.1 (at 0.35m)
    multiplier = torch.clamp(1.0 - (drawer_pos / 0.35), min=0.1, max=1.0)
    
    return action_rate * multiplier


def conditional_joint_vel_l2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg, robot_cfg: SceneEntityCfg) -> torch.Tensor:
    """Joint velocity penalty that scales down as the drawer opens."""
    # Base joint velocity penalty
    joint_vel = torch.sum(torch.square(env.scene[robot_cfg.name].data.joint_vel), dim=1)
    
    # Get drawer position
    drawer_pos = env.scene[asset_cfg.name].data.joint_pos[:, asset_cfg.joint_ids[0]]
    
    # Scale multiplier from 1.0 (at 0m) down to 0.1 (at 0.35m)
    multiplier = torch.clamp(1.0 - (drawer_pos / 0.35), min=0.1, max=1.0)
    
    return joint_vel * multiplier


def print_stage_curriculum(env: ManagerBasedRLEnv, env_ids: torch.Tensor, term_name: str, weight: float, num_steps: int) -> float:
    """A wrapper for modify_reward_weight that prints a message exactly when the stage transitions."""
    from isaaclab.envs.mdp import modify_reward_weight
    
    # env.common_step_counter increments by the number of environments each step.
    # We print a message when the step counter crosses the threshold.
    if env.common_step_counter >= num_steps and env.common_step_counter < num_steps + env.num_envs * 2:
        # Only print once (for the 0th environment) to avoid spamming the console
        if len(env_ids) > 0 and env_ids[0] == 0:
            print(f"\n=======================================================")
            print(f"CURRICULUM UNLOCKED: STAGE 2")
            print(f"=======================================================")
            print(f"The AI has mastered reaching! Now forcing it to pull...")
            print(f"Updating reward term '{term_name}' to {weight}")
            print(f"=======================================================\n")
            
    return modify_reward_weight(env, env_ids, term_name, weight, num_steps)

