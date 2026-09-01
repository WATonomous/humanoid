# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import torch

from pxr import Gf, Sdf


import isaaclab.utils.math as math_utils

from isaaclab.assets import RigidObject
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.sensors import ContactSensor
from isaaclab.managers import SceneEntityCfg



def any_vial_grasped(
    env: ManagerBasedRLEnv,
    contact_sensor_cfg: SceneEntityCfg,
    vials: list[str],
    min_height: float = 0.01,
    warmup_steps: int = 30,
    force_threshold: float = 0.1,
) -> torch.Tensor:
    """Check if any vial is currently grasped by the gripper.
    
    A vial is considered grasped if:
    1. The gripper has contact with it (force above threshold)
    2. The vial is above the minimum height threshold (only checked for initial grasp)
    3. We are past the warmup period
    
    Once grasped, stays grasped as long as contact is maintained (height no longer matters).
    Released only when contact is lost.
    
    Args:
        env: The environment instance.
        contact_sensor_cfg: Configuration for the contact sensor.
        vials: List of vial asset names to check.
        min_height: Minimum Z height (meters) for vial to be considered lifted. Default 1cm.
        warmup_steps: Number of initial steps to ignore (warmup period). Default 30.
        force_threshold: Minimum contact force (Newtons) to detect contact.
    
    Returns:
        Boolean tensor of shape (num_envs, 1) indicating grasp status per environment.
    """
    num_envs = env.num_envs
    device = env.device
    
    # Initialize state trackers on first call
    if not hasattr(any_vial_grasped, "_prev_grasped"):
        any_vial_grasped._prev_grasped = torch.zeros(num_envs, dtype=torch.bool, device=device)
    if not hasattr(any_vial_grasped, "_is_holding"):
        any_vial_grasped._is_holding = torch.zeros(num_envs, dtype=torch.bool, device=device)
    
    # Check warmup: no vial can be grasped in the first N steps
    current_step = env.episode_length_buf
    in_warmup = current_step < warmup_steps
    
    # Reset holding state for environments that just reset (step 0 or 1)
    just_reset = current_step <= 1
    any_vial_grasped._is_holding[just_reset] = False
    any_vial_grasped._prev_grasped[just_reset] = False
    
    # Get contact sensor
    contact_sensor: ContactSensor = env.scene[contact_sensor_cfg.name]
    
    # Get contact forces - shape: (num_envs, num_bodies, num_filters, 3)
    contact_forces = contact_sensor.data.force_matrix_w
    
    # Calculate force magnitude per filter (each filter corresponds to a vial)
    # Shape: (num_envs, num_bodies, num_filters)
    contact_force_norm = torch.linalg.vector_norm(contact_forces, dim=-1)
    
    # Sum over bodies to get per-env, per-filter contact detection
    # Shape: (num_envs, num_filters)
    contact_per_filter = contact_force_norm.sum(dim=1)
    
    # Track contact and lift status across all vials
    any_contact = torch.zeros(num_envs, dtype=torch.bool, device=device)
    new_grasp = torch.zeros(num_envs, dtype=torch.bool, device=device)
    
    for vial_idx, vial_name in enumerate(vials):
        vial: RigidObject = env.scene[vial_name]
        vial_z = vial.data.root_pos_w[:, 2]  # Z position per environment
        
        has_contact_with_vial = contact_per_filter[:, vial_idx] > force_threshold
        vial_is_lifted = vial_z > min_height
        
        # Track if we have contact with any vial
        any_contact = any_contact | has_contact_with_vial
        
        # New grasp: contact + lifted + not already holding
        new_grasp = new_grasp | (has_contact_with_vial & vial_is_lifted & (~any_vial_grasped._is_holding))
    
    # Update holding state with hysteresis:
    # - Start holding: new grasp detected (contact + lifted)
    # - Keep holding: was holding AND still have contact
    # - Stop holding: was holding AND lost contact
    was_holding = any_vial_grasped._is_holding.clone()
    any_vial_grasped._is_holding = (was_holding & any_contact) | new_grasp
    
    # Apply warmup mask
    is_grasped = any_vial_grasped._is_holding & (~in_warmup)
    
    # Debug: print state transitions
    prev = any_vial_grasped._prev_grasped
    just_grasped = is_grasped & (~prev)
    just_released = (~is_grasped) & prev
    
    if just_grasped.any():
        env_ids = torch.where(just_grasped)[0].tolist()
        print(f"[GRASP] Vial grasped in env(s): {env_ids}")
    
    if just_released.any():
        env_ids = torch.where(just_released)[0].tolist()
        print(f"[RELEASE] Vial released in env(s): {env_ids}")
    
    # Update previous state
    any_vial_grasped._prev_grasped = is_grasped.clone()
    
    # Return as float tensor with shape (num_envs, 1) for observation
    return is_grasped.float().unsqueeze(-1)




def vial_placed_on_rack(
    env: ManagerBasedRLEnv,
    contact_sensor_cfg: SceneEntityCfg,
    vials: list[str],
    rack_name: str,
    warmup_steps: int = 30,
    grasp_history_window: int = 20,
    force_threshold: float = 2.0,
    # Rack local dimensions (from Vial_rack_simple.usda extent: 0→0.12 x, 0→0.12 y)
    rack_local_x_min: float = 0.0,
    rack_local_x_max: float = 0.12,
    rack_local_y_min: float = 0.0,
    rack_local_y_max: float = 0.12,
    # Slot entries are at local z=0.1; rack body top at z=0.073
    rack_local_z_max: float = 0.1,
    # Orientation: abs(vial_up_z) must exceed this to count as "vertical" (in rack)
    vertical_threshold: float = 0.7,
) -> torch.Tensor:
    """Check if a vial has been placed into the rack.

    A vial is considered placed in the rack if:
    1. We are past the warmup period
    2. The vial is approximately vertical — its local Z axis is roughly aligned
       with world Z (either up or down).  This distinguishes a vial sitting in a
       slot from one lying on its side on the rack surface.
    3. The vial's position in rack-local coordinates is within the rack's XY bounding box
    4. The vial's rack-local Z is below the slot entry level (rack_local_z_max)
    5. THIS SPECIFIC vial was grasped at some point in the last N steps
    6. THIS SPECIFIC vial is no longer grasped

    Args:
        env: The environment instance.
        contact_sensor_cfg: Configuration for the contact sensor.
        vials: List of vial asset names to check.
        rack_name: Name of the rack asset in the scene.
        warmup_steps: Number of initial steps to ignore. Default 30.
        grasp_history_window: Number of steps to track grasp history. Default 20.
        force_threshold: Minimum contact force (N) to detect grasp.
        rack_local_x_min: Rack local X minimum bound.
        rack_local_x_max: Rack local X maximum bound.
        rack_local_y_min: Rack local Y minimum bound.
        rack_local_y_max: Rack local Y maximum bound.
        rack_local_z_max: Maximum rack-local Z for vial center (slot entry level).
        vertical_threshold: The vial's up-vector projected onto world-Z must
            have abs value above this to be considered "vertical" (in the slot).
            Default 0.7 (~45° from vertical).

    Returns:
        Float tensor of shape (num_envs, 1) indicating placement status per environment.
    """
    num_envs = env.num_envs
    device = env.device
    num_vials = len(vials)

    # Initialize state trackers on first call
    if not hasattr(vial_placed_on_rack, "_grasp_history"):
        vial_placed_on_rack._grasp_history = torch.zeros(
            num_envs, num_vials, grasp_history_window, dtype=torch.bool, device=device
        )
        vial_placed_on_rack._history_idx = 0
        vial_placed_on_rack._prev_placed = torch.zeros(num_envs, dtype=torch.bool, device=device)
        vial_placed_on_rack._vial_placed_flags = torch.zeros(
            num_envs, num_vials, dtype=torch.bool, device=device
        )

    current_step = env.episode_length_buf
    in_warmup = current_step < warmup_steps

    just_reset = current_step <= 1
    if just_reset.any():
        vial_placed_on_rack._grasp_history[just_reset] = False
        vial_placed_on_rack._prev_placed[just_reset] = False
        vial_placed_on_rack._vial_placed_flags[just_reset] = False

    # Get contact sensor for grasp detection
    contact_sensor: ContactSensor = env.scene[contact_sensor_cfg.name]
    contact_forces = contact_sensor.data.force_matrix_w
    contact_force_norm = torch.linalg.vector_norm(contact_forces, dim=-1)
    contact_per_filter = contact_force_norm.sum(dim=1)  # (num_envs, num_filters)

    # Get rack pose in world frame
    rack_obj: RigidObject = env.scene[rack_name]
    rack_pos_w = rack_obj.data.root_pos_w       # (num_envs, 3)
    rack_quat_w = rack_obj.data.root_quat_w     # (num_envs, 4)
    rack_quat_inv = math_utils.quat_inv(rack_quat_w)

    any_vial_newly_placed = torch.zeros(num_envs, dtype=torch.bool, device=device)

    # Unit Z vector used for the vertical orientation check
    unit_z = torch.zeros(num_envs, 3, device=device)
    unit_z[:, 2] = 1.0

    for vial_idx, vial_name in enumerate(vials):
        vial: RigidObject = env.scene[vial_name]
        vial_pos_w = vial.data.root_pos_w       # (num_envs, 3)
        vial_quat_w = vial.data.root_quat_w     # (num_envs, 4)

        # --- Grasp detection ---
        vial_grasped_now = contact_per_filter[:, vial_idx] > force_threshold
        vial_placed_on_rack._grasp_history[:, vial_idx, vial_placed_on_rack._history_idx] = vial_grasped_now
        vial_was_grasped_recently = vial_placed_on_rack._grasp_history[:, vial_idx, :].any(dim=1)

        # --- Vertical orientation check ---
        # Transform vial's local Z axis into world frame.  A vial sitting in a
        # rack slot will be roughly vertical (abs(z) close to 1), while one lying
        # on its side will have abs(z) close to 0.
        vial_up_world = math_utils.quat_apply(vial_quat_w, unit_z)
        is_vertical = torch.abs(vial_up_world[:, 2]) > vertical_threshold

        # --- Position check in rack-local coordinates ---
        vial_pos_relative = vial_pos_w - rack_pos_w
        vial_pos_local = math_utils.quat_apply(rack_quat_inv, vial_pos_relative)

        vial_local_x = vial_pos_local[:, 0]
        vial_local_y = vial_pos_local[:, 1]
        vial_local_z = vial_pos_local[:, 2]

        x_in_bounds = (vial_local_x >= rack_local_x_min) & (vial_local_x <= rack_local_x_max)
        y_in_bounds = (vial_local_y >= rack_local_y_min) & (vial_local_y <= rack_local_y_max)
        z_below_top = vial_local_z < rack_local_z_max

        position_ok = x_in_bounds & y_in_bounds & z_below_top

        # --- Combine all conditions ---
        vial_is_placed = (
            is_vertical
            & position_ok
            & vial_was_grasped_recently
            & (~vial_grasped_now)
            & (~in_warmup)
            & (~vial_placed_on_rack._vial_placed_flags[:, vial_idx])
        )

        newly_placed = vial_is_placed
        if newly_placed.any():
            env_ids = torch.where(newly_placed)[0].tolist()
            print(f"[RACK] {vial_name} placed in rack in env(s): {env_ids}")
            vial_placed_on_rack._vial_placed_flags[:, vial_idx] = (
                vial_placed_on_rack._vial_placed_flags[:, vial_idx] | newly_placed
            )

        any_vial_newly_placed = any_vial_newly_placed | newly_placed

    # Advance history ring buffer index
    vial_placed_on_rack._history_idx = (vial_placed_on_rack._history_idx + 1) % grasp_history_window

    any_placed = vial_placed_on_rack._vial_placed_flags.any(dim=1) & (~in_warmup)

    prev = vial_placed_on_rack._prev_placed
    vial_placed_on_rack._prev_placed = any_placed.clone()

    return any_placed.float().unsqueeze(-1)


def vial_placed_on_rack_termination(
    env: ManagerBasedRLEnv,
    contact_sensor_cfg: SceneEntityCfg,
    vials: list[str],
    rack_name: str,
    warmup_steps: int = 30,
    grasp_history_window: int = 20,
    force_threshold: float = 2.0,
    rack_local_x_min: float = 0.0,
    rack_local_x_max: float = 0.12,
    rack_local_y_min: float = 0.0,
    rack_local_y_max: float = 0.12,
    rack_local_z_max: float = 0.1,
    vertical_threshold: float = 0.7,
    confirm_steps: int = 25,
) -> torch.Tensor:
    """Termination term for vial placed in rack.

    Calls the observation to detect the initial placement event, then
    re-evaluates live physics conditions (vertical, in-bounds, released)
    for ``confirm_steps`` consecutive steps before reporting termination.
    If any condition fails during confirmation the counter resets.

    Returns:
        Boolean tensor of shape (num_envs,) for termination.
    """
    num_envs = env.num_envs
    device = env.device

    result = vial_placed_on_rack(
        env=env,
        contact_sensor_cfg=contact_sensor_cfg,
        vials=vials,
        rack_name=rack_name,
        warmup_steps=warmup_steps,
        grasp_history_window=grasp_history_window,
        force_threshold=force_threshold,
        rack_local_x_min=rack_local_x_min,
        rack_local_x_max=rack_local_x_max,
        rack_local_y_min=rack_local_y_min,
        rack_local_y_max=rack_local_y_max,
        rack_local_z_max=rack_local_z_max,
        vertical_threshold=vertical_threshold,
    )
    trigger = result.squeeze(-1).bool()

    if not hasattr(env, "_rack_success_counter"):
        env._rack_success_counter = torch.zeros(num_envs, dtype=torch.long, device=device)
        env._rack_confirm_active = torch.zeros(num_envs, dtype=torch.bool, device=device)

    env._rack_success_counter[env.episode_length_buf <= 1] = 0
    env._rack_confirm_active[env.episode_length_buf <= 1] = False

    newly_triggered = trigger & (~env._rack_confirm_active)
    if newly_triggered.any():
        env._rack_confirm_active[newly_triggered] = True
        env._rack_success_counter[newly_triggered] = 0

    still_valid = torch.zeros(num_envs, dtype=torch.bool, device=device)
    if env._rack_confirm_active.any():
        contact_sensor: ContactSensor = env.scene[contact_sensor_cfg.name]
        contact_forces = contact_sensor.data.force_matrix_w
        contact_force_norm = torch.linalg.vector_norm(contact_forces, dim=-1)
        contact_per_filter = contact_force_norm.sum(dim=1)

        rack_obj: RigidObject = env.scene[rack_name]
        rack_pos_w = rack_obj.data.root_pos_w
        rack_quat_w = rack_obj.data.root_quat_w
        rack_quat_inv = math_utils.quat_inv(rack_quat_w)

        unit_z = torch.zeros(num_envs, 3, device=device)
        unit_z[:, 2] = 1.0

        for vial_idx, vial_name in enumerate(vials):
            if not hasattr(vial_placed_on_rack, "_vial_placed_flags"):
                break
            was_placed = vial_placed_on_rack._vial_placed_flags[:, vial_idx]
            if not was_placed.any():
                continue

            vial_obj: RigidObject = env.scene[vial_name]
            vial_pos_w = vial_obj.data.root_pos_w
            vial_quat_w = vial_obj.data.root_quat_w

            vial_grasped_now = contact_per_filter[:, vial_idx] > force_threshold
            vial_up_world = math_utils.quat_apply(vial_quat_w, unit_z)
            is_vertical = torch.abs(vial_up_world[:, 2]) > vertical_threshold

            vial_pos_local = math_utils.quat_apply(rack_quat_inv, vial_pos_w - rack_pos_w)
            x_ok = (vial_pos_local[:, 0] >= rack_local_x_min) & (vial_pos_local[:, 0] <= rack_local_x_max)
            y_ok = (vial_pos_local[:, 1] >= rack_local_y_min) & (vial_pos_local[:, 1] <= rack_local_y_max)
            z_ok = vial_pos_local[:, 2] < rack_local_z_max

            vial_ok = was_placed & is_vertical & x_ok & y_ok & z_ok & (~vial_grasped_now)
            still_valid = still_valid | vial_ok

    env._rack_success_counter = torch.where(
        env._rack_confirm_active & still_valid,
        env._rack_success_counter + 1,
        torch.zeros_like(env._rack_success_counter),
    )
    env._rack_confirm_active[env._rack_success_counter == 0] = False

    confirmed = env._rack_success_counter >= confirm_steps
    # if (env._rack_success_counter > 0).any():
    #     print(f"[RACK CONFIRM] counter={env._rack_success_counter.tolist()} / {confirm_steps}")
    # if confirmed.any():
    #     print(f"[RACK CONFIRM] success confirmed in env(s): {torch.where(confirmed)[0].tolist()}")

    return confirmed

