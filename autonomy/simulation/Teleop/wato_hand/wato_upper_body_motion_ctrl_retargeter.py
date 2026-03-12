# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as PoseUtils
from isaaclab.devices.device_base import DeviceBase
from isaaclab.devices.retargeter_base import RetargeterBase, RetargeterCfg
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg


class WatoUpperBodyMotionControllerRetargeter(RetargeterBase):
    """Simple retargeter that maps motion controller inputs to Wato arm_assembly hand joints (15 DOF per hand).

    Mapping:
    - Trigger (analog 0-1) → Index finger (mcp_index, pip_index, dip_index)
    - Squeeze (analog 0-1) → Middle (and ring/pinky) finger joints
    - Thumb (max(trigger,squeeze)) → Thumb (cmc_thumb, mcp_thumb, ip_thumb)
    """

    def __init__(self, cfg: WatoUpperBodyMotionControllerRetargeterCfg):
        """Initialize the retargeter."""
        super().__init__(cfg)
        self._sim_device = cfg.sim_device
        self._hand_joint_names = cfg.hand_joint_names
        self._enable_visualization = cfg.enable_visualization

        if cfg.hand_joint_names is None:
            raise ValueError("hand_joint_names must be provided")

        # Initialize visualization if enabled
        if self._enable_visualization:
            marker_cfg = VisualizationMarkersCfg(
                prim_path="/Visuals/g1_controller_markers",
                markers={
                    "joint": sim_utils.SphereCfg(
                        radius=0.01,
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
                    ),
                },
            )
            self._markers = VisualizationMarkers(marker_cfg)

    def retarget(self, data: dict) -> torch.Tensor:
        """Convert controller inputs to robot commands.

        Args:
            data: Dictionary with MotionControllerTrackingTarget.LEFT/RIGHT keys
                 Each value is a 2D array: [pose(7), inputs(7)]

        Returns:
            Tensor: [left_wrist(7), right_wrist(7), hand_joints(30)]
            hand_joints: left 15 (index,middle,ring,pinky,thumb) then right 15, matching arm_assembly order.
        """

        # Get controller data
        left_controller_data = data.get(DeviceBase.TrackingTarget.CONTROLLER_LEFT, np.array([]))
        right_controller_data = data.get(DeviceBase.TrackingTarget.CONTROLLER_RIGHT, np.array([]))

        default_wrist = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        left_wrist = self._extract_wrist_pose(left_controller_data, default_wrist)
        right_wrist = self._extract_wrist_pose(right_controller_data, default_wrist)

        left_hand_joints = self._map_to_hand_joints(left_controller_data, is_left=True)
        right_hand_joints = self._map_to_hand_joints(right_controller_data, is_left=False)
        left_hand_joints = -left_hand_joints

        # Order per hand (arm_assembly): mcp_index..dip_index, mcp_middle..dip_middle, mcp_ring..dip_ring, mcp_pinky..dip_pinky, cmc_thumb,mcp_thumb,ip_thumb
        all_hand_joints = np.concatenate([left_hand_joints, right_hand_joints])

        # Convert to tensors
        left_wrist_tensor = torch.tensor(
            self._retarget_abs(left_wrist, is_left=True), dtype=torch.float32, device=self._sim_device
        )
        right_wrist_tensor = torch.tensor(
            self._retarget_abs(right_wrist, is_left=False), dtype=torch.float32, device=self._sim_device
        )
        hand_joints_tensor = torch.tensor(all_hand_joints, dtype=torch.float32, device=self._sim_device)

        return torch.cat([left_wrist_tensor, right_wrist_tensor, hand_joints_tensor])

    def get_requirements(self) -> list[RetargeterBase.Requirement]:
        return [RetargeterBase.Requirement.MOTION_CONTROLLER]

    def _extract_wrist_pose(self, controller_data: np.ndarray, default_pose: np.ndarray) -> np.ndarray:
        """Extract wrist pose from controller data.

        Args:
            controller_data: 2D array [pose(7), inputs(7)]
            default_pose: Default pose to use if no data

        Returns:
            Wrist pose array [x, y, z, w, x, y, z]
        """
        if len(controller_data) > DeviceBase.MotionControllerDataRowIndex.POSE.value:
            return controller_data[DeviceBase.MotionControllerDataRowIndex.POSE.value]
        return default_pose

    def _map_to_hand_joints(self, controller_data: np.ndarray, is_left: bool) -> np.ndarray:
        """Map controller inputs to hand joint angles.

        Args:
            controller_data: 2D array [pose(7), inputs(7)]
            is_left: True for left hand, False for right hand

        Returns:
            Hand joint angles (15 joints per hand) in radians: index(3), middle(3), ring(3), pinky(3), thumb(3).
        """
        hand_joints = np.zeros(15)

        if len(controller_data) <= DeviceBase.MotionControllerDataRowIndex.INPUTS.value:
            return hand_joints

        # Extract inputs from second row
        inputs = controller_data[DeviceBase.MotionControllerDataRowIndex.INPUTS.value]

        if len(inputs) < len(DeviceBase.MotionControllerInputIndex):
            return hand_joints

        # Extract specific inputs using enum
        trigger = inputs[DeviceBase.MotionControllerInputIndex.TRIGGER.value]  # 0.0 to 1.0 (analog)
        squeeze = inputs[DeviceBase.MotionControllerInputIndex.SQUEEZE.value]  # 0.0 to 1.0 (analog)

        # Grasping logic:
        #   If trigger is pressed, we grasp with index and thumb.
        #   If squeeze is pressed, we grasp with middle and thumb.
        #   If both are pressed, we grasp with index, middle, and thumb.
        # The thumb rotates towards the direction of the pressing finger.
        #   If both are pressed, the thumb stays in the middle.

        thumb_button = max(trigger, squeeze)
        thumb_angle = -thumb_button
        thumb_rotation = 0.5 * trigger - 0.5 * squeeze
        if not is_left:
            thumb_rotation = -thumb_rotation

        index_angle = trigger * 1.0
        middle_angle = squeeze * 1.0
        ring_angle = squeeze * 1.0
        pinky_angle = squeeze * 1.0

        # arm_assembly order: index(3), middle(3), ring(3), pinky(3), thumb(3)
        hand_joints[0:3] = index_angle  # mcp_index, pip_index, dip_index
        hand_joints[3:6] = middle_angle  # mcp_middle, pip_middle, dip_middle
        hand_joints[6:9] = ring_angle  # mcp_ring, pip_ring, dip_ring
        hand_joints[9:12] = pinky_angle  # mcp_pinky, pip_pinky, dip_pinky
        hand_joints[12] = thumb_rotation  # cmc_thumb
        hand_joints[13] = thumb_angle * 0.4  # mcp_thumb
        hand_joints[14] = thumb_angle * 0.7  # ip_thumb
        return hand_joints

    def _retarget_abs(self, wrist: np.ndarray, is_left: bool) -> np.ndarray:
        """Handle absolute pose retargeting for controller wrists."""
        wrist_pos = torch.tensor(wrist[:3], dtype=torch.float32)
        wrist_quat = torch.tensor(wrist[3:], dtype=torch.float32)

        # Combined -75° (rather than -90° for wrist comfort) Y rotation + 90° Z rotation
        # This is equivalent to (0, -75, 90) in euler angles
        combined_quat = torch.tensor([0.5358, -0.4619, 0.5358, 0.4619], dtype=torch.float32)

        openxr_pose = PoseUtils.make_pose(wrist_pos, PoseUtils.matrix_from_quat(wrist_quat))
        transform_pose = PoseUtils.make_pose(torch.zeros(3), PoseUtils.matrix_from_quat(combined_quat))

        result_pose = PoseUtils.pose_in_A_to_pose_in_B(transform_pose, openxr_pose)
        pos, rot_mat = PoseUtils.unmake_pose(result_pose)
        quat = PoseUtils.quat_from_matrix(rot_mat)

        return np.concatenate([pos.numpy(), quat.numpy()])


@dataclass
class WatoUpperBodyMotionControllerRetargeterCfg(RetargeterCfg):
    """Configuration for the Wato arm_assembly motion controller retargeter."""

    enable_visualization: bool = False
    hand_joint_names: list[str] | None = None  # List of robot hand joint names
    retargeter_type: type[RetargeterBase] = WatoUpperBodyMotionControllerRetargeter
