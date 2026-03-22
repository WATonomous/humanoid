# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import logging
import os
import re
import tempfile

import numpy as np
import torch
import yaml
from dex_retargeting.retargeting_config import RetargetingConfig
from scipy.spatial.transform import Rotation as R


# import logger
logger = logging.getLogger(__name__)

# yourdfpy loads visual/collision meshes with the hand URDFs; these aren't needed for
# retargeting and clutter the logs, so we suppress them.
logging.getLogger("dex_retargeting.yourdfpy").setLevel(logging.ERROR)

# OpenXR hand joint indices (XrHandJointEXT 0-25) to select 21 joints for dex-retargeting.
# Selects: wrist(1), thumb(2-5), index/middle/ring/pinky proximal→tip (skips metacarpals 6,11,16,21).
_OPENXR_HAND_JOINT_INDICES = [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25]

# The transformation matrices to convert hand pose to canonical view.
_OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
    ]
)

_OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, 1],
        [1, 0, 0],
        [0, 1, 0],
    ]
)

# Wato arm_assembly hand joint names - 5 fingers (index/middle/ring/pinky MCP,PIP,DIP + thumb CMC,MCP,IP)
_WATO_HAND_JOINT_NAMES = [
    "mcp_index", "pip_index", "dip_index",
    "mcp_middle", "pip_middle", "dip_middle",
    "mcp_ring", "pip_ring", "dip_ring",
    "mcp_pinky", "pip_pinky", "dip_pinky",
    "cmc_thumb", "mcp_thumb", "ip_thumb",
]
_LEFT_HAND_JOINT_NAMES = _WATO_HAND_JOINT_NAMES
_RIGHT_HAND_JOINT_NAMES = _WATO_HAND_JOINT_NAMES

# dex_retargeting's RobotWrapper requires model.nq == model.nv. Pinocchio models
# continuous joints as nq=2, nv=1, so we pass a URDF with continuous -> revolute + limits.
_DEX_LIMIT_LINE = '    <limit effort="100" lower="-3.14159" upper="3.14159" velocity="100"/>'


def _urdf_continuous_to_revolute_for_dex(source_urdf_path: str) -> str:
    """Write a temp URDF with continuous joints replaced by revolute + limits so nq==nv."""
    with open(source_urdf_path) as f:
        content = f.read()
    content = re.sub(r'\btype="continuous"', 'type="revolute"', content)
    # Add limit for joints that had no limit (the two former continuous ones)
    for joint_name in ("shoulder_flexion_extension", "shoulder_rotation"):
        pattern = (
            r'(<joint name="' + re.escape(joint_name) + r'"[^>]*>.*?<axis xyz="[^"]+"/>)'
            r'\s*(</joint>)'
        )
        content = re.sub(pattern, r'\1\n' + _DEX_LIMIT_LINE + r'\n  \2', content, count=1, flags=re.DOTALL)
    fd, path = tempfile.mkstemp(suffix=".urdf", prefix="wato_dex_")
    try:
        with os.fdopen(fd, "w") as f:
            f.write(content)
    except Exception:
        os.unlink(path)
        raise
    return path


class WatoHandDexRetargeting:
    """Hand retargeting for Wato arm_assembly hand.

    Retargets OpenXR hand tracking to Wato arm_assembly hand joint angles (15 DOF per hand).
    """

    def __init__(
        self,
        hand_joint_names: list[str],
        right_hand_config_filename: str = "wato_hand_right_dexpilot.yml",
        left_hand_config_filename: str = "wato_hand_right_dexpilot.yml",
        left_hand_urdf_path: str | None = None,
        right_hand_urdf_path: str | None = None,
    ):
        """Initialize the hand retargeting.

        Args:
            hand_joint_names: Names of hand joints in the robot model
            right_hand_config_filename: Config file for right hand retargeting
            left_hand_config_filename: Config file for left hand retargeting
        """
        data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "data/"))
        config_dir = os.path.join(data_dir, "configs/dex-retargeting")
        # Local Wato arm_assembly.urdf path (relative to this package)
        default_urdf = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "Humanoid_Wato", "arm_assembly", "arm_assembly.urdf")
        )
        local_left_urdf_path = left_hand_urdf_path if left_hand_urdf_path else default_urdf
        local_right_urdf_path = right_hand_urdf_path if right_hand_urdf_path else default_urdf

        # dex_retargeting RobotWrapper requires nq==nv; Pinocchio gives nq!=nv for continuous joints.
        # Use a temp URDF with continuous -> revolute + limits.
        self._dex_left_urdf_path = _urdf_continuous_to_revolute_for_dex(local_left_urdf_path)
        self._dex_right_urdf_path = _urdf_continuous_to_revolute_for_dex(local_right_urdf_path)

        left_config_path = os.path.join(config_dir, left_hand_config_filename)
        right_config_path = os.path.join(config_dir, right_hand_config_filename)

        self._update_yaml_with_urdf_path(left_config_path, self._dex_left_urdf_path)
        self._update_yaml_with_urdf_path(right_config_path, self._dex_right_urdf_path)

        self._dex_left_hand = RetargetingConfig.load_from_file(left_config_path).build()
        self._dex_right_hand = RetargetingConfig.load_from_file(right_config_path).build()

        self.left_dof_names = self._dex_left_hand.optimizer.robot.dof_joint_names
        self.right_dof_names = self._dex_right_hand.optimizer.robot.dof_joint_names
        self.dof_names = self.left_dof_names + self.right_dof_names
        self.isaac_lab_hand_joint_names = hand_joint_names

        logger.info("[WatoDexRetargeter] init done.")

    def _update_yaml_with_urdf_path(self, yaml_path: str, urdf_path: str):
        """Update YAML file with the correct URDF path.

        Args:
            yaml_path: Path to the YAML configuration file
            urdf_path: Path to the URDF file to use
        """
        try:
            # Read the YAML file
            with open(yaml_path) as file:
                config = yaml.safe_load(file)

            # Update the URDF path in the configuration
            if "retargeting" in config:
                config["retargeting"]["urdf_path"] = urdf_path
                logger.info(f"Updated URDF path in {yaml_path} to {urdf_path}")
            else:
                logger.warning(f"Unable to find 'retargeting' section in {yaml_path}")

            # Write the updated configuration back to the file
            with open(yaml_path, "w") as file:
                yaml.dump(config, file)

        except Exception as e:
            logger.error(f"Error updating YAML file {yaml_path}: {e}")

    def convert_hand_joints(self, hand_poses: dict[str, np.ndarray], operator2mano: np.ndarray) -> np.ndarray:
        """Prepares the hand joints data for retargeting.

        Args:
            hand_poses: Dictionary containing hand pose data with joint positions and rotations
            operator2mano: Transformation matrix to convert from operator to MANO frame

        Returns:
            Joint positions with shape (21, 3)
        """
        joint_position = np.zeros((21, 3))
        hand_joints = list(hand_poses.values())
        for i, joint_index in enumerate(_OPENXR_HAND_JOINT_INDICES):
            joint = hand_joints[joint_index]
            joint_position[i] = joint[:3]

        # Convert hand pose to the canonical frame.
        joint_position = joint_position - joint_position[0:1, :]
        xr_wrist_quat = hand_poses.get("wrist")[3:]
        # OpenXR hand uses w,x,y,z order for quaternions but scipy uses x,y,z,w order
        wrist_rot = R.from_quat([xr_wrist_quat[1], xr_wrist_quat[2], xr_wrist_quat[3], xr_wrist_quat[0]]).as_matrix()

        return joint_position @ wrist_rot @ operator2mano

    def compute_ref_value(self, joint_position: np.ndarray, indices: np.ndarray, retargeting_type: str) -> np.ndarray:
        """Computes reference value for retargeting.

        Args:
            joint_position: Joint positions array
            indices: Target link indices
            retargeting_type: Type of retargeting ("POSITION" or other)

        Returns:
            Reference value in cartesian space
        """
        if retargeting_type == "POSITION":
            return joint_position[indices, :]
        else:
            origin_indices = indices[0, :]
            task_indices = indices[1, :]
            ref_value = joint_position[task_indices, :] - joint_position[origin_indices, :]
            return ref_value

    def compute_one_hand(
        self, hand_joints: dict[str, np.ndarray], retargeting: RetargetingConfig, operator2mano: np.ndarray
    ) -> np.ndarray:
        """Computes retargeted joint angles for one hand.

        Args:
            hand_joints: Dictionary containing hand joint data
            retargeting: Retargeting configuration object
            operator2mano: Transformation matrix from operator to MANO frame

        Returns:
            Retargeted joint angles
        """
        joint_pos = self.convert_hand_joints(hand_joints, operator2mano)
        print(f"DEBUG joint_pos index MCP (row 5): {joint_pos[5]}")  # ADD THIS
        print(f"DEBUG joint_pos middle MCP (row 9): {joint_pos[9]}")  # ADD THIS
        ref_value = self.compute_ref_value(
            joint_pos,
            indices=retargeting.optimizer.target_link_human_indices,
            retargeting_type=retargeting.optimizer.retargeting_type,
        )
        # Arm joints (non-hand) are fixed during hand retargeting; use robot neutral for them.
        robot = retargeting.optimizer.robot
        fixed_qpos = np.array(robot.q0[retargeting.optimizer.idx_pin2fixed], dtype=np.float64)
        # Enable gradient calculation and inference mode in case some other script has disabled it
        # This is necessary for the retargeting to work since it uses gradient features that
        # are not available in inference mode
        with torch.enable_grad():
            with torch.inference_mode(False):
                return retargeting.retarget(ref_value, fixed_qpos=fixed_qpos)

    def get_joint_names(self) -> list[str]:
        """Returns list of all joint names."""
        return self.dof_names

    def get_left_joint_names(self) -> list[str]:
        """Returns list of left hand joint names."""
        return self.left_dof_names

    def get_right_joint_names(self) -> list[str]:
        """Returns list of right hand joint names."""
        return self.right_dof_names

    def get_hand_indices(self, robot) -> np.ndarray:
        """Gets indices of hand joints in robot's DOF array.

        Args:
            robot: Robot object containing DOF information

        Returns:
            Array of joint indices
        """
        return np.array([robot.dof_names.index(name) for name in self.dof_names], dtype=np.int64)

    def compute_left(self, left_hand_poses: dict[str, np.ndarray]) -> np.ndarray:
        """Computes retargeted joints for left hand.

        Args:
            left_hand_poses: Dictionary of left hand joint poses

        Returns:
            Retargeted joint angles for left hand
        """
        if left_hand_poses is not None:
            left_hand_q = self.compute_one_hand(left_hand_poses, self._dex_left_hand, _OPERATOR2MANO_LEFT)
        else:
            left_hand_q = np.zeros(len(_LEFT_HAND_JOINT_NAMES))
        return left_hand_q

    def compute_right(self, right_hand_poses: dict[str, np.ndarray]) -> np.ndarray:
        """Computes retargeted joints for right hand.

        Args:
            right_hand_poses: Dictionary of right hand joint poses

        Returns:
            Retargeted joint angles for right hand
        """
        if right_hand_poses is not None:
            right_hand_q = self.compute_one_hand(right_hand_poses, self._dex_right_hand, _OPERATOR2MANO_RIGHT)
        else:
            right_hand_q = np.zeros(len(_RIGHT_HAND_JOINT_NAMES))
        return right_hand_q
