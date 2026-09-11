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
import os

from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.sensors import FrameTransformerCfg, OffsetCfg

# import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from humanoid_so101_vial_task import asset_paths as ap
from humanoid_so101_vial_task.assets.so101 import SO101_CFG
from humanoid_so101_vial_task.mdp import (
    JointPositionActionCfg,
    joint_pos,
    reset_joints_by_offset,
    randomize_robot_color,
    ee_frame_state,
    joint_pos_rel,
)

assets_path = ap.assets_path


@configclass
class LerobotSo101BaseSceneCfg(InteractiveSceneCfg):

    env_spacing = 4.0
    num_envs = 1

    # robot
    robot: ArticulationCfg = SO101_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    ee_frame: FrameTransformerCfg = FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        debug_vis=False,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/gripper", name="gripper"
            ),  # no offset for ik convert
        ],
    )


##
# MDP settings
##
@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_positions = JointPositionActionCfg(
        asset_name="robot",
        joint_names=["Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll", "Jaw"],
        scale=1,
        use_default_offset=False,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_obs = ObsTerm(func=joint_pos)

        joint_pos_rel = ObsTerm(func=joint_pos_rel)
        ee_frame_state = ObsTerm(
            func=ee_frame_state,
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "robot_cfg": SceneEntityCfg("robot"),
            },
        )

        def __post_init__(self) -> None:
            self.enable_corruption = True
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # Reset robot position
    reset_robot_position = EventTerm(
        func=reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(
                "robot",
                joint_names=[
                    "Rotation",
                    "Pitch",
                    "Elbow",
                    "Wrist_Pitch",
                    "Wrist_Roll",
                    "Jaw",
                ],
            ),
            "position_range": (0, 0),
            "velocity_range": (0, 0),
        },
    )

    reset_set_robot_visual_material = EventTerm(
        func=randomize_robot_color,
        mode="reset",
        params={
            "color_names": ["orange"],
        },
    )


##
# Environment configuration
##
@configclass
class SO101TeleopEnvCfg(ManagerBasedRLEnvCfg):
    # Scene settings
    scene: LerobotSo101BaseSceneCfg = LerobotSo101BaseSceneCfg()

    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()

    # MDP settings
    rewards = None  # No rewards for teleoperation
    terminations = None  # No terminations for teleoperation

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5

        self.scene.num_envs = 1  # Always 1 env for teleoperation
        # viewer settings
        self.viewer.eye = (-0.25, -0.4, 0.22)
        self.viewer.lookat = (0.15, 0.0, 0.12)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation

        self.sim.render.rendering_mode = "quality"
        self.sim.render.enable_translucency = False
