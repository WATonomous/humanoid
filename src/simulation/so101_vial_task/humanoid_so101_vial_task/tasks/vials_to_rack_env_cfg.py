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
import numpy as np


import isaaclab.sim as sim_utils
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.assets import RigidObjectCfg, ArticulationCfg, AssetBaseCfg
from isaaclab.utils import configclass
from isaaclab.sensors import ContactSensorCfg
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm

from humanoid_so101_vial_task import asset_paths as ap
from humanoid_so101_vial_task.assets.so101 import S0101_CONTACT_GRASP_CFG
from humanoid_so101_vial_task.mdp import (
    reset_vials_rack,
    randomize_sky_light,
    ROBOT_COLORS,
    randomize_mat_rotation,
    randomize_robot_color,
    any_vial_grasped,
    vial_placed_on_rack,
    vial_placed_on_rack_termination,
    time_out,
)

from .so101_env_cfg import EventCfg
from .task_env_cfg import (
    SO101TaskSceneCfg,
    SO101TaskEnvCfg,
    TaskEventCfg,
    TaskObservationsCfg,
)

assets_path = ap.assets_path

manipulation_object_base = RigidObjectCfg(
    prim_path="{ENV_REGEX_NS}/ManipulationObject",
    spawn=sim_utils.UsdFileCfg(usd_path=""),
    init_state=RigidObjectCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.06),
    ),
)

vial = manipulation_object_base.replace()
vial.spawn.usd_path = f"{assets_path}/usd/Vial_opaque.usda"
vial.spawn.mass_props = sim_utils.MassPropertiesCfg(mass=0.02)
vial.spawn.rigid_props = sim_utils.RigidBodyPropertiesCfg(angular_damping=100.0)


rack = manipulation_object_base.replace()
rack.prim_path = "{ENV_REGEX_NS}/VialRack"
rack.spawn.usd_path = f"{assets_path}/usd/Vial_rack_simple.usda"
rack.spawn.mass_props = sim_utils.MassPropertiesCfg(mass=0.2)

vial.spawn.mass_props = sim_utils.MassPropertiesCfg(mass=0.02)
vial.spawn.rigid_props = sim_utils.RigidBodyPropertiesCfg(angular_damping=100.0)
VIAL_SPAWN_Z = 0.05


@configclass
class VialsToRackSceneCfg(SO101TaskSceneCfg):
    # Override robot with contact sensors enabled
    robot: ArticulationCfg = S0101_CONTACT_GRASP_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot"
    )

    vial_1 = vial.replace()
    vial_1.prim_path = "{ENV_REGEX_NS}/Vial_1"
    vial_1.init_state.pos = (0.23, -0.08, VIAL_SPAWN_Z)
    vial_1.init_state.rot = euler_angles_to_quat(np.array([0, 90, 0]), degrees=True)

    vial_2 = vial.replace()
    vial_2.prim_path = "{ENV_REGEX_NS}/Vial_2"
    vial_2.init_state.pos = (0.23, 0, VIAL_SPAWN_Z)
    vial_2.init_state.rot = euler_angles_to_quat(np.array([0, 90, 0]), degrees=True)

    vial_3 = vial.replace()
    vial_3.prim_path = "{ENV_REGEX_NS}/Vial_3"
    vial_3.init_state.pos = (0.23, -0.16, VIAL_SPAWN_Z)
    vial_3.init_state.rot = euler_angles_to_quat(np.array([0, 90, 0]), degrees=True)

    rack_left = rack.replace()
    rack_left.prim_path = "{ENV_REGEX_NS}/Rack_Left"
    rack_left.init_state.pos = (0.18, 0.08, 0.06)

    # Contact sensor on gripper jaw to detect vial grasping
    contact_grasp = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/jaw",
        update_period=0.0,
        history_length=1,
        debug_vis=False,
        filter_prim_paths_expr=[
            "{ENV_REGEX_NS}/Vial_1",
            "{ENV_REGEX_NS}/Vial_2",
            "{ENV_REGEX_NS}/Vial_3",
        ],
    )


@configclass
class VialsToRackDRSceneCfg(VialsToRackSceneCfg):
    sky_light = AssetBaseCfg(
        prim_path="/World/sky_light",
        spawn=sim_utils.DomeLightCfg(
            intensity=1000.0,
            texture_file=f"{assets_path}/hdri/moon_lab_1k.exr",
            visible_in_primary_ray=False,
            enable_color_temperature=True,
            color_temperature=6500.0,
        ),
    )

    def __post_init__(self) -> None:
        super().__post_init__()


@configclass
class VialsToRackEventCfg(TaskEventCfg):
    """Configuration for events."""

    reset_vials_setup = EventTerm(
        func=reset_vials_rack,
        mode="reset",
        params={
            "vials": ["vial_1", "vial_2", "vial_3"],
            "rack": "rack_left",
            "rack_pose_range": {
                "x": (-0.04, 0.04),
                "y": (-0.01, 0.01),
                "yaw": (-0.5, 0.5),
            },
            "pose_range": {
                "x": (-0.04, 0.04),
                "y": (-0.01, 0.01),
                "roll": (-0.3, 0.3),
                "yaw": (0.0, 0.0),
            },
            "fixed_vial_z": 0.05,
        },
    )


@configclass
class VialsToRackEventDRCfg(VialsToRackEventCfg):

    reset_set_robot_visual_material = EventTerm(
        func=randomize_robot_color,
        mode="reset",
        params={
            "color_names": list(ROBOT_COLORS.keys()),
        },
    )

    reset_sky_light = EventTerm(
        func=randomize_sky_light,
        mode="reset",
        params={
            "exposure_range": (-4.0, 3.0),
            "temperature_range": (2500.0, 9500.0),
            "textures_root": f"{assets_path}/hdri",
            "asset_cfg": SceneEntityCfg("sky_light"),
        },
    )

    reset_mat_rotation = EventTerm(
        func=randomize_mat_rotation,
        mode="reset",
        params={
            "yaw_range": (-0.3, 0.3),  # ±17° rotation
            "asset_cfg": SceneEntityCfg("mat"),
        },
    )


@configclass
class VialsToRackObservationsCfg(TaskObservationsCfg):
    """Configuration for observations."""

    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask tracking."""

        vial_grasped = ObsTerm(
            func=any_vial_grasped,
            params={
                "contact_sensor_cfg": SceneEntityCfg("contact_grasp"),
                "vials": ["vial_1", "vial_2", "vial_3"],
                "min_height": 0.055,  # 5.5 cm - check debug output for actual resting height
                "warmup_steps": 30,
                "force_threshold": 2,  # N
            },
        )

        vial_placed = ObsTerm(
            func=vial_placed_on_rack,
            params={
                "contact_sensor_cfg": SceneEntityCfg("contact_grasp"),
                "vials": ["vial_1", "vial_2", "vial_3"],
                "rack_name": "rack_left",
                "warmup_steps": 30,
                "grasp_history_window": 20,
                "force_threshold": 2,  # N
                # Rack local dimensions from Vial_rack_simple.usda extent
                "rack_local_x_min": 0.0,
                "rack_local_x_max": 0.12,
                "rack_local_y_min": 0.0,
                "rack_local_y_max": 0.12,
                # Slot entry at local z=0.1; vial center must be below this
                "rack_local_z_max": 0.1,
                # abs(vial_up_z) must exceed this (vial is vertical, not on its side)
                "vertical_threshold": 0.7,
            },
        )

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = False

    subtask_terms: SubtaskCfg = SubtaskCfg()


@configclass
class VialsToRackTerminationsCfg:
    """Termination terms for the vials to tray evaluation task."""

    time_out = DoneTerm(
        func=time_out,
        time_out=True,
    )

    success = DoneTerm(
        func=vial_placed_on_rack_termination,
        time_out=False,
        params={
            "contact_sensor_cfg": SceneEntityCfg("contact_grasp"),
            "vials": ["vial_1", "vial_2", "vial_3"],
            "rack_name": "rack_left",
            "warmup_steps": 30,
            "grasp_history_window": 20,
            "force_threshold": 2,  # N
            "rack_local_x_min": 0.0,
            "rack_local_x_max": 0.12,
            "rack_local_y_min": 0.0,
            "rack_local_y_max": 0.12,
            "rack_local_z_max": 0.1,
            "vertical_threshold": 0.7,
        },
    )



@configclass
class VialsToRackEnvCfg(SO101TaskEnvCfg):
    """
    Base config.
    """
    scene: VialsToRackSceneCfg = VialsToRackSceneCfg()
    events: VialsToRackEventCfg = VialsToRackEventCfg()
    observations: VialsToRackObservationsCfg = VialsToRackObservationsCfg()


@configclass
class VialsToRackDREnvCfg(VialsToRackEnvCfg):
    """
    Domain Randomization config.
    """
    scene: VialsToRackDRSceneCfg = VialsToRackDRSceneCfg()
    events: VialsToRackEventDRCfg = VialsToRackEventDRCfg()



@configclass
class VialsToRackEvalEnvCfg(VialsToRackEnvCfg):
    """
    Eval config.
    """
    terminations: VialsToRackTerminationsCfg = VialsToRackTerminationsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        self.episode_length_s = 450 / 60.0


@configclass
class VialsToRackEvalDREnvCfg(VialsToRackDREnvCfg):
    """
    Eval config with Domain Randomization.
    """
    terminations: VialsToRackTerminationsCfg = VialsToRackTerminationsCfg()

    def __post_init__(self) -> None:
        super().__post_init__()
        self.episode_length_s = 450 / 60.0
