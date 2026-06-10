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
#
# This file contains code derived from Isaac Lab
# (https://github.com/isaac-sim/IsaacLab)
# Copyright (c) 2022-2025, The Isaac Lab Project Developers. All rights reserved.
# Licensed under the BSD-3-Clause License.

"""Package containing task implementations for the extension."""

##
# Register Gym environments.
##

from isaaclab_tasks.utils import import_packages

# The blacklist is used to prevent importing configs from sub-packages
_BLACKLIST_PKGS = ["utils", ".mdp"]
# Import all configs in this package
import_packages(__name__, _BLACKLIST_PKGS)



import gymnasium as gym


##
# Register Gym environments.
##
gym.register(
    id="Lerobot-So101-Teleop-Base",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.so101_env_cfg:SO101TeleopEnvCfg",
    },
)

gym.register(
    id="Lerobot-So101-Teleop-Task",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.task_env_cfg:SO101TaskEnvCfg",
    },
)

gym.register(
    id="Lerobot-So101-Teleop-Vials-To-Rack",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vials_to_rack_env_cfg:VialsToRackEnvCfg",
    },
)

gym.register(
    id="Lerobot-So101-Teleop-Vials-To-Rack-DR",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vials_to_rack_env_cfg:VialsToRackDREnvCfg",
    },
)


gym.register(
    id="Lerobot-So101-Teleop-Vials-To-Rack-Eval",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vials_to_rack_env_cfg:VialsToRackEvalEnvCfg",
    },
)


gym.register(
    id="Lerobot-So101-Teleop-Vials-To-Rack-DR-Eval",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.vials_to_rack_env_cfg:VialsToRackEvalDREnvCfg",
    },
)

