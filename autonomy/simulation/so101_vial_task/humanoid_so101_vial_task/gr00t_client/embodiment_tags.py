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
from enum import Enum


"""
Embodiment tags are used to identify the robot embodiment in the data.

Naming convention:
<dataset>_<robot_name>

If using multiple datasets, e.g. sim GR1 and real GR1, we can drop the dataset name and use only the robot name.
"""


class EmbodimentTag(Enum):
    ##### Pretrain embodiment tags #####
    ROBOCASA_PANDA_OMRON = "robocasa_panda_omron"
    """
    The RoboCasa Panda robot with omron mobile base.
    """

    GR1 = "gr1"
    """
    The Fourier GR1 robot.
    """

    ##### Pre-registered posttrain embodiment tags #####
    UNITREE_G1 = "unitree_g1"
    """
    The Unitree G1 robot.
    """

    LIBERO_PANDA = "libero_panda"
    """
    The Libero panda robot.
    """

    OXE_GOOGLE = "oxe_google"
    """
    The Open-X-Embodiment Google robot.
    """

    OXE_WIDOWX = "oxe_widowx"
    """
    The Open-X-Embodiment WidowX robot.
    """

    OXE_DROID = "oxe_droid"
    """
    The Open-X-Embodiment DROID robot with relative joint position actions.
    """

    BEHAVIOR_R1_PRO = "behavior_r1_pro"
    """
    The Behavior R1 Pro robot.
    """

    # New embodiment during post-training
    NEW_EMBODIMENT = "new_embodiment"
    """
    Any new embodiment.
    """
