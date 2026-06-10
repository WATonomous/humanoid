# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents
from .dextrah_kuka_allegro_env_cfg import DextrahKukaAllegroEnvCfg

##
# Register Gym environments.
##

gym.register(
    id="Dextrah-Kuka-Allegro",
    entry_point="dextrah_rgb.tasks.dextrah_kuka_allegro.dextrah_kuka_allegro_env:DextrahKukaAllegroEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DextrahKukaAllegroEnvCfg,
        # Teacher training: HumanoidRLPackage/rsl_rl_scripts/train.py --task=Dextrah-Kuka-Allegro
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:DextrahKukaAllegroLstmPPORunnerCfg",
        # Distillation / eval still use RL-Games student + teacher configs
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_lstm_cfg.yaml",
    },
)
