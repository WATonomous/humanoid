import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Repose-Cube-WatoHand-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wato_hand_env_cfg:WatoHandCubeEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WatoHandCubePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Repose-Cube-WatoHand-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wato_hand_env_cfg:WatoHandCubeEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WatoHandCubePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Repose-Cube-WatoHand-NoVelObs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wato_hand_env_cfg:WatoHandCubeNoVelObsEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WatoHandCubeNoVelObsPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Repose-Cube-WatoHand-NoVelObs-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wato_hand_env_cfg:WatoHandCubeNoVelObsEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WatoHandCubeNoVelObsPPORunnerCfg",
    },
)
