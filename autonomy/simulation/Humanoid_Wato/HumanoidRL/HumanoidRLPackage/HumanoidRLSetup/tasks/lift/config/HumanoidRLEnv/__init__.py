import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Lift-Cube-SO101-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SO101LiftEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:SO101LiftPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Lift-Cube-SO101-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SO101LiftEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:SO101LiftPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Lift-Cube-Wato-Bimanual-Right-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wato_bimanual_right_env_cfg:WatoBimanualRightLiftEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WatoBimanualRightLiftPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Lift-Cube-Wato-Bimanual-Right-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.wato_bimanual_right_env_cfg:WatoBimanualRightLiftEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WatoBimanualRightLiftPPORunnerCfg",
    },
)
