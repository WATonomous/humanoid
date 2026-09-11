import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Repose-Cube-PioneerHand-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pioneer_hand_env_cfg:PioneerHandCubeEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PioneerHandCubePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Repose-Cube-PioneerHand-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pioneer_hand_env_cfg:PioneerHandCubeEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PioneerHandCubePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Repose-Cube-PioneerHand-NoVelObs-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pioneer_hand_env_cfg:PioneerHandCubeNoVelObsEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PioneerHandCubeNoVelObsPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Repose-Cube-PioneerHand-NoVelObs-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.pioneer_hand_env_cfg:PioneerHandCubeNoVelObsEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PioneerHandCubeNoVelObsPPORunnerCfg",
    },
)
