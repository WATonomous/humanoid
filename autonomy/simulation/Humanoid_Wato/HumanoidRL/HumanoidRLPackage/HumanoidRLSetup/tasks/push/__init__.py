import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-SO-ARM101-Push-Block-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SoArm101PushBlockEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-SO-ARM101-Push-Block-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:SoArm101PushBlockEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-SO-ARM101-Push-Block-Distill-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.distill_env_cfg:SoArm101PushBlockDistillEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-SO-ARM101-Push-Block-Distill-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.distill_env_cfg:SoArm101PushBlockDistillEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)
