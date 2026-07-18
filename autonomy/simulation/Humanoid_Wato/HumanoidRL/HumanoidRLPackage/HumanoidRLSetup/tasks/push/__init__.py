import gymnasium as gym

from . import agents

##
# Register Gym environments.
#
# Bimanual-arm push-block task (the SO101 variant was removed; the bimanual arm
# replaces it). Vision distillation is now bimanual-only as well.
##

gym.register(
    id="Isaac-Bimanual-Push-Block-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.bimanual_env_cfg:BimanualPushBlockEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BimanualPushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Bimanual-Push-Block-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.bimanual_env_cfg:BimanualPushBlockEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BimanualPushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Bimanual-Push-Block-Distill-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.distill_env_cfg:BimanualPushBlockDistillEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BimanualPushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)

gym.register(
    id="Isaac-Bimanual-Push-Block-Distill-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    kwargs={
        "env_cfg_entry_point": f"{__name__}.distill_env_cfg:BimanualPushBlockDistillEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BimanualPushBlockPPORunnerCfg",
    },
    disable_env_checker=True,
)
