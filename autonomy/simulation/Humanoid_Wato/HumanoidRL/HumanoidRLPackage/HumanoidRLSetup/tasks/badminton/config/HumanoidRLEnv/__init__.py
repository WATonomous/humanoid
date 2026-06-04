import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Badminton-Intercept-Humanoid-Arm-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:HumanoidArmBadmintonEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:HumanoidArmBadmintonPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:HumanoidArmBadmintonEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:HumanoidArmBadmintonPPORunnerCfg",
    },
)
