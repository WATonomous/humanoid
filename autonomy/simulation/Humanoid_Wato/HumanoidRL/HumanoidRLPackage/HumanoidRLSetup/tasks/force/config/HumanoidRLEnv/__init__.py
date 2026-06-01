import gymnasium as gym

from . import agents

_ENTRY = f"{__name__}.env_entry:make_force_env"

gym.register(
    id="Isaac-Force-Impulse-Humanoid-Arm-v0",
    entry_point=_ENTRY,
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:HumanoidArmForceEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:HumanoidArmForcePPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Force-Impulse-Humanoid-Arm-Play-v0",
    entry_point=_ENTRY,
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:HumanoidArmForceEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:HumanoidArmForcePPORunnerCfg",
    },
)
