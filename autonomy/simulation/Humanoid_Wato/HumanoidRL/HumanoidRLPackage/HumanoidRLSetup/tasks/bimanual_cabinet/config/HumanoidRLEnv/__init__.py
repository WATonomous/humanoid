import gymnasium as gym

from . import agents

gym.register(
    id="Isaac-Open-Drawer-Bimanual-Arm-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:BimanualArmCabinetEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BimanualArmCabinetPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Open-Drawer-Bimanual-Arm-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:BimanualArmCabinetEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:BimanualArmCabinetPPORunnerCfg",
    },
)
