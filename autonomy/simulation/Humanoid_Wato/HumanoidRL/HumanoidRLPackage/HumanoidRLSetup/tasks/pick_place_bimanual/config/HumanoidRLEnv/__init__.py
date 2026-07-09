import gymnasium as gym

gym.register(
    id="Isaac-PickPlace-BimanualLeft-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": (
            "HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual."
            "pick_place_env_cfg:PickPlaceBimanualEnvCfg"
        ),
    },
)
