def register_bimanual_tasks() -> list:
    """Register the bimanual teleop task after Isaac Sim starts."""
    import gymnasium as gym

    gym.register(
        id="Isaac-WatoBimanualTeleop-v0",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": "quest_isaac_teleop.bimanual_teleop_env_cfg:WatoBimanualTeleopEnvCfg",
        },
    )
    return []
