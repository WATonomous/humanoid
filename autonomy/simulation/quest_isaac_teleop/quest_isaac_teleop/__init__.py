def register_bimanual_tasks() -> list:
    """Register the single-articulation bimanual arm teleop task.

    Called via --external_callback from run_bimanual_teleop.sh so that Isaac Sim
    is initialized before our env cfg imports happen. Returns an empty list
    (no extra CLI args consumed).
    """
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
