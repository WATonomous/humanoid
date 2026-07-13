import gymnasium as gym

from . import agents

_WATO_ENVS = (
    ("Rough", "rough_env_cfg:WatoHumanoidRoughEnvCfg", "WatoHumanoidRoughPPORunnerCfg", ""),
    ("Rough", "rough_env_cfg:WatoHumanoidRoughEnvCfg_PLAY", "WatoHumanoidRoughPPORunnerCfg", "-Play"),
    ("Flat", "flat_env_cfg:WatoHumanoidFlatEnvCfg", "WatoHumanoidFlatPPORunnerCfg", ""),
    ("Flat", "flat_env_cfg:WatoHumanoidFlatEnvCfg_PLAY", "WatoHumanoidFlatPPORunnerCfg", "-Play"),
)

for _terrain, _env_cfg, _agent_cfg, _suffix in _WATO_ENVS:
    _kwargs = {
        "env_cfg_entry_point": f"{__name__}.{_env_cfg}",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:{_agent_cfg}",
    }
    for _prefix in ("Isaac-Locomotion", "Isaac-Velocity"):
        gym.register(
            id=f"{_prefix}-{_terrain}-WatoHumanoid{_suffix}-v0",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs=_kwargs,
        )
