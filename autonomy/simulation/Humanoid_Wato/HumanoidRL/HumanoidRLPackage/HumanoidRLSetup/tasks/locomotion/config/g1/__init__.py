import gymnasium as gym

from . import agents

_G1_ENVS = (
    ("Rough", "rough_env_cfg:G1RoughEnvCfg", "G1RoughPPORunnerCfg", ""),
    ("Rough", "rough_env_cfg:G1RoughEnvCfg_PLAY", "G1RoughPPORunnerCfg", "-Play"),
    ("Flat", "flat_env_cfg:G1FlatEnvCfg", "G1FlatPPORunnerCfg", ""),
    ("Flat", "flat_env_cfg:G1FlatEnvCfg_PLAY", "G1FlatPPORunnerCfg", "-Play"),
)

for _terrain, _env_cfg, _agent_cfg, _suffix in _G1_ENVS:
    _kwargs = {
        "env_cfg_entry_point": f"{__name__}.{_env_cfg}",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:{_agent_cfg}",
    }
    for _prefix in ("Isaac-Locomotion", "Isaac-Velocity"):
        gym.register(
            id=f"{_prefix}-{_terrain}-G1{_suffix}-v0",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            disable_env_checker=True,
            kwargs=_kwargs,
        )
