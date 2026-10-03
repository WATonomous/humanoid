"""Garment-fold Gym task(s).

`GarmentPioneerEnv` / `GarmentPioneerEnvCfg` are WATonomous originals; the
`GarmentEnv` base and its helpers are vendored from the LeHome Challenge
(Apache-2.0). See ../../NOTICE.
"""
import gymnasium as gym

gym.register(
    id="Humanoid-GarmentFold-Bimanual-Pioneer-v0",
    entry_point="humanoid_garment_fold.tasks.garment_pioneer_env:GarmentPioneerEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point":
            "humanoid_garment_fold.tasks.garment_pioneer_cfg:GarmentPioneerEnvCfg",
    },
)
