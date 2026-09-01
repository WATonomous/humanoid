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

# Isaac Lab Mimic variant: same scene/action space (see
# pick_place_bimanual_mimic_env.py), used by
# annotate_demos.py / generate_dataset.py / record_mimic_source_demos.py.
# The registered default cfg self-configures with table-mode
# PickPlaceTaskParams() defaults; pass make_mimic_env_cfg(params) explicitly
# via gym.make(..., cfg=...) for a specific YAML config (tray/stack/etc),
# the same way generate_demos.py does for the base env id.
gym.register(
    id="Isaac-PickPlace-BimanualLeft-Mimic-v0",
    entry_point=(
        "HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual."
        "pick_place_bimanual_mimic_env:PickPlaceBimanualMimicEnv"
    ),
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": (
            "HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual."
            "pick_place_bimanual_mimic_env:PickPlaceBimanualMimicEnvCfg"
        ),
    },
)
