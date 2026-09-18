"""Appended to openpi's src/openpi/training/config.py at Docker build time
(see isaac_pi.Dockerfile) rather than edited in-place in the openpi checkout,
so upstream diffs on that file stay visible/mergeable when we bump the pinned
openpi commit.

Safe to append at the end of the file: _CONFIGS_DICT (built from the _CONFIGS
list earlier in the file) is just a plain module-level dict, so mutating it
here after the fact works the same as if our entry had been in the original
list.

UNVERIFIED -- see pick_place_policy.py docstring for what to check before
trusting this config for a real fine-tuning run.
"""

import openpi.policies.pick_place_policy as pick_place_policy  # noqa: E402

_CONFIGS_DICT["pi05_pickplace_bimanual"] = TrainConfig(  # noqa: F821
    name="pi05_pickplace_bimanual",
    model=pi0_config.Pi0Config(pi05=True, action_horizon=10, discrete_state_input=False),  # noqa: F821
    data=SimpleDataConfig(  # noqa: F821
        repo_id="humanoid/pick_place_bimanual_left",
        assets=AssetsConfig(asset_id="wato_bimanual_left"),  # noqa: F821
        data_transforms=lambda model: _transforms.Group(  # noqa: F821
            inputs=[pick_place_policy.PickPlaceInputs(model_type=model.model_type)],
            outputs=[pick_place_policy.PickPlaceOutputs()],
        ),
        base_config=DataConfig(  # noqa: F821
            prompt_from_task=True,
            # Raw LeRobot column is singular "action" (see src/il/humanoid_il/schema.py),
            # not the DataConfig default "actions".
            action_sequence_keys=("action",),
            repack_transforms=_transforms.Group(  # noqa: F821
                inputs=[
                    _transforms.RepackTransform(  # noqa: F821
                        {
                            "observation/state": "observation.state",
                            "observation/image": "observation.images.external",
                            "observation/wrist_image": "observation.images.wrist",
                            "actions": "action",
                        }
                    )
                ]
            ),
        ),
    ),
    weight_loader=weight_loaders.CheckpointWeightLoader(  # noqa: F821
        "gs://openpi-assets/checkpoints/pi05_base/params"
    ),
    num_train_steps=30_000,
)
