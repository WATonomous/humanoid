"""openpi data-transform pair for the wato_bimanual_arm pick_place task (left arm only).

Modeled directly on openpi's own src/openpi/policies/libero_policy.py -- see that file
(in the openpi repo, pinned commit referenced in isaac_pi.Dockerfile) for the canonical
explanation of every field. This file is COPY'd into the openpi checkout at build time
(see isaac_pi.Dockerfile) and registered as a TrainConfig in the config.py append block
in that same Dockerfile, under the name "pi05_pickplace_bimanual".

UNVERIFIED: nobody has run a fine-tune with this yet. Before trusting it:
- Confirm the action/state dim (8: joints 1-8 of the left arm, see
  src/il/config/dataset_schema_pick_place_bimanual.yaml) matches what's actually recorded.
- Confirm prompt_from_task (set in the TrainConfig's base_config) actually populates
  "prompt" for this dataset -- if not, add an explicit repack entry or default_prompt.
"""

import dataclasses

import einops
import numpy as np

from openpi import transforms
from openpi.models import model as _model


def make_pick_place_example() -> dict:
    """Random input example for smoke-testing the pick_place policy without a robot/sim."""
    return {
        "observation/state": np.random.rand(8),
        "observation/image": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/wrist_image": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "prompt": "pick up the block and place it on the target",
    }


def _parse_image(image) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    return image


@dataclasses.dataclass(frozen=True)
class PickPlaceInputs(transforms.DataTransformFn):
    """Converts pick_place dataset/inference data into the model's expected input format."""

    model_type: _model.ModelType

    def __call__(self, data: dict) -> dict:
        base_image = _parse_image(data["observation/image"])  # "external" camera
        wrist_image = _parse_image(data["observation/wrist_image"])  # "wrist" camera

        inputs = {
            "state": data["observation/state"],
            "image": {
                "base_0_rgb": base_image,
                "left_wrist_0_rgb": wrist_image,
                # Right arm is frozen for this task and has no camera -- pad with zeros.
                "right_wrist_0_rgb": np.zeros_like(base_image),
            },
            "image_mask": {
                "base_0_rgb": np.True_,
                "left_wrist_0_rgb": np.True_,
                "right_wrist_0_rgb": np.True_ if self.model_type == _model.ModelType.PI0_FAST else np.False_,
            },
        }

        if "actions" in data:
            inputs["actions"] = data["actions"]

        if "prompt" in data:
            inputs["prompt"] = data["prompt"]

        return inputs


@dataclasses.dataclass(frozen=True)
class PickPlaceOutputs(transforms.DataTransformFn):
    """Un-pads model output actions back to our 8-dim left-arm action space."""

    def __call__(self, data: dict) -> dict:
        return {"actions": np.asarray(data["actions"][..., :8])}
