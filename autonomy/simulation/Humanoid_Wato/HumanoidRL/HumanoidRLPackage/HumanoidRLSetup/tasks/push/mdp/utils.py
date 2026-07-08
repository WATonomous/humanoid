"""Shared helpers for the push-block MDP terms."""

from __future__ import annotations

import torch

from isaaclab.assets import RigidObject
from isaaclab.utils.math import quat_apply

# block.usd keeps the OBJ's corner origin: the rigid-body root sits at the mesh
# corner, not the center. The block is a 50.8 mm cube.
BLOCK_HALF_SIZE = 0.0254


def block_center_w(object: RigidObject) -> torch.Tensor:
    """World-frame position of the block's geometric center.

    The converted USD root frame is at the mesh corner, so the center is the
    corner position plus the rotated half-diagonal.
    """
    offset = torch.full((object.num_instances, 3), BLOCK_HALF_SIZE, device=object.device)
    return object.data.root_pos_w + quat_apply(object.data.root_quat_w, offset)
