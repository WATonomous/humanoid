"""Not a standalone eval script — used by ``lerobot_eval.py`` for sim rollout eval."""

from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np
import torch

from humanoid_act.checkpoint import load_policy


def lerobot_cam_to_sim_name(camera_key: str) -> str:
    prefix = "observation.images."
    if camera_key.startswith(prefix):
        return camera_key[len(prefix) :]
    return camera_key


class LocalCustomACTPolicy:
    """
    Load ``best.pt`` / ``last.pt`` from humanoid-act-train and step the sim env.

    Expects the same sim camera names as training (e.g. ego, external_D455).
    State/action space matches the LeRobot dataset the checkpoint was trained on.
    """

    def __init__(
        self,
        robot_iface,
        checkpoint_path: str,
        *,
        action_horizon: int | None = None,
    ) -> None:
        self._iface = robot_iface
        self._device = torch.device(robot_iface.device)
        ckpt_path = Path(checkpoint_path)
        self._policy, self._config, self._stats = load_policy(ckpt_path, self._device)
        self._action_horizon = action_horizon or self._config.chunk_size
        self._action_queue: deque[torch.Tensor] = deque()
        self._camera_order = list(self._config.camera_names)
        self._validate_cameras()
        print(
            f"[INFO]: Loaded custom ACT checkpoint {ckpt_path} "
            f"(chunk_size={self._config.chunk_size}, cameras={self._camera_order})"
        )

    def _validate_cameras(self) -> None:
        sim_cams = set(self._iface.cameras.keys())
        for cam_key in self._camera_order:
            sim_name = lerobot_cam_to_sim_name(cam_key)
            if sim_name not in sim_cams:
                raise ValueError(
                    f"Checkpoint expects camera '{sim_name}' (from {cam_key}) "
                    f"but sim only has {sorted(sim_cams)}"
                )

    def connect(self) -> None:
        return

    def reset(self) -> None:
        self._action_queue.clear()

    def _build_images(self, visual_obs: dict) -> np.ndarray:
        images = []
        for cam_key in self._camera_order:
            sim_name = lerobot_cam_to_sim_name(cam_key)
            img = visual_obs[f"rgb_{sim_name}"][0].detach().cpu().numpy()
            if img.dtype != np.uint8:
                img = np.clip(img, 0, 255).astype(np.uint8)
            if img.ndim == 3 and img.shape[-1] == 3:
                img = np.transpose(img, (2, 0, 1))
            images.append(img.astype(np.float32) / 255.0)
        return np.stack(images, axis=0)

    def _predict_action_chunk(
        self, joint_positions: torch.Tensor, visual_obs: dict
    ) -> np.ndarray:
        raw_state = self._iface.get_raw_actions_from_radians(joint_positions)
        qpos = self._stats.normalize_state(raw_state.cpu().numpy().astype(np.float32))
        images = self._build_images(visual_obs)

        qpos_t = torch.from_numpy(qpos).float().unsqueeze(0).to(self._device)
        images_t = torch.from_numpy(images).float().unsqueeze(0).to(self._device)

        with torch.inference_mode():
            chunk = self._policy(qpos_t, images_t)
        chunk_np = chunk[0].detach().cpu().numpy()
        return self._stats.denormalize_action(chunk_np)

    def _refill_action_queue(
        self, joint_positions: torch.Tensor, visual_obs: dict
    ) -> None:
        chunk = self._predict_action_chunk(joint_positions, visual_obs)
        horizon = min(self._action_horizon, chunk.shape[0])
        for idx in range(horizon):
            action = torch.tensor(chunk[idx], dtype=torch.float32, device=self._device)
            sim_action = self._iface.get_mapped_actions_vectorized(action)
            self._action_queue.append(sim_action)

    def get_action(
        self, joint_positions: torch.Tensor, visual_obs: dict, log: bool = False
    ) -> torch.Tensor:
        del log
        if not self._action_queue:
            self._refill_action_queue(joint_positions, visual_obs)
        return self._action_queue.popleft()
