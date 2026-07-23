"""ACT policy wrapper (CVAE + L1/KL loss)."""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms

from humanoid_act.config import ACTConfig
from humanoid_act.detr.build import build_act_model_and_optimizer


def kl_divergence(mu, logvar):
    if mu.data.ndimension() == 4:
        mu = mu.view(mu.size(0), mu.size(1))
    if logvar.data.ndimension() == 4:
        logvar = logvar.view(logvar.size(0), logvar.size(1))
    klds = -0.5 * (1 + logvar - mu.pow(2) - logvar.exp())
    return klds.sum(1).mean(0, True)


class ACTPolicy(nn.Module):
    def __init__(self, config: ACTConfig) -> None:
        super().__init__()
        self.config = config
        model, optimizer = build_act_model_and_optimizer(config)
        self.model = model
        self._optimizer = optimizer
        self.kl_weight = config.kl_weight
        self._imagenet_norm = transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        )

    def configure_optimizers(self) -> torch.optim.Optimizer:
        return self._optimizer

    def _normalize_images(self, images: torch.Tensor) -> torch.Tensor:
        b, n, c, h, w = images.shape
        flat = images.reshape(b * n, c, h, w)
        flat = self._imagenet_norm(flat)
        return flat.reshape(b, n, c, h, w)

    def forward(
        self,
        qpos: torch.Tensor,
        images: torch.Tensor,
        actions: torch.Tensor | None = None,
        is_pad: torch.Tensor | None = None,
    ) -> torch.Tensor | dict[str, torch.Tensor]:
        images = self._normalize_images(images)
        env_state = None

        if actions is not None:
            assert is_pad is not None
            actions = actions[:, : self.model.num_queries]
            is_pad = is_pad[:, : self.model.num_queries]
            a_hat, _is_pad_hat, (mu, logvar) = self.model(
                qpos, images, env_state, actions, is_pad
            )
            total_kld = kl_divergence(mu, logvar)
            all_l1 = F.l1_loss(actions, a_hat, reduction="none")
            l1 = (all_l1 * ~is_pad.unsqueeze(-1)).mean()
            kl = total_kld[0]
            loss = l1 + kl * self.kl_weight
            return {"loss": loss, "l1": l1, "kl": kl}

        a_hat, _, _ = self.model(qpos, images, env_state)
        return a_hat
