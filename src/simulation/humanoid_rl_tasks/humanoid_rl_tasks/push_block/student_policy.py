"""Minimal CNN+MLP student for push-block vision distillation."""

from __future__ import annotations

import torch
import torch.nn as nn


class StudentVisionPolicy(nn.Module):
    """RGB CNN + proprio MLP -> continuous joint actions."""

    def __init__(
        self,
        proprio_dim: int,
        num_actions: int,
        image_shape: tuple[int, int, int] = (64, 64, 3),
        cnn_feat_dim: int = 128,
        mlp_hidden_dims: list[int] | None = None,
    ):
        super().__init__()
        if mlp_hidden_dims is None:
            mlp_hidden_dims = [256, 128, 64]

        h, w, c = image_shape
        self.image_shape = image_shape

        self.cnn = nn.Sequential(
            nn.Conv2d(c, 32, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Flatten(),
        )
        with torch.no_grad():
            dummy = torch.zeros(1, c, h, w)
            flat_dim = self.cnn(dummy).shape[-1]
        self.cnn_fc = nn.Sequential(
            nn.Linear(flat_dim, cnn_feat_dim),
            nn.ReLU(inplace=True),
        )

        layers: list[nn.Module] = []
        in_dim = cnn_feat_dim + proprio_dim
        for hidden in mlp_hidden_dims:
            layers.append(nn.Linear(in_dim, hidden))
            layers.append(nn.ELU(inplace=True))
            in_dim = hidden
        layers.append(nn.Linear(in_dim, num_actions))
        self.mlp = nn.Sequential(*layers)

    def forward(self, rgb: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        """
        Args:
            rgb: (N, H, W, C) float image (already normalized by env obs term).
            proprio: (N, D) student proprio vector.
        """
        if rgb.dim() == 4 and rgb.shape[-1] in (1, 3, 4):
            # NHWC -> NCHW
            x = rgb.permute(0, 3, 1, 2).contiguous()
        else:
            x = rgb
        # Drop alpha if present
        if x.shape[1] == 4:
            x = x[:, :3]
        feat = self.cnn_fc(self.cnn(x))
        return self.mlp(torch.cat([feat, proprio], dim=-1))
