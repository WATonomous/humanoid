"""Build ACT model + optimizer without argparse side effects."""

from __future__ import annotations

from types import SimpleNamespace

import torch

from humanoid_act.detr.models import build_ACT_model


def build_act_model_and_optimizer(config) -> tuple[torch.nn.Module, torch.optim.Optimizer]:
    args = SimpleNamespace(
        lr=config.lr,
        lr_backbone=config.lr_backbone,
        weight_decay=config.weight_decay,
        backbone=config.backbone,
        dilation=False,
        position_embedding=config.position_embedding,
        camera_names=list(config.camera_names),
        enc_layers=config.enc_layers,
        dec_layers=config.dec_layers,
        dim_feedforward=config.dim_feedforward,
        hidden_dim=config.hidden_dim,
        dropout=config.dropout,
        nheads=config.nheads,
        num_queries=config.chunk_size,
        pre_norm=False,
        masks=False,
        state_dim=config.state_dim,
    )

    model = build_ACT_model(args)
    param_dicts = [
        {
            "params": [
                p for n, p in model.named_parameters() if "backbone" not in n and p.requires_grad
            ]
        },
        {
            "params": [
                p for n, p in model.named_parameters() if "backbone" in n and p.requires_grad
            ],
            "lr": args.lr_backbone,
        },
    ]
    optimizer = torch.optim.AdamW(param_dicts, lr=args.lr, weight_decay=args.weight_decay)
    return model, optimizer
