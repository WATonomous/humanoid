from __future__ import annotations
from dataclasses import dataclass
import torch
from isaaclab.devices.retargeter_base import RetargeterBase, RetargeterCfg

# Current lower body retargeter just keep the lower body at constant height and not moving

class WatoLowerBodyStandingRetargeter(RetargeterBase):
    """Provides lower body standing commands"""

    def __init__(self, cfg: G1LowerBodyStandingRetargeterCfg):
        super().__init__(cfg)
        self.cfg = cfg

    def retarget(self, data: dict) -> torch.Tensor:
        return torch.tensor([0.0, 0.0, 0.0, self.cfg.hip_height], device=self.cfg.sim_device)

    def get_requirements(self) -> list[RetargeterBase.Requirement]:
        # This retargeter does not consume any device data
        return []


@dataclass
class WatoLowerBodyStandingRetargeterCfg(RetargeterCfg):
    hip_height: float = 0.72
    retargeter_type: type[RetargeterBase] = G1LowerBodyStandingRetargeter
