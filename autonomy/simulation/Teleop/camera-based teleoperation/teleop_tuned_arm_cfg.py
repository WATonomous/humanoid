"""Teleop overrides on canonical humanoid_arm_hand.ARM_CFG (camera hand teleop)."""

from __future__ import annotations

import sys
from pathlib import Path

from isaaclab.assets.articulation import ArticulationCfg

_HUMANOID_WATO_ROOT = Path(__file__).resolve().parents[2] / "Humanoid_Wato"
if str(_HUMANOID_WATO_ROOT) not in sys.path:
    sys.path.insert(0, str(_HUMANOID_WATO_ROOT))

from HumanoidRL.HumanoidRLPackage.HumanoidRLSetup.modelCfg.humanoid_arm_hand import (  # noqa: E402
    ARM_CFG,
)

# (stiffness, damping) — softer than RL defaults for live dex-retargeting.
_TELEOP_FINGER_GAINS = {
    "finger_mcp": (3.0, 0.12),
    "finger_pip": (1.5, 0.06),
    "finger_dip": (0.5, 0.02),
    "thumb_cmc": (7.6, 0.30),
    "thumb_mcp": (2.5, 0.10),
    "thumb_ip": (0.5, 0.02),
}

_teleop_zero_pose = {name: 0.0 for name in ARM_CFG.init_state.joint_pos}
_teleop_zero_pose["mcp_thumb"] = 0.79

TELEOP_TUNED_ARM_CFG = ARM_CFG.replace(
    spawn=ARM_CFG.spawn.replace(
        articulation_props=ARM_CFG.spawn.articulation_props.replace(
            enabled_self_collisions=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(joint_pos=_teleop_zero_pose),
    actuators={
        **ARM_CFG.actuators,
        **{
            name: ARM_CFG.actuators[name].replace(stiffness=k, damping=d)
            for name, (k, d) in _TELEOP_FINGER_GAINS.items()
        },
    },
)
