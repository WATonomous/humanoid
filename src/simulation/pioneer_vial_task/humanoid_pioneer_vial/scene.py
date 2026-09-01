"""Vial-rack manipulation scene for the pioneer bimanual arm.

Single source of truth for the scene geometry -- imported both by teleop
(``pioneer_humanoid.teleop_scenes`` -> ``SCENE_CFGS["vial_rack"]``) and, later,
by an RL/IL env cfg once someone trains a policy on it.

Grounding follows ``tools/isaac_harness/scenes/bimanual_vial_rack.sh``: the arm
at the origin (no floor-stand lift) and a low table (top at ``TABLE_TOP_Z``) so
the rack/vials sit in the arm's manipulation reach. The rack + vial USDs are the
so101 vial task's assets.

``robot`` and ``ee_frame`` are ``MISSING`` -- the teleop registry (or an env
cfg) fills them in.

FIRST PASS: the rack / vial xy placement is the harness starting point, pulled
in ~0.1 m. It should get a reach-tuning pass against the arm that actually
drives it (keyboard_teleop drives the LEFT / L-suffix chain) -- driving the
scene once is that loop.
"""
from __future__ import annotations

from dataclasses import MISSING
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.utils import configclass

_ASSETS = Path(__file__).resolve().parents[4] / "assets" / "lerobot" / "so101_vial_task" / "usd"
VIAL_RACK_USD = str(_ASSETS / "Vial_rack_simple.usda")
VIAL_USD = str(_ASSETS / "Vial_opaque.usda")

# ── grounding (harness bimanual_vial_rack.sh) ────────────────────────────────
GROUND_Z = -1.05
TABLE_SIZE = (0.9, 1.2, 0.05)
TABLE_POS = (0.55, -0.15, -0.275)   # top at TABLE_TOP_Z
TABLE_TOP_Z = TABLE_POS[2] + TABLE_SIZE[2] / 2  # -0.25

# ── rack + vials (first pass; reach-tune against the LEFT arm) ────────────────
RACK_POS = (0.48, -0.32, TABLE_TOP_Z)
VIAL_INIT_POS = [
    (0.42, -0.05, TABLE_TOP_Z + 0.03),
    (0.42, -0.13, TABLE_TOP_Z + 0.03),
    (0.42, -0.21, TABLE_TOP_Z + 0.03),
]

_VIAL_RIGID_PROPS = sim_utils.RigidBodyPropertiesCfg(
    solver_position_iteration_count=16,
    solver_velocity_iteration_count=1,
    max_depenetration_velocity=1.0,
    disable_gravity=False,
)


@configclass
class VialRackSceneCfg(InteractiveSceneCfg):
    """Table + vial rack + 3 loose vials, for pioneer-arm vial-placement teleop."""

    robot: ArticulationCfg = MISSING
    ee_frame: FrameTransformerCfg = MISSING

    ground = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, GROUND_Z)),
        spawn=sim_utils.GroundPlaneCfg(),
    )
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=TABLE_POS),
        spawn=sim_utils.CuboidCfg(
            size=TABLE_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.25, 0.15)),
        ),
    )

    # kinematic rack (collidable, not physics-driven)
    vial_rack = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/VialRack",
        init_state=AssetBaseCfg.InitialStateCfg(pos=RACK_POS),
        spawn=sim_utils.UsdFileCfg(
            usd_path=VIAL_RACK_USD,
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )

    vial_1 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Vial1",
        init_state=RigidObjectCfg.InitialStateCfg(pos=VIAL_INIT_POS[0], rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.UsdFileCfg(usd_path=VIAL_USD, rigid_props=_VIAL_RIGID_PROPS),
    )
    vial_2 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Vial2",
        init_state=RigidObjectCfg.InitialStateCfg(pos=VIAL_INIT_POS[1], rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.UsdFileCfg(usd_path=VIAL_USD, rigid_props=_VIAL_RIGID_PROPS),
    )
    vial_3 = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Vial3",
        init_state=RigidObjectCfg.InitialStateCfg(pos=VIAL_INIT_POS[2], rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.UsdFileCfg(usd_path=VIAL_USD, rigid_props=_VIAL_RIGID_PROPS),
    )
