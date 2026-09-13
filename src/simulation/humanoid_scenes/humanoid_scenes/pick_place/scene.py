"""Pick and place scene with two tables:
1. Pickup table holding a graspable block.
2. Placement table surrounded by a protective containment fence to prevent dropping.
"""
from __future__ import annotations

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.utils import configclass

from humanoid_scenes import scene

# ── Dimensions and coordinates ──────────────────────────────────────────────
GROUND_Z = -1.05
TABLE_TOP_Z = -0.25
TABLE_HEIGHT = 0.40
TABLE_Z_CENTER = TABLE_TOP_Z - TABLE_HEIGHT / 2  # -0.45

# Table 1 (Pickup Table)
PICK_TABLE_SIZE = (0.35, 0.28, TABLE_HEIGHT)
PICK_TABLE_POS = (0.46, -0.12, TABLE_Z_CENTER)

# Pickup Block
BLOCK_SIZE = (0.045, 0.045, 0.055)
BLOCK_POS = (0.46, -0.12, TABLE_TOP_Z + BLOCK_SIZE[2] / 2)  # -0.2225

# Table 2 (Place Table with Border Fence)
PLACE_TABLE_SIZE = (0.35, 0.28, TABLE_HEIGHT)
PLACE_TABLE_POS = (0.46, -0.42, TABLE_Z_CENTER)

# Fence parameters on Table 2
FENCE_HEIGHT = 0.05
FENCE_THICKNESS = 0.015
FENCE_Z = TABLE_TOP_Z + FENCE_HEIGHT / 2  # -0.225

WALL_N_POS = (PLACE_TABLE_POS[0] + PLACE_TABLE_SIZE[0] / 2 - FENCE_THICKNESS / 2, PLACE_TABLE_POS[1], FENCE_Z)
WALL_N_SIZE = (FENCE_THICKNESS, PLACE_TABLE_SIZE[1], FENCE_HEIGHT)

WALL_S_POS = (PLACE_TABLE_POS[0] - PLACE_TABLE_SIZE[0] / 2 + FENCE_THICKNESS / 2, PLACE_TABLE_POS[1], FENCE_Z)
WALL_S_SIZE = (FENCE_THICKNESS, PLACE_TABLE_SIZE[1], FENCE_HEIGHT)

WALL_E_POS = (PLACE_TABLE_POS[0], PLACE_TABLE_POS[1] + PLACE_TABLE_SIZE[1] / 2 - FENCE_THICKNESS / 2, FENCE_Z)
WALL_E_SIZE = (PLACE_TABLE_SIZE[0], FENCE_THICKNESS, FENCE_HEIGHT)

WALL_W_POS = (PLACE_TABLE_POS[0], PLACE_TABLE_POS[1] - PLACE_TABLE_SIZE[1] / 2 + FENCE_THICKNESS / 2, FENCE_Z)
WALL_W_SIZE = (PLACE_TABLE_SIZE[0], FENCE_THICKNESS, FENCE_HEIGHT)


@scene("pick_place", robot_pos=(0.0, 0.0, 0.0), camera=([1.1, -0.9, 0.35], [0.46, -0.27, -0.20]))
@configclass
class PickPlaceSceneCfg(InteractiveSceneCfg):
    """Pick-and-place scene with pick table + block + fenced place table."""

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

    # 1. Pickup Table
    pick_table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PickTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=PICK_TABLE_POS),
        spawn=sim_utils.CuboidCfg(
            size=PICK_TABLE_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.9, dynamic_friction=0.8, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.28, 0.24, 0.20)),
        ),
    )

    # 2. Block to pick up
    block = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Block",
        init_state=RigidObjectCfg.InitialStateCfg(pos=BLOCK_POS, rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.CuboidCfg(
            size=BLOCK_SIZE,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=1.0,
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.08),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.9, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.90, 0.25, 0.15)),
        ),
    )

    # 3. Placement Table
    place_table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PlaceTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=PLACE_TABLE_POS),
        spawn=sim_utils.CuboidCfg(
            size=PLACE_TABLE_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=0.9, dynamic_friction=0.8, restitution=0.0
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.18, 0.22)),
        ),
    )

    # 4. Containment Fences on Placement Table
    fence_north = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PlaceTable/FenceNorth",
        init_state=AssetBaseCfg.InitialStateCfg(pos=WALL_N_POS),
        spawn=sim_utils.CuboidCfg(
            size=WALL_N_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.75, 0.30)),
        ),
    )
    fence_south = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PlaceTable/FenceSouth",
        init_state=AssetBaseCfg.InitialStateCfg(pos=WALL_S_POS),
        spawn=sim_utils.CuboidCfg(
            size=WALL_S_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.75, 0.30)),
        ),
    )
    fence_east = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PlaceTable/FenceEast",
        init_state=AssetBaseCfg.InitialStateCfg(pos=WALL_E_POS),
        spawn=sim_utils.CuboidCfg(
            size=WALL_E_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.75, 0.30)),
        ),
    )
    fence_west = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/PlaceTable/FenceWest",
        init_state=AssetBaseCfg.InitialStateCfg(pos=WALL_W_POS),
        spawn=sim_utils.CuboidCfg(
            size=WALL_W_SIZE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.75, 0.30)),
        ),
    )
