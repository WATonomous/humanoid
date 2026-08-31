"""Shared InteractiveScene configs for pioneer_bimanual_arm teleop scripts.

Two scenes, selected by a ``--scene`` flag on the teleop script:

- ``BimanualBareSceneCfg``     ground + light + arm only (default; unchanged from the
                               original keyboard-teleop scene).
- ``BimanualTableBoxSceneCfg`` adds a work table, a graspable box, and a drop
                               container -- same table/box/container geometry as the
                               Quest scene's ``ArmV2SceneCfg``, minus the cameras and
                               the (visual-only) lightbox walls.

The Quest runner keeps its own ``ArmV2SceneCfg`` for now; this module is only wired
into the keyboard script.
"""

from __future__ import annotations

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from pioneer_humanoid.bimanual_arm import BIMANUAL_ARM_CFG

# autonomy/simulation/pioneer_humanoid/pioneer_humanoid/teleop_scenes.py -> autonomy/simulation
_SIM_DIR = Path(__file__).resolve().parents[2]
_REPO_ROOT = _SIM_DIR.parent

# table.usd was converted from a SolidWorks STEP export (units: inches, hence the scale).
_TABLE_USD_PATH = str(_SIM_DIR / "Humanoid_Wato" / "Table" / "table.usd")
_TABLE_SCALE = (0.0254, 0.0254, 0.0254)
_TABLE_POS = (0.69, 0.00612, 0.33)
_TABLE_ROT = (0.5000000000000001, 0.5, 0.5, 0.49999999999999994)  # wxyz

_BOX_USD_PATH = str(_SIM_DIR / "Humanoid_Wato" / "UsdModelAssets" / "block.usd")
_BOX_POS = (0.2, 0.2, 0.70917)

_CONTAINER_USD_PATH = str(
    _REPO_ROOT / "assets" / "lerobot" / "so101_vial_task" / "usd" / "tray.usda"
)
_CONTAINER_POS = (0.3, -0.07041, 0.70917)
_CONTAINER_ROT = (0.7071067811865476, 0.0, 0.0, 0.7071067811865475)  # wxyz

# The stand's bottom sits 1.1997m below base_link; lift by that so the feet rest on the
# floor (z=0) instead of the collision-safety ground plane at z=-1.05.
_ROBOT_STAND_LIFT_Z = 1.1997


def _ground() -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )


def _dome_light() -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="/World/Light",
        spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
    )


@configclass
class BimanualBareSceneCfg(InteractiveSceneCfg):
    """Ground + light + the pioneer bimanual arm. Nothing else."""

    ground = _ground()
    dome_light = _dome_light()
    robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class BimanualTableBoxSceneCfg(InteractiveSceneCfg):
    """Bare scene + a work table, a graspable box, and a drop container."""

    ground = _ground()
    dome_light = _dome_light()
    robot = BIMANUAL_ARM_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=BIMANUAL_ARM_CFG.init_state.replace(pos=(0.0, 0.0, _ROBOT_STAND_LIFT_Z)),
    )

    table: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=_TABLE_POS, rot=_TABLE_ROT),
        spawn=sim_utils.UsdFileCfg(
            usd_path=_TABLE_USD_PATH,
            scale=_TABLE_SCALE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )
    # Graspable box -- dynamic rigid body.
    box: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Box",
        init_state=RigidObjectCfg.InitialStateCfg(pos=_BOX_POS),
        spawn=sim_utils.UsdFileCfg(
            usd_path=_BOX_USD_PATH,
            scale=(0.9, 0.9, 0.9),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.05),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )
    # Container -- kinematic (just needs to collide with the box, not get knocked around).
    container: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Container",
        init_state=RigidObjectCfg.InitialStateCfg(pos=_CONTAINER_POS, rot=_CONTAINER_ROT),
        spawn=sim_utils.UsdFileCfg(
            usd_path=_CONTAINER_USD_PATH,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )


SCENE_CFGS = {
    "bare": BimanualBareSceneCfg,
    "box": BimanualTableBoxSceneCfg,
}
