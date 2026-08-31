"""Shared InteractiveScene configs for pioneer_bimanual_arm teleop scripts.

Two scenes, selected by a ``--scene`` flag on the teleop script:

- ``BimanualBareSceneCfg``  ground + light + arm only (default; unchanged from the
                            original keyboard-teleop scene).
- ``BimanualPushSceneCfg``  the arm standing on its floor stand facing a lightbox
                            enclosure + work table, with the push-block ramp-box and
                            block on the table.

Layout notes:
- The arm stands on the ground: its stand base sits 1.1997 m below ``base_link``, so
  the articulation is lifted by that much (feet at z = 0). Same lift the Quest scene
  (``ArmV2SceneCfg``) uses. The *old* RL push env had the arm at the origin with the
  table at z = 0 -- i.e. reaching across a table at shoulder height; that is no longer
  the setup.
- Table USD + lightbox enclosure geometry are the Quest scene's (``run_quest_bimanual_teleop.py``).
- Ramp-box (``box.usd``) + block (``block.usd``) keep the push task's relative X/Y
  (``push_env_cfg.py``: ``BOX_POS`` / ``BLOCK_INIT_POS``), shifted up onto the table top.
- No RL env import -- the arm and its action space behave exactly as in the bare scene.

The Quest runner keeps its own ``ArmV2SceneCfg``; this module is wired into the
keyboard script only.
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
_HW = _SIM_DIR / "Humanoid_Wato"
_TABLE_USD = str(_HW / "Table" / "table.usd")
_BLOCK_USD = str(_HW / "UsdModelAssets" / "block.usd")
_BOX_USD = str(_HW / "UsdModelAssets" / "box.usd")

# Stand base is 1.1997 m below base_link; lift the articulation so the feet rest on
# the floor (z = 0), same as ArmV2SceneCfg.
_ROBOT_STAND_LIFT_Z = 1.1997

# ── table (Quest ArmV2SceneCfg values) ─────────────────────────────────────────
_TABLE_POS = (0.69, 0.00612, 0.33)
_TABLE_ROT = (0.5000000000000001, 0.5, 0.5, 0.49999999999999994)  # wxyz
_TABLE_SCALE = (0.0254, 0.0254, 0.0254)  # table.usd is a SolidWorks inch export
_TABLE_TOP_Z = 0.705  # measured (harness bbox of table.usd at the pose above)

# ── lightbox enclosure (Quest ArmV2SceneCfg values) ───────────────────────────
_ENCLOSURE_MATERIAL = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), emissive_color=(1.0, 1.0, 1.0))
_ENCLOSURE_X_MIN = -0.45
_ENCLOSURE_X_MAX = 0.75
_ENCLOSURE_Y_MIN = -0.9
_ENCLOSURE_Y_MAX = 1.0
_ENCLOSURE_TOP_Z = 1.8
_ENCLOSURE_Y_CTR = (_ENCLOSURE_Y_MIN + _ENCLOSURE_Y_MAX) / 2
_ENCLOSURE_Y_SPAN = _ENCLOSURE_Y_MAX - _ENCLOSURE_Y_MIN
_ENCLOSURE_X_SPAN = _ENCLOSURE_X_MAX - _ENCLOSURE_X_MIN
_ENCLOSURE_X_CTR = (_ENCLOSURE_X_MIN + _ENCLOSURE_X_MAX) / 2
# Closed backdrop on +X (the reach direction); open face at _ENCLOSURE_X_MIN, behind
# the operator.
_ENCLOSURE_BACK_X = _ENCLOSURE_X_MAX

# ── push-block ramp-box + block (push_env_cfg.py X/Y, on the table top) ────────
_BLOCK_HALF = 0.0254  # 50.8 mm cube, corner-origin USD (mdp.BLOCK_HALF_SIZE)
_BLOCK_INIT_POS = (0.21 - _BLOCK_HALF, -_BLOCK_HALF, _TABLE_TOP_Z)  # in front of the ramp mouth
_BOX_POS = (0.27, 0.127, _TABLE_TOP_Z)
_BOX_QUAT = (0.70711, 0.0, 0.0, -0.70711)  # yaw -90deg: up-the-ramp -> env +x


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


def _enclosure_wall(name: str, pos, size) -> AssetBaseCfg:
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/" + name,
        init_state=AssetBaseCfg.InitialStateCfg(pos=pos),
        spawn=sim_utils.CuboidCfg(size=size, visual_material=_ENCLOSURE_MATERIAL),
    )


@configclass
class BimanualBareSceneCfg(InteractiveSceneCfg):
    """Ground + light + the pioneer bimanual arm. Nothing else."""

    ground = _ground()
    dome_light = _dome_light()
    robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


@configclass
class BimanualPushSceneCfg(InteractiveSceneCfg):
    """Arm on its floor stand, facing a lightbox + table with the push ramp-box and block."""

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
            usd_path=_TABLE_USD,
            scale=_TABLE_SCALE,
            collision_props=sim_utils.CollisionPropertiesCfg(),
        ),
    )
    # Dynamic block to push (corner-origin USD).
    block: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Object",
        init_state=RigidObjectCfg.InitialStateCfg(pos=_BLOCK_INIT_POS, rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.UsdFileCfg(
            usd_path=_BLOCK_USD,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
        ),
    )
    # Static open box + ramp (collision baked into the USD, no RigidBodyAPI).
    box: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Box",
        init_state=AssetBaseCfg.InitialStateCfg(pos=_BOX_POS, rot=_BOX_QUAT),
        spawn=sim_utils.UsdFileCfg(usd_path=_BOX_USD),
    )

    enclosure_back: AssetBaseCfg = _enclosure_wall(
        "EnclosureBack",
        (_ENCLOSURE_BACK_X, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z / 2),
        (0.003, _ENCLOSURE_Y_SPAN, _ENCLOSURE_TOP_Z),
    )
    enclosure_left: AssetBaseCfg = _enclosure_wall(
        "EnclosureLeft",
        (_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MIN, _ENCLOSURE_TOP_Z / 2),
        (_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z),
    )
    enclosure_right: AssetBaseCfg = _enclosure_wall(
        "EnclosureRight",
        (_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MAX, _ENCLOSURE_TOP_Z / 2),
        (_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z),
    )
    enclosure_top: AssetBaseCfg = _enclosure_wall(
        "EnclosureTop",
        (_ENCLOSURE_X_CTR, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z),
        (_ENCLOSURE_X_SPAN, _ENCLOSURE_Y_SPAN, 0.003),
    )


SCENE_CFGS = {
    "bare": BimanualBareSceneCfg,
    "push": BimanualPushSceneCfg,
}
