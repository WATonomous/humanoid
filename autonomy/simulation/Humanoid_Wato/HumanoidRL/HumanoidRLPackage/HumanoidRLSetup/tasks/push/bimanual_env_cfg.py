"""Bimanual-arm (left arm) joint-position env configs for the push-block task.

Robot swap only: scene geometry (table, ramp-box, block, spawn curriculum,
rewards) is unchanged from ``push_env_cfg.PushBlockEnvCfg`` — only the
SO101 follower is replaced by the wato_bimanual_arm left arm, plus a small
white "lightbox" enclosure (four wall cuboids) added purely as a visual
backdrop (it is NOT a physics floor/table; the existing table/ramp/box
geometry provides all collision surfaces, same as the SO101 variant).

The enclosure walls are self-contained here (positions/sizes in the
``_ENCLOSURE_*`` constants below). They were originally derived from a
perception test-stand USD (``bimanual_arm_lightbox.usd``, since removed) and
then widened to fit this task's table/ramp-box. The walls use an emissive
white material (there is no RectLight spawner cfg in this IsaacLab checkout —
only Cylinder/Disk/Distant/Dome/Sphere, see
``isaaclab/sim/spawners/lights/lights_cfg.py``) plus the scene's DomeLight for
fill.

NOTE: the original enclosure was sized for a small perception test stand
(1.016m x 0.762m x 0.508m), not the SeattleLabTable/ramp-box geometry
inherited from the SO101 push task. The wall placement here is a first-pass
visual approximation — check for clipping against the table in-sim (e.g. via
`play.py --task Isaac-Bimanual-Push-Block-Play-v0`) and adjust the wall
positions/sizes below if the table pokes through.
"""

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import (
    FrameTransformerCfg,
    OffsetCfg,
)
from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.pick_place_bimanual.robot_cfg_shim import (
    BIMANUAL_ARM_CFG,
    LEFT_ARM_JOINTS,
    LEFT_EE_BODY,
)

from . import mdp
from .push_env_cfg import PushBlockEnvCfg, PushBlockSceneCfg

_ENCLOSURE_MATERIAL = sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 1.0), emissive_color=(1.0, 1.0, 1.0))


# Widened from the original perception test-stand bounds (back x=-0.185, sides
# y=-0.466/0.55, top z=0.762) to comfortably clear the push table/ramp-box added
# below instead of the bare stand it was originally sized for.
_ENCLOSURE_BACK_X = -0.25
_ENCLOSURE_Y_MIN = -0.9
_ENCLOSURE_Y_MAX = 1.0
_ENCLOSURE_TOP_Z = 0.9
_ENCLOSURE_FRONT_X = 0.323  # open front, unchanged
_ENCLOSURE_Y_CTR = (_ENCLOSURE_Y_MIN + _ENCLOSURE_Y_MAX) / 2
_ENCLOSURE_Y_SPAN = _ENCLOSURE_Y_MAX - _ENCLOSURE_Y_MIN
_ENCLOSURE_X_SPAN = _ENCLOSURE_FRONT_X - _ENCLOSURE_BACK_X
_ENCLOSURE_X_CTR = (_ENCLOSURE_BACK_X + _ENCLOSURE_FRONT_X) / 2

# Plain cuboid table (replaces the SO101 variant's baked-mesh SeattleLabTable,
# which clipped through the lightbox enclosure). Fills the entire enclosure
# footprint (back wall to open front, full left-right width), with a bit of
# extra front reach past the enclosure's open edge to fully support the
# ramp-box (extends to x=0.524). Top surface at z=0, matching the
# block/box/table-top assumptions elsewhere in push_env_cfg.py.
_TABLE_X_MIN, _TABLE_X_MAX = _ENCLOSURE_BACK_X, 0.6
_TABLE_Y_MIN, _TABLE_Y_MAX = _ENCLOSURE_Y_MIN, _ENCLOSURE_Y_MAX
_TABLE_THICKNESS = 0.05


@configclass
class BimanualPushBlockSceneCfg(PushBlockSceneCfg):
    """PushBlockSceneCfg + the lightbox enclosure walls (visual only, no collision)."""

    enclosure_back: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureBack",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_BACK_X, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(0.003, _ENCLOSURE_Y_SPAN, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_left: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureLeft",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MIN, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_right: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureRight",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_MAX, _ENCLOSURE_TOP_Z / 2)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, 0.003, _ENCLOSURE_TOP_Z), visual_material=_ENCLOSURE_MATERIAL),
    )
    enclosure_top: AssetBaseCfg = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/EnclosureTop",
        init_state=AssetBaseCfg.InitialStateCfg(pos=(_ENCLOSURE_X_CTR, _ENCLOSURE_Y_CTR, _ENCLOSURE_TOP_Z)),
        spawn=sim_utils.CuboidCfg(size=(_ENCLOSURE_X_SPAN, _ENCLOSURE_Y_SPAN, 0.003), visual_material=_ENCLOSURE_MATERIAL),
    )


def _bimanual_ee_frame_cfg(*, debug_vis: bool) -> FrameTransformerCfg:
    """EE proxy at the left wrist link (link6l), zero offset.

    Unlike the SO101 variant's ``_GRIPPER_FRAME_POS`` (measured from the
    URDF gripper body), no verified fingertip-center offset from link6l exists
    for the bimanual arm as a static frame (the teleop code only computes it
    dynamically from both finger tips at runtime, see
    ``bimanual_arm_cfg.compute_gripper_tip_pose_b``). Zero offset is a
    reasonable proxy for the push reward terms; refine with a measured offset
    if pushing behavior looks visibly off from the wrist position.
    """
    marker_cfg = FRAME_MARKER_CFG.replace(prim_path="/Visuals/FrameTransformer/ee_tcp")
    marker_cfg.markers["frame"].scale = (0.03, 0.03, 0.03)
    return FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base_link",
        debug_vis=debug_vis,
        visualizer_cfg=marker_cfg,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/" + LEFT_EE_BODY,
                name="end_effector",
                offset=OffsetCfg(pos=(0.0, 0.0, 0.0), rot=(1.0, 0.0, 0.0, 0.0)),
            ),
        ],
    )


@configclass
class BimanualPushBlockEnvCfg(PushBlockEnvCfg):
    scene: BimanualPushBlockSceneCfg = BimanualPushBlockSceneCfg(num_envs=1024, env_spacing=2.5)

    def __post_init__(self):
        super().__post_init__()

        self.scene.replicate_physics = True
        self.scene.robot = BIMANUAL_ARM_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.articulation_props.enabled_self_collisions = False

        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=LEFT_ARM_JOINTS,
            scale=0.5,
            use_default_offset=True,
        )

        self.scene.ee_frame = _bimanual_ee_frame_cfg(debug_vis=False)

        # Plain cuboid table instead of the inherited SeattleLabTable (see
        # _TABLE_* constants above): sized to fit inside the lightbox enclosure,
        # which the baked-mesh SeattleLabTable clipped through.
        # NOTE: no visual/texture DR here on purpose. This is a STATE-based RL
        # task (obs = joint/EE/block pose + ramp geometry; no camera), so table
        # texture/color is invisible to the policy and randomizing it would only
        # add per-reset cost with zero learning benefit. Visual DR (texture /
        # light / camera) lives in the vision distillation variant instead --
        # see BimanualPushBlockDistillEnvCfg in distill_env_cfg.py.
        table_x_ctr = (_TABLE_X_MIN + _TABLE_X_MAX) / 2
        table_y_ctr = (_TABLE_Y_MIN + _TABLE_Y_MAX) / 2
        self.scene.table = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Table",
            init_state=AssetBaseCfg.InitialStateCfg(pos=(table_x_ctr, table_y_ctr, -_TABLE_THICKNESS / 2)),
            spawn=sim_utils.CuboidCfg(
                size=(_TABLE_X_MAX - _TABLE_X_MIN, _TABLE_Y_MAX - _TABLE_Y_MIN, _TABLE_THICKNESS),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.3, 0.25)),
            ),
        )


@configclass
class BimanualPushBlockEnvCfg_PLAY(BimanualPushBlockEnvCfg):
    # GUI CRASH WORKAROUND (interactive play/inspection only; does NOT affect
    # headless training):
    #   Playing this task in a live GUI window aborts ~29 s after the scene
    #   renders with a native Kit assertion:
    #     carb::thread::detail::BaseMutex::unlock(): "unlock() called by
    #     non-owning thread" (carb/delegate Mutex.h:158)
    #   It is a cross-thread race in Kit's tasking plugin, TRIGGERED by
    #   something bimanual-arm-specific (the SO101 push variant does not crash).
    #   Ruled out so far: async rendering (--/app/asyncRendering=false did not
    #   help); the EE-frame debug marker (tested with debug_vis=False -- its
    #   "mismatched prototypes on point instancer /Visuals/FrameTransformer/
    #   ee_tcp" warning disappeared but the crash remained). The last remaining
    #   lead is a Fabric visuals-prim quirk on the arm: a one-time init warning
    #   "getAttributeCount called on non-existent path .../Robot/link8l/visuals/
    #   link8l" -- i.e. the URDF->USD gripper-finger visual prim doesn't clone
    #   cleanly across envs. Unconfirmed as the true cause, and fixing it would
    #   be surgery on the shared bimanual_arm USD with no guarantee it resolves
    #   the native Kit race. Root-cause fix is still TODO; until then, launch
    #   play.py with Kit forced to a single tasking thread, which serializes the
    #   race away (measured cost: startup ~16 s vs ~11 s, no runtime impact on a
    #   small inspection scene; NOT needed and NOT used for headless training):
    #
    #     ./isaaclab.sh -p .../rsl_rl_scripts/play.py \
    #         --task Isaac-Bimanual-Push-Block-Play-v0 --num_envs 4 \
    #         --checkpoint <run>/model_<N>.pt \
    #         "--kit_args=--/plugins/carb.tasking.plugin/threadCount=1"
    #
    #   NOTE the "=" form ("--kit_args=--/..."): with a space, argparse mistakes
    #   the "--/..." value for a new flag and errors. This is a Kit *launch* arg,
    #   so it cannot live in the env cfg -- it must be passed on the CLI.
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 16
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        self.scene.ee_frame = _bimanual_ee_frame_cfg(debug_vis=True)
