<<<<<<< HEAD
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
=======
>>>>>>> eff69ae8 (refine-rl-and-add-rl-env)
from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.lift.lift_env_cfg import LiftEnvCfg

# assets/lerobot/so101_new_calib.urdf: gripper_frame_joint (parent gripper_link -> USD body "gripper")
_GRIPPER_FRAME_POS = (-0.0079, -0.000218121, -0.07)
# rpy="0 3.14159 0" on that joint -> quaternion (w, x, y, z)
_GRIPPER_FRAME_ROT = (0.0, 0.0, 1.0, 0.0)

<<<<<<< HEAD

def _so101_ee_frame_cfg(*, debug_vis: bool) -> FrameTransformerCfg:
    marker_cfg = FRAME_MARKER_CFG.replace(prim_path="/Visuals/FrameTransformer/ee_tcp")
    marker_cfg.markers["frame"].scale = (0.03, 0.03, 0.03)
    return FrameTransformerCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        debug_vis=debug_vis,
        visualizer_cfg=marker_cfg,
        target_frames=[
            FrameTransformerCfg.FrameCfg(
                prim_path="{ENV_REGEX_NS}/Robot/gripper",
                name="ee_tcp",
                offset=OffsetCfg(pos=_GRIPPER_FRAME_POS, rot=_GRIPPER_FRAME_ROT),
            ),
        ],
    )
=======
@configclass
class HumanoidArmLiftEnvCfg(LiftEnvCfg):
    """Lift-cube task for the humanoid arm (mu robot)."""
    pass
>>>>>>> eff69ae8 (refine-rl-and-add-rl-env)


@configclass
class SO101LiftEnvCfg(LiftEnvCfg):
    """Lift-cube task for the SO101 follower arm."""

    def __post_init__(self):
        super().__post_init__()
        # SO101 USD is not instanceable; physics replication can corrupt GPU buffers.
        self.scene.replicate_physics = False
        self.scene.ee_frame = _so101_ee_frame_cfg(debug_vis=False)


@configclass
class SO101LiftEnvCfg_PLAY(SO101LiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 1
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
        # Green cuboid = commanded object goal; RGB axes = URDF gripper_frame TCP on link "gripper".
        self.commands.object_pose.debug_vis = True
        self.scene.ee_frame = _so101_ee_frame_cfg(debug_vis=True)
