from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.lift.lift_env_cfg import LiftEnvCfg


@configclass
class HumanoidArmLiftEnvCfg(LiftEnvCfg):
    """Lift-cube task for the humanoid arm (mu robot)."""
    pass


@configclass
class HumanoidArmLiftEnvCfg_PLAY(HumanoidArmLiftEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
