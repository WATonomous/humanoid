from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.cabinet.cabinet_env_cfg import CabinetEnvCfg


@configclass
class HumanoidArmCabinetEnvCfg(CabinetEnvCfg):
    """Cabinet open-drawer task for the humanoid arm (mu robot)."""
    pass


@configclass
class HumanoidArmCabinetEnvCfg_PLAY(HumanoidArmCabinetEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
