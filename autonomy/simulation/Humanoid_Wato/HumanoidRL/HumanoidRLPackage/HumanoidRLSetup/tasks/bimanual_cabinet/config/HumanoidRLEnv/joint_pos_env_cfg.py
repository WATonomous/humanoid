from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.bimanual_cabinet.bimanual_cabinet_env_cfg import CabinetEnvCfg


@configclass
class BimanualArmCabinetEnvCfg(CabinetEnvCfg):
    """Cabinet open-drawer task for the bimanual arm."""
    pass


@configclass
class BimanualArmCabinetEnvCfg_PLAY(BimanualArmCabinetEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False
