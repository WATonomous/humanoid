from isaaclab.utils import configclass
from HumanoidRLPackage.HumanoidRLSetup.tasks.manipulation.reach_env_cfg import ReachEnvCfg


@configclass
class HumanoidArmReachEnvCfg(ReachEnvCfg):
    pass


@configclass
class HumanoidArmReachEnvCfg_PLAY(HumanoidArmReachEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False  # disable randomization for play


# (env_isaaclab) hy@hy-LOQ-15IRX9:~/Downloads/Humanoid_Wato/HumanoidRL$ PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py --task=Isaac-Reach-Humanoid-Arm-v0 --headless

# (env_isaaclab) hy@hy-LOQ-15IRX9:~/Downloads/Humanoid_Wato/HumanoidRL$ PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py --task=Isaac-Reach-Humanoid-Arm-v0 --num_envs=1
