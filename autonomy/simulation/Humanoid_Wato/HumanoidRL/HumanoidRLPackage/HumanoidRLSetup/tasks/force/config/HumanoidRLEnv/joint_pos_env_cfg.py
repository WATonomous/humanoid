from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.force.force_env_cfg import ForceEnvCfg


@configclass
class HumanoidArmForceEnvCfg(ForceEnvCfg):
    pass


@configclass
class HumanoidArmForceEnvCfg_PLAY(HumanoidArmForceEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False


<<<<<<< HEAD
# PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py --task=Isaac-Force-Impulse-Humanoid-Arm-v0 --headless

# PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py --task=Isaac-Force-Impulse-Humanoid-Arm-Play-v0 --num_envs=1
=======
# PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/train.py --task=Isaac-Force-Impulse-Humanoid-Arm-v0 --headless

# PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p HumanoidRLPackage/rsl_rl_scripts/play.py --task=Isaac-Force-Impulse-Humanoid-Arm-Play-v0 --num_envs=1
>>>>>>> eff69ae8 (refine-rl-and-add-rl-env)
