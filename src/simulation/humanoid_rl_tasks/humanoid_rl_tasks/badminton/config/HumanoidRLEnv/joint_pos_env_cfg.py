from isaaclab.utils import configclass

from humanoid_rl_tasks.badminton.badminton_env_cfg import BadmintonEnvCfg


@configclass
class HumanoidArmBadmintonEnvCfg(BadmintonEnvCfg):
    pass


@configclass
class HumanoidArmBadmintonEnvCfg_PLAY(HumanoidArmBadmintonEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        self.observations.policy.enable_corruption = False


# PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p $RL_RUNNERS/train.py --task=Isaac-Badminton-Intercept-Humanoid-Arm-v0 --headless
#
# PYTHONPATH=$(pwd) $ISAACLAB/isaaclab.sh -p $RL_RUNNERS/play.py --task=Isaac-Badminton-Intercept-Humanoid-Arm-Play-v0 --num_envs=1
