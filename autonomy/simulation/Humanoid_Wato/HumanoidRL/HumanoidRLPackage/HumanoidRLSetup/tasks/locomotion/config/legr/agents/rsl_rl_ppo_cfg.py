from isaaclab.utils import configclass

from HumanoidRLPackage.HumanoidRLSetup.tasks.locomotion.config.g1.agents.rsl_rl_ppo_cfg import (
    G1FlatPPORunnerCfg,
    G1RoughPPORunnerCfg,
)


@configclass
class LEGRRoughPPORunnerCfg(G1RoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.experiment_name = "legr_rough"


@configclass
class LEGRFlatPPORunnerCfg(G1FlatPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.experiment_name = "legr_flat"
        self.policy.init_noise_std = 0.8
        self.algorithm.learning_rate = 1.0e-3
        self.algorithm.desired_kl = 0.01
        self.algorithm.entropy_coef = 0.006
