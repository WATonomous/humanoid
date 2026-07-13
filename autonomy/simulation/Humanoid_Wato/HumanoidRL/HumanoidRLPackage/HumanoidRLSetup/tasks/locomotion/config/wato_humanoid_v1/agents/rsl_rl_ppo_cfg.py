from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class WatoHumanoidRoughPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 3000
    save_interval = 50
    experiment_name = "wato_humanoid_rough"
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.008,
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=1.0e-3,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )


@configclass
class WatoHumanoidFlatPPORunnerCfg(WatoHumanoidRoughPPORunnerCfg):
    def __post_init__(self):
        super().__post_init__()

        self.max_iterations = 10000
        self.experiment_name = "wato_humanoid_flat"
        # Raised from 0.5 to match G1's flat config: G1FlatPPORunnerCfg never overrides
        # init_noise_std, so it inherits G1RoughPPORunnerCfg's 1.0 -- Wato's flat config
        # had been sitting at half that (0.5) since before this session, unexamined.
        # (Separately: cutting it to 0.1, with entropy_coef also cut, made no difference
        # either -- episode length stayed ~35-36 through iteration 150 in that test,
        # ruling out low noise as a destabilizer. This is a different test: raising
        # toward G1's actual value, not lowering further.)
        self.policy.init_noise_std = 1.0
        self.policy.actor_hidden_dims = [256, 128, 128]
        self.policy.critic_hidden_dims = [256, 128, 128]
        self.algorithm.learning_rate = 1.0e-3
        self.algorithm.desired_kl = 0.01
        # 0.004 was too conservative: stepping_fix_fresh_004 plateaued hard at episode
        # length ~52 for 250+ iterations (noise std already annealed down to 0.22 by
        # iteration 750, likely too little exploration left to ever discover the
        # "don't fall" behavior the original entropy_coef=0.01 run stumbled onto).
        # The actual divergence cause was unnormalized value targets over long
        # episodes, not entropy per se -- empirical_normalization=True below should
        # fix that directly, so raising entropy back partway is safe to retest.
        self.algorithm.entropy_coef = 0.008
        # Episode length jumped from ~60 to ~1000 steps once ankle stiffness fixed the
        # fall-within-1s failure mode. Without observation/return normalization, value
        # function regression targets over a ~40x longer horizon likely blew up --
        # this was previously False, which is unusual and risky for long-horizon tasks.
        self.empirical_normalization = True
