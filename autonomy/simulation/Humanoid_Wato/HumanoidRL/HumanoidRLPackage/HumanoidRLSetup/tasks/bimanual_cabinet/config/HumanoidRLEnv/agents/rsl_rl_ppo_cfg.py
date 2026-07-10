from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class BimanualArmCabinetPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 192    # doubled from 96 — longer rollouts capture full pull sequences
    max_iterations = 500
    save_interval = 50
    experiment_name = "bimanual_cabinet"
    empirical_normalization = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[256, 128, 64],
        critic_hidden_dims=[256, 128, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,      # reduced from 1.0 — critic was dominating the optimizer
                                   # when value loss is 10k× larger than surrogate loss
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.0,
        num_learning_epochs=5,    # reverted from 8 — 8 epochs caused policy collapse
        num_mini_batches=4,
        learning_rate=5.0e-4,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.016,         # tightened from 0.02 — smaller steps when critic is unstable
        max_grad_norm=1.0,
    )
