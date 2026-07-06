from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class BimanualArmCabinetPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 96
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
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.0,          # Zero — let the policy sharpen and hold a grip
        num_learning_epochs=5,
        num_mini_batches=4,
        learning_rate=5.0e-4,      # Restored to original — was strangling learning at 1e-4
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.02,           # Loosened from 0.01 — allows larger policy updates
        max_grad_norm=1.0,         # Restored from 0.5 — allows full gradient steps
    )
