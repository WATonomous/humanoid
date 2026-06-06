# Copyright (c) 2024, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""RSL-RL agent configs for Dextrah-Kuka-Allegro teacher training."""

from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticRecurrentCfg,
    RslRlPpoAlgorithmCfg,
)


@configclass
class DextrahKukaAllegroLstmPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    """Privileged teacher policy with recurrent actor-critic (RL-Games LSTM analogue)."""

    seed = 42
    device = "cuda:0"
    num_steps_per_env = 16
    max_iterations = 20000
    save_interval = 200
    experiment_name = "dextrah_lstm"
    run_name = ""
    empirical_normalization = True
    clip_actions = 1.0
    logger = "tensorboard"
    wandb_project = "dextrah-kuka-allegro"

    policy = RslRlPpoActorCriticRecurrentCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[512, 512],
        critic_hidden_dims=[1024, 512],
        activation="elu",
        rnn_type="lstm",
        rnn_hidden_dim=1024,
        rnn_num_layers=1,
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=4.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.002,
        num_learning_epochs=4,
        num_mini_batches=4,
        learning_rate=3.0e-4,
        schedule="adaptive",
        gamma=0.998,
        lam=0.95,
        desired_kl=0.013,
        max_grad_norm=1.0,
    )
