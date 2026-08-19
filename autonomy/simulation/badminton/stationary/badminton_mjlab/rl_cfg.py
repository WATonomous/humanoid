"""RSL-RL configs: teacher PPO, then teacher->student distillation.

Phase 1 (Mjlab-Badminton-Receive-Teacher): PPO where both actor and critic
see the privileged "teacher" group.

Phase 2 (Mjlab-Badminton-Receive-Student): rsl_rl DistillationRunner. Load
the trained teacher with --agent.resume True --agent.load-run <teacher-run>;
Distillation.load restores only the teacher weights by default, and the
student (on the "student" group) learns to match its actions.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from mjlab.rl.config import (RslRlBaseRunnerCfg, RslRlModelCfg,
                             RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg)

HIDDEN = (512, 256, 128)


def make_teacher_ppo_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        num_steps_per_env=24,
        max_iterations=3000,
        save_interval=100,
        experiment_name="badminton_teacher",
        obs_groups={"actor": ("teacher",), "critic": ("teacher",)},
        actor=RslRlModelCfg(
            hidden_dims=HIDDEN,
            obs_normalization=True,
            distribution_cfg={"class_name": "GaussianDistribution",
                              "init_std": 0.5, "std_type": "scalar"}),
        critic=RslRlModelCfg(hidden_dims=HIDDEN, obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(
            learning_rate=3e-4,
            entropy_coef=0.005,
            num_learning_epochs=5,
            num_mini_batches=4),
    )


@dataclass
class RslRlDistillationAlgorithmCfg:
    num_learning_epochs: int = 1
    gradient_length: int = 15
    learning_rate: float = 1e-3
    max_grad_norm: float | None = 1.0
    loss_type: str = "mse"
    optimizer: str = "adam"
    class_name: str = "Distillation"


@dataclass
class RslRlDistillationRunnerCfg(RslRlBaseRunnerCfg):
    """Runner cfg shaped for rsl_rl Distillation.construct_algorithm: model
    cfgs under "student"/"teacher", algorithm.class_name = Distillation."""

    class_name: str = "DistillationRunner"
    student: RslRlModelCfg = field(
        default_factory=lambda: RslRlModelCfg(
            hidden_dims=HIDDEN,
            obs_normalization=True,
            distribution_cfg={"class_name": "GaussianDistribution",
                              "init_std": 0.1, "std_type": "scalar"}))
    teacher: RslRlModelCfg = field(
        default_factory=lambda: RslRlModelCfg(
            hidden_dims=HIDDEN,
            obs_normalization=True,
            distribution_cfg={"class_name": "GaussianDistribution",
                              "init_std": 0.5, "std_type": "scalar"}))
    algorithm: RslRlDistillationAlgorithmCfg = field(
        default_factory=RslRlDistillationAlgorithmCfg)


def make_distill_cfg() -> RslRlDistillationRunnerCfg:
    return RslRlDistillationRunnerCfg(
        num_steps_per_env=24,
        max_iterations=1500,
        save_interval=100,
        experiment_name="badminton_student",
        resume=True,                       # teacher checkpoint required
        load_run=".*badminton_teacher.*",
        obs_groups={"student": ("student",), "teacher": ("teacher",)},
    )
