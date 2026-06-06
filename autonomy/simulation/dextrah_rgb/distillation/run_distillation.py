"""Stereo student-teacher distillation for Dextrah (RealSense / dual-RGB)."""

import argparse
import os
import pathlib
import sys
from datetime import datetime

import gymnasium as gym
import torch.distributed as dist
from isaaclab.app import AppLauncher
from isaaclab_tasks.utils.hydra import hydra_task_config
from rl_games.algos_torch import model_builder

import dextrah_rgb.tasks.dextrah_kuka_allegro  # noqa: F401
from dextrah_rgb.distillation.a2c_with_aux_transformer_stereo import A2CBuilder as A2CWithAuxTransformerStereoBuilder
from distillation import Dagger

parser = argparse.ArgumentParser(description="Distill a stereo RGB student from a privileged teacher.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--distributed", action="store_true", default=False, help="Run training with multiple GPUs or nodes."
)
parser.add_argument("--teacher", type=str, default=None, help="Teacher checkpoint filename under pretrained_ckpts/")
parser.add_argument("--student_ckpt", type=str, default=None, help="Optional student checkpoint to resume from.")

AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
if args_cli.video:
    args_cli.enable_cameras = True

sys.argv = [sys.argv[0]] + hydra_args
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


@hydra_task_config(args_cli.task, "rl_games_cfg_entry_point")
def main(env_cfg, agent_cfg: dict):
    """Run stereo transformer distillation."""
    world_size = int(os.environ["WORLD_SIZE"])
    rank = int(os.environ["RANK"])
    dist.init_process_group("nccl", rank=rank, world_size=world_size)

    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    if args_cli.distributed:
        agent_cfg["params"]["seed"] += app_launcher.global_rank
        agent_cfg["params"]["config"]["device"] = f"cuda:{app_launcher.local_rank}"
        agent_cfg["params"]["config"]["device_name"] = f"cuda:{app_launcher.local_rank}"
        agent_cfg["params"]["config"]["multi_gpu"] = True
        env_cfg.sim.device = f"cuda:{app_launcher.local_rank}"

    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    parent_path = str(pathlib.Path(__file__).parent.parent.resolve())
    agent_cfg_folder = "tasks/dextrah_kuka_allegro/agents"

    student_cfg = os.path.join(parent_path, agent_cfg_folder, "rl_games_ppo_transformer_stereo.yaml")
    teacher_cfg = os.path.join(parent_path, agent_cfg_folder, "rl_games_ppo_lstm_cfg.yaml")

    if args_cli.teacher is not None:
        teacher_ckpt = os.path.join(parent_path, "pretrained_ckpts", args_cli.teacher)
    else:
        teacher_ckpt = os.path.join(parent_path, "pretrained_ckpts", "new_teacher.pth")

    student_ckpt = None
    if args_cli.student_ckpt is not None:
        student_ckpt = args_cli.student_ckpt
        if not os.path.isabs(student_ckpt):
            student_ckpt = os.path.join(parent_path, student_ckpt)

    if rank == 0:
        train_dir = "runs"
        experiment_name = "Dextrah-Kuka-Allegro" + datetime.now().strftime("_%d-%H-%M-%S")
        experiment_dir = os.path.join(train_dir, experiment_name)
        nn_dir = os.path.join(experiment_dir, "nn")
        summaries_dir = os.path.join(experiment_dir, "summaries")
        os.makedirs(train_dir, exist_ok=True)
        os.makedirs(experiment_dir, exist_ok=True)
        os.makedirs(nn_dir, exist_ok=True)
        os.makedirs(summaries_dir, exist_ok=True)
    else:
        summaries_dir = None
        nn_dir = None

    dagger_config = {
        "student": {
            "cfg": student_cfg,
            "ckpt": student_ckpt,
            "obs_type": "policy",
        },
        "teacher": {
            "cfg": teacher_cfg,
            "ckpt": teacher_ckpt,
            "obs_type": "expert_policy",
        },
    }

    model_builder.register_network("a2c_aux_transformer_stereo", A2CWithAuxTransformerStereoBuilder)

    dagger = Dagger(env, dagger_config, summaries_dir=summaries_dir, nn_dir=nn_dir)
    dagger.distill()
    if rank == 0:
        dagger.save("dextrah_student_final")


if __name__ == "__main__":
    main()
    simulation_app.close()
