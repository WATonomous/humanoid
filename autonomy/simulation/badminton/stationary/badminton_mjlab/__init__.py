"""mjlab (MuJoCo Warp) training tasks for the badminton receive env.

Importing this package registers two tasks:

  Mjlab-Badminton-Receive-Teacher   PPO on privileged observations (true
                                    shuttle state + true trajectory prior)
  Mjlab-Badminton-Receive-Student   distills the trained teacher into the
                                    student observation set (EKF-tracked
                                    shuttle + noisy trajectory prior)

Run from the stationary/ directory (the package imports aero/launcher/
perception_torch from there):

  uv run scripts/train_rl.py Mjlab-Badminton-Receive-Teacher \
      --env.scene.num-envs 4096
  uv run scripts/play_rl.py Mjlab-Badminton-Receive-Teacher --viewer viser
"""

import os
import sys

# aero/launcher/perception_torch live in stationary/, one level up.
_STATIONARY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _STATIONARY not in sys.path:
    sys.path.insert(0, _STATIONARY)

from mjlab.tasks.registry import register_mjlab_task
from rsl_rl.runners import DistillationRunner

from badminton_mjlab.env_cfg import make_env_cfg
from badminton_mjlab.rl_cfg import make_distill_cfg, make_teacher_ppo_cfg

register_mjlab_task(
    "Mjlab-Badminton-Receive-Teacher",
    env_cfg=make_env_cfg(),
    play_env_cfg=make_env_cfg(play=True),
    rl_cfg=make_teacher_ppo_cfg(),
)

register_mjlab_task(
    "Mjlab-Badminton-Receive-Student",
    env_cfg=make_env_cfg(),
    play_env_cfg=make_env_cfg(play=True),
    rl_cfg=make_distill_cfg(),
    runner_cls=DistillationRunner,
)
