"""mjlab training entry point: registers the badminton tasks, then hands
off to mjlab's train CLI (tyro; task id is the first positional argument).

  uv run scripts/train_rl.py Mjlab-Badminton-Receive-Teacher \
      --env.scene.num-envs 4096
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import badminton_mjlab  # noqa: F401  (registers the tasks)
from mjlab.scripts.train import main

if __name__ == "__main__":
    main()
