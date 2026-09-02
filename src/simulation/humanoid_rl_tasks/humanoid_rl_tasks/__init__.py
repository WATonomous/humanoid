"""Pure-RL training tasks (have a PPO agent config, get `rl-train`-ed).

Each task is one flat folder. Importing this package registers every task's
gym environments, the same way Isaac Lab's own task suite does.
"""

from isaaclab_tasks.utils import import_packages

# Sub-packages that are helpers, not tasks.
_BLACKLIST_PKGS = ["utils"]

import_packages(__name__, _BLACKLIST_PKGS)
