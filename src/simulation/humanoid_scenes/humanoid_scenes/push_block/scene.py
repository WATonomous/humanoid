"""Legacy shim: the RL push-block scene.

push still lives 4 dirs deep in HumanoidRLPackage and isn't an installable
package, so its scene is pulled in via sys.path and re-registered here. When
push is flattened into this folder (issue #245), replace this shim with the
real scene.py + rl/ subdir.
"""
from __future__ import annotations

import sys
from pathlib import Path

from humanoid_scenes import scene

_RL_TASKS = (
    Path(__file__).resolve().parents[3]  # -> src/simulation/
    / "Humanoid_Wato" / "HumanoidRL" / "HumanoidRLPackage" / "HumanoidRLSetup" / "tasks"
)
if str(_RL_TASKS) not in sys.path:
    sys.path.insert(0, str(_RL_TASKS))

from push.scene import PushBlockSceneCfg, ROBOT_STAND_LIFT_Z  # noqa: E402

# Register the imported cfg under "push" — arm lifted onto its floor stand.
scene(
    "push",
    robot_pos=(0.0, 0.0, ROBOT_STAND_LIFT_Z),
    camera=([1.4, -1.0, 0.9], [0.35, -0.2, 0.05]),
)(PushBlockSceneCfg)
