# humanoid_scenes

Manipulation scenes for **pioneer bimanual-arm teleop data collection** — one
folder per scene, auto-discovered. Adding a scene requires touching *only* this
package.

## Add a scene

```
humanoid_scenes/my_scene/
├── __init__.py          # empty
└── scene.py
```

```python
# humanoid_scenes/my_scene/scene.py
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from dataclasses import MISSING

from humanoid_scenes import scene


@scene("my_scene", robot_pos=(0.0, 0.0, 0.0), camera=([1.5, -1.5, 0.5], [0.4, 0.0, 0.0]))
@configclass
class MySceneCfg(InteractiveSceneCfg):
    robot = MISSING        # filled in by the teleop script's arm
    ee_frame = MISSING     # set to None for teleop
    # ... ground, light, table, objects ...
```

- **`robot_pos`** — arm base placement. Low table (arm reaches down from origin):
  `(0, 0, 0)`. Arm on its floor stand: `~(0, 0, 1.2)`.
- **`camera`** — optional `(eye, target)` for the teleop initial view.

That's it. `keyboard_teleop --scene my_scene` now works — no edits to
`keyboard_teleop`, or the Dockerfile.

## Use a scene

```
./watod -t simulation_isaac
# in the container:
keyboard_teleop --scene <name>      # pass an unknown name to list them
```

## What lives here vs. not

- **Here:** lightweight teleop/data-collection scenes — mostly a `scene.py`.
- **Not here:** full task packages with training/eval/recording infra
  (`so101_vial_task`) and the RL tasks (`humanoid_rl_tasks/` — `inhand`,
  `locomotion`, `pick_place`, `push_block`). A scene that doubles as an
  RL task keeps its geometry in `humanoid_rl_tasks/<task>/scene.py`; the folder
  here is just a one-liner that registers it for teleop (see `push_block/`).

## Scenes

| name | notes |
|------|-------|
| `vial_rack` | Low table + `Vial_rack_simple.usda` + 3 loose `Vial_opaque.usda`. Rack/vial xy is a first pass — reach-tune against the LEFT arm while driving it. Assets: `assets/lerobot/so101_vial_task/usd/`. |
