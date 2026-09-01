# pioneer_vial_task

Vial-rack manipulation scene for the **pioneer bimanual arm** — table + rack + 3
loose vials — built for teleop data collection.

Teleop-first: the package is just `humanoid_pioneer_vial/scene.py`
(`VialRackSceneCfg`). No RL/IL env cfg or gym registration yet — add those when a
policy is trained on it.

## Use it

```
./watod -t simulation_isaac
# in the container:
keyboard_teleop --scene vial_rack
```

`SCENE_CFGS["vial_rack"]` in `pioneer_humanoid.teleop_scenes` imports
`VialRackSceneCfg` and drops in the arm (LEFT / L-suffix chain, driven by
keyboard_teleop).

## Geometry

Grounding matches `tools/isaac_harness/scenes/bimanual_vial_rack.sh` — arm at the
origin (no floor-stand lift), low table (top at `TABLE_TOP_Z = -0.25`) so the
rack/vials sit in reach. Assets: `assets/lerobot/so101_vial_task/usd/`.

Rack / vial XY is a first pass — reach-tune it against the arm that drives it.
