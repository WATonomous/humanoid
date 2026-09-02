# `src/simulation/`

Isaac Lab / Isaac Sim packages: RL tasks, teleop data-collection scenes, the
training runners, and datagen glue. Everything here is `pip install -e`'d into the
`simulation_isaac` image (see `docker/simulation/isaac_lab/`).

```
src/simulation/
├── humanoid_rl/            # runners — train / play / distill / diagnose
│   └── humanoid_rl/scripts/
├── humanoid_rl_tasks/      # RL tasks, flat — one folder per task
│   └── humanoid_rl_tasks/
│       ├── inhand/         locomotion/
│       ├── pick_place/     # ManagerBasedRLEnv, driven by Isaac Lab Mimic (not PPO)
│       └── push_block/     # PPO + vision distillation; also a teleop scene
├── humanoid_scenes/        # teleop data-collection scenes — @scene-discovered
│   └── humanoid_scenes/    #   bare/  vial_rack/  push_block/
└── so101_vial_task/        # SO101 imitation-learning task
```

Robot URDF/USD/meshes and scene props live at the repo-root **`assets/`** (backend-neutral —
not tied to an Isaac Lab package): `assets/{pioneer_bimanual_arm, pioneer_hand,
whole_body_humanoid, props, lerobot}/`. RL checkpoints go to `outputs/rl/`.

## RL task vs teleop scene — where does new work go?

| you're adding… | put it in | registered by |
|---|---|---|
| an RL task (has a reward, gets PPO-trained) | `humanoid_rl_tasks/<task>/` | `import_packages` in `humanoid_rl_tasks/__init__.py` — auto |
| a teleop-only scene (collect demos, no reward) | `humanoid_scenes/<name>/scene.py` with `@scene("<name>")` | `humanoid_scenes` discovery — auto |
| a scene that is **both** (like `push_block`) | geometry + env live in `humanoid_rl_tasks/<task>/scene.py`; add a 1-line `humanoid_scenes/<name>/scene.py` that re-registers it (`from humanoid_rl_tasks.<task>.scene import ...; scene("<name>")(TheSceneCfg)`) | both, via the shim |

The scene cfg declares `robot = MISSING`; the RL env cfg and the teleop registry
each plug their own arm in. Geometry constants live in exactly one `scene.py` —
never hand-copied between the RL env and teleop.

## Invoking a scene from teleop

Teleop scripts resolve `--scene <name>` through `humanoid_scenes`:

```bash
# inside the simulation_isaac container
cd src/teleop/keyboard_teleop
isaaclab.sh -p keyboard_teleop.py --scene push        # or: bare, vial_rack, …
```

`keyboard_teleop` uses this today; `quest_isaac_teleop` and `task_space_controller`
will move onto the same `--scene` registry next.

## Training

```bash
cd $HUMANOID_ROOT     # checkpoints land in $HUMANOID_ROOT/outputs/rl/
rl-train --task=Isaac-Locomotion-Flat-PioneerHumanoid-v0 --headless
rl-play  --task=Isaac-Repose-Cube-PioneerHand-Play-v0 --num_envs=1
```

Per-task notes: `humanoid_rl_tasks/humanoid_rl_tasks/<task>/*.md`. Full walkthrough:
`docker/simulation/isaac_lab/QUICKSTART.md`.
