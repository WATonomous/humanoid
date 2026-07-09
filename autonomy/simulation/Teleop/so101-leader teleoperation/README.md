# SO101 teleoperation (sim)

Teleoperate the **SO101 follower** in Isaac Sim and optionally record with **`autonomy/il`**.

| Script | Input | Hardware needed |
|--------|-------|-----------------|
| `so101_keyboard_teleop.py` | Keyboard + differential IK | None |
| `so101_leader_teleop.py` | Physical SO101 Leader (USB) | Leader arm + `/dev/ttyACM0` |

Shared config: `so101_cfg.py`. Vial props: `vial_task_assets.py` → `assets/lerobot/so101_vial_task/`.
Dataset schemas:
- `autonomy/il/config/dataset_schema_so101_sim.yaml` — joint-only
- `autonomy/il/config/dataset_schema_so101_sim_vision.yaml` — joints + ego + external D455 cameras

By default **`--scene vial`** loads lightbox, mat, tray, three vials, and rack.

**Robot USD (`--robot`):**
| Value | USD | Ego camera |
|-------|-----|------------|
| `follower` (default) | `so101_follower_good.usd` | Pinhole spawned on gripper link |
| `arm_camera` | `so101_arm_camera.usd` (workshop) | Uses embedded `gripper_cam` mesh |

**Flags:**
- **`--cameras`** — ego + external D455 (480×640 RGB)
- **`--domain_rand`** — full VialsToRackDR (**requires HDRI pack** — run `sync_so101_vial_assets.sh --full`)

```bash
./assets/lerobot/sync_so101_vial_assets.sh --full   # tray + arm_camera + 23 HDRI + vial props
```

IL helpers: `humanoid_il/so101_cameras.py`, `humanoid_il/so101_domain_rand.py`.
Scene configs: `SO101VialTaskDRSceneCfg`, `SO101VialTaskDRVisionSceneCfg` in `so101_cfg.py`.

## Prerequisites

- Isaac Lab (`/home/hy/IsaacLab/isaaclab.sh`)
- `pip install -e ../../../il[record]`
- **Vial scene:** assets under `assets/lerobot/` (run `./assets/lerobot/sync_so101_vial_assets.sh` if missing)
- **Leader path only:** SO101 Leader USB, `sudo chmod 666 /dev/ttyACM0`

## Keyboard teleop

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_keyboard_teleop.py
# robot only:
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_keyboard_teleop.py --scene empty
```

Record:

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_keyboard_teleop.py --record \
  --sink lerobot,hdf5 --num_episodes 5 --task_description "so101 keyboard demo"
```

Viewport: **W/A/S/D/Q/E** move, **K** gripper, **R** reset. Recording: **S / N / D / Esc**.

## Leader teleop (physical arm)

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_leader_teleop.py --port /dev/ttyACM0
```

Record:

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_leader_teleop.py --record \
  --sink lerobot,hdf5 --num_episodes 5 --port /dev/ttyACM0 \
  --task_description "pick and place demo"
```

Output (joint-only): `datasets/record_so101_sim/001/`.

## Vision + domain randomization (workshop-style)

Record with cameras and per-episode randomization:

```bash
PYTHONPATH=$(pwd) /home/hy/IsaacLab/isaaclab.sh -p so101_leader_teleop.py \
  --record --cameras --domain_rand \
  --schema ../../../il/config/dataset_schema_so101_sim_vision.yaml \
  --sink lerobot,hdf5 --num_episodes 10 --port /dev/ttyACM0 \
  --task_description "vial to rack"
```

Output: `datasets/record_so101_sim_vision/001/` with `observation.images.ego` and `observation.images.external_D455`.

Optional HDRI sky randomization: copy workshop `assets/hdri/` to `assets/lerobot/so101_vial_task/hdri/`.

## Gym stack (eval, success, rich record)

For workshop-parity **Gym envs**, automatic success detection, policy eval, and depth/seg MP4 sidecars, use:

**[`docker/simulation/isaac_lab/QUICKSTART.md`](../../../../docker/simulation/isaac_lab/QUICKSTART.md)** — watod `simulation_isaac` Docker (recommended).

Task reference: `autonomy/simulation/so101_vial_task/README.md` (`lerobot_agent.py`, `lerobot_eval.py`).

The InteractiveScene teleop above stays the lightweight RGB collection path via `humanoid_il`.

## Train (LeRobot, external)

```bash
lerobot-train \
  --dataset.repo_id=humanoid/local_so101_sim \
  --dataset.root=datasets/record_so101_sim/001 \
  --policy.type=act \
  --output_dir=outputs/train/so101_act_v1
```

## Data contract

| Field | Source | Units |
|-------|--------|-------|
| `observation.state` | sim joint positions | SO101 leader raw |
| `action` | commanded targets | same |
| `observation.images.ego` | gripper camera (with `--cameras`) | RGB 480×640 |
| `observation.images.external_D455` | lightbox D455 (with `--cameras`) | RGB 480×640 |
| `task` | `--task_description` | string |

Mapping: `autonomy/il/humanoid_il/so101_sim.py`. Cameras: `so101_cameras.py`. Domain rand: `so101_domain_rand.py`.

## Troubleshooting

- **Leader not found**: `ls /dev/ttyACM*`, fix permissions.
- **Keyboard does nothing**: click the 3D viewport first.
- **`ImportError: lerobot`**: leader teleop only; keyboard recording needs `humanoid-il[record]`.
