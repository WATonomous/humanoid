<<<<<<< HEAD
# LeRobot simulation assets
=======
# LeRobot simulation assets (Git LFS)

Large binaries under this tree use **Git LFS** (see repo root `.gitattributes`).
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)

## Layout

```
assets/lerobot/
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
├── so101/
│   ├── so101_follower_good.usd    # default teleop (--robot follower)
│   └── so101_arm_camera.usd       # workshop arm w/ visible gripper cam mesh (--robot arm_camera)
├── so101_vial_task/usd/
<<<<<<< HEAD
│   ├── lightbox-simple.usd
│   ├── mat.usda
│   ├── tray.usda
│   ├── Vial_opaque.usda
│   ├── Vial_rack_simple.usda
│   └── tex/
├── so101_vial_task/hdri/          # required for --domain_rand (23 .exr + yaw_mapping.yaml)
└── sync_so101_vial_assets.sh
=======
├── so101/                         # SO101 follower arm (teleop)
│   └── so101_follower_good.usd
├── so101_vial_task/usd/           # NVIDIA vial-to-rack task props
=======
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
│   ├── lightbox-simple.usd
│   ├── mat.usda
│   ├── tray.usda
│   ├── Vial_opaque.usda
│   ├── Vial_rack_simple.usda
│   └── tex/
├── so101_vial_task/hdri/          # required for --domain_rand (23 .exr + yaw_mapping.yaml)
└── sync_so101_vial_assets.sh
```

## Git LFS setup (once per machine)

```bash
sudo apt install git-lfs    # or: brew install git-lfs
# or use ~/.local/bin/git-lfs if installed manually
export PATH="$HOME/.local/bin:$PATH"

cd /path/to/humanoid
git lfs install
```

After clone, teammates get assets with:

```bash
git lfs pull
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
```

## Refresh vial-task props from NVIDIA workshop

No local workshop clone needed:

```bash
<<<<<<< HEAD
<<<<<<< HEAD
./assets/lerobot/sync_so101_vial_assets.sh          # vial props + tray + textures
./assets/lerobot/sync_so101_vial_assets.sh --full   # + arm_camera.usd + full HDRI (for --domain_rand)
git add assets/lerobot/
=======
./assets/lerobot/sync_so101_vial_assets.sh
git add assets/lerobot/so101_vial_task/
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
=======
./assets/lerobot/sync_so101_vial_assets.sh          # vial props + tray + textures
./assets/lerobot/sync_so101_vial_assets.sh --full   # + arm_camera.usd + full HDRI (for --domain_rand)
git add assets/lerobot/
>>>>>>> ce24efc2 (add-to-IL-pipeline-and-add-16dof-hand)
```

Source: [isaac-sim/Sim-to-Real-SO-101-Workshop](https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop) (Apache-2.0).

<<<<<<< HEAD
Used by:
- `simulation_isaac` watod Docker — [`docker/simulation/isaac_lab/QUICKSTART.md`](../../docker/simulation/isaac_lab/QUICKSTART.md)
- `autonomy/simulation/so101_vial_task/`
- `autonomy/simulation/Teleop/so101_leader_teleoperation/`

Run `./assets/lerobot/sync_so101_vial_assets.sh --full` before first sim IL eval if you see `[WARNING] No textures found`.
=======
## First-time LFS migration for existing SO101 USD

If `so101_follower_good.usd` was committed before LFS:

```bash
git rm --cached assets/lerobot/so101/so101_follower_good.usd
git add assets/lerobot/so101/so101_follower_good.usd
git lfs ls-files
```

Used by `autonomy/simulation/Teleop/so101-leader teleoperation/`.
>>>>>>> 476bbbcc (Add SO101 vial-task assets under Git LFS.)
