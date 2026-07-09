# LeRobot simulation assets

## Layout

```
assets/lerobot/
├── so101/
│   ├── so101_follower_good.usd    # default teleop (--robot follower)
│   └── so101_arm_camera.usd       # workshop arm w/ visible gripper cam mesh (--robot arm_camera)
├── so101_vial_task/usd/
│   ├── lightbox-simple.usd
│   ├── mat.usda
│   ├── tray.usda
│   ├── Vial_opaque.usda
│   ├── Vial_rack_simple.usda
│   └── tex/
├── so101_vial_task/hdri/          # required for --domain_rand (23 .exr + yaw_mapping.yaml)
└── sync_so101_vial_assets.sh
```

## Refresh vial-task props from NVIDIA workshop

No local workshop clone needed:

```bash
./assets/lerobot/sync_so101_vial_assets.sh          # vial props + tray + textures
./assets/lerobot/sync_so101_vial_assets.sh --full   # + arm_camera.usd + full HDRI (for --domain_rand)
git add assets/lerobot/
```

Source: [isaac-sim/Sim-to-Real-SO-101-Workshop](https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop) (Apache-2.0).

Used by:
- `simulation_isaac` watod Docker — [`docker/simulation/isaac_lab/QUICKSTART.md`](../../docker/simulation/isaac_lab/QUICKSTART.md)
- `autonomy/simulation/so101_vial_task/`
- `autonomy/simulation/Teleop/so101-leader teleoperation/`

Run `./assets/lerobot/sync_so101_vial_assets.sh --full` before first sim IL eval if you see `[WARNING] No textures found`.
