# LeRobot simulation assets (Git LFS)

Large binaries under this tree use **Git LFS** (see repo root `.gitattributes`).

## Layout

```
assets/lerobot/
├── so101/                         # SO101 follower arm (teleop)
│   └── so101_follower_good.usd
├── so101_vial_task/usd/           # NVIDIA vial-to-rack task props
│   ├── lightbox-simple.usd
│   ├── mat.usda
│   ├── Vial_opaque.usda
│   ├── Vial_rack_simple.usda
│   └── tex/                       # vial textures
└── sync_so101_vial_assets.sh      # re-download props from GitHub if needed
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
```

## Refresh vial-task props from NVIDIA workshop

No local workshop clone needed:

```bash
./assets/lerobot/sync_so101_vial_assets.sh
git add assets/lerobot/so101_vial_task/
```

Source: [isaac-sim/Sim-to-Real-SO-101-Workshop](https://github.com/isaac-sim/Sim-to-Real-SO-101-Workshop) (Apache-2.0).

## First-time LFS migration for existing SO101 USD

If `so101_follower_good.usd` was committed before LFS:

```bash
git rm --cached assets/lerobot/so101/so101_follower_good.usd
git add assets/lerobot/so101/so101_follower_good.usd
git lfs ls-files
```

Used by `autonomy/simulation/Teleop/so101-leader teleoperation/`.
