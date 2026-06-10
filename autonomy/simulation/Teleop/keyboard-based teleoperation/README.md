# Task Space Teleoperation — Setup & Run Guide

## Prerequisites

This script requires **Isaac Lab** to be installed and accessible at `<path>/IsaacLab`, with the `env_isaaclab` conda environment activated.

---

## Step 1: Fix Warp Version

Isaac Sim requires `warp-lang==1.5.0`. Run this once to install the correct version:

```bash
<path>/IsaacLab/isaaclab.sh -p -m pip install warp-lang==1.5.0
```

> **Why?** Isaac Sim bundles `omni.warp 1.5.0`, and newer versions of `warp-lang` removed APIs that Isaac Sim depends on (`warp.types.array`, `warp.context`). This will cause startup failures if not pinned correctly.

---

## Step 2: Run teleop

From this directory (`keyboard-based teleoperation/`):

```bash
PYTHONPATH=$(pwd) <path>/IsaacLab/isaaclab.sh -p keyboard_teleop.py
```

---

## Step 3 (optional): Record demonstrations

Install the shared IL recorder:

```bash
pip install -e ../../../il[record]
```

Record to LeRobot + HDF5 (same contract as real-arm `humanoid-record`):

```bash
PYTHONPATH=$(pwd) <path>/IsaacLab/isaaclab.sh -p keyboard_teleop.py --record \
  --sink lerobot,hdf5 \
  --num_episodes 5 \
  --task_description "sim reach demo"
```

**Recording keys** (pynput, same as real robot):

| Key | Effect |
|-----|--------|
| S | Start logging this episode |
| N | Save episode |
| D | Discard and re-record |
| Esc | Stop and finalize |

Output: `datasets/record_sim/001/` (LeRobot tree + `trajectories.h5`).

---

## Notes

- Step 1 only needs to be run **once** (or after an Isaac Sim / Isaac Lab reinstall).
- If you see errors like `AttributeError: module 'warp.types' has no attribute 'array'`, repeat Step 1.
- Recording uses `autonomy/il` — not the legacy `IL_test.py` prototype.
