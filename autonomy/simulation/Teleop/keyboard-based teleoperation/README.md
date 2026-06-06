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

## Step 2: Run the Script

From this directory (`keyboard-based teleoperation/`), run:

```bash
PYTHONPATH=$(pwd) <path>/IsaacLab/isaaclab.sh -p task_space_test.py
```

---

## Notes

- Run from this directory so `demonstrations/` and `recordings/` are created here.
- Step 1 only needs to be run **once** (or after an Isaac Sim / Isaac Lab reinstall).
- If you see errors like `AttributeError: module 'warp.types' has no attribute 'array'`, repeat Step 1.
