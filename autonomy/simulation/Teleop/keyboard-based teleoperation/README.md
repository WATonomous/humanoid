# Task Space Teleoperation — Setup & Run Guide

## Prerequisites

This script requires **Isaac Lab** to be installed and accessible at `/workspace/isaaclab`.

---

## Step 1: Fix Warp Version

Isaac Sim requires `warp-lang==1.5.0`. Run this once to install the correct version:

```bash
/workspace/isaaclab/_isaac_sim/kit/python/bin/python3 -m pip install warp-lang==1.5.0
```

> **Why?** Isaac Sim bundles `omni.warp 1.5.0`, and newer versions of `warp-lang` removed APIs that Isaac Sim depends on (`warp.types.array`, `warp.context`). This will cause startup failures if not pinned correctly.

---

## Step 2: Run the Script

From the `/workspace/isaaclab` directory, run:

```bash
cd /workspace/isaaclab
./isaaclab.sh -p "final_repo/humanoid/autonomy/simulation/Teleop/keyboard-based teleoperation/task_space_test.py"
```

---

## Notes

- Always run from `/workspace/isaaclab` — the `./isaaclab.sh` script must be invoked from that directory.
- Step 1 only needs to be run **once** per container session (or after a container rebuild).
- If you see errors like `AttributeError: module 'warp.types' has no attribute 'array'`, repeat Step 1.
