"""Leader-arm teleop of the Pioneer left arm (no IK), in Isaac Sim (default) or plain MuJoCo.

    isaaclab.sh -p pioneer_leader_arm_teleop.py --scene push [--record]
    python pioneer_leader_arm_teleop.py --sim mujoco --scene peg_insert

The backend is picked before anything simulator-specific is imported, so --sim mujoco runs
without Isaac. See isaac_sim.py / mujoco_sim.py and README.md.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

_pre = argparse.ArgumentParser(add_help=False)
_pre.add_argument("--sim", choices=("isaac", "mujoco"), default="isaac")
if _pre.parse_known_args()[0].sim == "mujoco":
    import mujoco_sim as backend
else:
    import isaac_sim as backend

if __name__ == "__main__":
    backend.run()
