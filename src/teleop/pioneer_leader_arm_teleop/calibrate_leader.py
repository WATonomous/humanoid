"""Store the leader arm's calibration pose: hanging straight down under gravity, gripper fully open.

That pose is joint zero for the leader, the real arm (calibrated the same way) and the sim (URDF
zero), so leader angles map one-to-one. Run once per leader, or after a servo is re-mounted:

    python calibrate_leader.py                       # writes leader_calibration.json here
    python calibrate_leader.py --out other.json

Rotation joints (shoulder rotation, forearm rotation, wrist) do not settle under gravity: line
them up with their alignment marks. Leader torque stays off throughout.
"""
import argparse
import sys
from pathlib import Path

from servo_leader import SERVO_IDS, ServoLeader, save_calibration

DEFAULT_CALIBRATION = Path(__file__).resolve().parent / "leader_calibration.json"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyACM0", help="leader serial port")
    parser.add_argument("--baud", type=int, default=1_000_000, help="serial baud rate")
    parser.add_argument("--out", type=Path, default=DEFAULT_CALIBRATION)
    args = parser.parse_args()

    try:
        leader = ServoLeader(args.port, args.baud)
    except (RuntimeError, FileNotFoundError) as exc:
        print(f"ERROR: {exc}")
        return 1
    try:
        input(
            "\nLet the leader hang straight down, gripper fully OPEN, rotation joints on their marks.\n"
            "Press Enter to store this pose as zero (Ctrl+C to abort)... "
        )
        leader.read_radians()
        positions = leader.last_positions()
        save_calibration(args.out, positions)
        print(f"Saved {args.out}: " + ", ".join(f"{label}={positions[sid]}" for label, sid in SERVO_IDS.items()))
    except KeyboardInterrupt:
        print("\nAborted; nothing written.")
        return 1
    finally:
        leader.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
