"""Leader angles -> left-arm joint targets + gripper closure, shared by the Isaac and MuJoCo backends.

Plain Python: no simulator imports. The leader is an input device with torque always off.
"""
from __future__ import annotations

import argparse
import math
import time

from arm_limits import gripper_fraction, limited_target
from servo_leader import ARM_SERVOS, GRIPPER_SERVO, SERVO_IDS, ServoLeader, parse_signs

# Per-servo direction, A..F then G. Flip one with --signs if a joint moves the wrong way.
DEFAULT_SIGNS = "1,-1,-1,1,1,-1,1"
# Leader-to-sim gain per arm servo A..F (sim change = leader change x scale), times --scale.
JOINT_SCALES = (0.7, 0.7, 0.7, 1.0, 1.0, 1.0)
# Stock wrist damping (18) caps joint6l near 2 deg/s at the GL40's 0.73 Nm, so the sim wrist
# lags the leader. Each backend lowers it for this teleop only; the shared arm config keeps 18.
WRIST_DAMPING = 2.5
# Control rate: one leader read and target update per 10 ms of sim time.
CONTROL_DT = 0.01


def add_leader_args(parser: argparse.ArgumentParser, *, scene_help: str) -> None:
    parser.add_argument("--target", choices=("isaac", "mujoco"), default="isaac", help="what the leader drives")
    parser.add_argument("--scene", type=str, default="bare", help=scene_help)
    parser.add_argument("--port", default="/dev/ttyACM0", help="leader serial port")
    parser.add_argument("--baud", type=int, default=1_000_000, help="leader serial baud rate")
    parser.add_argument("--signs", default=DEFAULT_SIGNS, help=f"direction per servo A..G (default: {DEFAULT_SIGNS})")
    parser.add_argument("--scale", type=float, default=1.0, help="overall gain, multiplied into JOINT_SCALES")
    parser.add_argument(
        "--filter-alpha",
        type=float,
        default=0.35,
        help="target low-pass coefficient in (0,1]; 1 disables filtering",
    )


def check_leader_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if not 0.0 < args.filter_alpha <= 1.0:
        parser.error("--filter-alpha must be in (0, 1]")
    if not (math.isfinite(args.scale) and args.scale > 0.0):
        parser.error("--scale must be finite and > 0")
    try:
        parse_signs(args.signs)
    except ValueError as exc:
        parser.error(f"--signs: {exc}")


class LeaderMapping:
    """Filtered, URDF-clamped arm targets (rad) and gripper closure (0 open .. 1 closed)."""

    def __init__(self, args: argparse.Namespace, home_rad: list[float], limits_rad: list[tuple[float, float]]):
        if len(home_rad) != len(ARM_SERVOS):
            raise ValueError(f"leader drives {len(ARM_SERVOS)} joints but the arm has {len(home_rad)}")
        self.signs = parse_signs(args.signs)
        self.scales = [args.scale * s for s in JOINT_SCALES]
        self.alpha = args.filter_alpha
        # Leader zero maps to the arm's home pose (elbow bent).
        self.home = list(home_rad)
        self.limits_deg = [(math.degrees(lo), math.degrees(hi)) for lo, hi in limits_rad]
        self.gripper_axis = tuple(SERVO_IDS).index(GRIPPER_SERVO)
        self.reset()

    def reset(self) -> None:
        self.target: list[float] | None = None
        self.grip: float | None = None

    def update(self, angles: tuple[float, ...]) -> tuple[list[float], float]:
        desired = [
            limited_target(angles[i], self.signs[i], self.scales[i], self.limits_deg[i], offset_rad=self.home[i])
            for i in range(len(ARM_SERVOS))
        ]
        grip = gripper_fraction(angles[self.gripper_axis], self.signs[self.gripper_axis])
        if self.target is None:
            self.target, self.grip = desired, grip
        self.target = [t + self.alpha * (d - t) for t, d in zip(self.target, desired)]
        self.grip += self.alpha * (grip - self.grip)
        return self.target, self.grip

    def describe(self, joints: list[str]) -> str:
        return (
            "[LEADER] Mapping: "
            + " | ".join(
                f"{label}(ID{SERVO_IDS[label]}) {sign:+.0f}x{scale:g} -> {joint}"
                for label, joint, sign, scale in zip(ARM_SERVOS, joints, self.signs, self.scales)
            )
            + f" | {GRIPPER_SERVO}(ID{SERVO_IDS[GRIPPER_SERVO]}) {self.signs[self.gripper_axis]:+.0f} -> gripper"
        )


class LeaderInput:
    """ServoLeader that holds the last reading on a dropped serial frame instead of stopping the sim."""

    def __init__(self, args: argparse.Namespace):
        self.leader = ServoLeader(args.port, args.baud)
        self.angles = (0.0,) * len(SERVO_IDS)
        self._last_warning = 0.0
        self._last_report = time.monotonic()

    def read(self) -> tuple[float, ...]:
        try:
            self.angles = self.leader.read_radians()
        except RuntimeError as exc:
            now = time.monotonic()
            if now - self._last_warning >= 1.0:
                print(f"[WARN] {exc}; holding last leader target", flush=True)
                self._last_warning = now
        return self.angles

    def rezero(self) -> None:
        """Latch the current leader pose as zero; call together with the arm reset."""
        self.leader.rezero()
        self.angles = (0.0,) * len(SERVO_IDS)

    def report(self, grip: float) -> None:
        now = time.monotonic()
        if now - self._last_report >= 0.5:
            print(
                "\r[LEADER] "
                + " ".join(f"{label}={math.degrees(a):+6.1f}" for label, a in zip(SERVO_IDS, self.angles))
                + f" grip={grip:.2f}",
                end="",
                flush=True,
            )
            self._last_report = now

    def close(self) -> None:
        self.leader.close()


class WallClock:
    """Keep sim time at or behind wall time, so motion (and recordings) have real-world timing."""

    def __init__(self, dt: float):
        self.dt = dt
        self._next = time.monotonic()

    def wait(self) -> None:
        self._next += self.dt
        delay = self._next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self._next = time.monotonic()
