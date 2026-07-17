#!/usr/bin/env python3
"""Interactively send a single-joint PositionLoopCmd to one motor at a time, to
verify each AK-series motor (and its CAN ID) individually.

Confirmed against real bus traffic (candump can0): same base+id scheme as
zeroing.py / monitor_motor_feedback.py, matching this repo's DBC
(autonomy/interfacing/dbc/humanoid.dbc):

    BO_ 2147484672 PositionLoopCmd: 4 CubeMars   -- declared 0x80000400 -> real ID 0x400|motor_id
       SG_ PositionDeg : 7|32@0- (0.0001,0) [-36000|36000] "deg"   -- 32-bit signed, big-endian, scale 0.0001

Bypasses ROS entirely -- talks straight to can0 with python-can, same as
zeroing.py/monitor_motor_feedback.py. Run monitor_motor_feedback.py in another
terminal alongside this one to confirm each motor's real position updates.

Usage:
  /usr/bin/python3 send_motor_command.py
"""

import struct
import can

POSITION_LOOP_BASE_ID = 0x400  # DBC PositionLoopCmd, marker bit masked off

MOTORS = {
    "shoulder_pitch": 0x0E,
    "shoulder_roll": 0x0C,
    "shoulder_yaw": 0x0D,
    "elbow_pitch": 0x0A,
    "elbow_roll": 0x0B,
}


def encode_position_deg(angle_deg: float) -> bytes:
    raw = int(round(angle_deg / 0.0001))
    raw = max(-2**31, min(2**31 - 1, raw))
    return struct.pack(">i", raw)  # big-endian signed 32-bit (DBC "@0-" = Motorola, signed)


def print_menu():
    print("\nMotors:")
    for i, (label, mid) in enumerate(MOTORS.items(), start=1):
        print(f"  {i}. {label} (can_id=0x{mid:02X})")
    print("  q. quit")


def main():
    bus = can.Bus(interface="socketcan", channel="can0", bitrate=1000000)
    labels = list(MOTORS.keys())

    print("Sends ONE PositionLoopCmd at a time to ONE motor -- nothing else moves.")
    print("Move in small steps and watch the physical joint. Ctrl+C to quit.")

    try:
        while True:
            print_menu()
            choice = input("Select motor: ").strip().lower()
            if choice in ("q", "quit", "exit"):
                break
            try:
                idx = int(choice) - 1
                label = labels[idx]
            except (ValueError, IndexError):
                print("Invalid choice, try again.")
                continue

            motor_id = MOTORS[label]
            angle_str = input(f"[{label}] target angle in degrees (raw, uncalibrated): ").strip()
            try:
                angle_deg = float(angle_str)
            except ValueError:
                print("Invalid number, try again.")
                continue

            bus.send(can.Message(
                arbitration_id=POSITION_LOOP_BASE_ID | motor_id,
                data=encode_position_deg(angle_deg),
                is_extended_id=True,
            ))
            print(f"Sent: {label} (can_id=0x{motor_id:02X}) -> {angle_deg} deg")
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
