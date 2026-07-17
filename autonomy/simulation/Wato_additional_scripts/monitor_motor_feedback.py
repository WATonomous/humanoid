#!/usr/bin/env python3
"""Continuously print real motor feedback (position/angle) to the console.

Confirmed against real bus traffic (candump can0): feedback arrives unprompted on
extended IDs of the form 0x2900 | motor_id, matching this repo's DBC ServoStatusFeedback
message (autonomy/interfacing/dbc/humanoid.dbc, declared 0x80002900 -- the 0x80000000
bit is only the extended-frame marker, real ID is 0x2900 | motor_id):

    BO_ 2147494144 ServoStatusFeedback: 8 Motor
       SG_ FbkPosition    : 7|16@0-  (0.1,0)  [-3200|3200]   "deg"
       SG_ FbkSpeed       : 23|16@0- (10,0)   [-320000|320000] "ERPM"
       SG_ FbkCurrent     : 39|16@0- (0.01,0) [-60|60]       "A"
       SG_ FbkTemperature : 55|8@0-  (1,0)    [-20|127]      "degC"
       SG_ FbkErrorCode   : 63|8@0+  (1,0)    [0|7]          ""

All multi-byte fields are big-endian (Motorola byte order per the DBC's "@0"), so each
is just a plain big-endian signed 16-bit int at its byte offset, scaled.

This bypasses ROS/can_node.cpp entirely and reads the real CAN bus directly with
python-can, since can_node.cpp's DBC-id lookup subtracts the extended-frame marker bit
before matching (see can_node.cpp's `can_id_map` construction), which is a different
code path than this script needs -- reading here is simpler and directly verifiable
against candump.

Usage:
  /usr/bin/python3 monitor_motor_feedback.py
"""

import struct

import can

# can_id -> joint label, from hardware_mapping.yaml's 5 AK-series arm joints.
CAN_ID_LABELS = {
    0x0E: "shoulder_pitch",
    0x0C: "shoulder_roll",
    0x0D: "shoulder_yaw",
    0x0A: "elbow_pitch",
    0x0B: "elbow_roll",
}

FEEDBACK_BASE_ID = 0x2900  # DBC ServoStatusFeedback, marker bit masked off


def decode_feedback(data):
    position_deg = struct.unpack(">h", data[0:2])[0] * 0.1
    speed_erpm = struct.unpack(">h", data[2:4])[0] * 10
    current_a = struct.unpack(">h", data[4:6])[0] * 0.01
    temp_c = struct.unpack(">b", data[6:7])[0]
    error_code = data[7]
    return position_deg, speed_erpm, current_a, temp_c, error_code


def main():
    bus = can.Bus(interface="socketcan", channel="can0", bitrate=1000000)
    print("Listening for ServoStatusFeedback frames on can0 ...")

    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            if not msg.is_extended_id or len(msg.data) != 8:
                continue

            motor_id = msg.arbitration_id & 0xFF
            base = msg.arbitration_id & ~0xFF
            if base != FEEDBACK_BASE_ID:
                continue  # not a ServoStatusFeedback frame

            label = CAN_ID_LABELS.get(motor_id, f"0x{motor_id:02X}")
            position, speed, current, temp, err = decode_feedback(msg.data)
            print(f"[{label:16s}] can_id=0x{motor_id:02X}  position={position:+7.2f} deg  "
                  f"speed={speed:+8.0f} ERPM  current={current:+6.2f} A  "
                  f"temp={temp:4d}C  err={err}")
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
