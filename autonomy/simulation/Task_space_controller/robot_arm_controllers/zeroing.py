#!/usr/bin/env python3
"""Zero all motor origins. Run once, done."""

import can
import struct
import time

MOTOR_IDS = [10, 11, 12, 13, 14]
MODE_SET_ORIGIN = 5

bus = can.Bus(interface="socketcan", channel="can0", bitrate=1000000)

for mid in MOTOR_IDS:
    input(f"Please press Enter to zero motor {mid}")
    bus.send(can.Message(
        arbitration_id=(MODE_SET_ORIGIN << 8) | mid,
        data=bytes([1]),  # 0 = temporary, 1 = permanent (saved to flash)
        is_extended_id=True,
    ))
    print(f"Motor {mid}: origin set to 0")
    time.sleep(0.05)

bus.shutdown()
print("Done.")