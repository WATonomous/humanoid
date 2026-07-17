#!/usr/bin/env python3
"""Zero all motor origins. Run once, done.

Confirmed against real bus traffic (candump can0): these motors reply on extended IDs
of the form 0x2900 | motor_id, matching this repo's own DBC (autonomy/interfacing/dbc/
humanoid.dbc) ServoStatusFeedback message (declared as 0x80002900 -- the 0x80000000 bit
is only the extended-frame marker bit, not part of the real 29-bit arbitration ID; the
real ID is 0x2900 | motor_id). Commands use the same base+id scheme:
    BO_ 2147484928 SetOriginCmd: 1 CubeMars   -- declared 0x80000500 -> real ID 0x500|motor_id
    SG_ OriginMode : 7|8@0+ (1,0) [0|1]        -- single byte at offset 0
"""

import can
import time

MOTOR_IDS = [10, 11, 12, 13, 14]
SET_ORIGIN_BASE_ID = 0x500  # DBC SetOriginCmd, marker bit masked off

bus = can.Bus(interface="socketcan", channel="can0", bitrate=1000000)

for mid in MOTOR_IDS:
    input(f"Please press Enter to zero motor {mid}")
    bus.send(can.Message(
        arbitration_id=SET_ORIGIN_BASE_ID | mid,
        data=bytes([1]),  # 0 = temporary, 1 = permanent (saved to flash)
        is_extended_id=True,
    ))
    print(f"Motor {mid}: origin set to 0")
    time.sleep(0.05)

bus.shutdown()
print("Done.")
