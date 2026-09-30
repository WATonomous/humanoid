# DBC Interface

This folder contains DBC (CAN database) file handling and integration for the humanoid autonomy system.

## Overview

DBC files define CAN bus messages, signals, and communication protocols used in vehicle communication systems.

## Contents

- `humanoid.dbc` - message definitions

### One message is not like the others

`MITControlCmd` is a **standard 11-bit** message (`BO_ 0`), while every servo-mode message here
is extended (its base id carries `0x80000000`, which is what sets the extended flag when
`can_node` ORs in the motor id). A GL II drive in MIT mode only listens to standard frames, so
that difference is load-bearing.

Its signals are the **raw fixed-point codes** the CubeMars MIT protocol defines
(`pos(16) vel(12) kp(12) kd(12) t_ff(12)`), not physical units: `can_node` converts N·m/rad and
radians into those codes using each motor's range from `can/config/mit_profiles.yaml`. MIT
*feedback* has a different layout again, arrives on the drive's master id, and is decoded in
`can_node` (`decodeGl2Feedback`) rather than through this file. `can/test/test_mit_protocol.cpp`
pins the layout against the manual's worked example.

## Two ways to decode DBC

Turning raw CAN bytes into named signals can be done statically or dynamically. This repo
uses the dynamic path on the ROS side:

| | Static (`decode.c`) | Dynamic (`can_node` + `libdbcppp`) ← used here |
|---|---|---|
| DBC read | compiled into C ahead of time | `humanoid.dbc` loaded from file at startup |
| Change the DBC | regenerate + recompile | edit `.dbc`, restart the node |
| Cost | tiny, no runtime parsing | needs the lib + parses on start |
| Fits | bare-metal firmware (STM32/ESP32) | Linux / ROS host |

`can_node` links `libdbcppp.so` and installs `humanoid.dbc` into its package share, then
loads and decodes against it at runtime (you'll see `Loaded DBC message: ...` on startup).
So it **never uses `decode.c`** — that static decoder is only for embedded boards that
can't run libdbcppp.

## Usage

`decode.c` is generated on demand from `humanoid.dbc`, not committed. To regenerate, enter
the docker container and run:
`dbcparser dbc2 --dbc=dbc/humanoid.dbc --format=C >> decode.c`

