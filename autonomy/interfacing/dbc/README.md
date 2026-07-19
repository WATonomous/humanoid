# DBC Interface

This folder contains DBC (CAN database) file handling and integration for the humanoid autonomy system.

## Overview

DBC files define CAN bus messages, signals, and communication protocols used in vehicle communication systems.

## Contents

- `humanoid.dbc` - message definitions

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

