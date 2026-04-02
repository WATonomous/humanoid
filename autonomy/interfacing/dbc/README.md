# DBC Interface

This folder contains DBC (CAN database) file handling and integration for the humanoid autonomy system.

## Overview

DBC files define CAN bus messages, signals, and communication protocols used in vehicle communication systems.

## Contents

- `humanoid.dbc` - message definitions
- `decode.c` - generated c code for embedded system



## Usage

generate via entering the docker container and then:
`dbcparser dbc2 --dbc=dbc/humanoid.dbc --format=C >> decode.c`

