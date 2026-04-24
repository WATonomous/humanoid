# Motor ID and control type
int8 motor_id
int8 control_type

# Control type constants
int8 MIT_CONTROL=0
int8 CURRENT_LOOP=1
int8 CURRENT_BRAKE=2
int8 VELOCITY_LOOP=3
int8 POSITION_LOOP=4
int8 SET_ORIGIN=5
int8 POSITION_VELOCITY=6
int8 DUTY_CYCLE=7
int8 DISABLE=8

# TODO: define how hand motors will be controlled with this message

# Fields
float32 position
float32 velocity
float32 acceleration

float32 torque

float32 current
float32 duty_cycle

bool temporary false