# Cube mars documentation: https://www.cubemars.com/data/cms/202602/ak-series-prodcut-manual-v3-2-0-for-ak-3-0-robotic-actuator.pdf
# GL II (GL40) gimbal drive MIT protocol: https://www.cubemars.com/images/file/20241231/1735633965678815.pdf
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

# MIT special frames (FF..FF xx): only motor_id is read. A GL II ignores MIT commands until
# MIT_ENTER and goes limp on MIT_EXIT. MIT_SET_ZERO makes the current shaft position zero.
int8 MIT_ENTER=9          # 0xFC enter motor mode (enable)
int8 MIT_EXIT=10          # 0xFD exit motor mode (motor goes limp)
int8 MIT_SET_ZERO=11      # 0xFE set current position as zero
int8 MIT_CLEAR_ERRORS=12  # 0xFB clear drive error state

# TODO: define how hand motors will be controlled with this message

# Fields
# position: POSITION_LOOP/POSITION_VELOCITY -> degrees; MIT_CONTROL -> RADIANS
float32 position
float32 velocity
float32 acceleration

float32 torque

float32 current
float32 duty_cycle

bool temporary false

# Gains - allow gain scheduling via message
# TODO: tune and define default values in message
# MIT_CONTROL only, physical units (N.m/rad, N.m.s/rad); can_node scales them per motor.
float32 kd
float32 kp
