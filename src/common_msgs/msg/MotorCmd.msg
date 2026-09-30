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

# MIT mode "special" frames (FF FF FF FF FF FF FF xx). These carry no payload: only motor_id
# is read. Required by GL II drives, which ignore MIT command frames until MIT_ENTER is sent
# and go limp on MIT_EXIT. MIT_SET_ZERO makes the CURRENT shaft position the drive's zero --
# only send it when the joint is physically in its known zero pose.
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

# Gains - allow gain scheduling via message.
# MIT_CONTROL only: torque = kp*(position - position_actual) + kd*(velocity - velocity_actual)
# + torque. Units are physical (N.m/rad, N.m.s/rad); can_node scales them to the drive's
# 12-bit codes using that motor's entry in can/config/mit_profiles.yaml. Per-joint values
# live in joint_command/config/safety_limits.yaml (mit_kp / mit_kd).
float32 kd
float32 kp
