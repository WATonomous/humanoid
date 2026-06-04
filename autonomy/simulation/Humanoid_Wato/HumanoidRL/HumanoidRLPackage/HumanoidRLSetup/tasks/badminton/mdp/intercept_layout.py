"""Privileged intercept command tensor layout (shared by command + tracking)."""

# Command shape: (num_envs, 12)
#   [0:3]   intercept position in robot base frame
#   [3:7]   desired EE orientation (quaternion, base frame)
#   [7:10]  desired EE linear velocity at impact (base frame)
#   [10]    hit-moment pulse (1.0 on shuttle-arrival step)
#   [11]    seconds until shuttle arrival

COMMAND_DIM = 12
COMMAND_POS_SLICE = slice(0, 3)
COMMAND_QUAT_SLICE = slice(3, 7)
COMMAND_VEL_SLICE = slice(7, 10)
COMMAND_HIT_SLICE = 10
COMMAND_TIME_SLICE = 11
