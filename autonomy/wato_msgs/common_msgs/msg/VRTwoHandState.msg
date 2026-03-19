std_msgs/Header header

smple_msgs/VRHandState left
sample_msgs/VRHandState right

# right hand position - left hand position
geometry_msgs/msg/Vector3 position_diff
float32 distance

# sin/cos of right hand rotation - left hand rotation
geometry_msgs/Vector3 rotation_diff_sin 
geometry_msgs/Vector3 rotation_diff_cosa