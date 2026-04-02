std_msgs/Header header

string name

bool is_left

common_msgs/JointState shoulder
common_msgs/JointState elbow
common_msgs/JointState wrist

# Optional Overall Hand Pose
bool include_hand_pose
common_msgs/HandPose hand_pose