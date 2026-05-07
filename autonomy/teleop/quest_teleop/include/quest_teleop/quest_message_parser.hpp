#pragma once

#include <cstddef>
#include <string>

<<<<<<< HEAD
#include "common_msgs/msg/quest_hand_pose.hpp"
=======
#include "quest_teleop/msg/quest_hand_pose.hpp"
>>>>>>> cbcbea1d (new changes)

class QuestMessageParser {
public:
  static constexpr std::size_t WEBXR_JOINT_COUNT = 25;
  static constexpr std::size_t XYZ_VALUES_PER_JOINT = 3;
  static constexpr std::size_t HAND_ARRAY_SIZE = WEBXR_JOINT_COUNT * XYZ_VALUES_PER_JOINT;

<<<<<<< HEAD
  static common_msgs::msg::QuestHandPose parse(const std::string& json_text);

  static common_msgs::msg::QuestHandPose make_empty_message();
=======
  static quest_teleop::msg::QuestHandPose parse(const std::string& json_text);

  static quest_teleop::msg::QuestHandPose make_empty_message();
>>>>>>> cbcbea1d (new changes)
};