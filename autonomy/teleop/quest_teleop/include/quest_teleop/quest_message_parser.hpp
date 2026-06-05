#pragma once

#include <cstddef>
#include <string>

#include "common_msgs/msg/quest_hand_pose.hpp"

class QuestMessageParser {
public:
  static constexpr std::size_t WEBXR_JOINT_COUNT = 25;
  static constexpr std::size_t XYZ_VALUES_PER_JOINT = 3;
  static constexpr std::size_t HAND_ARRAY_SIZE = WEBXR_JOINT_COUNT * XYZ_VALUES_PER_JOINT;

  static common_msgs::msg::QuestHandPose parse(const std::string& json_text);

  static common_msgs::msg::QuestHandPose make_empty_message();
};