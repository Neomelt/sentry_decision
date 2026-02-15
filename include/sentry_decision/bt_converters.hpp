#ifndef RM_DECISION_BT_CONVERTERS_HPP_
#define RM_DECISION_BT_CONVERTERS_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace BT
{
// 显式全特化
template <>
inline geometry_msgs::msg::PoseStamped convertFromString<geometry_msgs::msg::PoseStamped>(StringView str)
{
  // std::cout << "DEBUG: Parsing PoseStamped: " << str << std::endl;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0; 

  auto parts = splitString(str, ',');
  if (parts.size() >= 7) {
    pose.pose.position.x = convertFromString<double>(parts[0]);
    pose.pose.position.y = convertFromString<double>(parts[1]);
    pose.pose.position.z = convertFromString<double>(parts[2]);
    pose.pose.orientation.x = convertFromString<double>(parts[3]);
    pose.pose.orientation.y = convertFromString<double>(parts[4]);
    pose.pose.orientation.z = convertFromString<double>(parts[5]);
    pose.pose.orientation.w = convertFromString<double>(parts[6]);
  } else if (parts.size() >= 2) {
    pose.pose.position.x = convertFromString<double>(parts[0]);
    pose.pose.position.y = convertFromString<double>(parts[1]);
    if (parts.size() > 2) pose.pose.position.z = convertFromString<double>(parts[2]);
  }
  return pose;
}
} // namespace BT

#endif // RM_DECISION_BT_CONVERTERS_HPP_
