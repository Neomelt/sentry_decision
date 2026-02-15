#ifndef RM_DECISION_NAV2POSE_HPP_
#define RM_DECISION_NAV2POSE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "sentry_decision/bt_converters.hpp"

using namespace BT;

namespace rm_decision
{

/**
 * @brief 向 Nav2 发送导航目标点
 *
 * StatefulActionNode：onStart 发送 goal，onRunning 检查状态，onHalted 取消导航。
 * goal 端口支持从 XML 属性读取坐标字符串（格式: "x,y,z,qx,qy,qz,qw"）
 * 或从黑板读取 PoseStamped。
 */
class Nav2Pose : public StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2Pose(const std::string &name, const NodeConfig &config,
           std::shared_ptr<rclcpp::Node> node);
  ~Nav2Pose() override = default;

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

  static PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_accepted_ = false;
  bool goal_done_ = false;
  bool goal_succeeded_ = false;

  /**
   * @brief 将 "x,y,z,qx,qy,qz,qw" 格式字符串解析为 PoseStamped
   */
  geometry_msgs::msg::PoseStamped parsePoseString(const std::string &pose_str);
};

} // namespace rm_decision



#endif // RM_DECISION_NAV2POSE_HPP_
