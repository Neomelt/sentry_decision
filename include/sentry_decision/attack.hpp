#ifndef RM_DECISION_ATTACK_HPP_
#define RM_DECISION_ATTACK_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include "sentry_decision/bt_converters.hpp"

using namespace BT;

namespace rm_decision
{

/**
 * @brief 根据目标位置计算攻击位姿
 *
 * 读取 target_position (PoseStamped)，在目标前方一定距离处
 * 计算一个面向目标的攻击位姿，输出到 attack_pose。
 */
class Attack : public SyncActionNode
{
public:
  Attack(const std::string &name, const NodeConfig &config,
         std::shared_ptr<rclcpp::Node> node);
  ~Attack() override = default;

  NodeStatus tick() override;
  static PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;

  // 攻击距离：在目标前方多远处停下
  double attack_distance_ = 2.0;
};

} // namespace rm_decision

#endif // RM_DECISION_ATTACK_HPP_
