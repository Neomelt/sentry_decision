#ifndef RM_DECISION_IS_IN_POSITION_HPP_
#define RM_DECISION_IS_IN_POSITION_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorator_node.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "sentry_decision/bt_converters.hpp"

using namespace BT;

namespace rm_decision
{

/**
 * @brief 检查机器人是否在指定矩形区域内
 *
 * DecoratorNode: 通过 p1-p4 定义一个矩形区域。
 * 如果机器人当前位置在区域内，执行子节点；否则返回 FAILURE。
 * 通过 TF2 获取机器人在 map 坐标系下的位置。
 */
class IsInPosition : public DecoratorNode
{
public:
  IsInPosition(const std::string &name, const NodeConfig &config,
               std::shared_ptr<rclcpp::Node> node);
  ~IsInPosition() override = default;

  NodeStatus tick() override;
  static PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /**
   * @brief 判断点 (px, py) 是否在由四个顶点定义的凸四边形内
   */
  bool isPointInQuad(double px, double py,
                     const geometry_msgs::msg::PoseStamped &p1,
                     const geometry_msgs::msg::PoseStamped &p2,
                     const geometry_msgs::msg::PoseStamped &p3,
                     const geometry_msgs::msg::PoseStamped &p4);
};

} // namespace rm_decision

#endif // RM_DECISION_IS_IN_POSITION_HPP_
