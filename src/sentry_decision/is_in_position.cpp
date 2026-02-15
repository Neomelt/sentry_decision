#include "sentry_decision/is_in_position.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace BT;

namespace rm_decision
{

IsInPosition::IsInPosition(const std::string &name, const NodeConfig &config,
                           std::shared_ptr<rclcpp::Node> node)
    : DecoratorNode(name, config), node_(node)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool IsInPosition::isPointInQuad(
    double px, double py,
    const geometry_msgs::msg::PoseStamped &p1,
    const geometry_msgs::msg::PoseStamped &p2,
    const geometry_msgs::msg::PoseStamped &p3,
    const geometry_msgs::msg::PoseStamped &p4)
{
  // 使用叉积法判断点是否在凸四边形内
  // 四个顶点按顺序排列（顺时针或逆时针）
  auto cross = [](double ax, double ay, double bx, double by) -> double {
    return ax * by - ay * bx;
  };

  double x[4] = {p1.pose.position.x, p2.pose.position.x,
                  p3.pose.position.x, p4.pose.position.x};
  double y[4] = {p1.pose.position.y, p2.pose.position.y,
                  p3.pose.position.y, p4.pose.position.y};

  bool all_positive = true;
  bool all_negative = true;

  for (int i = 0; i < 4; i++) {
    int j = (i + 1) % 4;
    double edge_x = x[j] - x[i];
    double edge_y = y[j] - y[i];
    double to_point_x = px - x[i];
    double to_point_y = py - y[i];
    double cp = cross(edge_x, edge_y, to_point_x, to_point_y);

    if (cp < 0) all_positive = false;
    if (cp > 0) all_negative = false;
  }

  return all_positive || all_negative;
}

NodeStatus IsInPosition::tick()
{
  // 获取四个角点
  auto p1_opt = getInput<geometry_msgs::msg::PoseStamped>("p1");
  auto p2_opt = getInput<geometry_msgs::msg::PoseStamped>("p2");
  auto p3_opt = getInput<geometry_msgs::msg::PoseStamped>("p3");
  auto p4_opt = getInput<geometry_msgs::msg::PoseStamped>("p4");

  if (!p1_opt || !p2_opt || !p3_opt || !p4_opt) {
    RCLCPP_WARN(node_->get_logger(), "IsInPosition: 无法读取角点端口");
    return NodeStatus::FAILURE;
  }

  // 获取机器人当前位置
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform("map", "base_link",
                                             tf2::TimePointZero,
                                             std::chrono::milliseconds(100));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "IsInPosition: TF 获取失败: %s", ex.what());
    // TF 不可用时，直接执行子节点（不限制区域）
    return child_node_->executeTick();
  }

  double robot_x = transform.transform.translation.x;
  double robot_y = transform.transform.translation.y;

  if (isPointInQuad(robot_x, robot_y,
                    p1_opt.value(), p2_opt.value(),
                    p3_opt.value(), p4_opt.value())) {
    // 在区域内，执行子节点
    return child_node_->executeTick();
  } else {
    // 不在区域内，返回 FAILURE
    RCLCPP_DEBUG(node_->get_logger(),
                 "IsInPosition: 机器人 (%.2f, %.2f) 不在指定区域内",
                 robot_x, robot_y);
    return NodeStatus::FAILURE;
  }
}

PortsList IsInPosition::providedPorts()
{
  return {
      InputPort<geometry_msgs::msg::PoseStamped>("p1", "矩形区域顶点1"),
      InputPort<geometry_msgs::msg::PoseStamped>("p2", "矩形区域顶点2"),
      InputPort<geometry_msgs::msg::PoseStamped>("p3", "矩形区域顶点3"),
      InputPort<geometry_msgs::msg::PoseStamped>("p4", "矩形区域顶点4"),
  };
}

} // namespace rm_decision
