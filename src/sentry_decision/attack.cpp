#include "sentry_decision/attack.hpp"

using namespace BT;

namespace rm_decision
{

Attack::Attack(const std::string &name, const NodeConfig &config,
               std::shared_ptr<rclcpp::Node> node)
    : SyncActionNode(name, config), node_(node)
{
  // 从参数服务器读取攻击距离，默认 2.0m
  node_->get_parameter_or("attack_distance", attack_distance_, 2.0);
}

NodeStatus Attack::tick()
{
  auto target_opt = getInput<geometry_msgs::msg::PoseStamped>("target_position");
  if (!target_opt) {
    RCLCPP_WARN(node_->get_logger(), "Attack: 无法读取 target_position");
    return NodeStatus::FAILURE;
  }

  const auto &target = target_opt.value();
  double tx = target.pose.position.x;
  double ty = target.pose.position.y;

  // 获取机器人当前位置（简化：假设从 TF 或其他方式获取）
  // 这里使用目标位置的反方向偏移作为攻击点
  // 实际使用时应从 TF 获取 base_link 在 map 下的位置

  // 计算攻击位姿：在目标前方 attack_distance_ 处
  // 朝向目标的方向
  geometry_msgs::msg::PoseStamped attack_pose;
  attack_pose.header.frame_id = "map";
  attack_pose.header.stamp = node_->now();

  // 默认从原点方向看目标（后续会从 TF 获取实际位置改进）
  double dx = tx;
  double dy = ty;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < 0.1) {
    // 目标太近，直接设置攻击点在目标位置
    attack_pose.pose = target.pose;
  } else {
    // 在目标前方 attack_distance_ 处
    double ratio = attack_distance_ / dist;
    attack_pose.pose.position.x = tx - dx * ratio;
    attack_pose.pose.position.y = ty - dy * ratio;
    attack_pose.pose.position.z = 0.0;

    // 朝向目标
    double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    attack_pose.pose.orientation.x = q.x();
    attack_pose.pose.orientation.y = q.y();
    attack_pose.pose.orientation.z = q.z();
    attack_pose.pose.orientation.w = q.w();
  }

  setOutput("attack_pose", attack_pose);

  RCLCPP_DEBUG(node_->get_logger(),
               "Attack: 目标(%.2f,%.2f) -> 攻击位姿(%.2f,%.2f)",
               tx, ty,
               attack_pose.pose.position.x,
               attack_pose.pose.position.y);

  return NodeStatus::SUCCESS;
}

PortsList Attack::providedPorts()
{
  return {
      InputPort<geometry_msgs::msg::PoseStamped>("target_position", "目标位置"),
      OutputPort<geometry_msgs::msg::PoseStamped>("attack_pose", "计算出的攻击位姿"),
  };
}

} // namespace rm_decision
