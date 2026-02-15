#include "sentry_decision/chassis_type_publisher.hpp"

using namespace BT;

namespace rm_decision
{

ChassisTypePublisher::ChassisTypePublisher(
    const std::string &name, const NodeConfig &config,
    std::shared_ptr<rclcpp::Node> node)
    : SyncActionNode(name, config), node_(node)
{
  chassis_type_pub_ = node_->create_publisher<std_msgs::msg::Int8>(
      "/chassis_type", rclcpp::QoS(1).transient_local());
}

NodeStatus ChassisTypePublisher::tick()
{
  auto chassis_cmd = getInput<int8_t>("chassis_cmd");
  if (!chassis_cmd) {
    RCLCPP_WARN(node_->get_logger(),
                "ChassisTypePublisher: 无法读取 chassis_cmd 端口");
    return NodeStatus::FAILURE;
  }

  std_msgs::msg::Int8 msg;
  msg.data = chassis_cmd.value();
  chassis_type_pub_->publish(msg);

  RCLCPP_DEBUG(node_->get_logger(),
               "ChassisTypePublisher: 发布底盘模式 %d", msg.data);
  return NodeStatus::SUCCESS;
}

PortsList ChassisTypePublisher::providedPorts()
{
  return {
      InputPort<int8_t>("chassis_cmd", "底盘模式编号 (数字可在 XML 中直接修改)"),
  };
}

} // namespace rm_decision
