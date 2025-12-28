#include "sentry_decision/send_goal.hpp"
using namespace BT;

namespace rm_decision
{

  Send_goal::Send_goal(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : SyncActionNode(name, config), node_(node)
  {
    goal_pub_ = node_->create_publisher<std_msgs::msg::Int8>("/sentry_goal", rclcpp::QoS(1).transient_local());
  }

  NodeStatus Send_goal::tick()
  {
    std_msgs::msg::Int8 goal_msg;
    goal_msg.data = 1; // Example goal data
    goal_pub_->publish(goal_msg);
    return NodeStatus::SUCCESS;
  }

  PortsList Send_goal::providedPorts()
  {
    return {};
  }

} // end namespace rm_decision