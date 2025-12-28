#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/qos.hpp>
#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

namespace rm_decision
{
  class Send_goal : public SyncActionNode
  {
  public:
        Send_goal(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
        ~Send_goal() override = default;
        NodeStatus tick() override;
        static PortsList providedPorts();
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr goal_pub_;
  };
} // end namespace rm_decision
