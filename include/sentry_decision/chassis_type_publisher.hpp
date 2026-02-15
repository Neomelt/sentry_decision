#ifndef RM_DECISION_CHASSIS_TYPE_PUBLISHER_HPP_
#define RM_DECISION_CHASSIS_TYPE_PUBLISHER_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

using namespace BT;

namespace rm_decision
{

/**
 * @brief 发布底盘模式命令
 *
 * 从输入端口读取 chassis_cmd (int8) 并发布到 /chassis_type 话题。
 * 底盘模式编号可在 XML 中直接修改数字来更换含义。
 *
 * 示例编号约定（可自行修改）：
 *   1 = 跟随云台
 *   2 = 不跟随
 *   4 = 小陀螺
 */
class ChassisTypePublisher : public SyncActionNode
{
public:
  ChassisTypePublisher(const std::string &name, const NodeConfig &config,
                       std::shared_ptr<rclcpp::Node> node);
  ~ChassisTypePublisher() override = default;

  NodeStatus tick() override;
  static PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr chassis_type_pub_;
};

} // namespace rm_decision

#endif // RM_DECISION_CHASSIS_TYPE_PUBLISHER_HPP_
