#ifndef RM_DECISION_TOPICS2BLACKBOARD_HPP_
#define RM_DECISION_TOPICS2BLACKBOARD_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include "sentry_decision/bt_converters.hpp"

using namespace BT;

namespace rm_decision
{

/**
 * @brief 订阅裁判系统和自瞄话题，将数据写入 BT 黑板
 *
 * 通过 ROS2 话题接收比赛状态数据，每次 tick 时更新到黑板变量中。
 * 下游节点通过黑板读取这些数据做决策。
 */
class Topics2Blackboard : public SyncActionNode
{
public:
  Topics2Blackboard(const std::string &name, const NodeConfig &config,
                    std::shared_ptr<rclcpp::Node> node);
  ~Topics2Blackboard() override = default;

  NodeStatus tick() override;
  static PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;

  // ===== 裁判系统数据 =====
  uint16_t current_hp_ = 600;
  uint16_t projectile_allowance_17mm_ = 200;
  uint16_t my_base_hp_ = 5000;
  uint16_t my_outpost_hp_ = 1500;
  uint16_t enemy_outpost_hp_ = 1500;
  uint16_t enemy_base_hp_ = 5000;
  uint8_t  game_progress_ = 0;       // 0:未开始 4:比赛中
  uint16_t stage_remain_time_ = 420;
  uint8_t  hurt_type_ = 0;

  // ===== 自瞄数据 =====
  bool     tracking_ = false;
  std::string target_armor_id_ = "";
  uint8_t  armor_id_ = 0;
  geometry_msgs::msg::PoseStamped target_position_;

  // ===== 订阅者 =====
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr current_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr projectile_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr my_base_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr my_outpost_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr enemy_outpost_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr enemy_base_hp_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  game_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr stage_remain_time_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  hurt_type_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   tracking_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_armor_id_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr  armor_id_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_position_sub_;
};

} // namespace rm_decision

#endif // RM_DECISION_TOPICS2BLACKBOARD_HPP_
