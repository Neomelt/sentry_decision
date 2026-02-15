#include "sentry_decision/topics2blackboard.hpp"

using namespace BT;

namespace rm_decision
{

Topics2Blackboard::Topics2Blackboard(
    const std::string &name, const NodeConfig &config,
    std::shared_ptr<rclcpp::Node> node)
    : SyncActionNode(name, config), node_(node)
{
  auto qos = rclcpp::QoS(1).best_effort();

  // ===== 裁判系统话题订阅 =====
  current_hp_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/referee/current_hp", qos,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        current_hp_ = msg->data;
      });

  projectile_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/referee/projectile_allowance_17mm", qos,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        projectile_allowance_17mm_ = msg->data;
      });

  my_base_hp_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/referee/my_base_hp", qos,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        my_base_hp_ = msg->data;
      });

  my_outpost_hp_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/referee/my_outpost_hp", qos,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        my_outpost_hp_ = msg->data;
      });

  enemy_outpost_hp_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/referee/enemy_outpost_hp", qos,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        enemy_outpost_hp_ = msg->data;
      });

  enemy_base_hp_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/referee/enemy_base_hp", qos,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        enemy_base_hp_ = msg->data;
      });

  game_progress_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/referee/game_progress", qos,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        game_progress_ = msg->data;
      });

  stage_remain_time_sub_ = node_->create_subscription<std_msgs::msg::UInt16>(
      "/referee/stage_remain_time", qos,
      [this](const std_msgs::msg::UInt16::SharedPtr msg) {
        stage_remain_time_ = msg->data;
      });

  hurt_type_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/referee/hurt_type", qos,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        hurt_type_ = msg->data;
      });

  // ===== 自瞄话题订阅 =====
  tracking_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/tracker/tracking", qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        tracking_ = msg->data;
      });

  target_armor_id_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/tracker/target_armor_id", qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        target_armor_id_ = msg->data;
      });

  armor_id_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "/tracker/armor_id", qos,
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        armor_id_ = msg->data;
      });

  target_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tracker/target_position", qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        target_position_ = *msg;
      });

  RCLCPP_INFO(node_->get_logger(), "Topics2Blackboard: 所有话题订阅已创建");
}

NodeStatus Topics2Blackboard::tick()
{
  // 处理最新的回调消息
  rclcpp::spin_some(node_);

  // 将所有数据写入黑板
  setOutput("current_hp", current_hp_);
  setOutput("projectile_allowance_17mm", projectile_allowance_17mm_);
  setOutput("my_base_hp", my_base_hp_);
  setOutput("my_outpost_hp", my_outpost_hp_);
  setOutput("enemy_outpost_hp", enemy_outpost_hp_);
  setOutput("enemy_base_hp", enemy_base_hp_);
  setOutput("game_progress", game_progress_);
  setOutput("stage_remain_time", stage_remain_time_);
  setOutput("hurt_type", hurt_type_);
  setOutput("tracking", tracking_);
  setOutput("target_armor_id", target_armor_id_);
  setOutput("armor_id", armor_id_);
  setOutput("target_position", target_position_);

  return NodeStatus::SUCCESS;
}

PortsList Topics2Blackboard::providedPorts()
{
  return {
      OutputPort<uint16_t>("current_hp"),
      OutputPort<uint16_t>("projectile_allowance_17mm"),
      OutputPort<uint16_t>("my_base_hp"),
      OutputPort<uint16_t>("my_outpost_hp"),
      OutputPort<uint16_t>("enemy_outpost_hp"),
      OutputPort<uint16_t>("enemy_base_hp"),
      OutputPort<uint8_t>("game_progress"),
      OutputPort<uint16_t>("stage_remain_time"),
      OutputPort<uint8_t>("hurt_type"),
      OutputPort<bool>("tracking"),
      OutputPort<std::string>("target_armor_id"),
      OutputPort<uint8_t>("armor_id"),
      OutputPort<geometry_msgs::msg::PoseStamped>("target_position"),
  };
}

} // namespace rm_decision
