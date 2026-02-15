#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

// 自定义节点头文件
#include "sentry_decision/topics2blackboard.hpp"
#include "sentry_decision/nav2pose.hpp"
#include "sentry_decision/chassis_type_publisher.hpp"
#include "sentry_decision/attack.hpp"
#include "sentry_decision/is_in_position.hpp"
#include "sentry_decision/bt_converters.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sentry_decision_node");

  // 声明参数
  node->declare_parameter<std::string>("bt_xml_file", "");
  node->declare_parameter<double>("tick_rate", 100.0);
  node->declare_parameter<double>("attack_distance", 2.0);

  // 获取行为树 XML 文件路径
  std::string bt_xml_file;
  node->get_parameter("bt_xml_file", bt_xml_file);

  if (bt_xml_file.empty()) {
    RCLCPP_ERROR(node->get_logger(),
                 "未指定行为树 XML 文件! 请设置 bt_xml_file 参数");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "加载行为树: %s", bt_xml_file.c_str());

  // 创建行为树工厂
  BT::BehaviorTreeFactory factory;

  // ===== 注册自定义节点 =====
  // 使用 lambda 传递 ROS 节点到构造函数
  factory.registerBuilder<rm_decision::Topics2Blackboard>(
      "Topics2Blackboard",
      [node](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<rm_decision::Topics2Blackboard>(name, config, node);
      });

  factory.registerBuilder<rm_decision::Nav2Pose>(
      "Nav2Pose",
      [node](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<rm_decision::Nav2Pose>(name, config, node);
      });

  factory.registerBuilder<rm_decision::ChassisTypePublisher>(
      "ChassisTypePublisher",
      [node](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<rm_decision::ChassisTypePublisher>(name, config, node);
      });

  factory.registerBuilder<rm_decision::Attack>(
      "Attack",
      [node](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<rm_decision::Attack>(name, config, node);
      });

  factory.registerBuilder<rm_decision::IsInPosition>(
      "IsInPosition",
      [node](const std::string &name, const BT::NodeConfig &config) {
        return std::make_unique<rm_decision::IsInPosition>(name, config, node);
      });

  // 从 XML 文件创建行为树
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_file);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "加载行为树失败: %s", e.what());
    return 1;
  }

  // 添加日志记录器（调试用）
  BT::StdCoutLogger logger(tree);

  // Groot2 实时监控
  std::unique_ptr<BT::Groot2Publisher> publisher_ptr;
  bool use_groot;
  int groot_port;
  node->declare_parameter<bool>("use_groot", true);
  node->declare_parameter<int>("groot_port", 1667);
  node->get_parameter("use_groot", use_groot);
  node->get_parameter("groot_port", groot_port);

  if (use_groot) {
    try {
      publisher_ptr = std::make_unique<BT::Groot2Publisher>(tree, groot_port);
      RCLCPP_INFO(node->get_logger(), "Groot2 Publisher 已启动，端口: %d", groot_port);
    } catch (const std::exception &e) {
      RCLCPP_WARN(node->get_logger(), "启动 Groot2 Publisher 失败: %s", e.what());
    }
  }

  RCLCPP_INFO(node->get_logger(), "行为树加载成功，开始执行...");



  // 获取 tick 频率
  double tick_rate;
  node->get_parameter("tick_rate", tick_rate);
  rclcpp::Rate rate(tick_rate);

  // 主循环
  while (rclcpp::ok()) {
    BT::NodeStatus status = tree.tickOnce();

    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                            "行为树执行完成 (SUCCESS)，重新开始...");
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                            "行为树执行失败 (FAILURE)，重新开始...");
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}