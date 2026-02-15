#include "sentry_decision/nav2pose.hpp"
#include "sentry_decision/bt_converters.hpp"
#include <sstream>
#include <vector>

using namespace BT;

namespace rm_decision
{

Nav2Pose::Nav2Pose(const std::string &name, const NodeConfig &config,
                   std::shared_ptr<rclcpp::Node> node)
    : StatefulActionNode(name, config), node_(node)
{
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
  RCLCPP_INFO(node_->get_logger(), "Nav2Pose: action client 已创建");
}

geometry_msgs::msg::PoseStamped Nav2Pose::parsePoseString(const std::string &pose_str)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = node_->now();

  std::vector<double> vals;
  std::stringstream ss(pose_str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    try {
      vals.push_back(std::stod(token));
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "Nav2Pose: 无法解析坐标值: %s", token.c_str());
      vals.push_back(0.0);
    }
  }

  if (vals.size() >= 7) {
    pose.pose.position.x = vals[0];
    pose.pose.position.y = vals[1];
    pose.pose.position.z = vals[2];
    pose.pose.orientation.x = vals[3];
    pose.pose.orientation.y = vals[4];
    pose.pose.orientation.z = vals[5];
    pose.pose.orientation.w = vals[6];
  } else if (vals.size() >= 2) {
    // 至少有 x, y
    pose.pose.position.x = vals[0];
    pose.pose.position.y = vals.size() > 1 ? vals[1] : 0.0;
    pose.pose.position.z = vals.size() > 2 ? vals[2] : 0.0;
    pose.pose.orientation.w = 1.0; // 默认朝向
  }

  return pose;
}

NodeStatus Nav2Pose::onStart()
{
  goal_accepted_ = false;
  goal_done_ = false;
  goal_succeeded_ = false;

  // 尝试从端口获取 PoseStamped 或字符串
  geometry_msgs::msg::PoseStamped goal_pose;

  auto pose_result = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (pose_result) {
    goal_pose = pose_result.value();
  } else {
    // 尝试作为字符串解析
    auto str_result = getInput<std::string>("goal");
    if (str_result) {
      goal_pose = parsePoseString(str_result.value());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Nav2Pose: 无法读取 goal 端口");
      return NodeStatus::FAILURE;
    }
  }

  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = "map";

  // 等待 action server
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), "Nav2Pose: navigate_to_pose action server 未就绪");
    return NodeStatus::FAILURE;
  }

  // 发送 goal
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
      [this](const GoalHandle::SharedPtr &goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(node_->get_logger(), "Nav2Pose: Goal 被服务器拒绝");
          goal_accepted_ = false;
          goal_done_ = true;
        } else {
          goal_handle_ = goal_handle;
          goal_accepted_ = true;
          RCLCPP_DEBUG(node_->get_logger(), "Nav2Pose: Goal 已被接受");
        }
      };

  send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult &result) {
        goal_done_ = true;
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          goal_succeeded_ = true;
          RCLCPP_INFO(node_->get_logger(), "Nav2Pose: 导航成功");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_WARN(node_->get_logger(), "Nav2Pose: 导航被中止");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(node_->get_logger(), "Nav2Pose: 导航已取消");
          break;
        default:
          RCLCPP_WARN(node_->get_logger(), "Nav2Pose: 未知结果");
          break;
        }
      };

  RCLCPP_INFO(node_->get_logger(), "Nav2Pose: 发送导航目标 (%.2f, %.2f)",
              goal_pose.pose.position.x, goal_pose.pose.position.y);

  nav_client_->async_send_goal(goal_msg, send_goal_options);

  return NodeStatus::RUNNING;
}

NodeStatus Nav2Pose::onRunning()
{
  rclcpp::spin_some(node_);

  if (goal_done_) {
    return goal_succeeded_ ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  return NodeStatus::RUNNING;
}

void Nav2Pose::onHalted()
{
  if (goal_handle_ && !goal_done_) {
    RCLCPP_INFO(node_->get_logger(), "Nav2Pose: 取消当前导航目标");
    nav_client_->async_cancel_goal(goal_handle_);
  }
}

PortsList Nav2Pose::providedPorts()
{
  return {
      InputPort<geometry_msgs::msg::PoseStamped>("goal", "导航目标点 (PoseStamped 或 \"x,y,z,qx,qy,qz,qw\" 字符串)"),
  };
}

} // namespace rm_decision
