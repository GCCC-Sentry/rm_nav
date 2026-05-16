// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/send_nav2_goal.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace pb2025_sentry_behavior
{

// 全局日志文件流
static std::ofstream log_file;
static std::string log_file_path = "/root/ros_ws/src/behavior_tree_log.txt";
static int goal_counter = 0;

// 获取当前时间字符串
std::string getCurrentTimeString() {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
  ss << "." << std::setfill('0') << std::setw(3) << ms.count();
  return ss.str();
}

// 写日志到文件和终端
void logBoth(rclcpp::Logger logger, const std::string& message, bool is_error = false) {
  // 打开日志文件（追加模式）
  if (!log_file.is_open()) {
    log_file.open(log_file_path, std::ios::app);
    if (log_file.is_open()) {
      log_file << "\n\n========================================\n";
      log_file << "行为树日志启动时间: " << getCurrentTimeString() << "\n";
      log_file << "========================================\n\n";
    }
  }
  
  std::string time_str = getCurrentTimeString();
  std::string full_message = "[" + time_str + "] " + message;
  
  // 写入文件
  if (log_file.is_open()) {
    log_file << full_message << std::endl;
    log_file.flush();  // 立即刷新到磁盘
  }
  
  // 输出到终端
  if (is_error) {
    RCLCPP_ERROR(logger, "%s", message.c_str());
  } else {
    RCLCPP_INFO(logger, "%s", message.c_str());
  }
}

SendNav2GoalAction::SendNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
{
  logBoth(logger(), "=== SendNav2Goal 构造函数开始 ===");
  logBoth(logger(), "  节点名称: " + name);
  logBoth(logger(), "  Action名称: " + params.default_port_value);
  logBoth(logger(), "=== SendNav2Goal 构造函数结束 ===");
}

bool SendNav2GoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
{
  goal_counter++;
  
  // 1. 读取目标坐标
  double x, y, yaw;
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    logBoth(logger(), "❌ 错误：缺少目标坐标 (x, y, yaw)", true);
    return false;
  }

  current_step_name_ = name();
  getInput("step_name", current_step_name_);

  std::stringstream ss;
  ss << "\n╔════════════════════════════════════════════════════════╗";
  logBoth(logger(), ss.str());
  
  ss.str("");
  ss << "║  【行为步骤 #" << goal_counter << "】" << current_step_name_;
  while (ss.str().length() < 56) ss << " ";
  ss << "║";
  logBoth(logger(), ss.str());
  
  ss.str("");
  ss << "║  目标坐标: X=" << std::fixed << std::setprecision(2) << x 
     << "m, Y=" << y << "m, 朝向=" << yaw << "rad (" 
     << static_cast<int>(yaw * 180.0 / 3.14159) << "°)";
  // 补齐空格
  while (ss.str().length() < 56) ss << " ";
  ss << "║";
  logBoth(logger(), ss.str());
  
  logBoth(logger(), "╚════════════════════════════════════════════════════════╝");

  // 2. 构建目标
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = now();
  
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;

  // 3. 转换朝向
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation = tf2::toMsg(q);

  logBoth(logger(), "✓ 目标设置成功，正在发送到Nav2导航系统...");
  
  return true;
}

BT::NodeStatus SendNav2GoalAction::onResultReceived(const WrappedResult & wr)
{
  logBoth(logger(), "\n╔════════════════════════════════════════════════════════╗");
  
  switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      logBoth(logger(), ("║  ✓✓✓ 步骤成功：" + current_step_name_).c_str());
      logBoth(logger(), "║  已到达目标点                                        ║");
      logBoth(logger(), "╚════════════════════════════════════════════════════════╝\n");
      return BT::NodeStatus::SUCCESS;
      
    case rclcpp_action::ResultCode::ABORTED:
      logBoth(logger(), ("║  ❌ 步骤失败：" + current_step_name_).c_str(), true);
      logBoth(logger(), "║  导航中止                                            ║", true);
      logBoth(logger(), "╚════════════════════════════════════════════════════════╝\n", true);
      return BT::NodeStatus::FAILURE;
      
    case rclcpp_action::ResultCode::CANCELED:
      logBoth(logger(), ("║  ⚠ 步骤取消：" + current_step_name_).c_str());
      logBoth(logger(), "║  导航取消                                            ║");
      logBoth(logger(), "╚════════════════════════════════════════════════════════╝\n");
      return BT::NodeStatus::FAILURE;
      
    default:
      {
        std::stringstream ss;
        ss << "║  ❌ 未知结果代码: " << static_cast<int>(wr.code);
        while (ss.str().length() < 56) ss << " ";
        ss << "║";
        logBoth(logger(), ss.str(), true);
        logBoth(logger(), "╚════════════════════════════════════════════════════════╝\n", true);
      }
      return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SendNav2GoalAction::onFeedback(
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
  // 安全检查：确保feedback不为空
  if (!feedback) {
    return BT::NodeStatus::RUNNING;
  }
  
  // 每3秒打印一次进度
  static auto last_print = std::chrono::steady_clock::now();
  auto now_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now_time - last_print).count();
  
  if (elapsed >= 3) {
    try {
      std::stringstream ss;
      ss << "  【导航进度】剩余距离: " << std::fixed << std::setprecision(2) 
         << feedback->distance_remaining << "米";
      logBoth(logger(), ss.str());
      last_print = now_time;
    } catch (...) {
      // 捕获任何异常，防止崩溃
      RCLCPP_WARN(logger(), "获取导航进度时出错");
    }
  }
  
  return BT::NodeStatus::RUNNING;
}

void SendNav2GoalAction::onHalt() {
    logBoth(logger(), "⚠ SendNav2Goal 被中止 - 正在取消导航目标");
    if (!current_step_name_.empty()) {
      logBoth(logger(), "  → 当前中止步骤: " + current_step_name_);
    }
    // 基类会自动调用cancelGoal()，这里添加日志确认
    logBoth(logger(), "  → 已发送取消请求到Nav2，等待Nav2处理...");
}

BT::NodeStatus SendNav2GoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  logBoth(logger(), "\n╔════════════════════════════════════════════════════════╗", true);
  logBoth(logger(), "║  ❌ 导航失败                                          ║", true);
  
  std::stringstream ss;
  ss << "║  错误代码: " << BT::toStr(error);
  while (ss.str().length() < 56) ss << " ";
  ss << "║";
  logBoth(logger(), ss.str(), true);
  
  switch(error) {
    case BT::SERVER_UNREACHABLE:
      logBoth(logger(), "║  原因: Nav2导航服务器无法访问                        ║", true);
      logBoth(logger(), "║  解决方案: 检查Nav2是否正在运行                      ║", true);
      break;
    case BT::SEND_GOAL_TIMEOUT:
      logBoth(logger(), "║  原因: Nav2响应超时                                   ║", true);
      logBoth(logger(), "║  解决方案: 系统将自动重试...                         ║", true);
      break;
    case BT::GOAL_REJECTED_BY_SERVER:
      logBoth(logger(), "║  原因: Nav2拒绝了目标                                 ║", true);
      logBoth(logger(), "║  解决方案: 检查目标坐标是否有效                      ║", true);
      break;
    case BT::ACTION_ABORTED:
      logBoth(logger(), "║  原因: Nav2中止了导航                                 ║", true);
      logBoth(logger(), "║  解决方案: 检查是否有障碍物或地图问题                ║", true);
      break;
    case BT::ACTION_CANCELLED:
      logBoth(logger(), "║  原因: 导航被取消                                     ║", true);
      break;
    case BT::INVALID_GOAL:
      logBoth(logger(), "║  原因: 目标坐标无效                                   ║", true);
      break;
  }
  
  logBoth(logger(), "╚════════════════════════════════════════════════════════╝\n", true);
  
  return BT::NodeStatus::FAILURE;
}

BT::PortsList SendNav2GoalAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("x", 0.0, "Goal X"),
    BT::InputPort<double>("y", 0.0, "Goal Y"),
    BT::InputPort<double>("yaw", 0.0, "Goal Yaw (rad)"),
    BT::InputPort<std::string>("step_name", "", "Human-readable step name for runtime logs")
  });
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::SendNav2GoalAction, "SendNav2Goal");
