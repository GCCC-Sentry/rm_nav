// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"

#include <chrono>
#include <sstream>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

PubNav2Goal::PubNav2Goal(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, config, params)
{
}

BT::PortsList PubNav2Goal::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("x", 0.0, "Goal X coordinate"),
    BT::InputPort<double>("y", 0.0, "Goal Y coordinate"),
    BT::InputPort<double>("yaw", 0.0, "Goal Yaw angle (rad)"),
    BT::InputPort<std::string>("step_name", "发送导航目标", "Readable step name")
  });
}

bool PubNav2Goal::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  double x, y, yaw;
  std::string step_name = "发送导航目标";
  if (!getInput("x", x) || !getInput("y", y) || !getInput("yaw", yaw)) {
    return false;
  }
  getInput("step_name", step_name);

  msg.header.frame_id = "map";
  msg.header.stamp = node_->now();
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  msg.pose.orientation = tf2::toMsg(q);

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_).count();
  if (elapsed >= 2000) {
    last_print_time_ = now;

    std::stringstream ss;
    ss << "【发导航点】" << step_name
       << " -> x=" << x
       << ", y=" << y
       << ", yaw=" << yaw;
    RCLCPP_INFO(logger_, "%s", ss.str().c_str());
  }

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2Goal, "PubNav2Goal");
