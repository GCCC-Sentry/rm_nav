// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_twist.hpp"
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

PublishTwistAction::PublishTwistAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params)
{
}

BT::PortsList PublishTwistAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("v_x", 0.0, "Linear X velocity (m/s)"),
    BT::InputPort<double>("v_y", 0.0, "Linear Y velocity (m/s)"),
    BT::InputPort<double>("v_yaw", 0.0, "Angular Z velocity (rad/s)"),
    // 修改点 1: 类型改为 int
    BT::InputPort<int>("duration", "Publish then sleep duration in milliseconds")
  });
}

bool PublishTwistAction::setMessage(geometry_msgs::msg::Twist & msg)
{
  double v_x, v_y, v_yaw;
  if (!getInput("v_x", v_x) || !getInput("v_y", v_y) || !getInput("v_yaw", v_yaw)) {
    return false;
  }

  msg.linear.x = v_x;
  msg.linear.y = v_y;
  msg.angular.z = v_yaw;

  RCLCPP_INFO(
    node_->get_logger(),
    "【动作】%s -> 发布 cmd_vel x=%.2f y=%.2f yaw=%.2f",
    name().c_str(),
    v_x,
    v_y,
    v_yaw);

  // 修改点 2: 读取 int 并转换为 milliseconds
  auto duration_ms = getInput<int>("duration");
  if(duration_ms && duration_ms.value() > 0)
  {
      std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms.value()));
  }
  
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishTwistAction, "PublishTwist");
