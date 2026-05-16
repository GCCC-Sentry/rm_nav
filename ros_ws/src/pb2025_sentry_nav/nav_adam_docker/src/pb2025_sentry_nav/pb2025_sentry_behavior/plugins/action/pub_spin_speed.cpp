// Copyright 2025 Lihan Chen
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/action/pub_spin_speed.hpp"
#include <thread> // 必须添加
#include <chrono> // 必须添加
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

PublishSpinSpeedAction::PublishSpinSpeedAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<example_interfaces::msg::Float32>(name, config, params)
{
}

BT::PortsList PublishSpinSpeedAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("spin_speed", 0.0, "Angular Z velocity (rad/s)"),
    // 修改点 1: 类型改为 int
    BT::InputPort<int>("duration", "Publish then sleep duration in milliseconds")
  });
}

bool PublishSpinSpeedAction::setMessage(example_interfaces::msg::Float32 & msg)
{
  double spin_speed;
  if (!getInput("spin_speed", spin_speed)) {
    return false;
  }
  msg.data = static_cast<float>(spin_speed);

  RCLCPP_INFO(
    node_->get_logger(),
    "【动作】%s -> 发布 cmd_spin=%.2f rad/s",
    name().c_str(),
    spin_speed);

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
CreateRosNodePlugin(pb2025_sentry_behavior::PublishSpinSpeedAction, "PublishSpinSpeed");
