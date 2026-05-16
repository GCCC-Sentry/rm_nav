// Copyright 2026
// Licensed under the Apache License, Version 2.0

#include "pb2025_sentry_behavior/plugins/condition/is_pose_near.hpp"

#include <cctype>
#include <cmath>
#include <sstream>

#include "tf2/exceptions.h"

namespace pb2025_sentry_behavior
{

IsPoseNearCondition::IsPoseNearCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPoseNearCondition::checkPoseNear, this), config)
{
  std::string helper_name = "is_pose_near_" + name;
  for (auto & ch : helper_name) {
    if (!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_') {
      ch = '_';
    }
  }
  helper_node_ = std::make_shared<rclcpp::Node>(helper_name);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(helper_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, helper_node_, true);
  RCLCPP_INFO(logger_, "【条件节点】IsPoseNear 初始化完成");
}

BT::NodeStatus IsPoseNearCondition::checkPoseNear()
{
  double target_x = 0.0;
  double target_y = 0.0;
  double tolerance = 0.35;
  std::string step_name = "目标点";
  std::string target_frame = "map";
  std::string base_frame = "base_footprint";
  double transform_timeout = 0.2;

  getInput("x", target_x);
  getInput("y", target_y);
  getInput("tolerance", tolerance);
  getInput("step_name", step_name);
  getInput("target_frame", target_frame);
  getInput("base_frame", base_frame);
  getInput("transform_timeout", transform_timeout);

  double current_x = 0.0;
  double current_y = 0.0;
  try {
    const auto tf = tf_buffer_->lookupTransform(
      target_frame, base_frame, tf2::TimePointZero,
      tf2::durationFromSec(transform_timeout));
    current_x = tf.transform.translation.x;
    current_y = tf.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *helper_node_->get_clock(), 3000,
      "⚠ IsPoseNear[%s]: TF %s -> %s 不可用: %s",
      step_name.c_str(), target_frame.c_str(), base_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double distance = std::hypot(current_x - target_x, current_y - target_y);
  const bool reached = distance <= tolerance;

  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_print_time_).count();
  if (elapsed >= 2000) {
    last_print_time_ = now;

    std::stringstream ss;
    ss << "IsPoseNear[" << step_name << "]: "
       << "当前位置=(" << current_x << ", " << current_y << ")"
       << " | 目标=(" << target_x << ", " << target_y << ")"
       << " | 距离=" << distance << "m"
       << " | 阈值=" << tolerance << "m"
       << " → " << (reached ? "SUCCESS(已到点)" : "FAILURE(未到点)");

    if (reached) {
      RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    } else {
      RCLCPP_INFO(logger_, "%s", ss.str().c_str());
    }
  }

  return reached ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsPoseNearCondition::providedPorts()
{
  return {
    BT::InputPort<double>("x", 0.0, "Target X coordinate"),
    BT::InputPort<double>("y", 0.0, "Target Y coordinate"),
    BT::InputPort<double>("tolerance", 0.35, "Reach tolerance in meters"),
    BT::InputPort<std::string>("target_frame", "map", "Target frame for TF lookup"),
    BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base frame for TF lookup"),
    BT::InputPort<double>("transform_timeout", 0.2, "TF lookup timeout in seconds"),
    BT::InputPort<std::string>("step_name", "目标点", "Readable target name")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsPoseNearCondition>("IsPoseNear");
}
