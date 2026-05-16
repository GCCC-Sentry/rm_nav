// Copyright 2026
// Licensed under the Apache License, Version 2.0

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_POSE_NEAR_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_POSE_NEAR_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

class IsPoseNearCondition : public BT::SimpleConditionNode
{
public:
  IsPoseNearCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus checkPoseNear();

  std::shared_ptr<rclcpp::Node> helper_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsPoseNearCondition");
  std::chrono::steady_clock::time_point last_print_time_ = std::chrono::steady_clock::now();
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_POSE_NEAR_HPP_
