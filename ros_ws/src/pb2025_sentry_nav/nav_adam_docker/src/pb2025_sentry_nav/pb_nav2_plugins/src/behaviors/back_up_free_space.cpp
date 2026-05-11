// Copyright 2024 Polaris Xia
// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace pb_nav2_behaviors
{

namespace
{

float normalizeAngle(float angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));

  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("visualize", visualize_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
      return nav2_behaviors::Status::FAILED;
    }
    RCLCPP_WARN(logger_, "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Interrupted while waiting for the service. Exiting.");
    return nav2_behaviors::Status::FAILED;
  }

  // get costmap
  auto costmap = result.get()->map;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  // get current pose
  geometry_msgs::msg::Pose2D pose;
  pose.x = initial_pose_.pose.position.x;
  pose.y = initial_pose_.pose.position.y;
  pose.theta = tf2::getYaw(initial_pose_.pose.orientation);

  // Find the best direction to back up
  float best_angle = findBestDirection(costmap, pose, -M_PI, M_PI, max_radius_, M_PI / 32.0);
  const float relative_angle = normalizeAngle(best_angle - pose.theta);

  // Convert the selected global direction into robot-frame velocity commands.
  twist_x_ = std::cos(relative_angle) * command->speed;
  twist_y_ = std::sin(relative_angle) * command->speed;
  command_x_ = command->target.x;
  command_time_allowance_ = command->time_allowance;

  end_time_ = clock_->now() + command_time_allowance_;

  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }
  RCLCPP_WARN(
    logger_,
    "backing up %f meters towards free space at global angle %f (robot relative angle %f)",
    command_x_, best_angle, relative_angle);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Current robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  float diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
  float diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
  float distance = hypot(diff_x, diff_y);

  feedback_->distance_traveled = distance;
  action_server_->publish_feedback(feedback_);

  if (distance >= std::fabs(command_x_)) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.y = twist_y_;
  cmd_vel->linear.x = twist_x_;

  geometry_msgs::msg::Pose2D pose;
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
  pose.theta = tf2::getYaw(current_pose.pose.orientation);

  if (!isCollisionFree(distance, cmd_vel.get(), pose)) {
    stopRobot();
    RCLCPP_WARN(logger_, "Collision Ahead - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

float BackUpFreeSpace::findBestDirection(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float start_angle,
  float end_angle, float radius, float angle_increment)
{
  struct AngleCandidate
  {
    float angle;
    float clearance;
    float support;
    bool safe;
  };

  float resolution = costmap.metadata.resolution;
  float origin_x = costmap.metadata.origin.position.x;
  float origin_y = costmap.metadata.origin.position.y;
  int size_x = costmap.metadata.size_x;
  int size_y = costmap.metadata.size_y;

  float map_min_x = origin_x;
  float map_max_x = origin_x + (size_x * resolution);
  float map_min_y = origin_y;
  float map_max_y = origin_y + (size_y * resolution);
  const auto free_points = gatherFreePoints(costmap, pose, radius);
  const float cone_half_angle = 0.35f;

  std::vector<AngleCandidate> candidates;
  candidates.reserve(static_cast<size_t>((end_angle - start_angle) / angle_increment) + 1);

  float max_clearance = resolution;
  float max_support = 0.0f;

  for (float angle = start_angle; angle <= end_angle; angle += angle_increment) {
    bool is_safe = true;
    float clearance = 0.0f;

    for (float r = 0; r <= radius; r += resolution) {
      float x = pose.x + r * std::cos(angle);
      float y = pose.y + r * std::sin(angle);

      if (x >= map_min_x && x <= map_max_x && y >= map_min_y && y <= map_max_y) {
        int i = static_cast<int>((x - origin_x) / resolution);
        int j = static_cast<int>((y - origin_y) / resolution);

        if (i >= 0 && i < size_x && j >= 0 && j < size_y) {
          if (costmap.data[i + j * size_x] >= 253) {
            is_safe = false;
            break;
          } else {
            clearance = r;
          }
        } else {
          is_safe = false;
          break;
        }
      } else {
        is_safe = false;
        break;
      }
    }

    float support = 0.0f;
    if (is_safe) {
      for (const auto & point : free_points) {
        const float dx = point.x - pose.x;
        const float dy = point.y - pose.y;
        const float dist = std::hypot(dx, dy);
        if (dist < resolution || dist > radius) {
          continue;
        }

        const float point_angle = std::atan2(dy, dx);
        const float delta = std::fabs(normalizeAngle(point_angle - angle));
        if (delta > cone_half_angle) {
          continue;
        }

        const float alignment = std::cos(delta);
        support += (alignment * alignment) /
          std::max(dist * dist, resolution * resolution);
      }
    }

    candidates.push_back({angle, clearance, support, is_safe});
    if (is_safe) {
      max_clearance = std::max(max_clearance, clearance);
      max_support = std::max(max_support, support);
    }
  }

  float best_angle = start_angle;
  float best_score = -1.0f;
  int best_index = -1;

  for (size_t idx = 0; idx < candidates.size(); ++idx) {
    const auto & candidate = candidates[idx];
    if (!candidate.safe) {
      continue;
    }

    const float clearance_score = candidate.clearance / std::max(max_clearance, resolution);
    const float support_score =
      (max_support > 0.0f) ? (candidate.support / max_support) : 0.0f;
    const float score = 0.45f * clearance_score + 0.55f * support_score;

    if (score > best_score) {
      best_score = score;
      best_angle = candidate.angle;
      best_index = static_cast<int>(idx);
    }
  }

  if (best_index == -1) {
    if (!free_points.empty()) {
      float nearest_dist = std::numeric_limits<float>::max();
      for (const auto & point : free_points) {
        const float dx = point.x - pose.x;
        const float dy = point.y - pose.y;
        const float dist = std::hypot(dx, dy);
        if (dist < nearest_dist) {
          nearest_dist = dist;
          best_angle = std::atan2(dy, dx);
        }
      }
    }
    best_index = 0;
  }

  float final_safe_angle = best_angle;
  float final_unsafe_angle = best_angle;
  if (!candidates.empty()) {
    int left = best_index;
    while (left > 0 && candidates[left - 1].safe) {
      --left;
    }
    int right = best_index;
    while (right + 1 < static_cast<int>(candidates.size()) && candidates[right + 1].safe) {
      ++right;
    }
    final_safe_angle = candidates[left].angle;
    final_unsafe_angle = candidates[right].angle;
  }

  if (visualize_) {
    const float relative_angle = normalizeAngle(best_angle - pose.theta);
    visualize(costmap, pose, radius, final_safe_angle, final_unsafe_angle, best_angle, relative_angle);
  }

  return best_angle;
}

std::vector<geometry_msgs::msg::Point> BackUpFreeSpace::gatherFreePoints(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius)
{
  std::vector<geometry_msgs::msg::Point> results;
  for (unsigned int i = 0; i < costmap.metadata.size_x; i++) {
    for (unsigned int j = 0; j < costmap.metadata.size_y; j++) {
      auto idx = i + j * costmap.metadata.size_x;
      auto x = i * costmap.metadata.resolution + costmap.metadata.origin.position.x;
      auto y = j * costmap.metadata.resolution + costmap.metadata.origin.position.y;
      if (std::hypot(x - pose.x, y - pose.y) <= radius && costmap.data[idx] == 0) {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        results.push_back(p);
      }
    }
  }
  return results;
}

void BackUpFreeSpace::visualize(
  const nav2_msgs::msg::Costmap & costmap, geometry_msgs::msg::Pose2D pose, float radius,
  float first_safe_angle, float last_unsafe_angle, float best_angle, float relative_angle)
{
  visualization_msgs::msg::MarkerArray markers;
  const auto stamp = clock_->now();
  const auto free_points = gatherFreePoints(costmap, pose, radius);

  visualization_msgs::msg::Marker free_points_marker;
  free_points_marker.header.frame_id = global_frame_;
  free_points_marker.header.stamp = stamp;
  free_points_marker.ns = "free_points";
  free_points_marker.id = 0;
  free_points_marker.type = visualization_msgs::msg::Marker::POINTS;
  free_points_marker.action = visualization_msgs::msg::Marker::ADD;
  free_points_marker.scale.x = 0.05;
  free_points_marker.scale.y = 0.05;
  free_points_marker.color.r = 0.1f;
  free_points_marker.color.g = 0.95f;
  free_points_marker.color.b = 0.4f;
  free_points_marker.color.a = 0.65f;
  free_points_marker.points = free_points;
  markers.markers.push_back(free_points_marker);

  visualization_msgs::msg::Marker robot_marker;
  robot_marker.header.frame_id = global_frame_;
  robot_marker.header.stamp = stamp;
  robot_marker.ns = "robot";
  robot_marker.id = 1;
  robot_marker.type = visualization_msgs::msg::Marker::SPHERE;
  robot_marker.action = visualization_msgs::msg::Marker::ADD;
  robot_marker.pose.position.x = pose.x;
  robot_marker.pose.position.y = pose.y;
  robot_marker.pose.position.z = 0.0;
  robot_marker.pose.orientation.w = 1.0;
  robot_marker.scale.x = 0.18;
  robot_marker.scale.y = 0.18;
  robot_marker.scale.z = 0.18;
  robot_marker.color.r = 1.0f;
  robot_marker.color.g = 0.75f;
  robot_marker.color.b = 0.1f;
  robot_marker.color.a = 0.95f;
  markers.markers.push_back(robot_marker);

  visualization_msgs::msg::Marker sector_marker;
  sector_marker.header.frame_id = global_frame_;
  sector_marker.header.stamp = stamp;
  sector_marker.ns = "direction";
  sector_marker.id = 2;
  sector_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  sector_marker.action = visualization_msgs::msg::Marker::ADD;
  sector_marker.scale.x = 1.0;
  sector_marker.scale.y = 1.0;
  sector_marker.scale.z = 1.0;
  sector_marker.color.r = 0.0f;
  sector_marker.color.g = 0.9f;
  sector_marker.color.b = 0.3f;
  sector_marker.color.a = 0.28f;

  if (last_unsafe_angle > first_safe_angle) {
    const float angle_step = 0.05f;
    for (float angle = first_safe_angle; angle <= last_unsafe_angle; angle += angle_step) {
      const float next_angle = std::min(angle + angle_step, last_unsafe_angle);

      geometry_msgs::msg::Point origin;
      origin.x = pose.x;
      origin.y = pose.y;
      origin.z = 0.0;

      geometry_msgs::msg::Point p1;
      p1.x = pose.x + radius * std::cos(angle);
      p1.y = pose.y + radius * std::sin(angle);
      p1.z = 0.0;

      geometry_msgs::msg::Point p2;
      p2.x = pose.x + radius * std::cos(next_angle);
      p2.y = pose.y + radius * std::sin(next_angle);
      p2.z = 0.0;

      sector_marker.points.push_back(origin);
      sector_marker.points.push_back(p1);
      sector_marker.points.push_back(p2);
    }
  }
  markers.markers.push_back(sector_marker);

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = global_frame_;
  path_marker.header.stamp = stamp;
  path_marker.ns = "planned_backup";
  path_marker.id = 3;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.scale.x = 0.06;
  path_marker.color.r = 1.0f;
  path_marker.color.g = 0.25f;
  path_marker.color.b = 0.2f;
  path_marker.color.a = 0.95f;
  geometry_msgs::msg::Point path_start;
  path_start.x = pose.x;
  path_start.y = pose.y;
  path_start.z = 0.03;
  geometry_msgs::msg::Point path_end;
  path_end.x = pose.x + radius * std::cos(best_angle);
  path_end.y = pose.y + radius * std::sin(best_angle);
  path_end.z = 0.03;
  path_marker.points.push_back(path_start);
  path_marker.points.push_back(path_end);
  markers.markers.push_back(path_marker);

  visualization_msgs::msg::Marker exec_marker;
  exec_marker.header.frame_id = global_frame_;
  exec_marker.header.stamp = stamp;
  exec_marker.ns = "executed_backup";
  exec_marker.id = 4;
  exec_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  exec_marker.action = visualization_msgs::msg::Marker::ADD;
  exec_marker.scale.x = 0.04;
  exec_marker.color.r = 1.0f;
  exec_marker.color.g = 0.9f;
  exec_marker.color.b = 0.1f;
  exec_marker.color.a = 0.95f;
  geometry_msgs::msg::Point exec_start;
  exec_start.x = pose.x;
  exec_start.y = pose.y;
  exec_start.z = 0.08;
  geometry_msgs::msg::Point exec_end;
  exec_end.x = pose.x + radius * std::cos(pose.theta + relative_angle);
  exec_end.y = pose.y + radius * std::sin(pose.theta + relative_angle);
  exec_end.z = 0.08;
  exec_marker.points.push_back(exec_start);
  exec_marker.points.push_back(exec_end);
  markers.markers.push_back(exec_marker);

  auto create_arrow = [&](float angle, int id, float r, float g, float b) {
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = global_frame_;
    arrow.header.stamp = stamp;
    arrow.ns = "direction";
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.09;
    arrow.scale.y = 0.18;
    arrow.scale.z = 0.18;
    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    arrow.color.a = 1.0;

    geometry_msgs::msg::Point start;
    start.x = pose.x;
    start.y = pose.y;
    start.z = 0.0;

    geometry_msgs::msg::Point end;
    end.x = start.x + radius * std::cos(angle);
    end.y = start.y + radius * std::sin(angle);
    end.z = 0.0;

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    return arrow;
  };

  if (last_unsafe_angle > first_safe_angle) {
    markers.markers.push_back(create_arrow(first_safe_angle, 5, 0.2f, 0.55f, 1.0f));
    markers.markers.push_back(create_arrow(last_unsafe_angle, 6, 0.2f, 0.55f, 1.0f));
  }
  markers.markers.push_back(create_arrow(best_angle, 7, 1.0f, 0.25f, 0.2f));
  markers.markers.push_back(create_arrow(pose.theta + relative_angle, 8, 1.0f, 0.9f, 0.1f));

  visualization_msgs::msg::Marker text_marker;
  text_marker.header.frame_id = global_frame_;
  text_marker.header.stamp = stamp;
  text_marker.ns = "direction_text";
  text_marker.id = 9;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position.x = pose.x;
  text_marker.pose.position.y = pose.y;
  text_marker.pose.position.z = 0.45;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.22;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;
  char label[128];
  std::snprintf(
    label, sizeof(label),
    "BackUpFreeSpace\nglobal=%.2f rad\nrelative=%.2f rad\nradius=%.2f m\nfree_pts=%zu",
    best_angle, relative_angle, radius, free_points.size());
  text_marker.text = label;
  markers.markers.push_back(text_marker);

  marker_pub_->publish(markers);
}

}  // namespace pb_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(pb_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
