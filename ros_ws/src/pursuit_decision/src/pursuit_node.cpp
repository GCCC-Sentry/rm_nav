#include "pursuit_decision/pursuit_node.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include <sstream>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pursuit_decision
{

PursuitNode::PursuitNode(const rclcpp::NodeOptions & options)
: Node("pursuit_decision", options)
{
  declare_and_load_parameters();
  initialize_zones();

  // 配置组件
  target_validator_.configure(
    params_.confirm_frames,
    params_.confirm_timeout,
    params_.pnp_noise_threshold,
    params_.max_position_jump);

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // === Subscribers ===
  aim_target_sub_ = this->create_subscription<std_msgs::msg::String>(
    "auto_aim_target_pos", rclcpp::SensorDataQoS(),
    std::bind(&PursuitNode::on_aim_target, this, std::placeholders::_1));

  // 裁判系统数据 - 使用通用String topic, 可按实际情况替换为具体msg类型
  robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "pursuit/robot_status", 10,
    std::bind(&PursuitNode::on_robot_status, this, std::placeholders::_1));

  game_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "pursuit/game_status", 10,
    std::bind(&PursuitNode::on_game_status, this, std::placeholders::_1));

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", rclcpp::QoS(1).transient_local(),
    std::bind(&PursuitNode::on_costmap, this, std::placeholders::_1));

  // === Publishers ===
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
  target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "pursuit_target_pose", 10);
  attack_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "pursuit_attack_pose", 10);

  pursuit_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "pursuit_status", 10);
  debug_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "pursuit_debug_markers", 10);
  goal_history_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "pursuit_goal_history", 10);

  // === Timers ===
  // 主决策循环: 20Hz
  decision_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&PursuitNode::decision_callback, this));

  // 目标更新: 按参数配置的频率
  auto goal_period_ms = static_cast<int>(1000.0 / params_.goal_update_rate);
  goal_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(goal_period_ms),
    std::bind(&PursuitNode::goal_update_callback, this));

  RCLCPP_INFO(this->get_logger(),
    "追击决策节点已启动. 触发距离=%.1fm, 确认帧数=%d, 最大追击时长=%.0fs, 区域检查=%s",
    params_.trigger_distance, params_.confirm_frames, params_.max_pursuit_duration,
    params_.enable_zone_check ? "开" : "关");

  goal_history_.header.frame_id = params_.map_frame_id;
}

// ============================================================
//  参数声明与加载
// ============================================================

void PursuitNode::declare_and_load_parameters()
{
  // 追击触发
  this->declare_parameter("trigger_distance", params_.trigger_distance);
  this->declare_parameter("confirm_frames", params_.confirm_frames);
  this->declare_parameter("confirm_timeout", params_.confirm_timeout);

  // 追击行为
  this->declare_parameter("goal_update_rate", params_.goal_update_rate);
  this->declare_parameter("max_pursuit_duration", params_.max_pursuit_duration);
  this->declare_parameter("target_lost_timeout", params_.target_lost_timeout);

  // 撤退
  this->declare_parameter("hp_retreat_threshold", params_.hp_retreat_threshold);

  // 坐标系
  this->declare_parameter("map_frame_id", params_.map_frame_id);
  this->declare_parameter("robot_frame_id", params_.robot_frame_id);
  this->declare_parameter("transform_tolerance", params_.transform_tolerance);
  this->declare_parameter("world_to_map_yaw_offset", params_.world_to_map_yaw_offset);

  // 噪声控制
  this->declare_parameter("pnp_noise_threshold", params_.pnp_noise_threshold);
  this->declare_parameter("max_position_jump", params_.max_position_jump);

  // 图层
  this->declare_parameter("z_layer_tolerance", params_.z_layer_tolerance);
  this->declare_parameter("enable_zone_check", params_.enable_zone_check);

  // 代价地图
  this->declare_parameter("costmap_cost_threshold", params_.costmap_cost_threshold);
  this->declare_parameter("num_candidate_sectors", params_.num_candidate_sectors);

  // 撤退点
  this->declare_parameter("retreat_x", params_.retreat_x);
  this->declare_parameter("retreat_y", params_.retreat_y);
  this->declare_parameter("retreat_yaw", params_.retreat_yaw);

  // 逼近距离 (英雄/步兵/哨兵)
  this->declare_parameter("approach_distance.hero", 4.0);
  this->declare_parameter("approach_distance.engineer", 3.0);
  this->declare_parameter("approach_distance.infantry", 2.5);
  this->declare_parameter("approach_distance.sentry", 3.5);
  this->declare_parameter("approach_distance.default", 3.0);

  // 加载参数
  this->get_parameter("trigger_distance", params_.trigger_distance);
  this->get_parameter("confirm_frames", params_.confirm_frames);
  this->get_parameter("confirm_timeout", params_.confirm_timeout);
  this->get_parameter("goal_update_rate", params_.goal_update_rate);
  this->get_parameter("max_pursuit_duration", params_.max_pursuit_duration);
  this->get_parameter("target_lost_timeout", params_.target_lost_timeout);
  this->get_parameter("hp_retreat_threshold", params_.hp_retreat_threshold);
  this->get_parameter("map_frame_id", params_.map_frame_id);
  this->get_parameter("robot_frame_id", params_.robot_frame_id);
  this->get_parameter("transform_tolerance", params_.transform_tolerance);
  this->get_parameter("world_to_map_yaw_offset", params_.world_to_map_yaw_offset);
  this->get_parameter("pnp_noise_threshold", params_.pnp_noise_threshold);
  this->get_parameter("max_position_jump", params_.max_position_jump);
  this->get_parameter("z_layer_tolerance", params_.z_layer_tolerance);
  this->get_parameter("enable_zone_check", params_.enable_zone_check);
  this->get_parameter("costmap_cost_threshold", params_.costmap_cost_threshold);
  this->get_parameter("num_candidate_sectors", params_.num_candidate_sectors);
  this->get_parameter("retreat_x", params_.retreat_x);
  this->get_parameter("retreat_y", params_.retreat_y);
  this->get_parameter("retreat_yaw", params_.retreat_yaw);

  // 逼近距离映射
  double hero_dist, eng_dist, inf_dist, sentry_dist, default_dist;
  this->get_parameter("approach_distance.hero", hero_dist);
  this->get_parameter("approach_distance.engineer", eng_dist);
  this->get_parameter("approach_distance.infantry", inf_dist);
  this->get_parameter("approach_distance.sentry", sentry_dist);
  this->get_parameter("approach_distance.default", default_dist);

  params_.approach_distance[1] = hero_dist;      // 英雄
  params_.approach_distance[2] = eng_dist;        // 工程
  params_.approach_distance[3] = inf_dist;        // 步兵3
  params_.approach_distance[4] = inf_dist;        // 步兵4
  params_.approach_distance[5] = inf_dist;        // 步兵5
  params_.approach_distance[6] = sentry_dist;     // 哨兵
  params_.approach_distance[7] = default_dist;    // 默认
}

// ============================================================
//  区域初始化 (RMUC 28m x 15m)
// ============================================================

void PursuitNode::initialize_zones()
{
  // ====== RMUC 2026场地区域定义 ======
  // 注意：具体坐标需要根据实际地图原点和朝向调整
  // 这里使用的是示意坐标，需要和你的map对齐
  //
  // 场地大致布局 (map frame, 原点在场地中心):
  //   - 己方半场: x < 0 (红方) 或 x > 0 (蓝方)
  //   - 中央高地: 场地中央高台
  //   - 对方半场: x > 0 (红方) 或 x < 0 (蓝方)
  //
  // 根据你的map文件实际坐标调整这些值！

  std::vector<Zone> zones;

  // 己方半场 (示意：左半场)
  zones.push_back({
    "own_half",
    {{-14.0, -7.5}, {0.0, -7.5}, {0.0, 7.5}, {-14.0, 7.5}},
    -0.5, 0.5,      // 地面高度范围
    false,
    LayerId::OWN_HALF
  });

  // 对方半场 (示意：右半场)
  zones.push_back({
    "enemy_half",
    {{0.0, -7.5}, {14.0, -7.5}, {14.0, 7.5}, {0.0, 7.5}},
    -0.5, 0.5,
    false,
    LayerId::ENEMY_HALF
  });

  // 中央高地 (示意)
  zones.push_back({
    "central_highland",
    {{-3.0, -3.0}, {3.0, -3.0}, {3.0, 3.0}, {-3.0, 3.0}},
    0.3, 1.5,        // 高地高度
    false,
    LayerId::CENTRAL_HIGHLAND
  });

  // ====== 禁区定义 ======
  // 对方补给区 (敌方半场的补给站附近)
  zones.push_back({
    "enemy_supply_zone",
    {{12.0, -3.0}, {14.0, -3.0}, {14.0, 0.0}, {12.0, 0.0}},
    -1.0, 2.0,
    true,              // 禁区！
    LayerId::ENEMY_HALF
  });

  // 颠簸路段1
  zones.push_back({
    "bumpy_road_1",
    {{-2.0, -7.5}, {2.0, -7.5}, {2.0, -5.0}, {-2.0, -5.0}},
    -1.0, 2.0,
    true,              // 禁区
    LayerId::OWN_HALF
  });

  // 颠簸路段2
  zones.push_back({
    "bumpy_road_2",
    {{-2.0, 5.0}, {2.0, 5.0}, {2.0, 7.5}, {-2.0, 7.5}},
    -1.0, 2.0,
    true,
    LayerId::OWN_HALF
  });

  zone_manager_.configure(zones, params_.z_layer_tolerance);

  RCLCPP_INFO(this->get_logger(), "区域管理器已配置: %zu 个区域 (含禁区)", zones.size());
}

// ============================================================
//  回调函数
// ============================================================

void PursuitNode::on_aim_target(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[收到自瞄数据] raw: '%s'", msg->data.c_str());

  // 解析格式: "x,y,z,armor_id,tracking_state"
  // 兼容旧格式: "x,y,1,armor_id"
  std::stringstream ss(msg->data);
  std::string token;
  std::vector<double> values;

  while (std::getline(ss, token, ',')) {
    try {
      values.push_back(std::stod(token));
    } catch (...) {
      RCLCPP_WARN(this->get_logger(),
        "[自瞄数据] 解析失败, token='%s', raw='%s'", token.c_str(), msg->data.c_str());
      return;  // 解析失败
    }
  }

  if (values.size() < 4) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[自瞄数据] 字段不足: size=%zu, 需要>=4, raw='%s'",
      values.size(), msg->data.c_str());
    return;
  }

  // 格式A (5字段): "world_x,world_y,world_z,armor_id,tracking_state"
  // 格式B (4字段): "world_x,world_y,world_z,armor_id" (tracking_state默认=1)
  double world_x = values[0];
  double world_y = values[1];
  double world_z = (values.size() >= 4) ? values[2] : 0.0;
  int armor_id = (values.size() >= 5)
    ? static_cast<int>(std::round(values[3]))
    : static_cast<int>(std::round(values[values.size() - 1]));
  int tracking_state = (values.size() >= 5) ? static_cast<int>(std::round(values[4])) : 1;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[自瞄数据] 解析结果: world=(%.3f, %.3f, %.3f), armor_id=%d, tracking_state=%d",
    world_x, world_y, world_z, armor_id, tracking_state);

  // 全零意味着没有有效目标
  if (std::abs(world_x) < 1e-6 && std::abs(world_y) < 1e-6 && armor_id == 0) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[自瞄数据] 全零数据, 忽略");
    return;
  }

  // 将IMU世界坐标系下的相对向量转换到map frame下的绝对位置
  // 不依赖动态云台TF, 解决宿主机auto_aim与Docker容器导航的解耦问题
  double mx, my, mz;
  if (!world_vector_to_map(world_x, world_y, world_z, mx, my, mz)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[自瞄数据] world_vector_to_map失败, 可能TF不可用");
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[坐标转换] world(%.3f,%.3f,%.3f) -> map(%.3f,%.3f,%.3f)",
    world_x, world_y, world_z, mx, my, mz);

  has_last_raw_target_map_ = true;
  last_raw_target_map_.x = mx;
  last_raw_target_map_.y = my;
  last_raw_target_map_.z = mz;

  // 构造观测并加入验证器
  TargetObservation obs;
  obs.armor_id = armor_id;
  obs.x = mx;
  obs.y = my;
  obs.z = mz;
  obs.tracking_state = tracking_state;
  obs.timestamp = std::chrono::steady_clock::now();

  target_validator_.add_observation(obs);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[验证器] 已添加观测, confirmed=%s, confirmed_id=%d, time_since_last=%.2fs",
    target_validator_.is_confirmed() ? "YES" : "NO",
    target_validator_.get_confirmed_target_id(),
    target_validator_.time_since_last_observation());
}

void PursuitNode::on_robot_status(const std_msgs::msg::String::SharedPtr msg)
{
  // 解析JSON: {"hp": 600, "heat": 0, "ammo": 500}
  try {
    auto j = nlohmann::json::parse(msg->data);
    if (j.contains("hp")) robot_status_.hp = j["hp"].get<double>();
    if (j.contains("heat")) robot_status_.heat = j["heat"].get<double>();
    if (j.contains("ammo")) robot_status_.ammo = j["ammo"].get<double>();
    robot_status_.last_update = std::chrono::steady_clock::now();
  } catch (...) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "robot_status JSON解析失败");
  }
}

void PursuitNode::on_game_status(const std_msgs::msg::String::SharedPtr msg)
{
  // 解析JSON: {"progress": 4, "remaining_time": 350}
  try {
    auto j = nlohmann::json::parse(msg->data);
    if (j.contains("progress")) game_status_.progress = j["progress"].get<int>();
    if (j.contains("remaining_time"))
      game_status_.remaining_time = j["remaining_time"].get<double>();
    game_status_.last_update = std::chrono::steady_clock::now();
  } catch (...) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "game_status JSON解析失败");
  }
}

void PursuitNode::on_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  costmap_ = msg;
}

// ============================================================
//  核心决策循环 (20Hz)
// ============================================================

void PursuitNode::decision_callback()
{
  // 每5秒打印一次当前总体状态
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "[决策循环] 当前状态=%d(%s), game_progress=%d, 目标confirmed=%s, confirmed_id=%d, hp=%.0f",
    static_cast<int>(state_),
    (state_ == PursuitState::IDLE ? "IDLE" :
     state_ == PursuitState::CONFIRMING ? "CONFIRMING" :
     state_ == PursuitState::PURSUING ? "PURSUING" : "RETREATING"),
    game_status_.progress,
    target_validator_.is_confirmed() ? "YES" : "NO",
    target_validator_.get_confirmed_target_id(),
    robot_status_.hp);

  // 只在比赛运行期间进行追击决策 (progress == 4)
  if (game_status_.progress != 0 && game_status_.progress != 4) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[决策循环] 比赛未运行 (progress=%d, 需要0或4), 不进行追击决策",
      game_status_.progress);
    if (state_ != PursuitState::IDLE) {
      transition_to(PursuitState::IDLE);
    }
    return;
  }

  switch (state_) {
    case PursuitState::IDLE: {
      // 检查是否有确认的目标可以追击
      double tx, ty, tz;
      bool confirmed = target_validator_.is_confirmed();
      bool has_pos = target_validator_.get_filtered_position(tx, ty, tz);

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[IDLE] confirmed=%s, has_filtered_pos=%s%s",
        confirmed ? "YES" : "NO",
        has_pos ? "YES" : "NO",
        has_pos ? (std::string(" pos=(" + std::to_string(tx) + "," + std::to_string(ty) + "," + std::to_string(tz) + ")").c_str()) : "");

      if (confirmed && has_pos)
      {
        if (should_start_pursuit(tx, ty, tz)) {
          RCLCPP_INFO(this->get_logger(),
            "[IDLE] 追击条件满足! 目标位置=(%.2f,%.2f,%.2f), 进入CONFIRMING",
            tx, ty, tz);
          transition_to(PursuitState::CONFIRMING);
        }
      }
      break;
    }

    case PursuitState::CONFIRMING: {
      // 二次确认: 检查目标是否仍然存在且有效
      double tx, ty, tz;
      bool still_confirmed = target_validator_.is_confirmed();
      bool still_has_pos = target_validator_.get_filtered_position(tx, ty, tz);
      RCLCPP_INFO(this->get_logger(),
        "[CONFIRMING] confirmed=%s, has_pos=%s",
        still_confirmed ? "YES" : "NO", still_has_pos ? "YES" : "NO");

      if (!still_confirmed || !still_has_pos)
      {
        // 确认失败，回到IDLE
        RCLCPP_WARN(this->get_logger(), "[CONFIRMING] 确认失败, 回到IDLE");
        transition_to(PursuitState::IDLE);
        break;
      }

      // 通过所有检查，开始追击
      current_pursuit_target_id_ = target_validator_.get_confirmed_target_id();
      pursuit_start_time_ = std::chrono::steady_clock::now();
      transition_to(PursuitState::PURSUING);

      RCLCPP_INFO(this->get_logger(),
        "开始追击! 目标ID=%d, 位置=(%.2f, %.2f, %.2f)",
        current_pursuit_target_id_, tx, ty, tz);

      // 立即发布第一个追击目标点
      {
        double approach_dist = get_approach_distance(current_pursuit_target_id_);
        geometry_msgs::msg::PoseStamped goal;
        if (compute_pursuit_goal(tx, ty, approach_dist, goal)) {
          RCLCPP_INFO(this->get_logger(),
            "[追击启动] 立即发布第一个goal: (%.2f,%.2f)",
            goal.pose.position.x, goal.pose.position.y);
          has_last_goal_ = true;
          last_goal_ = goal;
          goal_history_.header.stamp = goal.header.stamp;
          goal_history_.poses.push_back(goal);
          if (goal_history_.poses.size() > 200) {
            goal_history_.poses.erase(goal_history_.poses.begin());
          }
          goal_history_pub_->publish(goal_history_);
          goal_pub_->publish(goal);
        } else {
          RCLCPP_WARN(this->get_logger(), "[追击启动] 首次compute_pursuit_goal失败");
        }
      }
      break;
    }

    case PursuitState::PURSUING: {
      // 检查撤退条件
      if (should_retreat()) {
        RCLCPP_INFO(this->get_logger(), "[追击中] 触发撤退条件");
        transition_to(PursuitState::RETREATING);
        break;
      }

      // 检查追击超时
      auto now = std::chrono::steady_clock::now();
      double pursuit_duration =
        std::chrono::duration<double>(now - pursuit_start_time_).count();

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[追击中] 已追击 %.1fs/%.0fs, target_id=%d, last_obs=%.2fs_ago",
        pursuit_duration, params_.max_pursuit_duration,
        current_pursuit_target_id_,
        target_validator_.time_since_last_observation());

      if (pursuit_duration > params_.max_pursuit_duration) {
        RCLCPP_INFO(this->get_logger(), "追击超时 (%.0fs), 撤退", pursuit_duration);
        transition_to(PursuitState::RETREATING);
        break;
      }

      // 检查目标是否丢失
      if (target_validator_.time_since_last_observation() >
          params_.target_lost_timeout)
      {
        RCLCPP_INFO(this->get_logger(),
          "[追击中] 目标丢失超过 %.1fs, 结束追击",
          params_.target_lost_timeout);
        transition_to(PursuitState::IDLE);
        break;
      }

      break;  // 目标更新在goal_update_callback中处理
    }

    case PursuitState::RETREATING: {
      // 发布撤退目标
      publish_retreat_goal();

      // 检查是否已到达撤退点附近
      double rx, ry, rz;
      if (get_robot_position(rx, ry, rz)) {
        double dx = rx - params_.retreat_x;
        double dy = ry - params_.retreat_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < 0.5) {
          RCLCPP_INFO(this->get_logger(), "已到达撤退点");
          transition_to(PursuitState::IDLE);
        }
      }

      // 撤退也有超时
      auto now = std::chrono::steady_clock::now();
      double retreat_duration =
        std::chrono::duration<double>(now - pursuit_start_time_).count();
      if (retreat_duration > params_.max_pursuit_duration * 2) {
        transition_to(PursuitState::IDLE);
      }
      break;
    }
  }

  // 每次决策循环都发布状态
  publish_pursuit_status();
  publish_debug_visualization();
}

// ============================================================
//  目标更新 (按配置频率)
// ============================================================

void PursuitNode::goal_update_callback()
{
  if (state_ != PursuitState::PURSUING) return;

  double tx, ty, tz;
  if (!target_validator_.get_filtered_position(tx, ty, tz)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[目标更新] 追击中但无法获取滤波后位置");
    return;
  }

  double approach_dist = get_approach_distance(current_pursuit_target_id_);

  geometry_msgs::msg::PoseStamped goal;
  if (compute_pursuit_goal(tx, ty, approach_dist, goal)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[目标更新] 发布追击goal: (%.2f,%.2f) -> target(%.2f,%.2f), approach_dist=%.2f",
      goal.pose.position.x, goal.pose.position.y, tx, ty, approach_dist);
    has_last_goal_ = true;
    last_goal_ = goal;
    goal_history_.header.stamp = goal.header.stamp;
    goal_history_.poses.push_back(goal);
    if (goal_history_.poses.size() > 200) {
      goal_history_.poses.erase(goal_history_.poses.begin());
    }
    goal_history_pub_->publish(goal_history_);
    goal_pub_->publish(goal);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[目标更新] compute_pursuit_goal失败, target=(%.2f,%.2f)", tx, ty);
  }
}

// ============================================================
//  决策判断
// ============================================================

bool PursuitNode::should_start_pursuit(
  double target_x, double target_y, double target_z)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[追击判断] 开始检查, 目标=(%.2f,%.2f,%.2f)", target_x, target_y, target_z);

  // 1. 获取自身位置
  double rx, ry, rz;
  if (!get_robot_position(rx, ry, rz)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[追击判断] ×获取自身位置失败(TF不可用)");
    return false;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[追击判断] 自身位置=(%.2f,%.2f,%.2f)", rx, ry, rz);

  // 2. 计算与目标的距离
  double dx = target_x - rx;
  double dy = target_y - ry;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist > params_.trigger_distance) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[追击判断] ×距离过远: %.2fm > 触发距离%.2fm", dist, params_.trigger_distance);
    return false;  // 超出追击触发距离
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[追击判断] √距离检查通过: %.2fm <= %.2fm", dist, params_.trigger_distance);

  // 3. 图层检查: 必须在同一图层 (可通过参数关闭)
  if (params_.enable_zone_check) {
    if (!zone_manager_.is_same_layer(rx, ry, rz, target_x, target_y, target_z)) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[追击判断] ×图层不同, 不追击. 自(%.1f,%.1f,%.1f) 敌(%.1f,%.1f,%.1f)",
        rx, ry, rz, target_x, target_y, target_z);
      return false;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[追击判断] √图层检查通过");

    // 4. 禁区检查: 目标不能在禁区
    if (zone_manager_.is_in_forbidden_zone(target_x, target_y)) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[追击判断] ×目标位于禁区 (%s), 不追击",
        zone_manager_.get_zone_name(target_x, target_y).c_str());
      return false;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[追击判断] √禁区检查通过(目标不在禁区)");
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "[追击判断] 区域/图层检查已禁用 (enable_zone_check=false)");
  }

  // 5. 禁区检查: 追击路径终点不能在禁区
  // (先简单检查目标附近区域，后续可以做路径检查)
  double approach_dist = get_approach_distance(
    target_validator_.get_confirmed_target_id());
  double angle = std::atan2(ry - target_y, rx - target_x);
  double goal_x = target_x + approach_dist * std::cos(angle);
  double goal_y = target_y + approach_dist * std::sin(angle);
  if (params_.enable_zone_check && zone_manager_.is_in_forbidden_zone(goal_x, goal_y)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[追击判断] ×追击目标点(%.2f,%.2f)位于禁区, 不追击", goal_x, goal_y);
    return false;
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[追击判断] √追击目标点禁区检查通过");

  // 6. 自身状态检查: 血量足够
  if (robot_status_.hp <= params_.hp_retreat_threshold) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[追击判断] ×血量不足: %.0f <= 阈值%.0f",
      robot_status_.hp, params_.hp_retreat_threshold);
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
    "[追击判断] √全部检查通过! dist=%.2f, target=(%.2f,%.2f,%.2f), robot=(%.2f,%.2f)",
    dist, target_x, target_y, target_z, rx, ry);
  return true;
}

bool PursuitNode::should_retreat()
{
  // 条件1: 血量低于阈值
  if (robot_status_.hp <= params_.hp_retreat_threshold) {
    RCLCPP_INFO(this->get_logger(),
      "[撤退检查] 血量过低 (%.0f <= %.0f)",
      robot_status_.hp, params_.hp_retreat_threshold);
    return true;
  }

  // 条件2: 目标进入禁区 (仅开启区域检查时)
  if (params_.enable_zone_check) {
    double tx, ty, tz;
    if (target_validator_.get_filtered_position(tx, ty, tz)) {
      if (zone_manager_.is_in_forbidden_zone(tx, ty)) {
        RCLCPP_INFO(this->get_logger(), "[撤退检查] 目标进入禁区");
        return true;
      }
    }
  }

  // 条件3: 目标丢失超时
  double time_since = target_validator_.time_since_last_observation();
  if (time_since > params_.target_lost_timeout) {
    RCLCPP_INFO(this->get_logger(),
      "[撤退检查] 目标丢失 %.1fs > %.1fs",
      time_since, params_.target_lost_timeout);
    return true;
  }

  // 条件4: 追击超时
  auto now = std::chrono::steady_clock::now();
  double duration = std::chrono::duration<double>(now - pursuit_start_time_).count();
  if (duration > params_.max_pursuit_duration) {
    RCLCPP_INFO(this->get_logger(),
      "[撤退检查] 追击超时 %.1fs > %.1fs",
      duration, params_.max_pursuit_duration);
    return true;
  }

  return false;
}

// ============================================================
//  坐标变换
// ============================================================

bool PursuitNode::get_robot_position(double & x, double & y, double & z)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      params_.map_frame_id, params_.robot_frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(params_.transform_tolerance));

    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    z = transform.transform.translation.z;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "获取机器人位置失败: %s", ex.what());
    return false;
  }
}

bool PursuitNode::world_vector_to_map(
  double world_x, double world_y, double world_z,
  double & mx, double & my, double & mz)
{
  // 获取机器人当前在map frame中的位置
  // xyz_in_world 是目标相对于云台在IMU世界坐标系下的向量
  // target_map = robot_map + R(yaw_offset) * xyz_in_world
  double rx, ry, rz;
  if (!get_robot_position(rx, ry, rz)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[坐标转换] get_robot_position失败, 无法进行world->map变换");
    return false;
  }

  // 应用yaw偏移量将IMU世界坐标系向量旋转到map坐标系
  double cos_off = std::cos(params_.world_to_map_yaw_offset);
  double sin_off = std::sin(params_.world_to_map_yaw_offset);

  mx = rx + cos_off * world_x - sin_off * world_y;
  my = ry + sin_off * world_x + cos_off * world_y;
  mz = rz + world_z;  // z方向近似直接相加 (重力方向一致)

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "[坐标转换] robot_map=(%.2f,%.2f,%.2f), yaw_offset=%.3f, world_vec=(%.2f,%.2f,%.2f) -> map_target=(%.2f,%.2f,%.2f)",
    rx, ry, rz, params_.world_to_map_yaw_offset, world_x, world_y, world_z, mx, my, mz);

  return true;
}

// ============================================================
//  追击目标计算 (类似CalculateAttackPose)
// ============================================================

bool PursuitNode::compute_pursuit_goal(
  double target_x, double target_y,
  double approach_dist,
  geometry_msgs::msg::PoseStamped & goal)
{
  last_candidate_points_.clear();

  double rx, ry, rz;
  if (!get_robot_position(rx, ry, rz)) {
    return false;
  }

  // 生成圆形候选点
  struct Candidate {
    double x, y;
    double cost_to_robot;
  };
  std::vector<Candidate> feasible;

  for (int i = 0; i < params_.num_candidate_sectors; ++i) {
    double angle = i * 2.0 * M_PI / params_.num_candidate_sectors;
    double cx = target_x + approach_dist * std::cos(angle);
    double cy = target_y + approach_dist * std::sin(angle);

  // 禁区检查
    if (params_.enable_zone_check) {
      if (zone_manager_.is_in_forbidden_zone(cx, cy)) continue;
    }

    // 代价地图检查 (仅在有costmap时)
    if (costmap_) {
      const auto & info = costmap_->info;
      int cell_x = static_cast<int>((cx - info.origin.position.x) / info.resolution);
      int cell_y = static_cast<int>((cy - info.origin.position.y) / info.resolution);

      if (cell_x >= 0 && cell_x < static_cast<int>(info.width) &&
          cell_y >= 0 && cell_y < static_cast<int>(info.height))
      {
        int idx = cell_y * info.width + cell_x;
        int8_t cost = costmap_->data[idx];
        if (cost < 0 || cost > params_.costmap_cost_threshold) {
          continue;  // 不可通行
        }
      }
    }

    double dx = cx - rx;
    double dy = cy - ry;
    geometry_msgs::msg::Point candidate_point;
    candidate_point.x = cx;
    candidate_point.y = cy;
    candidate_point.z = 0.0;
    last_candidate_points_.push_back(candidate_point);
    feasible.push_back({cx, cy, std::sqrt(dx * dx + dy * dy)});
  }

  if (feasible.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "没有可行的追击候选点");
    return false;
  }

  // 选择离机器人最近的可行点
  auto best = std::min_element(feasible.begin(), feasible.end(),
    [](const Candidate & a, const Candidate & b) {
      return a.cost_to_robot < b.cost_to_robot;
    });

  // 构造PoseStamped, 朝向敌人
  goal.header.frame_id = params_.map_frame_id;
  goal.header.stamp = this->now();
  goal.pose.position.x = best->x;
  goal.pose.position.y = best->y;
  goal.pose.position.z = 0.0;

  double yaw = std::atan2(target_y - best->y, target_x - best->x);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.orientation = tf2::toMsg(q);

  return true;
}

double PursuitNode::get_approach_distance(int armor_id)
{
  auto it = params_.approach_distance.find(armor_id);
  if (it != params_.approach_distance.end()) {
    return it->second;
  }
  // 默认距离
  auto def_it = params_.approach_distance.find(7);
  return (def_it != params_.approach_distance.end()) ? def_it->second : 3.0;
}

// ============================================================
//  状态转换与发布
// ============================================================

void PursuitNode::transition_to(PursuitState new_state)
{
  if (state_ == new_state) return;

  auto state_name = [](PursuitState s) -> const char* {
    switch (s) {
      case PursuitState::IDLE: return "IDLE";
      case PursuitState::CONFIRMING: return "CONFIRMING";
      case PursuitState::PURSUING: return "PURSUING";
      case PursuitState::RETREATING: return "RETREATING";
      default: return "UNKNOWN";
    }
  };

  RCLCPP_INFO(this->get_logger(), "状态转换: %s -> %s",
    state_name(state_), state_name(new_state));

  if (new_state == PursuitState::IDLE) {
    current_pursuit_target_id_ = -1;
    // 不刍调用 reset()! 保留validator buffer,
    // 否则会丢失当前观测数据导致反复 IDLE<->CONFIRMING振荡
    // target_validator_.reset(); 
  }

  state_ = new_state;
}

void PursuitNode::publish_pursuit_status()
{
  nlohmann::json status;
  status["state"] = static_cast<int>(state_);
  status["state_name"] = [this]() -> std::string {
    switch (state_) {
      case PursuitState::IDLE: return "IDLE";
      case PursuitState::CONFIRMING: return "CONFIRMING";
      case PursuitState::PURSUING: return "PURSUING";
      case PursuitState::RETREATING: return "RETREATING";
      default: return "UNKNOWN";
    }
  }();
  status["target_id"] = current_pursuit_target_id_;

  double tx, ty, tz;
  if (target_validator_.get_filtered_position(tx, ty, tz)) {
    status["target_x"] = tx;
    status["target_y"] = ty;
    status["target_z"] = tz;
  }

  if (state_ == PursuitState::PURSUING) {
    auto now = std::chrono::steady_clock::now();
    status["pursuit_duration"] =
      std::chrono::duration<double>(now - pursuit_start_time_).count();
  }

  status["robot_hp"] = robot_status_.hp;

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = status.dump();
  pursuit_status_pub_->publish(std::move(msg));
}

void PursuitNode::publish_debug_visualization()
{
  visualization_msgs::msg::MarkerArray markers;
  auto now = this->now();
  geometry_msgs::msg::PoseStamped target_pose_msg;
  target_pose_msg.header.frame_id = params_.map_frame_id;
  target_pose_msg.header.stamp = now;

  auto make_marker =
    [&](int id, int type, double scale_x, double scale_y, double scale_z,
        double r, double g, double b, double a) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = params_.map_frame_id;
      marker.header.stamp = now;
      marker.ns = "pursuit_debug";
      marker.id = id;
      marker.type = type;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = scale_x;
      marker.scale.y = scale_y;
      marker.scale.z = scale_z;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.pose.orientation.w = 1.0;
      return marker;
    };

  auto append_delete_markers = [&]() {
    for (int id = 0; id < 12; ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = params_.map_frame_id;
      marker.header.stamp = now;
      marker.ns = "pursuit_debug";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      markers.markers.push_back(marker);
    }
  };

  double rx, ry, rz;
  if (!get_robot_position(rx, ry, rz)) {
    append_delete_markers();
    debug_markers_pub_->publish(markers);
    return;
  }

  {
    auto marker = make_marker(0, visualization_msgs::msg::Marker::SPHERE,
      0.35, 0.35, 0.35, 0.1, 0.9, 0.2, 1.0);
    marker.pose.position.x = rx;
    marker.pose.position.y = ry;
    marker.pose.position.z = rz;
    markers.markers.push_back(marker);
  }

  {
    auto marker = make_marker(1, visualization_msgs::msg::Marker::LINE_STRIP,
      0.05, 0.0, 0.0, 0.2, 0.8, 1.0, 0.8);
    constexpr int kCircleSegments = 36;
    for (int i = 0; i <= kCircleSegments; ++i) {
      double angle = (2.0 * M_PI * static_cast<double>(i)) / kCircleSegments;
      geometry_msgs::msg::Point p;
      p.x = rx + params_.trigger_distance * std::cos(angle);
      p.y = ry + params_.trigger_distance * std::sin(angle);
      p.z = rz;
      marker.points.push_back(p);
    }
    markers.markers.push_back(marker);
  }

  if (has_last_raw_target_map_) {
    auto marker = make_marker(2, visualization_msgs::msg::Marker::SPHERE,
      0.28, 0.28, 0.28, 1.0, 0.75, 0.0, 0.95);
    marker.pose.position = last_raw_target_map_;
    markers.markers.push_back(marker);
  } else {
    auto marker = make_marker(2, visualization_msgs::msg::Marker::SPHERE,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(marker);
  }

  double tx, ty, tz;
  bool has_filtered_target = target_validator_.get_filtered_position(tx, ty, tz);
  if (has_filtered_target) {
    target_pose_msg.pose.position.x = tx;
    target_pose_msg.pose.position.y = ty;
    target_pose_msg.pose.position.z = tz;
    target_pose_msg.pose.orientation.w = 1.0;
    target_pose_pub_->publish(target_pose_msg);

    auto marker = make_marker(3, visualization_msgs::msg::Marker::SPHERE,
      0.32, 0.32, 0.32, 1.0, 0.15, 0.15, 1.0);
    marker.pose.position.x = tx;
    marker.pose.position.y = ty;
    marker.pose.position.z = tz;
    markers.markers.push_back(marker);

    auto line = make_marker(4, visualization_msgs::msg::Marker::LINE_STRIP,
      0.06, 0.0, 0.0, 1.0, 0.35, 0.35, 0.85);
    geometry_msgs::msg::Point start;
    start.x = rx;
    start.y = ry;
    start.z = rz;
    geometry_msgs::msg::Point end;
    end.x = tx;
    end.y = ty;
    end.z = tz;
    line.points.push_back(start);
    line.points.push_back(end);
    markers.markers.push_back(line);

    auto label = make_marker(10, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0.0, 0.0, 0.32, 1.0, 0.25, 0.25, 1.0);
    label.pose.position.x = tx;
    label.pose.position.y = ty;
    label.pose.position.z = tz + 0.45;
    label.text = "enemy target";
    markers.markers.push_back(label);
  } else {
    for (int id : {3, 4, 10}) {
      auto marker = make_marker(id, visualization_msgs::msg::Marker::SPHERE,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      marker.action = visualization_msgs::msg::Marker::DELETE;
      markers.markers.push_back(marker);
    }
  }

  if (!last_candidate_points_.empty()) {
    auto marker = make_marker(5, visualization_msgs::msg::Marker::POINTS,
      0.12, 0.12, 0.0, 0.1, 0.9, 1.0, 0.95);
    marker.points = last_candidate_points_;
    markers.markers.push_back(marker);
  } else {
    auto marker = make_marker(5, visualization_msgs::msg::Marker::POINTS,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(marker);
  }

  if (has_last_goal_) {
    attack_pose_pub_->publish(last_goal_);

    auto marker = make_marker(6, visualization_msgs::msg::Marker::ARROW,
      0.55, 0.14, 0.14, 0.15, 0.35, 1.0, 1.0);
    marker.pose = last_goal_.pose;
    markers.markers.push_back(marker);

    auto point_marker = make_marker(8, visualization_msgs::msg::Marker::CYLINDER,
      0.45, 0.45, 0.12, 0.15, 0.35, 1.0, 0.9);
    point_marker.pose.position = last_goal_.pose.position;
    markers.markers.push_back(point_marker);

    if (has_filtered_target) {
      auto line = make_marker(9, visualization_msgs::msg::Marker::LINE_STRIP,
        0.06, 0.0, 0.0, 1.0, 0.35, 0.35, 0.85);
      geometry_msgs::msg::Point target_point;
      target_point.x = tx;
      target_point.y = ty;
      target_point.z = tz;
      geometry_msgs::msg::Point attack_point;
      attack_point.x = last_goal_.pose.position.x;
      attack_point.y = last_goal_.pose.position.y;
      attack_point.z = last_goal_.pose.position.z;
      line.points.push_back(target_point);
      line.points.push_back(attack_point);
      markers.markers.push_back(line);
    }

    auto label = make_marker(11, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0.0, 0.0, 0.32, 0.15, 0.35, 1.0, 1.0);
    label.pose.position.x = last_goal_.pose.position.x;
    label.pose.position.y = last_goal_.pose.position.y;
    label.pose.position.z = last_goal_.pose.position.z + 0.4;
    label.text = "selected attack point";
    markers.markers.push_back(label);
  } else {
    auto marker = make_marker(6, visualization_msgs::msg::Marker::ARROW,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(marker);
    auto point_marker = make_marker(8, visualization_msgs::msg::Marker::CYLINDER,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    point_marker.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(point_marker);
    auto line = make_marker(9, visualization_msgs::msg::Marker::LINE_STRIP,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    line.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(line);
    auto label = make_marker(11, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    label.action = visualization_msgs::msg::Marker::DELETE;
    markers.markers.push_back(label);
  }

  {
    auto marker = make_marker(7, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      0.0, 0.0, 0.45, 1.0, 1.0, 1.0, 1.0);
    marker.pose.position.x = rx;
    marker.pose.position.y = ry;
    marker.pose.position.z = rz + 1.0;

    std::ostringstream oss;
    oss << "state=";
    switch (state_) {
      case PursuitState::IDLE: oss << "IDLE"; break;
      case PursuitState::CONFIRMING: oss << "CONFIRMING"; break;
      case PursuitState::PURSUING: oss << "PURSUING"; break;
      case PursuitState::RETREATING: oss << "RETREATING"; break;
    }
    oss << " target_id=" << current_pursuit_target_id_;
    oss << " hp=" << static_cast<int>(robot_status_.hp);
    if (has_filtered_target) {
      oss << " target=(" << tx << "," << ty << "," << tz << ")";
    }
    marker.text = oss.str();
    markers.markers.push_back(marker);
  }

  debug_markers_pub_->publish(markers);
}

void PursuitNode::publish_retreat_goal()
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = params_.map_frame_id;
  goal.header.stamp = this->now();
  goal.pose.position.x = params_.retreat_x;
  goal.pose.position.y = params_.retreat_y;
  goal.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, params_.retreat_yaw);
  goal.pose.orientation = tf2::toMsg(q);

  has_last_goal_ = true;
  last_goal_ = goal;
  goal_history_.header.stamp = goal.header.stamp;
  goal_history_.poses.push_back(goal);
  if (goal_history_.poses.size() > 200) {
    goal_history_.poses.erase(goal_history_.poses.begin());
  }
  goal_history_pub_->publish(goal_history_);
  goal_pub_->publish(goal);
}

}  // namespace pursuit_decision
