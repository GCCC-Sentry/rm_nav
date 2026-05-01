#ifndef PURSUIT_DECISION__PURSUIT_NODE_HPP
#define PURSUIT_DECISION__PURSUIT_NODE_HPP

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pursuit_decision/target_validator.hpp"
#include "pursuit_decision/zone_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pursuit_decision
{

/// 追击状态机
enum class PursuitState : int {
  IDLE = 0,         // 空闲，不追击
  CONFIRMING = 1,   // 正在确认目标 (多帧校验中)
  PURSUING = 2,     // 追击中
  RETREATING = 3    // 撤退中
};

class PursuitNode : public rclcpp::Node
{
public:
  explicit PursuitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ===================== 参数 =====================
  struct Parameters {
    // 追击触发
    double trigger_distance{5.0};            // 追击触发最大距离 (m)
    int confirm_frames{10};                  // 多帧确认帧数
    double confirm_timeout{2.0};             // 确认超时 (s)

    // 追击行为
    double goal_update_rate{5.0};            // 追击目标更新频率 (Hz)
    double max_pursuit_duration{15.0};       // 最大追击时长 (s)
    double target_lost_timeout{3.0};         // 目标丢失超时 (s)

    // 逼近距离 (按装甲板类型)
    std::unordered_map<int, double> approach_distance{
      {1, 4.0},   // 英雄 - 保持距离
      {2, 3.0},   // 工程
      {3, 2.5},   // 步兵3
      {4, 2.5},   // 步兵4
      {5, 2.5},   // 步兵5
      {6, 3.5},   // 哨兵
      {7, 3.0},   // 其他
    };

    // 撤退条件
    double hp_retreat_threshold{150.0};      // 血量撤退阈值

    // 坐标系
    std::string map_frame_id{"map"};
    std::string robot_frame_id{"chassis"};
    double transform_tolerance{0.5};         // TF超时 (s)

    // IMU世界坐标系 → 导航map坐标系 的yaw偏移量 (rad)
    // 自瞄使用IMU世界坐标系, 导航使用map坐标系
    // 两者共享重力方向, 仅差一个绕Z轴的yaw偏移
    // 标定方法: 机器人初始位置确定时, offset = map_yaw - imu_world_yaw
    double world_to_map_yaw_offset{0.0};

    // 噪声滤波
    double pnp_noise_threshold{0.5};         // PnP噪声阈值 (m)
    double max_position_jump{1.5};           // 最大帧间跳变 (m)

    // 图层
    double z_layer_tolerance{0.3};           // Z值图层判断容差 (m)
    bool enable_zone_check{false};           // 启用区域/图层检查 (测试时建议关闭)

    // 代价地图
    int costmap_cost_threshold{50};          // 可通行的最大代价值
    int num_candidate_sectors{36};           // 候选点数量(圆上)

    // 安全撤退点 (己方半场)
    double retreat_x{2.0};
    double retreat_y{0.0};
    double retreat_yaw{0.0};
  } params_;

  // ===================== 状态 =====================
  PursuitState state_{PursuitState::IDLE};
  std::chrono::steady_clock::time_point pursuit_start_time_;
  int current_pursuit_target_id_{-1};

  // ===================== 组件 =====================
  ZoneManager zone_manager_;
  TargetValidator target_validator_;

  // ===================== TF =====================
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ===================== Subscribers =====================
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr aim_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr game_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  // ===================== Publishers =====================
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr attack_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pursuit_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr goal_history_pub_;

  // ===================== Timer =====================
  rclcpp::TimerBase::SharedPtr decision_timer_;    // 主决策循环
  rclcpp::TimerBase::SharedPtr goal_timer_;        // 目标更新

  // ===================== 缓存数据 =====================
  struct RobotStatus {
    double hp{600.0};
    double heat{0.0};
    double ammo{500.0};
    std::chrono::steady_clock::time_point last_update;
  } robot_status_;

  struct GameStatus {
    int progress{0};
    double remaining_time{420.0};
    std::chrono::steady_clock::time_point last_update;
  } game_status_;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  nav_msgs::msg::Path goal_history_;

  bool has_last_raw_target_map_{false};
  geometry_msgs::msg::Point last_raw_target_map_;
  bool has_last_goal_{false};
  geometry_msgs::msg::PoseStamped last_goal_;
  std::vector<geometry_msgs::msg::Point> last_candidate_points_;

  // ===================== 回调 =====================
  void on_aim_target(const std_msgs::msg::String::SharedPtr msg);
  void on_robot_status(const std_msgs::msg::String::SharedPtr msg);
  void on_game_status(const std_msgs::msg::String::SharedPtr msg);
  void on_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  // ===================== 决策逻辑 =====================
  void decision_callback();
  void goal_update_callback();

  /// 状态机转换
  void transition_to(PursuitState new_state);

  /// 判断是否应该开始追击
  bool should_start_pursuit(double target_x, double target_y, double target_z);

  /// 判断是否应该撤退
  bool should_retreat();

  /// 获取机器人当前在map frame的位置
  bool get_robot_position(double & x, double & y, double & z);

  /// 将IMU世界坐标系下的相对向量转换到map frame下的绝对位置
  /// auto_aim发布的是目标相对于云台在IMU世界坐标系的向量
  /// 转换: target_map = robot_map + R(yaw_offset) * xyz_in_world
  bool world_vector_to_map(
    double world_x, double world_y, double world_z,
    double & mx, double & my, double & mz);

  /// 计算追击目标点 (类似CalculateAttackPose的圆形候选)
  bool compute_pursuit_goal(
    double target_x, double target_y,
    double approach_dist,
    geometry_msgs::msg::PoseStamped & goal);

  /// 获取目标类型对应的逼近距离
  double get_approach_distance(int armor_id);

  /// 发布追击状态
  void publish_pursuit_status();

  /// 发布 RViz 调试可视化
  void publish_debug_visualization();

  /// 发布撤退目标
  void publish_retreat_goal();

  /// 声明并加载参数
  void declare_and_load_parameters();

  /// 初始化区域定义
  void initialize_zones();
};

}  // namespace pursuit_decision

#endif  // PURSUIT_DECISION__PURSUIT_NODE_HPP
