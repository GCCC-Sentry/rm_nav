#ifndef PURSUIT_DECISION__TARGET_VALIDATOR_HPP
#define PURSUIT_DECISION__TARGET_VALIDATOR_HPP

#include <chrono>
#include <deque>
#include <string>
#include <unordered_map>
#include <vector>

namespace pursuit_decision
{

/// 单次观测数据
struct TargetObservation {
  int armor_id;           // 装甲板编号 (1-9)
  double x, y, z;         // map frame 位置
  int tracking_state;     // 0=lost, 1=detecting, 2=tracking
  std::chrono::steady_clock::time_point timestamp;
};

class TargetValidator
{
public:
  TargetValidator() = default;

  void configure(
    int confirm_frames,
    double confirm_timeout_sec,
    double noise_threshold,
    double max_position_jump);

  /// 添加一次观测
  void add_observation(const TargetObservation & obs);

  /// 检查目标是否已确认 (满足多帧一致性)
  bool is_confirmed() const;

  /// 返回当前未确认/确认的具体原因，便于日志排查
  std::string explain_confirmation_status() const;

  /// 获取当前确认的目标ID (-1 = 无)
  int get_confirmed_target_id() const;

  /// 获取目标的滤波后位置 (中位数滤波)
  bool get_filtered_position(double & x, double & y, double & z) const;

  /// 获取自上次有效观测以来的时间（秒）
  double time_since_last_observation() const;

  /// 最近一次 add_observation 的处理结果
  const std::string & last_observation_status() const {return last_observation_status_;}

  /// 重置状态
  void reset();

private:
  int confirm_frames_{10};
  double confirm_timeout_sec_{2.0};
  double noise_threshold_{0.5};       // 单次位置跳变阈值 (m)
  double max_position_jump_{1.5};     // 最大允许帧间跳变 (m)

  std::deque<TargetObservation> buffer_;
  int confirmed_target_id_{-1};
  std::string last_observation_status_{"尚无观测"};

  /// 清理过期数据
  void cleanup_expired();

  /// 计算中位数
  static double median(std::vector<double> & values);
};

}  // namespace pursuit_decision

#endif  // PURSUIT_DECISION__TARGET_VALIDATOR_HPP
