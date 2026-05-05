#include "pursuit_decision/target_validator.hpp"

#include <algorithm>
#include <cmath>

namespace pursuit_decision
{

void TargetValidator::configure(
  int confirm_frames,
  double confirm_timeout_sec,
  double noise_threshold,
  double max_position_jump)
{
  confirm_frames_ = confirm_frames;
  confirm_timeout_sec_ = confirm_timeout_sec;
  noise_threshold_ = noise_threshold;
  max_position_jump_ = max_position_jump;
}

void TargetValidator::add_observation(const TargetObservation & obs)
{
  cleanup_expired();

  // 只接受tracking状态 (state=2) 或 detecting状态 (state=1) 的观测
  if (obs.tracking_state < 1) {
    last_observation_status_ = "丢弃: tracking_state < 1";
    return;
  }

  // 如果buffer中已有数据，检查帧间跳变
  if (!buffer_.empty()) {
    const auto & last = buffer_.back();

    // 如果目标ID发生变化，重置buffer
    if (obs.armor_id != last.armor_id) {
      buffer_.clear();
      confirmed_target_id_ = -1;
      last_observation_status_ = "目标ID切换，已重置buffer";
    } else {
      // 检查位置跳变
      double dx = obs.x - last.x;
      double dy = obs.y - last.y;
      double dz = obs.z - last.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (dist > max_position_jump_) {
        // 跳变过大，可能是PnP误差或目标切换，不加入buffer
        last_observation_status_ = "丢弃: 帧间跳变过大";
        return;
      }
    }
  }

  buffer_.push_back(obs);
  last_observation_status_ = "已加入buffer";

  // 保持buffer大小不超过2倍确认帧数
  while (static_cast<int>(buffer_.size()) > confirm_frames_ * 2) {
    buffer_.pop_front();
  }
}

bool TargetValidator::is_confirmed() const
{
  return explain_confirmation_status() == "confirmed";
}

std::string TargetValidator::explain_confirmation_status() const
{
  if (static_cast<int>(buffer_.size()) < confirm_frames_) {
    return "连续有效帧不足: " + std::to_string(buffer_.size()) + "/" +
           std::to_string(confirm_frames_);
  }

  // 检查最近N帧是否一致
  int consistent_count = 0;
  int target_id = buffer_.back().armor_id;

  // 从最新往回检查
  for (auto it = buffer_.rbegin(); it != buffer_.rend() && consistent_count < confirm_frames_;
       ++it)
  {
    if (it->armor_id != target_id) break;
    if (it->tracking_state < 1) break;
    consistent_count++;
  }

  if (consistent_count < confirm_frames_) {
    return "最近确认窗口内 armor_id 不一致或 tracking_state 失效: " +
           std::to_string(consistent_count) + "/" + std::to_string(confirm_frames_);
  }

  // 检查确认窗口内的时间跨度不超过超时
  auto newest = buffer_.back().timestamp;
  auto oldest_in_window = std::prev(buffer_.end(), confirm_frames_)->timestamp;
  double time_span =
    std::chrono::duration<double>(newest - oldest_in_window).count();

  if (time_span > confirm_timeout_sec_) {
    return "确认时间窗过长: " + std::to_string(time_span) + "s > " +
           std::to_string(confirm_timeout_sec_) + "s";
  }

  // 检查位置一致性 (噪声检查)
  // 计算最近N帧位置的标准差
  std::vector<double> xs, ys, zs;
  int count = 0;
  for (auto it = buffer_.rbegin();
       it != buffer_.rend() && count < confirm_frames_;
       ++it, ++count)
  {
    xs.push_back(it->x);
    ys.push_back(it->y);
    zs.push_back(it->z);
  }

  auto calc_std = [](const std::vector<double> & vals) -> double {
    double sum = 0, sq_sum = 0;
    for (double v : vals) {
      sum += v;
      sq_sum += v * v;
    }
    double mean = sum / vals.size();
    return std::sqrt(sq_sum / vals.size() - mean * mean);
  };

  double std_x = calc_std(xs);
  double std_y = calc_std(ys);
  double spatial_std = std::sqrt(std_x * std_x + std_y * std_y);

  // 如果位置标准差过大，说明目标不稳定
  if (spatial_std > noise_threshold_) {
    return "位置抖动过大: spatial_std=" + std::to_string(spatial_std) + " > " +
           std::to_string(noise_threshold_);
  }

  return "confirmed";
}

int TargetValidator::get_confirmed_target_id() const
{
  if (!is_confirmed()) return -1;
  return buffer_.back().armor_id;
}

bool TargetValidator::get_filtered_position(double & x, double & y, double & z) const
{
  if (buffer_.empty()) return false;

  // 使用最近N帧的中位数滤波
  int use_count = std::min(static_cast<int>(buffer_.size()), confirm_frames_);
  std::vector<double> xs, ys, zs;

  int count = 0;
  for (auto it = buffer_.rbegin();
       it != buffer_.rend() && count < use_count;
       ++it, ++count)
  {
    xs.push_back(it->x);
    ys.push_back(it->y);
    zs.push_back(it->z);
  }

  x = median(xs);
  y = median(ys);
  z = median(zs);
  return true;
}

double TargetValidator::time_since_last_observation() const
{
  if (buffer_.empty()) return 1e9;

  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - buffer_.back().timestamp).count();
}

void TargetValidator::reset()
{
  buffer_.clear();
  confirmed_target_id_ = -1;
}

void TargetValidator::cleanup_expired()
{
  auto now = std::chrono::steady_clock::now();
  while (!buffer_.empty()) {
    double age = std::chrono::duration<double>(now - buffer_.front().timestamp).count();
    if (age > confirm_timeout_sec_ * 3.0) {
      buffer_.pop_front();
    } else {
      break;
    }
  }
}

double TargetValidator::median(std::vector<double> & values)
{
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  size_t mid = values.size() / 2;
  if (values.size() % 2 == 0) {
    return (values[mid - 1] + values[mid]) / 2.0;
  }
  return values[mid];
}

}  // namespace pursuit_decision
