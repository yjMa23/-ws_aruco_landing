// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__ADAPTIVE_RELATIVE_VELOCITY_GAIN_HPP_
#define ARUCO_PRECISION_LANDING_CPP__ADAPTIVE_RELATIVE_VELOCITY_GAIN_HPP_

#include <optional>

#include <Eigen/Core>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 加速度感知相对速度增益调度参数。
 */
struct AdaptiveRelativeVelocityGainParameters
{
  double min_gain{0.25};
  double max_gain{1.0};
  double acceleration_low_threshold_mps2{0.05};
  double acceleration_high_threshold_mps2{0.35};
  double max_acceleration_mps2{1.50};
  double acceleration_filter_gain{0.20};
};

/**
 * @brief 单周期动态增益和过滤后甲板水平加速度。
 */
struct AdaptiveRelativeVelocityGainOutput
{
  double gain{0.25};
  Eigen::Vector2d filtered_acceleration_xy{Eigen::Vector2d::Zero()};
};

/**
 * @brief 由目标估计速度差分得到水平加速度，并连续调度相对速度反馈增益。
 *
 * 该类不依赖 ROS。原始速度差分先做模长限幅和一阶低通，再通过 smoothstep 将
 * 加速度模长映射到 `[min_gain, max_gain]`。匀速阶段保持最小增益，换向或加速阶段
 * 连续提高阻尼。
 */
class AdaptiveRelativeVelocityGain
{
public:
  /**
   * @brief 创建调度器并校验参数。
   *
   * @throws std::invalid_argument 参数非有限、阈值关系非法或滤波系数不在 `(0, 1]`。
   */
  explicit AdaptiveRelativeVelocityGain(
    const AdaptiveRelativeVelocityGainParameters & parameters);

  /**
   * @brief 使用本周期甲板估计速度更新动态增益。
   *
   * @param deck_velocity_xy PX4 local NED 水平估计速度 `[North, East]`，单位 m/s。
   * @param dt_s 与上一速度样本之间的正时间间隔，单位秒。
   * @return 输入有效时返回动态增益和过滤后加速度；输入含 NaN/Inf 或 `dt_s <= 0`
   *         时返回 `std::nullopt` 且内部状态不变。第一帧返回最小增益和零加速度。
   */
  std::optional<AdaptiveRelativeVelocityGainOutput> update(
    const Eigen::Vector2d & deck_velocity_xy,
    double dt_s);

  /**
   * @brief 清除速度和加速度历史。
   */
  void reset();

private:
  AdaptiveRelativeVelocityGainOutput make_output() const;

  AdaptiveRelativeVelocityGainParameters parameters_;
  Eigen::Vector2d previous_velocity_xy_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d filtered_acceleration_xy_{Eigen::Vector2d::Zero()};
  bool initialized_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__ADAPTIVE_RELATIVE_VELOCITY_GAIN_HPP_
