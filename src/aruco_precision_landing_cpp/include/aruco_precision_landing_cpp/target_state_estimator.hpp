// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__TARGET_STATE_ESTIMATOR_HPP_
#define ARUCO_PRECISION_LANDING_CPP__TARGET_STATE_ESTIMATOR_HPP_

#include <optional>

#include <Eigen/Core>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 视觉甲板状态估计器参数。
 */
struct TargetStateEstimatorParameters
{
  double process_acceleration_std_mps2{1.0};
  double measurement_horizontal_std_m{0.08};
  double measurement_vertical_std_m{0.12};
  double initial_position_std_m{0.20};
  double initial_velocity_std_mps{1.0};
  double minimum_sample_dt_s{0.001};
  double maximum_sample_dt_s{0.50};
  double reinitialize_gap_s{2.0};
  double innovation_gate_mahalanobis{5.0};
};

/**
 * @brief 一次视觉测量更新的处理结果。
 */
enum class TargetStateUpdateStatus
{
  kInitialized,
  kUpdated,
  kReinitialized,
  kRejectedInvalidInput,
  kRejectedNonMonotonicTime,
  kRejectedOutlier
};

/**
 * @brief 甲板在 PX4 local NED 中的六维状态估计。
 */
struct TargetStateEstimate
{
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 6, 6> covariance{
    Eigen::Matrix<double, 6, 6>::Zero()};
  double sample_time_s{0.0};
};

/**
 * @brief 估计器更新返回值。
 */
struct TargetStateUpdateResult
{
  TargetStateUpdateStatus status{TargetStateUpdateStatus::kRejectedInvalidInput};
  double normalized_innovation_squared{0.0};
  std::optional<TargetStateEstimate> estimate;
};

/**
 * @brief 使用三维常速度 Kalman Filter 估计视觉甲板位置和速度。
 *
 * 状态为 `[px, py, pz, vx, vy, vz]`，输入为 local NED 位置和单调采样时间。
 * 类本身不依赖 ROS，也不读取任何仿真 Ground Truth。
 */
class TargetStateEstimator
{
public:
  /**
   * @brief 创建状态估计器并校验参数。
   *
   * @param parameters 过程噪声、测量噪声、时间门限和创新门限。
   * @throws std::invalid_argument 任一参数非法或参数关系不成立。
   */
  explicit TargetStateEstimator(const TargetStateEstimatorParameters & parameters);

  /**
   * @brief 使用一帧视觉 local NED 位置更新估计器。
   *
   * @param position_ned Marker 位置 `[North, East, Down]`，单位为米。
   * @param sample_time_s 图像采样时间，单位为秒，必须单调增加。
   * @return 更新状态、归一化创新平方和更新后的可选估计。
   *
   * 超过 `reinitialize_gap_s` 的新测量会重新初始化并将速度清零。离群点被拒绝，
   * 但内部预测时间会推进到当前采样时刻，使后续正常测量可以继续更新。
   */
  TargetStateUpdateResult update(
    const Eigen::Vector3d & position_ned,
    double sample_time_s);

  /**
   * @brief 返回当前估计状态。
   *
   * @return 已初始化时返回位置、速度、协方差和状态时间，否则返回 `std::nullopt`。
   */
  std::optional<TargetStateEstimate> estimate() const;

  /**
   * @brief 清除全部状态和时间历史。
   */
  void reset();

private:
  void initialize(
    const Eigen::Vector3d & position_ned,
    double sample_time_s);
  void predict_in_place(double dt_s);
  TargetStateEstimate make_estimate() const;

  TargetStateEstimatorParameters parameters_;
  Eigen::Matrix<double, 6, 1> state_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 6> covariance_{
    Eigen::Matrix<double, 6, 6>::Zero()};
  double state_time_s_{0.0};
  bool initialized_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__TARGET_STATE_ESTIMATOR_HPP_
