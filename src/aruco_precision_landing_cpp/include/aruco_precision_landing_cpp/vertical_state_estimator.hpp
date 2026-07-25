// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__VERTICAL_STATE_ESTIMATOR_HPP_
#define ARUCO_PRECISION_LANDING_CPP__VERTICAL_STATE_ESTIMATOR_HPP_

#include <optional>

#include <Eigen/Core>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 甲板垂直状态估计器参数。
 */
struct VerticalStateEstimatorParameters
{
  double process_acceleration_std_mps2{0.40};
  double measurement_std_m{0.05};
  /** 已知测量偏差，更新前从原始测量中减去。 */
  double measurement_bias_m{0.0};
  double initial_position_std_m{0.10};
  double initial_velocity_std_mps{0.50};
  double minimum_sample_dt_s{0.001};
  double maximum_sample_dt_s{0.25};
  double reinitialize_gap_s{2.0};
  double innovation_gate_mahalanobis{5.0};
};

/**
 * @brief 垂直测量更新状态。
 */
enum class VerticalStateUpdateStatus
{
  kInitialized,
  kUpdated,
  kReinitialized,
  kRejectedInvalidInput,
  kRejectedNonMonotonicTime,
  kRejectedOutlier
};

/**
 * @brief 甲板在 PX4 local NED 中的垂直位置与速度估计。
 */
struct VerticalStateEstimate
{
  double deck_z_ned_m{0.0};
  double deck_vertical_velocity_ned_mps{0.0};
  Eigen::Matrix2d covariance{Eigen::Matrix2d::Zero()};
  double sample_time_s{0.0};
};

/**
 * @brief 一次垂直测量更新结果。
 */
struct VerticalStateUpdateResult
{
  VerticalStateUpdateStatus status{VerticalStateUpdateStatus::kRejectedInvalidInput};
  double normalized_innovation_squared{0.0};
  std::optional<VerticalStateEstimate> estimate;
};

/**
 * @brief 使用二状态常速度 Kalman Filter 估计甲板垂直位置和速度。
 *
 * 状态为 `[z, vz]`，均采用 PX4 local NED；Down 为正。输入必须使用视觉图像
 * 采样时间。该类不依赖 ROS、PX4 消息或仿真 Ground Truth。
 */
class VerticalStateEstimator
{
public:
  /**
   * @brief 创建垂直状态估计器并校验参数。
   *
   * @throws std::invalid_argument 参数非有限、必要参数非正或时间门限关系非法。
   */
  explicit VerticalStateEstimator(const VerticalStateEstimatorParameters & parameters);

  /**
   * @brief 使用一帧视觉甲板 z 测量更新估计器。
   *
   * @param measurement_z_ned_m 原始甲板 z 测量，单位米，Down 为正。
   * @param sample_time_s 图像采样时间，单位秒，必须严格单调增加。
   * @return 更新状态、归一化创新平方和当前可选估计。
   */
  VerticalStateUpdateResult update(
    double measurement_z_ned_m,
    double sample_time_s);

  /**
   * @brief 返回当前垂直状态估计。
   */
  std::optional<VerticalStateEstimate> estimate() const;

  /**
   * @brief 清除状态、协方差和时间历史。
   */
  void reset();

private:
  void initialize(double corrected_measurement_z_ned_m, double sample_time_s);
  void predict_in_place(double dt_s);
  VerticalStateEstimate make_estimate() const;

  VerticalStateEstimatorParameters parameters_;
  Eigen::Vector2d state_{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d covariance_{Eigen::Matrix2d::Zero()};
  double state_time_s_{0.0};
  bool initialized_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__VERTICAL_STATE_ESTIMATOR_HPP_
