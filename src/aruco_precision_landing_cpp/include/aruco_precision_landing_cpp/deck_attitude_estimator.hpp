// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__DECK_ATTITUDE_ESTIMATOR_HPP_
#define ARUCO_PRECISION_LANDING_CPP__DECK_ATTITUDE_ESTIMATOR_HPP_

#include <optional>

#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 基于 Marker 法向量的甲板姿态滤波参数。
 */
struct DeckAttitudeEstimatorParameters
{
  double filter_gain{0.20};
  double minimum_upward_normal_component{0.50};
};

/**
 * @brief 忽略甲板 yaw 后的倾斜姿态估计。
 */
struct DeckAttitudeEstimate
{
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double tilt_rad{0.0};
  Eigen::Vector3d upward_normal_ned{Eigen::Vector3d{0.0, 0.0, -1.0}};
};

/**
 * @brief 从 Marker `+Z` 向上法向量估计甲板倾斜，并进行一阶低通。
 *
 * OpenCV Marker 坐标在水平甲板上可能包含约 180 度固有翻转，因此不能直接将完整
 * Marker 四元数解释为甲板 roll/pitch。本类只使用 Marker `+Z` 在 local NED 中的方向，
 * 对法向量低通后生成 yaw 无关的等效 roll、pitch 和总倾角。
 */
class DeckAttitudeEstimator
{
public:
  explicit DeckAttitudeEstimator(const DeckAttitudeEstimatorParameters & parameters);

  /**
   * @brief 更新甲板姿态。
   *
   * @param marker_to_ned_rotation Marker 坐标向量到 local NED 的 Hamilton 四元数。
   * @return Marker `+Z` 指向 NED 上方且输入有效时返回滤波估计，否则返回 `std::nullopt`。
   */
  std::optional<DeckAttitudeEstimate> update(
    const Eigen::Quaterniond & marker_to_ned_rotation);

  void reset();

private:
  DeckAttitudeEstimate estimate_from_normal(const Eigen::Vector3d & normal_ned) const;

  DeckAttitudeEstimatorParameters parameters_;
  Eigen::Vector3d filtered_normal_ned_{Eigen::Vector3d{0.0, 0.0, -1.0}};
  bool initialized_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__DECK_ATTITUDE_ESTIMATOR_HPP_
