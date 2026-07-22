// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__MOTION_PREDICTOR_HPP_
#define ARUCO_PRECISION_LANDING_CPP__MOTION_PREDICTOR_HPP_

#include "aruco_precision_landing_cpp/target_state_estimator.hpp"

#include <optional>

#include <Eigen/Core>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 甲板短时运动预测参数。
 */
struct MotionPredictorParameters
{
  double additional_prediction_horizon_s{0.10};
  double max_prediction_horizon_s{0.50};
};

/**
 * @brief 常速度短时预测结果。
 */
struct MotionPrediction
{
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  double prediction_horizon_s{0.0};
};

/**
 * @brief 将甲板状态按受限常速度模型外推到控制时刻。
 *
 * 预测时域由最后有效观测的回调年龄和可配置附加补偿组成，不直接混减可能属于不同
 * 时间域的图像时间戳与控制器时间。
 */
class MotionPredictor
{
public:
  /**
   * @brief 创建运动预测器并校验预测时域参数。
   *
   * @param parameters 固定附加补偿和最大预测时域。
   * @throws std::invalid_argument 参数非法，或附加时域大于最大时域。
   */
  explicit MotionPredictor(const MotionPredictorParameters & parameters);

  /**
   * @brief 使用常速度模型计算短时预测位置。
   *
   * @param estimate 当前滤波位置、速度、协方差和采样时间。
   * @param observation_receipt_age_s 最后有效视觉测量到达后的经过时间，单位为秒。
   * @return 受最大时域限制的预测位置、速度和实际预测时域；输入非法时返回
   *         `std::nullopt`。
   */
  std::optional<MotionPrediction> predict(
    const TargetStateEstimate & estimate,
    double observation_receipt_age_s) const;

private:
  MotionPredictorParameters parameters_;
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__MOTION_PREDICTOR_HPP_
