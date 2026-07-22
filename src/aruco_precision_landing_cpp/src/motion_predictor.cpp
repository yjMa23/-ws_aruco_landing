// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/motion_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{

MotionPredictor::MotionPredictor(const MotionPredictorParameters & parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.additional_prediction_horizon_s) ||
    parameters_.additional_prediction_horizon_s < 0.0)
  {
    throw std::invalid_argument(
            "additional_prediction_horizon_s must be finite and non-negative");
  }
  if (!std::isfinite(parameters_.max_prediction_horizon_s) ||
    parameters_.max_prediction_horizon_s <= 0.0)
  {
    throw std::invalid_argument(
            "max_prediction_horizon_s must be finite and positive");
  }
  if (parameters_.additional_prediction_horizon_s >
    parameters_.max_prediction_horizon_s)
  {
    throw std::invalid_argument(
            "additional_prediction_horizon_s must not exceed max_prediction_horizon_s");
  }
}

std::optional<MotionPrediction> MotionPredictor::predict(
  const TargetStateEstimate & estimate,
  double observation_receipt_age_s) const
{
  if (!estimate.position_ned.allFinite() ||
    !estimate.velocity_ned.allFinite() ||
    !estimate.covariance.allFinite() ||
    !std::isfinite(estimate.sample_time_s) ||
    !std::isfinite(observation_receipt_age_s) ||
    observation_receipt_age_s < 0.0)
  {
    return std::nullopt;
  }

  const double prediction_horizon_s = std::min(
    observation_receipt_age_s + parameters_.additional_prediction_horizon_s,
    parameters_.max_prediction_horizon_s);

  MotionPrediction prediction;
  prediction.position_ned =
    estimate.position_ned + estimate.velocity_ned * prediction_horizon_s;
  prediction.velocity_ned = estimate.velocity_ned;
  prediction.prediction_horizon_s = prediction_horizon_s;

  if (!prediction.position_ned.allFinite() ||
    !prediction.velocity_ned.allFinite() ||
    !std::isfinite(prediction.prediction_horizon_s))
  {
    return std::nullopt;
  }
  return prediction;
}

}  // namespace aruco_precision_landing_cpp
