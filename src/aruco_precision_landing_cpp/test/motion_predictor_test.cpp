// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/motion_predictor.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

TargetStateEstimate valid_estimate()
{
  TargetStateEstimate estimate;
  estimate.position_ned = Eigen::Vector3d{1.0, 2.0, -0.5};
  estimate.velocity_ned = Eigen::Vector3d{0.2, -0.4, 0.1};
  estimate.covariance = Eigen::Matrix<double, 6, 6>::Identity();
  estimate.sample_time_s = 10.0;
  return estimate;
}

TEST(MotionPredictorTest, PredictsConstantVelocityWithObservationAgeAndAdditionalHorizon)
{
  MotionPredictorParameters parameters;
  parameters.additional_prediction_horizon_s = 0.10;
  parameters.max_prediction_horizon_s = 0.50;
  MotionPredictor predictor(parameters);
  const auto estimate = valid_estimate();

  const auto prediction = predictor.predict(estimate, 0.15);

  ASSERT_TRUE(prediction.has_value());
  EXPECT_DOUBLE_EQ(prediction->prediction_horizon_s, 0.25);
  EXPECT_TRUE(
    prediction->position_ned.isApprox(
      estimate.position_ned + estimate.velocity_ned * 0.25,
      1.0e-12));
  EXPECT_TRUE(prediction->velocity_ned.isApprox(estimate.velocity_ned, 1.0e-12));
}

TEST(MotionPredictorTest, ZeroAgeAndZeroAdditionalCompensationProduceZeroHorizon)
{
  MotionPredictorParameters parameters;
  parameters.additional_prediction_horizon_s = 0.0;
  parameters.max_prediction_horizon_s = 0.50;
  MotionPredictor predictor(parameters);
  const auto estimate = valid_estimate();

  const auto prediction = predictor.predict(estimate, 0.0);

  ASSERT_TRUE(prediction.has_value());
  EXPECT_DOUBLE_EQ(prediction->prediction_horizon_s, 0.0);
  EXPECT_TRUE(prediction->position_ned.isApprox(estimate.position_ned, 1.0e-12));
}

TEST(MotionPredictorTest, ZeroVelocityKeepsPositionUnchanged)
{
  MotionPredictor predictor(MotionPredictorParameters{});
  auto estimate = valid_estimate();
  estimate.velocity_ned.setZero();

  const auto prediction = predictor.predict(estimate, 0.20);

  ASSERT_TRUE(prediction.has_value());
  EXPECT_TRUE(prediction->position_ned.isApprox(estimate.position_ned, 1.0e-12));
}

TEST(MotionPredictorTest, ClampsPredictionHorizonToConfiguredMaximum)
{
  MotionPredictorParameters parameters;
  parameters.additional_prediction_horizon_s = 0.10;
  parameters.max_prediction_horizon_s = 0.30;
  MotionPredictor predictor(parameters);
  const auto estimate = valid_estimate();

  const auto prediction = predictor.predict(estimate, 2.0);

  ASSERT_TRUE(prediction.has_value());
  EXPECT_DOUBLE_EQ(prediction->prediction_horizon_s, 0.30);
  EXPECT_TRUE(
    prediction->position_ned.isApprox(
      estimate.position_ned + estimate.velocity_ned * 0.30,
      1.0e-12));
}

TEST(MotionPredictorTest, DoesNotModifyInputEstimate)
{
  MotionPredictor predictor(MotionPredictorParameters{});
  const auto estimate = valid_estimate();
  const auto original = estimate;

  ASSERT_TRUE(predictor.predict(estimate, 0.1).has_value());

  EXPECT_TRUE(estimate.position_ned.isApprox(original.position_ned, 0.0));
  EXPECT_TRUE(estimate.velocity_ned.isApprox(original.velocity_ned, 0.0));
  EXPECT_TRUE(estimate.covariance.isApprox(original.covariance, 0.0));
  EXPECT_DOUBLE_EQ(estimate.sample_time_s, original.sample_time_s);
}

TEST(MotionPredictorTest, RejectsInvalidAgeOrEstimate)
{
  MotionPredictor predictor(MotionPredictorParameters{});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(predictor.predict(valid_estimate(), -0.1).has_value());
  EXPECT_FALSE(predictor.predict(valid_estimate(), nan).has_value());

  auto estimate = valid_estimate();
  estimate.position_ned.x() = infinity;
  EXPECT_FALSE(predictor.predict(estimate, 0.1).has_value());

  estimate = valid_estimate();
  estimate.covariance(0, 0) = nan;
  EXPECT_FALSE(predictor.predict(estimate, 0.1).has_value());
}

TEST(MotionPredictorTest, RejectsInvalidParameters)
{
  MotionPredictorParameters parameters;
  parameters.additional_prediction_horizon_s = -0.1;
  EXPECT_THROW(MotionPredictor predictor(parameters), std::invalid_argument);

  parameters = MotionPredictorParameters{};
  parameters.max_prediction_horizon_s = 0.0;
  EXPECT_THROW(MotionPredictor predictor(parameters), std::invalid_argument);

  parameters.additional_prediction_horizon_s = 0.6;
  parameters.max_prediction_horizon_s = 0.5;
  EXPECT_THROW(MotionPredictor predictor(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
