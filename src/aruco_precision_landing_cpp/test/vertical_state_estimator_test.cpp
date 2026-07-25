// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/vertical_state_estimator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

VerticalStateEstimatorParameters test_parameters()
{
  VerticalStateEstimatorParameters parameters;
  parameters.process_acceleration_std_mps2 = 0.25;
  parameters.measurement_std_m = 0.04;
  parameters.measurement_bias_m = 0.0;
  parameters.initial_position_std_m = 0.10;
  parameters.initial_velocity_std_mps = 1.0;
  parameters.minimum_sample_dt_s = 0.001;
  parameters.maximum_sample_dt_s = 0.20;
  parameters.reinitialize_gap_s = 2.0;
  parameters.innovation_gate_mahalanobis = 5.0;
  return parameters;
}

TEST(VerticalStateEstimatorTest, FirstMeasurementInitializesCorrectedPosition)
{
  auto parameters = test_parameters();
  parameters.measurement_bias_m = 0.20;
  VerticalStateEstimator estimator(parameters);

  const auto result = estimator.update(-1.80, 10.0);

  ASSERT_EQ(result.status, VerticalStateUpdateStatus::kInitialized);
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_NEAR(result.estimate->deck_z_ned_m, -2.00, 1.0e-12);
  EXPECT_NEAR(result.estimate->deck_vertical_velocity_ned_mps, 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(result.estimate->sample_time_s, 10.0);
}

TEST(VerticalStateEstimatorTest, StaticMeasurementsConvergeToZeroVelocity)
{
  VerticalStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(-2.0, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 120; ++index) {
    const double noise = index % 2 == 0 ? 0.02 : -0.02;
    const auto result = estimator.update(-2.0 + noise, 0.05 * index);
    ASSERT_EQ(result.status, VerticalStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->deck_z_ned_m, -2.0, 0.015);
  EXPECT_NEAR(estimate->deck_vertical_velocity_ned_mps, 0.0, 0.02);
}

TEST(VerticalStateEstimatorTest, ConstantVelocityConvergesToTrueVerticalVelocity)
{
  VerticalStateEstimator estimator(test_parameters());
  constexpr double initial_z_m = -2.0;
  constexpr double velocity_mps = 0.18;
  ASSERT_EQ(
    estimator.update(initial_z_m, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 160; ++index) {
    const double time_s = 0.05 * index;
    const auto result = estimator.update(
      initial_z_m + velocity_mps * time_s, time_s);
    ASSERT_EQ(result.status, VerticalStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->deck_vertical_velocity_ned_mps, velocity_mps, 0.02);
  EXPECT_NEAR(
    estimate->deck_z_ned_m,
    initial_z_m + velocity_mps * 8.0,
    0.02);
}

TEST(VerticalStateEstimatorTest, MeasurementBiasCompensationRemovesKnownOffset)
{
  auto parameters = test_parameters();
  parameters.measurement_bias_m = 0.24;
  VerticalStateEstimator estimator(parameters);
  ASSERT_EQ(
    estimator.update(-1.76, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 80; ++index) {
    ASSERT_EQ(
      estimator.update(-1.76, 0.05 * index).status,
      VerticalStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->deck_z_ned_m, -2.0, 0.01);
}

TEST(VerticalStateEstimatorTest, FilterSuppressesAlternatingMeasurementNoise)
{
  auto parameters = test_parameters();
  parameters.measurement_std_m = 0.10;
  parameters.process_acceleration_std_mps2 = 0.05;
  VerticalStateEstimator estimator(parameters);
  ASSERT_EQ(
    estimator.update(-2.0, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);

  double maximum_final_error_m = 0.0;
  for (int index = 1; index <= 160; ++index) {
    const double noise_m = index % 2 == 0 ? 0.10 : -0.10;
    const auto result = estimator.update(-2.0 + noise_m, 0.05 * index);
    ASSERT_EQ(result.status, VerticalStateUpdateStatus::kUpdated);
    if (index > 120) {
      maximum_final_error_m = std::max(
        maximum_final_error_m,
        std::abs(result.estimate->deck_z_ned_m + 2.0));
    }
  }

  EXPECT_LT(maximum_final_error_m, 0.03);
}

TEST(VerticalStateEstimatorTest, RejectsRepeatedAndBackwardTime)
{
  VerticalStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(-2.0, 1.0).status,
    VerticalStateUpdateStatus::kInitialized);

  const auto repeated = estimator.update(-1.0, 1.0);
  EXPECT_EQ(repeated.status, VerticalStateUpdateStatus::kRejectedNonMonotonicTime);
  ASSERT_TRUE(repeated.estimate.has_value());
  EXPECT_DOUBLE_EQ(repeated.estimate->sample_time_s, 1.0);

  const auto backward = estimator.update(-1.0, 0.5);
  EXPECT_EQ(backward.status, VerticalStateUpdateStatus::kRejectedNonMonotonicTime);
  ASSERT_TRUE(backward.estimate.has_value());
  EXPECT_DOUBLE_EQ(backward.estimate->sample_time_s, 1.0);
}

TEST(VerticalStateEstimatorTest, RejectsOutlierAndRecovers)
{
  VerticalStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(-2.0, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);
  for (int index = 1; index <= 40; ++index) {
    ASSERT_EQ(
      estimator.update(-2.0, 0.05 * index).status,
      VerticalStateUpdateStatus::kUpdated);
  }

  const auto outlier = estimator.update(5.0, 2.05);
  EXPECT_EQ(outlier.status, VerticalStateUpdateStatus::kRejectedOutlier);
  ASSERT_TRUE(outlier.estimate.has_value());
  EXPECT_NEAR(outlier.estimate->deck_z_ned_m, -2.0, 0.05);

  const auto recovered = estimator.update(-2.0, 2.10);
  EXPECT_EQ(recovered.status, VerticalStateUpdateStatus::kUpdated);
  ASSERT_TRUE(recovered.estimate.has_value());
  EXPECT_NEAR(recovered.estimate->deck_z_ned_m, -2.0, 0.05);
}

TEST(VerticalStateEstimatorTest, LongGapReinitializesAndClearsVelocity)
{
  VerticalStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(-2.0, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);
  ASSERT_EQ(
    estimator.update(-1.9, 0.1).status,
    VerticalStateUpdateStatus::kUpdated);

  const auto result = estimator.update(-1.2, 2.5);

  EXPECT_EQ(result.status, VerticalStateUpdateStatus::kReinitialized);
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_NEAR(result.estimate->deck_z_ned_m, -1.2, 1.0e-12);
  EXPECT_NEAR(result.estimate->deck_vertical_velocity_ned_mps, 0.0, 1.0e-12);
}

TEST(VerticalStateEstimatorTest, CovarianceRemainsFiniteSymmetricAndNonnegative)
{
  VerticalStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(-2.0, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 100; ++index) {
    ASSERT_EQ(
      estimator.update(-2.0 + 0.001 * index, 0.04 * index).status,
      VerticalStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->covariance.allFinite());
  EXPECT_TRUE(
    estimate->covariance.isApprox(estimate->covariance.transpose(), 1.0e-12));
  EXPECT_GE(estimate->covariance(0, 0), 0.0);
  EXPECT_GE(estimate->covariance(1, 1), 0.0);
}

TEST(VerticalStateEstimatorTest, ResetClearsState)
{
  VerticalStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(-2.0, 0.0).status,
    VerticalStateUpdateStatus::kInitialized);
  ASSERT_TRUE(estimator.estimate().has_value());

  estimator.reset();

  EXPECT_FALSE(estimator.estimate().has_value());
  EXPECT_EQ(
    estimator.update(-1.5, 5.0).status,
    VerticalStateUpdateStatus::kInitialized);
}

TEST(VerticalStateEstimatorTest, RejectsInvalidInputAndParameters)
{
  VerticalStateEstimator estimator(test_parameters());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    estimator.update(nan, 0.0).status,
    VerticalStateUpdateStatus::kRejectedInvalidInput);
  EXPECT_EQ(
    estimator.update(-2.0, infinity).status,
    VerticalStateUpdateStatus::kRejectedInvalidInput);

  auto parameters = test_parameters();
  parameters.measurement_std_m = 0.0;
  EXPECT_THROW(VerticalStateEstimator invalid(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.measurement_bias_m = nan;
  EXPECT_THROW(VerticalStateEstimator invalid(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.maximum_sample_dt_s = parameters.minimum_sample_dt_s * 0.5;
  EXPECT_THROW(VerticalStateEstimator invalid(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.reinitialize_gap_s = parameters.maximum_sample_dt_s;
  EXPECT_THROW(VerticalStateEstimator invalid(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
