// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/target_state_estimator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

TargetStateEstimatorParameters test_parameters()
{
  TargetStateEstimatorParameters parameters;
  parameters.process_acceleration_std_mps2 = 0.3;
  parameters.measurement_horizontal_std_m = 0.03;
  parameters.measurement_vertical_std_m = 0.05;
  parameters.initial_position_std_m = 0.10;
  parameters.initial_velocity_std_mps = 2.0;
  parameters.minimum_sample_dt_s = 0.001;
  parameters.maximum_sample_dt_s = 0.25;
  parameters.reinitialize_gap_s = 2.0;
  parameters.innovation_gate_mahalanobis = 5.0;
  return parameters;
}

TEST(TargetStateEstimatorTest, FirstMeasurementInitializesPositionAndZeroVelocity)
{
  TargetStateEstimator estimator(test_parameters());
  const Eigen::Vector3d position{1.0, -2.0, 0.3};

  const auto result = estimator.update(position, 10.0);

  ASSERT_EQ(result.status, TargetStateUpdateStatus::kInitialized);
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_TRUE(result.estimate->position_ned.isApprox(position, 1.0e-12));
  EXPECT_TRUE(result.estimate->velocity_ned.isZero(1.0e-12));
  EXPECT_DOUBLE_EQ(result.estimate->sample_time_s, 10.0);
}

TEST(TargetStateEstimatorTest, StaticMeasurementsConvergeVelocityTowardZero)
{
  TargetStateEstimator estimator(test_parameters());
  const Eigen::Vector3d position{2.0, -1.0, 0.2};
  ASSERT_EQ(
    estimator.update(position, 0.0).status,
    TargetStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 80; ++index) {
    const auto result = estimator.update(position, 0.05 * index);
    ASSERT_EQ(result.status, TargetStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_LT(estimate->velocity_ned.norm(), 0.01);
  EXPECT_TRUE(estimate->position_ned.isApprox(position, 0.01));
}

TEST(TargetStateEstimatorTest, ConstantVelocityMeasurementsConvergeToTrueVelocity)
{
  TargetStateEstimator estimator(test_parameters());
  const Eigen::Vector3d initial_position{0.2, -0.4, 0.1};
  const Eigen::Vector3d true_velocity{0.25, 0.40, -0.05};
  ASSERT_EQ(
    estimator.update(initial_position, 0.0).status,
    TargetStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 120; ++index) {
    const double time_s = 0.05 * index;
    const auto result = estimator.update(
      initial_position + true_velocity * time_s,
      time_s);
    ASSERT_EQ(result.status, TargetStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->velocity_ned.isApprox(true_velocity, 0.025));
  EXPECT_TRUE(
    estimate->position_ned.isApprox(
      initial_position + true_velocity * 6.0,
      0.02));
}

TEST(TargetStateEstimatorTest, IrregularSampleIntervalsRemainAccurate)
{
  TargetStateEstimator estimator(test_parameters());
  const Eigen::Vector3d velocity{-0.15, 0.35, 0.02};
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), 0.0).status,
    TargetStateUpdateStatus::kInitialized);

  const std::vector<double> intervals{0.03, 0.08, 0.04, 0.12, 0.06, 0.20, 0.05};
  double time_s = 0.0;
  for (int cycle = 0; cycle < 20; ++cycle) {
    for (const double interval_s : intervals) {
      time_s += interval_s;
      const auto result = estimator.update(velocity * time_s, time_s);
      ASSERT_EQ(result.status, TargetStateUpdateStatus::kUpdated);
    }
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->velocity_ned.isApprox(velocity, 0.03));
}

TEST(TargetStateEstimatorTest, LargeRecoverableGapUsesBoundedPredictionSubsteps)
{
  auto parameters = test_parameters();
  parameters.maximum_sample_dt_s = 0.10;
  TargetStateEstimator estimator(parameters);
  const Eigen::Vector3d velocity{0.30, -0.10, 0.02};
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), 0.0).status,
    TargetStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 8; ++index) {
    const double time_s = 0.8 * index;
    const auto result = estimator.update(velocity * time_s, time_s);
    ASSERT_EQ(result.status, TargetStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->velocity_ned.isApprox(velocity, 0.08));
}

TEST(TargetStateEstimatorTest, RejectsRepeatedAndBackwardTimeWithoutChangingStateTime)
{
  TargetStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), 1.0).status,
    TargetStateUpdateStatus::kInitialized);

  const auto repeated = estimator.update(Eigen::Vector3d::Ones(), 1.0);
  EXPECT_EQ(repeated.status, TargetStateUpdateStatus::kRejectedNonMonotonicTime);
  ASSERT_TRUE(repeated.estimate.has_value());
  EXPECT_DOUBLE_EQ(repeated.estimate->sample_time_s, 1.0);

  const auto backward = estimator.update(Eigen::Vector3d::Ones(), 0.5);
  EXPECT_EQ(backward.status, TargetStateUpdateStatus::kRejectedNonMonotonicTime);
  ASSERT_TRUE(backward.estimate.has_value());
  EXPECT_DOUBLE_EQ(backward.estimate->sample_time_s, 1.0);
}

TEST(TargetStateEstimatorTest, RejectsOutlierAndRecoversOnFollowingMeasurement)
{
  TargetStateEstimator estimator(test_parameters());
  const Eigen::Vector3d velocity{0.2, 0.4, 0.0};
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), 0.0).status,
    TargetStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 30; ++index) {
    const double time_s = index * 0.1;
    ASSERT_EQ(
      estimator.update(velocity * time_s, time_s).status,
      TargetStateUpdateStatus::kUpdated);
  }

  const auto outlier = estimator.update(Eigen::Vector3d{10.0, -10.0, 5.0}, 3.1);
  EXPECT_EQ(outlier.status, TargetStateUpdateStatus::kRejectedOutlier);
  ASSERT_TRUE(outlier.estimate.has_value());
  EXPECT_LT((outlier.estimate->position_ned - velocity * 3.1).norm(), 0.2);

  const auto recovered = estimator.update(velocity * 3.2, 3.2);
  EXPECT_EQ(recovered.status, TargetStateUpdateStatus::kUpdated);
  ASSERT_TRUE(recovered.estimate.has_value());
  EXPECT_TRUE(recovered.estimate->velocity_ned.isApprox(velocity, 0.05));
}

TEST(TargetStateEstimatorTest, ContinuousOutliersDoNotTriggerReinitialization)
{
  TargetStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), 0.0).status,
    TargetStateUpdateStatus::kInitialized);
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d{0.02, 0.0, 0.0}, 0.1).status,
    TargetStateUpdateStatus::kUpdated);

  for (int index = 1; index <= 30; ++index) {
    const auto result = estimator.update(
      Eigen::Vector3d{10.0 + index, -10.0, 5.0},
      0.1 + index * 0.1);
    EXPECT_EQ(result.status, TargetStateUpdateStatus::kRejectedOutlier);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_LT(estimate->position_ned.norm(), 2.0);
}

TEST(TargetStateEstimatorTest, LongGapReinitializesAndClearsVelocity)
{
  TargetStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), 0.0).status,
    TargetStateUpdateStatus::kInitialized);
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d{0.1, 0.0, 0.0}, 0.1).status,
    TargetStateUpdateStatus::kUpdated);

  const Eigen::Vector3d reacquired_position{4.0, -3.0, 0.2};
  const auto result = estimator.update(reacquired_position, 2.5);

  EXPECT_EQ(result.status, TargetStateUpdateStatus::kReinitialized);
  ASSERT_TRUE(result.estimate.has_value());
  EXPECT_TRUE(result.estimate->position_ned.isApprox(reacquired_position, 1.0e-12));
  EXPECT_TRUE(result.estimate->velocity_ned.isZero(1.0e-12));
}

TEST(TargetStateEstimatorTest, RejectsInvalidMeasurementAndTime)
{
  TargetStateEstimator estimator(test_parameters());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_EQ(
    estimator.update(Eigen::Vector3d{nan, 0.0, 0.0}, 0.0).status,
    TargetStateUpdateStatus::kRejectedInvalidInput);
  EXPECT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), infinity).status,
    TargetStateUpdateStatus::kRejectedInvalidInput);
  EXPECT_FALSE(estimator.estimate().has_value());
}

TEST(TargetStateEstimatorTest, CovarianceRemainsFiniteSymmetricAndNonNegativeOnDiagonal)
{
  TargetStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Zero(), 0.0).status,
    TargetStateUpdateStatus::kInitialized);

  for (int index = 1; index <= 50; ++index) {
    const double time_s = 0.04 * index;
    ASSERT_EQ(
      estimator.update(Eigen::Vector3d{0.1 * time_s, 0.0, 0.2}, time_s).status,
      TargetStateUpdateStatus::kUpdated);
  }

  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->covariance.allFinite());
  EXPECT_TRUE(estimate->covariance.isApprox(estimate->covariance.transpose(), 1.0e-10));
  for (int index = 0; index < 6; ++index) {
    EXPECT_GE(estimate->covariance(index, index), 0.0);
  }
}

TEST(TargetStateEstimatorTest, ResetClearsState)
{
  TargetStateEstimator estimator(test_parameters());
  ASSERT_EQ(
    estimator.update(Eigen::Vector3d::Ones(), 1.0).status,
    TargetStateUpdateStatus::kInitialized);
  ASSERT_TRUE(estimator.estimate().has_value());

  estimator.reset();

  EXPECT_FALSE(estimator.estimate().has_value());
  EXPECT_EQ(
    estimator.update(Eigen::Vector3d{2.0, 3.0, 4.0}, 5.0).status,
    TargetStateUpdateStatus::kInitialized);
}

TEST(TargetStateEstimatorTest, RejectsInvalidParameterSets)
{
  auto parameters = test_parameters();
  parameters.process_acceleration_std_mps2 = 0.0;
  EXPECT_THROW(TargetStateEstimator estimator(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.maximum_sample_dt_s = parameters.minimum_sample_dt_s * 0.5;
  EXPECT_THROW(TargetStateEstimator estimator(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.reinitialize_gap_s = parameters.maximum_sample_dt_s;
  EXPECT_THROW(TargetStateEstimator estimator(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
