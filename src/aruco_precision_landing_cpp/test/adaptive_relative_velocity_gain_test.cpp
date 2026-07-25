// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/adaptive_relative_velocity_gain.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

AdaptiveRelativeVelocityGainParameters test_parameters()
{
  AdaptiveRelativeVelocityGainParameters parameters;
  parameters.min_gain = 0.25;
  parameters.max_gain = 1.0;
  parameters.acceleration_low_threshold_mps2 = 0.10;
  parameters.acceleration_high_threshold_mps2 = 0.50;
  parameters.max_acceleration_mps2 = 1.0;
  parameters.acceleration_filter_gain = 1.0;
  return parameters;
}

TEST(AdaptiveRelativeVelocityGainTest, FirstSampleReturnsMinimumGain)
{
  AdaptiveRelativeVelocityGain scheduler(test_parameters());

  const auto output = scheduler.update(Eigen::Vector2d{1.0, -2.0}, 0.1);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->gain, 0.25);
  EXPECT_TRUE(output->filtered_acceleration_xy.isZero(1.0e-12));
}

TEST(AdaptiveRelativeVelocityGainTest, ConstantVelocityKeepsMinimumGain)
{
  AdaptiveRelativeVelocityGain scheduler(test_parameters());
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d{0.4, 0.0}, 0.1).has_value());

  const auto output = scheduler.update(Eigen::Vector2d{0.4, 0.0}, 0.1);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->gain, 0.25);
  EXPECT_TRUE(output->filtered_acceleration_xy.isZero(1.0e-12));
}

TEST(AdaptiveRelativeVelocityGainTest, AccelerationBelowLowThresholdKeepsMinimumGain)
{
  AdaptiveRelativeVelocityGain scheduler(test_parameters());
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d::Zero(), 1.0).has_value());

  const auto output = scheduler.update(Eigen::Vector2d{0.05, 0.0}, 1.0);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->gain, 0.25);
  EXPECT_TRUE(output->filtered_acceleration_xy.isApprox(Eigen::Vector2d{0.05, 0.0}));
}

TEST(AdaptiveRelativeVelocityGainTest, AccelerationAboveHighThresholdUsesMaximumGain)
{
  AdaptiveRelativeVelocityGain scheduler(test_parameters());
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d::Zero(), 1.0).has_value());

  const auto output = scheduler.update(Eigen::Vector2d{0.7, 0.0}, 1.0);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->gain, 1.0);
}

TEST(AdaptiveRelativeVelocityGainTest, MiddleAccelerationUsesSmoothContinuousGain)
{
  AdaptiveRelativeVelocityGain scheduler(test_parameters());
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d::Zero(), 1.0).has_value());

  const auto output = scheduler.update(Eigen::Vector2d{0.3, 0.0}, 1.0);

  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->gain, 0.625, 1.0e-12);
  EXPECT_TRUE(output->filtered_acceleration_xy.isApprox(Eigen::Vector2d{0.3, 0.0}));
}

TEST(AdaptiveRelativeVelocityGainTest, LimitsRawAccelerationMagnitude)
{
  auto parameters = test_parameters();
  parameters.max_acceleration_mps2 = 0.6;
  AdaptiveRelativeVelocityGain scheduler(parameters);
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d::Zero(), 0.1).has_value());

  const auto output = scheduler.update(Eigen::Vector2d{3.0, 4.0}, 0.1);

  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->filtered_acceleration_xy.norm(), 0.6, 1.0e-12);
  EXPECT_DOUBLE_EQ(output->gain, 1.0);
}

TEST(AdaptiveRelativeVelocityGainTest, AppliesLowPassFilter)
{
  auto parameters = test_parameters();
  parameters.acceleration_filter_gain = 0.25;
  AdaptiveRelativeVelocityGain scheduler(parameters);
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d::Zero(), 1.0).has_value());

  const auto first = scheduler.update(Eigen::Vector2d{0.8, 0.0}, 1.0);
  const auto second = scheduler.update(Eigen::Vector2d{1.6, 0.0}, 1.0);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_TRUE(first->filtered_acceleration_xy.isApprox(Eigen::Vector2d{0.2, 0.0}));
  EXPECT_TRUE(second->filtered_acceleration_xy.isApprox(Eigen::Vector2d{0.35, 0.0}));
  EXPECT_LT(first->gain, second->gain);
}

TEST(AdaptiveRelativeVelocityGainTest, InvalidInputDoesNotChangeState)
{
  AdaptiveRelativeVelocityGain scheduler(test_parameters());
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d::Zero(), 1.0).has_value());

  EXPECT_FALSE(scheduler.update(Eigen::Vector2d::Ones(), 0.0).has_value());
  EXPECT_FALSE(scheduler.update(
    Eigen::Vector2d{std::numeric_limits<double>::quiet_NaN(), 0.0}, 1.0).has_value());

  const auto output = scheduler.update(Eigen::Vector2d{0.3, 0.0}, 1.0);
  ASSERT_TRUE(output.has_value());
  EXPECT_TRUE(output->filtered_acceleration_xy.isApprox(Eigen::Vector2d{0.3, 0.0}));
}

TEST(AdaptiveRelativeVelocityGainTest, ResetClearsVelocityAndAccelerationHistory)
{
  AdaptiveRelativeVelocityGain scheduler(test_parameters());
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d::Zero(), 1.0).has_value());
  ASSERT_TRUE(scheduler.update(Eigen::Vector2d{0.7, 0.0}, 1.0).has_value());

  scheduler.reset();
  const auto output = scheduler.update(Eigen::Vector2d{5.0, 0.0}, 1.0);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->gain, 0.25);
  EXPECT_TRUE(output->filtered_acceleration_xy.isZero(1.0e-12));
}

TEST(AdaptiveRelativeVelocityGainTest, RejectsInvalidParameters)
{
  auto parameters = test_parameters();
  parameters.min_gain = -0.1;
  EXPECT_THROW(AdaptiveRelativeVelocityGain scheduler(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.max_gain = 0.2;
  EXPECT_THROW(AdaptiveRelativeVelocityGain scheduler(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.acceleration_high_threshold_mps2 = 0.1;
  EXPECT_THROW(AdaptiveRelativeVelocityGain scheduler(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.max_acceleration_mps2 = 0.4;
  EXPECT_THROW(AdaptiveRelativeVelocityGain scheduler(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.acceleration_filter_gain = 0.0;
  EXPECT_THROW(AdaptiveRelativeVelocityGain scheduler(parameters), std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
