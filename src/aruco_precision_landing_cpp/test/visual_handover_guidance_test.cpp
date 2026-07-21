// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/visual_handover_guidance.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kTolerance = 1.0e-9;

VisualHandoverParameters default_parameters()
{
  VisualHandoverParameters parameters;
  parameters.handover_duration_s = 1.0;
  parameters.max_gnss_visual_difference_m = 3.0;
  parameters.max_visual_measurement_jump_m = 0.5;
  parameters.visual_loss_short_timeout_s = 0.5;
  parameters.visual_loss_long_timeout_s = 2.0;
  parameters.max_target_speed_mps = 2.0;
  parameters.max_target_step_m = 0.2;
  return parameters;
}

TEST(VisualHandoverGuidanceTest, FiltersOutOfOrderAndJumpingMeasurements)
{
  VisualHandoverGuidance guidance(default_parameters());
  EXPECT_TRUE(guidance.update_visual_position({1.0, 2.0, 3.0}, 1.0));
  EXPECT_FALSE(guidance.update_visual_position({1.1, 2.0, 3.0}, 0.9));
  EXPECT_FALSE(guidance.update_visual_position({2.0, 2.0, 3.0}, 1.1));

  const auto current = guidance.visual_position(1.1);
  ASSERT_TRUE(current.has_value());
  EXPECT_NEAR(current->x(), 1.0, kTolerance);
  EXPECT_NEAR(current->y(), 2.0, kTolerance);
}

TEST(VisualHandoverGuidanceTest, AllowsReinitializationAfterLongLoss)
{
  VisualHandoverGuidance guidance(default_parameters());
  ASSERT_TRUE(guidance.update_visual_position({0.0, 0.0, 0.0}, 0.0));
  EXPECT_TRUE(guidance.update_visual_position({5.0, 0.0, 0.0}, 2.1));

  const auto current = guidance.visual_position(2.1);
  ASSERT_TRUE(current.has_value());
  EXPECT_NEAR(current->x(), 5.0, kTolerance);
}

TEST(VisualHandoverGuidanceTest, ChecksGnssVisualHorizontalConsistency)
{
  VisualHandoverGuidance guidance(default_parameters());
  EXPECT_TRUE(guidance.consistent_with_gnss(
      {1.0, 2.0, -1.0}, {2.0, 4.0, 100.0}));
  EXPECT_FALSE(guidance.consistent_with_gnss(
      {1.0, 2.0, -1.0}, {5.0, 2.0, -1.0}));
}

TEST(VisualHandoverGuidanceTest, HandoverAlphaAndBlendAreMonotonic)
{
  VisualHandoverGuidance guidance(default_parameters());
  const Eigen::Vector2d gnss{0.0, 0.0};
  const Eigen::Vector2d visual{2.0, 4.0};

  const auto alpha0 = guidance.handover_alpha(0.0);
  const auto alpha_half = guidance.handover_alpha(0.5);
  const auto alpha_full = guidance.handover_alpha(1.0);
  ASSERT_TRUE(alpha0.has_value());
  ASSERT_TRUE(alpha_half.has_value());
  ASSERT_TRUE(alpha_full.has_value());
  EXPECT_DOUBLE_EQ(*alpha0, 0.0);
  EXPECT_DOUBLE_EQ(*alpha_half, 0.5);
  EXPECT_DOUBLE_EQ(*alpha_full, 1.0);

  const auto blended = guidance.blended_target_xy(gnss, visual, 0.5);
  ASSERT_TRUE(blended.has_value());
  EXPECT_NEAR(blended->x(), 1.0, kTolerance);
  EXPECT_NEAR(blended->y(), 2.0, kTolerance);
}

TEST(VisualHandoverGuidanceTest, ClassifiesFreshShortAndLongLoss)
{
  VisualHandoverGuidance guidance(default_parameters());
  ASSERT_TRUE(guidance.update_visual_position({0.0, 0.0, 0.0}, 1.0));

  EXPECT_EQ(guidance.loss_state(true, 1.1), VisualLossState::kFresh);
  EXPECT_EQ(guidance.loss_state(false, 1.6), VisualLossState::kShortLoss);
  EXPECT_EQ(guidance.loss_state(false, 3.0), VisualLossState::kLongLoss);
  EXPECT_FALSE(guidance.visual_position(1.6).has_value());
}

TEST(VisualHandoverGuidanceTest, LimitsTargetBySpeedAndPerCycleStep)
{
  VisualHandoverParameters parameters = default_parameters();
  parameters.max_target_speed_mps = 1.0;
  parameters.max_target_step_m = 0.2;
  VisualHandoverGuidance guidance(parameters);

  const auto speed_limited = guidance.limit_target_xy({0.0, 0.0}, {10.0, 0.0}, 0.1);
  ASSERT_TRUE(speed_limited.has_value());
  EXPECT_NEAR(speed_limited->x(), 0.1, kTolerance);

  const auto step_limited = guidance.limit_target_xy({0.0, 0.0}, {10.0, 0.0}, 1.0);
  ASSERT_TRUE(step_limited.has_value());
  EXPECT_NEAR(step_limited->x(), 0.2, kTolerance);
}

TEST(VisualHandoverGuidanceTest, ResetClearsMeasurementHistory)
{
  VisualHandoverGuidance guidance(default_parameters());
  ASSERT_TRUE(guidance.update_visual_position({0.0, 0.0, 0.0}, 1.0));
  guidance.reset();
  EXPECT_FALSE(guidance.visual_position(1.0).has_value());
  EXPECT_EQ(guidance.loss_state(false, 1.0), VisualLossState::kLongLoss);
  EXPECT_TRUE(guidance.update_visual_position({10.0, 0.0, 0.0}, 1.0));
}

TEST(VisualHandoverGuidanceTest, RejectsInvalidParametersAndInputs)
{
  VisualHandoverParameters parameters = default_parameters();
  parameters.handover_duration_s = 0.0;
  EXPECT_THROW(VisualHandoverGuidance{parameters}, std::invalid_argument);

  parameters = default_parameters();
  parameters.visual_loss_long_timeout_s = parameters.visual_loss_short_timeout_s;
  EXPECT_THROW(VisualHandoverGuidance{parameters}, std::invalid_argument);

  VisualHandoverGuidance guidance(default_parameters());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(guidance.update_visual_position({nan, 0.0, 0.0}, 0.0));
  EXPECT_FALSE(guidance.update_visual_position({0.0, 0.0, 0.0}, -1.0));
  EXPECT_FALSE(guidance.handover_alpha(-1.0).has_value());
  EXPECT_FALSE(guidance.blended_target_xy({nan, 0.0}, {0.0, 0.0}, 0.0).has_value());
  EXPECT_FALSE(guidance.limit_target_xy({0.0, 0.0}, {1.0, 0.0}, 0.0).has_value());
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
