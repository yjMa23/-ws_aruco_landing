// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/landing_window.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

LandingWindowInput valid_input(double now_s)
{
  LandingWindowInput input;
  input.visual_fresh = true;
  input.estimate_valid = true;
  input.prediction_valid = true;
  input.visual_age_s = 0.05;
  input.horizontal_error_m = 0.10;
  input.horizontal_relative_speed_mps = 0.10;
  input.deck_roll_rad = 0.02;
  input.deck_pitch_rad = -0.03;
  input.relative_height_m = 5.0;
  input.now_s = now_s;
  return input;
}

bool has_reason(std::uint32_t mask, LandingWindowRejectReason reason)
{
  return (mask & landing_window_reason_mask(reason)) != 0U;
}

TEST(LandingWindowTest, OpensOnlyAfterContinuousRequiredDuration)
{
  LandingWindow window(LandingWindowParameters{});

  const auto initial = window.update(valid_input(10.0));
  const auto before_duration = window.update(valid_input(10.99));
  const auto at_duration = window.update(valid_input(11.0));

  EXPECT_FALSE(initial.window_open);
  EXPECT_TRUE(initial.conditions_currently_satisfied);
  EXPECT_NEAR(initial.satisfied_duration_s, 0.0, 1.0e-12);
  EXPECT_FALSE(before_duration.window_open);
  EXPECT_NEAR(before_duration.satisfied_duration_s, 0.99, 1.0e-12);
  EXPECT_TRUE(at_duration.window_open);
  EXPECT_TRUE(at_duration.conditions_currently_satisfied);
  EXPECT_NEAR(at_duration.satisfied_duration_s, 1.0, 1.0e-12);
}

TEST(LandingWindowTest, FailedConditionResetsContinuousDuration)
{
  LandingWindow window(LandingWindowParameters{});
  ASSERT_FALSE(window.update(valid_input(1.0)).window_open);
  ASSERT_FALSE(window.update(valid_input(1.8)).window_open);

  auto invalid = valid_input(1.9);
  invalid.horizontal_error_m = 0.16;
  const auto rejected = window.update(invalid);
  EXPECT_FALSE(rejected.window_open);
  EXPECT_FALSE(rejected.conditions_currently_satisfied);
  EXPECT_NEAR(rejected.satisfied_duration_s, 0.0, 1.0e-12);
  EXPECT_TRUE(has_reason(rejected.reject_reasons, LandingWindowRejectReason::kHorizontalError));

  EXPECT_FALSE(window.update(valid_input(2.0)).window_open);
  EXPECT_FALSE(window.update(valid_input(2.9)).window_open);
  EXPECT_TRUE(window.update(valid_input(3.0)).window_open);
}

TEST(LandingWindowTest, OpenWindowUsesExitThresholdHysteresis)
{
  LandingWindow window(LandingWindowParameters{});
  ASSERT_FALSE(window.update(valid_input(1.0)).window_open);
  ASSERT_TRUE(window.update(valid_input(2.0)).window_open);

  auto between_thresholds = valid_input(2.1);
  between_thresholds.horizontal_error_m = 0.20;
  between_thresholds.horizontal_relative_speed_mps = 0.20;
  between_thresholds.deck_roll_rad = 0.11;
  const auto held_open = window.update(between_thresholds);

  EXPECT_TRUE(held_open.window_open);
  EXPECT_TRUE(held_open.conditions_currently_satisfied);
  EXPECT_EQ(held_open.reject_reasons, 0U);

  between_thresholds.now_s = 2.2;
  between_thresholds.horizontal_error_m = 0.251;
  const auto closed = window.update(between_thresholds);
  EXPECT_FALSE(closed.window_open);
  EXPECT_TRUE(has_reason(closed.reject_reasons, LandingWindowRejectReason::kHorizontalError));
}

TEST(LandingWindowTest, HardValidityFailureClosesOpenWindow)
{
  LandingWindow window(LandingWindowParameters{});
  ASSERT_FALSE(window.update(valid_input(1.0)).window_open);
  ASSERT_TRUE(window.update(valid_input(2.0)).window_open);

  auto input = valid_input(2.1);
  input.visual_fresh = false;
  input.estimate_valid = false;
  input.prediction_valid = false;
  const auto result = window.update(input);

  EXPECT_FALSE(result.window_open);
  EXPECT_TRUE(has_reason(result.reject_reasons, LandingWindowRejectReason::kVisualUnavailable));
  EXPECT_TRUE(has_reason(result.reject_reasons, LandingWindowRejectReason::kEstimateInvalid));
  EXPECT_TRUE(has_reason(result.reject_reasons, LandingWindowRejectReason::kPredictionInvalid));
}

TEST(LandingWindowTest, RejectsOldVisualMeasurement)
{
  LandingWindow window(LandingWindowParameters{});
  auto input = valid_input(1.0);
  input.visual_age_s = 0.201;

  const auto result = window.update(input);

  EXPECT_FALSE(result.window_open);
  EXPECT_TRUE(has_reason(result.reject_reasons, LandingWindowRejectReason::kVisualTooOld));
}

TEST(LandingWindowTest, RejectsRelativeHeightOutsideAllowedRange)
{
  LandingWindow window(LandingWindowParameters{});
  auto input = valid_input(1.0);
  input.relative_height_m = 0.49;
  EXPECT_TRUE(has_reason(
    window.update(input).reject_reasons,
    LandingWindowRejectReason::kRelativeHeight));

  input.now_s = 2.0;
  input.relative_height_m = 6.01;
  EXPECT_TRUE(has_reason(
    window.update(input).reject_reasons,
    LandingWindowRejectReason::kRelativeHeight));
}

TEST(LandingWindowTest, ReportsMultipleNumericRejectReasons)
{
  LandingWindow window(LandingWindowParameters{});
  auto input = valid_input(1.0);
  input.horizontal_error_m = 0.20;
  input.horizontal_relative_speed_mps = 0.20;
  input.deck_pitch_rad = 0.10;

  const auto result = window.update(input);

  EXPECT_TRUE(has_reason(result.reject_reasons, LandingWindowRejectReason::kHorizontalError));
  EXPECT_TRUE(has_reason(result.reject_reasons, LandingWindowRejectReason::kRelativeSpeed));
  EXPECT_TRUE(has_reason(result.reject_reasons, LandingWindowRejectReason::kDeckTilt));
}

TEST(LandingWindowTest, TimeRegressionResetsWindow)
{
  LandingWindow window(LandingWindowParameters{});
  ASSERT_FALSE(window.update(valid_input(10.0)).window_open);
  ASSERT_TRUE(window.update(valid_input(11.0)).window_open);

  const auto regressed = window.update(valid_input(5.0));
  EXPECT_FALSE(regressed.window_open);
  EXPECT_TRUE(has_reason(regressed.reject_reasons, LandingWindowRejectReason::kInvalidTime));

  EXPECT_FALSE(window.update(valid_input(5.1)).window_open);
  EXPECT_TRUE(window.update(valid_input(6.1)).window_open);
}

TEST(LandingWindowTest, ResetClearsWindowAndTimer)
{
  LandingWindow window(LandingWindowParameters{});
  ASSERT_FALSE(window.update(valid_input(1.0)).window_open);
  ASSERT_TRUE(window.update(valid_input(2.0)).window_open);

  window.reset();

  const auto result = window.update(valid_input(10.0));
  EXPECT_FALSE(result.window_open);
  EXPECT_NEAR(result.satisfied_duration_s, 0.0, 1.0e-12);
}

TEST(LandingWindowTest, ZeroRequiredDurationOpensImmediately)
{
  LandingWindowParameters parameters;
  parameters.required_duration_s = 0.0;
  LandingWindow window(parameters);

  EXPECT_TRUE(window.update(valid_input(1.0)).window_open);
}

TEST(LandingWindowTest, RejectsInvalidMetricsAndTime)
{
  LandingWindow window(LandingWindowParameters{});
  auto input = valid_input(1.0);
  input.horizontal_error_m = std::numeric_limits<double>::quiet_NaN();
  input.horizontal_relative_speed_mps = std::numeric_limits<double>::infinity();
  input.deck_roll_rad = std::numeric_limits<double>::quiet_NaN();
  input.relative_height_m = std::numeric_limits<double>::quiet_NaN();
  const auto metrics = window.update(input);
  EXPECT_TRUE(has_reason(metrics.reject_reasons, LandingWindowRejectReason::kHorizontalError));
  EXPECT_TRUE(has_reason(metrics.reject_reasons, LandingWindowRejectReason::kRelativeSpeed));
  EXPECT_TRUE(has_reason(metrics.reject_reasons, LandingWindowRejectReason::kDeckTilt));
  EXPECT_TRUE(has_reason(metrics.reject_reasons, LandingWindowRejectReason::kRelativeHeight));

  input = valid_input(std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(has_reason(
    window.update(input).reject_reasons,
    LandingWindowRejectReason::kInvalidTime));
}

TEST(LandingWindowTest, RejectsInvalidParameterRelationships)
{
  LandingWindowParameters parameters;
  parameters.enter_horizontal_error_m = parameters.exit_horizontal_error_m;
  EXPECT_THROW((void)LandingWindow{parameters}, std::invalid_argument);

  parameters = LandingWindowParameters{};
  parameters.enter_relative_speed_mps = parameters.exit_relative_speed_mps;
  EXPECT_THROW((void)LandingWindow{parameters}, std::invalid_argument);

  parameters = LandingWindowParameters{};
  parameters.enter_max_tilt_rad = parameters.exit_max_tilt_rad;
  EXPECT_THROW((void)LandingWindow{parameters}, std::invalid_argument);

  parameters = LandingWindowParameters{};
  parameters.minimum_relative_height_m = parameters.maximum_relative_height_m;
  EXPECT_THROW((void)LandingWindow{parameters}, std::invalid_argument);

  parameters = LandingWindowParameters{};
  parameters.required_duration_s = -0.1;
  EXPECT_THROW((void)LandingWindow{parameters}, std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
