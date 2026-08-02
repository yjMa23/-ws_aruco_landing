// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/touchdown_hold_controller.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

TouchdownHoldInput valid_input(
  double dt_s,
  double uav_z_ned_m,
  double deck_z_ned_m,
  double deck_vertical_velocity_ned_mps)
{
  TouchdownHoldInput input;
  input.dt_s = dt_s;
  input.deck_state_valid = true;
  input.uav_z_ned_m = uav_z_ned_m;
  input.deck_z_ned_m = deck_z_ned_m;
  input.deck_vertical_velocity_ned_mps = deck_vertical_velocity_ned_mps;
  return input;
}

TEST(TouchdownHoldControllerTest, InitializesWithoutWorldTargetJump)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});

  const auto output = controller.update(valid_input(0.05, 1.80, 2.00, 0.08));

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->relative_height_reference_m, 0.20);
  EXPECT_DOUBLE_EQ(output->vertical_target_z_ned_m, 1.80);
  ASSERT_TRUE(output->deck_vertical_velocity_ned_mps.has_value());
  EXPECT_DOUBLE_EQ(*output->deck_vertical_velocity_ned_mps, 0.08);
  EXPECT_EQ(output->mode, TouchdownHoldMode::kRelativeDeckHold);
  EXPECT_EQ(output->reason, TouchdownHoldReason::kTrackingDeck);
}

TEST(TouchdownHoldControllerTest, TracksDeckMotionAtConstantRelativeHeight)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  ASSERT_TRUE(controller.update(valid_input(0.05, 1.80, 2.00, 0.05)).has_value());

  const auto deck_down = controller.update(valid_input(0.10, 1.80, 2.03, 0.05));
  ASSERT_TRUE(deck_down.has_value());
  EXPECT_NEAR(deck_down->vertical_target_z_ned_m, 1.83, 1.0e-12);
  EXPECT_NEAR(
    2.03 - deck_down->vertical_target_z_ned_m,
    deck_down->relative_height_reference_m,
    1.0e-12);

  const auto deck_up = controller.update(valid_input(0.10, 1.83, 1.98, -0.05));
  ASSERT_TRUE(deck_up.has_value());
  EXPECT_NEAR(deck_up->vertical_target_z_ned_m, 1.78, 1.0e-12);
  EXPECT_NEAR(
    1.98 - deck_up->vertical_target_z_ned_m,
    deck_up->relative_height_reference_m,
    1.0e-12);
}

TEST(TouchdownHoldControllerTest, StationaryDeckNoiseDoesNotMoveTargetOrFeedForward)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  const auto initialized = controller.update(valid_input(0.05, 1.80, 2.00, 0.01));
  ASSERT_TRUE(initialized.has_value());
  EXPECT_EQ(initialized->mode, TouchdownHoldMode::kStationaryDeckHold);
  EXPECT_EQ(
    initialized->reason,
    TouchdownHoldReason::kDeckMotionBelowThreshold);
  EXPECT_FALSE(initialized->deck_vertical_velocity_ned_mps.has_value());

  const auto noisy = controller.update(valid_input(0.10, 1.80, 2.06, 0.03));
  ASSERT_TRUE(noisy.has_value());
  EXPECT_DOUBLE_EQ(noisy->vertical_target_z_ned_m, 1.80);
  EXPECT_EQ(noisy->mode, TouchdownHoldMode::kStationaryDeckHold);
  EXPECT_FALSE(noisy->deck_vertical_velocity_ned_mps.has_value());
}

TEST(TouchdownHoldControllerTest, MotionHysteresisAvoidsModeChatter)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  ASSERT_TRUE(controller.update(valid_input(0.05, 1.80, 2.00, 0.05)).has_value());

  const auto remains_active = controller.update(valid_input(0.10, 1.80, 2.02, 0.03));
  ASSERT_TRUE(remains_active.has_value());
  EXPECT_EQ(remains_active->mode, TouchdownHoldMode::kRelativeDeckHold);

  const auto becomes_stationary =
    controller.update(valid_input(0.10, 1.82, 2.03, 0.02));
  ASSERT_TRUE(becomes_stationary.has_value());
  EXPECT_EQ(becomes_stationary->mode, TouchdownHoldMode::kStationaryDeckHold);

  const auto stays_stationary =
    controller.update(valid_input(0.10, 1.82, 2.08, 0.03));
  ASSERT_TRUE(stays_stationary.has_value());
  EXPECT_EQ(stays_stationary->mode, TouchdownHoldMode::kStationaryDeckHold);
  EXPECT_DOUBLE_EQ(
    stays_stationary->vertical_target_z_ned_m,
    becomes_stationary->vertical_target_z_ned_m);
}

TEST(TouchdownHoldControllerTest, OptionalPreloadLowersReferenceAtBoundedRate)
{
  TouchdownHoldParameters parameters;
  parameters.max_reference_preload_rate_mps = 0.05;
  TouchdownHoldController controller(parameters);
  auto input = valid_input(0.05, 1.766, 2.00, 0.0);
  input.relative_height_target_m = 0.20;

  const auto first = controller.update(input);
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(first->relative_height_reference_m, 0.2315, 1.0e-12);
  EXPECT_NEAR(first->vertical_target_z_ned_m, 1.7685, 1.0e-12);

  input.dt_s = 0.10;
  const auto second = controller.update(input);
  ASSERT_TRUE(second.has_value());
  EXPECT_NEAR(second->relative_height_reference_m, 0.2265, 1.0e-12);
  EXPECT_NEAR(second->vertical_target_z_ned_m, 1.7735, 1.0e-12);
}

TEST(TouchdownHoldControllerTest, MissingPreloadPreservesStationaryTarget)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  const auto first = controller.update(valid_input(0.05, 1.80, 2.00, 0.0));
  ASSERT_TRUE(first.has_value());
  const auto second = controller.update(valid_input(0.10, 1.80, 2.10, 0.0));
  ASSERT_TRUE(second.has_value());
  EXPECT_DOUBLE_EQ(second->relative_height_reference_m, 0.20);
  EXPECT_DOUBLE_EQ(second->vertical_target_z_ned_m, 1.80);
}

TEST(TouchdownHoldControllerTest, RejectsInvalidPreloadTarget)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  auto input = valid_input(0.05, 1.80, 2.00, 0.0);
  input.relative_height_target_m = -0.01;
  EXPECT_FALSE(controller.update(input).has_value());
  input.relative_height_target_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.update(input).has_value());
}

TEST(TouchdownHoldControllerTest, LimitsVerticalTargetRate)
{
  TouchdownHoldParameters parameters;
  parameters.max_target_rate_mps = 0.40;
  TouchdownHoldController controller(parameters);
  ASSERT_TRUE(controller.update(valid_input(0.05, 1.80, 2.00, 0.0)).has_value());

  const auto output = controller.update(valid_input(0.10, 1.80, 2.50, 0.20));

  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->vertical_target_z_ned_m, 1.84, 1.0e-12);
}

TEST(TouchdownHoldControllerTest, InvalidDeckEstimateHoldsLastSafeTarget)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  const auto initialized = controller.update(valid_input(0.05, 1.80, 2.00, 0.05));
  ASSERT_TRUE(initialized.has_value());

  TouchdownHoldInput invalid;
  invalid.dt_s = 0.05;
  invalid.deck_state_valid = false;
  invalid.uav_z_ned_m = 1.81;
  invalid.deck_z_ned_m = std::numeric_limits<double>::quiet_NaN();
  invalid.deck_vertical_velocity_ned_mps = std::numeric_limits<double>::quiet_NaN();
  const auto output = controller.update(invalid);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(
    output->vertical_target_z_ned_m,
    initialized->vertical_target_z_ned_m);
  EXPECT_FALSE(output->deck_vertical_velocity_ned_mps.has_value());
  EXPECT_EQ(output->mode, TouchdownHoldMode::kHoldLastTarget);
  EXPECT_EQ(output->reason, TouchdownHoldReason::kDeckStateInvalid);
}

TEST(TouchdownHoldControllerTest, EstimateRecoveryRemainsRateLimited)
{
  TouchdownHoldParameters parameters;
  parameters.max_target_rate_mps = 0.20;
  TouchdownHoldController controller(parameters);
  ASSERT_TRUE(controller.update(valid_input(0.05, 1.80, 2.00, 0.0)).has_value());

  TouchdownHoldInput invalid;
  invalid.dt_s = 0.10;
  invalid.deck_state_valid = false;
  invalid.uav_z_ned_m = 1.80;
  ASSERT_TRUE(controller.update(invalid).has_value());

  const auto recovered = controller.update(valid_input(0.10, 1.80, 2.40, 0.10));

  ASSERT_TRUE(recovered.has_value());
  EXPECT_NEAR(recovered->vertical_target_z_ned_m, 1.82, 1.0e-12);
  EXPECT_EQ(recovered->mode, TouchdownHoldMode::kRelativeDeckHold);
}

TEST(TouchdownHoldControllerTest, InvalidInputBeforeInitializationCannotCreateTarget)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  TouchdownHoldInput invalid;
  invalid.dt_s = 0.05;
  invalid.deck_state_valid = false;

  EXPECT_FALSE(controller.update(invalid).has_value());
  EXPECT_FALSE(controller.initialized());
}

TEST(TouchdownHoldControllerTest, InvalidTimeDoesNotChangeState)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  const auto initialized = controller.update(valid_input(0.05, 1.80, 2.00, 0.0));
  ASSERT_TRUE(initialized.has_value());

  auto invalid = valid_input(0.0, 1.80, 2.20, 0.10);
  EXPECT_FALSE(controller.update(invalid).has_value());

  const auto next = controller.update(valid_input(0.05, 1.80, 2.00, 0.0));
  ASSERT_TRUE(next.has_value());
  EXPECT_DOUBLE_EQ(
    next->vertical_target_z_ned_m,
    initialized->vertical_target_z_ned_m);
}

TEST(TouchdownHoldControllerTest, ResetClearsReferenceAndTarget)
{
  TouchdownHoldController controller(TouchdownHoldParameters{});
  ASSERT_TRUE(controller.update(valid_input(0.05, 1.80, 2.00, 0.0)).has_value());
  ASSERT_TRUE(controller.initialized());

  controller.reset();

  EXPECT_FALSE(controller.initialized());
  TouchdownHoldInput invalid;
  invalid.dt_s = 0.05;
  invalid.deck_state_valid = false;
  EXPECT_FALSE(controller.update(invalid).has_value());
}

TEST(TouchdownHoldControllerTest, RejectsInvalidParameters)
{
  auto parameters = TouchdownHoldParameters{};
  parameters.max_target_rate_mps = 0.0;
  EXPECT_THROW(
    {TouchdownHoldController invalid_controller(parameters);},
    std::invalid_argument);

  parameters.max_target_rate_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    {TouchdownHoldController invalid_controller(parameters);},
    std::invalid_argument);

  parameters = TouchdownHoldParameters{};
  parameters.max_reference_preload_rate_mps = 0.0;
  EXPECT_THROW(
    {TouchdownHoldController invalid_controller(parameters);},
    std::invalid_argument);

  parameters = TouchdownHoldParameters{};
  parameters.motion_exit_speed_mps = parameters.motion_enter_speed_mps;
  EXPECT_THROW(
    {TouchdownHoldController invalid_controller(parameters);},
    std::invalid_argument);

  parameters = TouchdownHoldParameters{};
  parameters.motion_exit_speed_mps = -0.01;
  EXPECT_THROW(
    {TouchdownHoldController invalid_controller(parameters);},
    std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
