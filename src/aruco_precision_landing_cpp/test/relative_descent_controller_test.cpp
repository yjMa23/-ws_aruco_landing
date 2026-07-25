// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/relative_descent_controller.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

RelativeDescentInput make_input(
  double height_m,
  bool window_open,
  double dt_s = 1.0)
{
  RelativeDescentInput input;
  input.current_relative_height_m = height_m;
  input.window_open = window_open;
  input.vertical_reference_valid = true;
  input.severe_failure = false;
  input.dt_s = dt_s;
  return input;
}

TEST(RelativeDescentControllerTest, InitializesFromCurrentHeightWithoutJump)
{
  RelativeDescentController controller(RelativeDescentParameters{});

  const auto output = controller.update(make_input(5.2, false));

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->height_reference_m, 5.2);
  EXPECT_DOUBLE_EQ(output->vertical_reference_velocity_ned_mps, 0.0);
  EXPECT_EQ(output->phase, RelativeDescentPhase::kWaitingWindow);
  EXPECT_FALSE(output->reference_changed);
  EXPECT_FALSE(output->reached_test_height);
  EXPECT_TRUE(controller.initialized());
}

TEST(RelativeDescentControllerTest, UsesFastMediumAndSlowRates)
{
  RelativeDescentController fast_controller(RelativeDescentParameters{});
  ASSERT_TRUE(fast_controller.update(make_input(5.0, true)).has_value());
  const auto fast = fast_controller.update(make_input(5.0, true));
  ASSERT_TRUE(fast.has_value());
  EXPECT_NEAR(fast->height_reference_m, 4.70, 1.0e-12);
  EXPECT_NEAR(fast->vertical_reference_velocity_ned_mps, 0.30, 1.0e-12);
  EXPECT_EQ(fast->phase, RelativeDescentPhase::kDescending);

  RelativeDescentController medium_controller(RelativeDescentParameters{});
  ASSERT_TRUE(medium_controller.update(make_input(1.50, true)).has_value());
  const auto medium = medium_controller.update(make_input(1.50, true));
  ASSERT_TRUE(medium.has_value());
  EXPECT_NEAR(medium->height_reference_m, 1.35, 1.0e-12);
  EXPECT_NEAR(medium->vertical_reference_velocity_ned_mps, 0.15, 1.0e-12);

  RelativeDescentController slow_controller(RelativeDescentParameters{});
  ASSERT_TRUE(slow_controller.update(make_input(0.70, true)).has_value());
  const auto slow = slow_controller.update(make_input(0.70, true));
  ASSERT_TRUE(slow.has_value());
  EXPECT_NEAR(slow->height_reference_m, 0.65, 1.0e-12);
  EXPECT_NEAR(slow->vertical_reference_velocity_ned_mps, 0.05, 1.0e-12);
}

TEST(RelativeDescentControllerTest, ClampsAtMinimumTestHeight)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  ASSERT_TRUE(controller.update(make_input(0.52, true)).has_value());

  const auto output = controller.update(make_input(0.52, true));
  const auto held = controller.update(make_input(0.50, true));

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->height_reference_m, 0.50);
  EXPECT_EQ(output->phase, RelativeDescentPhase::kTestHeightHold);
  EXPECT_TRUE(output->reached_test_height);
  ASSERT_TRUE(held.has_value());
  EXPECT_DOUBLE_EQ(held->height_reference_m, 0.50);
  EXPECT_DOUBLE_EQ(held->vertical_reference_velocity_ned_mps, 0.0);
  EXPECT_FALSE(held->reference_changed);
}

TEST(RelativeDescentControllerTest, ClosedWindowPausesAfterDescentStarts)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  ASSERT_TRUE(controller.update(make_input(3.0, false)).has_value());
  const auto descending = controller.update(make_input(3.0, true));
  ASSERT_TRUE(descending.has_value());
  ASSERT_NEAR(descending->height_reference_m, 2.70, 1.0e-12);

  const auto paused = controller.update(make_input(2.7, false));

  ASSERT_TRUE(paused.has_value());
  EXPECT_NEAR(paused->height_reference_m, 2.70, 1.0e-12);
  EXPECT_EQ(paused->phase, RelativeDescentPhase::kPaused);
  EXPECT_FALSE(paused->reference_changed);
}

TEST(RelativeDescentControllerTest, ClosedWindowBeforeDescentRemainsWaiting)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  ASSERT_TRUE(controller.update(make_input(5.0, false)).has_value());

  const auto output = controller.update(make_input(5.0, false));

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, RelativeDescentPhase::kWaitingWindow);
  EXPECT_DOUBLE_EQ(output->height_reference_m, 5.0);
}

TEST(RelativeDescentControllerTest, LargeReferenceTrackingErrorPauses)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  ASSERT_TRUE(controller.update(make_input(5.0, true)).has_value());
  const auto descending = controller.update(make_input(5.0, true));
  ASSERT_TRUE(descending.has_value());
  ASSERT_NEAR(descending->height_reference_m, 4.70, 1.0e-12);

  const auto paused = controller.update(make_input(5.30, true));

  ASSERT_TRUE(paused.has_value());
  EXPECT_EQ(paused->phase, RelativeDescentPhase::kPaused);
  EXPECT_NEAR(paused->height_reference_m, 4.70, 1.0e-12);
}

TEST(RelativeDescentControllerTest, SevereFailureRaisesReferenceTowardRecoveryHeight)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  ASSERT_TRUE(controller.update(make_input(1.0, true)).has_value());
  const auto descending = controller.update(make_input(1.0, true));
  ASSERT_TRUE(descending.has_value());
  ASSERT_NEAR(descending->height_reference_m, 0.85, 1.0e-12);

  auto failure_input = make_input(0.90, false);
  failure_input.severe_failure = true;
  const auto recovery = controller.update(failure_input);

  ASSERT_TRUE(recovery.has_value());
  EXPECT_EQ(recovery->phase, RelativeDescentPhase::kRecovering);
  EXPECT_NEAR(recovery->height_reference_m, 1.15, 1.0e-12);
  EXPECT_NEAR(recovery->vertical_reference_velocity_ned_mps, -0.30, 1.0e-12);
  EXPECT_TRUE(recovery->reference_changed);

  for (int iteration = 0; iteration < 10; ++iteration) {
    ASSERT_TRUE(controller.update(failure_input).has_value());
  }
  const auto held = controller.update(failure_input);
  ASSERT_TRUE(held.has_value());
  EXPECT_DOUBLE_EQ(held->height_reference_m, 2.0);
  EXPECT_EQ(held->phase, RelativeDescentPhase::kPaused);
}

TEST(RelativeDescentControllerTest, SevereFailureAboveRecoveryHeightDoesNotDescend)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  auto input = make_input(5.0, true);
  input.severe_failure = true;

  const auto initialized = controller.update(input);
  const auto held = controller.update(input);

  ASSERT_TRUE(initialized.has_value());
  EXPECT_DOUBLE_EQ(initialized->height_reference_m, 5.0);
  ASSERT_TRUE(held.has_value());
  EXPECT_DOUBLE_EQ(held->height_reference_m, 5.0);
  EXPECT_EQ(held->phase, RelativeDescentPhase::kPaused);
}

TEST(RelativeDescentControllerTest, RejectsInvalidInputWithoutChangingState)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  auto input = make_input(5.0, true);
  input.vertical_reference_valid = false;
  EXPECT_FALSE(controller.update(input).has_value());
  EXPECT_FALSE(controller.initialized());

  input = make_input(-0.1, true);
  EXPECT_FALSE(controller.update(input).has_value());
  input = make_input(5.0, true, 0.0);
  EXPECT_FALSE(controller.update(input).has_value());
  input = make_input(std::numeric_limits<double>::quiet_NaN(), true);
  EXPECT_FALSE(controller.update(input).has_value());
}

TEST(RelativeDescentControllerTest, ResetClearsReferenceAndHistory)
{
  RelativeDescentController controller(RelativeDescentParameters{});
  ASSERT_TRUE(controller.update(make_input(5.0, true)).has_value());
  ASSERT_TRUE(controller.update(make_input(5.0, true)).has_value());

  controller.reset();

  EXPECT_FALSE(controller.initialized());
  const auto output = controller.update(make_input(3.0, false));
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->height_reference_m, 3.0);
  EXPECT_EQ(output->phase, RelativeDescentPhase::kWaitingWindow);
}

TEST(RelativeDescentControllerTest, RejectsInvalidParameters)
{
  RelativeDescentParameters parameters;
  parameters.minimum_test_height_m = 0.0;
  EXPECT_THROW((void)RelativeDescentController{parameters}, std::invalid_argument);

  parameters = RelativeDescentParameters{};
  parameters.slow_height_threshold_m = parameters.minimum_test_height_m;
  EXPECT_THROW((void)RelativeDescentController{parameters}, std::invalid_argument);

  parameters = RelativeDescentParameters{};
  parameters.fast_height_threshold_m = parameters.slow_height_threshold_m;
  EXPECT_THROW((void)RelativeDescentController{parameters}, std::invalid_argument);

  parameters = RelativeDescentParameters{};
  parameters.recovery_height_m = parameters.minimum_test_height_m;
  EXPECT_THROW((void)RelativeDescentController{parameters}, std::invalid_argument);

  parameters = RelativeDescentParameters{};
  parameters.fast_rate_mps = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)RelativeDescentController{parameters}, std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
