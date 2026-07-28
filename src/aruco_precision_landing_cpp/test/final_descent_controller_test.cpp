// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/final_descent_controller.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

FinalDescentInput make_input(double time_step_s = 1.0)
{
  FinalDescentInput input;
  input.current_relative_height_m = 0.50;
  input.current_reference_height_m = 0.50;
  input.final_descent_authorized = true;
  input.vertical_reference_valid = true;
  input.landing_window_open = true;
  input.touchdown_status = TouchdownStatus::kAirborne;
  input.dt_s = time_step_s;
  return input;
}

TEST(FinalDescentControllerTest, InitializesWithoutReferenceJump)
{
  FinalDescentController controller(FinalDescentParameters{});
  const auto output = controller.update(make_input());

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->relative_height_reference_m, 0.50);
  EXPECT_DOUBLE_EQ(output->vertical_reference_velocity_ned_mps, 0.0);
  EXPECT_FALSE(output->reference_changed);
  EXPECT_TRUE(controller.initialized());
}

TEST(FinalDescentControllerTest, UnauthorizedInputDoesNotDescend)
{
  FinalDescentController controller(FinalDescentParameters{});
  auto input = make_input();
  input.final_descent_authorized = false;
  ASSERT_TRUE(controller.update(input).has_value());
  const auto output = controller.update(input);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kWaitingAuthorization);
  EXPECT_DOUBLE_EQ(output->relative_height_reference_m, 0.50);
}

TEST(FinalDescentControllerTest, AuthorizedInputUsesApproachRate)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  const auto output = controller.update(make_input());

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kDescending);
  EXPECT_NEAR(output->relative_height_reference_m, 0.38, 1.0e-12);
  EXPECT_NEAR(output->vertical_reference_velocity_ned_mps, 0.12, 1.0e-12);
  EXPECT_TRUE(output->reference_changed);
}

TEST(FinalDescentControllerTest, SlowsBeforeNearContactSegment)
{
  FinalDescentParameters parameters;
  parameters.approach_rate_mps = 0.20;
  parameters.contact_rate_mps = 0.03;
  FinalDescentController controller(parameters);
  ASSERT_TRUE(controller.update(make_input()).has_value());

  auto output = controller.update(make_input());
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->relative_height_reference_m, 0.30, 1.0e-12);
  EXPECT_NEAR(output->vertical_reference_velocity_ned_mps, 0.20, 1.0e-12);

  output = controller.update(make_input());
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->relative_height_reference_m, 0.25, 1.0e-12);
  EXPECT_NEAR(output->vertical_reference_velocity_ned_mps, 0.05, 1.0e-12);

  auto contact_input = make_input();
  contact_input.current_relative_height_m = 0.25;
  contact_input.current_reference_height_m = 0.25;
  output = controller.update(contact_input);
  ASSERT_TRUE(output.has_value());
  EXPECT_NEAR(output->relative_height_reference_m, 0.22, 1.0e-12);
  EXPECT_NEAR(output->vertical_reference_velocity_ned_mps, 0.03, 1.0e-12);
}

TEST(FinalDescentControllerTest, ClampsAtMinimumCommandHeight)
{
  FinalDescentParameters parameters;
  parameters.approach_rate_mps = 0.20;
  parameters.contact_rate_mps = 0.20;
  FinalDescentController controller(parameters);
  auto input = make_input();
  ASSERT_TRUE(controller.update(input).has_value());

  std::optional<FinalDescentOutput> output;
  for (int index = 0; index < 3; ++index) {
    output = controller.update(input);
    ASSERT_TRUE(output.has_value());
    input.current_relative_height_m = output->relative_height_reference_m;
    input.current_reference_height_m = output->relative_height_reference_m;
  }
  output = controller.update(input);

  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->relative_height_reference_m, 0.15);
  EXPECT_EQ(output->phase, FinalDescentPhase::kPaused);
  EXPECT_DOUBLE_EQ(output->vertical_reference_velocity_ned_mps, 0.0);
}

TEST(FinalDescentControllerTest, CandidateImmediatelyHoldsReference)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto candidate = make_input();
  candidate.current_reference_height_m = 0.38;
  candidate.touchdown_status = TouchdownStatus::kCandidate;

  const auto output = controller.update(candidate);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kCandidateHold);
  EXPECT_NEAR(output->relative_height_reference_m, 0.38, 1.0e-12);
  EXPECT_DOUBLE_EQ(output->vertical_reference_velocity_ned_mps, 0.0);
  EXPECT_TRUE(output->touchdown_candidate_hold);
}

TEST(FinalDescentControllerTest, CandidateLossCanResumeDescent)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto candidate = make_input();
  candidate.current_reference_height_m = 0.38;
  candidate.touchdown_status = TouchdownStatus::kCandidate;
  ASSERT_EQ(
    controller.update(candidate)->phase,
    FinalDescentPhase::kCandidateHold);

  auto resumed = make_input();
  resumed.current_relative_height_m = 0.38;
  resumed.current_reference_height_m = 0.38;
  const auto output = controller.update(resumed);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kDescending);
  EXPECT_NEAR(output->relative_height_reference_m, 0.26, 1.0e-12);
}

TEST(FinalDescentControllerTest, ConfirmationLatchesTouchdownHold)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto confirmed = make_input();
  confirmed.touchdown_status = TouchdownStatus::kConfirmed;
  ASSERT_EQ(
    controller.update(confirmed)->phase,
    FinalDescentPhase::kTouchdownHold);

  auto unsafe = make_input();
  unsafe.touchdown_status = TouchdownStatus::kRejectedUnsafe;
  const auto output = controller.update(unsafe);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kTouchdownHold);
  EXPECT_TRUE(output->touchdown_confirmed_hold);
  EXPECT_DOUBLE_EQ(output->vertical_reference_velocity_ned_mps, 0.0);
}

TEST(FinalDescentControllerTest, InsufficientEvidencePauses)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto input = make_input();
  input.touchdown_status = TouchdownStatus::kInsufficientEvidence;

  const auto output = controller.update(input);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kPaused);
  EXPECT_FALSE(output->reference_changed);
}

TEST(FinalDescentControllerTest, UnsafeEvidenceRequestsRecovery)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto input = make_input();
  input.touchdown_status = TouchdownStatus::kRejectedUnsafe;

  const auto output = controller.update(input);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kRecoveryRequested);
  EXPECT_TRUE(output->recovery_requested);
}

TEST(FinalDescentControllerTest, ClosedLandingWindowPauses)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto input = make_input();
  input.landing_window_open = false;

  const auto output = controller.update(input);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kPaused);
}

TEST(FinalDescentControllerTest, ExcessiveTrackingErrorPauses)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto input = make_input();
  input.current_relative_height_m = 0.80;

  const auto output = controller.update(input);

  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(output->phase, FinalDescentPhase::kPaused);
}

TEST(FinalDescentControllerTest, RejectsInvalidInput)
{
  FinalDescentController controller(FinalDescentParameters{});
  auto input = make_input();
  input.vertical_reference_valid = false;
  EXPECT_FALSE(controller.update(input).has_value());

  input = make_input(0.0);
  EXPECT_FALSE(controller.update(input).has_value());

  input = make_input();
  input.current_relative_height_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.update(input).has_value());
}

TEST(FinalDescentControllerTest, ResetClearsHistoryAndConfirmation)
{
  FinalDescentController controller(FinalDescentParameters{});
  ASSERT_TRUE(controller.update(make_input()).has_value());
  auto confirmed = make_input();
  confirmed.touchdown_status = TouchdownStatus::kConfirmed;
  ASSERT_TRUE(controller.update(confirmed)->touchdown_confirmed_hold);

  controller.reset();
  EXPECT_FALSE(controller.initialized());
  const auto output = controller.update(make_input());
  ASSERT_TRUE(output.has_value());
  EXPECT_FALSE(output->touchdown_confirmed_hold);
  EXPECT_DOUBLE_EQ(output->relative_height_reference_m, 0.50);
}

TEST(FinalDescentControllerTest, RejectsInvalidParameterSets)
{
  auto parameters = FinalDescentParameters{};
  parameters.approach_rate_mps = 0.0;
  EXPECT_THROW(
    {FinalDescentController invalid_controller(parameters);},
    std::invalid_argument);

  parameters = FinalDescentParameters{};
  parameters.contact_slowdown_height_m = parameters.entry_height_m;
  EXPECT_THROW(
    {FinalDescentController invalid_controller(parameters);},
    std::invalid_argument);

  parameters = FinalDescentParameters{};
  parameters.approach_rate_mps = 0.02;
  parameters.contact_rate_mps = 0.03;
  EXPECT_THROW(
    {FinalDescentController invalid_controller(parameters);},
    std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
