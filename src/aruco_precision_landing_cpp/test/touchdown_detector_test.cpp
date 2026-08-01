// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/touchdown_detector.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

TouchdownDetectorParameters test_parameters()
{
  TouchdownDetectorParameters parameters;
  parameters.px4_status_timeout_s = 0.20;
  parameters.visual_timeout_s = 0.20;
  parameters.low_height_enter_m = 0.18;
  parameters.low_height_exit_m = 0.28;
  parameters.max_relative_vertical_speed_mps = 0.12;
  parameters.max_uav_vertical_speed_mps = 0.15;
  parameters.max_relative_horizontal_speed_mps = 0.15;
  parameters.terminal_contact_max_height_m = 0.24;
  parameters.terminal_contact_min_reference_error_m = 0.10;
  parameters.terminal_contact_max_vertical_speed_mps = 0.05;
  parameters.candidate_required_duration_s = 0.30;
  return parameters;
}

TouchdownDetectorInput base_input(double time_s)
{
  TouchdownDetectorInput input;
  input.sample_time_s = time_s;
  input.state_allows_touchdown_detection = true;
  input.px4_land_status_valid = true;
  input.px4_land_status_age_s = 0.02;
  input.visual_height_valid = true;
  input.visual_height_age_s = 0.02;
  input.relative_height_m = 0.10;
  input.relative_vertical_speed_valid = true;
  input.relative_vertical_velocity_mps = 0.01;
  input.uav_vertical_velocity_mps = 0.01;
  input.relative_horizontal_speed_valid = true;
  input.relative_horizontal_speed_mps = 0.01;
  input.close_to_ground = true;
  return input;
}

TouchdownDetectorOutput advance_normal_candidate(
  TouchdownDetector & detector,
  double end_time_s)
{
  TouchdownDetectorOutput output;
  for (int index = 0; index <= static_cast<int>(end_time_s * 10.0 + 0.5); ++index) {
    auto input = base_input(index * 0.1);
    input.ground_contact = true;
    output = detector.update(input);
  }
  return output;
}

TEST(TouchdownDetectorTest, VisualLowHeightAloneCannotConfirm)
{
  TouchdownDetector detector(test_parameters());

  TouchdownDetectorOutput output;
  for (int index = 0; index <= 10; ++index) {
    output = detector.update(base_input(index * 0.1));
  }

  EXPECT_EQ(output.status, TouchdownStatus::kAirborne);
  EXPECT_FALSE(output.confirmed_latched);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
  EXPECT_NE(
    output.evidence_mask & touchdown_evidence_mask(TouchdownEvidence::kVisualLowHeight),
    0U);
}

TEST(TouchdownDetectorTest, LandedAndAtRestConfirmWithoutVisualHeight)
{
  TouchdownDetector detector(test_parameters());

  TouchdownDetectorOutput output;
  for (int index = 0; index <= 3; ++index) {
    auto input = base_input(index * 0.1);
    input.visual_height_valid = false;
    input.landed = true;
    input.at_rest = true;
    output = detector.update(input);
  }

  EXPECT_EQ(output.status, TouchdownStatus::kConfirmed);
  EXPECT_TRUE(output.confirmed_latched);
}

TEST(TouchdownDetectorTest, ContactLowHeightAndLowVelocityConfirmAfterDuration)
{
  TouchdownDetector detector(test_parameters());

  const auto output = advance_normal_candidate(detector, 0.3);

  EXPECT_EQ(output.status, TouchdownStatus::kConfirmed);
  EXPECT_TRUE(output.confirmed_latched);
  EXPECT_GE(output.candidate_duration_s, 0.30 - 1.0e-12);
}

TEST(TouchdownDetectorTest, TerminalContactStallConfirmsWithoutPx4ContactFlag)
{
  TouchdownDetector detector(test_parameters());

  TouchdownDetectorOutput output;
  for (int index = 0; index <= 3; ++index) {
    auto input = base_input(index * 0.1);
    input.relative_height_m = 0.22;
    input.relative_height_reference_m = 0.05;
    input.terminal_descent_active = true;
    input.terminal_command_complete = true;
    input.relative_vertical_velocity_mps = 0.01;
    input.uav_vertical_velocity_mps = 0.01;
    output = detector.update(input);
  }

  EXPECT_EQ(output.status, TouchdownStatus::kConfirmed);
  EXPECT_TRUE(output.confirmed_latched);
  EXPECT_NE(
    output.evidence_mask &
    touchdown_evidence_mask(TouchdownEvidence::kTerminalContactStall),
    0U);
}

TEST(TouchdownDetectorTest, TerminalContactWaitsForMinimumCommand)
{
  TouchdownDetector detector(test_parameters());

  TouchdownDetectorOutput output;
  for (int index = 0; index <= 5; ++index) {
    auto input = base_input(index * 0.1);
    input.relative_height_m = 0.22;
    input.relative_height_reference_m = 0.12;
    input.terminal_descent_active = true;
    input.terminal_command_complete = false;
    input.relative_vertical_velocity_mps = 0.01;
    input.uav_vertical_velocity_mps = 0.01;
    output = detector.update(input);
  }

  EXPECT_EQ(output.status, TouchdownStatus::kAirborne);
  EXPECT_FALSE(output.confirmed_latched);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
  EXPECT_EQ(
    output.evidence_mask &
    touchdown_evidence_mask(TouchdownEvidence::kTerminalContactStall),
    0U);
}

TEST(TouchdownDetectorTest, TerminalContactNeedsReferencePenetrationAndVerticalStall)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.relative_height_m = 0.22;
  input.relative_height_reference_m = 0.17;
  input.terminal_descent_active = true;
  input.terminal_command_complete = true;
  EXPECT_EQ(detector.update(input).status, TouchdownStatus::kAirborne);

  input.sample_time_s = 0.1;
  input.relative_height_reference_m = 0.05;
  input.relative_vertical_velocity_mps = 0.08;
  input.uav_vertical_velocity_mps = 0.08;
  const auto moving = detector.update(input);
  EXPECT_EQ(moving.status, TouchdownStatus::kAirborne);
  EXPECT_EQ(
    moving.evidence_mask &
    touchdown_evidence_mask(TouchdownEvidence::kTerminalContactStall),
    0U);
}

TEST(TouchdownDetectorTest, TerminalContactDoesNotBypassVisualOrMotionSafety)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.relative_height_m = 0.22;
  input.relative_height_reference_m = 0.05;
  input.terminal_descent_active = true;
  input.terminal_command_complete = true;
  input.visual_height_valid = false;
  EXPECT_EQ(detector.update(input).status, TouchdownStatus::kInsufficientEvidence);

  input = base_input(0.1);
  input.relative_height_m = 0.22;
  input.relative_height_reference_m = 0.05;
  input.terminal_descent_active = true;
  input.terminal_command_complete = true;
  input.rotational_movement = true;
  EXPECT_EQ(detector.update(input).status, TouchdownStatus::kAirborne);
}

TEST(TouchdownDetectorTest, SingleContactFrameDoesNotConfirm)
{
  TouchdownDetector detector(test_parameters());
  auto contact = base_input(0.0);
  contact.ground_contact = true;

  const auto candidate = detector.update(contact);
  const auto airborne = detector.update(base_input(0.1));

  EXPECT_EQ(candidate.status, TouchdownStatus::kCandidate);
  EXPECT_FALSE(candidate.confirmed_latched);
  EXPECT_EQ(airborne.status, TouchdownStatus::kAirborne);
  EXPECT_DOUBLE_EQ(airborne.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, HighVerticalSpeedClearsCandidate)
{
  TouchdownDetector detector(test_parameters());
  auto first = base_input(0.0);
  first.ground_contact = true;
  auto second = base_input(0.1);
  second.ground_contact = true;
  ASSERT_EQ(detector.update(first).status, TouchdownStatus::kCandidate);
  ASSERT_EQ(detector.update(second).status, TouchdownStatus::kCandidate);

  auto fast = base_input(0.2);
  fast.ground_contact = true;
  fast.relative_vertical_velocity_mps = 0.40;
  const auto output = detector.update(fast);

  EXPECT_EQ(output.status, TouchdownStatus::kAirborne);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, HighRelativeHorizontalOrRotationalMovementRejectsCandidate)
{
  TouchdownDetector detector(test_parameters());
  auto moving = base_input(0.0);
  moving.ground_contact = true;
  moving.horizontal_movement = true;
  moving.relative_horizontal_speed_mps = 0.40;
  EXPECT_EQ(detector.update(moving).status, TouchdownStatus::kAirborne);

  moving.sample_time_s = 0.1;
  moving.horizontal_movement = false;
  moving.relative_horizontal_speed_mps = 0.01;
  moving.rotational_movement = true;
  EXPECT_EQ(detector.update(moving).status, TouchdownStatus::kAirborne);
}

TEST(TouchdownDetectorTest, MovingDeckCanConfirmAtLowRelativeHorizontalSpeed)
{
  TouchdownDetector detector(test_parameters());

  TouchdownDetectorOutput output;
  for (int index = 0; index <= 3; ++index) {
    auto input = base_input(index * 0.1);
    input.ground_contact = true;
    input.horizontal_movement = true;
    input.relative_horizontal_speed_mps = 0.04;
    output = detector.update(input);
  }

  EXPECT_EQ(output.status, TouchdownStatus::kConfirmed);
  EXPECT_TRUE(output.confirmed_latched);
  EXPECT_NE(
    output.evidence_mask &
    touchdown_evidence_mask(TouchdownEvidence::kLowRelativeHorizontalSpeed),
    0U);
  EXPECT_EQ(
    output.evidence_mask & touchdown_evidence_mask(TouchdownEvidence::kNoReportedMovement),
    0U);
}

TEST(TouchdownDetectorTest, MovingDeckNeedsRelativeHorizontalSpeedEstimate)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.ground_contact = true;
  input.horizontal_movement = true;
  input.relative_horizontal_speed_valid = false;

  const auto output = detector.update(input);

  EXPECT_EQ(output.status, TouchdownStatus::kInsufficientEvidence);
  EXPECT_FALSE(output.confirmed_latched);
}

TEST(TouchdownDetectorTest, HeavingDeckCanConfirmAtLowRelativeVerticalSpeed)
{
  TouchdownDetector detector(test_parameters());

  TouchdownDetectorOutput output;
  for (int index = 0; index <= 3; ++index) {
    auto input = base_input(index * 0.1);
    input.ground_contact = true;
    input.vertical_movement = true;
    input.uav_vertical_velocity_mps = 0.18;
    input.relative_vertical_velocity_mps = 0.02;
    output = detector.update(input);
  }

  EXPECT_EQ(output.status, TouchdownStatus::kConfirmed);
  EXPECT_TRUE(output.confirmed_latched);
  EXPECT_NE(
    output.evidence_mask &
    touchdown_evidence_mask(TouchdownEvidence::kLowRelativeVerticalSpeed),
    0U);
  EXPECT_EQ(
    output.evidence_mask & touchdown_evidence_mask(TouchdownEvidence::kLowUavVerticalSpeed),
    0U);
}

TEST(TouchdownDetectorTest, LowWorldSpeedCannotBypassHighRelativeVerticalSpeed)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.ground_contact = true;
  input.uav_vertical_velocity_mps = 0.01;
  input.relative_vertical_velocity_mps = 0.20;

  const auto output = detector.update(input);

  EXPECT_EQ(output.status, TouchdownStatus::kAirborne);
  EXPECT_FALSE(output.confirmed_latched);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, InvalidRelativeVerticalSpeedCannotUseStrongPx4Evidence)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.visual_height_valid = false;
  input.landed = true;
  input.at_rest = true;
  input.relative_vertical_speed_valid = false;
  input.relative_vertical_velocity_mps = 0.0;

  const auto output = detector.update(input);

  EXPECT_EQ(output.status, TouchdownStatus::kInsufficientEvidence);
  EXPECT_FALSE(output.confirmed_latched);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, FreefallIsRejectedUnsafe)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.freefall = true;
  input.ground_contact = true;

  const auto output = detector.update(input);

  EXPECT_EQ(output.status, TouchdownStatus::kRejectedUnsafe);
  EXPECT_FALSE(output.confirmed_latched);
}

TEST(TouchdownDetectorTest, StalePx4StatusCannotAccumulateCandidate)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.ground_contact = true;
  input.px4_land_status_age_s = 0.25;

  const auto output = detector.update(input);

  EXPECT_EQ(output.status, TouchdownStatus::kInsufficientEvidence);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, StaleVisualHeightCannotConfirmOrdinaryContact)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.ground_contact = true;
  input.visual_height_age_s = 0.25;

  const auto output = detector.update(input);

  EXPECT_EQ(output.status, TouchdownStatus::kInsufficientEvidence);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, HeightHysteresisRetainsLowHeightUntilExitThreshold)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.ground_contact = true;
  ASSERT_EQ(detector.update(input).status, TouchdownStatus::kCandidate);

  input.sample_time_s = 0.1;
  input.relative_height_m = 0.24;
  const auto retained = detector.update(input);
  EXPECT_EQ(retained.status, TouchdownStatus::kCandidate);
  EXPECT_NE(
    retained.evidence_mask & touchdown_evidence_mask(TouchdownEvidence::kVisualLowHeight),
    0U);

  input.sample_time_s = 0.2;
  input.relative_height_m = 0.30;
  const auto exited = detector.update(input);
  EXPECT_EQ(exited.status, TouchdownStatus::kAirborne);
  EXPECT_DOUBLE_EQ(exited.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, DetectionDisabledStateCannotAccumulateCandidate)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.ground_contact = true;
  input.state_allows_touchdown_detection = false;

  const auto output = detector.update(input);

  EXPECT_EQ(output.status, TouchdownStatus::kAirborne);
  EXPECT_DOUBLE_EQ(output.candidate_duration_s, 0.0);
}

TEST(TouchdownDetectorTest, RepeatedAndBackwardTimeRejectAndResetCandidate)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(1.0);
  input.ground_contact = true;
  ASSERT_EQ(detector.update(input).status, TouchdownStatus::kCandidate);

  input.sample_time_s = 1.0;
  const auto repeated = detector.update(input);
  EXPECT_EQ(repeated.status, TouchdownStatus::kRejectedUnsafe);
  EXPECT_DOUBLE_EQ(repeated.candidate_duration_s, 0.0);

  input.sample_time_s = 0.5;
  const auto backward = detector.update(input);
  EXPECT_EQ(backward.status, TouchdownStatus::kRejectedUnsafe);
  EXPECT_DOUBLE_EQ(backward.candidate_duration_s, 0.0);

  input.sample_time_s = 0.6;
  EXPECT_EQ(detector.update(input).status, TouchdownStatus::kCandidate);
}

TEST(TouchdownDetectorTest, ConfirmationRemainsLatchedUntilReset)
{
  TouchdownDetector detector(test_parameters());
  ASSERT_EQ(
    advance_normal_candidate(detector, 0.3).status,
    TouchdownStatus::kConfirmed);

  auto unsafe = base_input(0.4);
  unsafe.freefall = true;
  unsafe.ground_contact = false;
  const auto latched = detector.update(unsafe);

  EXPECT_EQ(latched.status, TouchdownStatus::kConfirmed);
  EXPECT_TRUE(latched.confirmed_latched);

  detector.reset();
  const auto reset_output = detector.update(base_input(1.0));
  EXPECT_EQ(reset_output.status, TouchdownStatus::kAirborne);
  EXPECT_FALSE(reset_output.confirmed_latched);
}

TEST(TouchdownDetectorTest, RejectsInvalidInputAndParameters)
{
  TouchdownDetector detector(test_parameters());
  auto input = base_input(0.0);
  input.sample_time_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(detector.update(input).status, TouchdownStatus::kRejectedUnsafe);

  auto parameters = test_parameters();
  parameters.px4_status_timeout_s = 0.0;
  EXPECT_THROW(
    {TouchdownDetector invalid_detector(parameters);},
    std::invalid_argument);

  parameters = test_parameters();
  parameters.low_height_exit_m = parameters.low_height_enter_m;
  EXPECT_THROW(
    {TouchdownDetector invalid_detector(parameters);},
    std::invalid_argument);

  parameters = test_parameters();
  parameters.max_relative_horizontal_speed_mps = 0.0;
  EXPECT_THROW(
    {TouchdownDetector invalid_detector(parameters);},
    std::invalid_argument);

  parameters = test_parameters();
  parameters.terminal_contact_min_reference_error_m = 0.0;
  EXPECT_THROW(
    {TouchdownDetector invalid_detector(parameters);},
    std::invalid_argument);

  parameters = test_parameters();
  parameters.terminal_contact_max_vertical_speed_mps = 0.0;
  EXPECT_THROW(
    {TouchdownDetector invalid_detector(parameters);},
    std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
