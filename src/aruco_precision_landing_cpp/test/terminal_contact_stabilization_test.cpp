// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/terminal_contact_stabilization.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

Eigen::Vector3d upward_normal_from_roll_pitch(double roll_rad, double pitch_rad)
{
  const Eigen::AngleAxisd yaw_rotation(0.0, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd pitch_rotation(pitch_rad, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd roll_rotation(roll_rad, Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d body_to_ned =
    (yaw_rotation * pitch_rotation * roll_rotation).toRotationMatrix();
  return -body_to_ned.col(2);
}

TerminalDeckNormalInput active_input(const Eigen::Vector3d & normal)
{
  TerminalDeckNormalInput input;
  input.now_s = 1.0;
  input.dt_s = 0.05;
  input.phase = TerminalStabilizationPhase::kFinalDescent;
  input.production_enabled = true;
  input.normal_valid = true;
  input.normal_age_s = 0.01;
  input.upward_normal_ned = normal;
  return input;
}

TEST(TerminalDeckNormalMath, HorizontalNormalProducesZeroBias)
{
  const auto bias = TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    Eigen::Vector3d{0.0, 0.0, -1.0}, 9.80665);
  ASSERT_TRUE(bias.has_value());
  EXPECT_NEAR(bias->x(), 0.0, 1.0e-12);
  EXPECT_NEAR(bias->y(), 0.0, 1.0e-12);
}

TEST(TerminalDeckNormalMath, PositiveAndNegativeRollSignsAreMathematicallyCorrect)
{
  const auto positive = TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    upward_normal_from_roll_pitch(2.0 * kDeg, 0.0), 9.80665);
  const auto negative = TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    upward_normal_from_roll_pitch(-2.0 * kDeg, 0.0), 9.80665);
  ASSERT_TRUE(positive.has_value());
  ASSERT_TRUE(negative.has_value());
  EXPECT_GT(positive->y(), 0.0);
  EXPECT_LT(negative->y(), 0.0);
  EXPECT_NEAR(std::abs(positive->y()), 9.80665 * std::tan(2.0 * kDeg), 1.0e-6);
}

TEST(TerminalDeckNormalMath, PositivePitchSignIsCorrect)
{
  const auto bias = TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    upward_normal_from_roll_pitch(0.0, 2.0 * kDeg), 9.80665);
  ASSERT_TRUE(bias.has_value());
  EXPECT_LT(bias->x(), 0.0);
  EXPECT_NEAR(bias->x(), -9.80665 * std::tan(2.0 * kDeg), 1.0e-6);
}

TEST(TerminalDeckNormalMath, RejectsDownwardAndNonFiniteNormals)
{
  EXPECT_FALSE(TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    Eigen::Vector3d{0.0, 0.0, 1.0}, 9.80665).has_value());
  EXPECT_FALSE(TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, -1.0},
    9.80665).has_value());
}

TEST(TerminalDeckNormalMath, NormalizationDoesNotChangeBias)
{
  const Eigen::Vector3d normal = upward_normal_from_roll_pitch(2.0 * kDeg, 0.0);
  const auto unit_bias = TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    normal, 9.80665);
  const auto scaled_bias = TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
    4.0 * normal, 9.80665);
  ASSERT_TRUE(unit_bias.has_value());
  ASSERT_TRUE(scaled_bias.has_value());
  EXPECT_NEAR((*unit_bias - *scaled_bias).norm(), 0.0, 1.0e-12);
}

TEST(TerminalDeckNormalMath, YawPreservationWorksAtCardinalHeadings)
{
  const Eigen::Vector3d body_z = -upward_normal_from_roll_pitch(2.0 * kDeg, 0.0);
  for (const double yaw : {0.0, 90.0 * kDeg, 180.0 * kDeg, -90.0 * kDeg}) {
    const auto attitude = TerminalDeckNormalStabilizer::body_z_and_yaw_to_attitude(
      body_z, yaw);
    ASSERT_TRUE(attitude.has_value());
    const Eigen::Matrix3d rotation = attitude->toRotationMatrix();
    EXPECT_NEAR((rotation.col(2) - body_z.normalized()).norm(), 0.0, 1.0e-9);
    const double actual_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    EXPECT_NEAR(std::remainder(actual_yaw - yaw, 2.0 * kPi), 0.0, 2.0e-3);
  }
}

TEST(TerminalDeckNormalMath, AlignmentAxisProjectionRejectsCrossAxisNoise)
{
  const Eigen::Vector3d mixed = upward_normal_from_roll_pitch(2.0 * kDeg, 1.7 * kDeg);
  const auto roll_only = TerminalDeckNormalStabilizer::project_normal_to_alignment_axis(
    mixed, 0.0, TerminalAlignmentAxis::kRollOnly);
  const auto pitch_only = TerminalDeckNormalStabilizer::project_normal_to_alignment_axis(
    mixed, 0.0, TerminalAlignmentAxis::kPitchOnly);
  ASSERT_TRUE(roll_only.has_value());
  ASSERT_TRUE(pitch_only.has_value());
  const auto roll_attitude = TerminalDeckNormalStabilizer::body_z_and_yaw_to_attitude(
    -*roll_only, 0.0);
  const auto pitch_attitude = TerminalDeckNormalStabilizer::body_z_and_yaw_to_attitude(
    -*pitch_only, 0.0);
  ASSERT_TRUE(roll_attitude.has_value());
  ASSERT_TRUE(pitch_attitude.has_value());
  const auto roll_rpy = roll_attitude->toRotationMatrix().eulerAngles(0, 1, 2);
  const auto pitch_rpy = pitch_attitude->toRotationMatrix().eulerAngles(0, 1, 2);
  EXPECT_NEAR(roll_rpy.x(), 2.0 * kDeg, 1.0e-3);
  EXPECT_NEAR(roll_rpy.y(), 0.0, 1.0e-6);
  EXPECT_NEAR(pitch_rpy.x(), 0.0, 1.0e-6);
  EXPECT_NEAR(pitch_rpy.y(), 1.7 * kDeg, 1.0e-3);
}

TEST(TerminalDeckNormalMath, RejectsInvalidAttitudeInputs)
{
  EXPECT_FALSE(TerminalDeckNormalStabilizer::body_z_and_yaw_to_attitude(
    Eigen::Vector3d::Zero(), 0.0).has_value());
  EXPECT_FALSE(TerminalDeckNormalStabilizer::body_z_and_yaw_to_attitude(
    Eigen::Vector3d{0.0, 0.0, 1.0},
    std::numeric_limits<double>::infinity()).has_value());
}

TEST(TerminalDeckNormalState, FinalDescentWaitsForNearContactHeight)
{
  EXPECT_FALSE(TerminalDeckNormalStabilizer::phase_height_authorized(
    TerminalStabilizationPhase::kFinalDescent, 0.50, 0.25));
  EXPECT_TRUE(TerminalDeckNormalStabilizer::phase_height_authorized(
    TerminalStabilizationPhase::kFinalDescent, 0.25, 0.25));
  EXPECT_TRUE(TerminalDeckNormalStabilizer::phase_height_authorized(
    TerminalStabilizationPhase::kTouchdownCandidateHold,
    std::numeric_limits<double>::quiet_NaN(), 0.25));
  EXPECT_TRUE(TerminalDeckNormalStabilizer::phase_height_authorized(
    TerminalStabilizationPhase::kTouchdownCandidateHold,
    std::numeric_limits<double>::quiet_NaN(), 0.25, true));
  EXPECT_TRUE(TerminalDeckNormalStabilizer::phase_height_authorized(
    TerminalStabilizationPhase::kTouchdownHold,
    std::numeric_limits<double>::quiet_NaN(), 0.25));
  EXPECT_FALSE(TerminalDeckNormalStabilizer::phase_height_authorized(
    TerminalStabilizationPhase::kInactive, 0.10, 0.25));
  EXPECT_FALSE(TerminalDeckNormalStabilizer::phase_height_authorized(
    TerminalStabilizationPhase::kFinalDescent,
    std::numeric_limits<double>::quiet_NaN(), 0.25));
}

TEST(TerminalDeckNormalState, FirstEnableIsContinuousAndRamps)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  auto output = stabilizer.update(input);
  EXPECT_TRUE(output.enabled);
  EXPECT_LT(output.acceleration_bias_ned_mps2.norm(), 0.05);
  const double first_norm = output.acceleration_bias_ned_mps2.norm();
  for (int index = 0; index < 10; ++index) {
    input.now_s += input.dt_s;
    output = stabilizer.update(input);
  }
  EXPECT_GT(output.acceleration_bias_ned_mps2.norm(), first_norm);
}

TEST(TerminalDeckNormalState, InactiveProductionStatesRemainStrictlyZero)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  for (const auto phase : {
      TerminalStabilizationPhase::kInactive,
      TerminalStabilizationPhase::kRecovery})
  {
    auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
    input.phase = phase;
    const auto output = stabilizer.update(input);
    EXPECT_FALSE(output.enabled);
    EXPECT_EQ(output.acceleration_bias_ned_mps2, Eigen::Vector2d::Zero());
  }
}

TEST(TerminalDeckNormalState, RehearsalRequiresSeparateAuthorization)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  input.phase = TerminalStabilizationPhase::kRehearsal;
  input.production_enabled = false;
  input.rehearsal_enabled = false;
  EXPECT_FALSE(stabilizer.update(input).enabled);
  input.rehearsal_enabled = true;
  input.now_s += 0.05;
  EXPECT_TRUE(stabilizer.update(input).enabled);
}

TEST(TerminalDeckNormalState, BiasAndTiltAreLimited)
{
  TerminalDeckNormalParameters parameters;
  parameters.maximum_target_tilt_rad = 2.5 * kDeg;
  parameters.acceleration_bias_limit_mps2 = 0.40;
  parameters.activation_duration_s = 0.01;
  parameters.acceleration_bias_slew_rate_mps3 = 100.0;
  TerminalDeckNormalStabilizer stabilizer(parameters);
  auto input = active_input(upward_normal_from_roll_pitch(15.0 * kDeg, 0.0));
  input.dt_s = 0.02;
  const auto output = stabilizer.update(input);
  EXPECT_LE(output.acceleration_bias_ned_mps2.norm(), 0.40 + 1.0e-12);
  EXPECT_LE(output.desired_roll_pitch_rad.cwiseAbs().maxCoeff(), 2.5 * kDeg + 1.0e-4);
}

TEST(TerminalDeckNormalState, BiasSlewRateIsLimited)
{
  TerminalDeckNormalParameters parameters;
  parameters.activation_duration_s = 0.01;
  parameters.acceleration_bias_slew_rate_mps3 = 0.20;
  TerminalDeckNormalStabilizer stabilizer(parameters);
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  input.dt_s = 0.05;
  const auto output = stabilizer.update(input);
  EXPECT_LE(output.acceleration_bias_ned_mps2.norm(), 0.20 * 0.05 + 1.0e-12);
}

TEST(TerminalDeckNormalState, UnauthorizedReturnUsesTiltSlewLimit)
{
  TerminalDeckNormalParameters parameters;
  parameters.activation_duration_s = 0.01;
  parameters.deactivation_duration_s = 0.01;
  parameters.tilt_slew_rate_radps = 4.0 * kDeg;
  parameters.acceleration_bias_slew_rate_mps3 = 100.0;
  TerminalDeckNormalStabilizer stabilizer(parameters);
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  input.phase = TerminalStabilizationPhase::kRehearsal;
  input.production_enabled = false;
  input.rehearsal_enabled = true;
  input.dt_s = 0.05;
  for (int index = 0; index < 20; ++index) {
    input.now_s += input.dt_s;
    stabilizer.update(input);
  }
  const auto before = stabilizer.update(input);
  input.now_s += input.dt_s;
  input.rehearsal_enabled = false;
  const auto after = stabilizer.update(input);
  const double before_tilt = std::acos(std::clamp(
    -before.desired_upward_normal_ned.z(), -1.0, 1.0));
  const double after_tilt = std::acos(std::clamp(
    -after.desired_upward_normal_ned.z(), -1.0, 1.0));
  EXPECT_LE(before_tilt - after_tilt, parameters.tilt_slew_rate_radps * input.dt_s + 1.0e-9);
}

TEST(TerminalDeckNormalState, MarkerSwitchJumpFallsBackWithoutCommandJump)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  for (int index = 0; index < 8; ++index) {
    input.now_s += input.dt_s;
    stabilizer.update(input);
  }
  const auto before = stabilizer.update(input);
  input.now_s += input.dt_s;
  input.marker_switched = true;
  input.marker_switch_jump_rad = 3.0 * kDeg;
  const auto after = stabilizer.update(input);
  EXPECT_TRUE(after.fallback_active);
  EXPECT_LT(
    (after.acceleration_bias_ned_mps2 - before.acceleration_bias_ned_mps2).norm(),
    0.05);
}

TEST(TerminalDeckNormalState, ShortLossHoldsThenSmoothlyReturnsToZero)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  for (int index = 0; index < 12; ++index) {
    input.now_s += input.dt_s;
    stabilizer.update(input);
  }
  input.normal_valid = false;
  input.normal_age_s = 0.05;
  input.now_s += input.dt_s;
  const auto short_loss = stabilizer.update(input);
  EXPECT_GT(short_loss.acceleration_bias_ned_mps2.norm(), 0.0);
  input.normal_age_s = 0.40;
  double previous = short_loss.acceleration_bias_ned_mps2.norm();
  for (int index = 0; index < 12; ++index) {
    input.now_s += input.dt_s;
    const auto fallback = stabilizer.update(input);
    EXPECT_LE(fallback.acceleration_bias_ned_mps2.norm(), previous + 1.0e-12);
    previous = fallback.acceleration_bias_ned_mps2.norm();
  }
  EXPECT_NEAR(previous, 0.0, 1.0e-9);
}

TEST(TerminalDeckNormalState, TouchdownHoldLatchesLastValidNormalWithoutFallback)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.latch_last_valid_normal = true;
  for (int index = 0; index < 20; ++index) {
    input.now_s += input.dt_s;
    stabilizer.update(input);
  }
  input.upward_normal_ned = upward_normal_from_roll_pitch(0.2 * kDeg, 0.0);
  input.normal_valid = true;
  input.normal_age_s = 0.01;
  input.now_s += input.dt_s;
  const auto latched = stabilizer.update(input);
  EXPECT_EQ(latched.mode, "ACTIVE");
  EXPECT_EQ(latched.reason, "touchdown_hold_latched_normal");
  EXPECT_TRUE(latched.valid);
  EXPECT_FALSE(latched.fallback_active);
  EXPECT_GT(latched.acceleration_bias_ned_mps2.norm(), 0.30);
}

TEST(TerminalDeckNormalState, TimeRegressionAndAbnormalDtResetHistory)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  stabilizer.update(input);
  input.now_s = 0.5;
  const auto regressed = stabilizer.update(input);
  EXPECT_EQ(regressed.acceleration_bias_ned_mps2, Eigen::Vector2d::Zero());
  input.now_s = 2.0;
  input.dt_s = 0.5;
  const auto abnormal = stabilizer.update(input);
  EXPECT_EQ(abnormal.acceleration_bias_ned_mps2, Eigen::Vector2d::Zero());
}

TEST(TerminalDeckNormalState, ResetClearsHistory)
{
  TerminalDeckNormalStabilizer stabilizer(TerminalDeckNormalParameters{});
  auto input = active_input(upward_normal_from_roll_pitch(2.0 * kDeg, 0.0));
  for (int index = 0; index < 10; ++index) {
    input.now_s += input.dt_s;
    stabilizer.update(input);
  }
  stabilizer.reset();
  input.now_s += input.dt_s;
  const auto output = stabilizer.update(input);
  EXPECT_LT(output.acceleration_bias_ned_mps2.norm(), 0.05);
}

TEST(TerminalContactCompliance, CandidateEntryHasNoTargetJump)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  input.enabled = true;
  input.deck_state_valid = true;
  input.nominal_target_xy_m = {1.0, 2.0};
  input.deck_position_xy_m = {0.9, 1.9};
  input.uav_position_xy_m = {1.0, 2.0};
  const auto output = controller.update(input);
  EXPECT_TRUE(output.active);
  EXPECT_EQ(output.compliant_target_xy_m, input.nominal_target_xy_m);
}

TEST(TerminalContactCompliance, DisabledFeatureDoesNotChangeTarget)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = false;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kFinalDescent;
  input.nominal_target_xy_m = {1.0, 2.0};
  const auto output = controller.update(input);
  EXPECT_FALSE(output.active);
  EXPECT_EQ(output.compliant_target_xy_m, input.nominal_target_xy_m);
}

TEST(TerminalContactCompliance, NearContactFinalDescentEntryHasNoTargetJump)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kFinalDescent;
  input.nominal_target_xy_m = {1.0, 2.0};
  input.deck_position_xy_m = {0.9, 1.9};
  input.uav_position_xy_m = {1.0, 2.0};
  const auto output = controller.update(input);
  EXPECT_TRUE(output.active);
  EXPECT_EQ(output.compliant_target_xy_m, input.nominal_target_xy_m);
}

TEST(TerminalContactCompliance, MovingDeckAnchorFollowsDeckVelocity)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  input.nominal_target_xy_m = {0.0, 0.0};
  controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.deck_position_xy_m = {0.01, 0.0};
  input.deck_velocity_xy_mps = {0.2, 0.0};
  input.uav_position_xy_m = {0.0, 0.0};
  const auto output = controller.update(input);
  EXPECT_GT(output.contact_anchor_xy_m.x(), 0.0);
  EXPECT_GT(output.compliant_target_xy_m.x(), 0.0);
}

TEST(TerminalContactCompliance, StaticDeckVelocityNoiseInsideDeadbandDoesNotMoveAnchor)
{
  TerminalContactComplianceParameters parameters;
  parameters.deck_velocity_deadband_mps = 0.035;
  TerminalContactComplianceController controller(parameters);
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  input.nominal_target_xy_m = {1.0, 2.0};
  input.deck_position_xy_m = {1.0, 2.0};
  controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  for (int index = 0; index < 20; ++index) {
    input.deck_velocity_xy_mps = {
      index % 2 == 0 ? 0.030 : -0.030,
      index % 2 == 0 ? -0.010 : 0.010};
    const auto output = controller.update(input);
    EXPECT_NEAR(output.contact_anchor_xy_m.x(), 1.0, 1.0e-12);
    EXPECT_NEAR(output.contact_anchor_xy_m.y(), 2.0, 1.0e-12);
  }
}

TEST(TerminalContactCompliance, StaticDeckPositionNoiseInsideDeadbandDoesNotMoveAnchor)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  input.nominal_target_xy_m = {1.0, 2.0};
  input.deck_position_xy_m = {1.0, 2.0};
  controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  for (int index = 0; index < 20; ++index) {
    input.deck_position_xy_m = {
      1.0 + (index % 2 == 0 ? 0.010 : -0.010),
      2.0 + (index % 2 == 0 ? -0.010 : 0.010)};
    const auto output = controller.update(input);
    EXPECT_NEAR(output.contact_anchor_xy_m.x(), 1.0, 1.0e-12);
    EXPECT_NEAR(output.contact_anchor_xy_m.y(), 2.0, 1.0e-12);
  }
}

TEST(TerminalContactCompliance, AnchorPositionCorrectionIsRateLimited)
{
  TerminalContactComplianceParameters parameters;
  parameters.horizontal_deadband_m = 0.0;
  parameters.maximum_anchor_correction_rate_mps = 0.02;
  TerminalContactComplianceController controller(parameters);
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.deck_position_xy_m = {0.10, 0.0};
  const auto output = controller.update(input);
  EXPECT_LE(output.contact_anchor_xy_m.norm(), 0.02 * 0.05 + 1.0e-12);
}

TEST(TerminalContactCompliance, DeadbandHoldsLastSafeTargetInsteadOfFollowingPositionNoise)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  const auto initial = controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.uav_position_xy_m = {0.010, 0.0};
  const auto output = controller.update(input);
  EXPECT_EQ(output.compliant_target_xy_m, initial.compliant_target_xy_m);
}

TEST(TerminalContactCompliance, LargeUavOffsetDoesNotDragAnchorCenteredTarget)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  controller.update(input);
  input.uav_position_xy_m = {0.20, 0.0};
  input.uav_velocity_xy_mps = {0.0, 0.0};
  const auto output = controller.update(input);
  EXPECT_NEAR(output.contact_anchor_xy_m.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(output.compliant_target_xy_m.x(), 0.0, 1.0e-12);
}

TEST(TerminalContactCompliance, RecoveryIsLimitedAndTargetRateIsBounded)
{
  TerminalContactComplianceParameters parameters;
  parameters.maximum_target_rate_mps = 0.10;
  TerminalContactComplianceController controller(parameters);
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.uav_position_xy_m = {0.20, 0.0};
  const auto output = controller.update(input);
  EXPECT_LE(output.compliant_target_xy_m.norm(), 0.10 * 0.05 + 1.0e-12);
}

TEST(TerminalContactCompliance, InvalidDeckStateHoldsLastSafeTarget)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  const auto initial = controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.deck_state_valid = false;
  input.nominal_target_xy_m = {10.0, 10.0};
  const auto invalid = controller.update(input);
  EXPECT_EQ(invalid.compliant_target_xy_m, initial.compliant_target_xy_m);
}

TEST(TerminalContactCompliance, RelativeVelocityDampingOpposesSlip)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.uav_velocity_xy_mps = {0.1, 0.0};
  const auto output = controller.update(input);
  EXPECT_LT(output.compliant_target_xy_m.x(), 0.0);
}

TEST(TerminalContactCompliance, VelocityDampingDoesNotIntegrateIntoPositionTarget)
{
  TerminalContactComplianceParameters parameters;
  parameters.relative_velocity_damping_s = 0.12;
  parameters.maximum_damping_offset_m = 0.020;
  TerminalContactComplianceController controller(parameters);
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  controller.update(input);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.uav_velocity_xy_mps = {0.10, 0.0};
  TerminalContactComplianceOutput output;
  for (int index = 0; index < 20; ++index) {
    output = controller.update(input);
  }
  EXPECT_LT(output.compliant_target_xy_m.x(), 0.0);
  EXPECT_GE(
    output.compliant_target_xy_m.x(),
    -parameters.maximum_damping_offset_m - 1.0e-12);
}

TEST(TerminalContactCompliance, ResetClearsAnchor)
{
  TerminalContactComplianceController controller(TerminalContactComplianceParameters{});
  TerminalContactComplianceInput input;
  input.dt_s = 0.05;
  input.enabled = true;
  input.deck_state_valid = true;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  controller.update(input);
  ASSERT_TRUE(controller.initialized());
  controller.reset();
  EXPECT_FALSE(controller.initialized());
}

TEST(TerminalAttitudeSafety, NormalTwoDegreeAttitudeDoesNotTrigger)
{
  TerminalAttitudeSafetyMonitor monitor(TerminalAttitudeSafetyParameters{});
  TerminalAttitudeSafetyInput input;
  input.dt_s = 0.05;
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.enabled = true;
  input.attitude_valid = true;
  input.roll_rad = 2.0 * kDeg;
  for (int index = 0; index < 20; ++index) {
    EXPECT_FALSE(monitor.update(input).recovery_requested);
  }
}

TEST(TerminalAttitudeSafety, ShortNoiseDoesNotTriggerButSustainedErrorDoes)
{
  TerminalAttitudeSafetyMonitor monitor(TerminalAttitudeSafetyParameters{});
  TerminalAttitudeSafetyInput input;
  input.dt_s = 0.05;
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.enabled = true;
  input.attitude_valid = true;
  input.roll_rad = 7.0 * kDeg;
  EXPECT_FALSE(monitor.update(input).recovery_requested);
  EXPECT_FALSE(monitor.update(input).recovery_requested);
  EXPECT_FALSE(monitor.update(input).recovery_requested);
  EXPECT_TRUE(monitor.update(input).recovery_requested);
}

TEST(TerminalAttitudeSafety, ExcessAngularRateTriggersAfterDuration)
{
  TerminalAttitudeSafetyMonitor monitor(TerminalAttitudeSafetyParameters{});
  TerminalAttitudeSafetyInput input;
  input.dt_s = 0.05;
  input.phase = TerminalStabilizationPhase::kTouchdownCandidateHold;
  input.enabled = true;
  input.attitude_valid = true;
  input.angular_velocity_body_radps = {1.0, 0.0, 0.0};
  TerminalAttitudeSafetyOutput output;
  for (int index = 0; index < 4; ++index) {
    output = monitor.update(input);
  }
  EXPECT_TRUE(output.recovery_requested);
}

TEST(TerminalAttitudeSafety, InactiveAndInvalidInputsCannotTrigger)
{
  TerminalAttitudeSafetyMonitor monitor(TerminalAttitudeSafetyParameters{});
  TerminalAttitudeSafetyInput input;
  input.dt_s = 0.05;
  input.phase = TerminalStabilizationPhase::kFinalDescent;
  input.enabled = true;
  input.attitude_valid = true;
  input.roll_rad = 20.0 * kDeg;
  EXPECT_FALSE(monitor.update(input).recovery_requested);
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.attitude_valid = false;
  EXPECT_FALSE(monitor.update(input).recovery_requested);
}

TEST(TerminalAttitudeSafety, ResetClearsLatchedRecovery)
{
  TerminalAttitudeSafetyMonitor monitor(TerminalAttitudeSafetyParameters{});
  TerminalAttitudeSafetyInput input;
  input.dt_s = 0.05;
  input.phase = TerminalStabilizationPhase::kTouchdownHold;
  input.enabled = true;
  input.attitude_valid = true;
  input.roll_rad = 7.0 * kDeg;
  for (int index = 0; index < 4; ++index) {
    monitor.update(input);
  }
  ASSERT_TRUE(monitor.update(input).recovery_requested);
  monitor.reset();
  input.roll_rad = 0.0;
  EXPECT_FALSE(monitor.update(input).recovery_requested);
}

TEST(TerminalContactStabilizationParameters, RejectsUnsafeLimits)
{
  TerminalDeckNormalParameters normal_parameters;
  normal_parameters.maximum_target_tilt_rad = 0.0;
  EXPECT_THROW(
    {TerminalDeckNormalStabilizer candidate(normal_parameters);},
    std::invalid_argument);

  TerminalContactComplianceParameters compliance_parameters;
  compliance_parameters.maximum_allowance_m = 0.0;
  EXPECT_THROW(
    {TerminalContactComplianceController candidate(compliance_parameters);},
    std::invalid_argument);

  TerminalAttitudeSafetyParameters safety_parameters;
  safety_parameters.attitude_trigger_rad = 10.0 * kDeg;
  EXPECT_THROW(
    {TerminalAttitudeSafetyMonitor candidate(safety_parameters);},
    std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
