// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/moving_target_tracking_controller.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

TargetStateEstimate make_estimate(
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & velocity)
{
  TargetStateEstimate estimate;
  estimate.position_ned = position;
  estimate.velocity_ned = velocity;
  estimate.covariance.setIdentity();
  estimate.sample_time_s = 10.0;
  return estimate;
}

MovingTargetTrackingInput make_input()
{
  MovingTargetTrackingInput input;
  input.current_target_xy = Eigen::Vector2d::Zero();
  input.raw_visual_position_xy = Eigen::Vector2d{1.0, 2.0};
  input.estimated_state = make_estimate(
    Eigen::Vector3d{1.0, 2.0, 0.2},
    Eigen::Vector3d{0.4, -0.2, 0.0});
  input.predicted_position_xy = Eigen::Vector2d{1.2, 1.9};
  input.uav_position_xy = Eigen::Vector2d::Zero();
  input.uav_velocity_xy = Eigen::Vector2d::Zero();
  input.visual_fresh = true;
  input.estimate_age_s = 0.0;
  input.dt_s = 1.0;
  return input;
}

MovingTargetTrackingParameters permissive_parameters(TrackingControlMode mode)
{
  MovingTargetTrackingParameters parameters;
  parameters.mode = mode;
  parameters.max_position_target_speed_mps = 100.0;
  parameters.max_position_target_step_m = 100.0;
  parameters.velocity_feedforward_gain = 1.0;
  parameters.relative_velocity_gain = 0.0;
  parameters.max_velocity_feedforward_mps = 100.0;
  parameters.max_velocity_feedforward_acceleration_mps2 = 100.0;
  parameters.max_prediction_age_s = 1.0;
  return parameters;
}

TEST(MovingTargetTrackingControllerTest, ParsesAndNamesAllSupportedModes)
{
  const auto raw = tracking_control_mode_from_string("RAW_VISUAL_POSITION");
  const auto estimated = tracking_control_mode_from_string("ESTIMATED_POSITION");
  const auto estimated_ff =
    tracking_control_mode_from_string("ESTIMATED_POSITION_VELOCITY_FF");
  const auto predicted_ff =
    tracking_control_mode_from_string("PREDICTED_POSITION_VELOCITY_FF");

  ASSERT_TRUE(raw.has_value());
  ASSERT_TRUE(estimated.has_value());
  ASSERT_TRUE(estimated_ff.has_value());
  ASSERT_TRUE(predicted_ff.has_value());
  EXPECT_STREQ(tracking_control_mode_name(*raw), "RAW_VISUAL_POSITION");
  EXPECT_STREQ(tracking_control_mode_name(*estimated), "ESTIMATED_POSITION");
  EXPECT_STREQ(
    tracking_control_mode_name(*estimated_ff),
    "ESTIMATED_POSITION_VELOCITY_FF");
  EXPECT_STREQ(
    tracking_control_mode_name(*predicted_ff),
    "PREDICTED_POSITION_VELOCITY_FF");
  EXPECT_FALSE(tracking_control_mode_from_string("UNKNOWN").has_value());
}

TEST(MovingTargetTrackingControllerTest, RawVisualModeUsesRawPositionWithoutFeedforward)
{
  MovingTargetTrackingController controller(
    permissive_parameters(TrackingControlMode::kRawVisualPosition));
  const auto command = controller.compute(make_input());

  ASSERT_TRUE(command.has_value());
  EXPECT_TRUE(command->position_target_xy.isApprox(Eigen::Vector2d{1.0, 2.0}));
  EXPECT_FALSE(command->velocity_feedforward_xy.has_value());
  EXPECT_FALSE(command->used_prediction);
}

TEST(MovingTargetTrackingControllerTest, EstimatedPositionModeUsesFilteredPosition)
{
  MovingTargetTrackingController controller(
    permissive_parameters(TrackingControlMode::kEstimatedPosition));
  auto input = make_input();
  input.raw_visual_position_xy = Eigen::Vector2d{8.0, 9.0};

  const auto command = controller.compute(input);

  ASSERT_TRUE(command.has_value());
  EXPECT_TRUE(command->position_target_xy.isApprox(Eigen::Vector2d{1.0, 2.0}));
  EXPECT_FALSE(command->velocity_feedforward_xy.has_value());
  EXPECT_FALSE(command->used_prediction);
}

TEST(MovingTargetTrackingControllerTest, PredictedModeUsesPredictionAndDeckVelocity)
{
  MovingTargetTrackingController controller(
    permissive_parameters(
      TrackingControlMode::kPredictedPositionVelocityFeedforward));
  const auto command = controller.compute(make_input());

  ASSERT_TRUE(command.has_value());
  EXPECT_TRUE(command->position_target_xy.isApprox(Eigen::Vector2d{1.2, 1.9}));
  ASSERT_TRUE(command->velocity_feedforward_xy.has_value());
  EXPECT_TRUE(command->velocity_feedforward_xy->isApprox(Eigen::Vector2d{0.4, -0.2}));
  EXPECT_TRUE(command->used_prediction);
  EXPECT_FALSE(command->used_short_loss_prediction);
}

TEST(MovingTargetTrackingControllerTest, RelativeVelocityGainDampsUavVelocityError)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.relative_velocity_gain = 0.5;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d{1.0, 0.0, 0.0});
  input.uav_velocity_xy = Eigen::Vector2d{0.2, 0.0};

  const auto command = controller.compute(input);

  ASSERT_TRUE(command.has_value());
  ASSERT_TRUE(command->velocity_feedforward_xy.has_value());
  EXPECT_TRUE(command->velocity_feedforward_xy->isApprox(Eigen::Vector2d{1.4, 0.0}));
  ASSERT_TRUE(command->effective_relative_velocity_gain.has_value());
  EXPECT_DOUBLE_EQ(*command->effective_relative_velocity_gain, 0.5);
  EXPECT_FALSE(command->estimated_deck_acceleration_xy.has_value());
}

TEST(MovingTargetTrackingControllerTest, AdaptiveGainUsesMinimumOnFirstEstimate)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.relative_velocity_gain = 0.25;
  parameters.adaptive_relative_velocity_gain_enabled = true;
  parameters.adaptive_relative_velocity_gain_parameters.min_gain = 0.25;
  parameters.adaptive_relative_velocity_gain_parameters.max_gain = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_low_threshold_mps2 = 0.10;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_high_threshold_mps2 = 0.50;
  parameters.adaptive_relative_velocity_gain_parameters.max_acceleration_mps2 = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_filter_gain = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d{1.0, 0.0, 0.0});
  input.uav_velocity_xy = Eigen::Vector2d::Zero();

  const auto command = controller.compute(input);

  ASSERT_TRUE(command.has_value());
  ASSERT_TRUE(command->effective_relative_velocity_gain.has_value());
  ASSERT_TRUE(command->estimated_deck_acceleration_xy.has_value());
  EXPECT_DOUBLE_EQ(*command->effective_relative_velocity_gain, 0.25);
  EXPECT_TRUE(command->estimated_deck_acceleration_xy->isZero(1.0e-12));
  ASSERT_TRUE(command->velocity_feedforward_xy.has_value());
  EXPECT_TRUE(command->velocity_feedforward_xy->isApprox(Eigen::Vector2d{1.25, 0.0}));
}

TEST(MovingTargetTrackingControllerTest, AdaptiveGainRisesOnNewAcceleratingEstimate)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.adaptive_relative_velocity_gain_enabled = true;
  parameters.adaptive_relative_velocity_gain_parameters.min_gain = 0.25;
  parameters.adaptive_relative_velocity_gain_parameters.max_gain = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_low_threshold_mps2 = 0.10;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_high_threshold_mps2 = 0.50;
  parameters.adaptive_relative_velocity_gain_parameters.max_acceleration_mps2 = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_filter_gain = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d{0.0, 0.0, 0.0});
  input.uav_velocity_xy = Eigen::Vector2d::Zero();
  ASSERT_TRUE(controller.compute(input).has_value());

  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d{0.6, 0.0, 0.0});
  input.estimated_state->sample_time_s = 11.0;
  const auto command = controller.compute(input);

  ASSERT_TRUE(command.has_value());
  ASSERT_TRUE(command->effective_relative_velocity_gain.has_value());
  ASSERT_TRUE(command->estimated_deck_acceleration_xy.has_value());
  EXPECT_DOUBLE_EQ(*command->effective_relative_velocity_gain, 1.0);
  EXPECT_TRUE(command->estimated_deck_acceleration_xy->isApprox(Eigen::Vector2d{0.6, 0.0}));
  ASSERT_TRUE(command->velocity_feedforward_xy.has_value());
  EXPECT_TRUE(command->velocity_feedforward_xy->isApprox(Eigen::Vector2d{1.2, 0.0}));
}

TEST(MovingTargetTrackingControllerTest, RepeatedEstimateKeepsLastAdaptiveGain)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.adaptive_relative_velocity_gain_enabled = true;
  parameters.adaptive_relative_velocity_gain_parameters.min_gain = 0.25;
  parameters.adaptive_relative_velocity_gain_parameters.max_gain = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_low_threshold_mps2 = 0.10;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_high_threshold_mps2 = 0.50;
  parameters.adaptive_relative_velocity_gain_parameters.max_acceleration_mps2 = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_filter_gain = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(controller.compute(input).has_value());
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d{0.6, 0.0, 0.0});
  input.estimated_state->sample_time_s = 11.0;
  const auto accelerated = controller.compute(input);
  const auto repeated = controller.compute(input);

  ASSERT_TRUE(accelerated.has_value());
  ASSERT_TRUE(repeated.has_value());
  ASSERT_TRUE(repeated->effective_relative_velocity_gain.has_value());
  ASSERT_TRUE(repeated->estimated_deck_acceleration_xy.has_value());
  EXPECT_DOUBLE_EQ(*repeated->effective_relative_velocity_gain, 1.0);
  EXPECT_TRUE(repeated->estimated_deck_acceleration_xy->isApprox(Eigen::Vector2d{0.6, 0.0}));
}

TEST(MovingTargetTrackingControllerTest, AdaptiveGainPersistsDuringShortLossAndResetClearsIt)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.adaptive_relative_velocity_gain_enabled = true;
  parameters.adaptive_relative_velocity_gain_parameters.min_gain = 0.25;
  parameters.adaptive_relative_velocity_gain_parameters.max_gain = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_low_threshold_mps2 = 0.10;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_high_threshold_mps2 = 0.50;
  parameters.adaptive_relative_velocity_gain_parameters.max_acceleration_mps2 = 1.0;
  parameters.adaptive_relative_velocity_gain_parameters.acceleration_filter_gain = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ASSERT_TRUE(controller.compute(input).has_value());
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(), Eigen::Vector3d{0.6, 0.0, 0.0});
  input.estimated_state->sample_time_s = 11.0;
  ASSERT_TRUE(controller.compute(input).has_value());

  input.visual_fresh = false;
  input.estimate_age_s = 0.5;
  const auto short_loss = controller.compute(input);
  ASSERT_TRUE(short_loss.has_value());
  ASSERT_TRUE(short_loss->effective_relative_velocity_gain.has_value());
  EXPECT_DOUBLE_EQ(*short_loss->effective_relative_velocity_gain, 1.0);

  controller.reset();
  input.visual_fresh = true;
  input.estimate_age_s = 0.0;
  const auto after_reset = controller.compute(input);
  ASSERT_TRUE(after_reset.has_value());
  ASSERT_TRUE(after_reset->effective_relative_velocity_gain.has_value());
  EXPECT_DOUBLE_EQ(*after_reset->effective_relative_velocity_gain, 0.25);
}

TEST(MovingTargetTrackingControllerTest, LimitsPositionBySpeedAndPerCycleStep)
{
  auto parameters = permissive_parameters(TrackingControlMode::kRawVisualPosition);
  parameters.max_position_target_speed_mps = 1.0;
  parameters.max_position_target_step_m = 0.20;
  MovingTargetTrackingController controller(parameters);

  auto input = make_input();
  input.raw_visual_position_xy = Eigen::Vector2d{10.0, 0.0};
  input.dt_s = 1.0;
  const auto step_limited = controller.compute(input);
  ASSERT_TRUE(step_limited.has_value());
  EXPECT_TRUE(step_limited->position_target_xy.isApprox(Eigen::Vector2d{0.20, 0.0}));

  input.current_target_xy.setZero();
  input.dt_s = 0.10;
  const auto speed_limited = controller.compute(input);
  ASSERT_TRUE(speed_limited.has_value());
  EXPECT_TRUE(speed_limited->position_target_xy.isApprox(Eigen::Vector2d{0.10, 0.0}));
}

TEST(MovingTargetTrackingControllerTest, LimitsVelocityMagnitude)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.max_velocity_feedforward_mps = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d{3.0, 4.0, 0.0});

  const auto command = controller.compute(input);

  ASSERT_TRUE(command.has_value());
  ASSERT_TRUE(command->velocity_feedforward_xy.has_value());
  EXPECT_NEAR(command->velocity_feedforward_xy->norm(), 1.0, 1.0e-12);
  EXPECT_TRUE(command->velocity_feedforward_xy->isApprox(Eigen::Vector2d{0.6, 0.8}));
}

TEST(MovingTargetTrackingControllerTest, LimitsVelocityAccelerationFromZeroAndHistory)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.max_velocity_feedforward_acceleration_mps2 = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.dt_s = 0.10;
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d{1.0, 0.0, 0.0});

  const auto first = controller.compute(input);
  const auto second = controller.compute(input);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  ASSERT_TRUE(first->velocity_feedforward_xy.has_value());
  ASSERT_TRUE(second->velocity_feedforward_xy.has_value());
  EXPECT_TRUE(first->velocity_feedforward_xy->isApprox(Eigen::Vector2d{0.10, 0.0}));
  EXPECT_TRUE(second->velocity_feedforward_xy->isApprox(Eigen::Vector2d{0.20, 0.0}));
}

TEST(MovingTargetTrackingControllerTest, ShortLossUsesPredictionAndDecaysFeedforward)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kPredictedPositionVelocityFeedforward);
  parameters.max_prediction_age_s = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.visual_fresh = false;
  input.estimate_age_s = 0.50;
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d{1.0, 0.0, 0.0});

  const auto command = controller.compute(input);

  ASSERT_TRUE(command.has_value());
  ASSERT_TRUE(command->velocity_feedforward_xy.has_value());
  EXPECT_TRUE(command->velocity_feedforward_xy->isApprox(Eigen::Vector2d{0.50, 0.0}));
  EXPECT_TRUE(command->used_prediction);
  EXPECT_TRUE(command->used_short_loss_prediction);
}

TEST(MovingTargetTrackingControllerTest, RejectsPredictionAfterMaximumAge)
{
  MovingTargetTrackingController controller(
    permissive_parameters(
      TrackingControlMode::kPredictedPositionVelocityFeedforward));
  auto input = make_input();
  input.visual_fresh = false;
  input.estimate_age_s = 1.01;

  EXPECT_FALSE(controller.compute(input).has_value());
}

TEST(MovingTargetTrackingControllerTest, FreshVisualDoesNotBypassStaleEstimateAge)
{
  MovingTargetTrackingController controller(
    permissive_parameters(
      TrackingControlMode::kPredictedPositionVelocityFeedforward));
  auto input = make_input();
  input.visual_fresh = true;
  input.estimate_age_s = 1.01;

  EXPECT_FALSE(controller.compute(input).has_value());
}

TEST(MovingTargetTrackingControllerTest, RawVisualModeDoesNotRequireEstimatorAge)
{
  MovingTargetTrackingController controller(
    permissive_parameters(TrackingControlMode::kRawVisualPosition));
  auto input = make_input();
  input.estimated_state.reset();
  input.predicted_position_xy.reset();
  input.estimate_age_s = std::numeric_limits<double>::infinity();

  const auto command = controller.compute(input);

  ASSERT_TRUE(command.has_value());
  EXPECT_TRUE(command->position_target_xy.isApprox(Eigen::Vector2d{1.0, 2.0}));
  EXPECT_FALSE(command->velocity_feedforward_xy.has_value());
}

TEST(MovingTargetTrackingControllerTest, RequiresUavVelocityForRelativeVelocityFeedback)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.relative_velocity_gain = 0.25;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.uav_velocity_xy.reset();

  EXPECT_FALSE(controller.compute(input).has_value());
}

TEST(MovingTargetTrackingControllerTest, ResetClearsVelocityAccelerationHistory)
{
  auto parameters = permissive_parameters(
    TrackingControlMode::kEstimatedPositionVelocityFeedforward);
  parameters.max_velocity_feedforward_acceleration_mps2 = 1.0;
  MovingTargetTrackingController controller(parameters);
  auto input = make_input();
  input.dt_s = 0.10;
  input.estimated_state = make_estimate(
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d{1.0, 0.0, 0.0});

  ASSERT_TRUE(controller.compute(input).has_value());
  const auto second = controller.compute(input);
  ASSERT_TRUE(second.has_value());
  ASSERT_TRUE(second->velocity_feedforward_xy.has_value());
  EXPECT_NEAR(second->velocity_feedforward_xy->x(), 0.20, 1.0e-12);

  controller.reset();
  const auto after_reset = controller.compute(input);
  ASSERT_TRUE(after_reset.has_value());
  ASSERT_TRUE(after_reset->velocity_feedforward_xy.has_value());
  EXPECT_NEAR(after_reset->velocity_feedforward_xy->x(), 0.10, 1.0e-12);
}

TEST(MovingTargetTrackingControllerTest, RejectsInvalidParametersAndInputs)
{
  auto invalid_parameters = permissive_parameters(TrackingControlMode::kRawVisualPosition);
  invalid_parameters.max_prediction_age_s = 0.0;
  EXPECT_THROW(
    MovingTargetTrackingController invalid_controller(invalid_parameters),
    std::invalid_argument);

  invalid_parameters = permissive_parameters(TrackingControlMode::kRawVisualPosition);
  invalid_parameters.relative_velocity_gain = -0.1;
  EXPECT_THROW(
    MovingTargetTrackingController invalid_controller(invalid_parameters),
    std::invalid_argument);

  invalid_parameters = permissive_parameters(TrackingControlMode::kRawVisualPosition);
  invalid_parameters.adaptive_relative_velocity_gain_parameters.max_gain = -1.0;
  EXPECT_THROW(
    MovingTargetTrackingController invalid_controller(invalid_parameters),
    std::invalid_argument);

  MovingTargetTrackingController controller(
    permissive_parameters(TrackingControlMode::kRawVisualPosition));
  auto input = make_input();
  input.dt_s = 0.0;
  EXPECT_FALSE(controller.compute(input).has_value());

  input = make_input();
  input.current_target_xy.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller.compute(input).has_value());

  input = make_input();
  input.raw_visual_position_xy.reset();
  EXPECT_FALSE(controller.compute(input).has_value());
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
