// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/deck_motion_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Eigen::Quaterniond exp_rotation(const Eigen::Vector3d & vector)
{
  const double angle = vector.norm();
  return angle < 1.0e-12 ?
         Eigen::Quaterniond::Identity() :
         Eigen::Quaterniond(Eigen::AngleAxisd(angle, vector / angle));
}

Eigen::Quaterniond flat_deck_orientation()
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()));
}

double wrapped_yaw(const Eigen::Quaterniond & orientation)
{
  const Eigen::Vector3d deck_x_ned = orientation * Eigen::Vector3d::UnitX();
  return std::atan2(deck_x_ned.y(), deck_x_ned.x());
}

Pose3d constant_acceleration_pose(
  double time_s,
  const Eigen::Vector3d & position,
  const Eigen::Vector3d & velocity,
  const Eigen::Vector3d & acceleration,
  double yaw_rate_radps,
  double yaw_acceleration_radps2)
{
  Pose3d pose;
  pose.translation =
    position + time_s * velocity + 0.5 * time_s * time_s * acceleration;
  const double yaw =
    time_s * yaw_rate_radps +
    0.5 * time_s * time_s * yaw_acceleration_radps2;
  pose.rotation =
    exp_rotation(yaw * Eigen::Vector3d::UnitZ()) * flat_deck_orientation();
  return pose;
}

DeckMotionEstimatorParameters precise_parameters()
{
  DeckMotionEstimatorParameters parameters;
  parameters.linear_jerk_std_mps3 = 0.10;
  parameters.angular_jerk_std_radps3 = 0.05;
  parameters.measurement_horizontal_std_m = 0.001;
  parameters.measurement_vertical_std_m = 0.001;
  parameters.measurement_orientation_std_rad = 0.001;
  parameters.position_innovation_gate_mahalanobis = 100.0;
  parameters.orientation_innovation_gate_mahalanobis = 100.0;
  return parameters;
}

TEST(DeckMotionEstimatorTest, RejectsInvalidParameters)
{
  auto parameters = precise_parameters();
  parameters.trusted_prediction_horizon_s = 1.1;
  EXPECT_THROW(DeckMotionEstimator{parameters}, std::invalid_argument);
}

TEST(DeckMotionEstimatorTest, EstimatesAndPredictsConstantAccelerationMotion)
{
  DeckMotionEstimator estimator(precise_parameters());
  const Eigen::Vector3d initial_position{1.0, -2.0, 4.0};
  const Eigen::Vector3d velocity{0.20, -0.10, 0.05};
  const Eigen::Vector3d acceleration{0.08, 0.04, -0.02};
  constexpr double yaw_rate = 0.08;
  constexpr double yaw_acceleration = 0.03;
  constexpr double dt_s = 0.05;

  for (int sample = 0; sample <= 160; ++sample) {
    const double time_s = sample * dt_s;
    const auto result = estimator.update(
      constant_acceleration_pose(
        time_s, initial_position, velocity, acceleration,
        yaw_rate, yaw_acceleration),
      Eigen::Vector3d::Zero(),
      sample < 80 ? 0 : 1,
      time_s);
    EXPECT_TRUE(
      result.status == DeckMotionUpdateStatus::kInitialized ||
      result.status == DeckMotionUpdateStatus::kUpdated);
  }

  const double now_s = 160 * dt_s;
  const auto prediction = estimator.predict(now_s);
  ASSERT_TRUE(prediction.has_value());
  ASSERT_EQ(prediction->points.size(), 21U);
  EXPECT_NEAR(prediction->trusted_horizon_s, 0.50, 1.0e-12);
  const auto & half_second = prediction->points[10];
  const auto expected = constant_acceleration_pose(
    now_s + 0.50, initial_position, velocity, acceleration,
    yaw_rate, yaw_acceleration);
  EXPECT_TRUE(half_second.trusted);
  EXPECT_TRUE(half_second.position_ned_m.isApprox(expected.translation, 0.01));
  const Eigen::Quaterniond attitude_error =
    expected.rotation * half_second.orientation_deck_to_ned.conjugate();
  EXPECT_LT(Eigen::AngleAxisd(attitude_error).angle(), 0.01);
  EXPECT_TRUE(
    half_second.velocity_ned_mps.isApprox(
      velocity + (now_s + 0.50) * acceleration, 0.02));
  EXPECT_NEAR(
    half_second.angular_velocity_ned_radps.z(),
    yaw_rate + (now_s + 0.50) * yaw_acceleration,
    0.02);
  EXPECT_FALSE(prediction->points.back().trusted);
}

TEST(DeckMotionEstimatorTest, CompensatesUavMotionAndPredictsFromFrozenOrigin)
{
  DeckMotionEstimator estimator(precise_parameters());
  const Eigen::Vector3d deck_position{1.0, -2.0, 4.0};
  const Eigen::Vector3d deck_velocity{0.30, 0.10, -0.05};
  const Eigen::Vector3d deck_acceleration{0.04, -0.02, 0.01};
  const Eigen::Vector3d uav_velocity{-0.20, 0.25, -0.10};
  constexpr double dt_s = 0.05;

  for (int sample = 0; sample <= 160; ++sample) {
    const double time_s = sample * dt_s;
    Pose3d relative_pose = constant_acceleration_pose(
      time_s, deck_position, deck_velocity, deck_acceleration, 0.0, 0.0);
    relative_pose.translation -= time_s * uav_velocity;
    const auto result = estimator.update(
      relative_pose, uav_velocity, 0, time_s);
    EXPECT_TRUE(
      result.status == DeckMotionUpdateStatus::kInitialized ||
      result.status == DeckMotionUpdateStatus::kUpdated);
  }

  const double now_s = 160 * dt_s;
  const auto estimate = estimator.estimate();
  const auto prediction = estimator.predict(now_s);
  ASSERT_TRUE(estimate.has_value());
  ASSERT_TRUE(prediction.has_value());
  const Eigen::Vector3d expected_relative_now =
    deck_position + now_s * deck_velocity +
    0.5 * now_s * now_s * deck_acceleration - now_s * uav_velocity;
  EXPECT_TRUE(estimate->position_ned_m.isApprox(expected_relative_now, 0.01));
  EXPECT_TRUE(estimate->velocity_ned_mps.isApprox(
    deck_velocity + now_s * deck_acceleration, 0.02));

  const auto & half_second = prediction->points[10];
  const double target_time_s = now_s + 0.50;
  const Eigen::Vector3d expected_from_frozen_origin =
    deck_position + target_time_s * deck_velocity +
    0.5 * target_time_s * target_time_s * deck_acceleration -
    now_s * uav_velocity;
  EXPECT_TRUE(half_second.position_ned_m.isApprox(
    expected_from_frozen_origin, 0.02));
  EXPECT_TRUE(half_second.velocity_ned_mps.isApprox(
    deck_velocity + target_time_s * deck_acceleration, 0.02));
}

TEST(DeckMotionEstimatorTest, HandlesQuaternionAntipodesAndRejectsBadSamples)
{
  DeckMotionEstimator estimator(precise_parameters());
  Pose3d pose{Eigen::Vector3d{0.0, 0.0, 2.0}, flat_deck_orientation()};
  EXPECT_EQ(
    estimator.update(pose, Eigen::Vector3d::Zero(), 0, 1.0).status,
    DeckMotionUpdateStatus::kInitialized);

  pose.rotation.coeffs() *= -1.0;
  EXPECT_EQ(
    estimator.update(pose, Eigen::Vector3d::Zero(), 1, 1.05).status,
    DeckMotionUpdateStatus::kUpdated);
  EXPECT_EQ(
    estimator.update(pose, Eigen::Vector3d::Zero(), 1, 1.05).status,
    DeckMotionUpdateStatus::kRejectedNonMonotonicTime);

  Pose3d flipped = pose;
  flipped.rotation = Eigen::Quaterniond::Identity();
  EXPECT_EQ(
    estimator.update(flipped, Eigen::Vector3d::Zero(), 1, 1.10).status,
    DeckMotionUpdateStatus::kRejectedInvalidInput);

  Pose3d invalid = pose;
  invalid.translation.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    estimator.update(invalid, Eigen::Vector3d::Zero(), 1, 1.15).status,
    DeckMotionUpdateStatus::kRejectedInvalidInput);
  invalid = pose;
  invalid.rotation = Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0};
  EXPECT_EQ(
    estimator.update(invalid, Eigen::Vector3d::Zero(), 1, 1.15).status,
    DeckMotionUpdateStatus::kRejectedInvalidInput);

  Pose3d outlier = pose;
  outlier.translation.x() = 100.0;
  EXPECT_EQ(
    estimator.update(outlier, Eigen::Vector3d::Zero(), 1, 1.20).status,
    DeckMotionUpdateStatus::kRejectedOutlier);
  ASSERT_TRUE(estimator.estimate().has_value());
  EXPECT_DOUBLE_EQ(estimator.estimate()->sample_time_s, 1.05);
  const auto rejected_prediction = estimator.predict(1.20);
  ASSERT_TRUE(rejected_prediction.has_value());
  EXPECT_DOUBLE_EQ(rejected_prediction->trusted_horizon_s, 0.0);
  EXPECT_TRUE(std::none_of(
    rejected_prediction->points.begin(), rejected_prediction->points.end(),
    [](const DeckMotionPredictionPoint & point) {return point.trusted;}));
  const auto old_prediction = estimator.predict(1.56);
  ASSERT_TRUE(old_prediction.has_value());
  EXPECT_DOUBLE_EQ(old_prediction->trusted_horizon_s, 0.0);

  EXPECT_EQ(
    estimator.update(pose, Eigen::Vector3d::Zero(), 1, 1.25).status,
    DeckMotionUpdateStatus::kUpdated);
  ASSERT_TRUE(estimator.predict(1.25).has_value());
  EXPECT_DOUBLE_EQ(estimator.predict(1.25)->trusted_horizon_s, 0.5);
}

TEST(DeckMotionEstimatorTest, TracksBothRotationDirectionsAcrossWrappedYaw)
{
  for (const double direction : {-1.0, 1.0}) {
    DeckMotionEstimator estimator(precise_parameters());
    for (int sample = 0; sample <= 80; ++sample) {
      const double time_s = 0.05 * sample;
      const double yaw = direction * (170.0 + 5.0 * time_s) * kPi / 180.0;
      Pose3d pose{
        Eigen::Vector3d{0.0, 0.0, 2.0},
        exp_rotation(yaw * Eigen::Vector3d::UnitZ()) * flat_deck_orientation()};
      const auto result = estimator.update(
        pose, Eigen::Vector3d::Zero(), 0, time_s);
      EXPECT_TRUE(
        result.status == DeckMotionUpdateStatus::kInitialized ||
        result.status == DeckMotionUpdateStatus::kUpdated);
    }
    const auto estimate = estimator.estimate();
    ASSERT_TRUE(estimate.has_value());
    EXPECT_NEAR(
      std::remainder(wrapped_yaw(estimate->orientation_deck_to_ned) -
      direction * 190.0 * kPi / 180.0, 2.0 * kPi),
      0.0, 0.02);
    EXPECT_NEAR(
      estimate->angular_velocity_ned_radps.z(),
      direction * 5.0 * kPi / 180.0, 0.03);
  }
}

TEST(DeckMotionEstimatorTest, HandlesSinusoidalSixDofJitterAndShortDropout)
{
  auto parameters = precise_parameters();
  parameters.linear_jerk_std_mps3 = 1.0;
  parameters.angular_jerk_std_radps3 = 0.5;
  DeckMotionEstimator estimator(parameters);
  double time_s = 0.0;
  Pose3d expected;
  for (int sample = 0; sample < 180; ++sample) {
    time_s += sample % 3 == 0 ? 0.04 : (sample % 3 == 1 ? 0.05 : 0.06);
    if (sample == 90) {
      time_s += 0.20;
    }
    expected.translation = Eigen::Vector3d{
      std::sin(0.5 * time_s),
      0.5 * std::sin(0.8 * time_s),
      2.0 + 0.2 * std::sin(0.6 * time_s)};
    const Eigen::Vector3d rotation_vector{
      0.05 * std::sin(0.7 * time_s),
      0.04 * std::sin(0.9 * time_s),
      0.10 * std::sin(0.4 * time_s)};
    expected.rotation = exp_rotation(rotation_vector) * flat_deck_orientation();
    const auto result = estimator.update(
      expected, Eigen::Vector3d::Zero(), sample < 100 ? 0 : 1, time_s);
    EXPECT_TRUE(
      result.status == DeckMotionUpdateStatus::kInitialized ||
      result.status == DeckMotionUpdateStatus::kUpdated);
  }
  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  EXPECT_TRUE(estimate->position_ned_m.isApprox(expected.translation, 0.05));
  const Eigen::Quaterniond error =
    expected.rotation * estimate->orientation_deck_to_ned.conjugate();
  EXPECT_LT(Eigen::AngleAxisd(error).angle(), 0.04);
  ASSERT_TRUE(estimator.predict(time_s + 0.10).has_value());
  EXPECT_NEAR(estimator.predict(time_s + 0.10)->trusted_horizon_s, 0.40, 1.0e-9);
}

TEST(DeckMotionEstimatorTest, ReinitializesAfterLongGapAndLimitsTrustedHorizon)
{
  DeckMotionEstimator estimator(precise_parameters());
  const Pose3d pose{Eigen::Vector3d{0.0, 0.0, 2.0}, flat_deck_orientation()};
  ASSERT_EQ(
    estimator.update(pose, Eigen::Vector3d::Zero(), 0, 1.0).status,
    DeckMotionUpdateStatus::kInitialized);
  EXPECT_EQ(
    estimator.update(pose, Eigen::Vector3d::Zero(), 0, 3.1).status,
    DeckMotionUpdateStatus::kReinitialized);

  const auto prediction = estimator.predict(3.3);
  ASSERT_TRUE(prediction.has_value());
  EXPECT_EQ(prediction->points.size(), 21U);
  EXPECT_NEAR(prediction->observation_age_s, 0.2, 1.0e-12);
  EXPECT_NEAR(prediction->trusted_horizon_s, 0.3, 1.0e-12);
  EXPECT_TRUE(prediction->points.front().trusted);
  EXPECT_FALSE(prediction->points.back().trusted);
  EXPECT_FALSE(estimator.predict(4.2).has_value());
}

TEST(DeckMotionEstimatorTest, KeepsCovariancesFiniteAndPositiveSemidefinite)
{
  DeckMotionEstimator estimator(precise_parameters());
  const Pose3d pose{Eigen::Vector3d{0.0, 0.0, 2.0}, flat_deck_orientation()};
  for (int sample = 0; sample <= 8; ++sample) {
    const auto result = estimator.update(
      pose, Eigen::Vector3d::Zero(), 0, 1.0 + 0.05 * sample);
    ASSERT_TRUE(
      result.status == DeckMotionUpdateStatus::kInitialized ||
      result.status == DeckMotionUpdateStatus::kUpdated);
  }
  const auto estimate = estimator.estimate();
  ASSERT_TRUE(estimate.has_value());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>>
  translation_solver(estimate->translation_covariance);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>>
  rotation_solver(estimate->rotation_covariance);
  ASSERT_EQ(translation_solver.info(), Eigen::Success);
  ASSERT_EQ(rotation_solver.info(), Eigen::Success);
  EXPECT_GE(translation_solver.eigenvalues().minCoeff(), -1.0e-12);
  EXPECT_GE(rotation_solver.eigenvalues().minCoeff(), -1.0e-12);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
