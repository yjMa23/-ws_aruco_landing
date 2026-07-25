// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/deck_attitude_estimator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kTolerance = 1.0e-9;

Eigen::Vector3d normal_from_roll_pitch(double roll_rad, double pitch_rad)
{
  return Eigen::Vector3d{
    -std::sin(pitch_rad) * std::cos(roll_rad),
    std::sin(roll_rad),
    -std::cos(pitch_rad) * std::cos(roll_rad)};
}

Eigen::Quaterniond rotation_with_normal(const Eigen::Vector3d & normal_ned)
{
  return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal_ned);
}

TEST(DeckAttitudeEstimatorTest, HorizontalMarkerFlipProducesZeroTilt)
{
  DeckAttitudeEstimator estimator({1.0, 0.5});
  const Eigen::Quaterniond marker_flip(
    Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitX()));

  const auto estimate = estimator.update(marker_flip);

  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->roll_rad, 0.0, kTolerance);
  EXPECT_NEAR(estimate->pitch_rad, 0.0, kTolerance);
  EXPECT_NEAR(estimate->tilt_rad, 0.0, kTolerance);
  EXPECT_TRUE(estimate->upward_normal_ned.isApprox(Eigen::Vector3d{0.0, 0.0, -1.0}));
}

TEST(DeckAttitudeEstimatorTest, RecoversKnownYawIndependentRollPitch)
{
  DeckAttitudeEstimator estimator({1.0, 0.5});
  const double roll = 5.0 * std::acos(-1.0) / 180.0;
  const double pitch = -3.0 * std::acos(-1.0) / 180.0;
  const Eigen::Vector3d normal = normal_from_roll_pitch(roll, pitch).normalized();
  const Eigen::Quaterniond base = rotation_with_normal(normal);
  const Eigen::Quaterniond spin(
    Eigen::AngleAxisd(1.2, normal));

  const auto estimate = estimator.update(spin * base);

  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->roll_rad, roll, 1.0e-9);
  EXPECT_NEAR(estimate->pitch_rad, pitch, 1.0e-9);
  EXPECT_NEAR(estimate->tilt_rad, std::acos(-normal.z()), 1.0e-9);
}

TEST(DeckAttitudeEstimatorTest, FiltersNormalBeforeComputingAngles)
{
  DeckAttitudeEstimator estimator({0.5, 0.5});
  const Eigen::Quaterniond horizontal = rotation_with_normal(
    Eigen::Vector3d{0.0, 0.0, -1.0});
  const Eigen::Vector3d tilted_normal = normal_from_roll_pitch(0.10, 0.0).normalized();
  const Eigen::Quaterniond tilted = rotation_with_normal(tilted_normal);

  ASSERT_TRUE(estimator.update(horizontal).has_value());
  const auto estimate = estimator.update(tilted);

  ASSERT_TRUE(estimate.has_value());
  const Eigen::Vector3d expected =
    (Eigen::Vector3d{0.0, 0.0, -1.0} + tilted_normal).normalized();
  EXPECT_TRUE(estimate->upward_normal_ned.isApprox(expected, 1.0e-12));
  EXPECT_GT(estimate->roll_rad, 0.0);
  EXPECT_LT(estimate->roll_rad, 0.10);
}

TEST(DeckAttitudeEstimatorTest, RejectsDownwardOrInvalidMarkerNormal)
{
  DeckAttitudeEstimator estimator({1.0, 0.5});
  EXPECT_FALSE(estimator.update(Eigen::Quaterniond::Identity()).has_value());
  EXPECT_FALSE(estimator.update(Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0}).has_value());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(estimator.update(Eigen::Quaterniond{nan, 0.0, 0.0, 1.0}).has_value());
}

TEST(DeckAttitudeEstimatorTest, ResetRemovesFilterHistory)
{
  DeckAttitudeEstimator estimator({0.1, 0.5});
  const Eigen::Quaterniond horizontal = rotation_with_normal(
    Eigen::Vector3d{0.0, 0.0, -1.0});
  const Eigen::Vector3d tilted_normal = normal_from_roll_pitch(0.15, 0.0).normalized();
  const Eigen::Quaterniond tilted = rotation_with_normal(tilted_normal);
  ASSERT_TRUE(estimator.update(horizontal).has_value());
  ASSERT_TRUE(estimator.update(tilted).has_value());

  estimator.reset();
  const auto estimate = estimator.update(tilted);

  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->roll_rad, 0.15, 1.0e-9);
}

TEST(DeckAttitudeEstimatorTest, RejectsInvalidParameters)
{
  EXPECT_THROW((void)DeckAttitudeEstimator({0.0, 0.5}), std::invalid_argument);
  EXPECT_THROW((void)DeckAttitudeEstimator({1.1, 0.5}), std::invalid_argument);
  EXPECT_THROW((void)DeckAttitudeEstimator({0.2, 0.0}), std::invalid_argument);
  EXPECT_THROW((void)DeckAttitudeEstimator({0.2, 1.1}), std::invalid_argument);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
