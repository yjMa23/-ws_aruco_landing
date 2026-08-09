// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/vehicle_pose_history.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{
namespace
{

Pose3d make_pose(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation = Eigen::Quaterniond::Identity())
{
  return Pose3d{translation, rotation};
}

void expect_quaternion_equivalent(
  const Eigen::Quaterniond & actual,
  const Eigen::Quaterniond & expected,
  double tolerance = 1.0e-9)
{
  const double dot = std::abs(actual.normalized().dot(expected.normalized()));
  EXPECT_NEAR(dot, 1.0, tolerance);
}

TEST(VehiclePoseHistoryTest, ExactTimeReturnsStoredPose)
{
  VehiclePoseHistory history({2.0, 0.03});
  const Pose3d pose = make_pose(
    Eigen::Vector3d{1.0, -2.0, 3.0},
    Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ())));

  ASSERT_TRUE(history.add_sample(pose, 10.0));
  const auto result = history.lookup(10.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->translation.isApprox(pose.translation, 1.0e-12));
  expect_quaternion_equivalent(result->rotation, pose.rotation);
}

TEST(VehiclePoseHistoryTest, InterpolatesTranslationLinearly)
{
  VehiclePoseHistory history({2.0, 0.03});
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d{0.0, 0.0, 0.0}), 1.0));
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d{2.0, -4.0, 6.0}), 3.0));

  const auto result = history.lookup(2.0);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->translation.isApprox(Eigen::Vector3d{1.0, -2.0, 3.0}, 1.0e-12));
}

TEST(VehiclePoseHistoryTest, InterpolatesOrientationWithShortestPathSlerp)
{
  VehiclePoseHistory history({2.0, 0.03});
  const Eigen::Quaterniond start = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond finish(
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero(), start), 1.0));
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero(), finish), 2.0));

  const auto result = history.lookup(1.5);

  ASSERT_TRUE(result.has_value());
  const Eigen::Quaterniond expected(
    Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  expect_quaternion_equivalent(result->rotation, expected, 1.0e-8);
}

TEST(VehiclePoseHistoryTest, InterpolatesNedVelocityWithPose)
{
  VehiclePoseHistory history({2.0, 0.03});
  ASSERT_TRUE(history.add_sample(
    VehicleKinematicState{
      make_pose(Eigen::Vector3d::Zero()), Eigen::Vector3d{1.0, -2.0, 0.5}},
    1.0));
  ASSERT_TRUE(history.add_sample(
    VehicleKinematicState{
      make_pose(Eigen::Vector3d::Ones()), Eigen::Vector3d{3.0, 2.0, -0.5}},
    2.0));

  const auto result = history.lookup_state(1.25);

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->pose.translation.isApprox(
    Eigen::Vector3d::Constant(0.25), 1.0e-12));
  EXPECT_TRUE(result->velocity_ned_mps.isApprox(
    Eigen::Vector3d{1.5, -1.0, 0.25}, 1.0e-12));
}

TEST(VehiclePoseHistoryTest, HandlesEquivalentQuaternionWithOppositeSign)
{
  VehiclePoseHistory history({2.0, 0.03});
  Eigen::Quaterniond rotation(
    Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY()));
  Eigen::Quaterniond opposite = rotation;
  opposite.coeffs() *= -1.0;
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero(), rotation), 1.0));
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero(), opposite), 2.0));

  const auto result = history.lookup(1.5);

  ASSERT_TRUE(result.has_value());
  expect_quaternion_equivalent(result->rotation, rotation);
}

TEST(VehiclePoseHistoryTest, HoldsEndpointsOnlyWithinConfiguredTolerance)
{
  VehiclePoseHistory history({2.0, 0.05});
  const Pose3d first = make_pose(Eigen::Vector3d{1.0, 0.0, 0.0});
  const Pose3d last = make_pose(Eigen::Vector3d{2.0, 0.0, 0.0});
  ASSERT_TRUE(history.add_sample(first, 1.0));
  ASSERT_TRUE(history.add_sample(last, 2.0));

  ASSERT_TRUE(history.lookup(0.96).has_value());
  ASSERT_TRUE(history.lookup(2.04).has_value());
  EXPECT_FALSE(history.lookup(0.94).has_value());
  EXPECT_FALSE(history.lookup(2.06).has_value());
}

TEST(VehiclePoseHistoryTest, RejectsRepeatedBackwardAndInvalidSamples)
{
  VehiclePoseHistory history({2.0, 0.03});
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero()), 1.0));

  EXPECT_FALSE(history.add_sample(make_pose(Eigen::Vector3d::Ones()), 1.0));
  EXPECT_FALSE(history.add_sample(make_pose(Eigen::Vector3d::Ones()), 0.5));
  EXPECT_FALSE(history.add_sample(
    make_pose(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())),
    2.0));
  EXPECT_FALSE(history.add_sample(
    make_pose(Eigen::Vector3d::Zero(), Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0}),
    2.0));
  EXPECT_FALSE(history.add_sample(
    make_pose(Eigen::Vector3d::Zero()),
    std::numeric_limits<double>::infinity()));
  EXPECT_EQ(history.size(), 1U);
}

TEST(VehiclePoseHistoryTest, PrunesSamplesOutsideHistoryDuration)
{
  VehiclePoseHistory history({1.0, 0.0});
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero()), 0.0));
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero()), 0.5));
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero()), 1.0));
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero()), 1.5));

  EXPECT_EQ(history.size(), 3U);
  EXPECT_FALSE(history.lookup(0.0).has_value());
  EXPECT_TRUE(history.lookup(0.5).has_value());
}

TEST(VehiclePoseHistoryTest, ResetClearsAllSamples)
{
  VehiclePoseHistory history({2.0, 0.03});
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero()), 1.0));

  history.reset();

  EXPECT_EQ(history.size(), 0U);
  EXPECT_FALSE(history.lookup(1.0).has_value());
}

TEST(VehiclePoseHistoryTest, RejectsInvalidQueryAndParameters)
{
  EXPECT_THROW(VehiclePoseHistory({0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(VehiclePoseHistory({1.0, -0.1}), std::invalid_argument);
  EXPECT_THROW(
    VehiclePoseHistory({std::numeric_limits<double>::quiet_NaN(), 0.0}),
    std::invalid_argument);

  VehiclePoseHistory history({2.0, 0.03});
  ASSERT_TRUE(history.add_sample(make_pose(Eigen::Vector3d::Zero()), 1.0));
  EXPECT_FALSE(history.lookup(std::numeric_limits<double>::quiet_NaN()).has_value());
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
