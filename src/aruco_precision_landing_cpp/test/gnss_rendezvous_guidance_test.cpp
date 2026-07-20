// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/gnss_rendezvous_guidance.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kTolerance = 1.0e-5;

Wgs84Position reference()
{
  return {47.397971057728974, 8.546163739800146, 2.2};
}

GnssRendezvousParameters parameters()
{
  GnssRendezvousParameters result;
  result.fix_timeout_s = 1.0;
  result.velocity_timeout_s = 1.0;
  result.stable_duration_s = 0.5;
  result.max_fix_jump_m = 5.0;
  result.max_target_step_m = 0.2;
  result.max_target_speed_mps = 2.0;
  result.search_offset_m = 1.0;
  result.search_point_hold_s = 1.0;
  result.max_geodetic_range_m = 10000.0;
  return result;
}

TEST(GnssRendezvousGuidanceTest, RequiresLocalReferenceBeforeAcceptingFix)
{
  GnssRendezvousGuidance guidance(parameters());
  EXPECT_FALSE(guidance.ingest_fix(reference(), 0.0));
  EXPECT_FALSE(guidance.has_local_reference());

  EXPECT_TRUE(guidance.set_local_reference(reference()));
  EXPECT_TRUE(guidance.has_local_reference());
  EXPECT_TRUE(guidance.ingest_fix(reference(), 0.0));
}

TEST(GnssRendezvousGuidanceTest, ConvertsWgs84AndEnuVelocityToLocalNed)
{
  GnssRendezvousGuidance guidance(parameters());
  ASSERT_TRUE(guidance.set_local_reference(reference()));

  const auto converter = GeodeticConverter::create(reference());
  ASSERT_TRUE(converter.has_value());
  const Wgs84Position deck_fix =
    *converter->local_enu_to_wgs84(Eigen::Vector3d{4.0, 3.0, -0.2});
  ASSERT_TRUE(guidance.ingest_fix(deck_fix, 1.0));
  ASSERT_TRUE(guidance.ingest_velocity_enu(Eigen::Vector3d{0.4, -0.2, 0.1}, 1.0));

  const auto estimate = guidance.estimate(1.1);
  ASSERT_TRUE(estimate.has_value());
  EXPECT_NEAR(estimate->position_ned.x(), 3.0, kTolerance);
  EXPECT_NEAR(estimate->position_ned.y(), 4.0, kTolerance);
  EXPECT_NEAR(estimate->position_ned.z(), 0.2, kTolerance);
  EXPECT_NEAR(estimate->velocity_ned.x(), -0.2, kTolerance);
  EXPECT_NEAR(estimate->velocity_ned.y(), 0.4, kTolerance);
  EXPECT_NEAR(estimate->velocity_ned.z(), -0.1, kTolerance);
}

TEST(GnssRendezvousGuidanceTest, RequiresContinuousStableFreshMeasurements)
{
  GnssRendezvousGuidance guidance(parameters());
  ASSERT_TRUE(guidance.set_local_reference(reference()));
  ASSERT_TRUE(guidance.ingest_fix(reference(), 1.0));
  ASSERT_TRUE(guidance.ingest_velocity_enu(Eigen::Vector3d::Zero(), 1.0));

  EXPECT_FALSE(guidance.ready(1.49));
  EXPECT_TRUE(guidance.ready(1.50));
  EXPECT_FALSE(guidance.ready(2.01));

  ASSERT_TRUE(guidance.ingest_fix(reference(), 2.1));
  ASSERT_TRUE(guidance.ingest_velocity_enu(Eigen::Vector3d::Zero(), 2.1));
  EXPECT_FALSE(guidance.ready(2.59));
  EXPECT_TRUE(guidance.ready(2.60));
}

TEST(GnssRendezvousGuidanceTest, DelayedVelocityStartsItsOwnStableWindow)
{
  GnssRendezvousGuidance guidance(parameters());
  ASSERT_TRUE(guidance.set_local_reference(reference()));
  ASSERT_TRUE(guidance.ingest_fix(reference(), 1.0));
  ASSERT_TRUE(guidance.ingest_velocity_enu(Eigen::Vector3d::Zero(), 1.3));

  EXPECT_FALSE(guidance.ready(1.79));
  EXPECT_TRUE(guidance.ready(1.80));
}

TEST(GnssRendezvousGuidanceTest, RejectsFreshLargeJumpButAcceptsAfterTimeout)
{
  GnssRendezvousGuidance guidance(parameters());
  ASSERT_TRUE(guidance.set_local_reference(reference()));
  const auto converter = GeodeticConverter::create(reference());
  ASSERT_TRUE(converter.has_value());

  const auto origin_fix = converter->local_enu_to_wgs84(Eigen::Vector3d::Zero());
  const auto far_fix = converter->local_enu_to_wgs84(Eigen::Vector3d{8.0, 0.0, 0.0});
  ASSERT_TRUE(origin_fix.has_value());
  ASSERT_TRUE(far_fix.has_value());

  EXPECT_TRUE(guidance.ingest_fix(*origin_fix, 1.0));
  EXPECT_FALSE(guidance.ingest_fix(*far_fix, 1.2));
  EXPECT_TRUE(guidance.ingest_fix(*far_fix, 2.1));
}

TEST(GnssRendezvousGuidanceTest, ReferenceChangeClearsMeasurements)
{
  GnssRendezvousGuidance guidance(parameters());
  ASSERT_TRUE(guidance.set_local_reference(reference()));
  ASSERT_TRUE(guidance.ingest_fix(reference(), 1.0));
  ASSERT_TRUE(guidance.ingest_velocity_enu(Eigen::Vector3d::Zero(), 1.0));
  ASSERT_TRUE(guidance.estimate(1.1).has_value());

  Wgs84Position changed_reference = reference();
  changed_reference.longitude_deg += 0.001;
  EXPECT_TRUE(guidance.set_local_reference(changed_reference));
  EXPECT_FALSE(guidance.estimate(1.1).has_value());
}

TEST(GnssRendezvousGuidanceTest, LimitsTargetBySpeedAndPerCycleStep)
{
  GnssRendezvousGuidance guidance(parameters());

  const auto speed_limited = guidance.limit_target_xy(
    Eigen::Vector2d{0.0, 0.0}, Eigen::Vector2d{3.0, 4.0}, 0.05);
  ASSERT_TRUE(speed_limited.has_value());
  EXPECT_NEAR(speed_limited->norm(), 0.1, kTolerance);
  EXPECT_NEAR(speed_limited->x(), 0.06, kTolerance);
  EXPECT_NEAR(speed_limited->y(), 0.08, kTolerance);

  const auto step_limited = guidance.limit_target_xy(
    Eigen::Vector2d{0.0, 0.0}, Eigen::Vector2d{3.0, 4.0}, 1.0);
  ASSERT_TRUE(step_limited.has_value());
  EXPECT_NEAR(step_limited->norm(), 0.2, kTolerance);

  const auto close_target = guidance.limit_target_xy(
    Eigen::Vector2d{0.0, 0.0}, Eigen::Vector2d{0.03, 0.04}, 1.0);
  ASSERT_TRUE(close_target.has_value());
  EXPECT_NEAR(close_target->x(), 0.03, kTolerance);
  EXPECT_NEAR(close_target->y(), 0.04, kTolerance);
}

TEST(GnssRendezvousGuidanceTest, GeneratesMovingCenteredSearchPattern)
{
  GnssRendezvousGuidance guidance(parameters());

  const auto center = guidance.search_offset(0.0);
  const auto north = guidance.search_offset(1.0);
  const auto east = guidance.search_offset(2.0);
  const auto south = guidance.search_offset(3.0);
  const auto west = guidance.search_offset(4.0);
  const auto repeated_center = guidance.search_offset(5.0);

  ASSERT_TRUE(center.has_value());
  ASSERT_TRUE(north.has_value());
  ASSERT_TRUE(east.has_value());
  ASSERT_TRUE(south.has_value());
  ASSERT_TRUE(west.has_value());
  ASSERT_TRUE(repeated_center.has_value());
  EXPECT_EQ(*center, Eigen::Vector2d::Zero());
  EXPECT_EQ(*north, (Eigen::Vector2d{1.0, 0.0}));
  EXPECT_EQ(*east, (Eigen::Vector2d{0.0, 1.0}));
  EXPECT_EQ(*south, (Eigen::Vector2d{-1.0, 0.0}));
  EXPECT_EQ(*west, (Eigen::Vector2d{0.0, -1.0}));
  EXPECT_EQ(*repeated_center, Eigen::Vector2d::Zero());
}

TEST(GnssRendezvousGuidanceTest, RejectsInvalidInputs)
{
  GnssRendezvousParameters invalid = parameters();
  invalid.fix_timeout_s = 0.0;
  EXPECT_THROW(
    {
      const GnssRendezvousGuidance guidance(invalid);
      static_cast<void>(guidance);
    },
    std::invalid_argument);

  GnssRendezvousGuidance guidance(parameters());
  Wgs84Position invalid_reference = reference();
  invalid_reference.latitude_deg = 91.0;
  EXPECT_FALSE(guidance.set_local_reference(invalid_reference));

  ASSERT_TRUE(guidance.set_local_reference(reference()));
  EXPECT_FALSE(guidance.ingest_fix(
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 0.0));
  EXPECT_FALSE(guidance.ingest_velocity_enu(
    Eigen::Vector3d{0.0, std::numeric_limits<double>::infinity(), 0.0}, 0.0));
  EXPECT_FALSE(guidance.limit_target_xy(
    Eigen::Vector2d::Zero(), Eigen::Vector2d::Ones(), 0.0).has_value());
  EXPECT_FALSE(guidance.search_offset(-1.0).has_value());
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
