// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/geodetic_converter.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kLatitudeDeg = 47.397971057728974;
constexpr double kLongitudeDeg = 8.546163739800146;
constexpr double kAltitudeM = 2.2;

Wgs84Position make_origin()
{
  return Wgs84Position{kLatitudeDeg, kLongitudeDeg, kAltitudeM};
}

void expect_vector_near(
  const Eigen::Vector3d & actual,
  const Eigen::Vector3d & expected,
  double tolerance)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

TEST(GeodeticConverterTest, OriginMapsToZero)
{
  const auto converter = GeodeticConverter::create(make_origin());
  ASSERT_TRUE(converter.has_value());

  const auto local = converter->wgs84_to_local_enu(make_origin());
  ASSERT_TRUE(local.has_value());
  expect_vector_near(*local, Eigen::Vector3d::Zero(), 1.0e-7);
}

TEST(GeodeticConverterTest, LongitudeLatitudeAndAltitudeHaveExpectedSigns)
{
  const auto converter = GeodeticConverter::create(make_origin());
  ASSERT_TRUE(converter.has_value());

  Wgs84Position east = make_origin();
  east.longitude_deg += 1.0e-5;
  const auto east_enu = converter->wgs84_to_local_enu(east);
  ASSERT_TRUE(east_enu.has_value());
  EXPECT_GT(east_enu->x(), 0.0);
  EXPECT_NEAR(east_enu->y(), 0.0, 1.0e-4);

  Wgs84Position north = make_origin();
  north.latitude_deg += 1.0e-5;
  const auto north_enu = converter->wgs84_to_local_enu(north);
  ASSERT_TRUE(north_enu.has_value());
  EXPECT_GT(north_enu->y(), 0.0);
  EXPECT_NEAR(north_enu->x(), 0.0, 1.0e-4);

  Wgs84Position up = make_origin();
  up.altitude_m += 10.0;
  const auto up_enu = converter->wgs84_to_local_enu(up);
  ASSERT_TRUE(up_enu.has_value());
  EXPECT_GT(up_enu->z(), 0.0);
  EXPECT_NEAR(up_enu->x(), 0.0, 1.0e-7);
  EXPECT_NEAR(up_enu->y(), 0.0, 1.0e-7);
  EXPECT_NEAR(up_enu->z(), 10.0, 1.0e-6);
}

TEST(GeodeticConverterTest, LocalEnuRoundTripIsAccurate)
{
  const auto converter = GeodeticConverter::create(make_origin());
  ASSERT_TRUE(converter.has_value());

  const Eigen::Vector3d expected_enu(123.4, -456.7, 31.2);
  const auto position_wgs84 = converter->local_enu_to_wgs84(expected_enu);
  ASSERT_TRUE(position_wgs84.has_value());

  const auto actual_enu = converter->wgs84_to_local_enu(*position_wgs84);
  ASSERT_TRUE(actual_enu.has_value());
  expect_vector_near(*actual_enu, expected_enu, 1.0e-4);
}

TEST(GeodeticConverterTest, Wgs84RoundTripIsAccurate)
{
  const auto converter = GeodeticConverter::create(make_origin());
  ASSERT_TRUE(converter.has_value());

  const Wgs84Position target{
    kLatitudeDeg + 0.001,
    kLongitudeDeg - 0.0015,
    kAltitudeM + 45.0};
  const auto target_enu = converter->wgs84_to_local_enu(target);
  ASSERT_TRUE(target_enu.has_value());

  const auto round_trip = converter->local_enu_to_wgs84(*target_enu);
  ASSERT_TRUE(round_trip.has_value());
  EXPECT_NEAR(round_trip->latitude_deg, target.latitude_deg, 1.0e-9);
  EXPECT_NEAR(round_trip->longitude_deg, target.longitude_deg, 1.0e-9);
  EXPECT_NEAR(round_trip->altitude_m, target.altitude_m, 1.0e-4);
}

TEST(GeodeticConverterTest, RejectsInvalidOriginAndConfiguration)
{
  Wgs84Position invalid_latitude = make_origin();
  invalid_latitude.latitude_deg = 91.0;
  EXPECT_FALSE(GeodeticConverter::create(invalid_latitude).has_value());

  Wgs84Position invalid_longitude = make_origin();
  invalid_longitude.longitude_deg = -181.0;
  EXPECT_FALSE(GeodeticConverter::create(invalid_longitude).has_value());

  Wgs84Position invalid_altitude = make_origin();
  invalid_altitude.altitude_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(GeodeticConverter::create(invalid_altitude).has_value());

  EXPECT_FALSE(GeodeticConverter::create(make_origin(), 0.0).has_value());
  EXPECT_FALSE(GeodeticConverter::create(
    make_origin(), std::numeric_limits<double>::infinity()).has_value());
}

TEST(GeodeticConverterTest, RejectsInvalidInput)
{
  const auto converter = GeodeticConverter::create(make_origin());
  ASSERT_TRUE(converter.has_value());

  Wgs84Position invalid_fix = make_origin();
  invalid_fix.longitude_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(converter->wgs84_to_local_enu(invalid_fix).has_value());

  Eigen::Vector3d invalid_enu = Eigen::Vector3d::Zero();
  invalid_enu.z() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(converter->local_enu_to_wgs84(invalid_enu).has_value());
}

TEST(GeodeticConverterTest, RejectsPositionsOutsideConfiguredLocalRange)
{
  const auto small_converter = GeodeticConverter::create(make_origin(), 100.0);
  const auto large_converter = GeodeticConverter::create(make_origin(), 1000.0);
  ASSERT_TRUE(small_converter.has_value());
  ASSERT_TRUE(large_converter.has_value());

  EXPECT_FALSE(small_converter->local_enu_to_wgs84(
    Eigen::Vector3d(101.0, 0.0, 0.0)).has_value());

  const auto far_fix = large_converter->local_enu_to_wgs84(
    Eigen::Vector3d(200.0, 0.0, 0.0));
  ASSERT_TRUE(far_fix.has_value());
  EXPECT_FALSE(small_converter->wgs84_to_local_enu(*far_fix).has_value());
}

TEST(GeodeticConverterTest, ExposesConfiguredOriginAndRange)
{
  const Wgs84Position origin = make_origin();
  const auto converter = GeodeticConverter::create(origin, 4321.0);
  ASSERT_TRUE(converter.has_value());

  EXPECT_DOUBLE_EQ(converter->origin().latitude_deg, origin.latitude_deg);
  EXPECT_DOUBLE_EQ(converter->origin().longitude_deg, origin.longitude_deg);
  EXPECT_DOUBLE_EQ(converter->origin().altitude_m, origin.altitude_m);
  EXPECT_DOUBLE_EQ(converter->max_distance_m(), 4321.0);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
