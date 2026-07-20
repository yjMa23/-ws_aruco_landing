// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/geodetic_converter.hpp"

#include <cmath>

namespace aruco_precision_landing_cpp
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84InverseFlattening = 298.257223563;
constexpr double kPoleToleranceM = 1.0e-9;

constexpr double kWgs84Flattening = 1.0 / kWgs84InverseFlattening;
constexpr double kWgs84SemiMinorAxisM =
  kWgs84SemiMajorAxisM * (1.0 - kWgs84Flattening);
constexpr double kWgs84FirstEccentricitySquared =
  kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kWgs84SecondEccentricitySquared =
  (kWgs84SemiMajorAxisM * kWgs84SemiMajorAxisM -
  kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM) /
  (kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM);

bool wgs84_is_valid(const Wgs84Position & position)
{
  return std::isfinite(position.latitude_deg) &&
         std::isfinite(position.longitude_deg) &&
         std::isfinite(position.altitude_m) &&
         position.latitude_deg >= -90.0 &&
         position.latitude_deg <= 90.0 &&
         position.longitude_deg >= -180.0 &&
         position.longitude_deg <= 180.0;
}

std::optional<Eigen::Vector3d> wgs84_to_ecef(const Wgs84Position & position)
{
  if (!wgs84_is_valid(position)) {
    return std::nullopt;
  }

  const double latitude_rad = position.latitude_deg * kDegreesToRadians;
  const double longitude_rad = position.longitude_deg * kDegreesToRadians;
  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double sin_longitude = std::sin(longitude_rad);
  const double cos_longitude = std::cos(longitude_rad);
  const double prime_vertical_radius = kWgs84SemiMajorAxisM /
    std::sqrt(1.0 - kWgs84FirstEccentricitySquared * sin_latitude * sin_latitude);

  Eigen::Vector3d ecef;
  ecef.x() = (prime_vertical_radius + position.altitude_m) *
    cos_latitude * cos_longitude;
  ecef.y() = (prime_vertical_radius + position.altitude_m) *
    cos_latitude * sin_longitude;
  ecef.z() =
    (prime_vertical_radius * (1.0 - kWgs84FirstEccentricitySquared) +
    position.altitude_m) * sin_latitude;

  if (!ecef.allFinite()) {
    return std::nullopt;
  }
  return ecef;
}

std::optional<Wgs84Position> ecef_to_wgs84(const Eigen::Vector3d & ecef)
{
  if (!ecef.allFinite()) {
    return std::nullopt;
  }

  const double horizontal_radius = std::hypot(ecef.x(), ecef.y());
  Wgs84Position position;

  if (horizontal_radius <= kPoleToleranceM) {
    position.latitude_deg = ecef.z() >= 0.0 ? 90.0 : -90.0;
    position.longitude_deg = 0.0;
    position.altitude_m = std::abs(ecef.z()) - kWgs84SemiMinorAxisM;
    return wgs84_is_valid(position) ? std::optional<Wgs84Position>(position) : std::nullopt;
  }

  const double longitude_rad = std::atan2(ecef.y(), ecef.x());
  const double theta = std::atan2(
    ecef.z() * kWgs84SemiMajorAxisM,
    horizontal_radius * kWgs84SemiMinorAxisM);
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double latitude_rad = std::atan2(
    ecef.z() + kWgs84SecondEccentricitySquared * kWgs84SemiMinorAxisM *
    sin_theta * sin_theta * sin_theta,
    horizontal_radius - kWgs84FirstEccentricitySquared * kWgs84SemiMajorAxisM *
    cos_theta * cos_theta * cos_theta);

  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double prime_vertical_radius = kWgs84SemiMajorAxisM /
    std::sqrt(1.0 - kWgs84FirstEccentricitySquared * sin_latitude * sin_latitude);

  position.latitude_deg = latitude_rad * kRadiansToDegrees;
  position.longitude_deg = longitude_rad * kRadiansToDegrees;
  position.altitude_m = horizontal_radius / cos_latitude - prime_vertical_radius;

  return wgs84_is_valid(position) ? std::optional<Wgs84Position>(position) : std::nullopt;
}

}  // namespace

std::optional<GeodeticConverter> GeodeticConverter::create(
  const Wgs84Position & origin_wgs84,
  double max_distance_m)
{
  if (!wgs84_is_valid(origin_wgs84) ||
    !std::isfinite(max_distance_m) || max_distance_m <= 0.0)
  {
    return std::nullopt;
  }

  const auto origin_ecef = wgs84_to_ecef(origin_wgs84);
  if (!origin_ecef) {
    return std::nullopt;
  }

  const double latitude_rad = origin_wgs84.latitude_deg * kDegreesToRadians;
  const double longitude_rad = origin_wgs84.longitude_deg * kDegreesToRadians;
  const double sin_latitude = std::sin(latitude_rad);
  const double cos_latitude = std::cos(latitude_rad);
  const double sin_longitude = std::sin(longitude_rad);
  const double cos_longitude = std::cos(longitude_rad);

  Eigen::Matrix3d ecef_to_enu;
  ecef_to_enu <<
    -sin_longitude, cos_longitude, 0.0,
    -sin_latitude * cos_longitude, -sin_latitude * sin_longitude, cos_latitude,
    cos_latitude * cos_longitude, cos_latitude * sin_longitude, sin_latitude;

  return GeodeticConverter(origin_wgs84, max_distance_m, *origin_ecef, ecef_to_enu);
}

GeodeticConverter::GeodeticConverter(
  const Wgs84Position & origin_wgs84,
  double max_distance_m,
  const Eigen::Vector3d & origin_ecef,
  const Eigen::Matrix3d & ecef_to_enu)
: origin_wgs84_(origin_wgs84),
  max_distance_m_(max_distance_m),
  origin_ecef_(origin_ecef),
  ecef_to_enu_(ecef_to_enu)
{
}

std::optional<Eigen::Vector3d> GeodeticConverter::wgs84_to_local_enu(
  const Wgs84Position & position_wgs84) const
{
  const auto position_ecef = wgs84_to_ecef(position_wgs84);
  if (!position_ecef) {
    return std::nullopt;
  }

  const Eigen::Vector3d position_enu = ecef_to_enu_ * (*position_ecef - origin_ecef_);
  if (!position_enu.allFinite() || position_enu.norm() > max_distance_m_) {
    return std::nullopt;
  }
  return position_enu;
}

std::optional<Wgs84Position> GeodeticConverter::local_enu_to_wgs84(
  const Eigen::Vector3d & position_enu) const
{
  if (!position_enu.allFinite() || position_enu.norm() > max_distance_m_) {
    return std::nullopt;
  }

  const Eigen::Vector3d position_ecef = origin_ecef_ + ecef_to_enu_.transpose() * position_enu;
  return ecef_to_wgs84(position_ecef);
}

const Wgs84Position & GeodeticConverter::origin() const
{
  return origin_wgs84_;
}

double GeodeticConverter::max_distance_m() const
{
  return max_distance_m_;
}

}  // namespace aruco_precision_landing_cpp
