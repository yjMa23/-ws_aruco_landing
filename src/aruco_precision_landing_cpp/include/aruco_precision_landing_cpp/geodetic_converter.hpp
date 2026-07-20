// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__GEODETIC_CONVERTER_HPP_
#define ARUCO_PRECISION_LANDING_CPP__GEODETIC_CONVERTER_HPP_

#include <optional>

#include <Eigen/Core>

namespace aruco_precision_landing_cpp
{

/**
 * @brief WGS84 大地坐标。
 */
struct Wgs84Position
{
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
};

/**
 * @brief 在固定 WGS84 参考点附近执行 WGS84、ECEF 和局部 ENU 转换。
 *
 * 转换使用 WGS84 椭球和 ECEF 中间坐标，不使用固定经纬度比例近似。实例仅用于参考点
 * 附近的局部会合；超过构造时配置的最大三维距离会返回失败。
 */
class GeodeticConverter
{
public:
  /**
   * @brief 创建局部地理坐标转换器。
   *
   * @param origin_wgs84 局部 ENU 原点，纬度和经度单位为度，海拔单位为米。
   * @param max_distance_m 允许转换的原点附近最大三维距离，单位为米，必须为有限正数。
   * @return 有效转换器；原点非法或距离限制非法时返回 `std::nullopt`。
   */
  static std::optional<GeodeticConverter> create(
    const Wgs84Position & origin_wgs84,
    double max_distance_m = 10000.0);

  /**
   * @brief 将 WGS84 坐标转换为以构造原点为中心的局部 ENU。
   *
   * @param position_wgs84 目标 WGS84 坐标，纬度和经度单位为度，海拔单位为米。
   * @return ENU `[East, North, Up]`，单位为米；输入非法或距离超过限制时返回
   *         `std::nullopt`。
   */
  std::optional<Eigen::Vector3d> wgs84_to_local_enu(
    const Wgs84Position & position_wgs84) const;

  /**
   * @brief 将局部 ENU 转换回 WGS84。
   *
   * @param position_enu 局部 ENU `[East, North, Up]`，单位为米。
   * @return WGS84 坐标；输入含 NaN/Inf、距离超过限制或数值求解失败时返回
   *         `std::nullopt`。
   */
  std::optional<Wgs84Position> local_enu_to_wgs84(
    const Eigen::Vector3d & position_enu) const;

  /**
   * @brief 返回转换器使用的 WGS84 局部原点。
   *
   * @return 纬度和经度单位为度、海拔单位为米的原点。
   */
  const Wgs84Position & origin() const;

  /**
   * @brief 返回允许转换的最大三维距离。
   *
   * @return 距离上限，单位为米。
   */
  double max_distance_m() const;

private:
  GeodeticConverter(
    const Wgs84Position & origin_wgs84,
    double max_distance_m,
    const Eigen::Vector3d & origin_ecef,
    const Eigen::Matrix3d & ecef_to_enu);

  Wgs84Position origin_wgs84_;
  double max_distance_m_{10000.0};
  Eigen::Vector3d origin_ecef_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d ecef_to_enu_{Eigen::Matrix3d::Identity()};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__GEODETIC_CONVERTER_HPP_
