// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__TIME_SOURCE_GUARD_HPP_
#define ARUCO_PRECISION_LANDING_CPP__TIME_SOURCE_GUARD_HPP_

#include <cmath>
#include <optional>

#include <rclcpp/time.hpp>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 仅在两个时间戳使用同一时钟源时计算时间差。
 *
 * ROS 2 启用 `use_sim_time` 时，节点启动早期可能先收到 SYSTEM_TIME，随后切换为
 * ROS_TIME。直接对不同来源的 `rclcpp::Time` 做减法会抛出异常并终止控制节点。
 *
 * @param later 较新的时间戳。
 * @param earlier 较旧的时间戳。
 * @return `later - earlier` 的秒数；时钟源不同或结果非有限值时返回空。
 */
inline std::optional<double> elapsed_seconds_same_clock(
  const rclcpp::Time & later,
  const rclcpp::Time & earlier) noexcept
{
  if (later.get_clock_type() != earlier.get_clock_type()) {
    return std::nullopt;
  }

  const double elapsed_s = (later - earlier).seconds();
  if (!std::isfinite(elapsed_s)) {
    return std::nullopt;
  }
  return elapsed_s;
}

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__TIME_SOURCE_GUARD_HPP_
