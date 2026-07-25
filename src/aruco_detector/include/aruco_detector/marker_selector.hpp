// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_DETECTOR__MARKER_SELECTOR_HPP_
#define ARUCO_DETECTOR__MARKER_SELECTOR_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace aruco_detector
{

/**
 * @brief 一个允许检测的 Marker 配置。
 */
struct MarkerConfiguration
{
  int id{0};
  double length_m{0.0};
  int priority{0};
  std::array<double, 3> target_offset_marker_m{{0.0, 0.0, 0.0}};
};

/**
 * @brief 图像中一个已检测 Marker 的选择信息。
 */
struct MarkerDetectionCandidate
{
  int id{0};
  double corner_area_px2{0.0};
  std::size_t detection_index{0U};
};

/**
 * @brief 已选 Marker 及其物理配置。
 */
struct SelectedMarker
{
  MarkerConfiguration configuration;
  std::size_t detection_index{0U};
  double corner_area_px2{0.0};
};

/**
 * @brief 校验 Marker ID、边长和优先级配置。
 *
 * @throws std::invalid_argument 配置为空、ID 重复、边长非法或优先级为负。
 */
void validate_marker_configurations(
  const std::vector<MarkerConfiguration> & configurations);

/**
 * @brief 按优先级和图像面积选择当前使用的 Marker。
 *
 * 优先级数值越小越优先；同优先级选择角点多边形面积更大的 Marker。未配置 ID、
 * 非有限面积和非正面积会被忽略。
 */
std::optional<SelectedMarker> select_marker(
  const std::vector<MarkerConfiguration> & configurations,
  const std::vector<MarkerDetectionCandidate> & candidates);

}  // namespace aruco_detector

#endif  // ARUCO_DETECTOR__MARKER_SELECTOR_HPP_
