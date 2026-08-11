// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_DETECTOR__BOARD_MARKER_CALIBRATION_HPP_
#define ARUCO_DETECTOR__BOARD_MARKER_CALIBRATION_HPP_

#include <array>

namespace aruco_detector
{

/**
 * @brief 单个 ArUco Marker 在统一甲板坐标系中的已知刚体标定。
 *
 * center_deck_m 是 Marker 几何中心在 deck frame 中的位置，单位 m；
 * rpy_deck_marker_rad 定义 Marker frame 到 deck frame 的固定旋转，单位 rad。
 */
struct BoardMarkerCalibration
{
  int id{0};
  double length_m{0.0};
  std::array<double, 3> center_deck_m{{0.0, 0.0, 0.0}};
  std::array<double, 3> rpy_deck_marker_rad{{0.0, 0.0, 0.0}};
};

}  // namespace aruco_detector

#endif  // ARUCO_DETECTOR__BOARD_MARKER_CALIBRATION_HPP_
