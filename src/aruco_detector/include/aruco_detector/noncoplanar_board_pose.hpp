// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_DETECTOR__NONCOPLANAR_BOARD_POSE_HPP_
#define ARUCO_DETECTOR__NONCOPLANAR_BOARD_POSE_HPP_

#include "aruco_detector/board_marker_calibration.hpp"

#include <cstddef>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace aruco_detector
{

/**
 * @brief 非共面角点联合 PnP 得到的甲板坐标系相对相机位姿。
 */
struct NoncoplanarBoardPose
{
  cv::Vec3d rvec_camera_deck;
  cv::Vec3d tvec_camera_deck;
  double reprojection_rmse_px{0.0};
  std::size_t marker_count{0U};
};

/**
 * @brief 检查 Marker 标定是否有限、ID 唯一且角点集合确实非共面。
 *
 * @param calibrations Marker 尺寸及甲板系刚体标定。
 * @return 至少包含两个有效 Marker，且其全部角点张成三维空间时返回 true。
 */
bool is_valid_noncoplanar_board_calibration(
  const std::vector<BoardMarkerCalibration> & calibrations);

/**
 * @brief 使用多个已标定 Marker 的非共面角点联合估计甲板位姿。
 *
 * 每个 Marker 的图像角点须采用 OpenCV ArUco 的左上、右上、右下、左下顺序。
 * 函数将所有可见且已标定的角点转换到统一甲板坐标系，确认三维点集非共面后调用
 * solvePnP，输出的旋转和平移满足 `p_camera = R_camera_deck p_deck + t`。
 *
 * @param detected_ids 本帧检测到的 Marker ID。
 * @param detected_corners 与 ID 一一对应的四个图像角点，单位为像素。
 * @param calibrations 至少两个 Marker 的尺寸及甲板系刚体标定。
 * @param camera_matrix 3x3 相机内参矩阵。
 * @param distortion_coefficients OpenCV 畸变系数，可为空。
 * @return 成功时返回甲板位姿、参与解算的 Marker 数及重投影 RMSE；可见已标定
 * 角点不足以形成非共面集合、输入非法或 PnP 失败时返回 std::nullopt。
 */
std::optional<NoncoplanarBoardPose> estimate_noncoplanar_board_pose(
  const std::vector<int> & detected_ids,
  const std::vector<std::vector<cv::Point2f>> & detected_corners,
  const std::vector<BoardMarkerCalibration> & calibrations,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients);

}  // namespace aruco_detector

#endif  // ARUCO_DETECTOR__NONCOPLANAR_BOARD_POSE_HPP_
