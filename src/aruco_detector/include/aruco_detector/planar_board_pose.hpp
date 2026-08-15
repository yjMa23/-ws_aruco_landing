// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_DETECTOR__PLANAR_BOARD_POSE_HPP_
#define ARUCO_DETECTOR__PLANAR_BOARD_POSE_HPP_

#include "aruco_detector/board_marker_calibration.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace aruco_detector
{

/**
 * @brief 共面多 Marker 联合 PnP 得到的 deck frame 相对 camera_optical 位姿。
 *
 * 位姿满足 `p_camera = R_camera_deck p_deck + t_camera_deck`。
 */
struct PlanarBoardPose
{
  cv::Vec3d rvec_camera_deck;
  cv::Vec3d tvec_camera_deck;
  double reprojection_rmse_px{0.0};
  std::size_t marker_count{0U};
  std::string selection_reason;
};

/**
 * @brief 检查 Board calibration 是否有限、ID 唯一且所有角点张成同一二维平面。
 *
 * @param calibrations Marker 尺寸及 deck-frame 刚体标定。
 * @return 至少两个有效 Marker、二维基线非退化且整体严格共面时返回 true。
 */
bool is_valid_planar_board_calibration(
  const std::vector<BoardMarkerCalibration> & calibrations);

/**
 * @brief 统计本帧角点完整且有限的已标定 Board Marker 数量。
 *
 * 未知 ID 被忽略；重复检测 ID、角点数量错误或 NaN/Inf 角点不会计入。
 */
std::size_t count_valid_board_markers(
  const std::vector<int> & detected_ids,
  const std::vector<std::vector<cv::Point2f>> & detected_corners,
  const std::vector<BoardMarkerCalibration> & calibrations);

/**
 * @brief 使用多个共面 Marker 的全部可见角点通过 IPPE 估计统一 deck 位姿。
 *
 * 函数允许 object points 严格共面，至少需要两个有效 Board Marker。使用
 * `cv::solvePnPGeneric(..., cv::SOLVEPNP_IPPE)` 获取平面双解，再按有限性、全部点
 * 正深度、deck +z 法向朝向相机、重投影 RMSE 和可选上一帧视觉位姿连续性消歧。
 * 选出合法最佳 IPPE 候选后，使用同一帧全部可见 Board corners 和该候选初值执行
 * `cv::solvePnPRefineLM`；精化结果只有在重新通过物理约束、既有重投影 hard gate 且
 * RMSE 不变差时才采用，否则安全回退到原 IPPE 候选。Ground Truth、运动模型相位和
 * 未来轨迹均不参与。
 *
 * @param detected_ids 本帧 ArUco ID。
 * @param detected_corners 与 ID 对应的左上、右上、右下、左下四角点。
 * @param calibrations 共面 Board 标定，坐标均在 deck frame 中。
 * @param camera_matrix 3x3 相机内参矩阵。
 * @param distortion_coefficients OpenCV 畸变系数，可为空。
 * @param previous_pose 可选上一帧有效 planar board pose，只用于候选消歧。
 * @param max_reprojection_rmse_px 最大允许像素 RMSE，必须为正且有限。
 * @return 成功时返回统一 deck pose；输入非法、可见 Marker 少于两个、IPPE 无合法
 * 候选、法向翻转或 RMSE 超限时返回 std::nullopt。
 */
std::optional<PlanarBoardPose> estimate_planar_board_pose(
  const std::vector<int> & detected_ids,
  const std::vector<std::vector<cv::Point2f>> & detected_corners,
  const std::vector<BoardMarkerCalibration> & calibrations,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients,
  const std::optional<PlanarBoardPose> & previous_pose = std::nullopt,
  double max_reprojection_rmse_px = 5.0);

/**
 * @brief 将单 Marker 相机位姿转换为统一 deck center 相机位姿。
 *
 * 输入 Marker pose 满足 `p_camera = R_camera_marker p_marker + t_camera_marker`，
 * calibration 提供 `T_deck_marker`。函数返回
 * `T_camera_deck = T_camera_marker * inverse(T_deck_marker)`。
 *
 * @return 输入 calibration/pose 非有限、Marker size 非正或结果深度非正时返回空。
 */
std::optional<PlanarBoardPose> deck_pose_from_single_marker(
  const BoardMarkerCalibration & calibration,
  const cv::Vec3d & rvec_camera_marker,
  const cv::Vec3d & tvec_camera_marker);

}  // namespace aruco_detector

#endif  // ARUCO_DETECTOR__PLANAR_BOARD_POSE_HPP_
