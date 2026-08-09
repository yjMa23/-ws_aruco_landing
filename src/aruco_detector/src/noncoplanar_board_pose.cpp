// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/noncoplanar_board_pose.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace aruco_detector
{
namespace
{

bool finite(double value)
{
  return std::isfinite(value);
}

cv::Matx33d rotationFromRpy(const std::array<double, 3> & rpy)
{
  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);
  return cv::Matx33d(
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr);
}

bool markerCornersDeck(
  const BoardMarkerCalibration & calibration,
  std::vector<cv::Point3f> & object_points)
{
  if (calibration.id < 0 || !finite(calibration.length_m) || calibration.length_m <= 0.0 ||
    !std::all_of(
      calibration.center_deck_m.begin(), calibration.center_deck_m.end(), finite) ||
    !std::all_of(
      calibration.rpy_deck_marker_rad.begin(), calibration.rpy_deck_marker_rad.end(), finite))
  {
    return false;
  }

  const double half = calibration.length_m * 0.5;
  const std::array<cv::Vec3d, 4> marker_corners{{
      {-half, half, 0.0},
      {half, half, 0.0},
      {half, -half, 0.0},
      {-half, -half, 0.0},
    }};
  const cv::Matx33d rotation_deck_marker = rotationFromRpy(
    calibration.rpy_deck_marker_rad);
  const cv::Vec3d center_deck{
    calibration.center_deck_m[0],
    calibration.center_deck_m[1],
    calibration.center_deck_m[2]};
  for (const auto & marker_corner : marker_corners) {
    const cv::Vec3d corner_deck = rotation_deck_marker * marker_corner + center_deck;
    object_points.emplace_back(corner_deck);
  }
  return true;
}

bool noncoplanar(const std::vector<cv::Point3f> & points)
{
  if (points.size() < 6U) {
    return false;
  }

  cv::Vec3d centroid{0.0, 0.0, 0.0};
  for (const auto & point : points) {
    centroid += cv::Vec3d(point.x, point.y, point.z);
  }
  centroid *= 1.0 / static_cast<double>(points.size());

  cv::Mat centered(static_cast<int>(points.size()), 3, CV_64F);
  for (std::size_t row = 0; row < points.size(); ++row) {
    centered.at<double>(static_cast<int>(row), 0) = points[row].x - centroid[0];
    centered.at<double>(static_cast<int>(row), 1) = points[row].y - centroid[1];
    centered.at<double>(static_cast<int>(row), 2) = points[row].z - centroid[2];
  }
  cv::Mat singular_values;
  cv::SVD::compute(centered, singular_values);
  return singular_values.rows >= 3 &&
         singular_values.at<double>(2, 0) > singular_values.at<double>(0, 0) * 1e-6;
}

bool validCameraModel(const cv::Mat & camera_matrix, const cv::Mat & distortion)
{
  if (camera_matrix.rows != 3 || camera_matrix.cols != 3 || camera_matrix.channels() != 1) {
    return false;
  }
  cv::Mat camera_matrix_64f;
  camera_matrix.convertTo(camera_matrix_64f, CV_64F);
  if (!cv::checkRange(camera_matrix_64f) || camera_matrix_64f.at<double>(0, 0) <= 0.0 ||
    camera_matrix_64f.at<double>(1, 1) <= 0.0 || camera_matrix_64f.at<double>(2, 2) == 0.0)
  {
    return false;
  }
  if (!distortion.empty()) {
    cv::Mat distortion_64f;
    distortion.convertTo(distortion_64f, CV_64F);
    if (!cv::checkRange(distortion_64f)) {
      return false;
    }
  }
  return true;
}

bool finitePose(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  return finite(rvec[0]) && finite(rvec[1]) && finite(rvec[2]) &&
         finite(tvec[0]) && finite(tvec[1]) && finite(tvec[2]) && tvec[2] > 0.0;
}

}  // namespace

bool is_valid_noncoplanar_board_calibration(
  const std::vector<BoardMarkerCalibration> & calibrations)
{
  if (calibrations.size() < 2U) {
    return false;
  }
  std::unordered_set<int> ids;
  std::vector<cv::Point3f> object_points;
  object_points.reserve(calibrations.size() * 4U);
  for (const auto & calibration : calibrations) {
    if (!ids.insert(calibration.id).second ||
      !markerCornersDeck(calibration, object_points))
    {
      return false;
    }
  }
  return noncoplanar(object_points);
}

std::optional<NoncoplanarBoardPose> estimate_noncoplanar_board_pose(
  const std::vector<int> & detected_ids,
  const std::vector<std::vector<cv::Point2f>> & detected_corners,
  const std::vector<BoardMarkerCalibration> & calibrations,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients)
{
  if (detected_ids.size() != detected_corners.size() ||
    !is_valid_noncoplanar_board_calibration(calibrations) ||
    !validCameraModel(camera_matrix, distortion_coefficients))
  {
    return std::nullopt;
  }

  std::unordered_set<int> calibration_ids;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  object_points.reserve(calibrations.size() * 4U);
  image_points.reserve(calibrations.size() * 4U);
  std::size_t visible_marker_count = 0U;
  for (const auto & calibration : calibrations) {
    if (!calibration_ids.insert(calibration.id).second) {
      return std::nullopt;
    }
    const auto detected = std::find(detected_ids.begin(), detected_ids.end(), calibration.id);
    if (detected == detected_ids.end()) {
      continue;
    }
    const auto index = static_cast<std::size_t>(std::distance(detected_ids.begin(), detected));
    const auto & corners = detected_corners[index];
    if (corners.size() != 4U || !std::all_of(
        corners.begin(), corners.end(), [](const cv::Point2f & corner) {
          return finite(corner.x) && finite(corner.y);
        }) || !markerCornersDeck(calibration, object_points))
    {
      return std::nullopt;
    }
    image_points.insert(image_points.end(), corners.begin(), corners.end());
    ++visible_marker_count;
  }
  if (!noncoplanar(object_points)) {
    return std::nullopt;
  }

  cv::Vec3d rvec;
  cv::Vec3d tvec;
  try {
    if (!cv::solvePnP(
        object_points, image_points, camera_matrix, distortion_coefficients,
        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE) || !finitePose(rvec, tvec))
    {
      return std::nullopt;
    }

    std::vector<cv::Point2f> reprojected;
    cv::projectPoints(
      object_points, rvec, tvec, camera_matrix, distortion_coefficients, reprojected);
    double squared_error = 0.0;
    for (std::size_t index = 0; index < image_points.size(); ++index) {
      const cv::Point2f residual = reprojected[index] - image_points[index];
      squared_error += static_cast<double>(residual.dot(residual));
    }
    const double rmse = std::sqrt(squared_error / static_cast<double>(image_points.size()));
    if (!finite(rmse)) {
      return std::nullopt;
    }
    return NoncoplanarBoardPose{rvec, tvec, rmse, visible_marker_count};
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
}

}  // namespace aruco_detector
