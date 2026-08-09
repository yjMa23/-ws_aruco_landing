// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/noncoplanar_board_pose.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

namespace aruco_detector
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::vector<BoardMarkerCalibration> calibrations()
{
  return {
    BoardMarkerCalibration{0, 0.50, {0.45, 0.0, 0.0}, {0.0, 0.0, 0.0}},
    BoardMarkerCalibration{4, 0.75, {-0.75, 0.0, 0.2851650429449553},
      {0.0, 45.0 * kPi / 180.0, 0.0}},
    BoardMarkerCalibration{5, 0.75, {0.0, 0.75, 0.2851650429449553},
      {45.0 * kPi / 180.0, 0.0, 0.0}},
    BoardMarkerCalibration{6, 0.75, {0.0, -0.75, 0.2851650429449553},
      {-45.0 * kPi / 180.0, 0.0, 0.0}},
  };
}

cv::Mat cameraMatrix()
{
  return (cv::Mat_<double>(3, 3) <<
         520.0, 0.0, 320.0,
         0.0, 520.0, 240.0,
         0.0, 0.0, 1.0);
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

std::vector<cv::Point3f> markerCorners(const BoardMarkerCalibration & calibration)
{
  const float half = static_cast<float>(calibration.length_m * 0.5);
  const std::array<cv::Vec3d, 4> local_corners{{
      {-half, half, 0.0},
      {half, half, 0.0},
      {half, -half, 0.0},
      {-half, -half, 0.0},
    }};
  const cv::Matx33d rotation = rotationFromRpy(calibration.rpy_deck_marker_rad);

  std::vector<cv::Point3f> result;
  result.reserve(local_corners.size());
  for (const auto & local_corner : local_corners) {
    const cv::Vec3d deck_corner = rotation * local_corner + cv::Vec3d(
      calibration.center_deck_m[0],
      calibration.center_deck_m[1],
      calibration.center_deck_m[2]);
    result.emplace_back(deck_corner);
  }
  return result;
}

std::vector<std::vector<cv::Point2f>> projectedCorners(
  const std::vector<BoardMarkerCalibration> & board_calibrations,
  const cv::Vec3d & rvec_camera_deck,
  const cv::Vec3d & tvec_camera_deck)
{
  std::vector<std::vector<cv::Point2f>> result;
  result.reserve(board_calibrations.size());
  for (const auto & calibration : board_calibrations) {
    std::vector<cv::Point2f> image_corners;
    cv::projectPoints(
      markerCorners(calibration), rvec_camera_deck, tvec_camera_deck,
      cameraMatrix(), cv::Mat::zeros(1, 5, CV_64F), image_corners);
    result.push_back(std::move(image_corners));
  }
  return result;
}

double rotationErrorRad(const cv::Vec3d & lhs, const cv::Vec3d & rhs)
{
  cv::Mat lhs_rotation;
  cv::Mat rhs_rotation;
  cv::Rodrigues(lhs, lhs_rotation);
  cv::Rodrigues(rhs, rhs_rotation);
  cv::Mat error_rotation = lhs_rotation * rhs_rotation.t();
  cv::Vec3d error_vector;
  cv::Rodrigues(error_rotation, error_vector);
  return cv::norm(error_vector);
}

TEST(NoncoplanarBoardPoseTest, RecoversKnownDeckPose)
{
  const auto board_calibrations = calibrations();
  ASSERT_TRUE(is_valid_noncoplanar_board_calibration(board_calibrations));
  const cv::Vec3d expected_rvec{0.10, -0.05, 0.20};
  const cv::Vec3d expected_tvec{0.20, -0.10, 5.0};

  const auto estimate = estimate_noncoplanar_board_pose(
    {0, 4, 5, 6}, projectedCorners(board_calibrations, expected_rvec, expected_tvec),
    board_calibrations, cameraMatrix(), cv::Mat::zeros(1, 5, CV_64F));

  ASSERT_TRUE(estimate.has_value());
  EXPECT_LT(cv::norm(estimate->tvec_camera_deck - expected_tvec), 1e-5);
  EXPECT_LT(rotationErrorRad(estimate->rvec_camera_deck, expected_rvec), 1e-5);
  EXPECT_LT(estimate->reprojection_rmse_px, 1e-4);
  EXPECT_EQ(estimate->marker_count, 4U);
}

TEST(NoncoplanarBoardPoseTest, RejectsIncompleteOrInvalidInputs)
{
  const auto board_calibrations = calibrations();
  const cv::Vec3d rvec{0.10, -0.05, 0.20};
  const cv::Vec3d tvec{0.20, -0.10, 5.0};
  const auto projected = projectedCorners(board_calibrations, rvec, tvec);
  const cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64F);

  const auto partial_estimate = estimate_noncoplanar_board_pose(
      {0, 4, 5}, {projected[0], projected[1], projected[2]}, board_calibrations,
      cameraMatrix(), distortion);
  ASSERT_TRUE(partial_estimate.has_value());
  EXPECT_EQ(partial_estimate->marker_count, 3U);

  EXPECT_FALSE(estimate_noncoplanar_board_pose(
      {0}, {projected[0]}, board_calibrations, cameraMatrix(), distortion).has_value());

  auto coplanar_calibrations = board_calibrations;
  for (auto & calibration : coplanar_calibrations) {
    calibration.center_deck_m[2] = 0.0;
    calibration.rpy_deck_marker_rad = {0.0, 0.0, 0.0};
  }
  EXPECT_FALSE(is_valid_noncoplanar_board_calibration(coplanar_calibrations));
  EXPECT_FALSE(estimate_noncoplanar_board_pose(
      {0, 4}, projected, coplanar_calibrations, cameraMatrix(), distortion).has_value());

  auto non_finite_corners = projected;
  non_finite_corners[1][0].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(estimate_noncoplanar_board_pose(
      {0, 4, 5, 6}, non_finite_corners, board_calibrations,
      cameraMatrix(), distortion).has_value());
}

}  // namespace
}  // namespace aruco_detector
