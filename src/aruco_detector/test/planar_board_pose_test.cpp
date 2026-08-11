// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/planar_board_pose.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace aruco_detector
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::vector<BoardMarkerCalibration> marineCalibrations()
{
  return {
    {4, 0.50, {0.78, 0.78, 0.002}, {0.0, 0.0, 0.0}},
    {5, 0.50, {0.78, -0.78, 0.002}, {0.0, 0.0, 0.0}},
    {6, 0.50, {-0.78, 0.78, 0.002}, {0.0, 0.0, 0.0}},
    {7, 0.50, {-0.78, -0.78, 0.002}, {0.0, 0.0, 0.0}},
  };
}

cv::Mat cameraMatrix()
{
  return (cv::Mat_<double>(3, 3) <<
         720.0, 0.0, 640.0,
         0.0, 720.0, 360.0,
         0.0, 0.0, 1.0);
}

cv::Mat zeroDistortion()
{
  return cv::Mat::zeros(1, 5, CV_64F);
}

cv::Matx33d rotationX(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return cv::Matx33d(
    1.0, 0.0, 0.0,
    0.0, c, -s,
    0.0, s, c);
}

cv::Matx33d rotationY(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return cv::Matx33d(
    c, 0.0, s,
    0.0, 1.0, 0.0,
    -s, 0.0, c);
}

cv::Matx33d rotationZ(double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return cv::Matx33d(
    c, -s, 0.0,
    s, c, 0.0,
    0.0, 0.0, 1.0);
}

cv::Vec3d cameraDeckRvec(double roll, double pitch, double yaw)
{
  // Fronto-parallel down-looking camera: deck +z points back toward camera_optical -z.
  const cv::Matx33d camera_deck_fronto(
    1.0, 0.0, 0.0,
    0.0, -1.0, 0.0,
    0.0, 0.0, -1.0);
  const cv::Matx33d rotation =
    camera_deck_fronto * rotationZ(yaw) * rotationY(pitch) * rotationX(roll);
  cv::Vec3d rvec;
  cv::Rodrigues(cv::Mat(rotation), rvec);
  return rvec;
}

std::vector<cv::Point3f> markerCornersDeck(const BoardMarkerCalibration & calibration)
{
  const float half = static_cast<float>(calibration.length_m * 0.5);
  const float x = static_cast<float>(calibration.center_deck_m[0]);
  const float y = static_cast<float>(calibration.center_deck_m[1]);
  const float z = static_cast<float>(calibration.center_deck_m[2]);
  return {
    {x - half, y + half, z},
    {x + half, y + half, z},
    {x + half, y - half, z},
    {x - half, y - half, z},
  };
}

struct SyntheticDetections
{
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
};

SyntheticDetections projectMarkers(
  const std::vector<BoardMarkerCalibration> & calibrations,
  const std::vector<int> & ids,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec)
{
  SyntheticDetections detections;
  detections.ids = ids;
  for (const int id : ids) {
    const auto calibration = std::find_if(
      calibrations.begin(), calibrations.end(),
      [id](const BoardMarkerCalibration & candidate) {return candidate.id == id;});
    EXPECT_NE(calibration, calibrations.end());
    if (calibration == calibrations.end()) {
      detections.corners.emplace_back();
      continue;
    }
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
      markerCornersDeck(*calibration), rvec, tvec,
      cameraMatrix(), zeroDistortion(), image_points);
    detections.corners.push_back(image_points);
  }
  return detections;
}

double rotationErrorRad(const cv::Vec3d & estimated, const cv::Vec3d & expected)
{
  cv::Mat estimated_rotation;
  cv::Mat expected_rotation;
  cv::Rodrigues(estimated, estimated_rotation);
  cv::Rodrigues(expected, expected_rotation);
  const cv::Mat delta = estimated_rotation * expected_rotation.t();
  const double cosine = std::clamp((cv::trace(delta)[0] - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

double normalErrorRad(const cv::Vec3d & estimated, const cv::Vec3d & expected)
{
  cv::Mat estimated_rotation;
  cv::Mat expected_rotation;
  cv::Rodrigues(estimated, estimated_rotation);
  cv::Rodrigues(expected, expected_rotation);
  const cv::Vec3d deck_normal(0.0, 0.0, 1.0);
  const cv::Vec3d estimated_normal = cv::Matx33d(estimated_rotation) * deck_normal;
  const cv::Vec3d expected_normal = cv::Matx33d(expected_rotation) * deck_normal;
  const double cosine = std::clamp(estimated_normal.dot(expected_normal), -1.0, 1.0);
  return std::acos(cosine);
}

void expectPoseClose(
  const PlanarBoardPose & pose,
  const cv::Vec3d & expected_rvec,
  const cv::Vec3d & expected_tvec,
  std::size_t expected_marker_count,
  double position_tolerance_m = 2e-3,
  double rotation_tolerance_rad = 2e-3)
{
  EXPECT_EQ(pose.marker_count, expected_marker_count);
  EXPECT_TRUE(std::isfinite(pose.reprojection_rmse_px));
  EXPECT_LT(cv::norm(pose.tvec_camera_deck - expected_tvec), position_tolerance_m);
  EXPECT_LT(rotationErrorRad(pose.rvec_camera_deck, expected_rvec), rotation_tolerance_rad);
  EXPECT_LT(normalErrorRad(pose.rvec_camera_deck, expected_rvec), rotation_tolerance_rad);
  EXPECT_LT(pose.reprojection_rmse_px, 0.05);
}

std::vector<cv::Point3f> allObjectPoints(
  const std::vector<BoardMarkerCalibration> & calibrations)
{
  std::vector<cv::Point3f> object_points;
  for (const auto & calibration : calibrations) {
    const auto corners = markerCornersDeck(calibration);
    object_points.insert(object_points.end(), corners.begin(), corners.end());
  }
  return object_points;
}

std::vector<cv::Point2f> flattenCorners(const SyntheticDetections & detections)
{
  std::vector<cv::Point2f> image_points;
  for (const auto & corners : detections.corners) {
    image_points.insert(image_points.end(), corners.begin(), corners.end());
  }
  return image_points;
}

TEST(PlanarBoardCalibration, AcceptsMarineCoplanarGeometry)
{
  EXPECT_TRUE(is_valid_planar_board_calibration(marineCalibrations()));
}

TEST(PlanarBoardCalibration, RejectsDuplicateId)
{
  auto calibrations = marineCalibrations();
  calibrations[1].id = calibrations[0].id;
  EXPECT_FALSE(is_valid_planar_board_calibration(calibrations));
}

TEST(PlanarBoardCalibration, RejectsNonPositiveMarkerSize)
{
  auto calibrations = marineCalibrations();
  calibrations[2].length_m = -0.50;
  EXPECT_FALSE(is_valid_planar_board_calibration(calibrations));
  calibrations[2].length_m = 0.0;
  EXPECT_FALSE(is_valid_planar_board_calibration(calibrations));
}

TEST(PlanarBoardCalibration, RejectsNanInfAndNonplanarPose)
{
  auto calibrations = marineCalibrations();
  calibrations[0].center_deck_m[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(is_valid_planar_board_calibration(calibrations));

  calibrations = marineCalibrations();
  calibrations[0].rpy_deck_marker_rad[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(is_valid_planar_board_calibration(calibrations));

  calibrations = marineCalibrations();
  calibrations[0].center_deck_m[2] += 0.05;
  EXPECT_FALSE(is_valid_planar_board_calibration(calibrations));
}

TEST(PlanarBoardPose, EstimatesFourMarkerFrontoParallelPose)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_rvec = cameraDeckRvec(0.0, 0.0, 0.0);
  const cv::Vec3d expected_tvec(0.08, -0.04, 5.0);
  const auto detections = projectMarkers(calibrations, {4, 5, 6, 7}, expected_rvec, expected_tvec);

  const auto pose = estimate_planar_board_pose(
    detections.ids, detections.corners, calibrations,
    cameraMatrix(), zeroDistortion());

  ASSERT_TRUE(pose.has_value());
  expectPoseClose(*pose, expected_rvec, expected_tvec, 4U);
}

TEST(PlanarBoardPose, SupportsAnyThreeMarkerVisibility)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_rvec = cameraDeckRvec(0.04, -0.03, 0.08);
  const cv::Vec3d expected_tvec(0.10, 0.06, 5.2);
  const std::array<int, 4> all_ids{{4, 5, 6, 7}};

  for (const int missing_id : all_ids) {
    SCOPED_TRACE(missing_id);
    std::vector<int> visible_ids;
    for (const int id : all_ids) {
      if (id != missing_id) {
        visible_ids.push_back(id);
      }
    }
    const auto detections = projectMarkers(
      calibrations, visible_ids, expected_rvec, expected_tvec);
    const auto pose = estimate_planar_board_pose(
      detections.ids, detections.corners, calibrations,
      cameraMatrix(), zeroDistortion());
    ASSERT_TRUE(pose.has_value());
    expectPoseClose(*pose, expected_rvec, expected_tvec, 3U);
  }
}

TEST(PlanarBoardPose, SupportsAllTwoMarkerCombinations)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_rvec = cameraDeckRvec(-0.05, 0.04, -0.12);
  const cv::Vec3d expected_tvec(-0.06, 0.03, 4.8);
  const std::array<int, 4> all_ids{{4, 5, 6, 7}};

  for (std::size_t first = 0; first < all_ids.size(); ++first) {
    for (std::size_t second = first + 1U; second < all_ids.size(); ++second) {
      SCOPED_TRACE(::testing::Message() << all_ids[first] << "," << all_ids[second]);
      const auto detections = projectMarkers(
        calibrations, {all_ids[first], all_ids[second]}, expected_rvec, expected_tvec);
      const auto pose = estimate_planar_board_pose(
        detections.ids, detections.corners, calibrations,
        cameraMatrix(), zeroDistortion());
      ASSERT_TRUE(pose.has_value());
      expectPoseClose(*pose, expected_rvec, expected_tvec, 2U, 3e-3, 3e-3);
    }
  }
}

TEST(PlanarBoardPose, IsIndependentOfDetectionOrderAndIgnoresUnknownId)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_rvec = cameraDeckRvec(0.03, 0.02, 0.18);
  const cv::Vec3d expected_tvec(0.03, -0.09, 5.1);
  auto detections = projectMarkers(
    calibrations, {7, 4, 6, 5}, expected_rvec, expected_tvec);

  detections.ids.insert(detections.ids.begin() + 2, 42);
  detections.corners.insert(
    detections.corners.begin() + 2,
    std::vector<cv::Point2f>{{10.0F, 10.0F}, {20.0F, 10.0F}, {20.0F, 20.0F}, {10.0F, 20.0F}});

  EXPECT_EQ(count_valid_board_markers(
      detections.ids, detections.corners, calibrations), 4U);
  const auto pose = estimate_planar_board_pose(
    detections.ids, detections.corners, calibrations,
    cameraMatrix(), zeroDistortion());
  ASSERT_TRUE(pose.has_value());
  expectPoseClose(*pose, expected_rvec, expected_tvec, 4U);
}

TEST(PlanarBoardPose, RejectsInvalidCameraModel)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_rvec = cameraDeckRvec(0.0, 0.0, 0.0);
  const cv::Vec3d expected_tvec(0.0, 0.0, 5.0);
  const auto detections = projectMarkers(calibrations, {4, 5}, expected_rvec, expected_tvec);

  cv::Mat bad_camera = cameraMatrix();
  bad_camera.at<double>(0, 0) = 0.0;
  EXPECT_FALSE(estimate_planar_board_pose(
      detections.ids, detections.corners, calibrations,
      bad_camera, zeroDistortion()).has_value());

  bad_camera = cameraMatrix();
  bad_camera.at<double>(1, 1) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(estimate_planar_board_pose(
      detections.ids, detections.corners, calibrations,
      bad_camera, zeroDistortion()).has_value());

  cv::Mat bad_distortion = zeroDistortion();
  bad_distortion.at<double>(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(estimate_planar_board_pose(
      detections.ids, detections.corners, calibrations,
      cameraMatrix(), bad_distortion).has_value());
}

TEST(PlanarBoardPose, RejectsNanInfImagePointsAndInsufficientValidMarkers)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_rvec = cameraDeckRvec(0.0, 0.0, 0.0);
  const cv::Vec3d expected_tvec(0.0, 0.0, 5.0);
  auto detections = projectMarkers(calibrations, {4, 5}, expected_rvec, expected_tvec);

  detections.corners[0][0].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(count_valid_board_markers(
      detections.ids, detections.corners, calibrations), 1U);
  EXPECT_FALSE(estimate_planar_board_pose(
      detections.ids, detections.corners, calibrations,
      cameraMatrix(), zeroDistortion()).has_value());

  detections = projectMarkers(calibrations, {4, 5}, expected_rvec, expected_tvec);
  detections.corners[1][2].y = std::numeric_limits<float>::infinity();
  EXPECT_EQ(count_valid_board_markers(
      detections.ids, detections.corners, calibrations), 1U);
  EXPECT_FALSE(estimate_planar_board_pose(
      detections.ids, detections.corners, calibrations,
      cameraMatrix(), zeroDistortion()).has_value());
}

TEST(PlanarBoardPose, RecoversFrontoRollPitchYawAndCombinedAttitudes)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_tvec(0.07, 0.02, 5.0);
  const std::vector<std::array<double, 3>> attitudes{{
      {0.0, 0.0, 0.0},
      {5.0 * kPi / 180.0, 0.0, 0.0},
      {0.0, -4.0 * kPi / 180.0, 0.0},
      {0.0, 0.0, 12.0 * kPi / 180.0},
      {4.0 * kPi / 180.0, -3.0 * kPi / 180.0, 0.0},
    }};

  for (const auto & attitude : attitudes) {
    SCOPED_TRACE(
      ::testing::Message() << attitude[0] << "," << attitude[1] << "," << attitude[2]);
    const cv::Vec3d expected_rvec = cameraDeckRvec(
      attitude[0], attitude[1], attitude[2]);
    const auto detections = projectMarkers(
      calibrations, {4, 5, 6, 7}, expected_rvec, expected_tvec);
    const auto pose = estimate_planar_board_pose(
      detections.ids, detections.corners, calibrations,
      cameraMatrix(), zeroDistortion());
    ASSERT_TRUE(pose.has_value());
    expectPoseClose(*pose, expected_rvec, expected_tvec, 4U);
  }
}

TEST(PlanarBoardPose, ResolvesIppeAmbiguityWithoutFrameToFrameFlip)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_rvec = cameraDeckRvec(0.002, -0.003, 0.01);
  const cv::Vec3d expected_tvec(0.015, -0.01, 5.0);
  const auto detections = projectMarkers(
    calibrations, {4, 5, 6, 7}, expected_rvec, expected_tvec);

  std::vector<cv::Mat> raw_rvecs;
  std::vector<cv::Mat> raw_tvecs;
  const int raw_solution_count = cv::solvePnPGeneric(
    allObjectPoints(calibrations), flattenCorners(detections),
    cameraMatrix(), zeroDistortion(), raw_rvecs, raw_tvecs,
    false, cv::SOLVEPNP_IPPE);
  ASSERT_GE(raw_solution_count, 2);

  const auto first = estimate_planar_board_pose(
    detections.ids, detections.corners, calibrations,
    cameraMatrix(), zeroDistortion());
  ASSERT_TRUE(first.has_value());
  expectPoseClose(*first, expected_rvec, expected_tvec, 4U);

  auto second_detections = detections;
  // Sub-pixel perturbation simulates adjacent-frame detection noise near the ambiguous fronto-parallel case.
  for (std::size_t marker = 0; marker < second_detections.corners.size(); ++marker) {
    for (std::size_t corner = 0; corner < second_detections.corners[marker].size(); ++corner) {
      const float sign = ((marker + corner) % 2U == 0U) ? 1.0F : -1.0F;
      second_detections.corners[marker][corner].x += sign * 0.01F;
      second_detections.corners[marker][corner].y -= sign * 0.01F;
    }
  }
  const auto second = estimate_planar_board_pose(
    second_detections.ids, second_detections.corners, calibrations,
    cameraMatrix(), zeroDistortion(), first);
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->selection_reason, "RMSE_AND_TEMPORAL_CONTINUITY");
  EXPECT_LT(cv::norm(second->tvec_camera_deck - first->tvec_camera_deck), 0.01);
  EXPECT_LT(rotationErrorRad(second->rvec_camera_deck, first->rvec_camera_deck), 0.01);
  EXPECT_LT(normalErrorRad(second->rvec_camera_deck, expected_rvec), 0.01);
  EXPECT_TRUE(std::isfinite(second->reprojection_rmse_px));
}

TEST(PlanarBoardPose, SingleMarkerFallbackReturnsUnifiedDeckCenter)
{
  const auto calibrations = marineCalibrations();
  const auto & calibration = calibrations.front();
  const cv::Vec3d expected_deck_rvec = cameraDeckRvec(0.04, -0.02, 0.08);
  const cv::Vec3d expected_deck_tvec(0.09, -0.03, 5.1);

  cv::Mat rotation_camera_deck;
  cv::Rodrigues(expected_deck_rvec, rotation_camera_deck);
  const cv::Vec3d center_deck(
    calibration.center_deck_m[0], calibration.center_deck_m[1], calibration.center_deck_m[2]);
  const cv::Vec3d marker_tvec =
    cv::Matx33d(rotation_camera_deck) * center_deck + expected_deck_tvec;
  const cv::Vec3d marker_rvec = expected_deck_rvec;

  const auto pose = deck_pose_from_single_marker(calibration, marker_rvec, marker_tvec);
  ASSERT_TRUE(pose.has_value());
  EXPECT_EQ(pose->marker_count, 1U);
  EXPECT_TRUE(std::isnan(pose->reprojection_rmse_px));
  EXPECT_LT(cv::norm(pose->tvec_camera_deck - expected_deck_tvec), 1e-8);
  EXPECT_LT(rotationErrorRad(pose->rvec_camera_deck, expected_deck_rvec), 1e-8);
  EXPECT_LT(normalErrorRad(pose->rvec_camera_deck, expected_deck_rvec), 1e-8);
}

TEST(PlanarBoardPose, RejectsSingleMarkerBehindCameraOrFlippedNormal)
{
  const auto calibration = marineCalibrations().front();
  EXPECT_FALSE(deck_pose_from_single_marker(
      calibration, cameraDeckRvec(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, -1.0)).has_value());

  EXPECT_FALSE(deck_pose_from_single_marker(
      calibration, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 5.0)).has_value());
}

}  // namespace
}  // namespace aruco_detector
