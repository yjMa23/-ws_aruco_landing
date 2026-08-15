// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/planar_board_pose.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
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

struct RawIppePose
{
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  double reprojection_rmse_px;
};

double reprojectionRmse(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, rvec, tvec, cameraMatrix(), zeroDistortion(), projected);
  double squared_error = 0.0;
  for (std::size_t index = 0; index < image_points.size(); ++index) {
    const cv::Point2f residual = projected[index] - image_points[index];
    squared_error += static_cast<double>(residual.dot(residual));
  }
  return std::sqrt(squared_error / static_cast<double>(image_points.size()));
}

std::optional<RawIppePose> estimateRawIppePose(const SyntheticDetections & detections)
{
  const auto object_points = allObjectPoints(marineCalibrations());
  auto solver_object_points = object_points;
  for (auto & point : solver_object_points) {
    point.z -= 0.002F;
  }
  const auto image_points = flattenCorners(detections);

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  const int solution_count = cv::solvePnPGeneric(
    solver_object_points, image_points, cameraMatrix(), zeroDistortion(),
    rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
  if (solution_count <= 0) {
    return std::nullopt;
  }

  std::optional<RawIppePose> best;
  for (std::size_t index = 0; index < rvecs.size(); ++index) {
    cv::Mat rvec_64f;
    cv::Mat solver_tvec_64f;
    rvecs[index].reshape(1, 3).convertTo(rvec_64f, CV_64F);
    tvecs[index].reshape(1, 3).convertTo(solver_tvec_64f, CV_64F);
    const cv::Vec3d rvec(
      rvec_64f.at<double>(0, 0), rvec_64f.at<double>(1, 0), rvec_64f.at<double>(2, 0));
    const cv::Vec3d solver_tvec(
      solver_tvec_64f.at<double>(0, 0), solver_tvec_64f.at<double>(1, 0),
      solver_tvec_64f.at<double>(2, 0));

    cv::Mat rotation_mat;
    cv::Rodrigues(rvec, rotation_mat);
    const cv::Matx33d rotation(rotation_mat);
    const cv::Vec3d tvec = solver_tvec - rotation * cv::Vec3d(0.0, 0.0, 0.002);
    if (tvec[2] <= 0.0 || rotation(2, 2) >= 0.0) {
      continue;
    }
    bool points_in_front = true;
    for (const auto & point : object_points) {
      const cv::Vec3d point_camera =
        rotation * cv::Vec3d(point.x, point.y, point.z) + tvec;
      points_in_front = points_in_front && point_camera[2] > 0.0;
    }
    if (!points_in_front) {
      continue;
    }

    const double rmse = reprojectionRmse(object_points, image_points, rvec, tvec);
    if (!best.has_value() || rmse < best->reprojection_rmse_px) {
      best = RawIppePose{rvec, tvec, rmse};
    }
  }
  return best;
}

double rmse(const std::vector<double> & values)
{
  const double squared_sum = std::inner_product(
    values.begin(), values.end(), values.begin(), 0.0);
  return std::sqrt(squared_sum / static_cast<double>(values.size()));
}

double p95(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
    std::ceil(0.95 * static_cast<double>(values.size()))) - 1U;
  return values[std::min(index, values.size() - 1U)];
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

TEST(PlanarBoardPose, RefinesNearFrontoParallelNoisyMarineBoard)
{
  const auto calibrations = marineCalibrations();
  const cv::Vec3d expected_tvec(0.04, -0.03, 5.5);
  const std::array<double, 4> tilts_deg{{0.0, 0.5, 2.0, 5.0}};
  std::mt19937 random(20260815U);
  std::normal_distribution<double> corner_noise_px(0.0, 0.45);

  for (const double tilt_deg : tilts_deg) {
    SCOPED_TRACE(::testing::Message() << "tilt_deg=" << tilt_deg);
    const cv::Vec3d expected_rvec = cameraDeckRvec(0.0, tilt_deg * kPi / 180.0, 0.0);
    const auto ideal = projectMarkers(
      calibrations, {4, 5, 6, 7}, expected_rvec, expected_tvec);

    std::vector<double> raw_normal_errors_deg;
    std::vector<double> refined_normal_errors_deg;
    std::vector<double> raw_reprojection_rmse_px;
    std::vector<double> refined_reprojection_rmse_px;
    int invalid_raw_count = 0;
    int invalid_refined_count = 0;
    int reprojection_worse_count = 0;

    for (int trial = 0; trial < 500; ++trial) {
      auto noisy = ideal;
      for (auto & marker_corners : noisy.corners) {
        for (auto & corner : marker_corners) {
          corner.x += static_cast<float>(corner_noise_px(random));
          corner.y += static_cast<float>(corner_noise_px(random));
        }
      }

      const auto raw = estimateRawIppePose(noisy);
      if (!raw.has_value()) {
        ++invalid_raw_count;
        continue;
      }
      const auto refined = estimate_planar_board_pose(
        noisy.ids, noisy.corners, calibrations, cameraMatrix(), zeroDistortion());
      if (!refined.has_value()) {
        ++invalid_refined_count;
        continue;
      }

      raw_normal_errors_deg.push_back(
        normalErrorRad(raw->rvec, expected_rvec) * 180.0 / kPi);
      refined_normal_errors_deg.push_back(
        normalErrorRad(refined->rvec_camera_deck, expected_rvec) * 180.0 / kPi);
      raw_reprojection_rmse_px.push_back(raw->reprojection_rmse_px);
      refined_reprojection_rmse_px.push_back(refined->reprojection_rmse_px);
      if (refined->reprojection_rmse_px > raw->reprojection_rmse_px + 1e-6) {
        ++reprojection_worse_count;
      }

      cv::Mat refined_rotation_mat;
      cv::Rodrigues(refined->rvec_camera_deck, refined_rotation_mat);
      const cv::Matx33d refined_rotation(refined_rotation_mat);
      EXPECT_LT(refined_rotation(2, 2), 0.0);
      EXPECT_GT(refined->tvec_camera_deck[2], 0.0);
      EXPECT_TRUE(std::isfinite(refined->reprojection_rmse_px));
    }

    ASSERT_EQ(invalid_raw_count, 0);
    ASSERT_EQ(invalid_refined_count, 0);
    ASSERT_EQ(raw_normal_errors_deg.size(), 500U);
    ASSERT_EQ(refined_normal_errors_deg.size(), 500U);
    EXPECT_EQ(reprojection_worse_count, 0);

    const double raw_normal_rmse = rmse(raw_normal_errors_deg);
    const double refined_normal_rmse = rmse(refined_normal_errors_deg);
    const double raw_normal_p95 = p95(raw_normal_errors_deg);
    const double refined_normal_p95 = p95(refined_normal_errors_deg);
    std::cout << "synthetic tilt=" << tilt_deg << "deg raw normal RMSE/P95="
              << raw_normal_rmse << "/" << raw_normal_p95
              << "deg refined=" << refined_normal_rmse << "/" << refined_normal_p95
              << "deg raw/refined reprojection RMS=" << rmse(raw_reprojection_rmse_px)
              << "/" << rmse(refined_reprojection_rmse_px) << "px" << std::endl;
    EXPECT_LT(refined_normal_rmse, 0.5 * raw_normal_rmse);
    EXPECT_LT(refined_normal_p95, raw_normal_p95);
    EXPECT_LT(rmse(refined_reprojection_rmse_px), rmse(raw_reprojection_rmse_px) + 1e-6);
  }
}

TEST(PlanarBoardPose, FallsBackWhenRefinedPoseViolatesDeckNormalDirection)
{
  const auto calibrations = marineCalibrations();
  SyntheticDetections detections{
    {4, 5, 6, 7},
    {
      {{669.7396F, 89.3006F}, {212.6208F, 336.5889F}, {489.9025F, 493.8973F},
        {424.1419F, 621.1867F}},
      {{1026.4141F, 505.2187F}, {742.0280F, 463.3254F}, {370.9999F, 663.2423F},
        {599.4517F, 407.2244F}},
      {{643.9877F, 60.4481F}, {428.1008F, 72.7182F}, {681.9301F, 236.9781F},
        {467.0085F, 251.3473F}},
      {{693.6324F, 400.5563F}, {775.4152F, 539.3513F}, {852.1837F, 519.0226F},
        {541.5914F, 485.0388F}},
    }};

  const auto raw = estimateRawIppePose(detections);
  ASSERT_TRUE(raw.has_value());
  ASSERT_GT(raw->reprojection_rmse_px, 600.0);

  cv::Vec3d refined_rvec = raw->rvec;
  cv::Vec3d refined_tvec = raw->tvec;
  cv::solvePnPRefineLM(
    allObjectPoints(calibrations), flattenCorners(detections),
    cameraMatrix(), zeroDistortion(), refined_rvec, refined_tvec);
  cv::Mat invalid_refined_rotation_mat;
  cv::Rodrigues(refined_rvec, invalid_refined_rotation_mat);
  ASSERT_GT(cv::Matx33d(invalid_refined_rotation_mat)(2, 2), 0.0);

  const auto pose = estimate_planar_board_pose(
    detections.ids, detections.corners, calibrations,
    cameraMatrix(), zeroDistortion(), std::nullopt, 1000.0);
  ASSERT_TRUE(pose.has_value());
  EXPECT_NEAR(pose->reprojection_rmse_px, raw->reprojection_rmse_px, 1e-3);
  EXPECT_LT(cv::norm(pose->tvec_camera_deck - raw->tvec), 1e-6);
  EXPECT_LT(rotationErrorRad(pose->rvec_camera_deck, raw->rvec), 1e-6);
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
