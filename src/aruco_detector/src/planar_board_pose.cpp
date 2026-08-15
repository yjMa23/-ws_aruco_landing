// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/planar_board_pose.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>

namespace aruco_detector
{
namespace
{

constexpr double kGeometryTolerance = 1e-6;
constexpr double kMinimumDepthM = 1e-6;
constexpr double kMinimumFacingNormalZ = 1e-6;
constexpr double kRefinementReprojectionTolerancePx = 1e-6;

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

bool validCalibrationFields(const BoardMarkerCalibration & calibration)
{
  return calibration.id >= 0 && finite(calibration.length_m) && calibration.length_m > 0.0 &&
         std::all_of(
    calibration.center_deck_m.begin(), calibration.center_deck_m.end(), finite) &&
         std::all_of(
    calibration.rpy_deck_marker_rad.begin(), calibration.rpy_deck_marker_rad.end(), finite);
}

bool markerCornersDeck(
  const BoardMarkerCalibration & calibration,
  std::vector<cv::Point3f> & object_points)
{
  if (!validCalibrationFields(calibration)) {
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

bool pointsArePlanarAndNondegenerate(const std::vector<cv::Point3f> & points)
{
  if (points.size() < 8U) {
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
  if (singular_values.total() < 3U) {
    return false;
  }
  const double s0 = singular_values.at<double>(0, 0);
  const double s1 = singular_values.at<double>(1, 0);
  const double s2 = singular_values.at<double>(2, 0);
  return finite(s0) && finite(s1) && finite(s2) && s0 > 0.0 &&
         s1 > s0 * kGeometryTolerance && s2 <= s0 * kGeometryTolerance;
}

std::optional<double> commonDeckPlaneZ(const std::vector<cv::Point3f> & points)
{
  if (points.empty()) {
    return std::nullopt;
  }
  double minimum_z = points.front().z;
  double maximum_z = points.front().z;
  for (const auto & point : points) {
    if (!finite(point.z)) {
      return std::nullopt;
    }
    minimum_z = std::min(minimum_z, static_cast<double>(point.z));
    maximum_z = std::max(maximum_z, static_cast<double>(point.z));
  }
  if (maximum_z - minimum_z > kGeometryTolerance) {
    return std::nullopt;
  }
  return 0.5 * (minimum_z + maximum_z);
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
         finite(tvec[0]) && finite(tvec[1]) && finite(tvec[2]) &&
         tvec[2] > kMinimumDepthM;
}

bool finiteImageCorners(const std::vector<cv::Point2f> & corners)
{
  return corners.size() == 4U && std::all_of(
    corners.begin(), corners.end(), [](const cv::Point2f & corner) {
      return finite(corner.x) && finite(corner.y);
    });
}

std::optional<cv::Vec3d> matToVec3d(const cv::Mat & value)
{
  if (value.total() != 3U || value.channels() != 1) {
    return std::nullopt;
  }
  cv::Mat value_64f;
  value.reshape(1, 3).convertTo(value_64f, CV_64F);
  const cv::Vec3d result{
    value_64f.at<double>(0, 0),
    value_64f.at<double>(1, 0),
    value_64f.at<double>(2, 0)};
  if (!finite(result[0]) || !finite(result[1]) || !finite(result[2])) {
    return std::nullopt;
  }
  return result;
}

bool allObjectPointsInFront(
  const std::vector<cv::Point3f> & object_points,
  const cv::Matx33d & rotation_camera_deck,
  const cv::Vec3d & tvec_camera_deck)
{
  for (const auto & point : object_points) {
    const cv::Vec3d point_camera =
      rotation_camera_deck * cv::Vec3d(point.x, point.y, point.z) + tvec_camera_deck;
    if (!finite(point_camera[2]) || point_camera[2] <= kMinimumDepthM) {
      return false;
    }
  }
  return true;
}

bool deckNormalFacesCamera(const cv::Matx33d & rotation_camera_deck)
{
  // 下视 camera_optical 的 +z 指向甲板，deck +z 指回相机，因此合法法向的 camera z 为负。
  const cv::Vec3d normal_camera = rotation_camera_deck * cv::Vec3d(0.0, 0.0, 1.0);
  return finite(normal_camera[0]) && finite(normal_camera[1]) && finite(normal_camera[2]) &&
         normal_camera[2] < -kMinimumFacingNormalZ;
}

double reprojectionRmse(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Vec3d & rvec,
  const cv::Vec3d & tvec,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion)
{
  std::vector<cv::Point2f> reprojected;
  cv::projectPoints(
    object_points, rvec, tvec, camera_matrix, distortion, reprojected);
  if (reprojected.size() != image_points.size() || reprojected.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double squared_error = 0.0;
  for (std::size_t index = 0; index < image_points.size(); ++index) {
    if (!finite(reprojected[index].x) || !finite(reprojected[index].y)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const cv::Point2f residual = reprojected[index] - image_points[index];
    squared_error += static_cast<double>(residual.dot(residual));
  }
  return std::sqrt(squared_error / static_cast<double>(image_points.size()));
}

double rotationDistanceRad(const cv::Vec3d & first_rvec, const cv::Vec3d & second_rvec)
{
  cv::Mat first_rotation;
  cv::Mat second_rotation;
  cv::Rodrigues(first_rvec, first_rotation);
  cv::Rodrigues(second_rvec, second_rotation);
  const cv::Mat delta = first_rotation.t() * second_rotation;
  const double cosine = std::clamp((cv::trace(delta)[0] - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

bool validPreviousPose(const PlanarBoardPose & pose)
{
  return finitePose(pose.rvec_camera_deck, pose.tvec_camera_deck) &&
         finite(pose.reprojection_rmse_px);
}

struct PoseCandidate
{
  cv::Vec3d rvec;
  cv::Vec3d tvec;
  double rmse_px{0.0};
  double score{0.0};
};

}  // namespace

bool is_valid_planar_board_calibration(
  const std::vector<BoardMarkerCalibration> & calibrations)
{
  if (calibrations.size() < 2U) {
    return false;
  }

  std::unordered_set<int> ids;
  std::vector<cv::Point3f> object_points;
  object_points.reserve(calibrations.size() * 4U);
  for (const auto & calibration : calibrations) {
    if (!ids.insert(calibration.id).second || !markerCornersDeck(calibration, object_points)) {
      return false;
    }
  }
  return pointsArePlanarAndNondegenerate(object_points) && commonDeckPlaneZ(object_points).has_value();
}

std::size_t count_valid_board_markers(
  const std::vector<int> & detected_ids,
  const std::vector<std::vector<cv::Point2f>> & detected_corners,
  const std::vector<BoardMarkerCalibration> & calibrations)
{
  if (detected_ids.size() != detected_corners.size()) {
    return 0U;
  }

  std::unordered_set<int> calibration_ids;
  for (const auto & calibration : calibrations) {
    if (validCalibrationFields(calibration)) {
      calibration_ids.insert(calibration.id);
    }
  }

  std::unordered_set<int> counted_ids;
  std::size_t count = 0U;
  for (std::size_t index = 0; index < detected_ids.size(); ++index) {
    if (calibration_ids.count(detected_ids[index]) == 0U ||
      counted_ids.count(detected_ids[index]) != 0U ||
      !finiteImageCorners(detected_corners[index]))
    {
      continue;
    }
    counted_ids.insert(detected_ids[index]);
    ++count;
  }
  return count;
}

std::optional<PlanarBoardPose> estimate_planar_board_pose(
  const std::vector<int> & detected_ids,
  const std::vector<std::vector<cv::Point2f>> & detected_corners,
  const std::vector<BoardMarkerCalibration> & calibrations,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients,
  const std::optional<PlanarBoardPose> & previous_pose,
  double max_reprojection_rmse_px)
{
  if (detected_ids.size() != detected_corners.size() ||
    !is_valid_planar_board_calibration(calibrations) ||
    !validCameraModel(camera_matrix, distortion_coefficients) ||
    !finite(max_reprojection_rmse_px) || max_reprojection_rmse_px <= 0.0)
  {
    return std::nullopt;
  }

  std::unordered_map<int, const BoardMarkerCalibration *> calibration_by_id;
  calibration_by_id.reserve(calibrations.size());
  for (const auto & calibration : calibrations) {
    calibration_by_id.emplace(calibration.id, &calibration);
  }

  std::unordered_set<int> used_ids;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  object_points.reserve(calibrations.size() * 4U);
  image_points.reserve(calibrations.size() * 4U);
  std::size_t marker_count = 0U;
  for (std::size_t index = 0; index < detected_ids.size(); ++index) {
    const auto calibration = calibration_by_id.find(detected_ids[index]);
    if (calibration == calibration_by_id.end() || used_ids.count(detected_ids[index]) != 0U ||
      !finiteImageCorners(detected_corners[index]))
    {
      continue;
    }
    if (!markerCornersDeck(*calibration->second, object_points)) {
      return std::nullopt;
    }
    image_points.insert(
      image_points.end(), detected_corners[index].begin(), detected_corners[index].end());
    used_ids.insert(detected_ids[index]);
    ++marker_count;
  }
  const auto plane_z = commonDeckPlaneZ(object_points);
  if (marker_count < 2U || !pointsArePlanarAndNondegenerate(object_points) ||
    !plane_z.has_value())
  {
    return std::nullopt;
  }

  // OpenCV 4.x IPPE expects the supplied object plane to be z=0 in its solver frame.
  // Keep calibration at the physical print plane (for Marine z=0.002 m), translate only
  // the temporary solver points, then transform each candidate translation back to deck origin.
  std::vector<cv::Point3f> solver_object_points = object_points;
  for (auto & point : solver_object_points) {
    point.z = static_cast<float>(static_cast<double>(point.z) - *plane_z);
  }

  try {
    std::vector<cv::Mat> rvec_candidates;
    std::vector<cv::Mat> tvec_candidates;
    const int solution_count = cv::solvePnPGeneric(
      solver_object_points, image_points, camera_matrix, distortion_coefficients,
      rvec_candidates, tvec_candidates, false, cv::SOLVEPNP_IPPE);
    if (solution_count <= 0 || rvec_candidates.size() != tvec_candidates.size()) {
      return std::nullopt;
    }

    const bool use_previous = previous_pose.has_value() && validPreviousPose(*previous_pose);
    std::vector<PoseCandidate> candidates;
    candidates.reserve(rvec_candidates.size());
    for (std::size_t index = 0; index < rvec_candidates.size(); ++index) {
      const auto rvec = matToVec3d(rvec_candidates[index]);
      const auto solver_tvec = matToVec3d(tvec_candidates[index]);
      if (!rvec.has_value() || !solver_tvec.has_value()) {
        continue;
      }

      cv::Mat rotation_matrix;
      cv::Rodrigues(*rvec, rotation_matrix);
      cv::Mat rotation_64f;
      rotation_matrix.convertTo(rotation_64f, CV_64F);
      const cv::Matx33d rotation_camera_deck(
        rotation_64f.at<double>(0, 0), rotation_64f.at<double>(0, 1),
        rotation_64f.at<double>(0, 2), rotation_64f.at<double>(1, 0),
        rotation_64f.at<double>(1, 1), rotation_64f.at<double>(1, 2),
        rotation_64f.at<double>(2, 0), rotation_64f.at<double>(2, 1),
        rotation_64f.at<double>(2, 2));
      const cv::Vec3d deck_plane_origin{0.0, 0.0, *plane_z};
      const cv::Vec3d tvec =
        *solver_tvec - rotation_camera_deck * deck_plane_origin;
      if (!finitePose(*rvec, tvec) ||
        !allObjectPointsInFront(object_points, rotation_camera_deck, tvec) ||
        !deckNormalFacesCamera(rotation_camera_deck))
      {
        continue;
      }

      const double rmse = reprojectionRmse(
        object_points, image_points, *rvec, tvec, camera_matrix, distortion_coefficients);
      if (!finite(rmse) || rmse > max_reprojection_rmse_px) {
        continue;
      }

      double score = rmse;
      if (use_previous) {
        // 连续性只做软代价：RMSE 仍占主导，避免旧先验强行覆盖当前真实图像证据。
        score += 0.5 * cv::norm(tvec - previous_pose->tvec_camera_deck);
        score += 0.25 * rotationDistanceRad(previous_pose->rvec_camera_deck, *rvec);
      }
      if (!finite(score)) {
        continue;
      }
      candidates.push_back(PoseCandidate{*rvec, tvec, rmse, score});
    }

    if (candidates.empty()) {
      return std::nullopt;
    }
    const auto best = std::min_element(
      candidates.begin(), candidates.end(),
      [](const PoseCandidate & lhs, const PoseCandidate & rhs) {
        return lhs.score < rhs.score;
      });

    cv::Vec3d selected_rvec = best->rvec;
    cv::Vec3d selected_tvec = best->tvec;
    double selected_rmse = best->rmse_px;

    // IPPE 继续负责平面双解与时间连续性消歧；LM 只用同一帧全部 Board corners
    // 对已选合法候选做重投影精化。任何异常、物理约束失效或 RMSE 变差都保留原 IPPE。
    try {
      cv::Vec3d refined_rvec = selected_rvec;
      cv::Vec3d refined_tvec = selected_tvec;
      cv::solvePnPRefineLM(
        object_points, image_points, camera_matrix, distortion_coefficients,
        refined_rvec, refined_tvec);

      cv::Mat refined_rotation_matrix;
      cv::Rodrigues(refined_rvec, refined_rotation_matrix);
      cv::Mat refined_rotation_64f;
      refined_rotation_matrix.convertTo(refined_rotation_64f, CV_64F);
      const cv::Matx33d refined_rotation_camera_deck(
        refined_rotation_64f.at<double>(0, 0), refined_rotation_64f.at<double>(0, 1),
        refined_rotation_64f.at<double>(0, 2), refined_rotation_64f.at<double>(1, 0),
        refined_rotation_64f.at<double>(1, 1), refined_rotation_64f.at<double>(1, 2),
        refined_rotation_64f.at<double>(2, 0), refined_rotation_64f.at<double>(2, 1),
        refined_rotation_64f.at<double>(2, 2));
      const double refined_rmse = reprojectionRmse(
        object_points, image_points, refined_rvec, refined_tvec,
        camera_matrix, distortion_coefficients);

      if (finitePose(refined_rvec, refined_tvec) &&
        allObjectPointsInFront(
          object_points, refined_rotation_camera_deck, refined_tvec) &&
        deckNormalFacesCamera(refined_rotation_camera_deck) && finite(refined_rmse) &&
        refined_rmse <= max_reprojection_rmse_px &&
        refined_rmse <= selected_rmse + kRefinementReprojectionTolerancePx)
      {
        selected_rvec = refined_rvec;
        selected_tvec = refined_tvec;
        selected_rmse = refined_rmse;
      }
    } catch (const cv::Exception &) {
      // 原 IPPE 候选已经通过全部 hard checks；refinement 失败不能把有效观测变成无效。
    }

    return PlanarBoardPose{
      selected_rvec,
      selected_tvec,
      selected_rmse,
      marker_count,
      use_previous ? "RMSE_AND_TEMPORAL_CONTINUITY" : "RMSE"};
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
}

std::optional<PlanarBoardPose> deck_pose_from_single_marker(
  const BoardMarkerCalibration & calibration,
  const cv::Vec3d & rvec_camera_marker,
  const cv::Vec3d & tvec_camera_marker)
{
  if (!validCalibrationFields(calibration) ||
    !finitePose(rvec_camera_marker, tvec_camera_marker))
  {
    return std::nullopt;
  }

  try {
    cv::Mat rotation_camera_marker_mat;
    cv::Rodrigues(rvec_camera_marker, rotation_camera_marker_mat);
    cv::Mat rotation_camera_marker_64f;
    rotation_camera_marker_mat.convertTo(rotation_camera_marker_64f, CV_64F);
    const cv::Matx33d rotation_camera_marker(
      rotation_camera_marker_64f.at<double>(0, 0),
      rotation_camera_marker_64f.at<double>(0, 1),
      rotation_camera_marker_64f.at<double>(0, 2),
      rotation_camera_marker_64f.at<double>(1, 0),
      rotation_camera_marker_64f.at<double>(1, 1),
      rotation_camera_marker_64f.at<double>(1, 2),
      rotation_camera_marker_64f.at<double>(2, 0),
      rotation_camera_marker_64f.at<double>(2, 1),
      rotation_camera_marker_64f.at<double>(2, 2));

    const cv::Matx33d rotation_deck_marker = rotationFromRpy(
      calibration.rpy_deck_marker_rad);
    const cv::Matx33d rotation_marker_deck = rotation_deck_marker.t();
    const cv::Vec3d center_deck{
      calibration.center_deck_m[0],
      calibration.center_deck_m[1],
      calibration.center_deck_m[2]};
    const cv::Vec3d translation_marker_deck = -(rotation_marker_deck * center_deck);

    const cv::Matx33d rotation_camera_deck = rotation_camera_marker * rotation_marker_deck;
    const cv::Vec3d tvec_camera_deck =
      tvec_camera_marker + rotation_camera_marker * translation_marker_deck;
    if (!finitePose(cv::Vec3d(0.0, 0.0, 0.0), tvec_camera_deck) ||
      !deckNormalFacesCamera(rotation_camera_deck))
    {
      return std::nullopt;
    }

    cv::Mat rotation_camera_deck_mat(rotation_camera_deck);
    cv::Vec3d rvec_camera_deck;
    cv::Rodrigues(rotation_camera_deck_mat, rvec_camera_deck);
    if (!finitePose(rvec_camera_deck, tvec_camera_deck)) {
      return std::nullopt;
    }
    return PlanarBoardPose{
      rvec_camera_deck,
      tvec_camera_deck,
      std::numeric_limits<double>::quiet_NaN(),
      1U,
      "SINGLE_MARKER_TRANSFORM"};
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
}

}  // namespace aruco_detector
