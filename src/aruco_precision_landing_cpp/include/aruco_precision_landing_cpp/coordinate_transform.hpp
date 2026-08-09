// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__COORDINATE_TRANSFORM_HPP_
#define ARUCO_PRECISION_LANDING_CPP__COORDINATE_TRANSFORM_HPP_

#include <optional>

#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 表示 PX4 位姿参考坐标系。
 */
enum class PoseReferenceFrame
{
  kUnknown,
  kLocalNed,
  kLocalFrd
};

/**
 * @brief 使用平移和 Hamilton 四元数表示三维刚体位姿。
 *
 * `translation` 的单位为米；`rotation` 表示从子坐标系到父坐标系的主动向量旋转。
 */
struct Pose3d
{
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
};

/**
 * @brief 根据平移和四元数构造刚体变换。
 *
 * @param translation 子坐标系原点在父坐标系中的位置，单位为米。
 * @param rotation_from_child_to_parent Hamilton 四元数，表示子坐标系向量到父坐标系的旋转；
 *        Eigen 构造顺序为 `(w, x, y, z)`。
 * @return 归一化后的刚体变换；位置含 NaN/Inf、四元数含 NaN/Inf 或范数过小时返回
 *         `std::nullopt`。有限且范数有效的未归一化四元数会自动归一化。
 */
std::optional<Eigen::Isometry3d> make_isometry(
  const Eigen::Vector3d & translation,
  const Eigen::Quaterniond & rotation_from_child_to_parent);

/**
 * @brief 将 ArUco Marker 位姿从相机光学坐标系转换到 PX4 local NED。
 *
 * 计算链为：
 * `T_local_ned_marker = T_local_ned_body_frd * T_body_frd_camera_optical *
 * T_camera_optical_marker`。
 *
 * @param local_body_pose 无人机 `body_frd` 在本地参考系中的位姿，位置单位为米。
 * @param local_pose_frame `local_body_pose` 的参考系；当前只接受 `kLocalNed`。
 * @param body_camera_pose `camera_optical` 在 `body_frd` 中的外参，位置单位为米。
 * @param camera_marker_pose Marker 在 `camera_optical` 中的 PnP 位姿，位置单位为米。
 * @return Marker 在 `local_ned` 中的位姿；参考系错误或任一输入无效时返回
 *         `std::nullopt`。
 */
std::optional<Pose3d> transform_marker_to_local_ned(
  const Pose3d & local_body_pose,
  PoseReferenceFrame local_pose_frame,
  const Pose3d & body_camera_pose,
  const Pose3d & camera_marker_pose);

/**
 * @brief 将 ArUco Marker 位姿转换到以无人机为原点、轴平行 local NED 的坐标系。
 *
 * 平移为 `p_marker_ned - p_uav_ned`，姿态仍表示 Marker 到 local NED 的旋转。
 * 该转换只使用图像时刻的机体姿态和相机外参，不使用无人机绝对位置。
 *
 * @param body_to_ned_rotation 图像时刻从 `body_frd` 到 `local_ned` 的旋转。
 * @param body_camera_pose `camera_optical` 在 `body_frd` 中的外参。
 * @param camera_marker_pose Marker 在 `camera_optical` 中的 PnP 位姿。
 * @return Marker 在 `uav_centered_ned` 中的位姿；任一输入无效时返回
 *         `std::nullopt`。
 */
std::optional<Pose3d> transform_marker_to_uav_centered_ned(
  const Eigen::Quaterniond & body_to_ned_rotation,
  const Pose3d & body_camera_pose,
  const Pose3d & camera_marker_pose);

/**
 * @brief 将 Gazebo / ROS ENU 向量转换为 PX4 NED 向量。
 *
 * @param vector_enu ENU 向量，分量依次为 East、North、Up，单位由调用方保持一致。
 * @return NED 向量 `[North, East, Down]`；输入含 NaN/Inf 时返回 `std::nullopt`。
 */
std::optional<Eigen::Vector3d> enu_to_ned(const Eigen::Vector3d & vector_enu);

/**
 * @brief 将 PX4 NED 向量转换为 Gazebo / ROS ENU 向量。
 *
 * @param vector_ned NED 向量，分量依次为 North、East、Down，单位由调用方保持一致。
 * @return ENU 向量 `[East, North, Up]`；输入含 NaN/Inf 时返回 `std::nullopt`。
 */
std::optional<Eigen::Vector3d> ned_to_enu(const Eigen::Vector3d & vector_ned);

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__COORDINATE_TRANSFORM_HPP_
