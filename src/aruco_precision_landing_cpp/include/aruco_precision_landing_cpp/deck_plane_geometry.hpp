// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__DECK_PLANE_GEOMETRY_HPP_
#define ARUCO_PRECISION_LANDING_CPP__DECK_PLANE_GEOMETRY_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>

#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 甲板平面几何计算参数。
 */
struct DeckPlaneGeometryParameters
{
  double minimum_normal_norm{1.0e-6};
  double minimum_upward_component{0.5};
};

/**
 * @brief 甲板平面几何输入。
 *
 * 所有位置和线速度均使用 PX4 `local_ned`（North、East、Down），单位分别为米和米每秒。
 * `body_frd_to_ned` 是 Hamilton 四元数，表示将 `base_link_frd` 向量旋转到
 * `local_ned`；Eigen 构造顺序为 `(w, x, y, z)`。接触点使用 `base_link_frd`
 * （Forward、Right、Down），单位为米。UAV 角速度使用 `base_link_frd`，甲板角速度
 * 使用 `local_ned`，单位均为弧度每秒。
 */
struct DeckPlaneGeometryInput
{
  Eigen::Vector3d deck_reference_position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upward_normal_ned{Eigen::Vector3d{0.0, 0.0, -1.0}};
  Eigen::Vector3d uav_reference_position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond body_frd_to_ned{Eigen::Quaterniond::Identity()};
  std::array<Eigen::Vector3d, 4> contact_points_body_frd_m{};

  std::optional<Eigen::Vector3d> deck_linear_velocity_ned_mps;
  std::optional<Eigen::Vector3d> uav_linear_velocity_ned_mps;
  std::optional<Eigen::Vector3d> deck_angular_velocity_ned_radps;
  std::optional<Eigen::Vector3d> uav_angular_velocity_body_frd_radps;
};

/**
 * @brief 甲板平面几何有效输出。
 *
 * 法向始终归一化并指向甲板上方。间隙为 `nᵀ(p-p_d)`，点在甲板上方时为正。
 * 法向相对速度为 UAV 相对甲板速度沿向上法向的投影，正值表示分离，负值表示闭合。
 * 平面内位置和速度仍以 `local_ned` 三维向量表达，法向分量理论上为零。
 */
struct DeckPlaneGeometryOutput
{
  Eigen::Vector3d upward_normal_ned{Eigen::Vector3d{0.0, 0.0, -1.0}};
  double body_normal_gap_m{0.0};
  std::array<Eigen::Vector3d, 4> contact_positions_ned_m{};
  std::array<double, 4> contact_gaps_m{};
  double minimum_contact_gap_m{0.0};
  double maximum_contact_gap_m{0.0};
  double contact_gap_spread_m{0.0};
  std::size_t first_contact_index{0};
  std::optional<double> body_normal_relative_velocity_mps;
  std::array<std::optional<double>, 4> contact_normal_relative_velocity_mps{};
  Eigen::Vector3d tangential_position_error_ned_m{Eigen::Vector3d::Zero()};
  std::optional<Eigen::Vector3d> tangential_relative_velocity_ned_mps;
  std::string velocity_status{"velocity inputs not provided"};
};

/**
 * @brief 甲板平面几何计算结果。
 */
struct DeckPlaneGeometryResult
{
  bool valid{false};
  std::string failure_reason;
  DeckPlaneGeometryOutput output{};
};

/**
 * @brief 计算甲板平面、X500 四滑橇端点及相对速度的纯数学工具。
 *
 * 本类不依赖 ROS、Gazebo 或 Ground Truth，不读取参数服务器，也不改变任何控制输出。
 * 普通传感器输入异常通过 `DeckPlaneGeometryResult` 显式返回，不抛出异常。
 */
class DeckPlaneGeometry
{
public:
  /**
   * @brief 返回 X500 两条滑橇碰撞盒底部中心线的四个等效端点。
   *
   * @return 四个 `base_link_frd` 接触点，单位为米，顺序为
   *         `(-x,-y)、(+x,-y)、(-x,+y)、(+x,+y)`。
   *
   * 该近似来自 `x500_base/model.sdf` 中两条 `0.25×0.015×0.015 m` 碰撞盒：
   * FLU 中心 `y=±0.132 m, z=-0.2195 m`，转换到 FRD 后最低表面为
   * `z=0.2195+0.015/2=0.227 m`。模型忽略碰撞盒半宽支持面、柔性和接触压入。
   */
  static std::array<Eigen::Vector3d, 4> x500_default_contact_points_body_frd_m();

  /**
   * @brief 计算甲板平面几何和可用的相对速度诊断。
   *
   * @param input 位置、姿态、接触点和可选速度输入；坐标系与单位见
   *        `DeckPlaneGeometryInput`。
   * @param parameters 法向范数和向上分量有效性门限。
   * @return 输入位置、法向、姿态或接触点无效时 `valid=false` 并给出失败原因；
   *         速度缺失或非法时位置几何仍有效，相关可选速度为空并在
   *         `velocity_status` 中说明原因。
   *
   * 失败条件包括 NaN/Inf 位置或接触点、法向范数过小、法向未指向 NED 上方、
   * 四元数范数过小以及非法参数。有限且范数有效的非单位四元数会自动归一化。
   */
  static DeckPlaneGeometryResult compute(
    const DeckPlaneGeometryInput & input,
    const DeckPlaneGeometryParameters & parameters = DeckPlaneGeometryParameters{});
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__DECK_PLANE_GEOMETRY_HPP_
