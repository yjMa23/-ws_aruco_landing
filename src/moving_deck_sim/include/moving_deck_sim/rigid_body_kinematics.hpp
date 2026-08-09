#pragma once

#include <array>

namespace moving_deck_sim
{

/**
 * @brief 一个刚体参考点的 world 位姿与速度状态。
 *
 * 位置和线速度在 Gazebo world ENU 中表达；四元数采用 wxyz，表示 body 到 world
 * 的旋转；角速度在当前刚体 body frame 中表达。所有量必须有限。
 */
struct RigidBodyState
{
  std::array<double, 3> position_world{};
  std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
  std::array<double, 3> linear_velocity_world{};
  std::array<double, 3> angular_velocity_body{};
};

/**
 * @brief 从父刚体参考点到固定子参考点的刚体变换。
 *
 * `translation_body` 在父 body frame 中表达；`rotation_wxyz` 表示子 frame 到父
 * frame 的旋转。默认值为零平移和单位旋转。
 */
struct FixedRigidTransform
{
  std::array<double, 3> translation_body{};
  std::array<double, 4> rotation_wxyz{1.0, 0.0, 0.0, 0.0};
};

/**
 * @brief 将父刚体状态转换为固定子参考点状态，并包含角速度导致的杠杆臂线速度。
 *
 * @param parent 父参考点状态；线速度为 world ENU，角速度为父 body frame。
 * @param transform 固定 parent→child 变换。
 * @return 子参考点 world pose、world linear velocity 和 child-body angular velocity。
 * @throws std::invalid_argument 输入包含 NaN/Inf 或四元数无法归一化时抛出。
 */
RigidBodyState transform_rigid_body_state(
  const RigidBodyState & parent,
  const FixedRigidTransform & transform);

}  // namespace moving_deck_sim
