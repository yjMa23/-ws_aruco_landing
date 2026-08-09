#include "moving_deck_sim/rigid_body_kinematics.hpp"

#include <cmath>
#include <stdexcept>

namespace moving_deck_sim
{
namespace
{

using Quaternion = std::array<double, 4>;
using Vector3 = std::array<double, 3>;

bool finite(const Vector3 & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
}

bool finite(const Quaternion & value)
{
  return std::isfinite(value[0]) && std::isfinite(value[1]) &&
         std::isfinite(value[2]) && std::isfinite(value[3]);
}

Quaternion normalize(const Quaternion & value)
{
  if (!finite(value)) {
    throw std::invalid_argument("quaternion must be finite");
  }
  const double norm = std::sqrt(
    value[0] * value[0] + value[1] * value[1] +
    value[2] * value[2] + value[3] * value[3]);
  if (!(norm > 1e-12)) {
    throw std::invalid_argument("quaternion norm is too small");
  }
  return {value[0] / norm, value[1] / norm, value[2] / norm, value[3] / norm};
}

Quaternion multiply(const Quaternion & lhs, const Quaternion & rhs)
{
  return {
    lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2] - lhs[3] * rhs[3],
    lhs[0] * rhs[1] + lhs[1] * rhs[0] + lhs[2] * rhs[3] - lhs[3] * rhs[2],
    lhs[0] * rhs[2] - lhs[1] * rhs[3] + lhs[2] * rhs[0] + lhs[3] * rhs[1],
    lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1] + lhs[3] * rhs[0],
  };
}

Quaternion conjugate(const Quaternion & value)
{
  return {value[0], -value[1], -value[2], -value[3]};
}

Vector3 rotate(const Quaternion & unit_quaternion, const Vector3 & vector)
{
  const Quaternion pure{0.0, vector[0], vector[1], vector[2]};
  const Quaternion rotated = multiply(
    multiply(unit_quaternion, pure), conjugate(unit_quaternion));
  return {rotated[1], rotated[2], rotated[3]};
}

Vector3 cross(const Vector3 & lhs, const Vector3 & rhs)
{
  return {
    lhs[1] * rhs[2] - lhs[2] * rhs[1],
    lhs[2] * rhs[0] - lhs[0] * rhs[2],
    lhs[0] * rhs[1] - lhs[1] * rhs[0],
  };
}

Vector3 add(const Vector3 & lhs, const Vector3 & rhs)
{
  return {lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]};
}

}  // namespace

RigidBodyState transform_rigid_body_state(
  const RigidBodyState & parent,
  const FixedRigidTransform & transform)
{
  if (!finite(parent.position_world) || !finite(parent.linear_velocity_world) ||
    !finite(parent.angular_velocity_body) || !finite(transform.translation_body))
  {
    throw std::invalid_argument("rigid-body state and fixed translation must be finite");
  }

  const Quaternion world_from_parent = normalize(parent.orientation_wxyz);
  const Quaternion parent_from_child = normalize(transform.rotation_wxyz);

  RigidBodyState child;
  child.orientation_wxyz = normalize(multiply(world_from_parent, parent_from_child));

  const Vector3 world_offset = rotate(world_from_parent, transform.translation_body);
  child.position_world = add(parent.position_world, world_offset);

  // ω×r 先在父 body frame 中计算，再旋转到 world；这样保持 MotionSample 既有
  // “线速度 world ENU、角速度 body frame”契约。
  const Vector3 lever_arm_velocity_parent = cross(
    parent.angular_velocity_body, transform.translation_body);
  child.linear_velocity_world = add(
    parent.linear_velocity_world,
    rotate(world_from_parent, lever_arm_velocity_parent));

  child.angular_velocity_body = rotate(
    conjugate(parent_from_child), parent.angular_velocity_body);
  return child;
}

}  // namespace moving_deck_sim
