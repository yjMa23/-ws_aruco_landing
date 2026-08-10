#include "moving_deck_sim/rigid_body_kinematics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace moving_deck_sim
{
namespace
{

constexpr double kTolerance = 1e-9;
constexpr double kSqrtHalf = 0.70710678118654752440;

void expect_vector_near(
  const std::array<double, 3> & actual,
  const std::array<double, 3> & expected)
{
  for (std::size_t i = 0; i < actual.size(); ++i) {
    EXPECT_NEAR(actual[i], expected[i], kTolerance);
  }
}

TEST(RigidBodyKinematicsTest, ZeroOffsetPreservesState)
{
  RigidBodyState vessel;
  vessel.position_world = {1.0, 2.0, 3.0};
  vessel.orientation_wxyz = {1.0, 0.0, 0.0, 0.0};
  vessel.linear_velocity_world = {0.4, -0.2, 0.1};
  vessel.angular_velocity_body = {0.1, 0.2, 0.3};

  const auto deck = transform_rigid_body_state(vessel, FixedRigidTransform{});
  expect_vector_near(deck.position_world, vessel.position_world);
  expect_vector_near(deck.linear_velocity_world, vessel.linear_velocity_world);
  expect_vector_near(deck.angular_velocity_body, vessel.angular_velocity_body);
}

TEST(RigidBodyKinematicsTest, PureTranslationAddsRotatedOffset)
{
  RigidBodyState vessel;
  vessel.position_world = {3.0, -1.0, 0.0};
  FixedRigidTransform transform;
  transform.translation_body = {0.0, 0.0, 2.0};

  const auto deck = transform_rigid_body_state(vessel, transform);
  expect_vector_near(deck.position_world, {3.0, -1.0, 2.0});
}

TEST(RigidBodyKinematicsTest, PureRollMovesDeckThroughLeverArm)
{
  RigidBodyState vessel;
  vessel.orientation_wxyz = {kSqrtHalf, kSqrtHalf, 0.0, 0.0};
  FixedRigidTransform transform;
  transform.translation_body = {0.0, 0.0, 2.0};

  const auto deck = transform_rigid_body_state(vessel, transform);
  expect_vector_near(deck.position_world, {0.0, -2.0, 0.0});
}

TEST(RigidBodyKinematicsTest, PurePitchMovesDeckThroughLeverArm)
{
  RigidBodyState vessel;
  vessel.orientation_wxyz = {kSqrtHalf, 0.0, kSqrtHalf, 0.0};
  FixedRigidTransform transform;
  transform.translation_body = {0.0, 0.0, 2.0};

  const auto deck = transform_rigid_body_state(vessel, transform);
  expect_vector_near(deck.position_world, {2.0, 0.0, 0.0});
}

TEST(RigidBodyKinematicsTest, PureYawKeepsVerticalDeckOffset)
{
  RigidBodyState vessel;
  vessel.orientation_wxyz = {kSqrtHalf, 0.0, 0.0, kSqrtHalf};
  FixedRigidTransform transform;
  transform.translation_body = {0.0, 0.0, 2.0};

  const auto deck = transform_rigid_body_state(vessel, transform);
  expect_vector_near(deck.position_world, {0.0, 0.0, 2.0});
}

TEST(RigidBodyKinematicsTest, AngularVelocityAddsLeverArmVelocity)
{
  RigidBodyState vessel;
  vessel.linear_velocity_world = {0.5, 0.0, 0.0};
  vessel.angular_velocity_body = {1.0, 0.0, 0.0};
  FixedRigidTransform transform;
  transform.translation_body = {0.0, 0.0, 2.0};

  const auto deck = transform_rigid_body_state(vessel, transform);
  expect_vector_near(deck.linear_velocity_world, {0.5, -2.0, 0.0});
}

TEST(RigidBodyKinematicsTest, FixedRotationTransformsDeckAngularVelocity)
{
  RigidBodyState vessel;
  vessel.angular_velocity_body = {1.0, 0.0, 0.0};
  FixedRigidTransform transform;
  transform.rotation_wxyz = {kSqrtHalf, 0.0, 0.0, kSqrtHalf};

  const auto deck = transform_rigid_body_state(vessel, transform);
  expect_vector_near(deck.angular_velocity_body, {0.0, -1.0, 0.0});
}

TEST(RigidBodyKinematicsTest, MarineNeutralGeometryPlacesDeckAtTwoMeters)
{
  RigidBodyState vessel;
  vessel.position_world = {0.0, 0.0, 0.2};
  FixedRigidTransform transform;
  transform.translation_body = {0.0, 0.0, 1.8};

  const auto deck = transform_rigid_body_state(vessel, transform);
  EXPECT_NEAR(deck.position_world[2], 2.0, kTolerance);
}

TEST(RigidBodyKinematicsTest, NormalizesParentAndFixedQuaternions)
{
  RigidBodyState vessel;
  vessel.orientation_wxyz = {2.0, 0.0, 0.0, 0.0};
  FixedRigidTransform transform;
  transform.rotation_wxyz = {2.0 * kSqrtHalf, 0.0, 0.0, 2.0 * kSqrtHalf};

  const auto deck = transform_rigid_body_state(vessel, transform);
  EXPECT_NEAR(deck.orientation_wxyz[0], kSqrtHalf, kTolerance);
  EXPECT_NEAR(deck.orientation_wxyz[1], 0.0, kTolerance);
  EXPECT_NEAR(deck.orientation_wxyz[2], 0.0, kTolerance);
  EXPECT_NEAR(deck.orientation_wxyz[3], kSqrtHalf, kTolerance);
}

TEST(RigidBodyKinematicsTest, FixedRotationComposesDeckOrientation)
{
  RigidBodyState vessel;
  vessel.orientation_wxyz = {kSqrtHalf, kSqrtHalf, 0.0, 0.0};
  FixedRigidTransform transform;
  transform.rotation_wxyz = {kSqrtHalf, 0.0, kSqrtHalf, 0.0};

  const auto deck = transform_rigid_body_state(vessel, transform);
  EXPECT_NEAR(deck.orientation_wxyz[0], 0.5, kTolerance);
  EXPECT_NEAR(deck.orientation_wxyz[1], 0.5, kTolerance);
  EXPECT_NEAR(deck.orientation_wxyz[2], 0.5, kTolerance);
  EXPECT_NEAR(deck.orientation_wxyz[3], 0.5, kTolerance);
}

TEST(RigidBodyKinematicsTest, RejectsNonFiniteInputsAndInvalidQuaternion)
{
  RigidBodyState vessel;
  vessel.position_world[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(transform_rigid_body_state(vessel, FixedRigidTransform{}), std::invalid_argument);

  vessel = RigidBodyState{};
  vessel.orientation_wxyz = {0.0, 0.0, 0.0, 0.0};
  EXPECT_THROW(transform_rigid_body_state(vessel, FixedRigidTransform{}), std::invalid_argument);

  vessel = RigidBodyState{};
  FixedRigidTransform transform;
  transform.translation_body[2] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(transform_rigid_body_state(vessel, transform), std::invalid_argument);
}

}  // namespace
}  // namespace moving_deck_sim
