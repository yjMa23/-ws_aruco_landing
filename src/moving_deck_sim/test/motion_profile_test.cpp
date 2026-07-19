#include "moving_deck_sim/motion_profile.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace moving_deck_sim
{
namespace
{

constexpr double kTolerance = 1e-9;

TEST(MotionProfileTest, StaticProfileDoesNotMove)
{
  MotionParameters parameters;
  parameters.scenario = Scenario::kStatic;
  parameters.initial_position_enu = {1.0, -2.0, 2.0};
  const MotionSample sample = MotionProfile(parameters).sample(12.0);

  EXPECT_EQ(sample.position_enu, parameters.initial_position_enu);
  EXPECT_EQ(sample.velocity_enu, (std::array<double, 3>{0.0, 0.0, 0.0}));
}

TEST(MotionProfileTest, ConstantProfileMatchesAnalyticState)
{
  MotionParameters parameters;
  parameters.scenario = Scenario::kConstantXy;
  parameters.initial_position_enu = {1.0, -2.0, 2.0};
  parameters.velocity_xy = {0.4, -0.2};
  const MotionSample sample = MotionProfile(parameters).sample(2.5);

  EXPECT_NEAR(sample.position_enu[0], 2.0, kTolerance);
  EXPECT_NEAR(sample.position_enu[1], -2.5, kTolerance);
  EXPECT_NEAR(sample.position_enu[2], 2.0, kTolerance);
  EXPECT_NEAR(sample.velocity_enu[0], 0.4, kTolerance);
  EXPECT_NEAR(sample.velocity_enu[1], -0.2, kTolerance);
}

TEST(MotionProfileTest, SinusoidalProfileMatchesQuarterAndFullPeriod)
{
  MotionParameters parameters;
  parameters.scenario = Scenario::kSinusoidalXy;
  parameters.initial_position_enu = {0.0, 0.0, 2.0};
  parameters.amplitude_xy = {1.0, 0.5};
  parameters.period_xy = {10.0, 10.0};
  const MotionProfile profile(parameters);

  const MotionSample quarter = profile.sample(2.5);
  EXPECT_NEAR(quarter.position_enu[0], 1.0, kTolerance);
  EXPECT_NEAR(quarter.position_enu[1], 0.5, kTolerance);
  EXPECT_NEAR(quarter.velocity_enu[0], 0.0, kTolerance);
  EXPECT_NEAR(quarter.velocity_enu[1], 0.0, kTolerance);

  const MotionSample full = profile.sample(10.0);
  EXPECT_NEAR(full.position_enu[0], 0.0, kTolerance);
  EXPECT_NEAR(full.position_enu[1], 0.0, kTolerance);
  EXPECT_NEAR(full.velocity_enu[0], 2.0 * std::acos(-1.0) / 10.0, kTolerance);
  EXPECT_NEAR(full.velocity_enu[1], std::acos(-1.0) / 10.0, kTolerance);
}

TEST(MotionProfileTest, ResetReturnsToDeterministicInitialState)
{
  MotionParameters parameters;
  parameters.scenario = Scenario::kSinusoidalXy;
  parameters.initial_position_enu = {1.0, -2.0, 2.0};
  parameters.amplitude_xy = {1.0, 0.5};
  parameters.period_xy = {10.0, 6.0};
  const MotionProfile profile(parameters);

  const MotionSample first_start = profile.sample(0.0);
  const MotionSample reset_start = profile.sample(0.0);

  EXPECT_EQ(first_start.position_enu, parameters.initial_position_enu);
  EXPECT_EQ(reset_start.position_enu, first_start.position_enu);
  EXPECT_EQ(reset_start.velocity_enu, first_start.velocity_enu);
  EXPECT_NEAR(first_start.velocity_enu[0], 2.0 * std::acos(-1.0) / 10.0, kTolerance);
  EXPECT_NEAR(first_start.velocity_enu[1], std::acos(-1.0) / 6.0, kTolerance);
}

TEST(MotionProfileTest, RejectsInvalidInputs)
{
  EXPECT_THROW(MotionProfile::parse_scenario("UNKNOWN"), std::invalid_argument);

  MotionParameters parameters;
  parameters.period_xy[0] = 0.0;
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.period_xy[0] = 1.0;
  parameters.velocity_xy[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.velocity_xy[1] = 0.0;
  parameters.update_rate_hz = 0.0;
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.update_rate_hz = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.update_rate_hz = 50.0;
  const MotionProfile profile(parameters);
  EXPECT_THROW(profile.sample(-0.1), std::invalid_argument);
  EXPECT_THROW(
    profile.sample(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

}  // namespace
}  // namespace moving_deck_sim
