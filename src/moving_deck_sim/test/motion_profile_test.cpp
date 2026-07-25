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

TEST(MotionProfileTest, HeaveProfileMatchesAnalyticQuarterPeriod)
{
  MotionParameters parameters;
  parameters.scenario = Scenario::kHeave;
  parameters.initial_position_enu = {1.0, -2.0, 2.0};
  parameters.amplitude_z_m = 0.3;
  parameters.period_z_s = 8.0;
  const MotionProfile profile(parameters);

  const MotionSample start = profile.sample(0.0);
  EXPECT_NEAR(start.position_enu[2], 2.0, kTolerance);
  EXPECT_NEAR(start.velocity_enu[2], 0.3 * 2.0 * std::acos(-1.0) / 8.0, kTolerance);

  const MotionSample quarter = profile.sample(2.0);
  EXPECT_NEAR(quarter.position_enu[0], 1.0, kTolerance);
  EXPECT_NEAR(quarter.position_enu[1], -2.0, kTolerance);
  EXPECT_NEAR(quarter.position_enu[2], 2.3, kTolerance);
  EXPECT_NEAR(quarter.velocity_enu[2], 0.0, kTolerance);
  EXPECT_EQ(quarter.orientation_rpy_enu, (std::array<double, 3>{0.0, 0.0, 0.0}));
}

TEST(MotionProfileTest, RollPitchProfileMatchesAnalyticStateAndBodyRates)
{
  MotionParameters parameters;
  parameters.scenario = Scenario::kRollPitch;
  parameters.amplitude_rpy_rad = {0.1, 0.2, 0.0};
  parameters.period_rpy_s = {8.0, 4.0, 10.0};
  const MotionProfile profile(parameters);

  const MotionSample start = profile.sample(0.0);
  const double roll_rate = 0.1 * 2.0 * std::acos(-1.0) / 8.0;
  const double pitch_rate = 0.2 * 2.0 * std::acos(-1.0) / 4.0;
  EXPECT_NEAR(start.orientation_rpy_enu[0], 0.0, kTolerance);
  EXPECT_NEAR(start.orientation_rpy_enu[1], 0.0, kTolerance);
  EXPECT_NEAR(start.angular_velocity_body[0], roll_rate, kTolerance);
  EXPECT_NEAR(start.angular_velocity_body[1], pitch_rate, kTolerance);
  EXPECT_NEAR(start.angular_velocity_body[2], 0.0, kTolerance);

  const MotionSample roll_quarter = profile.sample(2.0);
  EXPECT_NEAR(roll_quarter.orientation_rpy_enu[0], 0.1, kTolerance);
  EXPECT_NEAR(roll_quarter.orientation_rpy_enu[1], 0.0, kTolerance);
  EXPECT_NEAR(roll_quarter.angular_velocity_body[0], 0.0, kTolerance);
  EXPECT_NEAR(
    roll_quarter.angular_velocity_body[2],
    -(-pitch_rate) * std::sin(0.1),
    kTolerance);
}

TEST(MotionProfileTest, CombinedProfileEnablesAllConfiguredComponents)
{
  MotionParameters parameters;
  parameters.scenario = Scenario::kCombined;
  parameters.initial_position_enu = {0.0, 0.0, 2.0};
  parameters.amplitude_xy = {1.0, 0.5};
  parameters.period_xy = {8.0, 8.0};
  parameters.amplitude_z_m = 0.3;
  parameters.period_z_s = 8.0;
  parameters.amplitude_rpy_rad = {0.1, 0.05, 0.0};
  parameters.period_rpy_s = {8.0, 8.0, 8.0};
  const MotionSample quarter = MotionProfile(parameters).sample(2.0);

  EXPECT_NEAR(quarter.position_enu[0], 1.0, kTolerance);
  EXPECT_NEAR(quarter.position_enu[1], 0.5, kTolerance);
  EXPECT_NEAR(quarter.position_enu[2], 2.3, kTolerance);
  EXPECT_NEAR(quarter.orientation_rpy_enu[0], 0.1, kTolerance);
  EXPECT_NEAR(quarter.orientation_rpy_enu[1], 0.05, kTolerance);
  EXPECT_NEAR(quarter.velocity_enu[0], 0.0, kTolerance);
  EXPECT_NEAR(quarter.velocity_enu[1], 0.0, kTolerance);
  EXPECT_NEAR(quarter.velocity_enu[2], 0.0, kTolerance);
}

TEST(MotionProfileTest, ParsesAllSupportedScenarios)
{
  EXPECT_EQ(MotionProfile::parse_scenario("S0_STATIC"), Scenario::kStatic);
  EXPECT_EQ(MotionProfile::parse_scenario("S1_CONSTANT_XY"), Scenario::kConstantXy);
  EXPECT_EQ(MotionProfile::parse_scenario("S2_SINUSOIDAL_XY"), Scenario::kSinusoidalXy);
  EXPECT_EQ(MotionProfile::parse_scenario("S3_HEAVE"), Scenario::kHeave);
  EXPECT_EQ(MotionProfile::parse_scenario("S4_ROLL_PITCH"), Scenario::kRollPitch);
  EXPECT_EQ(MotionProfile::parse_scenario("S5_COMBINED"), Scenario::kCombined);
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
  parameters.amplitude_z_m = -0.1;
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.amplitude_z_m = 0.0;
  parameters.period_z_s = 0.0;
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.period_z_s = 1.0;
  parameters.amplitude_rpy_rad[0] = 0.5 * std::acos(-1.0);
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.amplitude_rpy_rad[0] = 0.0;
  parameters.period_rpy_s[1] = 0.0;
  EXPECT_THROW(MotionProfile{parameters}, std::invalid_argument);

  parameters.period_rpy_s[1] = 1.0;
  const MotionProfile profile(parameters);
  EXPECT_THROW(profile.sample(-0.1), std::invalid_argument);
  EXPECT_THROW(
    profile.sample(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

}  // namespace
}  // namespace moving_deck_sim
