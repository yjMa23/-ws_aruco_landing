#include "moving_deck_sim/motion_profile.hpp"

#include <cmath>
#include <stdexcept>

namespace moving_deck_sim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaximumAttitudeAmplitudeRad = 0.5 * kPi;

template<std::size_t Size>
bool all_finite(const std::array<double, Size> & values)
{
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool uses_sinusoidal_xy(Scenario scenario)
{
  return scenario == Scenario::kSinusoidalXy || scenario == Scenario::kCombined;
}

bool uses_heave(Scenario scenario)
{
  return scenario == Scenario::kHeave || scenario == Scenario::kCombined;
}

bool uses_attitude_motion(Scenario scenario)
{
  return scenario == Scenario::kRollPitch || scenario == Scenario::kCombined;
}

std::array<double, 3> euler_rates_to_body_angular_velocity(
  const std::array<double, 3> & rpy,
  const std::array<double, 3> & rpy_rates)
{
  const double roll = rpy[0];
  const double pitch = rpy[1];
  const double roll_rate = rpy_rates[0];
  const double pitch_rate = rpy_rates[1];
  const double yaw_rate = rpy_rates[2];

  return {
    roll_rate - yaw_rate * std::sin(pitch),
    pitch_rate * std::cos(roll) + yaw_rate * std::sin(roll) * std::cos(pitch),
    -pitch_rate * std::sin(roll) + yaw_rate * std::cos(roll) * std::cos(pitch),
  };
}

}  // namespace

Scenario MotionProfile::parse_scenario(const std::string & name)
{
  if (name == "S0_STATIC") {
    return Scenario::kStatic;
  }
  if (name == "S1_CONSTANT_XY") {
    return Scenario::kConstantXy;
  }
  if (name == "S2_SINUSOIDAL_XY") {
    return Scenario::kSinusoidalXy;
  }
  if (name == "S3_HEAVE") {
    return Scenario::kHeave;
  }
  if (name == "S4_ROLL_PITCH") {
    return Scenario::kRollPitch;
  }
  if (name == "S5_COMBINED") {
    return Scenario::kCombined;
  }
  throw std::invalid_argument("unsupported scenario: " + name);
}

MotionProfile::MotionProfile(MotionParameters parameters)
: parameters_(parameters)
{
  if (!all_finite(parameters_.initial_position_enu) ||
    !all_finite(parameters_.velocity_xy) ||
    !all_finite(parameters_.amplitude_xy) ||
    !all_finite(parameters_.period_xy) ||
    !std::isfinite(parameters_.amplitude_z_m) ||
    !std::isfinite(parameters_.period_z_s) ||
    !all_finite(parameters_.initial_rpy_rad) ||
    !all_finite(parameters_.amplitude_rpy_rad) ||
    !all_finite(parameters_.period_rpy_s))
  {
    throw std::invalid_argument("motion parameters must be finite");
  }
  if (parameters_.period_xy[0] <= 0.0 || parameters_.period_xy[1] <= 0.0 ||
    parameters_.period_z_s <= 0.0)
  {
    throw std::invalid_argument("translational sinusoidal periods must be positive");
  }
  for (std::size_t axis = 0; axis < parameters_.period_rpy_s.size(); ++axis) {
    if (parameters_.period_rpy_s[axis] <= 0.0) {
      throw std::invalid_argument("attitude sinusoidal periods must be positive");
    }
    if (std::abs(parameters_.amplitude_rpy_rad[axis]) >=
      kMaximumAttitudeAmplitudeRad)
    {
      throw std::invalid_argument("attitude amplitudes must stay within (-pi/2, pi/2)");
    }
  }
  if (parameters_.amplitude_z_m < 0.0) {
    throw std::invalid_argument("heave amplitude must be non-negative");
  }
  if (!std::isfinite(parameters_.update_rate_hz) || parameters_.update_rate_hz <= 0.0) {
    throw std::invalid_argument("update rate must be finite and positive");
  }
}

MotionSample MotionProfile::sample(const double elapsed_s) const
{
  if (!std::isfinite(elapsed_s) || elapsed_s < 0.0) {
    throw std::invalid_argument("elapsed time must be finite and non-negative");
  }

  MotionSample result;
  result.position_enu = parameters_.initial_position_enu;
  result.orientation_rpy_enu = parameters_.initial_rpy_rad;

  if (parameters_.scenario == Scenario::kConstantXy) {
    for (std::size_t axis = 0; axis < 2; ++axis) {
      result.position_enu[axis] += parameters_.velocity_xy[axis] * elapsed_s;
      result.velocity_enu[axis] = parameters_.velocity_xy[axis];
    }
  } else if (uses_sinusoidal_xy(parameters_.scenario)) {
    for (std::size_t axis = 0; axis < 2; ++axis) {
      const double angular_frequency = 2.0 * kPi / parameters_.period_xy[axis];
      const double phase = angular_frequency * elapsed_s;
      result.position_enu[axis] += parameters_.amplitude_xy[axis] * std::sin(phase);
      result.velocity_enu[axis] =
        parameters_.amplitude_xy[axis] * angular_frequency * std::cos(phase);
    }
  }

  if (uses_heave(parameters_.scenario)) {
    const double angular_frequency = 2.0 * kPi / parameters_.period_z_s;
    const double phase = angular_frequency * elapsed_s;
    result.position_enu[2] += parameters_.amplitude_z_m * std::sin(phase);
    result.velocity_enu[2] =
      parameters_.amplitude_z_m * angular_frequency * std::cos(phase);
  }

  if (uses_attitude_motion(parameters_.scenario)) {
    std::array<double, 3> rpy_rates{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const double angular_frequency = 2.0 * kPi / parameters_.period_rpy_s[axis];
      const double phase = angular_frequency * elapsed_s;
      result.orientation_rpy_enu[axis] +=
        parameters_.amplitude_rpy_rad[axis] * std::sin(phase);
      rpy_rates[axis] =
        parameters_.amplitude_rpy_rad[axis] * angular_frequency * std::cos(phase);
    }
    result.angular_velocity_body = euler_rates_to_body_angular_velocity(
      result.orientation_rpy_enu, rpy_rates);
  }

  return result;
}

}  // namespace moving_deck_sim
