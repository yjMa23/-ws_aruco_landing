#include "moving_deck_sim/motion_profile.hpp"

#include <cmath>
#include <stdexcept>

namespace moving_deck_sim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

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
  throw std::invalid_argument("unsupported scenario: " + name);
}

MotionProfile::MotionProfile(MotionParameters parameters)
: parameters_(parameters)
{
  if (!all_finite(parameters_.initial_position_enu) ||
    !all_finite(parameters_.velocity_xy) ||
    !all_finite(parameters_.amplitude_xy) ||
    !all_finite(parameters_.period_xy))
  {
    throw std::invalid_argument("motion parameters must be finite");
  }
  if (parameters_.period_xy[0] <= 0.0 || parameters_.period_xy[1] <= 0.0) {
    throw std::invalid_argument("sinusoidal periods must be positive");
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

  if (parameters_.scenario == Scenario::kConstantXy) {
    for (std::size_t axis = 0; axis < 2; ++axis) {
      result.position_enu[axis] += parameters_.velocity_xy[axis] * elapsed_s;
      result.velocity_enu[axis] = parameters_.velocity_xy[axis];
    }
  } else if (parameters_.scenario == Scenario::kSinusoidalXy) {
    for (std::size_t axis = 0; axis < 2; ++axis) {
      const double angular_frequency = 2.0 * kPi / parameters_.period_xy[axis];
      const double phase = angular_frequency * elapsed_s;
      result.position_enu[axis] += parameters_.amplitude_xy[axis] * std::sin(phase);
      result.velocity_enu[axis] =
        parameters_.amplitude_xy[axis] * angular_frequency * std::cos(phase);
    }
  }

  return result;
}

}  // namespace moving_deck_sim
