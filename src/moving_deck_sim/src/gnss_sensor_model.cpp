#include "moving_deck_sim/gnss_sensor_model.hpp"

#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace moving_deck_sim
{
namespace
{

constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr double kMaximumSupportedLatencyS = 3600.0;

bool is_finite_array(const std::array<double, 3> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

gz::math::Angle angle_from_degrees(double degrees)
{
  gz::math::Angle angle;
  angle.SetDegree(degrees);
  return angle;
}

}  // namespace

GnssSensorModel::GnssSensorModel(const GnssSensorParameters & parameters)
: parameters_(validate_parameters(parameters)),
  spherical_coordinates_(
    gz::math::SphericalCoordinates::EARTH_WGS84,
    angle_from_degrees(parameters_.reference_latitude_deg),
    angle_from_degrees(parameters_.reference_longitude_deg),
    parameters_.reference_elevation_m,
    gz::math::Angle::Zero),
  sampling_period_ns_(
    static_cast<std::int64_t>(std::llround(kNanosecondsPerSecond / parameters_.publish_rate_hz))),
  latency_ns_(static_cast<std::int64_t>(std::llround(
      kNanosecondsPerSecond * parameters_.latency_s))),
  random_engine_(parameters_.random_seed)
{
  if (sampling_period_ns_ <= 0) {
    throw std::invalid_argument("publish_rate_hz produces an invalid sampling period");
  }
}

std::vector<GnssMeasurement> GnssSensorModel::update(const DeckStateSample & sample)
{
  if (sample.timestamp_ns < 0) {
    return {};
  }

  if (have_last_input_timestamp_ && sample.timestamp_ns < last_input_timestamp_ns_) {
    // Gazebo 重启或仿真时间回拨时，旧延迟队列和采样相位均不可继续使用。
    reset();
  }
  last_input_timestamp_ns_ = sample.timestamp_ns;
  have_last_input_timestamp_ = true;

  std::vector<GnssMeasurement> ready = release_ready(sample.timestamp_ns);

  if (!have_sampling_schedule_) {
    next_sample_timestamp_ns_ = sample.timestamp_ns;
    have_sampling_schedule_ = true;
  }

  if (sample.timestamp_ns < next_sample_timestamp_ns_) {
    return ready;
  }

  const std::int64_t elapsed_ns = sample.timestamp_ns - next_sample_timestamp_ns_;
  const std::int64_t skipped_periods = elapsed_ns / sampling_period_ns_;
  const std::int64_t periods_to_advance = skipped_periods + 1;
  if (periods_to_advance <=
    (std::numeric_limits<std::int64_t>::max() - next_sample_timestamp_ns_) /
    sampling_period_ns_)
  {
    next_sample_timestamp_ns_ += periods_to_advance * sampling_period_ns_;
  } else {
    next_sample_timestamp_ns_ = std::numeric_limits<std::int64_t>::max();
  }

  const bool state_valid = sample_state_is_finite(sample);
  const bool timestamp_supports_latency =
    sample.timestamp_ns <= std::numeric_limits<std::int64_t>::max() - latency_ns_;
  const bool dropped = parameters_.packet_drop_probability >= 1.0 ||
    (parameters_.packet_drop_probability > 0.0 &&
    uniform_distribution_(random_engine_) < parameters_.packet_drop_probability);

  if (state_valid && timestamp_supports_latency && !dropped) {
    delayed_measurements_.push_back(make_measurement(sample));
  }

  std::vector<GnssMeasurement> newly_ready = release_ready(sample.timestamp_ns);
  ready.insert(ready.end(), newly_ready.begin(), newly_ready.end());
  return ready;
}

void GnssSensorModel::reset()
{
  delayed_measurements_.clear();
  random_engine_.seed(parameters_.random_seed);
  standard_normal_distribution_.reset();
  uniform_distribution_.reset();
  next_sample_timestamp_ns_ = 0;
  last_input_timestamp_ns_ = 0;
  have_sampling_schedule_ = false;
  have_last_input_timestamp_ = false;
}

std::array<double, 9> GnssSensorModel::position_covariance_enu_m2() const
{
  const double horizontal_variance =
    parameters_.horizontal_noise_std_m * parameters_.horizontal_noise_std_m;
  const double vertical_variance =
    parameters_.vertical_noise_std_m * parameters_.vertical_noise_std_m;
  return {
    horizontal_variance, 0.0, 0.0,
    0.0, horizontal_variance, 0.0,
    0.0, 0.0, vertical_variance};
}

const GnssSensorParameters & GnssSensorModel::parameters() const
{
  return parameters_;
}

GnssSensorParameters GnssSensorModel::validate_parameters(
  const GnssSensorParameters & parameters)
{
  if (!std::isfinite(parameters.publish_rate_hz) || parameters.publish_rate_hz <= 0.0 ||
    parameters.publish_rate_hz > kNanosecondsPerSecond)
  {
    throw std::invalid_argument("publish_rate_hz must be finite and in (0, 1e9]");
  }

  const std::array<double, 3> noise_values{
    parameters.horizontal_noise_std_m,
    parameters.vertical_noise_std_m,
    parameters.velocity_noise_std_mps};
  if (!is_finite_array(noise_values) ||
    std::any_of(noise_values.begin(), noise_values.end(), [](double value) {return value < 0.0;}))
  {
    throw std::invalid_argument("GNSS noise standard deviations must be finite and non-negative");
  }

  if (!std::isfinite(parameters.latency_s) || parameters.latency_s < 0.0 ||
    parameters.latency_s > kMaximumSupportedLatencyS)
  {
    throw std::invalid_argument("latency_s must be finite and in [0, 3600]");
  }
  if (!std::isfinite(parameters.packet_drop_probability) ||
    parameters.packet_drop_probability < 0.0 || parameters.packet_drop_probability > 1.0)
  {
    throw std::invalid_argument("packet_drop_probability must be in [0, 1]");
  }
  if (!std::isfinite(parameters.reference_latitude_deg) ||
    parameters.reference_latitude_deg < -90.0 || parameters.reference_latitude_deg > 90.0)
  {
    throw std::invalid_argument("reference_latitude_deg must be in [-90, 90]");
  }
  if (!std::isfinite(parameters.reference_longitude_deg) ||
    parameters.reference_longitude_deg < -180.0 || parameters.reference_longitude_deg > 180.0)
  {
    throw std::invalid_argument("reference_longitude_deg must be in [-180, 180]");
  }
  if (!std::isfinite(parameters.reference_elevation_m)) {
    throw std::invalid_argument("reference_elevation_m must be finite");
  }
  return parameters;
}

bool GnssSensorModel::sample_state_is_finite(const DeckStateSample & sample)
{
  return is_finite_array(sample.position_enu_m) && is_finite_array(sample.velocity_enu_mps);
}

GnssMeasurement GnssSensorModel::make_measurement(const DeckStateSample & sample)
{
  const gz::math::Vector3d noisy_position_enu(
    sample.position_enu_m[0] + gaussian_noise(parameters_.horizontal_noise_std_m),
    sample.position_enu_m[1] + gaussian_noise(parameters_.horizontal_noise_std_m),
    sample.position_enu_m[2] + gaussian_noise(parameters_.vertical_noise_std_m));

  // LOCAL2 在 heading=0 且 world_frame_orientation=ENU 时与 Gazebo world ENU 一致；
  // SPHERICAL 输出的纬度和经度为弧度，海拔为米。
  const gz::math::Vector3d spherical = spherical_coordinates_.PositionTransform(
    noisy_position_enu,
    gz::math::SphericalCoordinates::LOCAL2,
    gz::math::SphericalCoordinates::SPHERICAL);

  GnssMeasurement measurement;
  measurement.sample_timestamp_ns = sample.timestamp_ns;
  measurement.release_timestamp_ns = sample.timestamp_ns + latency_ns_;
  measurement.latitude_deg = gz::math::Angle(spherical.X()).Degree();
  measurement.longitude_deg = gz::math::Angle(spherical.Y()).Degree();
  measurement.altitude_m = spherical.Z();
  for (std::size_t axis = 0; axis < measurement.velocity_enu_mps.size(); ++axis) {
    measurement.velocity_enu_mps[axis] = sample.velocity_enu_mps[axis] +
      gaussian_noise(parameters_.velocity_noise_std_mps);
  }
  return measurement;
}

std::vector<GnssMeasurement> GnssSensorModel::release_ready(
  std::int64_t current_timestamp_ns)
{
  std::vector<GnssMeasurement> ready;
  while (!delayed_measurements_.empty() &&
    delayed_measurements_.front().release_timestamp_ns <= current_timestamp_ns)
  {
    ready.push_back(delayed_measurements_.front());
    delayed_measurements_.pop_front();
  }
  return ready;
}

double GnssSensorModel::gaussian_noise(double standard_deviation)
{
  if (standard_deviation == 0.0) {
    return 0.0;
  }
  return standard_deviation * standard_normal_distribution_(random_engine_);
}

}  // namespace moving_deck_sim
