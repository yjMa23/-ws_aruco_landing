#include "moving_deck_sim/gnss_sensor_model.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace moving_deck_sim
{
namespace
{

constexpr std::int64_t kMillisecondNs = 1000000;
constexpr double kPositionToleranceDeg = 1e-10;
constexpr double kAltitudeToleranceM = 1e-6;

GnssSensorParameters ideal_parameters()
{
  GnssSensorParameters parameters;
  parameters.publish_rate_hz = 5.0;
  parameters.horizontal_noise_std_m = 0.0;
  parameters.vertical_noise_std_m = 0.0;
  parameters.velocity_noise_std_mps = 0.0;
  parameters.latency_s = 0.0;
  parameters.packet_drop_probability = 0.0;
  parameters.random_seed = 7;
  return parameters;
}

DeckStateSample make_sample(
  std::int64_t timestamp_ns,
  const std::array<double, 3> & position_enu_m = {0.0, 0.0, 0.0},
  const std::array<double, 3> & velocity_enu_mps = {0.0, 0.0, 0.0})
{
  DeckStateSample sample;
  sample.timestamp_ns = timestamp_ns;
  sample.position_enu_m = position_enu_m;
  sample.velocity_enu_mps = velocity_enu_mps;
  return sample;
}

void expect_measurements_equal(
  const std::vector<GnssMeasurement> & lhs,
  const std::vector<GnssMeasurement> & rhs)
{
  ASSERT_EQ(lhs.size(), rhs.size());
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    EXPECT_EQ(lhs[index].sample_timestamp_ns, rhs[index].sample_timestamp_ns);
    EXPECT_EQ(lhs[index].release_timestamp_ns, rhs[index].release_timestamp_ns);
    EXPECT_DOUBLE_EQ(lhs[index].latitude_deg, rhs[index].latitude_deg);
    EXPECT_DOUBLE_EQ(lhs[index].longitude_deg, rhs[index].longitude_deg);
    EXPECT_DOUBLE_EQ(lhs[index].altitude_m, rhs[index].altitude_m);
    EXPECT_EQ(lhs[index].velocity_enu_mps, rhs[index].velocity_enu_mps);
  }
}

TEST(GnssSensorModelTest, IdealOriginAndAxesHaveExpectedSigns)
{
  const GnssSensorParameters parameters = ideal_parameters();
  GnssSensorModel model(parameters);

  const auto origin = model.update(make_sample(0));
  ASSERT_EQ(origin.size(), 1U);
  EXPECT_NEAR(origin.front().latitude_deg, parameters.reference_latitude_deg,
    kPositionToleranceDeg);
  EXPECT_NEAR(origin.front().longitude_deg, parameters.reference_longitude_deg,
    kPositionToleranceDeg);
  EXPECT_NEAR(origin.front().altitude_m, parameters.reference_elevation_m,
    kAltitudeToleranceM);

  const auto east = model.update(make_sample(200 * kMillisecondNs, {1.0, 0.0, 0.0}));
  ASSERT_EQ(east.size(), 1U);
  EXPECT_GT(east.front().longitude_deg, parameters.reference_longitude_deg);

  const auto north = model.update(make_sample(400 * kMillisecondNs, {0.0, 1.0, 0.0}));
  ASSERT_EQ(north.size(), 1U);
  EXPECT_GT(north.front().latitude_deg, parameters.reference_latitude_deg);

  const auto up = model.update(make_sample(600 * kMillisecondNs, {0.0, 0.0, 1.0}));
  ASSERT_EQ(up.size(), 1U);
  EXPECT_GT(up.front().altitude_m, parameters.reference_elevation_m);
}

TEST(GnssSensorModelTest, SamplesAtConfiguredRate)
{
  GnssSensorModel model(ideal_parameters());

  EXPECT_EQ(model.update(make_sample(0)).size(), 1U);
  EXPECT_TRUE(model.update(make_sample(100 * kMillisecondNs)).empty());
  EXPECT_EQ(model.update(make_sample(200 * kMillisecondNs)).size(), 1U);
  EXPECT_TRUE(model.update(make_sample(399 * kMillisecondNs)).empty());
  EXPECT_EQ(model.update(make_sample(400 * kMillisecondNs)).size(), 1U);
}

TEST(GnssSensorModelTest, FixedLatencyPreservesSampleTimeAndOrder)
{
  GnssSensorParameters parameters = ideal_parameters();
  parameters.publish_rate_hz = 10.0;
  parameters.latency_s = 0.15;
  GnssSensorModel model(parameters);

  EXPECT_TRUE(model.update(make_sample(0, {0.0, 0.0, 0.0})).empty());
  EXPECT_TRUE(model.update(make_sample(100 * kMillisecondNs, {1.0, 0.0, 0.0})).empty());

  const auto first = model.update(make_sample(150 * kMillisecondNs));
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first.front().sample_timestamp_ns, 0);
  EXPECT_EQ(first.front().release_timestamp_ns, 150 * kMillisecondNs);

  const auto second = model.update(make_sample(250 * kMillisecondNs));
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second.front().sample_timestamp_ns, 100 * kMillisecondNs);
  EXPECT_EQ(second.front().release_timestamp_ns, 250 * kMillisecondNs);
}

TEST(GnssSensorModelTest, FixedSeedAndResetAreDeterministic)
{
  GnssSensorParameters parameters = ideal_parameters();
  parameters.horizontal_noise_std_m = 0.8;
  parameters.vertical_noise_std_m = 1.5;
  parameters.velocity_noise_std_mps = 0.1;
  parameters.random_seed = 42;

  GnssSensorModel first_model(parameters);
  GnssSensorModel second_model(parameters);
  const DeckStateSample first_sample = make_sample(
    0, {4.0, -2.0, 3.0}, {0.4, -0.2, 0.1});
  const DeckStateSample second_sample = make_sample(
    200 * kMillisecondNs, {4.1, -2.0, 3.0}, {0.4, -0.2, 0.1});

  const auto first_a = first_model.update(first_sample);
  const auto first_b = second_model.update(first_sample);
  const auto second_a = first_model.update(second_sample);
  const auto second_b = second_model.update(second_sample);
  expect_measurements_equal(first_a, first_b);
  expect_measurements_equal(second_a, second_b);

  first_model.reset();
  expect_measurements_equal(first_a, first_model.update(first_sample));
  expect_measurements_equal(second_a, first_model.update(second_sample));
}

TEST(GnssSensorModelTest, PacketDropProbabilityBoundariesAreHonored)
{
  GnssSensorParameters keep_parameters = ideal_parameters();
  keep_parameters.packet_drop_probability = 0.0;
  EXPECT_EQ(GnssSensorModel(keep_parameters).update(make_sample(0)).size(), 1U);

  GnssSensorParameters drop_parameters = ideal_parameters();
  drop_parameters.packet_drop_probability = 1.0;
  EXPECT_TRUE(GnssSensorModel(drop_parameters).update(make_sample(0)).empty());
}

TEST(GnssSensorModelTest, InvalidGroundTruthDoesNotGenerateMeasurement)
{
  GnssSensorModel model(ideal_parameters());
  DeckStateSample invalid = make_sample(0);
  invalid.position_enu_m[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(model.update(invalid).empty());

  const auto valid = model.update(make_sample(200 * kMillisecondNs));
  EXPECT_EQ(valid.size(), 1U);
  EXPECT_TRUE(model.update(make_sample(-1)).empty());
}

TEST(GnssSensorModelTest, ResetClearsDelayedMeasurements)
{
  GnssSensorParameters parameters = ideal_parameters();
  parameters.latency_s = 0.2;
  GnssSensorModel model(parameters);

  EXPECT_TRUE(model.update(make_sample(0, {5.0, 0.0, 0.0})).empty());
  model.reset();
  EXPECT_TRUE(model.update(make_sample(0, {-5.0, 0.0, 0.0})).empty());

  const auto released = model.update(make_sample(200 * kMillisecondNs));
  ASSERT_EQ(released.size(), 1U);
  EXPECT_LT(released.front().longitude_deg, parameters.reference_longitude_deg);
}

TEST(GnssSensorModelTest, CovarianceMatchesConfiguredNoise)
{
  GnssSensorParameters parameters = ideal_parameters();
  parameters.horizontal_noise_std_m = 0.8;
  parameters.vertical_noise_std_m = 1.5;
  const GnssSensorModel model(parameters);

  const auto covariance = model.position_covariance_enu_m2();
  EXPECT_DOUBLE_EQ(covariance[0], 0.64);
  EXPECT_DOUBLE_EQ(covariance[4], 0.64);
  EXPECT_DOUBLE_EQ(covariance[8], 2.25);
  for (const std::size_t index : {1U, 2U, 3U, 5U, 6U, 7U}) {
    EXPECT_DOUBLE_EQ(covariance[index], 0.0);
  }
}

TEST(GnssSensorModelTest, RejectsInvalidParameters)
{
  GnssSensorParameters parameters = ideal_parameters();
  parameters.publish_rate_hz = 0.0;
  EXPECT_THROW((void)GnssSensorModel{parameters}, std::invalid_argument);

  parameters = ideal_parameters();
  parameters.horizontal_noise_std_m = -1.0;
  EXPECT_THROW((void)GnssSensorModel{parameters}, std::invalid_argument);

  parameters = ideal_parameters();
  parameters.latency_s = std::numeric_limits<double>::infinity();
  EXPECT_THROW((void)GnssSensorModel{parameters}, std::invalid_argument);

  parameters = ideal_parameters();
  parameters.packet_drop_probability = 1.1;
  EXPECT_THROW((void)GnssSensorModel{parameters}, std::invalid_argument);

  parameters = ideal_parameters();
  parameters.reference_latitude_deg = 91.0;
  EXPECT_THROW((void)GnssSensorModel{parameters}, std::invalid_argument);
}

}  // namespace
}  // namespace moving_deck_sim
