#ifndef MOVING_DECK_SIM__GNSS_SENSOR_MODEL_HPP_
#define MOVING_DECK_SIM__GNSS_SENSOR_MODEL_HPP_

#include <gz/math/SphericalCoordinates.hh>

#include <array>
#include <cstdint>
#include <deque>
#include <random>
#include <vector>

namespace moving_deck_sim
{

/**
 * @brief 船舶 GNSS 传感器模型参数。
 */
struct GnssSensorParameters
{
  double publish_rate_hz{5.0};
  double horizontal_noise_std_m{0.8};
  double vertical_noise_std_m{1.5};
  double velocity_noise_std_mps{0.1};
  double latency_s{0.1};
  double packet_drop_probability{0.0};
  std::uint32_t random_seed{1};
  double reference_latitude_deg{47.397971057728974};
  double reference_longitude_deg{8.546163739800146};
  double reference_elevation_m{0.0};
};

/**
 * @brief 单帧甲板真值输入，位置和速度均采用 Gazebo world ENU。
 */
struct DeckStateSample
{
  std::int64_t timestamp_ns{0};
  std::array<double, 3> position_enu_m{};
  std::array<double, 3> velocity_enu_mps{};
};

/**
 * @brief 经过采样、噪声、延迟和丢包处理后的 GNSS 测量。
 */
struct GnssMeasurement
{
  std::int64_t sample_timestamp_ns{0};
  std::int64_t release_timestamp_ns{0};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  std::array<double, 3> velocity_enu_mps{};
};

/**
 * @brief 将甲板 ENU 真值转换为可重复的船舶 GNSS / 遥测测量。
 *
 * 模型按固定频率采样，先进行概率丢包，再添加固定种子高斯噪声，最后进入
 * 固定延迟队列。输出保留原始采样时间戳，调用者可在发布时识别传感器延迟。
 */
class GnssSensorModel
{
public:
  /**
   * @brief 创建 GNSS 传感器模型并校验参数。
   *
   * @param parameters 采样频率、噪声、延迟、丢包、随机种子和 Gazebo world
   * WGS84 原点。经纬度单位为度，其余距离单位为米。
   * @throws std::invalid_argument 参数非有限、越界或采样周期无法表示时抛出。
   */
  explicit GnssSensorModel(const GnssSensorParameters & parameters);

  /**
   * @brief 输入一帧 world ENU 甲板状态并返回当前时刻可发布的 GNSS 测量。
   *
   * @param sample 甲板位置和速度，坐标系为 Gazebo world ENU；时间戳单位为纳秒。
   * @return 延迟到期的测量，按采样时间升序排列。输入状态无效时不会生成新测量，
   * 但仍允许先前有效样本从延迟队列中释放。
   */
  std::vector<GnssMeasurement> update(const DeckStateSample & sample);

  /**
   * @brief 清空延迟队列、重置采样相位并恢复随机数初始种子。
   */
  void reset();

  /**
   * @brief 返回 NavSatFix 使用的 ENU 位置协方差矩阵。
   *
   * @return 3x3 行优先矩阵，轴顺序为 East、North、Up，单位为平方米。
   */
  std::array<double, 9> position_covariance_enu_m2() const;

  /**
   * @brief 返回已校验的传感器参数。
   */
  const GnssSensorParameters & parameters() const;

private:
  static GnssSensorParameters validate_parameters(const GnssSensorParameters & parameters);
  static bool sample_state_is_finite(const DeckStateSample & sample);
  GnssMeasurement make_measurement(const DeckStateSample & sample);
  std::vector<GnssMeasurement> release_ready(std::int64_t current_timestamp_ns);
  double gaussian_noise(double standard_deviation);

  GnssSensorParameters parameters_;
  gz::math::SphericalCoordinates spherical_coordinates_;
  std::int64_t sampling_period_ns_{0};
  std::int64_t latency_ns_{0};
  std::int64_t next_sample_timestamp_ns_{0};
  std::int64_t last_input_timestamp_ns_{0};
  bool have_sampling_schedule_{false};
  bool have_last_input_timestamp_{false};
  std::deque<GnssMeasurement> delayed_measurements_;
  std::mt19937 random_engine_;
  std::normal_distribution<double> standard_normal_distribution_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_distribution_{0.0, 1.0};
};

}  // namespace moving_deck_sim

#endif  // MOVING_DECK_SIM__GNSS_SENSOR_MODEL_HPP_
