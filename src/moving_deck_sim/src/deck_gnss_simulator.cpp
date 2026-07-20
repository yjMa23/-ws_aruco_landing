#include "moving_deck_sim/gnss_sensor_model.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace moving_deck_sim
{

/**
 * @brief 将甲板仿真真值转换为带噪声、延迟和丢包的船舶 GNSS / 遥测话题。
 *
 * 本节点属于仿真传感器层，允许订阅 Ground Truth。降落控制器只能订阅本节点输出，
 * 禁止直接订阅 `/simulation/deck/ground_truth`。
 */
class DeckGnssSimulator : public rclcpp::Node
{
public:
  /**
   * @brief 声明传感器参数、创建纯数学模型并建立 ROS 2 接口。
   */
  DeckGnssSimulator()
  : Node("deck_gnss_simulator")
  {
    GnssSensorParameters parameters;
    parameters.publish_rate_hz = declare_parameter<double>("publish_rate_hz", 5.0);
    parameters.horizontal_noise_std_m =
      declare_parameter<double>("horizontal_noise_std_m", 0.8);
    parameters.vertical_noise_std_m =
      declare_parameter<double>("vertical_noise_std_m", 1.5);
    parameters.velocity_noise_std_mps =
      declare_parameter<double>("velocity_noise_std_mps", 0.1);
    parameters.latency_s = declare_parameter<double>("latency_ms", 100.0) / 1000.0;
    parameters.packet_drop_probability =
      declare_parameter<double>("packet_drop_probability", 0.0);

    const std::int64_t random_seed = declare_parameter<std::int64_t>("random_seed", 1);
    if (random_seed < 0 ||
      random_seed > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()))
    {
      throw std::invalid_argument("random_seed must fit in uint32");
    }
    parameters.random_seed = static_cast<std::uint32_t>(random_seed);
    parameters.reference_latitude_deg =
      declare_parameter<double>("world_origin.latitude_deg", 47.397971057728974);
    parameters.reference_longitude_deg =
      declare_parameter<double>("world_origin.longitude_deg", 8.546163739800146);
    parameters.reference_elevation_m =
      declare_parameter<double>("world_origin.elevation_m", 0.0);

    gps_frame_id_ = declare_parameter<std::string>("gps_frame_id", "moving_deck_gps");
    velocity_frame_id_ = declare_parameter<std::string>("velocity_frame_id", "world_enu");
    expected_ground_truth_frame_id_ =
      declare_parameter<std::string>("expected_ground_truth_frame_id", "world");
    if (gps_frame_id_.empty() || velocity_frame_id_.empty() ||
      expected_ground_truth_frame_id_.empty())
    {
      throw std::invalid_argument("GNSS frame ids must not be empty");
    }

    model_ = std::make_unique<GnssSensorModel>(parameters);
    position_covariance_enu_m2_ = model_->position_covariance_enu_m2();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/deck/gps/fix", sensor_qos);
    velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/deck/gps/velocity", sensor_qos);
    ground_truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/simulation/deck/ground_truth", rclcpp::QoS(10).reliable(),
      std::bind(&DeckGnssSimulator::on_ground_truth, this, std::placeholders::_1));

    auto reset_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    reset_count_subscription_ = create_subscription<std_msgs::msg::UInt32>(
      "/simulation/episode/reset_count", reset_qos,
      std::bind(&DeckGnssSimulator::on_reset_count, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Deck GNSS configured at %.2f Hz, noise h=%.2f m v=%.2f m, latency=%.0f ms, drop=%.3f",
      parameters.publish_rate_hz, parameters.horizontal_noise_std_m,
      parameters.vertical_noise_std_m, parameters.latency_s * 1000.0,
      parameters.packet_drop_probability);
  }

private:
  void on_ground_truth(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (message->header.frame_id != expected_ground_truth_frame_id_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Rejecting deck Ground Truth frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), expected_ground_truth_frame_id_.c_str());
      return;
    }

    const rclcpp::Time sample_time(message->header.stamp, RCL_ROS_TIME);
    DeckStateSample sample;
    sample.timestamp_ns = sample_time.nanoseconds();
    sample.position_enu_m = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
    sample.velocity_enu_mps = {
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z};

    for (const GnssMeasurement & measurement : model_->update(sample)) {
      publish_measurement(measurement);
    }
  }

  void on_reset_count(const std_msgs::msg::UInt32::SharedPtr message)
  {
    if (have_reset_count_ && message->data == last_reset_count_) {
      return;
    }
    last_reset_count_ = message->data;
    have_reset_count_ = true;
    model_->reset();
    RCLCPP_INFO(get_logger(), "Reset GNSS model for episode %u", message->data);
  }

  void publish_measurement(const GnssMeasurement & measurement)
  {
    const auto stamp = static_cast<builtin_interfaces::msg::Time>(
      rclcpp::Time(measurement.sample_timestamp_ns, RCL_ROS_TIME));

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = gps_frame_id_;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.latitude = measurement.latitude_deg;
    fix.longitude = measurement.longitude_deg;
    fix.altitude = measurement.altitude_m;
    fix.position_covariance = position_covariance_enu_m2_;
    fix.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    fix_publisher_->publish(fix);

    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = velocity_frame_id_;
    velocity.twist.linear.x = measurement.velocity_enu_mps[0];
    velocity.twist.linear.y = measurement.velocity_enu_mps[1];
    velocity.twist.linear.z = measurement.velocity_enu_mps[2];
    velocity_publisher_->publish(velocity);
  }

  std::string gps_frame_id_;
  std::string velocity_frame_id_;
  std::string expected_ground_truth_frame_id_;
  std::unique_ptr<GnssSensorModel> model_;
  std::array<double, 9> position_covariance_enu_m2_{};
  std::uint32_t last_reset_count_{0};
  bool have_reset_count_{false};

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr reset_count_subscription_;
};

}  // namespace moving_deck_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<moving_deck_sim::DeckGnssSimulator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("deck_gnss_simulator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
