#include "moving_deck_sim/motion_profile.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace moving_deck_sim
{
namespace
{

constexpr unsigned int kGazeboServiceTimeoutMs = 1000;

template<std::size_t Size>
std::array<double, Size> to_array(
  const std::vector<double> & values, const std::string & parameter_name)
{
  if (values.size() != Size) {
    throw std::invalid_argument(
            parameter_name + " must contain " + std::to_string(Size) + " values");
  }

  std::array<double, Size> result{};
  for (std::size_t index = 0; index < Size; ++index) {
    result[index] = values[index];
  }
  return result;
}

}  // namespace

/**
 * @brief 按 YAML 轨迹驱动 Gazebo 甲板，并提供可重复的甲板重置服务。
 */
class MovingDeckController : public rclcpp::Node
{
public:
  /**
   * @brief 声明并校验参数，建立 Gazebo Transport 和 ROS 2 接口。
   *
   * 节点使用 ROS 仿真时间计算运动相位；Gazebo 服务未就绪时每秒重试，
   * 不会在初始位姿和随机种子设置成功前发布运动指令。
   */
  MovingDeckController()
  : Node("moving_deck_controller")
  {
    world_name_ = declare_parameter<std::string>("world_name", "aruco");
    model_name_ = declare_parameter<std::string>("model_name", "moving_deck");
    const std::string scenario_name =
      declare_parameter<std::string>("scenario", "S1_CONSTANT_XY");
    const auto initial_position = declare_parameter<std::vector<double>>(
      "initial_position_enu", {0.0, 0.0, 2.0});
    const auto velocity_xy = declare_parameter<std::vector<double>>(
      "velocity_xy", {0.4, 0.0});
    const auto amplitude_xy = declare_parameter<std::vector<double>>(
      "amplitude_xy", {1.0, 0.5});
    const auto period_xy = declare_parameter<std::vector<double>>(
      "period_xy", {10.0, 6.0});
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);
    const std::int64_t random_seed = declare_parameter<std::int64_t>("random_seed", 1);

    if (world_name_.empty() || model_name_.empty()) {
      throw std::invalid_argument("world_name and model_name must not be empty");
    }
    if (random_seed < 0 ||
      random_seed > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()))
    {
      throw std::invalid_argument("random_seed must fit in uint32");
    }
    random_seed_ = static_cast<std::uint32_t>(random_seed);

    MotionParameters parameters;
    parameters.scenario = MotionProfile::parse_scenario(scenario_name);
    parameters.initial_position_enu = to_array<3>(initial_position, "initial_position_enu");
    parameters.velocity_xy = to_array<2>(velocity_xy, "velocity_xy");
    parameters.amplitude_xy = to_array<2>(amplitude_xy, "amplitude_xy");
    parameters.period_xy = to_array<2>(period_xy, "period_xy");
    parameters.update_rate_hz = update_rate_hz_;
    initial_position_enu_ = parameters.initial_position_enu;
    motion_profile_ = std::make_unique<MotionProfile>(parameters);

    velocity_publisher_ = gz_node_.Advertise<gz::msgs::Twist>(
      "/model/" + model_name_ + "/cmd_vel");
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/simulation/episode/reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        response->success = initialize_deck();
        response->message = response->success ?
        "moving deck reset" : "Gazebo rejected moving deck reset";
      });

    const auto period = std::chrono::duration<double>(1.0 / update_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MovingDeckController::on_timer, this));

    RCLCPP_INFO(
      get_logger(), "Configured %s in Gazebo world '%s' with seed %u",
      scenario_name.c_str(), world_name_.c_str(), random_seed_);
  }

private:
  bool request_seed()
  {
    gz::msgs::WorldControl request;
    request.set_seed(random_seed_);
    gz::msgs::Boolean response;
    bool result = false;
    const bool executed = gz_node_.Request(
      "/world/" + world_name_ + "/control", request,
      kGazeboServiceTimeoutMs, response, result);
    return executed && result && response.data();
  }

  bool request_initial_pose()
  {
    gz::msgs::Pose request;
    request.set_name(model_name_);
    request.mutable_position()->set_x(initial_position_enu_[0]);
    request.mutable_position()->set_y(initial_position_enu_[1]);
    request.mutable_position()->set_z(initial_position_enu_[2]);
    request.mutable_orientation()->set_w(1.0);
    gz::msgs::Boolean response;
    bool result = false;
    const bool executed = gz_node_.Request(
      "/world/" + world_name_ + "/set_pose", request,
      kGazeboServiceTimeoutMs, response, result);
    return executed && result && response.data();
  }

  void publish_velocity(const std::array<double, 3> & velocity_enu)
  {
    gz::msgs::Twist command;
    command.mutable_linear()->set_x(velocity_enu[0]);
    command.mutable_linear()->set_y(velocity_enu[1]);
    command.mutable_linear()->set_z(velocity_enu[2]);
    if (!velocity_publisher_.Publish(command)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to publish Gazebo deck velocity");
    }
  }

  bool initialize_deck()
  {
    publish_velocity({0.0, 0.0, 0.0});
    if (!request_seed() || !request_initial_pose()) {
      initialized_ = false;
      return false;
    }

    trajectory_start_ = now();
    initialized_ = true;
    publish_velocity(motion_profile_->sample(0.0).velocity_enu);
    return true;
  }

  void on_timer()
  {
    if (!initialized_) {
      const auto steady_now = std::chrono::steady_clock::now();
      if (steady_now < next_initialization_attempt_) {
        return;
      }
      next_initialization_attempt_ = steady_now + std::chrono::seconds(1);
      if (!initialize_deck()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Waiting for Gazebo moving deck services");
      }
      return;
    }

    const rclcpp::Time current_time = now();
    if (current_time < trajectory_start_) {
      initialized_ = false;
      return;
    }

    const double elapsed_s = (current_time - trajectory_start_).seconds();
    publish_velocity(motion_profile_->sample(elapsed_s).velocity_enu);
  }

  std::string world_name_;
  std::string model_name_;
  double update_rate_hz_{50.0};
  std::uint32_t random_seed_{1};
  std::array<double, 3> initial_position_enu_{};
  std::unique_ptr<MotionProfile> motion_profile_;

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher velocity_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point next_initialization_attempt_{};
  bool initialized_{false};
};

}  // namespace moving_deck_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<moving_deck_sim::MovingDeckController>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("moving_deck_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
