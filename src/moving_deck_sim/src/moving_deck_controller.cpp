#include "moving_deck_sim/motion_profile.hpp"
#include "moving_deck_sim/rigid_body_kinematics.hpp"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/math/Quaternion.hh>
#include <gz/transport/Node.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>
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
constexpr double kResetOdometryGuardDurationS = 0.1;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

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

template<std::size_t Size>
std::array<double, Size> degrees_to_radians(
  const std::array<double, Size> & degrees)
{
  std::array<double, Size> radians{};
  for (std::size_t index = 0; index < Size; ++index) {
    radians[index] = degrees[index] * kDegreesToRadians;
  }
  return radians;
}

std::array<double, 4> quaternion_wxyz_from_rpy(
  const std::array<double, 3> & rpy)
{
  const double half_roll = 0.5 * rpy[0];
  const double half_pitch = 0.5 * rpy[1];
  const double half_yaw = 0.5 * rpy[2];
  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);
  return {
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  };
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
    uav_model_name_ = declare_parameter<std::string>(
      "uav_model_name", "x500_mono_cam_down_0");
    uav_body_offset_model_ = to_array<3>(
      declare_parameter<std::vector<double>>(
        "uav_body_offset_model", {0.0, 0.0, 0.24}),
      "uav_body_offset_model");
    const std::string scenario_name =
      declare_parameter<std::string>("scenario", "S1_CONSTANT_XY");
    const auto initial_position = declare_parameter<std::vector<double>>(
      "initial_position_enu", {0.0, 0.0, 2.0});
    const auto initial_position_offset = to_array<3>(
      declare_parameter<std::vector<double>>(
        "initial_position_offset_enu", {0.0, 0.0, 0.0}),
      "initial_position_offset_enu");
    deck_transform_.translation_body = to_array<3>(
      declare_parameter<std::vector<double>>(
        "deck_offset_body", {0.0, 0.0, 0.0}),
      "deck_offset_body");
    deck_transform_.rotation_wxyz = to_array<4>(
      declare_parameter<std::vector<double>>(
        "deck_rotation_wxyz", {1.0, 0.0, 0.0, 0.0}),
      "deck_rotation_wxyz");
    deck_frame_id_ = declare_parameter<std::string>("deck_frame_id", "");
    const auto velocity_xy = declare_parameter<std::vector<double>>(
      "velocity_xy", {0.4, 0.0});
    const auto amplitude_xy = declare_parameter<std::vector<double>>(
      "amplitude_xy", {1.0, 0.5});
    const auto period_xy = declare_parameter<std::vector<double>>(
      "period_xy", {10.0, 6.0});
    const double amplitude_z_m = declare_parameter<double>("amplitude_z_m", 0.0);
    const double period_z_s = declare_parameter<double>("period_z_s", 8.0);
    const auto initial_rpy_deg = declare_parameter<std::vector<double>>(
      "initial_rpy_deg", {0.0, 0.0, 0.0});
    const auto amplitude_rpy_deg = declare_parameter<std::vector<double>>(
      "amplitude_rpy_deg", {0.0, 0.0, 0.0});
    const auto period_rpy_s = declare_parameter<std::vector<double>>(
      "period_rpy_s", {8.0, 6.0, 10.0});
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", 50.0);
    const std::int64_t random_seed = declare_parameter<std::int64_t>("random_seed", 1);

    if (world_name_.empty() || model_name_.empty() || uav_model_name_.empty()) {
      throw std::invalid_argument("world_name and model names must not be empty");
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
    for (std::size_t index = 0; index < parameters.initial_position_enu.size(); ++index) {
      parameters.initial_position_enu[index] += initial_position_offset[index];
    }
    parameters.velocity_xy = to_array<2>(velocity_xy, "velocity_xy");
    parameters.amplitude_xy = to_array<2>(amplitude_xy, "amplitude_xy");
    parameters.period_xy = to_array<2>(period_xy, "period_xy");
    parameters.amplitude_z_m = amplitude_z_m;
    parameters.period_z_s = period_z_s;
    parameters.initial_rpy_rad = degrees_to_radians(
      to_array<3>(initial_rpy_deg, "initial_rpy_deg"));
    parameters.amplitude_rpy_rad = degrees_to_radians(
      to_array<3>(amplitude_rpy_deg, "amplitude_rpy_deg"));
    parameters.period_rpy_s = to_array<3>(period_rpy_s, "period_rpy_s");
    parameters.update_rate_hz = update_rate_hz_;
    initial_position_enu_ = parameters.initial_position_enu;
    initial_rpy_rad_ = parameters.initial_rpy_rad;
    motion_profile_ = std::make_unique<MotionProfile>(parameters);
    if (deck_frame_id_.empty()) {
      deck_frame_id_ = model_name_;
    }
    // 构造阶段先触发 fixed transform 的有限性和四元数校验，避免运行后才出现坏 GT。
    (void)transform_rigid_body_state(RigidBodyState{}, deck_transform_);

    velocity_publisher_ = gz_node_.Advertise<gz::msgs::Twist>(
      "/model/" + model_name_ + "/cmd_vel");
    ground_truth_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/simulation/deck/ground_truth", 10);
    uav_ground_truth_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/simulation/uav/ground_truth_pose", 10);
    if (!gz_node_.Subscribe(
        "/world/" + world_name_ + "/pose/info",
        &MovingDeckController::on_world_pose, this))
    {
      throw std::runtime_error("failed to subscribe to Gazebo world poses");
    }
    auto reset_count_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    reset_count_publisher_ = create_publisher<std_msgs::msg::UInt32>(
      "/simulation/episode/reset_count", reset_count_qos);
    raw_ground_truth_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/simulation/deck/ground_truth_raw", 10,
      std::bind(&MovingDeckController::on_raw_ground_truth, this, std::placeholders::_1));
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
  /**
   * @brief 从 Gazebo 世界位姿中只发布指定无人机模型的离线评测真值。
   *
   * 输出不被控制节点订阅；线速度由 evaluator 对该无噪声位姿离线差分。
   */
  void on_world_pose(const gz::msgs::Pose_V & poses)
  {
    for (const auto & pose : poses.pose()) {
      if (pose.name() == uav_model_name_) {
        nav_msgs::msg::Odometry ground_truth;
        ground_truth.header.stamp = now();
        ground_truth.header.frame_id = "world";
        ground_truth.child_frame_id = uav_model_name_ + "/base_link";
        const gz::math::Quaterniond orientation{
          pose.orientation().w(), pose.orientation().x(),
          pose.orientation().y(), pose.orientation().z()};
        const gz::math::Vector3d body_offset = orientation.RotateVector(
          gz::math::Vector3d{
            uav_body_offset_model_[0],
            uav_body_offset_model_[1],
            uav_body_offset_model_[2]});
        ground_truth.pose.pose.position.x = pose.position().x() + body_offset.X();
        ground_truth.pose.pose.position.y = pose.position().y() + body_offset.Y();
        ground_truth.pose.pose.position.z = pose.position().z() + body_offset.Z();
        ground_truth.pose.pose.orientation.w = pose.orientation().w();
        ground_truth.pose.pose.orientation.x = pose.orientation().x();
        ground_truth.pose.pose.orientation.y = pose.orientation().y();
        ground_truth.pose.pose.orientation.z = pose.orientation().z();
        uav_ground_truth_publisher_->publish(ground_truth);
        return;
      }
    }
  }

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
    const auto orientation = quaternion_wxyz_from_rpy(initial_rpy_rad_);
    request.mutable_orientation()->set_w(orientation[0]);
    request.mutable_orientation()->set_x(orientation[1]);
    request.mutable_orientation()->set_y(orientation[2]);
    request.mutable_orientation()->set_z(orientation[3]);
    gz::msgs::Boolean response;
    bool result = false;
    const bool executed = gz_node_.Request(
      "/world/" + world_name_ + "/set_pose", request,
      kGazeboServiceTimeoutMs, response, result);
    return executed && result && response.data();
  }

  void publish_motion_command(const MotionSample & sample)
  {
    gz::msgs::Twist command;
    command.mutable_linear()->set_x(sample.velocity_enu[0]);
    command.mutable_linear()->set_y(sample.velocity_enu[1]);
    command.mutable_linear()->set_z(sample.velocity_enu[2]);
    command.mutable_angular()->set_x(sample.angular_velocity_body[0]);
    command.mutable_angular()->set_y(sample.angular_velocity_body[1]);
    command.mutable_angular()->set_z(sample.angular_velocity_body[2]);
    if (!velocity_publisher_.Publish(command)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to publish Gazebo deck motion command");
    }
  }

  RigidBodyState deck_state_from_sample(const MotionSample & sample) const
  {
    RigidBodyState vessel_state;
    vessel_state.position_world = sample.position_enu;
    vessel_state.orientation_wxyz = quaternion_wxyz_from_rpy(sample.orientation_rpy_enu);
    vessel_state.linear_velocity_world = sample.velocity_enu;
    vessel_state.angular_velocity_body = sample.angular_velocity_body;
    return transform_rigid_body_state(vessel_state, deck_transform_);
  }

  void fill_deck_ground_truth(
    nav_msgs::msg::Odometry & message, const RigidBodyState & deck_state) const
  {
    message.header.frame_id = "world";
    message.child_frame_id = deck_frame_id_;
    message.pose.pose.position.x = deck_state.position_world[0];
    message.pose.pose.position.y = deck_state.position_world[1];
    message.pose.pose.position.z = deck_state.position_world[2];
    message.pose.pose.orientation.w = deck_state.orientation_wxyz[0];
    message.pose.pose.orientation.x = deck_state.orientation_wxyz[1];
    message.pose.pose.orientation.y = deck_state.orientation_wxyz[2];
    message.pose.pose.orientation.z = deck_state.orientation_wxyz[3];
    message.twist.twist.linear.x = deck_state.linear_velocity_world[0];
    message.twist.twist.linear.y = deck_state.linear_velocity_world[1];
    message.twist.twist.linear.z = deck_state.linear_velocity_world[2];
    message.twist.twist.angular.x = deck_state.angular_velocity_body[0];
    message.twist.twist.angular.y = deck_state.angular_velocity_body[1];
    message.twist.twist.angular.z = deck_state.angular_velocity_body[2];
  }

  void publish_initial_ground_truth()
  {
    const MotionSample sample = motion_profile_->sample(0.0);
    nav_msgs::msg::Odometry ground_truth;
    ground_truth.header.stamp = trajectory_start_;
    fill_deck_ground_truth(ground_truth, deck_state_from_sample(sample));
    ground_truth_publisher_->publish(ground_truth);
  }

  bool initialize_deck()
  {
    publish_motion_command(MotionSample{});
    if (!request_seed() || !request_initial_pose()) {
      initialized_ = false;
      return false;
    }

    trajectory_start_ = now();
    initialized_ = true;

    std_msgs::msg::UInt32 reset_count;
    reset_count.data = ++reset_count_;
    reset_count_publisher_->publish(reset_count);

    publish_motion_command(motion_profile_->sample(0.0));
    publish_initial_ground_truth();
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
    publish_motion_command(motion_profile_->sample(elapsed_s));
  }

  void on_raw_ground_truth(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!initialized_) {
      return;
    }

    const rclcpp::Time sample_time(message->header.stamp, RCL_ROS_TIME);
    if (sample_time < trajectory_start_) {
      return;
    }

    RigidBodyState vessel_state;
    vessel_state.position_world = {
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      message->pose.pose.position.z};
    vessel_state.orientation_wxyz = {
      message->pose.pose.orientation.w,
      message->pose.pose.orientation.x,
      message->pose.pose.orientation.y,
      message->pose.pose.orientation.z};
    vessel_state.linear_velocity_world = {
      message->twist.twist.linear.x,
      message->twist.twist.linear.y,
      message->twist.twist.linear.z};
    vessel_state.angular_velocity_body = {
      message->twist.twist.angular.x,
      message->twist.twist.angular.y,
      message->twist.twist.angular.z};

    const double elapsed_s = (sample_time - trajectory_start_).seconds();
    if (elapsed_s <= kResetOdometryGuardDurationS) {
      // Gazebo OdometryPublisher 使用历史位姿差分计算速度；teleport 后短时改用同一解析
      // vessel 轨迹速度，再统一经过 lever-arm 转换，避免旧位姿污染 deck Ground Truth。
      const MotionSample sample = motion_profile_->sample(elapsed_s);
      vessel_state.linear_velocity_world = sample.velocity_enu;
      vessel_state.angular_velocity_body = sample.angular_velocity_body;
    }

    nav_msgs::msg::Odometry ground_truth;
    ground_truth.header.stamp = message->header.stamp;
    fill_deck_ground_truth(
      ground_truth, transform_rigid_body_state(vessel_state, deck_transform_));
    ground_truth_publisher_->publish(ground_truth);
  }

  std::string world_name_;
  std::string model_name_;
  std::string uav_model_name_;
  double update_rate_hz_{50.0};
  std::uint32_t random_seed_{1};
  std::array<double, 3> initial_position_enu_{};
  std::array<double, 3> initial_rpy_rad_{};
  std::array<double, 3> uav_body_offset_model_{};
  FixedRigidTransform deck_transform_{};
  std::string deck_frame_id_;
  std::unique_ptr<MotionProfile> motion_profile_;

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher velocity_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr uav_ground_truth_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr reset_count_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_ground_truth_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time trajectory_start_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point next_initialization_attempt_{};
  std::uint32_t reset_count_{0};
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
