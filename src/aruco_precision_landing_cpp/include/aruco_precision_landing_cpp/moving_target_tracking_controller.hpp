// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__MOVING_TARGET_TRACKING_CONTROLLER_HPP_
#define ARUCO_PRECISION_LANDING_CPP__MOVING_TARGET_TRACKING_CONTROLLER_HPP_

#include "aruco_precision_landing_cpp/adaptive_relative_velocity_gain.hpp"
#include "aruco_precision_landing_cpp/target_state_estimator.hpp"

#include <memory>
#include <optional>
#include <string>

#include <Eigen/Core>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 移动甲板水平跟踪控制模式。
 */
enum class TrackingControlMode
{
  kRawVisualPosition,
  kEstimatedPosition,
  kEstimatedPositionVelocityFeedforward,
  kPredictedPositionVelocityFeedforward
};

/**
 * @brief 将配置字符串转换为跟踪控制模式。
 *
 * @param value 模式字符串。
 * @return 字符串属于允许集合时返回对应模式，否则返回 `std::nullopt`。
 */
std::optional<TrackingControlMode> tracking_control_mode_from_string(
  const std::string & value);

/**
 * @brief 返回跟踪控制模式的稳定配置名称。
 *
 * @param mode 跟踪控制模式。
 * @return 用于日志、参数和调试话题的常量字符串。
 */
const char * tracking_control_mode_name(TrackingControlMode mode);

/**
 * @brief 移动目标水平跟踪参数。
 */
struct MovingTargetTrackingParameters
{
  TrackingControlMode mode{TrackingControlMode::kPredictedPositionVelocityFeedforward};
  double max_position_target_speed_mps{2.0};
  double max_position_target_step_m{0.20};
  double velocity_feedforward_gain{1.0};
  double relative_velocity_gain{0.25};
  bool adaptive_relative_velocity_gain_enabled{false};
  AdaptiveRelativeVelocityGainParameters adaptive_relative_velocity_gain_parameters{};
  double max_velocity_feedforward_mps{1.5};
  double max_velocity_feedforward_acceleration_mps2{1.0};
  double max_prediction_age_s{0.75};
};

/**
 * @brief 单周期移动目标跟踪输入。
 */
struct MovingTargetTrackingInput
{
  Eigen::Vector2d current_target_xy{Eigen::Vector2d::Zero()};
  std::optional<Eigen::Vector2d> raw_visual_position_xy;
  std::optional<TargetStateEstimate> estimated_state;
  std::optional<Eigen::Vector2d> predicted_position_xy;
  Eigen::Vector2d uav_position_xy{Eigen::Vector2d::Zero()};
  std::optional<Eigen::Vector2d> uav_velocity_xy;
  bool visual_fresh{false};
  double estimate_age_s{0.0};
  double dt_s{0.0};
};

/**
 * @brief 单周期移动目标跟踪输出。
 */
struct MovingTargetTrackingCommand
{
  Eigen::Vector2d position_target_xy{Eigen::Vector2d::Zero()};
  std::optional<Eigen::Vector2d> velocity_feedforward_xy;
  std::optional<double> effective_relative_velocity_gain;
  std::optional<Eigen::Vector2d> estimated_deck_acceleration_xy;
  TrackingControlMode mode{TrackingControlMode::kRawVisualPosition};
  bool used_prediction{false};
  bool used_short_loss_prediction{false};
};

/**
 * @brief 根据视觉、状态估计和无人机状态生成受限水平位置与速度前馈。
 *
 * 该类只处理 PX4 local NED 水平数学量，不依赖 ROS 或 PX4 消息。位置误差反馈由 PX4
 * 内部位置控制器完成，本类只生成位置参考、目标速度前馈和相对速度阻尼。
 */
class MovingTargetTrackingController
{
public:
  /**
   * @brief 创建移动目标跟踪控制器并校验参数。
   *
   * @param parameters 模式、位置目标、速度前馈、加速度和短时预测限制。
   * @throws std::invalid_argument 任一参数非法或模式枚举不受支持。
   */
  explicit MovingTargetTrackingController(
    const MovingTargetTrackingParameters & parameters);

  /**
   * @brief 计算一个控制周期的水平位置目标和可选速度前馈。
   *
   * @param input 当前目标、视觉/估计/预测状态、无人机状态、估计年龄和控制周期。
   * @return 输入和当前模式所需数据有效时返回受限指令，否则返回 `std::nullopt`。
   */
  std::optional<MovingTargetTrackingCommand> compute(
    const MovingTargetTrackingInput & input);

  /**
   * @brief 清除上一周期速度前馈历史，使下次前馈从零按加速度限制重新建立。
   */
  void reset();

  /**
   * @brief 返回当前配置的跟踪模式。
   */
  TrackingControlMode mode() const;

private:
  std::optional<Eigen::Vector2d> select_position_reference(
    const MovingTargetTrackingInput & input,
    bool & used_prediction) const;
  std::optional<Eigen::Vector2d> compute_velocity_feedforward(
    const MovingTargetTrackingInput & input,
    double & effective_relative_velocity_gain,
    std::optional<Eigen::Vector2d> & estimated_deck_acceleration_xy);
  std::optional<AdaptiveRelativeVelocityGainOutput> update_adaptive_gain(
    const TargetStateEstimate & estimate,
    double fallback_dt_s);
  std::optional<Eigen::Vector2d> limit_position_target(
    const Eigen::Vector2d & current_target_xy,
    const Eigen::Vector2d & desired_target_xy,
    double dt_s) const;

  MovingTargetTrackingParameters parameters_;
  std::unique_ptr<AdaptiveRelativeVelocityGain> adaptive_gain_scheduler_;
  std::optional<AdaptiveRelativeVelocityGainOutput> last_adaptive_gain_output_;
  double last_adaptive_estimate_sample_time_s_{0.0};
  bool have_last_adaptive_estimate_sample_time_{false};
  Eigen::Vector2d last_velocity_feedforward_xy_{Eigen::Vector2d::Zero()};
  bool have_last_velocity_feedforward_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__MOVING_TARGET_TRACKING_CONTROLLER_HPP_
