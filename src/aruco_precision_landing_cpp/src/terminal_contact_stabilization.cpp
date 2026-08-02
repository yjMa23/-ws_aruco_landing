// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/terminal_contact_stabilization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aruco_precision_landing_cpp
{
namespace
{

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

Eigen::Vector2d limit_norm(const Eigen::Vector2d & value, double maximum_norm)
{
  const double norm = value.norm();
  if (!std::isfinite(norm) || norm <= maximum_norm || norm <= 1.0e-12) {
    return value;
  }
  return value * (maximum_norm / norm);
}

Eigen::Vector2d euler_roll_pitch(const Eigen::Matrix3d & rotation)
{
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  return {roll, pitch};
}

bool contact_phase(TerminalStabilizationPhase phase)
{
  return phase == TerminalStabilizationPhase::kFinalDescent ||
         phase == TerminalStabilizationPhase::kTouchdownCandidateHold ||
         phase == TerminalStabilizationPhase::kTouchdownHold;
}

}  // namespace

TerminalDeckNormalStabilizer::TerminalDeckNormalStabilizer(
  const TerminalDeckNormalParameters & parameters)
: parameters_(parameters)
{
  if (!finite_positive(parameters_.gravity_mps2) ||
    !finite_positive(parameters_.maximum_target_tilt_rad) ||
    parameters_.maximum_target_tilt_rad >= M_PI_2 ||
    !finite_positive(parameters_.minimum_upward_component) ||
    parameters_.minimum_upward_component > 1.0 ||
    !finite_positive(parameters_.normal_freshness_timeout_s) ||
    !finite_nonnegative(parameters_.short_loss_hold_s) ||
    parameters_.short_loss_hold_s > parameters_.normal_freshness_timeout_s ||
    !finite_positive(parameters_.marker_switch_jump_gate_rad) ||
    !finite_positive(parameters_.tilt_slew_rate_radps) ||
    !finite_positive(parameters_.acceleration_bias_limit_mps2) ||
    !finite_positive(parameters_.acceleration_bias_slew_rate_mps3) ||
    !finite_positive(parameters_.activation_duration_s) ||
    !finite_positive(parameters_.deactivation_duration_s) ||
    !finite_positive(parameters_.maximum_dt_s))
  {
    throw std::invalid_argument("invalid terminal deck-normal stabilization parameters");
  }
}

std::optional<Eigen::Vector2d>
TerminalDeckNormalStabilizer::normal_to_acceleration_bias(
  const Eigen::Vector3d & upward_normal_ned,
  double gravity_mps2)
{
  if (!upward_normal_ned.allFinite() || !finite_positive(gravity_mps2)) {
    return std::nullopt;
  }
  const double norm = upward_normal_ned.norm();
  if (!std::isfinite(norm) || norm <= 1.0e-9) {
    return std::nullopt;
  }
  const Eigen::Vector3d normal = upward_normal_ned / norm;
  if (!(normal.z() < -1.0e-9)) {
    return std::nullopt;
  }

  // PX4 mc_pos_control 在悬停附近形成 acc_sp=[a_n,a_e,-g]，随后用
  // body_z=-acc_sp/|acc_sp| 构造姿态。令 body_z=-normal_up 可得下式。
  const Eigen::Vector2d bias =
    -gravity_mps2 * normal.head<2>() / normal.z();
  if (!bias.allFinite()) {
    return std::nullopt;
  }
  return bias;
}

std::optional<Eigen::Quaterniond>
TerminalDeckNormalStabilizer::body_z_and_yaw_to_attitude(
  const Eigen::Vector3d & desired_body_z_ned,
  double yaw_ned_rad)
{
  if (!desired_body_z_ned.allFinite() || !std::isfinite(yaw_ned_rad)) {
    return std::nullopt;
  }
  const double norm = desired_body_z_ned.norm();
  if (!std::isfinite(norm) || norm <= 1.0e-9) {
    return std::nullopt;
  }
  const Eigen::Vector3d body_z = desired_body_z_ned / norm;
  const Eigen::Vector3d y_c{-std::sin(yaw_ned_rad), std::cos(yaw_ned_rad), 0.0};
  Eigen::Vector3d body_x = y_c.cross(body_z);
  if (body_z.z() < 0.0) {
    body_x = -body_x;
  }
  if (std::abs(body_z.z()) < 1.0e-6) {
    body_x = Eigen::Vector3d::UnitZ();
  }
  const double body_x_norm = body_x.norm();
  if (!std::isfinite(body_x_norm) || body_x_norm <= 1.0e-9) {
    return std::nullopt;
  }
  body_x /= body_x_norm;
  const Eigen::Vector3d body_y = body_z.cross(body_x);
  if (!body_y.allFinite() || body_y.norm() <= 1.0e-9) {
    return std::nullopt;
  }

  Eigen::Matrix3d rotation;
  rotation.col(0) = body_x;
  rotation.col(1) = body_y.normalized();
  rotation.col(2) = body_z;
  Eigen::Quaterniond attitude(rotation);
  if (!attitude.coeffs().allFinite() || attitude.norm() <= 1.0e-9) {
    return std::nullopt;
  }
  attitude.normalize();
  return attitude;
}

std::optional<Eigen::Vector3d>
TerminalDeckNormalStabilizer::project_normal_to_alignment_axis(
  const Eigen::Vector3d & upward_normal_ned,
  double yaw_ned_rad,
  TerminalAlignmentAxis axis)
{
  if (axis == TerminalAlignmentAxis::kFull) {
    const double norm = upward_normal_ned.norm();
    if (!upward_normal_ned.allFinite() || !std::isfinite(norm) || norm <= 1.0e-9) {
      return std::nullopt;
    }
    return upward_normal_ned / norm;
  }
  const auto attitude = body_z_and_yaw_to_attitude(-upward_normal_ned, yaw_ned_rad);
  if (!attitude.has_value()) {
    return std::nullopt;
  }
  Eigen::Vector2d roll_pitch = euler_roll_pitch(attitude->toRotationMatrix());
  if (axis == TerminalAlignmentAxis::kRollOnly) {
    roll_pitch.y() = 0.0;
  } else if (axis == TerminalAlignmentAxis::kPitchOnly) {
    roll_pitch.x() = 0.0;
  } else {
    return std::nullopt;
  }
  const Eigen::Quaterniond projected_attitude =
    Eigen::AngleAxisd(yaw_ned_rad, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(roll_pitch.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll_pitch.x(), Eigen::Vector3d::UnitX());
  const Eigen::Vector3d projected_upward_normal =
    -projected_attitude.toRotationMatrix().col(2);
  if (!projected_upward_normal.allFinite() || projected_upward_normal.z() >= 0.0) {
    return std::nullopt;
  }
  return projected_upward_normal.normalized();
}

bool TerminalDeckNormalStabilizer::phase_height_authorized(
  TerminalStabilizationPhase phase,
  double relative_height_m,
  double final_descent_activation_height_m,
  bool geometry_proximity_candidate)
{
  if (phase == TerminalStabilizationPhase::kTouchdownCandidateHold) {
    return true;
  }
  if (phase == TerminalStabilizationPhase::kTouchdownHold ||
    phase == TerminalStabilizationPhase::kRehearsal)
  {
    return true;
  }
  return phase == TerminalStabilizationPhase::kFinalDescent &&
         std::isfinite(relative_height_m) && relative_height_m >= 0.0 &&
         finite_positive(final_descent_activation_height_m) &&
         relative_height_m <= final_descent_activation_height_m;
}

bool TerminalDeckNormalStabilizer::phase_authorized(
  const TerminalDeckNormalInput & input) const
{
  const bool production_phase =
    input.phase == TerminalStabilizationPhase::kFinalDescent ||
    input.phase == TerminalStabilizationPhase::kTouchdownCandidateHold ||
    input.phase == TerminalStabilizationPhase::kTouchdownHold;
  return (input.production_enabled && production_phase) ||
         (input.rehearsal_enabled &&
         input.phase == TerminalStabilizationPhase::kRehearsal);
}

Eigen::Vector2d TerminalDeckNormalStabilizer::limit_bias_for_tilt(
  const Eigen::Vector2d & bias) const
{
  const double tilt_limited_norm =
    parameters_.gravity_mps2 * std::tan(parameters_.maximum_target_tilt_rad);
  return limit_norm(
    bias,
    std::min(tilt_limited_norm, parameters_.acceleration_bias_limit_mps2));
}

Eigen::Vector2d TerminalDeckNormalStabilizer::rate_limit_vector(
  const Eigen::Vector2d & previous,
  const Eigen::Vector2d & target,
  double maximum_step)
{
  const Eigen::Vector2d delta = target - previous;
  const double norm = delta.norm();
  if (!std::isfinite(norm) || norm <= maximum_step || norm <= 1.0e-12) {
    return target;
  }
  return previous + delta * (maximum_step / norm);
}

TerminalDeckNormalOutput TerminalDeckNormalStabilizer::make_output(
  const std::string & mode,
  const std::string & reason,
  bool valid,
  bool fallback_active,
  const Eigen::Vector2d & bias,
  double yaw_ned_rad) const
{
  TerminalDeckNormalOutput output;
  output.enabled = activation_weight_ > 0.0 || bias.norm() > 1.0e-12;
  output.valid = valid;
  output.fallback_active = fallback_active;
  output.mode = mode;
  output.reason = reason;
  output.acceleration_bias_ned_mps2 = bias.allFinite() ? bias : Eigen::Vector2d::Zero();
  output.activation_weight = activation_weight_;

  Eigen::Vector3d acceleration{
    output.acceleration_bias_ned_mps2.x(),
    output.acceleration_bias_ned_mps2.y(),
    -parameters_.gravity_mps2};
  if (acceleration.allFinite() && acceleration.norm() > 1.0e-9) {
    output.desired_upward_normal_ned = acceleration.normalized();
    output.desired_body_z_ned = -output.desired_upward_normal_ned;
    const auto attitude = body_z_and_yaw_to_attitude(
      output.desired_body_z_ned, yaw_ned_rad);
    if (attitude.has_value()) {
      output.desired_roll_pitch_rad = euler_roll_pitch(attitude->toRotationMatrix());
    }
  }
  return output;
}

TerminalDeckNormalOutput TerminalDeckNormalStabilizer::update(
  const TerminalDeckNormalInput & input)
{
  const bool time_valid =
    std::isfinite(input.now_s) && std::isfinite(input.dt_s) &&
    input.dt_s > 0.0 && input.dt_s <= parameters_.maximum_dt_s;
  if (!time_valid || (initialized_ && input.now_s < last_update_time_s_)) {
    reset();
    return make_output(
      "FALLBACK", "invalid_or_non_monotonic_time", false, true,
      Eigen::Vector2d::Zero(), 0.0);
  }
  initialized_ = true;
  last_update_time_s_ = input.now_s;

  const bool authorized = phase_authorized(input);
  if (!authorized) {
    activation_weight_ = std::max(
      0.0,
      activation_weight_ - input.dt_s / parameters_.deactivation_duration_s);
    const Eigen::Vector2d target = Eigen::Vector2d::Zero();
    // 回零必须遵守与激活相同的等效倾角变化率，避免授权结束时产生姿态命令尖峰。
    const double tilt_based_slew =
      parameters_.gravity_mps2 * parameters_.tilt_slew_rate_radps;
    const double maximum_step = std::min(
      parameters_.acceleration_bias_slew_rate_mps3,
      tilt_based_slew) * input.dt_s;
    current_bias_mps2_ = rate_limit_vector(
      current_bias_mps2_, target, maximum_step);
    if (activation_weight_ <= 0.0 && current_bias_mps2_.norm() <= 1.0e-12) {
      current_bias_mps2_.setZero();
    }
    return make_output(
      "DISABLED", "phase_or_feature_not_authorized", false,
      current_bias_mps2_.norm() > 1.0e-12, current_bias_mps2_, input.yaw_ned_rad);
  }

  const bool marker_jump_valid =
    !input.marker_switched ||
    (std::isfinite(input.marker_switch_jump_rad) &&
    input.marker_switch_jump_rad <= parameters_.marker_switch_jump_gate_rad);
  const bool fresh_normal =
    input.normal_valid && input.upward_normal_ned.allFinite() &&
    std::isfinite(input.normal_age_s) && input.normal_age_s >= 0.0 &&
    input.normal_age_s <= parameters_.normal_freshness_timeout_s &&
    marker_jump_valid;

  Eigen::Vector2d target_bias = Eigen::Vector2d::Zero();
  bool command_valid = false;
  bool fallback_active = false;
  std::string reason;
  std::string mode = input.phase == TerminalStabilizationPhase::kRehearsal ?
    "REHEARSAL" : "ACTIVE";

  if (input.latch_last_valid_normal && have_valid_normal_) {
    // 已确认接触后，固定 T1 甲板法向不会突变。HOLD 从 candidate 末端锁存
    // 最后有效主轴法向，即使近距视觉仍有消息也不再接受其幅值抖动。
    target_bias = last_valid_target_bias_mps2_;
    command_valid = true;
    fallback_active = false;
    reason = "touchdown_hold_latched_normal";
  } else if (fresh_normal) {
    const auto projected_normal = project_normal_to_alignment_axis(
      input.upward_normal_ned, input.yaw_ned_rad, input.alignment_axis);
    const Eigen::Vector3d normalized = projected_normal.value_or(Eigen::Vector3d::Zero());
    const auto raw_bias = projected_normal.has_value() ?
      normal_to_acceleration_bias(*projected_normal, parameters_.gravity_mps2) :
      std::nullopt;
    if (raw_bias.has_value() &&
      normalized.z() <= -parameters_.minimum_upward_component)
    {
      target_bias = limit_bias_for_tilt(*raw_bias);
      last_valid_target_bias_mps2_ = target_bias;
      last_valid_normal_ned_ = normalized;
      last_valid_normal_time_s_ = input.now_s;
      have_valid_normal_ = true;
      command_valid = true;
      reason = "fresh_visual_normal";
    } else {
      fallback_active = true;
      reason = "invalid_upward_normal";
    }
  } else if (have_valid_normal_ &&
    std::isfinite(input.now_s - last_valid_normal_time_s_) &&
    input.now_s - last_valid_normal_time_s_ <= parameters_.short_loss_hold_s &&
    marker_jump_valid)
  {
    target_bias = last_valid_target_bias_mps2_;
    command_valid = true;
    fallback_active = true;
    reason = "short_visual_loss_hold";
  } else {
    fallback_active = true;
    reason = marker_jump_valid ? "normal_stale_or_invalid" : "marker_switch_jump_rejected";
  }

  if (command_valid) {
    activation_weight_ = std::min(
      1.0,
      activation_weight_ + input.dt_s / parameters_.activation_duration_s);
  } else {
    activation_weight_ = std::max(
      0.0,
      activation_weight_ - input.dt_s / parameters_.deactivation_duration_s);
    mode = "FALLBACK";
  }
  target_bias *= activation_weight_;

  // 同时按等效倾角变化率和显式 acceleration-bias 变化率限幅，取更严格者。
  const double tilt_based_slew =
    parameters_.gravity_mps2 * parameters_.tilt_slew_rate_radps;
  const double maximum_step = std::min(
    parameters_.acceleration_bias_slew_rate_mps3,
    tilt_based_slew) * input.dt_s;
  current_bias_mps2_ = rate_limit_vector(
    current_bias_mps2_, target_bias, maximum_step);
  current_bias_mps2_ = limit_bias_for_tilt(current_bias_mps2_);
  if (!current_bias_mps2_.allFinite()) {
    reset();
    return make_output(
      "FALLBACK", "non_finite_output_rejected", false, true,
      Eigen::Vector2d::Zero(), 0.0);
  }

  return make_output(
    mode, reason, command_valid, fallback_active,
    current_bias_mps2_, input.yaw_ned_rad);
}

void TerminalDeckNormalStabilizer::reset()
{
  current_bias_mps2_.setZero();
  last_valid_target_bias_mps2_.setZero();
  last_valid_normal_ned_ = Eigen::Vector3d{0.0, 0.0, -1.0};
  activation_weight_ = 0.0;
  last_update_time_s_ = 0.0;
  last_valid_normal_time_s_ = 0.0;
  initialized_ = false;
  have_valid_normal_ = false;
}

TerminalContactComplianceController::TerminalContactComplianceController(
  const TerminalContactComplianceParameters & parameters)
: parameters_(parameters)
{
  if (!finite_nonnegative(parameters_.horizontal_deadband_m) ||
    !finite_positive(parameters_.maximum_allowance_m) ||
    parameters_.horizontal_deadband_m >= parameters_.maximum_allowance_m ||
    !finite_positive(parameters_.maximum_target_rate_mps) ||
    !finite_positive(parameters_.maximum_anchor_correction_rate_mps) ||
    !finite_nonnegative(parameters_.deck_velocity_deadband_mps) ||
    !finite_nonnegative(parameters_.relative_velocity_damping_s) ||
    !finite_nonnegative(parameters_.maximum_damping_offset_m) ||
    parameters_.maximum_damping_offset_m > parameters_.maximum_allowance_m ||
    !finite_positive(parameters_.maximum_dt_s))
  {
    throw std::invalid_argument("invalid terminal contact compliance parameters");
  }
}

TerminalContactComplianceOutput TerminalContactComplianceController::update(
  const TerminalContactComplianceInput & input)
{
  TerminalContactComplianceOutput output;
  output.compliant_target_xy_m = input.nominal_target_xy_m.allFinite() ?
    input.nominal_target_xy_m : Eigen::Vector2d::Zero();

  const bool authorized = input.enabled && contact_phase(input.phase);
  const bool finite_input =
    std::isfinite(input.dt_s) && input.dt_s > 0.0 &&
    input.dt_s <= parameters_.maximum_dt_s &&
    input.nominal_target_xy_m.allFinite() &&
    input.deck_position_xy_m.allFinite() &&
    input.deck_velocity_xy_mps.allFinite() &&
    input.uav_position_xy_m.allFinite() &&
    input.uav_velocity_xy_mps.allFinite();
  if (!authorized || !finite_input) {
    if (!authorized) {
      reset();
      output.reason = "phase_or_feature_not_authorized";
    } else {
      output.reason = "invalid_input";
    }
    return output;
  }

  output.active = true;
  if (!initialized_) {
    if (!input.deck_state_valid) {
      output.reason = "deck_state_invalid_before_anchor";
      return output;
    }
    anchor_offset_from_deck_m_ =
      input.nominal_target_xy_m - input.deck_position_xy_m;
    contact_anchor_xy_m_ = input.nominal_target_xy_m;
    undamped_target_xy_m_ = input.nominal_target_xy_m;
    last_target_xy_m_ = input.nominal_target_xy_m;
    initialized_ = true;
    output.valid = true;
    output.reason = "contact_anchor_initialized";
    output.contact_anchor_xy_m = input.nominal_target_xy_m;
    output.compliant_target_xy_m = input.nominal_target_xy_m;
    return output;
  }

  if (!input.deck_state_valid) {
    output.valid = false;
    output.reason = "deck_state_invalid_hold_last_safe_target";
    output.contact_anchor_xy_m = last_target_xy_m_;
    output.compliant_target_xy_m = last_target_xy_m_;
    return output;
  }

  // 接触锚点以甲板速度连续传播；绝对视觉位置只做带死区、限速的慢校正，
  // 避免静止甲板的逐帧位置噪声直接变成接触目标抖动。
  const double deck_speed_mps = input.deck_velocity_xy_mps.norm();
  const Eigen::Vector2d effective_deck_velocity =
    std::isfinite(deck_speed_mps) &&
    deck_speed_mps > parameters_.deck_velocity_deadband_mps ?
    input.deck_velocity_xy_mps : Eigen::Vector2d::Zero();
  const Eigen::Vector2d predicted_anchor =
    contact_anchor_xy_m_ + effective_deck_velocity * input.dt_s;
  const Eigen::Vector2d measured_anchor =
    input.deck_position_xy_m + anchor_offset_from_deck_m_;
  const Eigen::Vector2d anchor_measurement_error = measured_anchor - predicted_anchor;
  const double anchor_error_norm = anchor_measurement_error.norm();
  Eigen::Vector2d anchor_correction = Eigen::Vector2d::Zero();
  if (std::isfinite(anchor_error_norm) &&
    anchor_error_norm > parameters_.horizontal_deadband_m)
  {
    anchor_correction = anchor_measurement_error *
      ((anchor_error_norm - parameters_.horizontal_deadband_m) / anchor_error_norm);
  }
  anchor_correction = limit_norm(
    anchor_correction,
    parameters_.maximum_anchor_correction_rate_mps * input.dt_s);
  const Eigen::Vector2d previous_contact_anchor = contact_anchor_xy_m_;
  contact_anchor_xy_m_ = predicted_anchor + anchor_correction;
  const Eigen::Vector2d anchor_translation =
    contact_anchor_xy_m_ - previous_contact_anchor;
  undamped_target_xy_m_ += anchor_translation;
  last_target_xy_m_ += anchor_translation;

  const Eigen::Vector2d contact_anchor = contact_anchor_xy_m_;
  const Eigen::Vector2d relative_velocity =
    input.uav_velocity_xy_mps - input.deck_velocity_xy_mps;
  const Eigen::Vector2d damping = limit_norm(
    -parameters_.relative_velocity_damping_s * relative_velocity,
    parameters_.maximum_damping_offset_m);

  // 位置基准始终以接触锚点为中心。不得用 UAV 当前位姿重建目标，否则控制器
  // 会沿真实滑移方向追随机体；顺应只由非积分速度阻尼偏移提供。
  const Eigen::Vector2d desired_undamped = contact_anchor;

  const double maximum_step = parameters_.maximum_target_rate_mps * input.dt_s;
  const Eigen::Vector2d undamped_delta = desired_undamped - undamped_target_xy_m_;
  undamped_target_xy_m_ =
    undamped_delta.norm() <= maximum_step || undamped_delta.norm() <= 1.0e-12 ?
    desired_undamped :
    undamped_target_xy_m_ + undamped_delta.normalized() * maximum_step;

  Eigen::Vector2d desired = undamped_target_xy_m_ + damping;
  Eigen::Vector2d allowance = desired - contact_anchor;
  allowance = limit_norm(allowance, parameters_.maximum_allowance_m);
  desired = contact_anchor + allowance;

  const Eigen::Vector2d delta = desired - last_target_xy_m_;
  last_target_xy_m_ = delta.norm() <= maximum_step || delta.norm() <= 1.0e-12 ?
    desired : last_target_xy_m_ + delta.normalized() * maximum_step;

  output.valid = last_target_xy_m_.allFinite();
  output.reason = output.valid ? "contact_compliance_active" : "non_finite_output";
  output.contact_anchor_xy_m = contact_anchor;
  output.compliant_target_xy_m = output.valid ?
    last_target_xy_m_ : input.nominal_target_xy_m;
  output.relative_velocity_xy_mps = relative_velocity;
  return output;
}

void TerminalContactComplianceController::reset()
{
  anchor_offset_from_deck_m_.setZero();
  contact_anchor_xy_m_.setZero();
  undamped_target_xy_m_.setZero();
  last_target_xy_m_.setZero();
  initialized_ = false;
}

bool TerminalContactComplianceController::initialized() const
{
  return initialized_;
}

TerminalAttitudeSafetyMonitor::TerminalAttitudeSafetyMonitor(
  const TerminalAttitudeSafetyParameters & parameters)
: parameters_(parameters)
{
  constexpr double evaluator_hard_limit_rad = 10.0 * M_PI / 180.0;
  if (!finite_positive(parameters_.attitude_trigger_rad) ||
    parameters_.attitude_trigger_rad >= evaluator_hard_limit_rad ||
    !finite_nonnegative(parameters_.attitude_clear_rad) ||
    parameters_.attitude_clear_rad >= parameters_.attitude_trigger_rad ||
    !finite_positive(parameters_.angular_rate_trigger_radps) ||
    !finite_positive(parameters_.required_duration_s) ||
    !finite_positive(parameters_.clear_duration_s) ||
    !finite_positive(parameters_.maximum_dt_s))
  {
    throw std::invalid_argument("invalid terminal attitude safety parameters");
  }
}

TerminalAttitudeSafetyOutput TerminalAttitudeSafetyMonitor::update(
  const TerminalAttitudeSafetyInput & input)
{
  TerminalAttitudeSafetyOutput output;
  output.recovery_requested = latched_;
  output.violation_duration_s = violation_duration_s_;

  const bool authorized = input.enabled && contact_phase(input.phase);
  const bool valid =
    authorized && input.attitude_valid &&
    std::isfinite(input.dt_s) && input.dt_s > 0.0 &&
    input.dt_s <= parameters_.maximum_dt_s &&
    std::isfinite(input.roll_rad) && std::isfinite(input.pitch_rad) &&
    input.angular_velocity_body_radps.allFinite();
  if (!valid) {
    if (!authorized) {
      reset();
      output.reason = "phase_or_feature_not_authorized";
      output.recovery_requested = false;
    } else {
      output.reason = "invalid_attitude_input";
    }
    return output;
  }

  output.valid = true;
  output.attitude_error_rad = std::max(
    std::abs(input.roll_rad), std::abs(input.pitch_rad));
  output.angular_rate_radps = input.angular_velocity_body_radps.head<2>().norm();
  const bool violation =
    output.attitude_error_rad >= parameters_.attitude_trigger_rad ||
    output.angular_rate_radps >= parameters_.angular_rate_trigger_radps;
  output.warning = violation;

  if (violation) {
    violation_duration_s_ += input.dt_s;
    clear_duration_s_ = 0.0;
    if (violation_duration_s_ + 1.0e-12 >= parameters_.required_duration_s) {
      latched_ = true;
    }
  } else if (output.attitude_error_rad <= parameters_.attitude_clear_rad) {
    clear_duration_s_ += input.dt_s;
    if (!latched_ && clear_duration_s_ >= parameters_.clear_duration_s) {
      violation_duration_s_ = 0.0;
    }
  } else {
    clear_duration_s_ = 0.0;
  }

  output.recovery_requested = latched_;
  output.violation_duration_s = violation_duration_s_;
  if (latched_) {
    output.reason = output.angular_rate_radps >= parameters_.angular_rate_trigger_radps ?
      "sustained_angular_rate_divergence" : "sustained_attitude_divergence";
  } else if (violation) {
    output.reason = "transient_divergence_warning";
  } else {
    output.reason = "attitude_within_safe_envelope";
  }
  return output;
}

void TerminalAttitudeSafetyMonitor::reset()
{
  violation_duration_s_ = 0.0;
  clear_duration_s_ = 0.0;
  latched_ = false;
}

}  // namespace aruco_precision_landing_cpp
