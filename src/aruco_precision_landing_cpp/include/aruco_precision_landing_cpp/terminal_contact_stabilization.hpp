// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__TERMINAL_CONTACT_STABILIZATION_HPP_
#define ARUCO_PRECISION_LANDING_CPP__TERMINAL_CONTACT_STABILIZATION_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{

/**
 * @brief terminal contact stabilization 终端稳定化允许介入的离散阶段。
 */
enum class TerminalStabilizationPhase
{
  kInactive,
  kFinalDescent,
  kTouchdownCandidateHold,
  kTouchdownHold,
  kRehearsal,
  kRecovery,
};

/** T1 固定倾角实验允许的姿态轴；默认保留完整视觉法向。 */
enum class TerminalAlignmentAxis
{
  kFull,
  kRollOnly,
  kPitchOnly,
};

/**
 * @brief 终端甲板法向整形参数。
 */
struct TerminalDeckNormalParameters
{
  double gravity_mps2{9.80665};
  double maximum_target_tilt_rad{0.0436332313};
  double minimum_upward_component{0.50};
  double normal_freshness_timeout_s{0.20};
  double short_loss_hold_s{0.10};
  double marker_switch_jump_gate_rad{0.0174532925};
  double tilt_slew_rate_radps{0.0698131701};
  double acceleration_bias_limit_mps2{0.45};
  double acceleration_bias_slew_rate_mps3{0.80};
  double activation_duration_s{0.50};
  double deactivation_duration_s{0.30};
  double maximum_dt_s{0.20};
};

/**
 * @brief 终端甲板法向整形输入。
 */
struct TerminalDeckNormalInput
{
  double now_s{0.0};
  double dt_s{0.0};
  TerminalStabilizationPhase phase{TerminalStabilizationPhase::kInactive};
  bool production_enabled{false};
  bool rehearsal_enabled{false};
  bool normal_valid{false};
  double normal_age_s{0.0};
  bool marker_switched{false};
  double marker_switch_jump_rad{0.0};
  TerminalAlignmentAxis alignment_axis{TerminalAlignmentAxis::kFull};
  /** 已确认接触后允许保持最后有效视觉法向，直到退出 HOLD 或安全保护触发。 */
  bool latch_last_valid_normal{false};
  Eigen::Vector3d upward_normal_ned{0.0, 0.0, -1.0};
  double yaw_ned_rad{0.0};
};

/**
 * @brief 终端甲板法向整形输出。
 */
struct TerminalDeckNormalOutput
{
  bool enabled{false};
  bool valid{false};
  bool fallback_active{false};
  std::string mode{"DISABLED"};
  std::string reason{"feature_disabled"};
  Eigen::Vector3d desired_upward_normal_ned{0.0, 0.0, -1.0};
  Eigen::Vector3d desired_body_z_ned{0.0, 0.0, 1.0};
  Eigen::Vector2d desired_roll_pitch_rad{0.0, 0.0};
  Eigen::Vector2d acceleration_bias_ned_mps2{0.0, 0.0};
  double activation_weight{0.0};
};

/**
 * @brief 将视觉甲板法向转换为受限、连续的 NED 水平加速度偏置。
 *
 * PX4 位置控制器在悬停附近使用 `acc_sp=[a_n,a_e,-g]` 构造机体 z 轴，
 * 因此使机体 Down 轴对齐 `-normal_up` 所需的水平前馈为
 * `a_xy=-g*normal_xy/normal_z`。该类只生成附加前馈，不发布姿态设定点。
 */
class TerminalDeckNormalStabilizer
{
public:
  explicit TerminalDeckNormalStabilizer(const TerminalDeckNormalParameters & parameters);

  /**
   * @brief 更新受限终端法向整形命令。
   *
   * @param input NED 法向、当前阶段和控制时间；法向必须指向 Up，故 z 为负。
   * @return 始终有限的诊断和控制输出；非法输入会平滑回零。
   */
  TerminalDeckNormalOutput update(const TerminalDeckNormalInput & input);

  /**
   * @brief 清空法向、渐入权重和加速度历史。
   */
  void reset();

  /**
   * @brief 纯数学地将向上法向转换为未做时间连续性处理的 NED 水平偏置。
   *
   * @param upward_normal_ned NED 中向上法向，有限、非零且 z<0。
   * @param gravity_mps2 重力加速度，单位 m/s²。
   * @return 水平 acceleration bias；输入非法时为空。
   */
  static std::optional<Eigen::Vector2d> normal_to_acceleration_bias(
    const Eigen::Vector3d & upward_normal_ned,
    double gravity_mps2);

  /**
   * @brief 由目标机体 z 轴和保留 yaw 构造 body-to-NED 姿态。
   *
   * @param desired_body_z_ned 期望 FRD Down 轴在 NED 中的单位方向。
   * @param yaw_ned_rad 期望航向，弧度。
   * @return 有效 body-to-NED 四元数；退化或非法输入时为空。
   */
  static std::optional<Eigen::Quaterniond> body_z_and_yaw_to_attitude(
    const Eigen::Vector3d & desired_body_z_ned,
    double yaw_ned_rad);

  /** 将视觉法向投影到 T1 允许的纯 roll 或纯 pitch 轴。 */
  static std::optional<Eigen::Vector3d> project_normal_to_alignment_axis(
    const Eigen::Vector3d & upward_normal_ned,
    double yaw_ned_rad,
    TerminalAlignmentAxis axis);

  /**
   * @brief 判断法向整形是否已进入允许介入的阶段/近接触高度。
   */
  static bool phase_height_authorized(
    TerminalStabilizationPhase phase,
    double relative_height_m,
    double final_descent_activation_height_m,
    bool geometry_proximity_candidate = false);

private:
  TerminalDeckNormalOutput make_output(
    const std::string & mode,
    const std::string & reason,
    bool valid,
    bool fallback_active,
    const Eigen::Vector2d & bias,
    double yaw_ned_rad) const;
  bool phase_authorized(const TerminalDeckNormalInput & input) const;
  Eigen::Vector2d limit_bias_for_tilt(const Eigen::Vector2d & bias) const;
  static Eigen::Vector2d rate_limit_vector(
    const Eigen::Vector2d & previous,
    const Eigen::Vector2d & target,
    double maximum_step);

  TerminalDeckNormalParameters parameters_;
  Eigen::Vector2d current_bias_mps2_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_valid_target_bias_mps2_{Eigen::Vector2d::Zero()};
  Eigen::Vector3d last_valid_normal_ned_{0.0, 0.0, -1.0};
  double activation_weight_{0.0};
  double last_update_time_s_{0.0};
  double last_valid_normal_time_s_{0.0};
  bool initialized_{false};
  bool have_valid_normal_{false};
};

/**
 * @brief 接触后水平顺应参数。
 */
struct TerminalContactComplianceParameters
{
  double horizontal_deadband_m{0.015};
  double maximum_allowance_m{0.040};
  double maximum_target_rate_mps{0.10};
  double maximum_anchor_correction_rate_mps{0.05};
  double deck_velocity_deadband_mps{0.035};
  double relative_velocity_damping_s{0.12};
  double maximum_damping_offset_m{0.020};
  double maximum_dt_s{0.20};
};

/**
 * @brief 接触后水平顺应输入，全部位于 local NED 水平面。
 */
struct TerminalContactComplianceInput
{
  double dt_s{0.0};
  TerminalStabilizationPhase phase{TerminalStabilizationPhase::kInactive};
  bool enabled{false};
  bool deck_state_valid{false};
  Eigen::Vector2d nominal_target_xy_m{Eigen::Vector2d::Zero()};
  Eigen::Vector2d deck_position_xy_m{Eigen::Vector2d::Zero()};
  Eigen::Vector2d deck_velocity_xy_mps{Eigen::Vector2d::Zero()};
  Eigen::Vector2d uav_position_xy_m{Eigen::Vector2d::Zero()};
  Eigen::Vector2d uav_velocity_xy_mps{Eigen::Vector2d::Zero()};
};

/**
 * @brief 接触后水平顺应输出。
 */
struct TerminalContactComplianceOutput
{
  bool active{false};
  bool valid{false};
  std::string reason{"disabled"};
  Eigen::Vector2d contact_anchor_xy_m{Eigen::Vector2d::Zero()};
  Eigen::Vector2d compliant_target_xy_m{Eigen::Vector2d::Zero()};
  Eigen::Vector2d relative_velocity_xy_mps{Eigen::Vector2d::Zero()};
};

/**
 * @brief candidate/hold 中建立甲板相对锚点并提供有限水平让步。
 */
class TerminalContactComplianceController
{
public:
  explicit TerminalContactComplianceController(
    const TerminalContactComplianceParameters & parameters);

  /**
   * @brief 更新接触顺应目标。
   *
   * 首次 candidate 更新严格返回名义目标以避免跳变；后续目标随甲板平移，
   * 小误差落入 deadband，超出部分有限恢复，并加入相对速度阻尼。
   */
  TerminalContactComplianceOutput update(const TerminalContactComplianceInput & input);

  /**
   * @brief 清空接触锚点和最后安全目标。
   */
  void reset();

  bool initialized() const;

private:
  TerminalContactComplianceParameters parameters_;
  Eigen::Vector2d anchor_offset_from_deck_m_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d contact_anchor_xy_m_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d undamped_target_xy_m_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d last_target_xy_m_{Eigen::Vector2d::Zero()};
  bool initialized_{false};
};

/**
 * @brief candidate/hold 姿态发散保护参数。
 */
struct TerminalAttitudeSafetyParameters
{
  double attitude_trigger_rad{0.1047197551};
  double attitude_clear_rad{0.0698131701};
  double angular_rate_trigger_radps{0.7853981634};
  double required_duration_s{0.20};
  double clear_duration_s{0.30};
  double maximum_dt_s{0.20};
};

/**
 * @brief 姿态安全监视器输入。
 */
struct TerminalAttitudeSafetyInput
{
  double dt_s{0.0};
  TerminalStabilizationPhase phase{TerminalStabilizationPhase::kInactive};
  bool enabled{false};
  bool attitude_valid{false};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  Eigen::Vector3d angular_velocity_body_radps{Eigen::Vector3d::Zero()};
};

/**
 * @brief 姿态安全监视器输出。
 */
struct TerminalAttitudeSafetyOutput
{
  bool valid{false};
  bool warning{false};
  bool recovery_requested{false};
  std::string reason{"disabled"};
  double attitude_error_rad{0.0};
  double angular_rate_radps{0.0};
  double violation_duration_s{0.0};
};

/**
 * @brief 在 evaluator 10° 硬门之前检测持续姿态或角速度发散。
 */
class TerminalAttitudeSafetyMonitor
{
public:
  explicit TerminalAttitudeSafetyMonitor(
    const TerminalAttitudeSafetyParameters & parameters);

  TerminalAttitudeSafetyOutput update(const TerminalAttitudeSafetyInput & input);

  void reset();

private:
  TerminalAttitudeSafetyParameters parameters_;
  double violation_duration_s_{0.0};
  double clear_duration_s_{0.0};
  bool latched_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__TERMINAL_CONTACT_STABILIZATION_HPP_
