// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__TOUCHDOWN_DETECTOR_HPP_
#define ARUCO_PRECISION_LANDING_CPP__TOUCHDOWN_DETECTOR_HPP_

#include <cstdint>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 触地检测输出状态。
 */
enum class TouchdownStatus
{
  kInsufficientEvidence,
  kAirborne,
  kCandidate,
  kConfirmed,
  kRejectedUnsafe,
};

/**
 * @brief 触地证据位。
 */
enum class TouchdownEvidence : std::uint32_t
{
  kNone = 0U,
  kGroundContact = 1U << 0,
  kMaybeLanded = 1U << 1,
  kLanded = 1U << 2,
  kAtRest = 1U << 3,
  kLowThrottle = 1U << 4,
  kCloseToGround = 1U << 5,
  kVisualLowHeight = 1U << 6,
  kLowRelativeVerticalSpeed = 1U << 7,
  kLowUavVerticalSpeed = 1U << 8,
  kNoReportedMovement = 1U << 9,
  kLowRelativeHorizontalSpeed = 1U << 10,
  kTerminalContactStall = 1U << 11,
  kTerminalGeometryProximity = 1U << 12,
};

/**
 * @brief 触地检测器参数。
 */
struct TouchdownDetectorParameters
{
  double px4_status_timeout_s{0.20};
  double visual_timeout_s{0.20};
  double low_height_enter_m{0.18};
  double low_height_exit_m{0.28};
  double max_relative_vertical_speed_mps{0.12};
  double max_uav_vertical_speed_mps{0.15};
  double max_relative_horizontal_speed_mps{0.15};
  double terminal_contact_max_height_m{0.24};
  double terminal_contact_min_reference_error_m{0.10};
  double terminal_contact_max_geometry_gap_m{0.03};
  double terminal_contact_max_vertical_speed_mps{0.05};
  double terminal_contact_px4_status_timeout_s{2.0};
  double candidate_required_duration_s{0.50};
};

/**
 * @brief 单周期触地检测输入。
 */
struct TouchdownDetectorInput
{
  double sample_time_s{0.0};
  bool state_allows_touchdown_detection{false};

  bool px4_land_status_valid{false};
  double px4_land_status_age_s{0.0};
  bool freefall{false};
  bool ground_contact{false};
  bool maybe_landed{false};
  bool landed{false};
  bool at_rest{false};
  bool has_low_throttle{false};
  bool vertical_movement{false};
  bool horizontal_movement{false};
  bool rotational_movement{false};
  bool close_to_ground{false};

  bool visual_height_valid{false};
  double visual_height_age_s{0.0};
  double relative_height_m{0.0};
  /** 相对垂直速度是否由有效且新鲜的甲板垂直状态估计得到。 */
  bool relative_vertical_speed_valid{false};
  /** `deck_vz_ned - uav_vz_ned`，单位 m/s，Down 为正。 */
  double relative_vertical_velocity_mps{0.0};
  double uav_vertical_velocity_mps{0.0};
  bool relative_horizontal_speed_valid{false};
  double relative_horizontal_speed_mps{0.0};
  bool terminal_descent_active{false};
  bool terminal_command_complete{false};
  double relative_height_reference_m{0.0};
  /** active terminal contact stabilization 在线甲板平面几何是否有效；禁止使用 Ground Truth。 */
  bool terminal_contact_geometry_valid{false};
  /** 在线估计的四滑橇最小法向间隙，单位 m。 */
  double minimum_skid_clearance_m{0.0};
};

/**
 * @brief 单周期触地检测结果。
 */
struct TouchdownDetectorOutput
{
  TouchdownStatus status{TouchdownStatus::kInsufficientEvidence};
  std::uint32_t evidence_mask{0U};
  double candidate_duration_s{0.0};
  bool confirmed_latched{false};
};

/**
 * @brief 使用 PX4 接触状态、相对低运动状态和视觉低高度联合确认触地。
 *
 * 视觉高度只能作为普通触地路径的辅助证据，不能单独确认触地。PX4 报告世界系
 * 水平运动时，只有无人机相对估计甲板的水平速度足够小才允许形成候选；PX4 报告
 * 世界系垂直运动时，必须有有效且足够小的甲板相对垂直速度，从而支持升沉平台而
 * 不绕过相对运动约束。终端落板段保留“最低落板命令已到达、参考已压入甲板”
 * 的原组合路径；active terminal contact stabilization 还可使用在线视觉甲板平面给出的四滑橇最小间隙，
 * 与低高度、实际垂直运动持续停滞和 PX4 close-to-ground 联合形成候选。该路径
 * 不读取 Ground Truth，且不能绕过视觉新鲜度和相对运动约束。
 * 用于 Offboard 位置环已经压住起落架但 PX4 land detector 尚未置位的情况。强触地
 * 路径要求 PX4 同时报告 `landed` 与 `at_rest`。确认结果会锁存，只有显式 reset 才清除。
 */
class TouchdownDetector
{
public:
  /**
   * @brief 创建触地检测器并校验参数。
   *
   * @throws std::invalid_argument 参数非有限、非正或高度迟滞关系非法。
   */
  explicit TouchdownDetector(const TouchdownDetectorParameters & parameters);

  /**
   * @brief 更新一次多源触地判定。
   *
   * @param input 当前状态、PX4 land detector、视觉高度及相对垂直/水平速度证据。
   * @return 当前状态、证据位、连续候选时间和确认锁存标志。
   */
  TouchdownDetectorOutput update(const TouchdownDetectorInput & input);

  /**
   * @brief 清除候选持续时间、低高度迟滞和确认锁存。
   */
  void reset();

private:
  TouchdownDetectorOutput make_output(
    TouchdownStatus status,
    std::uint32_t evidence_mask) const;
  void clear_candidate();

  TouchdownDetectorParameters parameters_;
  double last_sample_time_s_{0.0};
  double candidate_duration_s_{0.0};
  bool have_last_sample_time_{false};
  bool visual_low_height_latched_{false};
  bool confirmed_latched_{false};
};

constexpr std::uint32_t touchdown_evidence_mask(TouchdownEvidence evidence)
{
  return static_cast<std::uint32_t>(evidence);
}

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__TOUCHDOWN_DETECTOR_HPP_
