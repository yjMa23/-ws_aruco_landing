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
  double relative_vertical_velocity_mps{0.0};
  double uav_vertical_velocity_mps{0.0};
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
 * @brief 使用 PX4 接触状态、低运动状态和视觉低高度联合确认触地。
 *
 * 视觉高度只能作为普通触地路径的辅助证据，不能单独确认触地。强触地路径要求
 * PX4 同时报告 `landed` 与 `at_rest`。确认结果会锁存，只有显式 reset 才清除。
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
   * @param input 当前状态、PX4 land detector、视觉高度和垂直速度证据。
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
