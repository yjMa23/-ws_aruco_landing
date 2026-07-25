// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__LANDING_WINDOW_HPP_
#define ARUCO_PRECISION_LANDING_CPP__LANDING_WINDOW_HPP_

#include <cstdint>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 规则式着陆窗口拒绝原因位掩码。
 */
enum class LandingWindowRejectReason : std::uint32_t
{
  kNone = 0U,
  kVisualUnavailable = 1U << 0U,
  kVisualTooOld = 1U << 1U,
  kEstimateInvalid = 1U << 2U,
  kPredictionInvalid = 1U << 3U,
  kHorizontalError = 1U << 4U,
  kRelativeSpeed = 1U << 5U,
  kDeckTilt = 1U << 6U,
  kRelativeHeight = 1U << 7U,
  kInvalidTime = 1U << 8U,
};

constexpr std::uint32_t landing_window_reason_mask(
  LandingWindowRejectReason reason)
{
  return static_cast<std::uint32_t>(reason);
}

/**
 * @brief 规则式着陆窗口阈值。
 */
struct LandingWindowParameters
{
  double enter_horizontal_error_m{0.15};
  double exit_horizontal_error_m{0.25};
  double enter_relative_speed_mps{0.15};
  double exit_relative_speed_mps{0.25};
  double enter_max_tilt_rad{0.08726646259971647};
  double exit_max_tilt_rad{0.13962634015954636};
  double max_visual_age_s{0.20};
  double minimum_relative_height_m{0.50};
  double maximum_relative_height_m{6.00};
  double required_duration_s{1.00};
};

/**
 * @brief 单周期着陆窗口输入。
 */
struct LandingWindowInput
{
  bool visual_fresh{false};
  bool estimate_valid{false};
  bool prediction_valid{false};
  double visual_age_s{0.0};
  double horizontal_error_m{0.0};
  double horizontal_relative_speed_mps{0.0};
  double deck_roll_rad{0.0};
  double deck_pitch_rad{0.0};
  double relative_height_m{0.0};
  double now_s{0.0};
};

/**
 * @brief 单周期着陆窗口输出。
 */
struct LandingWindowResult
{
  bool window_open{false};
  bool conditions_currently_satisfied{false};
  double satisfied_duration_s{0.0};
  std::uint32_t reject_reasons{landing_window_reason_mask(
      LandingWindowRejectReason::kNone)};
};

/**
 * @brief 使用进入/退出迟滞和连续满足时间判断是否具备下降条件。
 *
 * 该类不依赖 ROS、PX4 或仿真 Ground Truth。窗口关闭时使用进入阈值，窗口打开后使用
 * 更宽松的退出阈值。任一硬有效性条件失败、时间回退或指标超过退出阈值时窗口关闭。
 */
class LandingWindow
{
public:
  /**
   * @brief 创建着陆窗口并校验阈值关系。
   *
   * @throws std::invalid_argument 参数非有限、负值、进入阈值不小于退出阈值，或高度范围非法。
   */
  explicit LandingWindow(const LandingWindowParameters & parameters);

  /**
   * @brief 更新一次窗口状态。
   *
   * @param input 当前视觉、估计、相对状态、甲板姿态和单调时间。
   * @return 当前窗口状态、连续满足时间和拒绝原因。
   */
  LandingWindowResult update(const LandingWindowInput & input);

  /**
   * @brief 清除窗口、持续时间和时间历史。
   */
  void reset();

private:
  std::uint32_t evaluate_reject_reasons(
    const LandingWindowInput & input,
    bool use_exit_thresholds) const;

  LandingWindowParameters parameters_;
  bool window_open_{false};
  bool have_satisfied_since_{false};
  bool have_last_update_time_{false};
  double satisfied_since_s_{0.0};
  double last_update_time_s_{0.0};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__LANDING_WINDOW_HPP_
