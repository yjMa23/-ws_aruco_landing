// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__TOUCHDOWN_HOLD_CONTROLLER_HPP_
#define ARUCO_PRECISION_LANDING_CPP__TOUCHDOWN_HOLD_CONTROLLER_HPP_

#include <optional>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 触地确认后的垂直保持模式。
 */
enum class TouchdownHoldMode
{
  kRelativeDeckHold,
  kStationaryDeckHold,
  kHoldLastTarget,
};

/**
 * @brief 触地保持输出原因。
 */
enum class TouchdownHoldReason
{
  kTrackingDeck,
  kDeckMotionBelowThreshold,
  kDeckStateInvalid,
};

/**
 * @brief 触地后甲板相对垂直保持参数。
 */
struct TouchdownHoldParameters
{
  /** 世界系 NED 垂直目标允许的最大变化率，单位 m/s。 */
  double max_target_rate_mps{0.60};
  /** 可选相对高度预压参考的最大变化率，单位 m/s。 */
  double max_reference_preload_rate_mps{0.05};
  /** 从静止保持进入甲板随动所需的垂直速度，单位 m/s。 */
  double motion_enter_speed_mps{0.04};
  /** 从甲板随动退出到静止保持的垂直速度，单位 m/s。 */
  double motion_exit_speed_mps{0.02};
};

/**
 * @brief 触地后甲板相对垂直保持输入。
 */
struct TouchdownHoldInput
{
  /** 控制周期，单位秒，必须为有限正数。 */
  double dt_s{0.0};
  /** 甲板垂直位置和速度估计是否新鲜且有效。 */
  bool deck_state_valid{false};
  /** 无人机 PX4 local NED z，单位米，Down 为正。 */
  double uav_z_ned_m{0.0};
  /** 甲板 PX4 local NED z，单位米，Down 为正。 */
  double deck_z_ned_m{0.0};
  /** 甲板 PX4 local NED 垂直速度，单位 m/s，Down 为正。 */
  double deck_vertical_velocity_ned_mps{0.0};
  /** 可选的甲板相对高度预压目标；为空时保持触地确认时的原参考。 */
  std::optional<double> relative_height_target_m;
};

/**
 * @brief 触地后甲板相对垂直保持输出。
 */
struct TouchdownHoldOutput
{
  /** 触地确认时锁存的甲板相对高度，单位米。 */
  double relative_height_reference_m{0.0};
  /** 发送给 PX4 的 local NED z 目标，单位米，Down 为正。 */
  double vertical_target_z_ned_m{0.0};
  /** 有效甲板 NED 垂直速度；估计失效时为空并禁止前馈。 */
  std::optional<double> deck_vertical_velocity_ned_mps;
  TouchdownHoldMode mode{TouchdownHoldMode::kHoldLastTarget};
  TouchdownHoldReason reason{TouchdownHoldReason::kDeckStateInvalid};
};

/**
 * @brief 在触地确认后保持无人机与升沉甲板的相对垂直关系。
 *
 * 首次有效更新锁存 `deck_z_ned - uav_z_ned`，后续使用甲板垂直状态估计
 * 计算世界系 z 目标，并限制每周期目标变化。甲板估计失效时保持最后一个安全
 * 世界系目标，不继续下降且不输出垂直速度前馈。该类不使用仿真 Ground Truth。
 */
class TouchdownHoldController
{
public:
  /**
   * @brief 创建触地保持控制器并校验参数。
   *
   * @throws std::invalid_argument 最大目标变化率无效或不为正。
   */
  explicit TouchdownHoldController(const TouchdownHoldParameters & parameters);

  /**
   * @brief 更新一次甲板相对垂直保持目标。
   *
   * @param input PX4 local NED 下的无人机/甲板垂直状态和控制周期。
   * @return 有效输出；未初始化且甲板状态无效，或输入时间/有限性非法时返回空。
   */
  std::optional<TouchdownHoldOutput> update(const TouchdownHoldInput & input);

  /**
   * @brief 清除锁存的相对高度参考和世界系目标。
   */
  void reset();

  /**
   * @brief 返回是否已经由有效甲板状态完成初始化。
   */
  bool initialized() const;

private:
  TouchdownHoldOutput make_output(
    TouchdownHoldMode mode,
    TouchdownHoldReason reason,
    std::optional<double> deck_vertical_velocity_ned_mps) const;

  TouchdownHoldParameters parameters_;
  double relative_height_reference_m_{0.0};
  double vertical_target_z_ned_m_{0.0};
  bool initialized_{false};
  bool deck_motion_active_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__TOUCHDOWN_HOLD_CONTROLLER_HPP_
