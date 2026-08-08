// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__FINAL_DESCENT_CONTROLLER_HPP_
#define ARUCO_PRECISION_LANDING_CPP__FINAL_DESCENT_CONTROLLER_HPP_

#include <optional>

#include "aruco_precision_landing_cpp/touchdown_detector.hpp"

namespace aruco_precision_landing_cpp
{

/**
 * @brief final descent 最终下降阶段。
 */
enum class FinalDescentPhase
{
  kWaitingAuthorization,
  kDescending,
  kCandidateHold,
  kTouchdownHold,
  kPaused,
  kRecoveryRequested,
};

/**
 * @brief 最终下降控制器参数。
 */
struct FinalDescentParameters
{
  double entry_height_m{0.50};
  double approach_rate_mps{0.12};
  double contact_rate_mps{0.03};
  double contact_slowdown_height_m{0.25};
  double terminal_descent_entry_height_m{0.20};
  double minimum_command_height_m{0.05};
  double maximum_reference_tracking_error_m{0.20};
};

/**
 * @brief 单周期最终下降输入。
 */
struct FinalDescentInput
{
  double current_relative_height_m{0.0};
  double current_reference_height_m{0.0};
  bool final_descent_authorized{false};
  bool vertical_reference_valid{false};
  bool landing_window_open{false};
  bool terminal_descent_allowed{false};
  TouchdownStatus touchdown_status{TouchdownStatus::kInsufficientEvidence};
  double dt_s{0.0};
};

/**
 * @brief 单周期最终下降输出。
 */
struct FinalDescentOutput
{
  double relative_height_reference_m{0.0};
  double vertical_reference_velocity_ned_mps{0.0};
  FinalDescentPhase phase{FinalDescentPhase::kWaitingAuthorization};
  bool reference_changed{false};
  bool touchdown_candidate_hold{false};
  bool touchdown_confirmed_hold{false};
  bool recovery_requested{false};
};

/**
 * @brief 从 relative descent 测试高度生成分段最终下降参考。
 *
 * 控制器不读取 ROS 或 Ground Truth。进入最终下降后先使用较快接近速率，在
 * `contact_slowdown_height_m` 以下切换为近接触低速。当参考进入
 * `terminal_descent_entry_height_m` 且上层确认除低高度外的着陆窗口条件仍安全时，
 * 允许继续执行终端落板段，避免仅因低高度窗口或暂时缺少 PX4 接触证据而悬停。
 * touchdown evidence 进入候选后立即冻结参考；确认后锁存保持。不安全拒绝仍请求恢复。最低命令高度
 * 只用于限制向甲板内部发送的目标，不能作为触地证据。
 */
class FinalDescentController
{
public:
  /**
   * @brief 创建并校验最终下降参数。
   *
   * @throws std::invalid_argument 高度、速率或误差参数非法时抛出。
   */
  explicit FinalDescentController(const FinalDescentParameters & parameters);

  /**
   * @brief 更新一次最终下降参考。
   *
   * @return 输入有效时返回输出；高度、dt 非有限或垂直参考无效时返回 `std::nullopt`。
   */
  std::optional<FinalDescentOutput> update(const FinalDescentInput & input);

  /**
   * @brief 清除参考、初始化状态和触地确认锁存。
   */
  void reset();

  bool initialized() const;

private:
  FinalDescentOutput make_output(
    double previous_reference_m,
    double vertical_reference_velocity_ned_mps,
    FinalDescentPhase phase) const;

  FinalDescentParameters parameters_;
  double relative_height_reference_m_{0.0};
  bool initialized_{false};
  bool touchdown_confirmed_latched_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__FINAL_DESCENT_CONTROLLER_HPP_
