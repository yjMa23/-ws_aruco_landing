// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__RELATIVE_DESCENT_CONTROLLER_HPP_
#define ARUCO_PRECISION_LANDING_CPP__RELATIVE_DESCENT_CONTROLLER_HPP_

#include <optional>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 相对甲板高度下降阶段。
 */
enum class RelativeDescentPhase
{
  kWaitingWindow,
  kDescending,
  kPaused,
  kRecovering,
  kTestHeightHold,
};

/**
 * @brief 分阶段相对高度下降参数。
 */
struct RelativeDescentParameters
{
  double minimum_test_height_m{0.50};
  double fast_height_threshold_m{2.00};
  double slow_height_threshold_m{0.80};
  double fast_rate_mps{0.30};
  double medium_rate_mps{0.15};
  double slow_rate_mps{0.05};
  double recovery_height_m{2.00};
  double recovery_rate_mps{0.30};
  double max_reference_tracking_error_m{0.50};
};

/**
 * @brief 单周期相对下降输入。
 */
struct RelativeDescentInput
{
  double current_relative_height_m{0.0};
  bool window_open{false};
  bool vertical_reference_valid{false};
  bool severe_failure{false};
  double dt_s{0.0};
};

/**
 * @brief 单周期相对下降输出。
 */
struct RelativeDescentOutput
{
  double height_reference_m{0.0};
  RelativeDescentPhase phase{RelativeDescentPhase::kWaitingWindow};
  bool reference_changed{false};
  bool reached_test_height{false};
};

/**
 * @brief 生成相对甲板高度参考，不直接生成 PX4 世界坐标目标。
 *
 * 首次有效输入使用当前相对高度初始化参考，避免状态切换跳变。窗口打开时按高度分段降低
 * 参考；窗口关闭或实际高度跟踪误差过大时保持；严重失效时向恢复高度增大参考；最低只到
 * `minimum_test_height_m`。
 */
class RelativeDescentController
{
public:
  /**
   * @brief 创建并校验相对下降控制器。
   *
   * @throws std::invalid_argument 参数非有限、阈值顺序非法或速率非正。
   */
  explicit RelativeDescentController(const RelativeDescentParameters & parameters);

  /**
   * @brief 更新一次相对高度参考。
   *
   * @return 输入有效时返回输出；相对高度、dt 非法或垂直参考无效时返回 `std::nullopt`。
   */
  std::optional<RelativeDescentOutput> update(const RelativeDescentInput & input);

  /**
   * @brief 清除参考和下降历史。
   */
  void reset();

  bool initialized() const;

private:
  double descent_rate(double height_reference_m) const;
  RelativeDescentOutput make_output(
    double previous_reference_m,
    RelativeDescentPhase phase) const;

  RelativeDescentParameters parameters_;
  double height_reference_m_{0.0};
  bool initialized_{false};
  bool descent_started_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__RELATIVE_DESCENT_CONTROLLER_HPP_
