// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__VISUAL_HANDOVER_GUIDANCE_HPP_
#define ARUCO_PRECISION_LANDING_CPP__VISUAL_HANDOVER_GUIDANCE_HPP_

#include <optional>

#include <Eigen/Core>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 视觉观测相对最近有效测量的丢失状态。
 */
enum class VisualLossState
{
  kFresh,
  kGraceHold,
  kShortLoss,
  kLongLoss
};

/**
 * @brief GNSS 到视觉接管和安全高度视觉跟踪参数。
 */
struct VisualHandoverParameters
{
  double handover_duration_s{0.5};
  double max_gnss_visual_difference_m{3.0};
  double max_visual_measurement_jump_m{0.5};
  double visual_loss_short_timeout_s{0.5};
  double visual_loss_long_timeout_s{2.0};
  double max_target_speed_mps{2.0};
  double max_target_step_m{0.20};
};

/**
 * @brief 实现视觉测量过滤、GNSS 一致性检查、线性接管和目标限幅。
 *
 * 该类只处理 local NED 数学量和调用方提供的秒时间，不依赖 ROS 节点或消息类型。
 * 高度控制不在本类中实现，P2D 调用方必须始终保持安全会合高度。
 */
class VisualHandoverGuidance
{
public:
  /**
   * @brief 创建视觉接管逻辑并校验全部参数。
   *
   * @param parameters 接管、丢失判定、测量跳变和目标限幅参数。
   * @throws std::invalid_argument 任一参数非法，或长时丢失阈值不大于短时阈值。
   */
  explicit VisualHandoverGuidance(const VisualHandoverParameters & parameters);

  /**
   * @brief 接收一帧 Marker 在 PX4 local NED 中的完整位置。
   *
   * @param position_ned Marker 位置 `[North, East, Down]`，单位为米。
   * @param receipt_time_s 控制器接收并完成坐标转换的单调时间，单位为秒。
   * @return 测量有限、时间不回退且相对最近有效测量未发生过大水平跳变时返回 true。
   *         超过长时丢失阈值后的首帧允许重新初始化。
   */
  bool update_visual_position(
    const Eigen::Vector3d & position_ned,
    double receipt_time_s);

  /**
   * @brief 返回当前短时新鲜的视觉位置。
   *
   * @param now_s 当前控制时间，单位为秒。
   * @return 最近有效 local NED 位置；无测量、时间回退或测量年龄超过短时阈值时
   *         返回 `std::nullopt`。
   */
  std::optional<Eigen::Vector3d> visual_position(double now_s) const;

  /**
   * @brief 检查视觉候选位置与船舶 GNSS 粗位置是否水平一致。
   *
   * @param visual_position_ned 视觉 Marker local NED 位置，单位为米。
   * @param gnss_position_ned 船舶 GNSS local NED 粗位置，单位为米。
   * @return 两个位置有限且水平距离不超过配置阈值时返回 true。
   */
  bool consistent_with_gnss(
    const Eigen::Vector3d & visual_position_ned,
    const Eigen::Vector3d & gnss_position_ned) const;

  /**
   * @brief 根据当前观测有效性和最近有效视觉测量年龄判断丢失等级。
   *
   * @param currently_valid 当前控制周期是否得到可用于控制的视觉位置。
   * @param now_s 当前控制时间，单位为秒。
   * @return 当前有效为 `kFresh`；丢失时间未超过短时阈值为 `kGraceHold`；短时与
   *         长时阈值之间为 `kShortLoss`；无历史、时间非法或超过长时阈值为
   *         `kLongLoss`。
   */
  VisualLossState loss_state(bool currently_valid, double now_s) const;

  /**
   * @brief 计算从 GNSS 到视觉的线性接管权重。
   *
   * @param elapsed_s 进入 `VISUAL_HANDOVER` 后经过的时间，单位为秒。
   * @return `[0,1]` 接管权重；输入含 NaN/Inf 或为负时返回 `std::nullopt`。
   */
  std::optional<double> handover_alpha(double elapsed_s) const;

  /**
   * @brief 线性混合 GNSS 和视觉水平目标。
   *
   * @param gnss_target_xy GNSS local NED 水平目标 `[North, East]`，单位为米。
   * @param visual_target_xy 视觉 local NED 水平目标 `[North, East]`，单位为米。
   * @param elapsed_s 进入接管状态后的经过时间，单位为秒。
   * @return 混合目标；输入非法时返回 `std::nullopt`。
   */
  std::optional<Eigen::Vector2d> blended_target_xy(
    const Eigen::Vector2d & gnss_target_xy,
    const Eigen::Vector2d & visual_target_xy,
    double elapsed_s) const;

  /**
   * @brief 同时按最大速度和单周期最大步长限制水平设定点变化。
   *
   * @param current_target_xy 当前 local NED 水平设定点，单位为米。
   * @param desired_target_xy 期望 local NED 水平设定点，单位为米。
   * @param dt_s 控制周期，单位为秒且必须为有限正数。
   * @return 限幅后的水平目标；输入非法时返回 `std::nullopt`。
   */
  std::optional<Eigen::Vector2d> limit_target_xy(
    const Eigen::Vector2d & current_target_xy,
    const Eigen::Vector2d & desired_target_xy,
    double dt_s) const;

  /**
   * @brief 清除视觉历史，使下一帧重新初始化测量序列。
   */
  void reset();

private:
  VisualHandoverParameters parameters_;
  Eigen::Vector3d last_visual_position_ned_{Eigen::Vector3d::Zero()};
  double last_visual_time_s_{0.0};
  bool have_visual_position_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__VISUAL_HANDOVER_GUIDANCE_HPP_
