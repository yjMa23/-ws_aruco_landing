// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_DETECTOR__MARKER_SELECTOR_HPP_
#define ARUCO_DETECTOR__MARKER_SELECTOR_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

namespace aruco_detector
{

/**
 * @brief 一个允许检测的 Marker 配置。
 */
struct MarkerConfiguration
{
  int id{0};
  double length_m{0.0};
  int priority{0};
  std::array<double, 3> target_offset_marker_m{{0.0, 0.0, 0.0}};
};

/**
 * @brief 图像中一个已检测 Marker 的选择信息。
 */
struct MarkerDetectionCandidate
{
  int id{0};
  double corner_area_px2{0.0};
  double distance_to_image_border_px{0.0};
  std::size_t detection_index{0U};
};

/**
 * @brief 有状态 Marker 选择器参数。
 */
struct MarkerSelectorParameters
{
  std::vector<double> marker_min_switch_areas_px2;
  double active_hold_area_ratio{0.60};
  double minimum_border_margin_px{12.0};
  int switch_required_consecutive_frames{5};
  int active_missing_grace_frames{2};
};

/**
 * @brief Marker 选择状态和切换原因。
 */
enum class MarkerSelectionReason
{
  NO_VALID_CANDIDATE,
  INITIAL_ACQUIRE,
  HOLD_ACTIVE,
  ACTIVE_NEAR_BORDER,
  ACTIVE_AREA_LOW,
  ACTIVE_MISSING,
  CHALLENGER_STABILIZING,
  SWITCH_STABLE,
  ACTIVE_CLEARED,
};

/**
 * @brief 已选 Marker 及其物理配置和本帧观测质量。
 */
struct SelectedMarker
{
  MarkerConfiguration configuration;
  std::size_t detection_index{0U};
  double corner_area_px2{0.0};
  double distance_to_image_border_px{0.0};
};

/**
 * @brief 单帧 Marker 选择结果。
 *
 * selected_marker 只表示本帧可用于 PnP 的真实检测；active_marker_id 可以在短时漏检时继续
 * 保留，但不得被调用方解释为存在有效位姿。
 */
struct MarkerSelectionResult
{
  std::optional<SelectedMarker> selected_marker;
  std::optional<int> active_marker_id;
  double selected_corner_area_px2{std::numeric_limits<double>::quiet_NaN()};
  double selected_border_margin_px{std::numeric_limits<double>::quiet_NaN()};
  std::optional<int> challenger_marker_id;
  int challenger_stable_frames{0};
  MarkerSelectionReason selection_reason{MarkerSelectionReason::NO_VALID_CANDIDATE};
  bool active_changed{false};
};

/**
 * @brief 校验 Marker ID、边长和优先级配置。
 *
 * @throws std::invalid_argument 配置为空、ID 重复、边长非法或优先级为负。
 */
void validate_marker_configurations(
  const std::vector<MarkerConfiguration> & configurations);

/**
 * @brief 在多尺度 Marker 之间进行带质量门限和时间迟滞的有状态选择。
 *
 * 选择器仅依赖普通 C++ 数值结构。多 Marker 模式优先保持当前可靠 Marker，只有当前
 * Marker 降级后，连续可靠的挑战者才可接管；单 Marker 模式忽略尺度切换门限以兼容旧接口。
 */
class MarkerSelector
{
public:
  /**
   * @brief 构造选择器并校验配置与参数。
   *
   * @param configurations Marker 物理配置，ID 必须唯一，边长单位为米。
   * @param parameters 各 Marker 进入面积门限以及保持、边界、连续帧参数。
   * @throws std::invalid_argument 配置或参数非法时抛出。
   */
  MarkerSelector(
    std::vector<MarkerConfiguration> configurations,
    MarkerSelectorParameters parameters);

  /**
   * @brief 使用当前图像中的候选更新选择状态。
   *
   * @param candidates 本帧检测候选；面积单位为像素平方，边界距离单位为像素。
   * @return 本帧可用于 PnP 的候选、内部 active/challenger 状态及稳定原因。
   * @note 本帧没有 active 检测时不会返回陈旧 selected_marker。
   */
  MarkerSelectionResult update(
    const std::vector<MarkerDetectionCandidate> & candidates);

  /**
   * @brief 清除 active、challenger 和连续帧计数。
   */
  void reset();

private:
  bool isValidCandidate(const MarkerDetectionCandidate & candidate) const;
  bool isEntryReliable(const MarkerDetectionCandidate & candidate) const;
  bool isActiveReliable(const MarkerDetectionCandidate & candidate) const;
  std::optional<SelectedMarker> bestCandidate(
    const std::vector<MarkerDetectionCandidate> & candidates,
    bool require_entry_quality,
    std::optional<int> excluded_id = std::nullopt,
    std::optional<int> required_id = std::nullopt) const;
  bool isBetterCandidate(
    const SelectedMarker & candidate,
    const SelectedMarker & current) const;
  MarkerSelectionResult makeResult(
    const std::optional<SelectedMarker> & selected,
    MarkerSelectionReason reason,
    bool active_changed) const;
  void clearChallenger();

  std::vector<MarkerConfiguration> configurations_;
  MarkerSelectorParameters parameters_;
  std::unordered_map<int, MarkerConfiguration> configurations_by_id_;
  std::unordered_map<int, double> min_switch_area_by_id_;
  std::optional<int> active_marker_id_;
  std::optional<int> challenger_marker_id_;
  int challenger_stable_frames_{0};
  int active_missing_frames_{0};
  bool single_marker_compatibility_mode_{false};
};

}  // namespace aruco_detector

#endif  // ARUCO_DETECTOR__MARKER_SELECTOR_HPP_
