// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/marker_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace aruco_detector
{

void validate_marker_configurations(
  const std::vector<MarkerConfiguration> & configurations)
{
  if (configurations.empty()) {
    throw std::invalid_argument("at least one marker configuration is required");
  }

  std::unordered_set<int> ids;
  for (const auto & configuration : configurations) {
    if (configuration.id < 0) {
      throw std::invalid_argument("marker IDs must be non-negative");
    }
    if (!std::isfinite(configuration.length_m) || configuration.length_m <= 0.0) {
      throw std::invalid_argument("marker lengths must be finite and positive");
    }
    if (configuration.priority < 0) {
      throw std::invalid_argument("marker priorities must be non-negative");
    }
    if (!std::all_of(
        configuration.target_offset_marker_m.begin(),
        configuration.target_offset_marker_m.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument("marker target offsets must be finite");
    }
    if (!ids.insert(configuration.id).second) {
      throw std::invalid_argument("marker IDs must be unique");
    }
  }
}

MarkerSelector::MarkerSelector(
  std::vector<MarkerConfiguration> configurations,
  MarkerSelectorParameters parameters)
: configurations_(std::move(configurations)), parameters_(std::move(parameters))
{
  validate_marker_configurations(configurations_);

  if (parameters_.marker_min_switch_areas_px2.size() != configurations_.size()) {
    throw std::invalid_argument(
            "marker_min_switch_areas_px2 must match marker configuration count");
  }
  for (const double area : parameters_.marker_min_switch_areas_px2) {
    if (!std::isfinite(area) || area <= 0.0) {
      throw std::invalid_argument("marker minimum switch areas must be finite and positive");
    }
  }
  if (!std::isfinite(parameters_.active_hold_area_ratio) ||
    parameters_.active_hold_area_ratio <= 0.0 ||
    parameters_.active_hold_area_ratio > 1.0)
  {
    throw std::invalid_argument("active_hold_area_ratio must be in (0, 1]");
  }
  if (!std::isfinite(parameters_.minimum_border_margin_px) ||
    parameters_.minimum_border_margin_px < 0.0)
  {
    throw std::invalid_argument("minimum_border_margin_px must be finite and non-negative");
  }
  if (parameters_.switch_required_consecutive_frames < 1) {
    throw std::invalid_argument("switch_required_consecutive_frames must be at least 1");
  }
  if (parameters_.active_missing_grace_frames < 0) {
    throw std::invalid_argument("active_missing_grace_frames must be non-negative");
  }

  configurations_by_id_.reserve(configurations_.size());
  min_switch_area_by_id_.reserve(configurations_.size());
  for (std::size_t index = 0; index < configurations_.size(); ++index) {
    configurations_by_id_.emplace(configurations_[index].id, configurations_[index]);
    min_switch_area_by_id_.emplace(
      configurations_[index].id,
      parameters_.marker_min_switch_areas_px2[index]);
  }
  single_marker_compatibility_mode_ = configurations_.size() == 1U;
}

bool MarkerSelector::isValidCandidate(const MarkerDetectionCandidate & candidate) const
{
  return configurations_by_id_.find(candidate.id) != configurations_by_id_.end() &&
         std::isfinite(candidate.corner_area_px2) && candidate.corner_area_px2 > 0.0 &&
         std::isfinite(candidate.distance_to_image_border_px) &&
         candidate.distance_to_image_border_px >= 0.0;
}

bool MarkerSelector::isEntryReliable(const MarkerDetectionCandidate & candidate) const
{
  if (!isValidCandidate(candidate)) {
    return false;
  }
  return candidate.corner_area_px2 >= min_switch_area_by_id_.at(candidate.id) &&
         candidate.distance_to_image_border_px >= parameters_.minimum_border_margin_px;
}

bool MarkerSelector::isActiveReliable(const MarkerDetectionCandidate & candidate) const
{
  if (!isValidCandidate(candidate)) {
    return false;
  }
  const double hold_area =
    min_switch_area_by_id_.at(candidate.id) * parameters_.active_hold_area_ratio;
  return candidate.corner_area_px2 >= hold_area &&
         candidate.distance_to_image_border_px > parameters_.minimum_border_margin_px;
}

bool MarkerSelector::isBetterCandidate(
  const SelectedMarker & candidate,
  const SelectedMarker & current) const
{
  if (candidate.configuration.length_m != current.configuration.length_m) {
    return candidate.configuration.length_m > current.configuration.length_m;
  }
  if (candidate.configuration.priority != current.configuration.priority) {
    return candidate.configuration.priority < current.configuration.priority;
  }
  if (candidate.corner_area_px2 != current.corner_area_px2) {
    return candidate.corner_area_px2 > current.corner_area_px2;
  }
  if (candidate.configuration.id != current.configuration.id) {
    return candidate.configuration.id < current.configuration.id;
  }
  return candidate.detection_index < current.detection_index;
}

std::optional<SelectedMarker> MarkerSelector::bestCandidate(
  const std::vector<MarkerDetectionCandidate> & candidates,
  bool require_entry_quality,
  std::optional<int> excluded_id,
  std::optional<int> required_id) const
{
  std::optional<SelectedMarker> selected;
  for (const auto & candidate : candidates) {
    if ((excluded_id.has_value() && candidate.id == *excluded_id) ||
      (required_id.has_value() && candidate.id != *required_id) ||
      !isValidCandidate(candidate) ||
      (require_entry_quality && !isEntryReliable(candidate)))
    {
      continue;
    }

    const SelectedMarker current{
      configurations_by_id_.at(candidate.id),
      candidate.detection_index,
      candidate.corner_area_px2,
      candidate.distance_to_image_border_px};
    if (!selected.has_value() || isBetterCandidate(current, *selected)) {
      selected = current;
    }
  }
  return selected;
}

MarkerSelectionResult MarkerSelector::makeResult(
  const std::optional<SelectedMarker> & selected,
  MarkerSelectionReason reason,
  bool active_changed) const
{
  MarkerSelectionResult result;
  result.selected_marker = selected;
  if (selected.has_value()) {
    result.selected_corner_area_px2 = selected->corner_area_px2;
    result.selected_border_margin_px = selected->distance_to_image_border_px;
  }
  result.active_marker_id = active_marker_id_;
  result.challenger_marker_id = challenger_marker_id_;
  result.challenger_stable_frames = challenger_stable_frames_;
  result.selection_reason = reason;
  result.active_changed = active_changed;
  return result;
}

void MarkerSelector::clearChallenger()
{
  challenger_marker_id_.reset();
  challenger_stable_frames_ = 0;
}

MarkerSelectionResult MarkerSelector::update(
  const std::vector<MarkerDetectionCandidate> & candidates)
{
  if (single_marker_compatibility_mode_) {
    const int configured_id = configurations_.front().id;
    const auto selected = bestCandidate(
      candidates, false, std::nullopt, configured_id);
    if (selected.has_value()) {
      const bool active_changed =
        !active_marker_id_.has_value() || *active_marker_id_ != configured_id;
      active_marker_id_ = configured_id;
      active_missing_frames_ = 0;
      clearChallenger();
      return makeResult(
        selected,
        active_changed ? MarkerSelectionReason::INITIAL_ACQUIRE :
        MarkerSelectionReason::HOLD_ACTIVE,
        active_changed);
    }

    clearChallenger();
    if (!active_marker_id_.has_value()) {
      return makeResult(std::nullopt, MarkerSelectionReason::NO_VALID_CANDIDATE, false);
    }

    ++active_missing_frames_;
    if (active_missing_frames_ > parameters_.active_missing_grace_frames) {
      active_marker_id_.reset();
      active_missing_frames_ = 0;
      return makeResult(std::nullopt, MarkerSelectionReason::ACTIVE_CLEARED, true);
    }
    return makeResult(std::nullopt, MarkerSelectionReason::ACTIVE_MISSING, false);
  }

  if (!active_marker_id_.has_value()) {
    const auto initial = bestCandidate(candidates, true);
    if (!initial.has_value()) {
      clearChallenger();
      return makeResult(std::nullopt, MarkerSelectionReason::NO_VALID_CANDIDATE, false);
    }

    active_marker_id_ = initial->configuration.id;
    active_missing_frames_ = 0;
    clearChallenger();
    return makeResult(initial, MarkerSelectionReason::INITIAL_ACQUIRE, true);
  }

  const int active_id = *active_marker_id_;
  const auto active_candidate = bestCandidate(
    candidates, false, std::nullopt, active_id);

  MarkerSelectionReason degraded_reason = MarkerSelectionReason::ACTIVE_MISSING;
  std::optional<SelectedMarker> challenger;
  if (!active_candidate.has_value()) {
    ++active_missing_frames_;
  } else {
    active_missing_frames_ = 0;
    const double hold_area =
      min_switch_area_by_id_.at(active_id) * parameters_.active_hold_area_ratio;
    if (active_candidate->corner_area_px2 < hold_area) {
      degraded_reason = MarkerSelectionReason::ACTIVE_AREA_LOW;
    } else if (
      active_candidate->distance_to_image_border_px <=
      parameters_.minimum_border_margin_px)
    {
      degraded_reason = MarkerSelectionReason::ACTIVE_NEAR_BORDER;
    } else if (isActiveReliable(MarkerDetectionCandidate{
        active_candidate->configuration.id,
        active_candidate->corner_area_px2,
        active_candidate->distance_to_image_border_px,
        active_candidate->detection_index}))
    {
      // 当前可靠 Marker 只禁止更小尺度抢占。若后续出现严格更大的可靠 Marker，
      // 允许其按同样连续帧迟滞回退接管，避免初次捕获较小 Marker 后永久锁定。
      challenger = bestCandidate(candidates, true, active_id);
      if (!challenger.has_value() ||
        challenger->configuration.length_m <= active_candidate->configuration.length_m)
      {
        clearChallenger();
        return makeResult(active_candidate, MarkerSelectionReason::HOLD_ACTIVE, false);
      }
    }
  }

  if (!challenger.has_value()) {
    challenger = bestCandidate(candidates, true, active_id);
  }
  if (challenger.has_value()) {
    if (challenger_marker_id_.has_value() &&
      *challenger_marker_id_ == challenger->configuration.id)
    {
      ++challenger_stable_frames_;
    } else {
      challenger_marker_id_ = challenger->configuration.id;
      challenger_stable_frames_ = 1;
    }

    if (challenger_stable_frames_ >= parameters_.switch_required_consecutive_frames) {
      active_marker_id_ = challenger->configuration.id;
      active_missing_frames_ = 0;
      clearChallenger();
      return makeResult(challenger, MarkerSelectionReason::SWITCH_STABLE, true);
    }

    return makeResult(
      active_candidate,
      MarkerSelectionReason::CHALLENGER_STABILIZING,
      false);
  }

  clearChallenger();
  if (!active_candidate.has_value() &&
    active_missing_frames_ > parameters_.active_missing_grace_frames)
  {
    active_marker_id_.reset();
    active_missing_frames_ = 0;
    return makeResult(std::nullopt, MarkerSelectionReason::ACTIVE_CLEARED, true);
  }

  return makeResult(active_candidate, degraded_reason, false);
}

void MarkerSelector::reset()
{
  active_marker_id_.reset();
  active_missing_frames_ = 0;
  clearChallenger();
}

}  // namespace aruco_detector
