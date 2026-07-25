// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/marker_selector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

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

std::optional<SelectedMarker> select_marker(
  const std::vector<MarkerConfiguration> & configurations,
  const std::vector<MarkerDetectionCandidate> & candidates)
{
  validate_marker_configurations(configurations);

  std::unordered_map<int, MarkerConfiguration> by_id;
  by_id.reserve(configurations.size());
  for (const auto & configuration : configurations) {
    by_id.emplace(configuration.id, configuration);
  }

  std::optional<SelectedMarker> selected;
  for (const auto & candidate : candidates) {
    const auto configuration = by_id.find(candidate.id);
    if (configuration == by_id.end() ||
      !std::isfinite(candidate.corner_area_px2) || candidate.corner_area_px2 <= 0.0)
    {
      continue;
    }

    const SelectedMarker current{
      configuration->second,
      candidate.detection_index,
      candidate.corner_area_px2};
    if (!selected.has_value() ||
      current.configuration.priority < selected->configuration.priority ||
      (current.configuration.priority == selected->configuration.priority &&
      current.corner_area_px2 > selected->corner_area_px2))
    {
      selected = current;
    }
  }
  return selected;
}

}  // namespace aruco_detector
