// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/marker_selector.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace aruco_detector
{
namespace
{

std::vector<MarkerConfiguration> configurations()
{
  return {
    MarkerConfiguration{0, 0.50, 3},
    MarkerConfiguration{1, 0.20, 2},
    MarkerConfiguration{2, 0.04, 1},
    MarkerConfiguration{3, 0.02, 0},
  };
}

MarkerSelectorParameters parameters()
{
  MarkerSelectorParameters result;
  result.marker_min_switch_areas_px2 = {400.0, 400.0, 400.0, 400.0};
  result.active_hold_area_ratio = 0.60;
  result.minimum_border_margin_px = 12.0;
  result.switch_required_consecutive_frames = 3;
  result.active_missing_grace_frames = 2;
  return result;
}

MarkerDetectionCandidate candidate(
  int id,
  double area = 1000.0,
  double border = 30.0,
  std::size_t index = 0U)
{
  return MarkerDetectionCandidate{id, area, border, index};
}

TEST(MarkerSelectorTest, RejectsInvalidConfigurations)
{
  EXPECT_THROW(MarkerSelector({}, parameters()), std::invalid_argument);

  auto invalid = configurations();
  invalid.push_back(MarkerConfiguration{1, 0.10, 0});
  EXPECT_THROW(MarkerSelector(invalid, parameters()), std::invalid_argument);

  invalid = configurations();
  invalid[0].length_m = 0.0;
  EXPECT_THROW(MarkerSelector(invalid, parameters()), std::invalid_argument);

  invalid = configurations();
  invalid[0].priority = -1;
  EXPECT_THROW(MarkerSelector(invalid, parameters()), std::invalid_argument);

  invalid = configurations();
  invalid[0].id = -1;
  EXPECT_THROW(MarkerSelector(invalid, parameters()), std::invalid_argument);
}

TEST(MarkerSelectorTest, RejectsInvalidParameters)
{
  auto invalid = parameters();
  invalid.marker_min_switch_areas_px2.pop_back();
  EXPECT_THROW(MarkerSelector(configurations(), invalid), std::invalid_argument);

  invalid = parameters();
  invalid.marker_min_switch_areas_px2[0] = 0.0;
  EXPECT_THROW(MarkerSelector(configurations(), invalid), std::invalid_argument);

  invalid = parameters();
  invalid.active_hold_area_ratio = 1.1;
  EXPECT_THROW(MarkerSelector(configurations(), invalid), std::invalid_argument);

  invalid = parameters();
  invalid.minimum_border_margin_px = -1.0;
  EXPECT_THROW(MarkerSelector(configurations(), invalid), std::invalid_argument);

  invalid = parameters();
  invalid.switch_required_consecutive_frames = 0;
  EXPECT_THROW(MarkerSelector(configurations(), invalid), std::invalid_argument);

  invalid = parameters();
  invalid.active_missing_grace_frames = -1;
  EXPECT_THROW(MarkerSelector(configurations(), invalid), std::invalid_argument);
}

TEST(MarkerSelectorTest, InitialAcquireChoosesLargestReliablePhysicalMarker)
{
  MarkerSelector selector(configurations(), parameters());
  const auto result = selector.update({
      candidate(3, 3000.0, 30.0, 3U),
      candidate(1, 1000.0, 30.0, 1U),
      candidate(0, 500.0, 30.0, 0U),
    });

  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 0);
  EXPECT_EQ(result.active_marker_id, 0);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::INITIAL_ACQUIRE);
  EXPECT_TRUE(result.active_changed);
}

TEST(MarkerSelectorTest, HigherPrioritySmallMarkerCannotEnterBelowAreaThreshold)
{
  MarkerSelector selector(configurations(), parameters());
  const auto result = selector.update({
      candidate(0, 500.0),
      candidate(3, 399.0),
    });

  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 0);
}

TEST(MarkerSelectorTest, ReliableActiveIgnoresPersistentSmallerMarkers)
{
  MarkerSelector selector(configurations(), parameters());
  ASSERT_TRUE(selector.update({candidate(0)}).selected_marker.has_value());

  for (int frame = 0; frame < 10; ++frame) {
    const auto result = selector.update({candidate(0), candidate(1), candidate(3, 5000.0)});
    ASSERT_TRUE(result.selected_marker.has_value());
    EXPECT_EQ(result.selected_marker->configuration.id, 0);
    EXPECT_EQ(result.active_marker_id, 0);
    EXPECT_FALSE(result.challenger_marker_id.has_value());
    EXPECT_EQ(result.challenger_stable_frames, 0);
    EXPECT_EQ(result.selection_reason, MarkerSelectionReason::HOLD_ACTIVE);
  }
}

TEST(MarkerSelectorTest, ReliableSmallerActiveCanFallBackToLargerMarker)
{
  MarkerSelector selector(configurations(), parameters());
  auto result = selector.update({candidate(1)});
  ASSERT_EQ(result.active_marker_id, 1);

  result = selector.update({candidate(1), candidate(0)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 1);
  EXPECT_EQ(result.challenger_marker_id, 0);
  EXPECT_EQ(result.challenger_stable_frames, 1);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::CHALLENGER_STABILIZING);

  selector.update({candidate(1), candidate(0)});
  result = selector.update({candidate(1), candidate(0)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 0);
  EXPECT_EQ(result.active_marker_id, 0);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::SWITCH_STABLE);
}

TEST(MarkerSelectorTest, NearBorderStartsChallengerAndSwitchesOnRequiredFrame)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});

  auto result = selector.update({candidate(0, 1000.0, 12.0), candidate(1)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 0);
  EXPECT_EQ(result.challenger_marker_id, 1);
  EXPECT_EQ(result.challenger_stable_frames, 1);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::CHALLENGER_STABILIZING);

  result = selector.update({candidate(0, 1000.0, 8.0), candidate(1)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 0);
  EXPECT_EQ(result.challenger_stable_frames, 2);

  result = selector.update({candidate(0, 1000.0, 8.0), candidate(1)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 1);
  EXPECT_EQ(result.active_marker_id, 1);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::SWITCH_STABLE);
  EXPECT_TRUE(result.active_changed);
}

TEST(MarkerSelectorTest, NearBorderWithoutChallengerReportsReason)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});

  const auto result = selector.update({candidate(0, 1000.0, 12.0)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::ACTIVE_NEAR_BORDER);
}

TEST(MarkerSelectorTest, ChallengerInterruptionResetsStableCount)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});

  auto result = selector.update({candidate(0, 1000.0, 5.0), candidate(1)});
  EXPECT_EQ(result.challenger_stable_frames, 1);

  result = selector.update({candidate(0, 1000.0, 5.0)});
  EXPECT_FALSE(result.challenger_marker_id.has_value());
  EXPECT_EQ(result.challenger_stable_frames, 0);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::ACTIVE_NEAR_BORDER);

  result = selector.update({candidate(0, 1000.0, 5.0), candidate(1, 399.0)});
  EXPECT_FALSE(result.challenger_marker_id.has_value());
  EXPECT_EQ(result.challenger_stable_frames, 0);

  result = selector.update({candidate(0, 1000.0, 5.0), candidate(1, 1000.0, 11.0)});
  EXPECT_FALSE(result.challenger_marker_id.has_value());
  EXPECT_EQ(result.challenger_stable_frames, 0);
}

TEST(MarkerSelectorTest, LargestReliableChallengerAvoidsCrossScaleJump)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});

  const auto result = selector.update({
      candidate(0, 1000.0, 5.0),
      candidate(1),
      candidate(2),
      candidate(3),
    });
  EXPECT_EQ(result.challenger_marker_id, 1);
}

TEST(MarkerSelectorTest, HoldAreaHysteresisKeepsActiveBelowEntryThreshold)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0, 500.0)});

  const auto result = selector.update({candidate(0, 300.0), candidate(1, 2000.0)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 0);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::HOLD_ACTIVE);
  EXPECT_FALSE(result.challenger_marker_id.has_value());
}

TEST(MarkerSelectorTest, AreaBelowHoldThresholdAllowsSwitch)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0, 500.0)});

  auto result = selector.update({candidate(0, 239.0), candidate(1)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::CHALLENGER_STABILIZING);
  EXPECT_EQ(result.challenger_marker_id, 1);

  selector.update({candidate(0, 239.0), candidate(1)});
  result = selector.update({candidate(0, 239.0), candidate(1)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 1);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::SWITCH_STABLE);
}

TEST(MarkerSelectorTest, AreaLowWithoutChallengerReportsReason)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0, 500.0)});

  const auto result = selector.update({candidate(0, 200.0)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::ACTIVE_AREA_LOW);
}

TEST(MarkerSelectorTest, SingleMissingFrameKeepsInternalActiveButReturnsNoPose)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});

  const auto result = selector.update({});
  EXPECT_FALSE(result.selected_marker.has_value());
  EXPECT_EQ(result.active_marker_id, 0);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::ACTIVE_MISSING);
  EXPECT_TRUE(std::isnan(result.selected_corner_area_px2));
  EXPECT_TRUE(std::isnan(result.selected_border_margin_px));
}

TEST(MarkerSelectorTest, MissingActiveSwitchesAfterStableChallenger)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});

  auto result = selector.update({candidate(1)});
  EXPECT_FALSE(result.selected_marker.has_value());
  EXPECT_EQ(result.challenger_stable_frames, 1);

  selector.update({candidate(1)});
  result = selector.update({candidate(1)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 1);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::SWITCH_STABLE);
}

TEST(MarkerSelectorTest, MissingBeyondGraceClearsActiveWithoutChallenger)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});

  EXPECT_EQ(selector.update({}).selection_reason, MarkerSelectionReason::ACTIVE_MISSING);
  EXPECT_EQ(selector.update({}).selection_reason, MarkerSelectionReason::ACTIVE_MISSING);
  const auto result = selector.update({});
  EXPECT_FALSE(result.selected_marker.has_value());
  EXPECT_FALSE(result.active_marker_id.has_value());
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::ACTIVE_CLEARED);
  EXPECT_TRUE(result.active_changed);
}

TEST(MarkerSelectorTest, SmallMarkerCanStablyFallBackToLargerMarker)
{
  MarkerSelector selector(configurations(), parameters());
  auto result = selector.update({candidate(3)});
  ASSERT_EQ(result.active_marker_id, 3);

  result = selector.update({candidate(3, 200.0), candidate(1)});
  EXPECT_EQ(result.challenger_marker_id, 1);
  selector.update({candidate(3, 200.0), candidate(1)});
  result = selector.update({candidate(3, 200.0), candidate(1)});

  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 1);
  EXPECT_EQ(result.active_marker_id, 1);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::SWITCH_STABLE);
}

TEST(MarkerSelectorTest, SamePhysicalLengthUsesPriorityThenAreaForDeterminism)
{
  auto same_length = configurations();
  same_length[1].length_m = 0.20;
  same_length[2].length_m = 0.20;
  MarkerSelector selector(same_length, parameters());

  auto result = selector.update({candidate(1, 500.0), candidate(2, 5000.0)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 2);

  same_length[1].priority = 1;
  same_length[2].priority = 1;
  MarkerSelector area_selector(same_length, parameters());
  result = area_selector.update({candidate(1, 500.0), candidate(2, 1000.0)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 2);
}

TEST(MarkerSelectorTest, InvalidUnknownAndOutOfImageCandidatesAreIgnored)
{
  MarkerSelector selector(configurations(), parameters());
  const auto result = selector.update({
      candidate(42),
      candidate(0, std::numeric_limits<double>::quiet_NaN()),
      candidate(1, -1.0),
      candidate(2, 1000.0, std::numeric_limits<double>::infinity()),
      candidate(3, 1000.0, -0.1),
    });

  EXPECT_FALSE(result.selected_marker.has_value());
  EXPECT_FALSE(result.active_marker_id.has_value());
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::NO_VALID_CANDIDATE);
}

TEST(MarkerSelectorTest, ResetClearsActiveAndChallengerState)
{
  MarkerSelector selector(configurations(), parameters());
  selector.update({candidate(0)});
  selector.update({candidate(0, 1000.0, 5.0), candidate(1)});

  selector.reset();
  const auto result = selector.update({candidate(2)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 2);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::INITIAL_ACQUIRE);
  EXPECT_EQ(result.challenger_stable_frames, 0);
}

TEST(MarkerSelectorTest, SingleMarkerModePreservesLegacyPositiveAreaBehavior)
{
  MarkerSelectorParameters legacy_parameters;
  legacy_parameters.marker_min_switch_areas_px2 = {400.0};
  legacy_parameters.minimum_border_margin_px = 12.0;
  legacy_parameters.switch_required_consecutive_frames = 5;
  legacy_parameters.active_missing_grace_frames = 1;
  MarkerSelector selector({MarkerConfiguration{7, 0.12, 0}}, legacy_parameters);

  auto result = selector.update({candidate(7, 1.0, 0.0, 4U)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selected_marker->configuration.id, 7);
  EXPECT_EQ(result.selected_marker->detection_index, 4U);
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::INITIAL_ACQUIRE);

  result = selector.update({candidate(7, 2.0, 0.0, 5U)});
  ASSERT_TRUE(result.selected_marker.has_value());
  EXPECT_EQ(result.selection_reason, MarkerSelectionReason::HOLD_ACTIVE);
}

}  // namespace
}  // namespace aruco_detector
