// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_detector/marker_selector.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_detector
{
namespace
{

std::vector<MarkerConfiguration> configurations()
{
  return {
    MarkerConfiguration{0, 0.50, 0},
    MarkerConfiguration{1, 0.16, 1},
    MarkerConfiguration{2, 0.16, 1},
  };
}

TEST(MarkerSelectorTest, LowerPriorityNumberWinsEvenWithSmallerImageArea)
{
  const auto selected = select_marker(
    configurations(),
    {
      MarkerDetectionCandidate{1, 5000.0, 4U},
      MarkerDetectionCandidate{0, 1000.0, 2U},
    });

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->configuration.id, 0);
  EXPECT_DOUBLE_EQ(selected->configuration.length_m, 0.50);
  EXPECT_EQ(selected->detection_index, 2U);
}

TEST(MarkerSelectorTest, LargestAreaWinsWithinSamePriority)
{
  const auto selected = select_marker(
    configurations(),
    {
      MarkerDetectionCandidate{1, 1200.0, 0U},
      MarkerDetectionCandidate{2, 2400.0, 3U},
    });

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->configuration.id, 2);
  EXPECT_EQ(selected->detection_index, 3U);
  EXPECT_DOUBLE_EQ(selected->corner_area_px2, 2400.0);
}

TEST(MarkerSelectorTest, UnconfiguredAndInvalidCandidatesAreIgnored)
{
  const auto selected = select_marker(
    configurations(),
    {
      MarkerDetectionCandidate{42, 5000.0, 0U},
      MarkerDetectionCandidate{1, -1.0, 1U},
      MarkerDetectionCandidate{2, std::numeric_limits<double>::quiet_NaN(), 2U},
    });

  EXPECT_FALSE(selected.has_value());
}

TEST(MarkerSelectorTest, SelectedConfigurationKeepsPerIdPhysicalLength)
{
  const auto selected = select_marker(
    configurations(),
    {MarkerDetectionCandidate{1, 1000.0, 7U}});

  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->configuration.id, 1);
  EXPECT_DOUBLE_EQ(selected->configuration.length_m, 0.16);
}

TEST(MarkerSelectorTest, RejectsEmptyConfiguration)
{
  EXPECT_THROW(
    select_marker({}, {MarkerDetectionCandidate{0, 100.0, 0U}}),
    std::invalid_argument);
}

TEST(MarkerSelectorTest, RejectsDuplicateIds)
{
  auto invalid = configurations();
  invalid.push_back(MarkerConfiguration{1, 0.20, 2});
  EXPECT_THROW(validate_marker_configurations(invalid), std::invalid_argument);
}

TEST(MarkerSelectorTest, RejectsInvalidLengthsAndPriorities)
{
  auto invalid = configurations();
  invalid[0].length_m = 0.0;
  EXPECT_THROW(validate_marker_configurations(invalid), std::invalid_argument);

  invalid = configurations();
  invalid[0].priority = -1;
  EXPECT_THROW(validate_marker_configurations(invalid), std::invalid_argument);

  invalid = configurations();
  invalid[0].id = -1;
  EXPECT_THROW(validate_marker_configurations(invalid), std::invalid_argument);
}

}  // namespace
}  // namespace aruco_detector
