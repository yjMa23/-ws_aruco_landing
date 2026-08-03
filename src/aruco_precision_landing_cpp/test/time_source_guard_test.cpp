// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/time_source_guard.hpp"

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

TEST(TimeSourceGuardTest, ComputesElapsedTimeForMatchingRosClock)
{
  const rclcpp::Time earlier(1'000'000'000LL, RCL_ROS_TIME);
  const rclcpp::Time later(2'250'000'000LL, RCL_ROS_TIME);

  const auto elapsed_s = elapsed_seconds_same_clock(later, earlier);

  ASSERT_TRUE(elapsed_s.has_value());
  EXPECT_DOUBLE_EQ(*elapsed_s, 1.25);
}

TEST(TimeSourceGuardTest, RejectsMixedClockSourcesWithoutThrowing)
{
  const rclcpp::Time system_time(1'000'000'000LL, RCL_SYSTEM_TIME);
  const rclcpp::Time ros_time(2'000'000'000LL, RCL_ROS_TIME);

  EXPECT_NO_THROW({
    const auto elapsed_s = elapsed_seconds_same_clock(ros_time, system_time);
    EXPECT_FALSE(elapsed_s.has_value());
  });
}

TEST(TimeSourceGuardTest, PreservesNegativeElapsedTimeForCallerValidation)
{
  const rclcpp::Time earlier(2'000'000'000LL, RCL_ROS_TIME);
  const rclcpp::Time later(1'000'000'000LL, RCL_ROS_TIME);

  const auto elapsed_s = elapsed_seconds_same_clock(later, earlier);

  ASSERT_TRUE(elapsed_s.has_value());
  EXPECT_DOUBLE_EQ(*elapsed_s, -1.0);
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
