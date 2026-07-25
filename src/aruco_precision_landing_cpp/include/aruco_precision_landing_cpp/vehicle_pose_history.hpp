// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__VEHICLE_POSE_HISTORY_HPP_
#define ARUCO_PRECISION_LANDING_CPP__VEHICLE_POSE_HISTORY_HPP_

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"

#include <cstddef>
#include <deque>
#include <optional>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 无人机 local NED 位姿历史参数。
 */
struct VehiclePoseHistoryParameters
{
  double history_duration_s{2.0};
  double max_endpoint_hold_s{0.03};
};

/**
 * @brief 保存 PX4 local NED 机体位姿，并按采样时间插值查询。
 *
 * 位置使用线性插值，姿态使用最短路径四元数 Slerp。该类只接受严格单调递增的
 * ROS 时间秒，不依赖 ROS 消息或 PX4 类型。
 */
class VehiclePoseHistory
{
public:
  /**
   * @brief 创建位姿历史并校验参数。
   *
   * @param parameters 历史时长和端点保持时间，单位均为秒。
   * @throws std::invalid_argument 参数非有限、历史时长非正或端点保持时间为负。
   */
  explicit VehiclePoseHistory(const VehiclePoseHistoryParameters & parameters);

  /**
   * @brief 写入一帧 local NED 机体位姿。
   *
   * @param pose 无人机 body FRD 在 local NED 中的位姿，位置单位为米。
   * @param sample_time_s 与图像时间戳同一 ROS 时间域中的采样时间，单位为秒。
   * @return 输入有效且时间严格晚于上一帧时返回 true，否则返回 false；失败时历史不变。
   */
  bool add_sample(const Pose3d & pose, double sample_time_s);

  /**
   * @brief 查询指定 ROS 时间对应的 local NED 机体位姿。
   *
   * @param query_time_s 图像采样时间，单位为秒。
   * @return 查询位于历史范围内时返回插值位姿；距离首尾端点不超过
   *         `max_endpoint_hold_s` 时返回对应端点；历史不足或时间超限时返回
   *         `std::nullopt`。
   */
  std::optional<Pose3d> lookup(double query_time_s) const;

  /**
   * @brief 清除全部位姿和时间历史。
   */
  void reset();

  /**
   * @brief 返回当前保存的样本数量，仅用于状态检查和单元测试。
   */
  std::size_t size() const;

private:
  struct Sample
  {
    double time_s{0.0};
    Pose3d pose{};
  };

  VehiclePoseHistoryParameters parameters_;
  std::deque<Sample> samples_;
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__VEHICLE_POSE_HISTORY_HPP_
