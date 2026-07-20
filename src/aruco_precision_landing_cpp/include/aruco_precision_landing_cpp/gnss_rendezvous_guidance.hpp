// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__GNSS_RENDEZVOUS_GUIDANCE_HPP_
#define ARUCO_PRECISION_LANDING_CPP__GNSS_RENDEZVOUS_GUIDANCE_HPP_

#include "aruco_precision_landing_cpp/geodetic_converter.hpp"

#include <Eigen/Core>

#include <optional>

namespace aruco_precision_landing_cpp
{

/**
 * @brief GNSS 会合阶段的校验、局部坐标转换和目标限幅参数。
 */
struct GnssRendezvousParameters
{
  double fix_timeout_s{1.0};
  double velocity_timeout_s{1.0};
  double stable_duration_s{1.0};
  double max_fix_jump_m{5.0};
  double max_target_step_m{0.20};
  double max_target_speed_mps{2.0};
  double search_offset_m{1.0};
  double search_point_hold_s{1.0};
  double max_geodetic_range_m{10000.0};
};

/**
 * @brief 保存已接受的船舶 GNSS 在 PX4 local NED 中的状态。
 */
struct DeckGnssEstimate
{
  Eigen::Vector3d position_ned{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned{Eigen::Vector3d::Zero()};
  double fix_receive_time_s{0.0};
  double velocity_receive_time_s{0.0};
};

/**
 * @brief 处理船舶 GNSS 粗引导，不依赖 ROS 节点或消息类型。
 *
 * 类内部使用 PX4 local NED 地理参考将 WGS84 船舶位置转换到 local NED，
 * 拒绝超时、跳变和非法输入，并生成限速后的会合目标及移动搜索偏移。
 */
class GnssRendezvousGuidance
{
public:
  /**
   * @brief 创建 GNSS 会合引导器并校验所有参数。
   *
   * @param parameters 超时、跳变阈值、目标限幅和搜索参数。
   * @throws std::invalid_argument 参数为 NaN、Inf、非正数或不满足范围约束时抛出。
   */
  explicit GnssRendezvousGuidance(const GnssRendezvousParameters & parameters);

  /**
   * @brief 设置 PX4 local NED 原点对应的 WGS84 参考。
   *
   * @param reference PX4 `VehicleLocalPosition.ref_lat/ref_lon/ref_alt`，单位分别为度、度、米。
   * @return 参考有效时返回 true；无效时返回 false。参考变化会清空已有 GNSS 状态。
   */
  bool set_local_reference(const Wgs84Position & reference);

  /**
   * @brief 接收一帧船舶 WGS84 位置并转换为 PX4 local NED。
   *
   * @param fix 船舶 GNSS，经纬度单位为度，高度单位为米。
   * @param receive_time_s 控制器本地单调时间或同一时钟域时间，单位为秒。
   * @return 位置有效、未超出局部范围且未触发跳变拒绝时返回 true。
   */
  bool ingest_fix(const Wgs84Position & fix, double receive_time_s);

  /**
   * @brief 接收船舶在 Gazebo world ENU 中的速度。
   *
   * @param velocity_enu ENU 速度，单位为米每秒，分量为 East、North、Up。
   * @param receive_time_s 控制器本地单调时间或同一时钟域时间，单位为秒。
   * @return 输入有限且时间有效时返回 true。
   */
  bool ingest_velocity_enu(const Eigen::Vector3d & velocity_enu, double receive_time_s);

  /**
   * @brief 清空位置、速度和稳定计时，但保留地理参考。
   */
  void reset_measurements();

  /**
   * @brief 判断位置与速度是否新鲜且已连续稳定达到设定时长。
   *
   * @param now_s 与接收时间相同的时钟域，单位为秒。
   * @return 满足会合使用条件时返回 true。
   */
  bool ready(double now_s) const;

  /**
   * @brief 获取当前新鲜的船舶 local NED 估计。
   *
   * @param now_s 与接收时间相同的时钟域，单位为秒。
   * @return 位置和速度均新鲜时返回估计，否则返回 std::nullopt。
   */
  std::optional<DeckGnssEstimate> estimate(double now_s) const;

  /**
   * @brief 按最大速度和单周期最大步长限制水平目标移动。
   *
   * @param current_target_xy 当前 PX4 local NED 水平目标，单位为米。
   * @param desired_target_xy 期望 PX4 local NED 水平目标，单位为米。
   * @param dt_s 控制周期，单位为秒，必须为有限正数。
   * @return 限幅后的 NED 水平目标；输入无效时返回 std::nullopt。
   */
  std::optional<Eigen::Vector2d> limit_target_xy(
    const Eigen::Vector2d & current_target_xy,
    const Eigen::Vector2d & desired_target_xy,
    double dt_s) const;

  /**
   * @brief 生成相对实时船舶 GNSS 中心的搜索偏移。
   *
   * 搜索顺序为中心、北、东、南、西，每个点保持 `search_point_hold_s`。
   *
   * @param elapsed_s 进入搜索状态后的时间，单位为秒，必须有限且非负。
   * @return NED 水平偏移，单位为米；输入无效时返回 std::nullopt。
   */
  std::optional<Eigen::Vector2d> search_offset(double elapsed_s) const;

  /**
   * @brief 返回是否已有有效 PX4 地理参考。
   */
  bool has_local_reference() const;

private:
  bool fix_is_fresh(double now_s) const;
  bool velocity_is_fresh(double now_s) const;

  GnssRendezvousParameters parameters_{};
  std::optional<Wgs84Position> reference_{};
  std::optional<GeodeticConverter> converter_{};
  std::optional<Eigen::Vector3d> position_ned_{};
  std::optional<Eigen::Vector3d> velocity_ned_{};
  double last_fix_receive_time_s_{0.0};
  double last_velocity_receive_time_s_{0.0};
  double fix_stable_since_s_{0.0};
  double velocity_stable_since_s_{0.0};
  bool have_fix_stable_since_{false};
  bool have_velocity_stable_since_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__GNSS_RENDEZVOUS_GUIDANCE_HPP_
