// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__DECK_MOTION_ESTIMATOR_HPP_
#define ARUCO_PRECISION_LANDING_CPP__DECK_MOTION_ESTIMATOR_HPP_

#include "aruco_precision_landing_cpp/coordinate_transform.hpp"

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aruco_precision_landing_cpp
{

/**
 * @brief 甲板完整刚体运动 shadow 估计与预测参数。
 *
 * 平移量使用 `uav_centered_ned`（甲板减无人机，轴平行 local NED），姿态表示
 * deck_landing_up 到 local NED 的旋转；过程噪声分别按白噪声 jerk 和白噪声角
 * jerk 建模。该参数只影响诊断，不进入生产控制。
 */
struct DeckMotionEstimatorParameters
{
  double linear_jerk_std_mps3{1.0};
  double angular_jerk_std_radps3{0.20};
  double measurement_horizontal_std_m{0.08};
  double measurement_vertical_std_m{0.12};
  double measurement_orientation_std_rad{0.02617993877991494};
  double initial_position_std_m{0.20};
  double initial_velocity_std_mps{1.0};
  double initial_acceleration_std_mps2{1.0};
  double initial_orientation_std_rad{0.05235987755982988};
  double initial_angular_velocity_std_radps{0.17453292519943295};
  double initial_angular_acceleration_std_radps2{0.17453292519943295};
  double minimum_sample_dt_s{0.001};
  double maximum_sample_dt_s{0.25};
  double reinitialize_gap_s{2.0};
  double position_innovation_gate_mahalanobis{5.0};
  double orientation_innovation_gate_mahalanobis{5.0};
  double minimum_upward_normal_component{0.50};
  double prediction_sample_period_s{0.05};
  double trusted_prediction_horizon_s{0.50};
  double maximum_prediction_horizon_s{1.00};
  double kinematic_fit_window_s{0.30};
};

enum class DeckMotionUpdateStatus
{
  kInitialized,
  kUpdated,
  kReinitialized,
  kRejectedInvalidInput,
  kRejectedNonMonotonicTime,
  kRejectedOutlier
};

/**
 * @brief 图像采样时刻的甲板刚体状态估计。
 *
 * position 是图像时刻的 `deck-uav` 相对位置；velocity/acceleration 是甲板自身
 * 运动，均在 NED 轴表达。角速度和角加速度也是甲板自身运动；
 * orientation_deck_to_ned 把
 * deck_landing_up 向量旋转到 local NED。
 */
struct DeckMotionEstimate
{
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_ned_mps2{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_deck_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_ned_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_ned_radps2{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 9, 9> translation_covariance{
    Eigen::Matrix<double, 9, 9>::Zero()};
  Eigen::Matrix<double, 9, 9> rotation_covariance{
    Eigen::Matrix<double, 9, 9>::Zero()};
  double sample_time_s{0.0};
  std::int32_t marker_id{-1};
};

struct DeckMotionUpdateResult
{
  DeckMotionUpdateStatus status{DeckMotionUpdateStatus::kRejectedInvalidInput};
  double position_normalized_innovation_squared{0.0};
  double orientation_normalized_innovation_squared{0.0};
  std::optional<DeckMotionEstimate> estimate;
};

/**
 * @brief 相对当前发布时刻的一点甲板刚体预测。
 */
struct DeckMotionPredictionPoint
{
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_ned_mps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration_ned_mps2{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_deck_to_ned{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_ned_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_ned_radps2{Eigen::Vector3d::Zero()};
  double relative_time_s{0.0};
  bool trusted{false};
};

struct DeckMotionPrediction
{
  std::vector<DeckMotionPredictionPoint> points;
  double observation_age_s{0.0};
  double trusted_horizon_s{0.0};
};

/**
 * @brief 使用常加速度误差状态 Kalman Filter 估计并预测甲板 6-DoF 运动。
 *
 * 输入只能是完成图像时刻对齐后的 ArUco 甲板中心相对位姿和 PX4 NED
 * 速度。类不依赖 ROS、不读取 Ground Truth，也不产生控制输出。时间倒退、非法旋转、
 * 向上法向错误或创新离群时拒绝本次观测；超过重初始化间隔后以新位姿和零相对速度假设重建。
 */
class DeckMotionEstimator
{
public:
  explicit DeckMotionEstimator(const DeckMotionEstimatorParameters & parameters);

  /**
   * @brief 使用一帧甲板中心位姿更新完整刚体状态。
   *
   * @param pose_ned 甲板中心在图像时刻 `uav_centered_ned` 中的相对位姿。
   * @param uav_velocity_ned_mps 图像时刻无人机 NED 速度，用于补偿观测原点变化。
   * @param marker_id 产生该观测的 Marker ID，仅用于连续性诊断。
   * @param sample_time_s 图像采样时间，单位秒，必须严格递增。
   * @return 更新状态、位置/姿态 NIS 和当前可选估计。
   */
  DeckMotionUpdateResult update(
    const Pose3d & pose_ned,
    const Eigen::Vector3d & uav_velocity_ned_mps,
    std::int32_t marker_id,
    double sample_time_s);

  /**
   * @brief 返回图像采样时刻的当前估计。
   */
  std::optional<DeckMotionEstimate> estimate() const;

  /**
   * @brief 从最后观测外推到当前时刻并生成受限预测轨迹。
   *
   * @param now_s 与图像采样时间相同 ROS 时间域的当前时间。
   * @return 观测年龄不超过最大诊断时域时返回从当前时刻起的完整轨迹；未初始化、
   *         时间回退或状态非法时返回 std::nullopt。
   */
  std::optional<DeckMotionPrediction> predict(double now_s) const;

  /**
   * @brief 清空状态、协方差、Marker 和时间历史。
   */
  void reset();

private:
  void initialize(
    const Pose3d & pose_ned,
    const Eigen::Vector3d & uav_velocity_ned_mps,
    std::int32_t marker_id,
    double sample_time_s);
  void predict_in_place(double dt_s);
  void update_translation_kinematic_fit(
    const Eigen::Vector3d & relative_position_ned_m,
    const Eigen::Vector3d & uav_displacement_ned_m,
    double sample_time_s);
  DeckMotionEstimate make_estimate() const;

  DeckMotionEstimatorParameters parameters_;
  Eigen::Matrix<double, 9, 1> translation_state_{
    Eigen::Matrix<double, 9, 1>::Zero()};
  Eigen::Quaterniond orientation_deck_to_ned_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d angular_velocity_ned_radps_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration_ned_radps2_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 9, 9> translation_covariance_{
    Eigen::Matrix<double, 9, 9>::Zero()};
  Eigen::Matrix<double, 9, 9> rotation_covariance_{
    Eigen::Matrix<double, 9, 9>::Zero()};
  double state_time_s_{0.0};
  Eigen::Vector3d uav_velocity_ned_mps_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d uav_displacement_ned_m_{Eigen::Vector3d::Zero()};
  struct TranslationSample
  {
    double time_s{0.0};
    Eigen::Vector3d deck_position_ned_m{Eigen::Vector3d::Zero()};
  };
  std::deque<TranslationSample> translation_samples_;
  Eigen::Vector3d fitted_deck_velocity_ned_mps_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d fitted_deck_acceleration_ned_mps2_{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 6, 6> fitted_kinematic_covariance_{
    Eigen::Matrix<double, 6, 6>::Zero()};
  bool translation_fit_valid_{false};
  std::int32_t marker_id_{-1};
  bool last_update_accepted_{false};
  bool initialized_{false};
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__DECK_MOTION_ESTIMATOR_HPP_
