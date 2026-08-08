// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#ifndef ARUCO_PRECISION_LANDING_CPP__RELATIVE_MPC_CONTROLLER_HPP_
#define ARUCO_PRECISION_LANDING_CPP__RELATIVE_MPC_CONTROLLER_HPP_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

namespace aruco_precision_landing_cpp
{

/**
 * @brief relative MPC 水平相对运动 MPC 的连续时间系统矩阵。
 */
struct RelativeMpcContinuousModel
{
  Eigen::Matrix4d a{Eigen::Matrix4d::Zero()};
  Eigen::Matrix<double, 4, 2> b{Eigen::Matrix<double, 4, 2>::Zero()};
  Eigen::Matrix<double, 4, 2> e{Eigen::Matrix<double, 4, 2>::Zero()};
};

/**
 * @brief relative MPC 水平相对运动 MPC 的离散时间系统矩阵。
 */
struct RelativeMpcDiscreteModel
{
  Eigen::Matrix4d a{Eigen::Matrix4d::Identity()};
  Eigen::Matrix<double, 4, 2> b{Eigen::Matrix<double, 4, 2>::Zero()};
  Eigen::Matrix<double, 4, 2> e{Eigen::Matrix<double, 4, 2>::Zero()};
};

/**
 * @brief MPC 求解状态。
 */
enum class RelativeMpcStatus
{
  kNotInitialized,
  kInvalidInput,
  kSolved,
  kSolvedInaccurate,
  kPrimalInfeasible,
  kDualInfeasible,
  kMaximumIterations,
  kTimeLimit,
  kNonConvex,
  kSolverError,
  kInvalidSolution,
  kTerminalPhaseDisengaged
};

/**
 * @brief 返回稳定的 MPC 状态名称。
 *
 * @param status 求解状态。
 * @return 用于日志和诊断话题的常量字符串。
 */
const char * relative_mpc_status_name(RelativeMpcStatus status);

/**
 * @brief relative MPC 水平相对运动线性 MPC 参数。
 */
struct RelativeMpcParameters
{
  double sample_period_s{0.05};
  int horizon_steps{20};
  Eigen::Vector4d state_weights{Eigen::Vector4d{8.0, 8.0, 2.0, 2.0}};
  Eigen::Vector4d terminal_state_weights{Eigen::Vector4d{16.0, 16.0, 4.0, 4.0}};
  Eigen::Vector2d control_weights{Eigen::Vector2d{0.20, 0.20}};
  Eigen::Vector2d control_increment_weights{Eigen::Vector2d{1.0, 1.0}};
  double speed_slack_weight{1000.0};
  double maximum_uav_speed_mps{2.0};
  double maximum_acceleration_mps2{1.5};
  double maximum_acceleration_increment_mps2{0.25};
  double maximum_speed_slack_mps{2.0};
  int maximum_iterations{1000};
  double absolute_tolerance{1.0e-4};
  double relative_tolerance{1.0e-4};
  double time_limit_s{0.02};
  bool warm_start_enabled{true};
  double active_constraint_tolerance{1.0e-3};
};

/**
 * @brief 单周期 MPC 输入，全部使用 PX4 local NED 水平分量。
 */
struct RelativeMpcInput
{
  Eigen::Vector4d relative_state{Eigen::Vector4d::Zero()};
  Eigen::Vector2d deck_velocity_xy{Eigen::Vector2d::Zero()};
  std::optional<Eigen::Vector2d> deck_acceleration_xy;
  Eigen::Vector2d previous_control_xy{Eigen::Vector2d::Zero()};
  double dt_s{0.05};
};

/**
 * @brief 单周期 MPC 求解结果和论文评测诊断。
 */
struct RelativeMpcResult
{
  RelativeMpcStatus status{RelativeMpcStatus::kNotInitialized};
  std::string solver_status{"not_initialized"};
  std::string fallback_reason{"solver_not_initialized"};
  bool success{false};
  bool fallback_required{true};
  bool solver_called{false};
  bool warm_start_applied{false};
  bool deck_acceleration_valid{false};
  double solve_time_ms{0.0};
  int iteration_count{0};
  double objective{0.0};
  std::size_t active_constraints{0};
  Eigen::Vector4d current_relative_state{Eigen::Vector4d::Zero()};
  Eigen::Vector2d first_control_xy{Eigen::Vector2d::Zero()};
  Eigen::Vector2d deck_acceleration_used_xy{Eigen::Vector2d::Zero()};
  std::vector<Eigen::Vector4d> predicted_states;
  std::vector<Eigen::Vector2d> predicted_controls;
};

/**
 * @brief 使用 OSQP 求解二维水平相对双积分有限时域凸 QP。
 *
 * 状态定义为 `[e_x, e_y, v_rel_x, v_rel_y]`，其中
 * `e = p_deck - p_uav`、`v_rel = v_deck - v_uav`；控制输入为 UAV NED
 * 水平加速度。该类不依赖 ROS，不修改垂直设定点、状态机或触地语义。
 */
class RelativeMpcController
{
public:
  /**
   * @brief 构造控制器并校验所有模型、权重、约束和求解器参数。
   *
   * @param parameters 固定采样周期、预测时域、代价、约束和 OSQP 设置。
   * @throws std::invalid_argument 任一参数非法。
   */
  explicit RelativeMpcController(const RelativeMpcParameters & parameters);
  ~RelativeMpcController();

  RelativeMpcController(const RelativeMpcController &) = delete;
  RelativeMpcController & operator=(const RelativeMpcController &) = delete;
  RelativeMpcController(RelativeMpcController &&) noexcept;
  RelativeMpcController & operator=(RelativeMpcController &&) noexcept;

  /**
   * @brief 构造固定稀疏 QP 并初始化 OSQP workspace。
   *
   * @return 初始化成功返回 true；失败时控制器保持不可求解状态。
   */
  bool initialize();

  /**
   * @brief 使用当前相对状态和可测甲板运动求解一个控制周期。
   *
   * @param input 相对状态、甲板速度、可选甲板加速度、上一控制和实际周期。
   * @return 始终返回有限诊断；输入非法、求解失败或解无效时要求回退 adaptive rule-based tracking。
   */
  RelativeMpcResult solve(const RelativeMpcInput & input);

  /**
   * @brief 清除 warm-start 历史，但保留已初始化的固定 QP workspace。
   */
  void reset();

  /**
   * @brief 返回求解器是否已初始化。
   */
  bool initialized() const;

  /**
   * @brief 返回连续相对运动模型。
   */
  static RelativeMpcContinuousModel continuous_model();

  /**
   * @brief 使用零阶保持返回离散相对运动模型。
   *
   * @param sample_period_s 正且有限的采样周期，单位秒。
   * @throws std::invalid_argument 采样周期非法。
   */
  static RelativeMpcDiscreteModel discrete_model(double sample_period_s);

  /**
   * @brief 判定本周期是否允许向 PX4 发布 MPC 水平加速度前馈。
   *
   * FINAL_DESCENT 及后续接触候选/确认保持阶段必须停用 MPC 加速度，改用并行
   * adaptive rule-based tracking 指令，避免横向加速度经姿态耦合破坏已验收的 final descent/heave touchdown 终端接触行为。
   * 求解失败时同样禁止发布加速度。
   *
   * @param solver_success 本周期求解结果是否为严格 solved 且输出有限。
   * @param terminal_phase 是否处于 FINAL_DESCENT 或后续接触阶段。
   */
  static bool acceleration_feedforward_allowed(
    bool solver_success, bool terminal_phase) noexcept;

  /**
   * @brief 返回固定 QP Hessian，供结构测试和离线诊断使用。
   */
  const Eigen::SparseMatrix<double> & hessian() const;

  /**
   * @brief 返回固定 QP 约束矩阵，供结构测试和离线诊断使用。
   */
  const Eigen::SparseMatrix<double> & constraint_matrix() const;

  /**
   * @brief 返回 QP 决策变量数量。
   */
  std::size_t variable_count() const;

  /**
   * @brief 返回 QP 约束数量。
   */
  std::size_t constraint_count() const;

  /**
   * @brief 返回当前固定参数。
   */
  const RelativeMpcParameters & parameters() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace aruco_precision_landing_cpp

#endif  // ARUCO_PRECISION_LANDING_CPP__RELATIVE_MPC_CONTROLLER_HPP_
