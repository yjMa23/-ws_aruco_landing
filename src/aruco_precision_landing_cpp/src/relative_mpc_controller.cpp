// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/relative_mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <OsqpEigen/Constants.hpp>
#include <osqp/osqp.h>

namespace aruco_precision_landing_cpp
{

namespace
{

bool is_positive_finite(double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool is_non_negative_finite(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

bool vector_is_non_negative_finite(const Eigen::VectorXd & value)
{
  return value.allFinite() && (value.array() >= 0.0).all();
}

bool vector_is_positive_finite(const Eigen::VectorXd & value)
{
  return value.allFinite() && (value.array() > 0.0).all();
}

struct CscStorage
{
  OSQPCscMatrix matrix{};
  std::vector<OSQPFloat> values;
  std::vector<OSQPInt> row_indices;
  std::vector<OSQPInt> column_pointers;

  void assign(const Eigen::SparseMatrix<double> & source)
  {
    Eigen::SparseMatrix<double> compressed = source;
    compressed.makeCompressed();

    values.resize(static_cast<std::size_t>(compressed.nonZeros()));
    row_indices.resize(static_cast<std::size_t>(compressed.nonZeros()));
    column_pointers.resize(static_cast<std::size_t>(compressed.cols() + 1));

    for (Eigen::Index index = 0; index < compressed.nonZeros(); ++index) {
      values[static_cast<std::size_t>(index)] =
        static_cast<OSQPFloat>(compressed.valuePtr()[index]);
      row_indices[static_cast<std::size_t>(index)] =
        static_cast<OSQPInt>(compressed.innerIndexPtr()[index]);
    }
    for (Eigen::Index index = 0; index <= compressed.cols(); ++index) {
      column_pointers[static_cast<std::size_t>(index)] =
        static_cast<OSQPInt>(compressed.outerIndexPtr()[index]);
    }

    OSQPCscMatrix_set_data(
      &matrix,
      static_cast<OSQPInt>(compressed.rows()),
      static_cast<OSQPInt>(compressed.cols()),
      static_cast<OSQPInt>(compressed.nonZeros()),
      values.data(),
      row_indices.data(),
      column_pointers.data());
  }
};

RelativeMpcStatus status_from_osqp(OSQPInt status)
{
  switch (status) {
    case OSQP_SOLVED:
      return RelativeMpcStatus::kSolved;
    case OSQP_SOLVED_INACCURATE:
      return RelativeMpcStatus::kSolvedInaccurate;
    case OSQP_PRIMAL_INFEASIBLE:
    case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
      return RelativeMpcStatus::kPrimalInfeasible;
    case OSQP_DUAL_INFEASIBLE:
    case OSQP_DUAL_INFEASIBLE_INACCURATE:
      return RelativeMpcStatus::kDualInfeasible;
    case OSQP_MAX_ITER_REACHED:
      return RelativeMpcStatus::kMaximumIterations;
    case OSQP_TIME_LIMIT_REACHED:
      return RelativeMpcStatus::kTimeLimit;
    case OSQP_NON_CVX:
      return RelativeMpcStatus::kNonConvex;
    default:
      return RelativeMpcStatus::kSolverError;
  }
}

}  // namespace

const char * relative_mpc_status_name(RelativeMpcStatus status)
{
  switch (status) {
    case RelativeMpcStatus::kNotInitialized:
      return "NOT_INITIALIZED";
    case RelativeMpcStatus::kInvalidInput:
      return "INVALID_INPUT";
    case RelativeMpcStatus::kSolved:
      return "SOLVED";
    case RelativeMpcStatus::kSolvedInaccurate:
      return "SOLVED_INACCURATE";
    case RelativeMpcStatus::kPrimalInfeasible:
      return "PRIMAL_INFEASIBLE";
    case RelativeMpcStatus::kDualInfeasible:
      return "DUAL_INFEASIBLE";
    case RelativeMpcStatus::kMaximumIterations:
      return "MAXIMUM_ITERATIONS";
    case RelativeMpcStatus::kTimeLimit:
      return "TIME_LIMIT";
    case RelativeMpcStatus::kNonConvex:
      return "NON_CONVEX";
    case RelativeMpcStatus::kSolverError:
      return "SOLVER_ERROR";
    case RelativeMpcStatus::kInvalidSolution:
      return "INVALID_SOLUTION";
    case RelativeMpcStatus::kTerminalPhaseDisengaged:
      return "TERMINAL_RULE_BASED_TRACKING";
  }
  return "UNKNOWN";
}

class RelativeMpcController::Impl
{
public:
  explicit Impl(RelativeMpcParameters parameters)
  : parameters_(std::move(parameters)),
    model_(RelativeMpcController::discrete_model(parameters_.sample_period_s))
  {
    validate_parameters();
    calculate_dimensions();
    build_hessian();
    build_constraint_matrix();
    gradient_.assign(variable_count_, 0.0);
    lower_bounds_.assign(constraint_count_, 0.0);
    upper_bounds_.assign(constraint_count_, 0.0);
  }

  ~Impl()
  {
    cleanup();
  }

  void validate_parameters() const
  {
    if (!is_positive_finite(parameters_.sample_period_s)) {
      throw std::invalid_argument("relative_mpc.sample_period_s must be finite and positive");
    }
    if (parameters_.horizon_steps < 2 || parameters_.horizon_steps > 200) {
      throw std::invalid_argument("relative_mpc.horizon_steps must be in [2, 200]");
    }
    if (!vector_is_non_negative_finite(parameters_.state_weights) ||
      !vector_is_non_negative_finite(parameters_.terminal_state_weights))
    {
      throw std::invalid_argument("relative_mpc state weights must be finite and non-negative");
    }
    if (!vector_is_positive_finite(parameters_.control_weights) ||
      !vector_is_non_negative_finite(parameters_.control_increment_weights))
    {
      throw std::invalid_argument(
              "relative_mpc control weights must be positive and increment weights non-negative");
    }
    if (!is_positive_finite(parameters_.speed_slack_weight) ||
      !is_positive_finite(parameters_.maximum_uav_speed_mps) ||
      !is_positive_finite(parameters_.maximum_acceleration_mps2) ||
      !is_positive_finite(parameters_.maximum_acceleration_increment_mps2) ||
      !is_non_negative_finite(parameters_.maximum_speed_slack_mps) ||
      parameters_.maximum_iterations <= 0 ||
      !is_positive_finite(parameters_.absolute_tolerance) ||
      !is_positive_finite(parameters_.relative_tolerance) ||
      !is_positive_finite(parameters_.time_limit_s) ||
      !is_positive_finite(parameters_.active_constraint_tolerance))
    {
      throw std::invalid_argument("relative_mpc constraints and solver settings are invalid");
    }
  }

  void calculate_dimensions()
  {
    const std::size_t horizon = static_cast<std::size_t>(parameters_.horizon_steps);
    state_offset_ = 0U;
    input_offset_ = 4U * (horizon + 1U);
    slack_offset_ = input_offset_ + 2U * horizon;
    variable_count_ = slack_offset_ + 2U * (horizon + 1U);

    initial_row_ = 0U;
    dynamics_row_ = initial_row_ + 4U;
    acceleration_row_ = dynamics_row_ + 4U * horizon;
    increment_row_ = acceleration_row_ + 2U * horizon;
    speed_row_ = increment_row_ + 2U * horizon;
    slack_row_ = speed_row_ + 4U * (horizon + 1U);
    constraint_count_ = slack_row_ + 2U * (horizon + 1U);
  }

  std::size_t state_index(std::size_t step, std::size_t component) const
  {
    return state_offset_ + 4U * step + component;
  }

  std::size_t input_index(std::size_t step, std::size_t component) const
  {
    return input_offset_ + 2U * step + component;
  }

  std::size_t slack_index(std::size_t step, std::size_t component) const
  {
    return slack_offset_ + 2U * step + component;
  }

  void build_hessian()
  {
    const std::size_t horizon = static_cast<std::size_t>(parameters_.horizon_steps);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(variable_count_ * 2U);

    for (std::size_t step = 0; step <= horizon; ++step) {
      const Eigen::Vector4d & weights =
        step == horizon ? parameters_.terminal_state_weights : parameters_.state_weights;
      for (std::size_t component = 0; component < 4U; ++component) {
        triplets.emplace_back(
          static_cast<Eigen::Index>(state_index(step, component)),
          static_cast<Eigen::Index>(state_index(step, component)),
          2.0 * weights[static_cast<Eigen::Index>(component)]);
      }
      for (std::size_t component = 0; component < 2U; ++component) {
        triplets.emplace_back(
          static_cast<Eigen::Index>(slack_index(step, component)),
          static_cast<Eigen::Index>(slack_index(step, component)),
          2.0 * parameters_.speed_slack_weight);
      }
    }

    for (std::size_t step = 0; step < horizon; ++step) {
      for (std::size_t component = 0; component < 2U; ++component) {
        const double increment_weight =
          parameters_.control_increment_weights[static_cast<Eigen::Index>(component)];
        const double increment_contributions =
          step + 1U < horizon ? 2.0 : 1.0;
        const double diagonal = 2.0 * (
          parameters_.control_weights[static_cast<Eigen::Index>(component)] +
          increment_contributions * increment_weight);
        triplets.emplace_back(
          static_cast<Eigen::Index>(input_index(step, component)),
          static_cast<Eigen::Index>(input_index(step, component)),
          diagonal);
        if (step > 0U && increment_weight > 0.0) {
          const Eigen::Index previous =
            static_cast<Eigen::Index>(input_index(step - 1U, component));
          const Eigen::Index current =
            static_cast<Eigen::Index>(input_index(step, component));
          triplets.emplace_back(previous, current, -2.0 * increment_weight);
          triplets.emplace_back(current, previous, -2.0 * increment_weight);
        }
      }
    }

    hessian_.resize(
      static_cast<Eigen::Index>(variable_count_),
      static_cast<Eigen::Index>(variable_count_));
    hessian_.setFromTriplets(triplets.begin(), triplets.end());
    hessian_.makeCompressed();

    std::vector<Eigen::Triplet<double>> upper_triplets;
    upper_triplets.reserve(static_cast<std::size_t>(hessian_.nonZeros()));
    for (int column = 0; column < hessian_.outerSize(); ++column) {
      for (Eigen::SparseMatrix<double>::InnerIterator entry(hessian_, column); entry; ++entry) {
        if (entry.row() <= entry.col()) {
          upper_triplets.emplace_back(entry.row(), entry.col(), entry.value());
        }
      }
    }
    hessian_upper_.resize(hessian_.rows(), hessian_.cols());
    hessian_upper_.setFromTriplets(upper_triplets.begin(), upper_triplets.end());
    hessian_upper_.makeCompressed();
  }

  void build_constraint_matrix()
  {
    const std::size_t horizon = static_cast<std::size_t>(parameters_.horizon_steps);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(constraint_count_ * 3U);

    for (std::size_t component = 0; component < 4U; ++component) {
      triplets.emplace_back(
        static_cast<Eigen::Index>(initial_row_ + component),
        static_cast<Eigen::Index>(state_index(0U, component)),
        1.0);
    }

    for (std::size_t step = 0; step < horizon; ++step) {
      for (std::size_t row = 0; row < 4U; ++row) {
        const std::size_t constraint_row = dynamics_row_ + 4U * step + row;
        triplets.emplace_back(
          static_cast<Eigen::Index>(constraint_row),
          static_cast<Eigen::Index>(state_index(step + 1U, row)),
          1.0);
        for (std::size_t column = 0; column < 4U; ++column) {
          const double value = -model_.a(
            static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
          if (value != 0.0) {
            triplets.emplace_back(
              static_cast<Eigen::Index>(constraint_row),
              static_cast<Eigen::Index>(state_index(step, column)),
              value);
          }
        }
        for (std::size_t column = 0; column < 2U; ++column) {
          const double value = -model_.b(
            static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
          if (value != 0.0) {
            triplets.emplace_back(
              static_cast<Eigen::Index>(constraint_row),
              static_cast<Eigen::Index>(input_index(step, column)),
              value);
          }
        }
      }
    }

    for (std::size_t step = 0; step < horizon; ++step) {
      for (std::size_t component = 0; component < 2U; ++component) {
        triplets.emplace_back(
          static_cast<Eigen::Index>(acceleration_row_ + 2U * step + component),
          static_cast<Eigen::Index>(input_index(step, component)),
          1.0);
        const std::size_t row = increment_row_ + 2U * step + component;
        triplets.emplace_back(
          static_cast<Eigen::Index>(row),
          static_cast<Eigen::Index>(input_index(step, component)),
          1.0);
        if (step > 0U) {
          triplets.emplace_back(
            static_cast<Eigen::Index>(row),
            static_cast<Eigen::Index>(input_index(step - 1U, component)),
            -1.0);
        }
      }
    }

    for (std::size_t step = 0; step <= horizon; ++step) {
      for (std::size_t component = 0; component < 2U; ++component) {
        const std::size_t first_row = speed_row_ + 4U * step + 2U * component;
        const std::size_t velocity_component = 2U + component;
        triplets.emplace_back(
          static_cast<Eigen::Index>(first_row),
          static_cast<Eigen::Index>(state_index(step, velocity_component)),
          -1.0);
        triplets.emplace_back(
          static_cast<Eigen::Index>(first_row),
          static_cast<Eigen::Index>(slack_index(step, component)),
          -1.0);
        triplets.emplace_back(
          static_cast<Eigen::Index>(first_row + 1U),
          static_cast<Eigen::Index>(state_index(step, velocity_component)),
          1.0);
        triplets.emplace_back(
          static_cast<Eigen::Index>(first_row + 1U),
          static_cast<Eigen::Index>(slack_index(step, component)),
          -1.0);
        triplets.emplace_back(
          static_cast<Eigen::Index>(slack_row_ + 2U * step + component),
          static_cast<Eigen::Index>(slack_index(step, component)),
          1.0);
      }
    }

    constraint_matrix_.resize(
      static_cast<Eigen::Index>(constraint_count_),
      static_cast<Eigen::Index>(variable_count_));
    constraint_matrix_.setFromTriplets(triplets.begin(), triplets.end());
    constraint_matrix_.makeCompressed();
  }

  void cleanup()
  {
    if (solver_ != nullptr) {
      osqp_cleanup(solver_);
      solver_ = nullptr;
    }
    initialized_ = false;
    have_warm_start_ = false;
    last_primal_.clear();
    last_dual_.clear();
  }

  bool initialize()
  {
    cleanup();
    if (std::string(osqp_version()) != "1.0.0") {
      return false;
    }
    update_problem_vectors(RelativeMpcInput{});

    p_storage_.assign(hessian_upper_);
    a_storage_.assign(constraint_matrix_);

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.allocate_solution = 1;
    settings.warm_starting = parameters_.warm_start_enabled ? 1 : 0;
    settings.max_iter = static_cast<OSQPInt>(parameters_.maximum_iterations);
    settings.eps_abs = static_cast<OSQPFloat>(parameters_.absolute_tolerance);
    settings.eps_rel = static_cast<OSQPFloat>(parameters_.relative_tolerance);
    settings.time_limit = static_cast<OSQPFloat>(parameters_.time_limit_s);
    settings.polishing = 0;

    const OSQPInt exit_flag = osqp_setup(
      &solver_,
      &p_storage_.matrix,
      gradient_.data(),
      &a_storage_.matrix,
      lower_bounds_.data(),
      upper_bounds_.data(),
      static_cast<OSQPInt>(constraint_count_),
      static_cast<OSQPInt>(variable_count_),
      &settings);
    initialized_ = exit_flag == 0 && solver_ != nullptr;
    return initialized_;
  }

  bool input_is_valid(const RelativeMpcInput & input) const
  {
    return input.relative_state.allFinite() &&
           input.deck_velocity_xy.allFinite() &&
           input.previous_control_xy.allFinite() &&
           is_positive_finite(input.dt_s) &&
           (!input.deck_acceleration_xy.has_value() ||
           input.deck_acceleration_xy->allFinite());
  }

  void update_problem_vectors(const RelativeMpcInput & input)
  {
    const std::size_t horizon = static_cast<std::size_t>(parameters_.horizon_steps);
    std::fill(gradient_.begin(), gradient_.end(), static_cast<OSQPFloat>(0.0));
    std::fill(
      lower_bounds_.begin(), lower_bounds_.end(),
      static_cast<OSQPFloat>(-OsqpEigen::INFTY));
    std::fill(
      upper_bounds_.begin(), upper_bounds_.end(),
      static_cast<OSQPFloat>(OsqpEigen::INFTY));

    for (std::size_t component = 0; component < 2U; ++component) {
      gradient_[input_index(0U, component)] = static_cast<OSQPFloat>(
        -2.0 * parameters_.control_increment_weights[static_cast<Eigen::Index>(component)] *
        input.previous_control_xy[static_cast<Eigen::Index>(component)]);
    }

    for (std::size_t component = 0; component < 4U; ++component) {
      const OSQPFloat value =
        static_cast<OSQPFloat>(input.relative_state[static_cast<Eigen::Index>(component)]);
      lower_bounds_[initial_row_ + component] = value;
      upper_bounds_[initial_row_ + component] = value;
    }

    const Eigen::Vector2d deck_acceleration =
      input.deck_acceleration_xy.value_or(Eigen::Vector2d::Zero());
    const Eigen::Vector4d disturbance = model_.e * deck_acceleration;
    for (std::size_t step = 0; step < horizon; ++step) {
      for (std::size_t component = 0; component < 4U; ++component) {
        const OSQPFloat value =
          static_cast<OSQPFloat>(disturbance[static_cast<Eigen::Index>(component)]);
        lower_bounds_[dynamics_row_ + 4U * step + component] = value;
        upper_bounds_[dynamics_row_ + 4U * step + component] = value;
      }
    }

    for (std::size_t step = 0; step < horizon; ++step) {
      for (std::size_t component = 0; component < 2U; ++component) {
        lower_bounds_[acceleration_row_ + 2U * step + component] =
          static_cast<OSQPFloat>(-parameters_.maximum_acceleration_mps2);
        upper_bounds_[acceleration_row_ + 2U * step + component] =
          static_cast<OSQPFloat>(parameters_.maximum_acceleration_mps2);

        const double center = step == 0U ?
          input.previous_control_xy[static_cast<Eigen::Index>(component)] : 0.0;
        lower_bounds_[increment_row_ + 2U * step + component] =
          static_cast<OSQPFloat>(
          step == 0U ? center - parameters_.maximum_acceleration_increment_mps2 :
          -parameters_.maximum_acceleration_increment_mps2);
        upper_bounds_[increment_row_ + 2U * step + component] =
          static_cast<OSQPFloat>(
          step == 0U ? center + parameters_.maximum_acceleration_increment_mps2 :
          parameters_.maximum_acceleration_increment_mps2);
      }
    }

    for (std::size_t step = 0; step <= horizon; ++step) {
      const double prediction_time = parameters_.sample_period_s * static_cast<double>(step);
      const Eigen::Vector2d predicted_deck_velocity =
        input.deck_velocity_xy + prediction_time * deck_acceleration;
      for (std::size_t component = 0; component < 2U; ++component) {
        const std::size_t row = speed_row_ + 4U * step + 2U * component;
        const double deck_velocity =
          predicted_deck_velocity[static_cast<Eigen::Index>(component)];
        upper_bounds_[row] = static_cast<OSQPFloat>(
          parameters_.maximum_uav_speed_mps - deck_velocity);
        upper_bounds_[row + 1U] = static_cast<OSQPFloat>(
          parameters_.maximum_uav_speed_mps + deck_velocity);

        lower_bounds_[slack_row_ + 2U * step + component] = 0.0;
        upper_bounds_[slack_row_ + 2U * step + component] =
          static_cast<OSQPFloat>(parameters_.maximum_speed_slack_mps);
      }
    }
  }

  RelativeMpcResult failure_result(
    RelativeMpcStatus status,
    const RelativeMpcInput & input,
    const std::string & reason) const
  {
    RelativeMpcResult result;
    result.status = status;
    result.solver_status = relative_mpc_status_name(status);
    result.fallback_reason = reason;
    result.current_relative_state = input.relative_state.allFinite() ?
      input.relative_state : Eigen::Vector4d::Zero();
    if (input.deck_acceleration_xy.has_value() && input.deck_acceleration_xy->allFinite()) {
      result.deck_acceleration_valid = true;
      result.deck_acceleration_used_xy = *input.deck_acceleration_xy;
    }
    return result;
  }

  RelativeMpcResult solve(const RelativeMpcInput & input)
  {
    if (!initialized_ || solver_ == nullptr) {
      return failure_result(
        RelativeMpcStatus::kNotInitialized, input, "solver_not_initialized");
    }
    if (!input_is_valid(input)) {
      return failure_result(RelativeMpcStatus::kInvalidInput, input, "invalid_input");
    }

    update_problem_vectors(input);
    RelativeMpcResult result;
    result.current_relative_state = input.relative_state;
    result.deck_acceleration_valid = input.deck_acceleration_xy.has_value();
    result.deck_acceleration_used_xy =
      input.deck_acceleration_xy.value_or(Eigen::Vector2d::Zero());

    if (osqp_update_data_vec(
        solver_, gradient_.data(), lower_bounds_.data(), upper_bounds_.data()) != 0)
    {
      return failure_result(RelativeMpcStatus::kSolverError, input, "solver_update_failed");
    }

    if (parameters_.warm_start_enabled && have_warm_start_) {
      if (osqp_warm_start(solver_, last_primal_.data(), last_dual_.data()) == 0) {
        result.warm_start_applied = true;
      } else {
        osqp_cold_start(solver_);
      }
    } else {
      osqp_cold_start(solver_);
    }

    result.solver_called = true;
    const auto start = std::chrono::steady_clock::now();
    const OSQPInt exit_flag = osqp_solve(solver_);
    const auto end = std::chrono::steady_clock::now();
    const double wall_time_ms =
      std::chrono::duration<double, std::milli>(end - start).count();

    if (exit_flag != 0 || solver_->info == nullptr) {
      result.status = RelativeMpcStatus::kSolverError;
      result.solver_status = "osqp_exit_" + std::to_string(exit_flag);
      result.fallback_reason = "solver_call_failed";
      result.solve_time_ms = wall_time_ms;
      return result;
    }

    result.status = status_from_osqp(solver_->info->status_val);
    result.solver_status = solver_->info->status;
    result.iteration_count = static_cast<int>(solver_->info->iter);
    result.objective = static_cast<double>(solver_->info->obj_val);
    result.solve_time_ms =
      std::isfinite(solver_->info->solve_time) && solver_->info->solve_time >= 0.0 ?
      1000.0 * static_cast<double>(solver_->info->solve_time) : wall_time_ms;

    if (result.status != RelativeMpcStatus::kSolved) {
      result.fallback_reason = relative_mpc_status_name(result.status);
      return result;
    }
    if (solver_->solution == nullptr || solver_->solution->x == nullptr ||
      solver_->solution->y == nullptr)
    {
      result.status = RelativeMpcStatus::kInvalidSolution;
      result.solver_status = relative_mpc_status_name(result.status);
      result.fallback_reason = "missing_solver_solution";
      return result;
    }

    Eigen::VectorXd solution(static_cast<Eigen::Index>(variable_count_));
    for (std::size_t index = 0; index < variable_count_; ++index) {
      solution[static_cast<Eigen::Index>(index)] =
        static_cast<double>(solver_->solution->x[index]);
    }
    if (!solution.allFinite() || !std::isfinite(result.objective) ||
      !std::isfinite(result.solve_time_ms))
    {
      result.status = RelativeMpcStatus::kInvalidSolution;
      result.solver_status = relative_mpc_status_name(result.status);
      result.fallback_reason = "non_finite_solver_output";
      return result;
    }

    const std::size_t horizon = static_cast<std::size_t>(parameters_.horizon_steps);
    result.predicted_states.reserve(horizon + 1U);
    result.predicted_controls.reserve(horizon);
    for (std::size_t step = 0; step <= horizon; ++step) {
      result.predicted_states.push_back(
        solution.segment<4>(static_cast<Eigen::Index>(state_index(step, 0U))));
    }
    for (std::size_t step = 0; step < horizon; ++step) {
      result.predicted_controls.push_back(
        solution.segment<2>(static_cast<Eigen::Index>(input_index(step, 0U))));
    }

    Eigen::Vector2d first_control = result.predicted_controls.front();
    for (Eigen::Index component = 0; component < 2; ++component) {
      const double lower = std::max(
        -parameters_.maximum_acceleration_mps2,
        input.previous_control_xy[component] -
        parameters_.maximum_acceleration_increment_mps2);
      const double upper = std::min(
        parameters_.maximum_acceleration_mps2,
        input.previous_control_xy[component] +
        parameters_.maximum_acceleration_increment_mps2);
      first_control[component] = std::clamp(first_control[component], lower, upper);
    }
    if (!first_control.allFinite()) {
      result.status = RelativeMpcStatus::kInvalidSolution;
      result.solver_status = relative_mpc_status_name(result.status);
      result.fallback_reason = "non_finite_first_control";
      return result;
    }
    result.first_control_xy = first_control;

    const Eigen::VectorXd constraint_values = constraint_matrix_ * solution;
    for (std::size_t row = acceleration_row_; row < constraint_count_; ++row) {
      const double value = constraint_values[static_cast<Eigen::Index>(row)];
      const double lower = static_cast<double>(lower_bounds_[row]);
      const double upper = static_cast<double>(upper_bounds_[row]);
      const bool lower_active = std::isfinite(lower) &&
        std::abs(value - lower) <= parameters_.active_constraint_tolerance;
      const bool upper_active = std::isfinite(upper) &&
        std::abs(value - upper) <= parameters_.active_constraint_tolerance;
      if (lower_active || upper_active) {
        ++result.active_constraints;
      }
    }

    last_primal_.resize(variable_count_);
    last_dual_.resize(constraint_count_);
    for (std::size_t index = 0; index < variable_count_; ++index) {
      last_primal_[index] = solver_->solution->x[index];
    }
    for (std::size_t index = 0; index < constraint_count_; ++index) {
      last_dual_[index] = solver_->solution->y[index];
    }
    have_warm_start_ = parameters_.warm_start_enabled;

    result.success = true;
    result.fallback_required = false;
    result.fallback_reason.clear();
    return result;
  }

  void reset()
  {
    have_warm_start_ = false;
    last_primal_.clear();
    last_dual_.clear();
    if (solver_ != nullptr) {
      osqp_cold_start(solver_);
    }
  }

  RelativeMpcParameters parameters_;
  RelativeMpcDiscreteModel model_;
  Eigen::SparseMatrix<double> hessian_;
  Eigen::SparseMatrix<double> hessian_upper_;
  Eigen::SparseMatrix<double> constraint_matrix_;
  CscStorage p_storage_;
  CscStorage a_storage_;
  std::vector<OSQPFloat> gradient_;
  std::vector<OSQPFloat> lower_bounds_;
  std::vector<OSQPFloat> upper_bounds_;
  std::vector<OSQPFloat> last_primal_;
  std::vector<OSQPFloat> last_dual_;
  OSQPSolver * solver_{nullptr};
  bool initialized_{false};
  bool have_warm_start_{false};

  std::size_t state_offset_{0U};
  std::size_t input_offset_{0U};
  std::size_t slack_offset_{0U};
  std::size_t variable_count_{0U};
  std::size_t initial_row_{0U};
  std::size_t dynamics_row_{0U};
  std::size_t acceleration_row_{0U};
  std::size_t increment_row_{0U};
  std::size_t speed_row_{0U};
  std::size_t slack_row_{0U};
  std::size_t constraint_count_{0U};
};

RelativeMpcController::RelativeMpcController(const RelativeMpcParameters & parameters)
: impl_(std::make_unique<Impl>(parameters))
{
}

RelativeMpcController::~RelativeMpcController() = default;
RelativeMpcController::RelativeMpcController(RelativeMpcController &&) noexcept = default;
RelativeMpcController & RelativeMpcController::operator=(RelativeMpcController &&) noexcept = default;

bool RelativeMpcController::initialize()
{
  return impl_->initialize();
}

RelativeMpcResult RelativeMpcController::solve(const RelativeMpcInput & input)
{
  return impl_->solve(input);
}

void RelativeMpcController::reset()
{
  impl_->reset();
}

bool RelativeMpcController::initialized() const
{
  return impl_->initialized_;
}

RelativeMpcContinuousModel RelativeMpcController::continuous_model()
{
  RelativeMpcContinuousModel model;
  model.a.block<2, 2>(0, 2).setIdentity();
  model.b.block<2, 2>(2, 0) = -Eigen::Matrix2d::Identity();
  model.e.block<2, 2>(2, 0).setIdentity();
  return model;
}

bool RelativeMpcController::acceleration_feedforward_allowed(
  bool solver_success, bool terminal_phase) noexcept
{
  return solver_success && !terminal_phase;
}

RelativeMpcDiscreteModel RelativeMpcController::discrete_model(double sample_period_s)
{
  if (!is_positive_finite(sample_period_s)) {
    throw std::invalid_argument("MPC sample period must be finite and positive");
  }

  RelativeMpcDiscreteModel model;
  const Eigen::Matrix2d identity = Eigen::Matrix2d::Identity();
  model.a.block<2, 2>(0, 2) = sample_period_s * identity;
  model.b.block<2, 2>(0, 0) =
    -0.5 * sample_period_s * sample_period_s * identity;
  model.b.block<2, 2>(2, 0) = -sample_period_s * identity;
  model.e.block<2, 2>(0, 0) =
    0.5 * sample_period_s * sample_period_s * identity;
  model.e.block<2, 2>(2, 0) = sample_period_s * identity;
  return model;
}

const Eigen::SparseMatrix<double> & RelativeMpcController::hessian() const
{
  return impl_->hessian_;
}

const Eigen::SparseMatrix<double> & RelativeMpcController::constraint_matrix() const
{
  return impl_->constraint_matrix_;
}

std::size_t RelativeMpcController::variable_count() const
{
  return impl_->variable_count_;
}

std::size_t RelativeMpcController::constraint_count() const
{
  return impl_->constraint_count_;
}

const RelativeMpcParameters & RelativeMpcController::parameters() const
{
  return impl_->parameters_;
}

}  // namespace aruco_precision_landing_cpp
