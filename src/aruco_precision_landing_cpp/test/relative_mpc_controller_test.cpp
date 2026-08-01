// Copyright 2026 user
// SPDX-License-Identifier: Apache-2.0

#include "aruco_precision_landing_cpp/relative_mpc_controller.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace aruco_precision_landing_cpp
{
namespace
{

RelativeMpcParameters test_parameters()
{
  RelativeMpcParameters parameters;
  parameters.sample_period_s = 0.05;
  parameters.horizon_steps = 8;
  parameters.state_weights = Eigen::Vector4d{10.0, 10.0, 3.0, 3.0};
  parameters.terminal_state_weights = Eigen::Vector4d{20.0, 20.0, 6.0, 6.0};
  parameters.control_weights = Eigen::Vector2d{0.1, 0.1};
  parameters.control_increment_weights = Eigen::Vector2d{0.5, 0.5};
  parameters.speed_slack_weight = 1000.0;
  parameters.maximum_uav_speed_mps = 2.0;
  parameters.maximum_acceleration_mps2 = 1.5;
  parameters.maximum_acceleration_increment_mps2 = 0.4;
  parameters.maximum_speed_slack_mps = 2.0;
  parameters.maximum_iterations = 2000;
  parameters.absolute_tolerance = 1.0e-5;
  parameters.relative_tolerance = 1.0e-5;
  parameters.time_limit_s = 0.05;
  parameters.warm_start_enabled = true;
  parameters.active_constraint_tolerance = 2.0e-3;
  return parameters;
}

RelativeMpcInput zero_input()
{
  RelativeMpcInput input;
  input.relative_state.setZero();
  input.deck_velocity_xy.setZero();
  input.previous_control_xy.setZero();
  input.dt_s = 0.05;
  return input;
}

RelativeMpcResult initialize_and_solve(
  RelativeMpcController & controller,
  const RelativeMpcInput & input)
{
  EXPECT_TRUE(controller.initialize());
  EXPECT_TRUE(controller.initialized());
  return controller.solve(input);
}

TEST(RelativeMpcControllerTest, AccelerationFeedforwardIsDisabledForFailureAndContact)
{
  EXPECT_TRUE(RelativeMpcController::acceleration_feedforward_allowed(true, false));
  EXPECT_FALSE(RelativeMpcController::acceleration_feedforward_allowed(false, false));
  EXPECT_FALSE(RelativeMpcController::acceleration_feedforward_allowed(true, true));
  EXPECT_FALSE(RelativeMpcController::acceleration_feedforward_allowed(false, true));
  EXPECT_STREQ(
    relative_mpc_status_name(RelativeMpcStatus::kTerminalPhaseDisengaged),
    "TERMINAL_PHASE_P47");
}

TEST(RelativeMpcControllerTest, ContinuousModelHasRequiredSignsAndDimensions)
{
  const auto model = RelativeMpcController::continuous_model();
  Eigen::Matrix4d expected_a = Eigen::Matrix4d::Zero();
  expected_a.block<2, 2>(0, 2).setIdentity();
  Eigen::Matrix<double, 4, 2> expected_b = Eigen::Matrix<double, 4, 2>::Zero();
  expected_b.block<2, 2>(2, 0) = -Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 4, 2> expected_e = Eigen::Matrix<double, 4, 2>::Zero();
  expected_e.block<2, 2>(2, 0).setIdentity();

  EXPECT_EQ(model.a.rows(), 4);
  EXPECT_EQ(model.a.cols(), 4);
  EXPECT_EQ(model.b.rows(), 4);
  EXPECT_EQ(model.b.cols(), 2);
  EXPECT_EQ(model.e.rows(), 4);
  EXPECT_EQ(model.e.cols(), 2);
  EXPECT_TRUE(model.a.isApprox(expected_a));
  EXPECT_TRUE(model.b.isApprox(expected_b));
  EXPECT_TRUE(model.e.isApprox(expected_e));
}

TEST(RelativeMpcControllerTest, DiscreteModelMatchesExactZeroOrderHold)
{
  constexpr double sample_period_s = 0.2;
  const auto model = RelativeMpcController::discrete_model(sample_period_s);
  Eigen::Matrix4d expected_a = Eigen::Matrix4d::Identity();
  expected_a.block<2, 2>(0, 2) = sample_period_s * Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 4, 2> expected_b = Eigen::Matrix<double, 4, 2>::Zero();
  expected_b.block<2, 2>(0, 0) = -0.5 * sample_period_s * sample_period_s *
    Eigen::Matrix2d::Identity();
  expected_b.block<2, 2>(2, 0) = -sample_period_s * Eigen::Matrix2d::Identity();
  Eigen::Matrix<double, 4, 2> expected_e = Eigen::Matrix<double, 4, 2>::Zero();
  expected_e.block<2, 2>(0, 0) = 0.5 * sample_period_s * sample_period_s *
    Eigen::Matrix2d::Identity();
  expected_e.block<2, 2>(2, 0) = sample_period_s * Eigen::Matrix2d::Identity();

  EXPECT_TRUE(model.a.isApprox(expected_a));
  EXPECT_TRUE(model.b.isApprox(expected_b));
  EXPECT_TRUE(model.e.isApprox(expected_e));
  EXPECT_THROW(RelativeMpcController::discrete_model(0.0), std::invalid_argument);
  EXPECT_THROW(
    RelativeMpcController::discrete_model(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(RelativeMpcControllerTest, QpDimensionsAndMatricesAreConsistent)
{
  const auto parameters = test_parameters();
  RelativeMpcController controller(parameters);
  const std::size_t horizon = static_cast<std::size_t>(parameters.horizon_steps);

  EXPECT_EQ(controller.variable_count(), 8U * horizon + 6U);
  EXPECT_EQ(controller.constraint_count(), 14U * horizon + 10U);
  EXPECT_EQ(controller.hessian().rows(), static_cast<Eigen::Index>(controller.variable_count()));
  EXPECT_EQ(controller.hessian().cols(), static_cast<Eigen::Index>(controller.variable_count()));
  EXPECT_EQ(
    controller.constraint_matrix().rows(),
    static_cast<Eigen::Index>(controller.constraint_count()));
  EXPECT_EQ(
    controller.constraint_matrix().cols(),
    static_cast<Eigen::Index>(controller.variable_count()));
  EXPECT_TRUE(
    Eigen::MatrixXd(controller.hessian()).isApprox(
      Eigen::MatrixXd(controller.hessian().transpose()), 1.0e-12));
  EXPECT_GT(controller.hessian().nonZeros(), 0);
  EXPECT_GT(controller.constraint_matrix().nonZeros(), 0);
}

TEST(RelativeMpcControllerTest, ZeroErrorProducesFiniteNearZeroControl)
{
  RelativeMpcController controller(test_parameters());
  const auto result = initialize_and_solve(controller, zero_input());

  ASSERT_TRUE(result.success) << result.solver_status;
  EXPECT_FALSE(result.fallback_required);
  EXPECT_TRUE(result.solver_called);
  EXPECT_EQ(result.status, RelativeMpcStatus::kSolved);
  EXPECT_TRUE(result.first_control_xy.allFinite());
  EXPECT_NEAR(result.first_control_xy.norm(), 0.0, 1.0e-5);
  EXPECT_EQ(result.predicted_states.size(), 9U);
  EXPECT_EQ(result.predicted_controls.size(), 8U);
  EXPECT_GE(result.iteration_count, 1);
  EXPECT_GE(result.solve_time_ms, 0.0);
  EXPECT_TRUE(std::isfinite(result.objective));
}

TEST(RelativeMpcControllerTest, PositiveAndNegativePositionErrorsCommandCorrectDirection)
{
  RelativeMpcController positive_controller(test_parameters());
  auto positive_input = zero_input();
  positive_input.relative_state.x() = 1.0;
  const auto positive = initialize_and_solve(positive_controller, positive_input);
  ASSERT_TRUE(positive.success) << positive.solver_status;
  EXPECT_GT(positive.first_control_xy.x(), 0.0);
  EXPECT_NEAR(positive.first_control_xy.y(), 0.0, 1.0e-5);

  RelativeMpcController negative_controller(test_parameters());
  auto negative_input = zero_input();
  negative_input.relative_state.x() = -1.0;
  const auto negative = initialize_and_solve(negative_controller, negative_input);
  ASSERT_TRUE(negative.success) << negative.solver_status;
  EXPECT_LT(negative.first_control_xy.x(), 0.0);
  EXPECT_NEAR(negative.first_control_xy.y(), 0.0, 1.0e-5);
}

TEST(RelativeMpcControllerTest, RelativeVelocityIsDampedInBothDirections)
{
  RelativeMpcController positive_controller(test_parameters());
  auto positive_input = zero_input();
  positive_input.relative_state.z() = 0.8;
  const auto positive = initialize_and_solve(positive_controller, positive_input);
  ASSERT_TRUE(positive.success) << positive.solver_status;
  EXPECT_GT(positive.first_control_xy.x(), 0.0);

  RelativeMpcController negative_controller(test_parameters());
  auto negative_input = zero_input();
  negative_input.relative_state.z() = -0.8;
  const auto negative = initialize_and_solve(negative_controller, negative_input);
  ASSERT_TRUE(negative.success) << negative.solver_status;
  EXPECT_LT(negative.first_control_xy.x(), 0.0);
}

TEST(RelativeMpcControllerTest, DeckAccelerationDisturbanceProducesMatchingControlDirection)
{
  RelativeMpcController controller(test_parameters());
  auto input = zero_input();
  input.deck_acceleration_xy = Eigen::Vector2d{0.5, -0.3};
  const auto result = initialize_and_solve(controller, input);

  ASSERT_TRUE(result.success) << result.solver_status;
  EXPECT_TRUE(result.deck_acceleration_valid);
  EXPECT_TRUE(result.deck_acceleration_used_xy.isApprox(Eigen::Vector2d{0.5, -0.3}));
  EXPECT_GT(result.first_control_xy.x(), 0.0);
  EXPECT_LT(result.first_control_xy.y(), 0.0);
}

TEST(RelativeMpcControllerTest, AccelerationConstraintIsNeverExceeded)
{
  auto parameters = test_parameters();
  parameters.maximum_acceleration_mps2 = 0.3;
  parameters.maximum_acceleration_increment_mps2 = 1.0;
  RelativeMpcController controller(parameters);
  auto input = zero_input();
  input.relative_state.x() = 100.0;
  const auto result = initialize_and_solve(controller, input);

  ASSERT_TRUE(result.success) << result.solver_status;
  EXPECT_LE(std::abs(result.first_control_xy.x()), 0.3 + 1.0e-6);
  for (const auto & control : result.predicted_controls) {
    EXPECT_LE(control.cwiseAbs().maxCoeff(), 0.3 + 2.0e-3);
  }
  EXPECT_GT(result.active_constraints, 0U);
}

TEST(RelativeMpcControllerTest, ControlIncrementConstraintLimitsFirstAndFutureControls)
{
  auto parameters = test_parameters();
  parameters.maximum_acceleration_mps2 = 2.0;
  parameters.maximum_acceleration_increment_mps2 = 0.1;
  RelativeMpcController controller(parameters);
  auto input = zero_input();
  input.relative_state.x() = 20.0;
  input.previous_control_xy = Eigen::Vector2d{0.4, -0.2};
  const auto result = initialize_and_solve(controller, input);

  ASSERT_TRUE(result.success) << result.solver_status;
  EXPECT_LE(
    (result.first_control_xy - input.previous_control_xy).cwiseAbs().maxCoeff(),
    0.1 + 1.0e-6);
  Eigen::Vector2d previous = input.previous_control_xy;
  for (const auto & control : result.predicted_controls) {
    EXPECT_LE((control - previous).cwiseAbs().maxCoeff(), 0.1 + 2.0e-3);
    previous = control;
  }
}

TEST(RelativeMpcControllerTest, HardSpeedConstraintLimitsPredictedUavVelocity)
{
  auto parameters = test_parameters();
  parameters.maximum_uav_speed_mps = 0.35;
  parameters.maximum_speed_slack_mps = 0.0;
  parameters.maximum_acceleration_mps2 = 2.0;
  parameters.maximum_acceleration_increment_mps2 = 2.0;
  RelativeMpcController controller(parameters);
  auto input = zero_input();
  input.relative_state.x() = 5.0;
  const auto result = initialize_and_solve(controller, input);

  ASSERT_TRUE(result.success) << result.solver_status;
  for (const auto & state : result.predicted_states) {
    const Eigen::Vector2d uav_velocity = input.deck_velocity_xy - state.tail<2>();
    EXPECT_LE(uav_velocity.cwiseAbs().maxCoeff(), 0.35 + 3.0e-3);
  }
}

TEST(RelativeMpcControllerTest, PredictedTrajectorySatisfiesDiscreteDynamics)
{
  RelativeMpcController controller(test_parameters());
  auto input = zero_input();
  input.relative_state = Eigen::Vector4d{1.0, -0.5, 0.2, -0.1};
  input.deck_acceleration_xy = Eigen::Vector2d{0.1, 0.05};
  const auto result = initialize_and_solve(controller, input);
  ASSERT_TRUE(result.success) << result.solver_status;
  const auto model = RelativeMpcController::discrete_model(0.05);

  for (std::size_t step = 0; step < result.predicted_controls.size(); ++step) {
    const Eigen::Vector4d expected =
      model.a * result.predicted_states[step] +
      model.b * result.predicted_controls[step] +
      model.e * *input.deck_acceleration_xy;
    EXPECT_TRUE(result.predicted_states[step + 1U].isApprox(expected, 3.0e-4));
  }
}

TEST(RelativeMpcControllerTest, SecondSolveUsesWarmStart)
{
  RelativeMpcController controller(test_parameters());
  ASSERT_TRUE(controller.initialize());
  auto input = zero_input();
  input.relative_state.x() = 1.0;
  const auto first = controller.solve(input);
  input.relative_state.x() = 0.9;
  const auto second = controller.solve(input);

  ASSERT_TRUE(first.success) << first.solver_status;
  ASSERT_TRUE(second.success) << second.solver_status;
  EXPECT_FALSE(first.warm_start_applied);
  EXPECT_TRUE(second.warm_start_applied);
}

TEST(RelativeMpcControllerTest, ResetClearsWarmStartHistory)
{
  RelativeMpcController controller(test_parameters());
  ASSERT_TRUE(controller.initialize());
  auto input = zero_input();
  input.relative_state.x() = 1.0;
  ASSERT_TRUE(controller.solve(input).success);
  controller.reset();
  const auto after_reset = controller.solve(input);

  ASSERT_TRUE(after_reset.success) << after_reset.solver_status;
  EXPECT_FALSE(after_reset.warm_start_applied);
}

TEST(RelativeMpcControllerTest, InfeasibleSpeedConstraintRequestsFallback)
{
  auto parameters = test_parameters();
  parameters.maximum_uav_speed_mps = 0.1;
  parameters.maximum_speed_slack_mps = 0.0;
  RelativeMpcController controller(parameters);
  auto input = zero_input();
  input.relative_state.z() = -1.0;
  const auto result = initialize_and_solve(controller, input);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.fallback_required);
  EXPECT_TRUE(result.solver_called);
  EXPECT_EQ(result.status, RelativeMpcStatus::kPrimalInfeasible);
  EXPECT_EQ(result.predicted_states.size(), 0U);
  EXPECT_TRUE(result.first_control_xy.isZero());
}

TEST(RelativeMpcControllerTest, MaximumIterationStatusRequestsFallback)
{
  auto parameters = test_parameters();
  parameters.maximum_iterations = 1;
  parameters.absolute_tolerance = 1.0e-12;
  parameters.relative_tolerance = 1.0e-12;
  RelativeMpcController controller(parameters);
  auto input = zero_input();
  input.relative_state = Eigen::Vector4d{20.0, -10.0, 2.0, -1.0};
  const auto result = initialize_and_solve(controller, input);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.fallback_required);
  EXPECT_TRUE(result.solver_called);
  EXPECT_TRUE(
    result.status == RelativeMpcStatus::kMaximumIterations ||
    result.status == RelativeMpcStatus::kTimeLimit ||
    result.status == RelativeMpcStatus::kSolvedInaccurate);
}

TEST(RelativeMpcControllerTest, NaNAndInfInputsNeverCallSolver)
{
  RelativeMpcController controller(test_parameters());
  ASSERT_TRUE(controller.initialize());

  auto nan_input = zero_input();
  nan_input.relative_state.x() = std::numeric_limits<double>::quiet_NaN();
  const auto nan_result = controller.solve(nan_input);
  EXPECT_EQ(nan_result.status, RelativeMpcStatus::kInvalidInput);
  EXPECT_FALSE(nan_result.solver_called);
  EXPECT_TRUE(nan_result.fallback_required);
  EXPECT_TRUE(nan_result.first_control_xy.allFinite());

  auto inf_input = zero_input();
  inf_input.deck_velocity_xy.y() = std::numeric_limits<double>::infinity();
  const auto inf_result = controller.solve(inf_input);
  EXPECT_EQ(inf_result.status, RelativeMpcStatus::kInvalidInput);
  EXPECT_FALSE(inf_result.solver_called);
  EXPECT_TRUE(inf_result.fallback_required);
  EXPECT_TRUE(inf_result.first_control_xy.allFinite());
}

TEST(RelativeMpcControllerTest, SolverMustBeInitializedExplicitly)
{
  RelativeMpcController controller(test_parameters());
  const auto result = controller.solve(zero_input());

  EXPECT_FALSE(controller.initialized());
  EXPECT_EQ(result.status, RelativeMpcStatus::kNotInitialized);
  EXPECT_FALSE(result.solver_called);
  EXPECT_TRUE(result.fallback_required);
}

TEST(RelativeMpcControllerTest, RecoversAfterInvalidInputAndInfeasibleSolve)
{
  RelativeMpcController controller(test_parameters());
  ASSERT_TRUE(controller.initialize());

  auto invalid = zero_input();
  invalid.dt_s = 0.0;
  EXPECT_EQ(controller.solve(invalid).status, RelativeMpcStatus::kInvalidInput);

  auto valid = zero_input();
  valid.relative_state.x() = 0.5;
  const auto recovered = controller.solve(valid);
  EXPECT_TRUE(recovered.success) << recovered.solver_status;
  EXPECT_FALSE(recovered.fallback_required);
}

TEST(RelativeMpcControllerTest, InvalidParametersAreRejected)
{
  auto parameters = test_parameters();
  parameters.sample_period_s = 0.0;
  EXPECT_THROW(RelativeMpcController controller(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.horizon_steps = 1;
  EXPECT_THROW(RelativeMpcController controller(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.control_weights.x() = -1.0;
  EXPECT_THROW(RelativeMpcController controller(parameters), std::invalid_argument);

  parameters = test_parameters();
  parameters.maximum_acceleration_mps2 =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(RelativeMpcController controller(parameters), std::invalid_argument);
}

TEST(RelativeMpcControllerTest, OutputIsContinuousAcrossSuccessiveCommands)
{
  auto parameters = test_parameters();
  parameters.maximum_acceleration_increment_mps2 = 0.08;
  RelativeMpcController controller(parameters);
  ASSERT_TRUE(controller.initialize());

  auto input = zero_input();
  input.relative_state.x() = 10.0;
  const auto first = controller.solve(input);
  ASSERT_TRUE(first.success) << first.solver_status;

  input.previous_control_xy = first.first_control_xy;
  input.relative_state.x() = -10.0;
  const auto switched = controller.solve(input);
  ASSERT_TRUE(switched.success) << switched.solver_status;
  EXPECT_LE(
    (switched.first_control_xy - first.first_control_xy).cwiseAbs().maxCoeff(),
    0.08 + 1.0e-6);
}

TEST(RelativeMpcControllerTest, ControllerOnlyProducesHorizontalControl)
{
  RelativeMpcController controller(test_parameters());
  auto input = zero_input();
  input.relative_state = Eigen::Vector4d{1.0, 2.0, 0.1, 0.2};
  const auto result = initialize_and_solve(controller, input);

  ASSERT_TRUE(result.success) << result.solver_status;
  EXPECT_EQ(result.first_control_xy.size(), 2);
  EXPECT_EQ(result.current_relative_state.size(), 4);
  EXPECT_TRUE(result.first_control_xy.allFinite());
}

}  // namespace
}  // namespace aruco_precision_landing_cpp
