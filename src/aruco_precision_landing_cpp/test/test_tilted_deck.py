#!/usr/bin/env python3
"""不启动 PX4/Gazebo 的 deck-geometry shadow evaluator、参数和 shadow 隔离测试。"""

from __future__ import annotations

import math
import subprocess
import sys
import unittest
from pathlib import Path

import yaml

WORKSPACE_DIR = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = WORKSPACE_DIR / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from evaluate_tilted_deck import (  # noqa: E402
    BODY_CLEARANCE_TOPIC,
    MarkerHistorySample,
    NORMAL_TOPIC,
    REQUIRED_SHADOW_TOPICS,
    TimedSample,
    calibration_axis_from_truth,
    calibration_gate,
    compare_evaluation_results,
    clearance_spread,
    contact_point_index,
    expected_attitude_deg,
    expected_upward_normal_ned,
    finite_values,
    ground_truth_upward_normal_ned,
    interpolate_sample,
    marker_normal_statistics,
    marker_history_replay_statistics,
    nonfinite_times_at_or_after,
    nearest_sample,
    normal_angle_error_deg,
    normal_to_attitude_deg,
    fixed_tilt_safe_descent_clearance_threshold_definition,
    fixed_tilt_safe_descent_gate,
    tilted_deck_touchdown_gate,
    terminal_stabilization_counterfactual_bias_from_normal,
    terminal_stabilization_replay_failure_explanation,
    terminal_stabilization_gate,
    terminal_stabilization_tracking_state_authorized,
    scalar_summary,
    select_marker_history_height_topic,
    timed_samples_between,
    topic_has_messages,
    validate_required_topics,
)


class TiltedDeckTests(unittest.TestCase):
    FIXED_TILT_SCENARIOS = {
        "tilt_roll_pos_2deg": ([2.0, 0.0, 0.0], 1, (1, 3)),
        "tilt_roll_neg_2deg": ([-2.0, 0.0, 0.0], 0, (0, 2)),
        "tilt_pitch_pos_2deg": ([0.0, 2.0, 0.0], 0, (0, 1)),
        "tilt_pitch_neg_2deg": ([0.0, -2.0, 0.0], 2, (2, 3)),
    }

    def test_fixed_tilt_scenario_configs_are_unique_static_profiles(self) -> None:
        config_dir = WORKSPACE_DIR / "src" / "moving_deck_sim" / "config"
        loaded_configs = {}
        for scenario, (expected_rpy, _, _) in self.FIXED_TILT_SCENARIOS.items():
            config_path = config_dir / f"{scenario}.yaml"
            self.assertTrue(config_path.is_file(), f"missing fixed-tilt config: {config_path}")
            parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
                "moving_deck_controller"
            ]["ros__parameters"]
            loaded_configs[scenario] = parameters
            self.assertEqual(parameters["scenario"], "S0_STATIC")
            self.assertEqual(parameters["initial_position_enu"], [0.0, 0.0, 2.0])
            self.assertEqual(parameters["velocity_xy"], [0.0, 0.0])
            self.assertEqual(parameters["amplitude_xy"], [0.0, 0.0])
            self.assertEqual(parameters["amplitude_z_m"], 0.0)
            self.assertEqual(parameters["initial_rpy_deg"], expected_rpy)
            self.assertEqual(parameters["amplitude_rpy_deg"], [0.0, 0.0, 0.0])
            self.assertEqual(parameters["update_rate_hz"], 50.0)
            self.assertEqual(parameters["random_seed"], 1)

        serialized = {
            scenario: tuple(parameters["initial_rpy_deg"])
            for scenario, parameters in loaded_configs.items()
        }
        self.assertEqual(len(set(serialized.values())), len(serialized))

    def test_fixed_tilt_scenario_theoretical_contact_order_is_frozen(self) -> None:
        half_length_m = 0.125
        half_width_m = 0.132
        contact_points = (
            (-half_length_m, -half_width_m, 0.227),
            (half_length_m, -half_width_m, 0.227),
            (-half_length_m, half_width_m, 0.227),
            (half_length_m, half_width_m, 0.227),
        )
        for _, (rpy_deg, expected_first, tied_first_side) in self.FIXED_TILT_SCENARIOS.items():
            roll = math.radians(rpy_deg[0])
            pitch = math.radians(rpy_deg[1])
            # Gazebo profile RPY is world ENU. Rotate body +Z in ENU, then
            # apply ENU -> local NED: (x, y, z) -> (y, x, -z).
            normal = (
                -math.sin(roll) * math.cos(pitch),
                math.sin(pitch),
                -math.cos(roll) * math.cos(pitch),
            )
            clearances = tuple(
                sum(component * point_component for component, point_component in zip(normal, point))
                for point in contact_points
            )
            self.assertEqual(contact_point_index(clearances), expected_first)
            self.assertAlmostEqual(clearances[tied_first_side[0]], clearances[tied_first_side[1]])
            expected_spread = (
                0.250 * math.sin(math.radians(2.0))
                if rpy_deg[0]
                else 0.264 * math.sin(math.radians(2.0))
            )
            self.assertAlmostEqual(clearance_spread(clearances), expected_spread)

    def test_start_sitl_maps_all_fixed_tilt_scenarios(self) -> None:
        start_script = (WORKSPACE_DIR / "scripts" / "start_sitl.sh").read_text(
            encoding="utf-8"
        )
        for scenario in self.FIXED_TILT_SCENARIOS:
            self.assertIn(f"{scenario})", start_script)
            self.assertIn(f'{scenario}.yaml', start_script)

    def _run_start_gate(self, *arguments: str) -> subprocess.CompletedProcess[str]:
        script = WORKSPACE_DIR / "scripts" / "start_sitl.sh"
        return subprocess.run(
            [str(script), *arguments, "--dry-run"],
            cwd=WORKSPACE_DIR,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )

    def test_positive_fixed_tilt_preserves_fixed_tilt_safe_altitude_and_fixed_tilt_safe_descent_modes(self) -> None:
        for scenario in ("tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"):
            safe_altitude = self._run_start_gate("--scenario", scenario)
            self.assertEqual(safe_altitude.returncode, 0, safe_altitude.stdout)
            self.assertIn("fixed-tilt safe altitude", safe_altitude.stdout)

            safe_descent = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--descent-test-height",
                "0.50",
            )
            self.assertEqual(safe_descent.returncode, 0, safe_descent.stdout)
            self.assertIn("positive fixed-tilt safe descent", safe_descent.stdout)

    def test_positive_fixed_tilt_allows_tilted_deck_touchdown_only_with_frozen_flags(self) -> None:
        for scenario in ("tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"):
            completed = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--descent-test-height",
                "0.50",
                "--enable-final-descent",
            )
            self.assertEqual(completed.returncode, 0, completed.stdout)
            self.assertIn("positive fixed-tilt touchdown", completed.stdout)

    def test_positive_fixed_tilt_rejects_final_without_relative_descent(self) -> None:
        for scenario in ("tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"):
            completed = self._run_start_gate(
                "--scenario", scenario, "--enable-final-descent"
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("requires --enable-relative-descent", completed.stdout)

    def test_positive_fixed_tilt_rejects_non_050_touchdown_height(self) -> None:
        for scenario in ("tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"):
            completed = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--descent-test-height",
                "0.60",
                "--enable-final-descent",
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("exactly 0.50 m", completed.stdout)

    def test_negative_fixed_tilt_keeps_relative_and_final_descent_closed(self) -> None:
        for scenario in ("tilt_roll_neg_2deg", "tilt_pitch_neg_2deg"):
            relative = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--descent-test-height",
                "0.50",
            )
            self.assertNotEqual(relative.returncode, 0)
            self.assertIn("safe-altitude shadow only", relative.stdout)

            final = self._run_start_gate(
                "--scenario", scenario, "--enable-final-descent"
            )
            self.assertNotEqual(final.returncode, 0)
            self.assertIn("negative fixed-tilt final descent", final.stdout)

    def test_all_fixed_tilt_defaults_remain_safe_altitude_shadow(self) -> None:
        for scenario in self.FIXED_TILT_SCENARIOS:
            completed = self._run_start_gate("--scenario", scenario)
            self.assertEqual(completed.returncode, 0, completed.stdout)
            self.assertIn("fixed-tilt safe altitude", completed.stdout)

    def test_dynamic_tilt_profiles_keep_final_descent_closed(self) -> None:
        for scenario in ("rollpitch", "combined"):
            completed = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--enable-final-descent",
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("restricted to safe-altitude shadow validation", completed.stdout)

    def test_terminal_stabilization_shadow_gate_accepts_only_positive_fixed_tilt(self) -> None:
        for scenario in ("tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"):
            completed = self._run_start_gate(
                "--scenario", scenario, "--terminal-contact-stabilization-shadow"
            )
            self.assertEqual(completed.returncode, 0, completed.stdout)
            self.assertIn("terminal_contact_stabilization_enabled=true", completed.stdout)
            self.assertIn("terminal_contact_stabilization_shadow_only=true", completed.stdout)

        for scenario in (
            "static",
            "tilt_roll_neg_2deg",
            "tilt_pitch_neg_2deg",
            "rollpitch",
            "combined",
        ):
            completed = self._run_start_gate(
                "--scenario", scenario, "--terminal-contact-stabilization-shadow"
            )
            self.assertNotEqual(completed.returncode, 0, completed.stdout)

    def test_terminal_stabilization_rehearsal_gate_requires_safe_descent_without_final_descent(self) -> None:
        for scenario in ("tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"):
            accepted = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--descent-test-height",
                "0.50",
                "--terminal-contact-stabilization-rehearsal",
            )
            self.assertEqual(accepted.returncode, 0, accepted.stdout)
            self.assertIn("terminal_contact_stabilization_rehearsal_enabled=true", accepted.stdout)

            no_relative = self._run_start_gate(
                "--scenario", scenario, "--terminal-contact-stabilization-rehearsal"
            )
            self.assertNotEqual(no_relative.returncode, 0, no_relative.stdout)
            self.assertIn("relative descent", no_relative.stdout)

            with_final = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--enable-final-descent",
                "--terminal-contact-stabilization-rehearsal",
            )
            self.assertNotEqual(with_final.returncode, 0, with_final.stdout)
            self.assertIn("final descent disabled", with_final.stdout)

    def test_terminal_stabilization_active_gate_requires_full_positive_touchdown_whitelist(self) -> None:
        for scenario in ("tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"):
            accepted = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--descent-test-height",
                "0.50",
                "--enable-final-descent",
                "--enable-terminal-contact-stabilization",
            )
            self.assertEqual(accepted.returncode, 0, accepted.stdout)
            self.assertIn("terminal_contact_stabilization_shadow_only=false", accepted.stdout)

            no_final = self._run_start_gate(
                "--scenario",
                scenario,
                "--enable-relative-descent",
                "--enable-terminal-contact-stabilization",
            )
            self.assertNotEqual(no_final.returncode, 0, no_final.stdout)
            self.assertIn("final descent", no_final.stdout)

    def test_terminal_stabilization_defaults_and_ros_interface_are_frozen(self) -> None:
        config = yaml.safe_load(
            (
                WORKSPACE_DIR
                / "src"
                / "aruco_precision_landing_cpp"
                / "config"
                / "px4_aruco_landing.yaml"
            ).read_text(encoding="utf-8")
        )["px4_aruco_landing_node"]["ros__parameters"]
        self.assertFalse(config["terminal_contact_stabilization.enabled"])
        self.assertTrue(config["terminal_contact_stabilization.shadow_only"])
        self.assertEqual(
            config["terminal_contact_stabilization.rehearsal_max_duration_s"],
            1.0,
        )
        self.assertEqual(
            config["terminal_contact_stabilization.preload_relative_height_m"],
            0.20,
        )
        self.assertEqual(
            config["terminal_contact_stabilization.preload_acceleration_mps2"],
            1.0,
        )
        self.assertEqual(
            config["terminal_contact_stabilization.preload_acceleration_slew_mps3"],
            1.0,
        )
        self.assertEqual(
            config["touchdown_hold.max_reference_preload_rate_mps"],
            0.05,
        )
        self.assertEqual(
            config["touchdown_detector.terminal_contact_max_geometry_gap_m"],
            0.03,
        )
        self.assertEqual(
            config[
                "terminal_contact_stabilization.compliance.maximum_target_rate_mps"
            ],
            0.10,
        )
        self.assertEqual(
            config[
                "terminal_contact_stabilization.compliance.maximum_anchor_correction_rate_mps"
            ],
            0.05,
        )
        self.assertEqual(
            config["terminal_contact_stabilization.compliance.deck_velocity_deadband_mps"],
            0.035,
        )

        source = (
            WORKSPACE_DIR
            / "src"
            / "aruco_precision_landing_cpp"
            / "src"
            / "px4_aruco_landing_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("msg.position = true", source)
        self.assertIn("msg.attitude = false", source)
        self.assertNotIn("VehicleAttitudeSetpoint", source)
        touchdown_gate = source[
            source.index("input.terminal_descent_active") :
            source.index("input.relative_horizontal_speed_valid")
        ]
        self.assertIn("final_descent_contact_slowdown_height_m_", touchdown_gate)
        self.assertIn("terminal_contact_stabilization_enabled_", touchdown_gate)
        self.assertIn("!terminal_contact_stabilization_shadow_only_", touchdown_gate)
        self.assertIn("confirmed_touchdown_hold", source)
        self.assertIn("Touchdown hold visual loss: holding latched contact targets", source)
        self.assertIn("set_vertical_acceleration_feedforward", source)
        self.assertIn("vertical_acceleration_feedforward_down_mps2_", source)

    def test_terminal_stabilization_counterfactual_replay_uses_frozen_signs_and_limits(self) -> None:
        roll_bias = terminal_stabilization_counterfactual_bias_from_normal(
            (0.0, math.sin(math.radians(2.0)), -math.cos(math.radians(2.0)))
        )
        self.assertGreater(roll_bias[1], 0.0)
        self.assertAlmostEqual(
            roll_bias[1], 9.80665 * math.tan(math.radians(2.0)), places=6
        )
        pitch_bias = terminal_stabilization_counterfactual_bias_from_normal(
            (-math.sin(math.radians(2.0)), 0.0, -math.cos(math.radians(2.0)))
        )
        self.assertLess(pitch_bias[0], 0.0)
        limited = terminal_stabilization_counterfactual_bias_from_normal(
            (math.sin(math.radians(20.0)), 0.0, -math.cos(math.radians(20.0)))
        )
        self.assertLessEqual(math.hypot(*limited), 0.45)
        with self.assertRaises(ValueError):
            terminal_stabilization_counterfactual_bias_from_normal((0.0, 0.0, 1.0))

    def test_terminal_stabilization_replay_failure_explanation_preserves_historical_failure(self) -> None:
        self.assertEqual(
            terminal_stabilization_replay_failure_explanation(
                {
                    "post_touchdown_tangential_slip_m": 0.106767,
                    "post_touchdown_roll_max_abs_deg": 2.1,
                    "post_touchdown_pitch_max_abs_deg": 0.2,
                    "recovery_count": 0,
                    "detach_count": 0,
                }
            ),
            "post_contact_slip",
        )
        self.assertEqual(
            terminal_stabilization_replay_failure_explanation(
                {
                    "post_touchdown_tangential_slip_m": 0.67,
                    "post_touchdown_roll_max_abs_deg": 60.0,
                    "post_touchdown_pitch_max_abs_deg": 55.0,
                    "recovery_count": 1,
                    "detach_count": 1,
                }
            ),
            "attitude_divergence_or_detach",
        )

    def test_terminal_stabilization_active_tracking_uses_contact_stable_states_only(self) -> None:
        self.assertFalse(terminal_stabilization_tracking_state_authorized("active", "FINAL_DESCENT"))
        self.assertTrue(
            terminal_stabilization_tracking_state_authorized("active", "TOUCHDOWN_CANDIDATE_HOLD")
        )
        self.assertTrue(terminal_stabilization_tracking_state_authorized("active", "TOUCHDOWN_HOLD"))
        self.assertTrue(terminal_stabilization_tracking_state_authorized("rehearsal", "TEST_HEIGHT_HOLD"))
        self.assertFalse(terminal_stabilization_tracking_state_authorized("rehearsal", "FINAL_DESCENT"))

    def test_terminal_stabilization_gate_covers_frozen_feature_limits(self) -> None:
        valid = {
            "mode": "active",
            "activation_sample_count": 50,
            "command_tilt_max_deg": 2.4,
            "command_tilt_slew_max_degps": 4.4,
            "combined_acceleration_max_mps2": 1.49,
            "attitude_tracking_error_p95_deg": 1.4,
            "rehearsal_horizontal_drift_max_m": 0.10,
            "fallback_count_after_activation": 0,
            "divergence_protection_count": 0,
            "nan_inf_count": 0,
        }
        passed = terminal_stabilization_gate(valid)
        self.assertTrue(passed["passed"], passed)
        self.assertEqual(passed["thresholds"]["command_tilt_max_deg"], 2.5)
        self.assertEqual(passed["thresholds"]["command_tilt_slew_max_degps"], 4.5)

        failures = {
            "activation": {"activation_sample_count": 0},
            "command_tilt": {"command_tilt_max_deg": 2.51},
            "command_tilt_slew": {"command_tilt_slew_max_degps": 4.51},
            "combined_acceleration": {"combined_acceleration_max_mps2": 1.51},
            "attitude_tracking": {"attitude_tracking_error_p95_deg": 1.51},
            "rehearsal_drift": {
                "mode": "rehearsal",
                "rehearsal_horizontal_drift_max_m": 0.151,
            },
            "fallback": {"fallback_count_after_activation": 1},
            "divergence_protection": {"divergence_protection_count": 1},
            "nan_inf": {"nan_inf_count": 1},
        }
        for expected_check, override in failures.items():
            metrics = dict(valid)
            metrics.update(override)
            result = terminal_stabilization_gate(metrics)
            self.assertFalse(result["passed"], expected_check)
            self.assertIn(expected_check, result["failed_checks"])

    def test_marker_switch_jump_event_is_latched_until_publish(self) -> None:
        source = (
            WORKSPACE_DIR
            / "src"
            / "aruco_precision_landing_cpp"
            / "src"
            / "px4_aruco_landing_node.cpp"
        ).read_text(encoding="utf-8")
        update_start = source.index(
            "void Px4ArucoLandingNode::update_estimated_deck_attitude"
        )
        publish_start = source.index(
            "void Px4ArucoLandingNode::publish_deck_normal_calibration_debug"
        )
        update_body = source[update_start:publish_start]
        publish_body = source[publish_start:]
        self.assertNotIn(
            "deck_normal_rate_valid_ = false;\n  marker_switch_normal_jump_valid_ = false;",
            update_body,
        )
        self.assertIn(
            "marker_switch_normal_jump_pub_->publish(jump_msg);\n"
            "    marker_switch_normal_jump_valid_ = false;",
            publish_body,
        )

    def test_tilted_deck_touchdown_gate_covers_all_frozen_hard_failures(self) -> None:
        valid = {
            "state_sequence": [
                "WAIT_LANDING_WINDOW",
                "DESCEND",
                "FINAL_DESCENT",
                "TOUCHDOWN_CANDIDATE_HOLD",
                "TOUCHDOWN_HOLD",
            ],
            "candidate_duration_s": 0.55,
            "hold_duration_s": 10.2,
            "horizontal_error_rmse_m": 0.05,
            "horizontal_error_max_m": 0.10,
            "relative_horizontal_velocity_rmse_mps": 0.08,
            "tangential_velocity_rmse_mps": 0.08,
            "touchdown_h_min_m": -0.01,
            "touchdown_h_max_m": 0.02,
            "touchdown_clearance_spread_m": 0.02,
            "touchdown_normal_relative_velocity_mps": -0.08,
            "touchdown_tangential_velocity_mps": 0.04,
            "post_touchdown_tangential_slip_m": 0.05,
            "hold_tangential_velocity_p95_mps": 0.04,
            "post_touchdown_roll_max_abs_deg": 4.0,
            "post_touchdown_pitch_max_abs_deg": 5.0,
            "attitude_divergence_delta_deg": 0.8,
            "detach_count": 0,
            "secondary_contact_count": 0,
            "recovery_count": 0,
            "time_sync_failure_count": 0,
            "nan_inf_count": 0,
            "nav_land_command_count": 0,
            "disarm_command_count": 0,
        }
        passed = tilted_deck_touchdown_gate(valid)
        self.assertTrue(passed["passed"], passed)
        self.assertFalse(passed["target_touchdown_speed_achieved"])
        self.assertEqual(passed["thresholds"]["touchdown_normal_speed_target_mps"], 0.05)
        self.assertEqual(passed["thresholds"]["attitude_divergence_delta_max_deg"], 2.0)

        failures = {
            "missing_final_descent": {"state_sequence": ["WAIT_LANDING_WINDOW", "DESCEND", "TOUCHDOWN_CANDIDATE_HOLD", "TOUCHDOWN_HOLD"]},
            "missing_candidate": {"state_sequence": ["WAIT_LANDING_WINDOW", "DESCEND", "FINAL_DESCENT", "TOUCHDOWN_HOLD"]},
            "missing_touchdown_hold": {"state_sequence": ["WAIT_LANDING_WINDOW", "DESCEND", "FINAL_DESCENT", "TOUCHDOWN_CANDIDATE_HOLD"]},
            "hold_duration": {"hold_duration_s": 9.99},
            "candidate_duration": {"candidate_duration_s": 0.49},
            "horizontal_rmse": {"horizontal_error_rmse_m": 0.081},
            "horizontal_max": {"horizontal_error_max_m": 0.151},
            "relative_horizontal_velocity_rmse": {"relative_horizontal_velocity_rmse_mps": 0.101},
            "tangential_velocity_rmse": {"tangential_velocity_rmse_mps": 0.101},
            "touchdown_h_min": {"touchdown_h_min_m": -0.051},
            "touchdown_h_max": {"touchdown_h_max_m": 0.051},
            "touchdown_clearance_spread": {"touchdown_clearance_spread_m": 0.031},
            "touchdown_normal_speed": {"touchdown_normal_relative_velocity_mps": -0.121},
            "post_touchdown_slip": {"post_touchdown_tangential_slip_m": 0.101},
            "hold_tangential_velocity_p95": {"hold_tangential_velocity_p95_mps": 0.051},
            "post_touchdown_roll": {"post_touchdown_roll_max_abs_deg": 10.01},
            "post_touchdown_pitch": {"post_touchdown_pitch_max_abs_deg": 10.01},
            "attitude_divergence": {"attitude_divergence_delta_deg": 2.01},
            "detach": {"detach_count": 1},
            "secondary_contact": {"secondary_contact_count": 1},
            "recovery": {"recovery_count": 1},
            "time_sync": {"time_sync_failure_count": 1},
            "nan_inf": {"nan_inf_count": 1},
            "nav_land": {"nav_land_command_count": 1},
            "disarm": {"disarm_command_count": 1},
        }
        for expected_check, override in failures.items():
            metrics = dict(valid)
            metrics.update(override)
            result = tilted_deck_touchdown_gate(metrics)
            self.assertFalse(result["passed"], expected_check)
            self.assertIn(expected_check, result["failed_checks"])

        for key in (
            "horizontal_error_rmse_m",
            "touchdown_h_min_m",
            "touchdown_normal_relative_velocity_mps",
        ):
            metrics = dict(valid)
            metrics[key] = math.nan
            result = tilted_deck_touchdown_gate(metrics)
            self.assertFalse(result["passed"])
            self.assertIn("nan_inf", result["failed_checks"])

    def test_expected_scenario_normals_apply_gazebo_enu_to_local_ned_axes(self) -> None:
        expected_ned = {
            "static": (0.0, 0.0),
            "tilt_roll_pos_2deg": (0.0, 2.0),
            "tilt_roll_neg_2deg": (0.0, -2.0),
            "tilt_pitch_pos_2deg": (2.0, 0.0),
            "tilt_pitch_neg_2deg": (-2.0, 0.0),
        }
        for scenario, (roll_ned_deg, pitch_ned_deg) in expected_ned.items():
            self.assertEqual(
                expected_attitude_deg(scenario), (roll_ned_deg, pitch_ned_deg)
            )
            attitude = normal_to_attitude_deg(expected_upward_normal_ned(scenario))
            self.assertAlmostEqual(attitude[0], roll_ned_deg, places=9)
            self.assertAlmostEqual(attitude[1], pitch_ned_deg, places=9)
            self.assertAlmostEqual(
                attitude[2], math.hypot(roll_ned_deg, pitch_ned_deg), places=3
            )

    def test_calibration_axis_is_selected_from_ground_truth_not_scene_name(self) -> None:
        self.assertEqual(calibration_axis_from_truth((0.0, 2.0)), "pitch")
        self.assertEqual(calibration_axis_from_truth((-2.0, 0.0)), "roll")
        self.assertEqual(calibration_axis_from_truth((0.0, 0.0)), "tilt")

    def test_scalar_summary_includes_rmse_and_rejects_nonfinite_samples(self) -> None:
        summary = scalar_summary([-1.0, 1.0, math.nan])
        self.assertEqual(summary["count"], 2)
        self.assertAlmostEqual(summary["mean"], 0.0)
        self.assertAlmostEqual(summary["rmse"], 1.0)
        self.assertAlmostEqual(summary["max_abs"], 1.0)

    def test_marker_normal_statistics_groups_samples_and_truth_errors(self) -> None:
        normals = {
            0: [(0.0, 0.0, -1.0), (0.0, 0.0, -1.0)],
            2: [(0.0, math.sin(math.radians(2.0)), -math.cos(math.radians(2.0)))],
        }
        truth = (0.0, 0.0, -1.0)
        statistics = marker_normal_statistics(normals, truth)
        self.assertEqual(statistics["0"]["sample_count"], 2)
        self.assertEqual(statistics["0"]["mean_normal_ned"], [0.0, 0.0, -1.0])
        self.assertAlmostEqual(statistics["0"]["normal_angle_error_deg"]["rmse"], 0.0)
        self.assertEqual(statistics["2"]["sample_count"], 1)
        self.assertAlmostEqual(statistics["2"]["normal_angle_error_deg"]["mean"], 2.0)

    def test_marker_history_replay_requires_all_ids_and_quantifies_switches(self) -> None:
        up = (0.0, 0.0, -1.0)
        samples = [
            MarkerHistorySample(1.0, 0, up, up, 2.0),
            MarkerHistorySample(2.0, 0, up, up, 1.0),
            MarkerHistorySample(3.0, 1, up, up, 0.8),
            MarkerHistorySample(4.0, 2, up, up, 0.4),
            MarkerHistorySample(5.0, 3, up, up, 0.22),
        ]
        replay = marker_history_replay_statistics(samples, filter_gain=1.0)
        self.assertTrue(replay["passed"])
        self.assertEqual(replay["marker_ids_observed"], [0, 1, 2, 3])
        self.assertEqual(replay["marker_ids_missing"], [])
        self.assertEqual(replay["switch_count"], 3)
        self.assertEqual(replay["switch_sequence"], ["0->1", "1->2", "2->3"])
        self.assertAlmostEqual(replay["switch_jump_deg"]["max"], 0.0)
        self.assertEqual(replay["marker_statistics"]["3"]["sample_count"], 1)
        self.assertAlmostEqual(
            replay["marker_statistics"]["3"]["relative_height_m"]["mean"],
            0.22,
        )
        self.assertTrue(
            replay["synthetic_two_degree_sign_observation"]["all_signs_correct"]
        )
        self.assertEqual(
            replay["synthetic_two_degree_sign_observation"]["status"],
            "observation_only_not_real_tilted_imagery",
        )

        incomplete = marker_history_replay_statistics(samples[:3], filter_gain=1.0)
        self.assertFalse(incomplete["passed"])
        self.assertEqual(incomplete["marker_ids_missing"], [2, 3])
        self.assertIn("missing_marker_ids", incomplete["failed_checks"])

    def test_cross_bag_comparison_reports_zero_vs_two_degree_separation(self) -> None:
        def result(scenario, axis, axis_mean, tilt_mean, rmse, p95, sign_accuracy):
            return {
                "scenario": scenario,
                "seed": 1,
                "final_result": "PASS",
                "calibration_axis_from_ground_truth": axis,
                "calibration_gate": {
                    "rmse_deg": rmse,
                    "p95_deg": p95,
                    "sign_accuracy": sign_accuracy,
                },
                "estimated_attitude_from_normal_deg": {
                    "roll": {"mean": axis_mean if axis == "roll" else 0.1},
                    "pitch": {"mean": axis_mean if axis == "pitch" else -0.1},
                    "tilt": {"mean": tilt_mean},
                },
                "active_marker_sample_counts": {"0": 100, "1": 0, "2": 0, "3": 0},
                "marker_switch_count": 0,
                "nav_land_command_count": 0,
                "disarm_command_count": 0,
            }

        comparison = compare_evaluation_results(
            [
                result("static", "tilt", 0.5, 0.5, 0.6, 1.2, 1.0),
                result("tilt_roll_pos_2deg", "pitch", 1.9, 2.0, 0.5, 1.0, 1.0),
                result("tilt_roll_neg_2deg", "pitch", -1.8, 1.9, 0.6, 1.1, 0.99),
            ]
        )
        self.assertEqual(comparison["run_count"], 3)
        self.assertEqual(comparison["failed_run_count"], 0)
        self.assertAlmostEqual(
            comparison["scenarios"]["tilt_roll_pos_2deg"][
                "minimum_axis_mean_separation_from_static_deg"
            ],
            2.0,
        )
        self.assertEqual(comparison["marker_ids_observed"], [0])
        self.assertEqual(comparison["marker_ids_missing"], [1, 2, 3])
        self.assertEqual(comparison["total_nav_land_commands"], 0)
        self.assertEqual(comparison["total_disarm_commands"], 0)
        self.assertEqual(
            comparison["zero_vs_two_degree_threshold_status"],
            "observation_only_plan_has_no_frozen_cross_bag_separation_threshold",
        )

    def test_fixed_tilt_safe_descent_clearance_threshold_is_frozen_before_real_experiments(self) -> None:
        definition = fixed_tilt_safe_descent_clearance_threshold_definition()
        self.assertAlmostEqual(definition["target_body_height_m"], 0.50)
        self.assertAlmostEqual(definition["worst_theoretical_minimum_skid_clearance_m"], 0.268227, places=6)
        self.assertAlmostEqual(definition["fixed_tilt_safe_altitude_worst_absolute_skid_geometry_error_m"], 0.16652911274062454)
        self.assertEqual(definition["frozen_minimum_ground_truth_skid_clearance_m"], 0.09)
        self.assertGreater(definition["additional_unallocated_margin_m"], 0.01)

    def test_fixed_tilt_safe_descent_gate_covers_all_hard_failures(self) -> None:
        valid = {
            "state_sequence": [
                "WAIT_LANDING_WINDOW",
                "DESCEND",
                "TEST_HEIGHT_HOLD",
            ],
            "test_height_hold_duration_s": 12.0,
            "ground_truth_minimum_skid_clearance_m": 0.20,
            "ground_truth_contact_count": 0,
            "ground_truth_penetration_count": 0,
            "horizontal_error_rmse_m": 0.05,
            "horizontal_error_max_m": 0.10,
            "normal_rmse_deg": 0.8,
            "normal_p95_deg": 1.4,
            "sign_accuracy": 0.99,
            "marker_switch_jump_max_deg": 0.4,
            "time_sync_failure_count": 0,
            "nan_inf_count": 0,
            "nav_land_command_count": 0,
            "disarm_command_count": 0,
        }
        self.assertTrue(fixed_tilt_safe_descent_gate(valid)["passed"])

        failures = {
            "missing_descend": {"state_sequence": ["WAIT_LANDING_WINDOW", "TEST_HEIGHT_HOLD"]},
            "missing_test_height_hold": {"state_sequence": ["WAIT_LANDING_WINDOW", "DESCEND"]},
            "final_descent": {"state_sequence": valid["state_sequence"] + ["FINAL_DESCENT"]},
            "touchdown": {"state_sequence": valid["state_sequence"] + ["TOUCHDOWN_HOLD"]},
            "contact": {"ground_truth_contact_count": 1},
            "penetration": {"ground_truth_penetration_count": 1},
            "clearance": {"ground_truth_minimum_skid_clearance_m": 0.089},
            "horizontal_rmse": {"horizontal_error_rmse_m": 0.081},
            "horizontal_max": {"horizontal_error_max_m": 0.151},
            "normal_rmse": {"normal_rmse_deg": 1.01},
            "normal_p95": {"normal_p95_deg": 1.51},
            "sign_accuracy": {"sign_accuracy": 0.949},
            "marker_jump": {"marker_switch_jump_max_deg": 1.01},
            "time_sync": {"time_sync_failure_count": 1},
            "nan_inf": {"nan_inf_count": 1},
            "nav_land": {"nav_land_command_count": 1},
            "disarm": {"disarm_command_count": 1},
        }
        for expected_check, override in failures.items():
            metrics = dict(valid)
            metrics.update(override)
            result = fixed_tilt_safe_descent_gate(metrics)
            self.assertFalse(result["passed"], expected_check)
            self.assertIn(expected_check, result["failed_checks"])

    def test_calibration_gate_uses_frozen_fixed_tilt_safe_altitude_thresholds(self) -> None:
        passed = calibration_gate(
            signed_axis_errors_deg=[0.1, -0.2, 0.3],
            normal_angle_errors_deg=[0.1, 0.2, 0.3],
            sign_correct_samples=[True, True, True],
            marker_switch_jumps_deg=[0.4, 0.7],
        )
        self.assertTrue(passed["passed"])
        self.assertEqual(passed["thresholds"]["mean_signed_error_abs_max_deg"], 0.5)
        self.assertEqual(passed["thresholds"]["rmse_max_deg"], 1.0)
        self.assertEqual(passed["thresholds"]["p95_max_deg"], 1.5)
        self.assertEqual(passed["thresholds"]["sign_accuracy_min"], 0.95)
        self.assertEqual(passed["thresholds"]["marker_switch_jump_max_deg"], 1.0)

        failed = calibration_gate(
            signed_axis_errors_deg=[0.2, 0.3],
            normal_angle_errors_deg=[1.4, 1.7],
            sign_correct_samples=[True, False],
            marker_switch_jumps_deg=[1.2],
        )
        self.assertFalse(failed["passed"])
        self.assertNotIn("mean_signed_error", failed["failed_checks"])
        self.assertIn("p95", failed["failed_checks"])
        self.assertIn("sign_accuracy", failed["failed_checks"])
        self.assertIn("marker_switch_jump", failed["failed_checks"])

    def test_normal_angle_error(self) -> None:
        self.assertAlmostEqual(
            normal_angle_error_deg((0.0, 0.0, -1.0), (0.0, 0.0, -2.0)), 0.0
        )
        self.assertAlmostEqual(
            normal_angle_error_deg((0.0, 0.0, -1.0), (0.0, 1.0, 0.0)), 90.0
        )
        two_deg = math.radians(2.0)
        tilted = (0.0, math.sin(two_deg), -math.cos(two_deg))
        self.assertAlmostEqual(
            normal_angle_error_deg(tilted, (0.0, 0.0, -1.0)), 2.0
        )

    def test_contact_index_and_clearance_spread(self) -> None:
        clearances = (0.12, 0.11, 0.14, 0.13)
        self.assertEqual(contact_point_index(clearances), 1)
        self.assertAlmostEqual(clearance_spread(clearances), 0.03)
        # 并列最小值必须保持首索引，和 C++ std::min_element 语义一致。
        self.assertEqual(contact_point_index((0.1, 0.1, 0.2, 0.2)), 0)

    def test_contact_helpers_reject_invalid_data(self) -> None:
        with self.assertRaises(ValueError):
            contact_point_index((0.1, 0.2, 0.3))
        with self.assertRaises(ValueError):
            clearance_spread((0.1, math.nan, 0.2, 0.3))
        self.assertFalse(finite_values((0.0, math.inf)))

    def test_trajectory_nan_is_hard_failure_only_after_tracking_begins(self) -> None:
        times = [1.0, 5.0, 10.0]
        self.assertEqual(nonfinite_times_at_or_after(times, 5.0), [5.0, 10.0])
        self.assertEqual(nonfinite_times_at_or_after(times, None), times)

    def test_hard_gate_sample_window_excludes_shutdown_tail(self) -> None:
        samples = [
            TimedSample(10.0, (0.02,)),
            TimedSample(20.0, (0.02,)),
            TimedSample(20.05, (0.06,)),
        ]
        bounded = timed_samples_between(samples, 10.0, 20.0)
        self.assertEqual([sample.values[0] for sample in bounded], [0.02, 0.02])
        self.assertEqual(timed_samples_between(samples, None, 20.0), [])

    def test_nearest_sample_preserves_same_cycle_shadow_values(self) -> None:
        samples = [
            TimedSample(1.000, (1.0,)),
            TimedSample(1.050, (2.0,)),
        ]
        self.assertEqual(nearest_sample(samples, 1.002, 0.01), (1.0,))
        self.assertEqual(nearest_sample(samples, 1.048, 0.01), (2.0,))
        self.assertIsNone(nearest_sample(samples, 1.025, 0.01))

    def test_static_calibration_does_not_apply_signed_axis_or_sign_gate(self) -> None:
        result = calibration_gate(
            signed_axis_errors_deg=[0.7, 0.8],
            normal_angle_errors_deg=[0.7, 0.8],
            sign_correct_samples=[True, True],
            marker_switch_jumps_deg=[],
            require_signed_mean=False,
            require_sign_accuracy=False,
        )
        self.assertTrue(result["passed"])
        self.assertEqual(result["applicability"]["mean_signed_error"], "not_applicable")
        self.assertEqual(result["applicability"]["sign_accuracy"], "not_applicable")

    def test_time_interpolation_and_gap_rejection(self) -> None:
        samples = [
            TimedSample(1.0, (0.0, 2.0)),
            TimedSample(3.0, (2.0, 4.0)),
        ]
        self.assertEqual(interpolate_sample(samples, 2.0, 1.0), (1.0, 3.0))
        self.assertEqual(interpolate_sample(samples, 1.0, 0.0), (0.0, 2.0))
        self.assertIsNone(interpolate_sample(samples, 2.0, 0.9))
        self.assertEqual(interpolate_sample(samples, 0.95, 0.1), (0.0, 2.0))
        self.assertIsNone(interpolate_sample(samples, 0.0, 0.5))
        self.assertIsNone(interpolate_sample([], 1.0, 0.1))

    def test_marker_history_height_topic_falls_back_to_raw_safe_altitude_topic(self) -> None:
        self.assertEqual(
            select_marker_history_height_topic({"/landing/relative_height"}),
            "/landing/relative_height",
        )
        self.assertEqual(
            select_marker_history_height_topic({"/landing/raw_relative_height"}),
            "/landing/raw_relative_height",
        )
        self.assertEqual(
            select_marker_history_height_topic(
                {"/landing/relative_height", "/landing/raw_relative_height"},
                {
                    "/landing/relative_height": 0,
                    "/landing/raw_relative_height": 10,
                },
            ),
            "/landing/raw_relative_height",
        )
        with self.assertRaisesRegex(RuntimeError, "relative height topic"):
            select_marker_history_height_topic(set())

    def test_special_bag_containers_count_as_topic_messages(self) -> None:
        numeric: dict[str, list[TimedSample]] = {}
        strings: dict[str, list[tuple[float, str]]] = {}
        loaded = {
            "local_positions": [(1.0, object())],
            "vehicle_odometry": [TimedSample(1.0, (0.0,) * 7)],
            "commands": [(400, 1.0)],
            "land_flags": [(1.0, {"landed": False})],
        }
        self.assertTrue(
            topic_has_messages("/fmu/out/vehicle_local_position_v1", numeric, strings, loaded)
        )
        self.assertTrue(
            topic_has_messages("/fmu/out/vehicle_odometry", numeric, strings, loaded)
        )
        self.assertTrue(
            topic_has_messages("/fmu/in/vehicle_command", numeric, strings, loaded)
        )
        self.assertTrue(
            topic_has_messages("/fmu/out/vehicle_land_detected", numeric, strings, loaded)
        )
        self.assertFalse(topic_has_messages("/missing", numeric, strings, loaded))

    def test_missing_topic_error_names_missing_topics(self) -> None:
        with self.assertRaisesRegex(RuntimeError, BODY_CLEARANCE_TOPIC):
            validate_required_topics({NORMAL_TOPIC}, {NORMAL_TOPIC, BODY_CLEARANCE_TOPIC})
        validate_required_topics(REQUIRED_SHADOW_TOPICS)

    def test_ground_truth_identity_normal_converts_enu_to_ned_up(self) -> None:
        normal = ground_truth_upward_normal_ned((1.0, 0.0, 0.0, 0.0))
        self.assertEqual(normal, (0.0, 0.0, -1.0))

    def test_yaml_keeps_geometry_shadow_only_and_production_gates_closed(self) -> None:
        config_path = (
            WORKSPACE_DIR
            / "src"
            / "aruco_precision_landing_cpp"
            / "config"
            / "px4_aruco_landing.yaml"
        )
        parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
            "px4_aruco_landing_node"
        ]["ros__parameters"]
        self.assertTrue(parameters["deck_plane_geometry.enabled"])
        self.assertTrue(parameters["deck_plane_geometry.shadow_only"])
        self.assertEqual(
            parameters["deck_plane_geometry.contact_points_body_frd_m"],
            [
                -0.125,
                -0.132,
                0.227,
                0.125,
                -0.132,
                0.227,
                -0.125,
                0.132,
                0.227,
                0.125,
                0.132,
                0.227,
            ],
        )
        self.assertFalse(parameters["descent.enabled"])
        self.assertFalse(parameters["final_descent.enabled"])
        self.assertFalse(parameters["enable_auto_land"])
        self.assertEqual(parameters["tracking.mode"], "PREDICTED_POSITION_VELOCITY_FF")

    def test_shadow_normal_filter_is_independent_from_production_attitude_filter(self) -> None:
        config_path = (
            WORKSPACE_DIR
            / "src"
            / "aruco_precision_landing_cpp"
            / "config"
            / "px4_aruco_landing.yaml"
        )
        parameters = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
            "px4_aruco_landing_node"
        ]["ros__parameters"]
        self.assertEqual(parameters["deck_attitude.filter_gain"], 0.20)
        self.assertEqual(parameters["deck_plane_geometry.normal_filter_gain"], 0.08)

        header = (
            WORKSPACE_DIR
            / "src"
            / "aruco_precision_landing_cpp"
            / "include"
            / "aruco_precision_landing_cpp"
            / "px4_aruco_landing_node.hpp"
        ).read_text(encoding="utf-8")
        source = (
            WORKSPACE_DIR
            / "src"
            / "aruco_precision_landing_cpp"
            / "src"
            / "px4_aruco_landing_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("deck_plane_shadow_attitude_estimator_", header)
        self.assertIn("shadow_deck_attitude_", header)
        self.assertIn(
            "input.upward_normal_ned = shadow_deck_attitude_.upward_normal_ned;",
            source,
        )

        landing_window_start = source.index("void Px4ArucoLandingNode::update_landing_window")
        landing_window_end = source.index(
            "std::optional<RelativeDescentOutput>", landing_window_start
        )
        landing_window_source = source[landing_window_start:landing_window_end]
        self.assertIn("estimated_deck_attitude_", landing_window_source)
        self.assertNotIn("shadow_deck_attitude_", landing_window_source)
        self.assertNotIn("deck_plane_shadow_attitude_estimator_", landing_window_source)

    def test_shadow_computation_is_after_trajectory_publish_and_not_in_state_machine(self) -> None:
        source_path = (
            WORKSPACE_DIR
            / "src"
            / "aruco_precision_landing_cpp"
            / "src"
            / "px4_aruco_landing_node.cpp"
        )
        source = source_path.read_text(encoding="utf-8")
        timer_start = source.index("void Px4ArucoLandingNode::control_timer_callback()")
        state_machine_start = source.index("void Px4ArucoLandingNode::run_state_machine")
        timer = source[timer_start:state_machine_start]
        self.assertLess(
            timer.index("publish_trajectory_setpoint();"),
            timer.index("update_deck_plane_geometry_shadow(now);")
        )
        state_machine_end = source.index(
            "void Px4ArucoLandingNode::transition_to", state_machine_start
        )
        state_machine = source[state_machine_start:state_machine_end]
        self.assertNotIn("deck_plane_geometry", state_machine)
        ground_truth_topic = "/simulation/deck/" + "ground_truth"
        self.assertNotIn(ground_truth_topic, source)

    def test_ground_truth_is_absent_from_all_production_control_and_detection_sources(self) -> None:
        ground_truth_topic = "/simulation/deck/" + "ground_truth"
        production_files = [
            WORKSPACE_DIR / "src" / "aruco_precision_landing_cpp" / "src" / name
            for name in (
                "px4_aruco_landing_node.cpp",
                "relative_descent_controller.cpp",
                "final_descent_controller.cpp",
                "touchdown_detector.cpp",
                "touchdown_hold_controller.cpp",
                "relative_mpc_controller.cpp",
            )
        ]
        production_files.extend(
            (WORKSPACE_DIR / "src" / "aruco_detector").rglob("*.cpp")
        )
        for path in production_files:
            self.assertNotIn(
                ground_truth_topic,
                path.read_text(encoding="utf-8"),
                f"Ground Truth leaked into production source: {path}",
            )

    def test_evaluator_treats_time_sync_failure_as_hard_failure(self) -> None:
        evaluator_source = (WORKSPACE_DIR / "scripts" / "evaluate_tilted_deck.py").read_text(
            encoding="utf-8"
        )
        self.assertIn("time_sync_failures == 0", evaluator_source)
        self.assertIn("total_invalid_numeric == 0", evaluator_source)
        self.assertIn("consistency_failures == 0", evaluator_source)

    def test_stale_process_gate_excludes_automation_ancestor_chain(self) -> None:
        start_script = (WORKSPACE_DIR / "scripts" / "start_sitl.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('ancestor_pids=" $$ "', start_script)
        self.assertIn('ps -o ppid= -p "$ancestor_pid"', start_script)
        self.assertIn('*" $process_pid "*) ;;', start_script)
        self.assertIn('{ pgrep -af "$stale_pattern" || true; }', start_script)

    def test_all_shadow_topics_are_recorded(self) -> None:
        start_script = (WORKSPACE_DIR / "scripts" / "start_sitl.sh").read_text(
            encoding="utf-8"
        )
        for topic in REQUIRED_SHADOW_TOPICS:
            self.assertIn(topic, start_script)


if __name__ == "__main__":
    unittest.main()
