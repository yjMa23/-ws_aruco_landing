#!/usr/bin/env python3
"""不启动 PX4/Gazebo 的 P7 批量实验管线测试。"""

from __future__ import annotations

import json
import sys
import tempfile
import textwrap
import unittest
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

WORKSPACE_DIR = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = WORKSPACE_DIR / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

import aggregate_results  # noqa: E402
import evaluate_p4_bag  # noqa: E402
import evaluate_p6b_touchdown  # noqa: E402
import mavlink_gcs_heartbeat  # noqa: E402
import reevaluate_experiment  # noqa: E402
import run_batch_experiments  # noqa: E402
import run_single_experiment  # noqa: E402
from p7_experiment_utils import (  # noqa: E402
    classify_failure,
    combination_is_applicable,
    episode_result_complete,
    expand_seeds,
    load_batch_config,
    make_batch_id,
    make_episode_id,
    metric_value,
    read_evaluation_json,
    summarize_values,
)


class P7ExperimentTests(unittest.TestCase):
    def test_yaml_config_parsing_and_seed_expansion(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "config.yaml"
            config_path.write_text(
                textwrap.dedent(
                    """
                    name: smoke
                    output_root: results
                    episode_timeout_s: 30
                    startup_timeout_s: 5
                    touchdown_hold_s: 10
                    scenarios:
                      - scenario: static
                        repetitions: 3
                        seeds: [10]
                      - scenario: constant02
                        repetitions: 2
                        seeds: [20, 30]
                    """
                ),
                encoding="utf-8",
            )
            config = load_batch_config(config_path, workspace_dir=root)
            self.assertEqual(config.scenarios[0].seeds, (10, 11, 12))
            self.assertEqual(config.scenarios[1].seeds, (20, 30))
            self.assertEqual(config.output_root, root / "results")

    def test_p8a_heave_profile_is_supported_by_batch_automation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "p8a.yaml"
            config_path.write_text(
                textwrap.dedent(
                    """
                    name: p8a-h1
                    output_root: results
                    episode_timeout_s: 180
                    startup_timeout_s: 60
                    touchdown_hold_s: 10
                    scenarios:
                      - scenario: heave_h1
                        repetitions: 3
                        seeds: [101]
                    """
                ),
                encoding="utf-8",
            )
            config = load_batch_config(config_path, workspace_dir=root)
            self.assertEqual(config.scenarios[0].scenario, "heave_h1")
            self.assertEqual(config.scenarios[0].seeds, (101, 102, 103))

    def test_p8c3_positive_fixed_tilt_profiles_are_supported_by_automation(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "p8c3.yaml"
            config_path.write_text(
                textwrap.dedent(
                    """
                    name: p8c3
                    output_root: results
                    scenarios:
                      - scenario: tilt_roll_pos_2deg
                        repetitions: 3
                        seeds: [1]
                      - scenario: tilt_pitch_pos_2deg
                        repetitions: 3
                        seeds: [11]
                    """
                ),
                encoding="utf-8",
            )
            config = load_batch_config(config_path, workspace_dir=root)
            self.assertEqual(config.scenarios[0].seeds, (1, 2, 3))
            self.assertEqual(config.scenarios[1].seeds, (11, 12, 13))

    def test_seed_expansion_rejects_mismatch(self) -> None:
        with self.assertRaises(ValueError):
            expand_seeds(3, [1, 2])

    def test_episode_ids_are_unique(self) -> None:
        batch_id = make_batch_id("smoke", datetime(2026, 7, 29, 12, 0, 0))
        first = make_episode_id(batch_id, "static", 1, 1)
        second = make_episode_id(batch_id, "static", 2, 2)
        third = make_episode_id(batch_id, "constant02", 1, 1)
        self.assertEqual(len({first, second, third}), 3)

    def test_completed_success_can_be_archived_after_code_change(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            episode_dir = Path(directory) / "episode"
            episode_dir.mkdir()
            (episode_dir / "manifest.json").write_text(
                json.dumps({"completed": True, "success": True}),
                encoding="utf-8",
            )
            archive = run_single_experiment.archive_completed_episode(
                episode_dir, require_failure=False
            )
            self.assertFalse(episode_dir.exists())
            self.assertTrue(archive.exists())
            self.assertIn("superseded_attempt01", archive.name)

    def test_interrupted_episode_can_be_archived_after_code_change(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            episode_dir = Path(directory) / "episode"
            episode_dir.mkdir()
            (episode_dir / "manifest.json").write_text(
                json.dumps({"completed": False, "success": False}),
                encoding="utf-8",
            )
            archive = run_single_experiment.archive_completed_episode(
                episode_dir, require_failure=False
            )
            self.assertFalse(episode_dir.exists())
            self.assertTrue(archive.exists())
            self.assertIn("interrupted_attempt01", archive.name)

    def test_failed_episode_retry_archives_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            episode_dir = Path(directory) / "episode"
            episode_dir.mkdir()
            (episode_dir / "run.log").write_text("startup failure\n", encoding="utf-8")
            (episode_dir / "manifest.json").write_text(
                json.dumps({"completed": True, "success": False}),
                encoding="utf-8",
            )
            archive = run_single_experiment.archive_failed_episode(episode_dir)
            self.assertFalse(episode_dir.exists())
            self.assertTrue((archive / "run.log").is_file())
            self.assertIn("failed_attempt01", archive.name)

    def test_resume_requires_complete_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            episode_dir = Path(directory)
            manifest = {
                "completed": True,
                "success": False,
                "failure_reason": "STARTUP_FAILURE",
                "evaluation_path": None,
            }
            (episode_dir / "manifest.json").write_text(
                json.dumps(manifest), encoding="utf-8"
            )
            self.assertTrue(episode_result_complete(episode_dir))
            manifest["completed"] = False
            (episode_dir / "manifest.json").write_text(
                json.dumps(manifest), encoding="utf-8"
            )
            self.assertFalse(episode_result_complete(episode_dir))

    def test_evaluator_json_parsing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "evaluation.json"
            path.write_text(
                json.dumps({"positive_touchdown_passed": True}), encoding="utf-8"
            )
            self.assertTrue(read_evaluation_json(path)["positive_touchdown_passed"])
            path.write_text("{}", encoding="utf-8")
            with self.assertRaises(ValueError):
                read_evaluation_json(path)

    def test_success_timeout_and_startup_failure_classification(self) -> None:
        self.assertEqual(classify_failure(success=True), "NONE")
        self.assertEqual(
            classify_failure(
                success=False,
                event="EPISODE_TIMEOUT",
                state_sequence=["FINAL_DESCENT"],
            ),
            "TOUCHDOWN_NOT_CONFIRMED",
        )
        self.assertEqual(
            classify_failure(success=False, event="STARTUP_FAILURE"),
            "STARTUP_FAILURE",
        )
        self.assertEqual(
            classify_failure(
                success=False,
                event="EPISODE_TIMEOUT",
                state_sequence=["FINAL_DESCENT"],
                log_text="created rt/fmu/out/failsafe_flags data writer",
            ),
            "TOUCHDOWN_NOT_CONFIRMED",
        )

    def test_batch_continues_after_single_failure(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "batch.yaml"
            config_path.write_text(
                textwrap.dedent(
                    f"""
                    name: continue-test
                    output_root: {root / 'results'}
                    episode_timeout_s: 30
                    startup_timeout_s: 5
                    touchdown_hold_s: 10
                    scenarios:
                      - scenario: static
                        repetitions: 2
                        seeds: [1, 2]
                    """
                ),
                encoding="utf-8",
            )
            fake_runner = root / "fake_runner.py"
            fake_runner.write_text(
                textwrap.dedent(
                    """
                    import argparse
                    import json
                    from pathlib import Path

                    parser = argparse.ArgumentParser()
                    parser.add_argument('--scenario')
                    parser.add_argument('--seed', type=int)
                    parser.add_argument('--output-directory', type=Path)
                    parser.add_argument('--batch-id')
                    parser.add_argument('--episode-id')
                    args, _ = parser.parse_known_args()
                    episode_dir = args.output_directory / args.episode_id
                    episode_dir.mkdir(parents=True, exist_ok=True)
                    success = args.seed == 2
                    manifest = {
                        'episode_id': args.episode_id,
                        'batch_id': args.batch_id,
                        'scenario': args.scenario,
                        'seed': args.seed,
                        'completed': True,
                        'success': success,
                        'failure_reason': 'NONE' if success else 'STARTUP_FAILURE',
                        'evaluation_path': None,
                    }
                    (episode_dir / 'manifest.json').write_text(json.dumps(manifest))
                    raise SystemExit(0 if success else 3)
                    """
                ),
                encoding="utf-8",
            )
            args = SimpleNamespace(
                config=config_path,
                batch_id="batch-fixed",
                resume=False,
                dry_run=False,
                workspace_dir=WORKSPACE_DIR,
                single_runner=fake_runner,
            )
            result = run_batch_experiments.run_batch(args)
            self.assertEqual(result["completed_episodes"], 2)
            self.assertEqual(result["successful_episodes"], 1)
            self.assertEqual(result["failed_episodes"], 1)

    def test_mpc_contact_disengagement_is_not_counted_as_solver_failure(self) -> None:
        self.assertTrue(
            evaluate_p4_bag.mpc_status_is_intentional_disengagement(
                "TERMINAL_PHASE_P47"
            )
        )
        self.assertFalse(
            evaluate_p4_bag.mpc_status_is_solver_success("TERMINAL_PHASE_P47")
        )
        self.assertTrue(evaluate_p4_bag.mpc_status_is_solver_success("solved"))
        self.assertFalse(
            evaluate_p4_bag.mpc_status_is_intentional_disengagement(
                "MAXIMUM_ITERATIONS"
            )
        )

    def test_aggregate_statistics(self) -> None:
        values = [1.0, 2.0, 3.0, 4.0]
        summary = summarize_values(values)
        self.assertEqual(summary["mean"], 2.5)
        self.assertEqual(summary["median"], 2.5)
        self.assertAlmostEqual(summary["p90"], 3.7)
        self.assertAlmostEqual(summary["p95"], 3.85)

    def test_empty_and_corrupt_aggregate_results(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            batch_dir = Path(directory)
            summary = aggregate_results.aggregate(batch_dir)
            self.assertEqual(summary["total_experiments"], 0)
            self.assertEqual(summary["success_rate"], 0.0)
            corrupt_dir = batch_dir / "broken"
            corrupt_dir.mkdir()
            (corrupt_dir / "manifest.json").write_text("{bad", encoding="utf-8")
            summary = aggregate_results.aggregate(batch_dir)
            self.assertEqual(summary["corrupt_result_count"], 1)
            self.assertTrue((batch_dir / "failures.csv").is_file())

    def test_aggregate_successful_episode_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            batch_dir = Path(directory)
            for index, landing_time in enumerate((4.0, 6.0), start=1):
                episode_dir = batch_dir / f"episode-{index}"
                episode_dir.mkdir()
                evaluation_path = episode_dir / "evaluation.json"
                evaluation_path.write_text(
                    json.dumps(
                        {
                            "positive_touchdown_passed": True,
                            "landing_time_s": landing_time,
                            "horizontal_error_rmse_m": 0.1 * index,
                            "horizontal_error_max_m": 0.2 * index,
                            "touchdown_vertical_speed_mps": 0.03,
                            "candidate_to_confirm_delay_s": 0.5,
                            "recovery_count": 0,
                            "marker_switch_count": index,
                            "hold_duration_s": 10.0,
                        }
                    ),
                    encoding="utf-8",
                )
                (episode_dir / "manifest.json").write_text(
                    json.dumps(
                        {
                            "episode_id": f"episode-{index}",
                            "scenario": "static",
                            "seed": index,
                            "completed": True,
                            "success": True,
                            "failure_reason": "NONE",
                            "evaluation_path": str(evaluation_path),
                        }
                    ),
                    encoding="utf-8",
                )
            summary = aggregate_results.aggregate(batch_dir)
            self.assertEqual(summary["success_count"], 2)
            self.assertEqual(summary["metrics"]["landing_time_s"]["mean"], 5.0)

    def test_evaluator_selection_uses_p8a_for_heave_profiles(self) -> None:
        self.assertEqual(
            run_single_experiment.evaluator_path_for_scenario(
                WORKSPACE_DIR, "static"
            ).name,
            "evaluate_p6b_touchdown.py",
        )
        self.assertEqual(
            run_single_experiment.evaluator_path_for_scenario(
                WORKSPACE_DIR, "heave_h1"
            ).name,
            "evaluate_p8a_heave_touchdown.py",
        )

    def test_dual_evaluators_must_both_pass(self) -> None:
        self.assertTrue(
            run_single_experiment.evaluators_passed(
                {"positive_touchdown_passed": True},
                {"positive_touchdown_passed": True},
            )
        )
        self.assertFalse(
            run_single_experiment.evaluators_passed(
                {"positive_touchdown_passed": True},
                {"positive_touchdown_passed": False},
            )
        )
        self.assertFalse(run_single_experiment.evaluators_passed(None, None))

    def test_terminal_recovery_fails_fast_instead_of_second_landing(self) -> None:
        self.assertTrue(
            run_single_experiment.recovery_after_terminal_phase(
                ["FINAL_DESCENT", "TOUCHDOWN_CANDIDATE_HOLD", "TOUCHDOWN_HOLD"],
                "RECOVER_TO_GNSS",
            )
        )
        self.assertFalse(
            run_single_experiment.recovery_after_terminal_phase(
                ["TRACK_TARGET"], "RECOVER_TO_GNSS"
            )
        )

    def test_touchdown_hold_recording_adds_shutdown_margin(self) -> None:
        self.assertEqual(
            run_single_experiment.required_hold_recording_duration_s(10.0), 11.0
        )
        with self.assertRaises(ValueError):
            run_single_experiment.required_hold_recording_duration_s(-0.1)

    def test_local_gcs_heartbeat_fields_are_standard_and_noncontrolling(self) -> None:
        fields = mavlink_gcs_heartbeat.heartbeat_fields()
        self.assertEqual(fields["mav_type"], 6)  # MAV_TYPE_GCS
        self.assertEqual(fields["autopilot"], 8)  # MAV_AUTOPILOT_INVALID
        self.assertEqual(fields["base_mode"], 0)
        self.assertEqual(fields["custom_mode"], 0)
        self.assertEqual(fields["system_status"], 4)  # MAV_STATE_ACTIVE
        self.assertEqual(fields["mavlink_version"], 3)

    def test_local_gcs_heartbeat_uses_pymavlink_argument_order(self) -> None:
        class Recorder:
            def __init__(self) -> None:
                self.arguments: tuple[int, ...] | None = None

            def heartbeat_send(self, *arguments: int) -> None:
                self.arguments = arguments

        class Connection:
            def __init__(self) -> None:
                self.mav = Recorder()

        connection = Connection()
        mavlink_gcs_heartbeat.send_heartbeat(connection)
        self.assertEqual(connection.mav.arguments, (6, 8, 0, 0, 4, 3))

    def test_start_sitl_owns_gcs_heartbeat_and_cleanup(self) -> None:
        script = (WORKSPACE_DIR / "scripts" / "start_sitl.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('start_process "local GCS heartbeat"', script)
        self.assertIn("mavlink_gcs_heartbeat.py", script)
        self.assertIn("mavlink_gcs_heartbeat.py", run_single_experiment.STALE_PATTERN)
        self.assertLess(
            script.index('start_process "local GCS heartbeat"'),
            script.index('echo "Waiting for PX4 ROS topics..."'),
        )

    def test_p8c4_profile_completion_requirements_are_state_driven(self) -> None:
        states, duration = run_single_experiment.completion_requirement(
            "safe-altitude", 10.0
        )
        self.assertEqual(states, frozenset({"WAIT_LANDING_WINDOW"}))
        self.assertEqual(duration, 10.0)
        states, duration = run_single_experiment.completion_requirement(
            "safe-descent", 10.0
        )
        self.assertEqual(states, frozenset({"TEST_HEIGHT_HOLD"}))
        self.assertEqual(duration, 9.0)
        states, duration = run_single_experiment.completion_requirement(
            "rehearsal", 10.0
        )
        self.assertEqual(states, frozenset({"TEST_HEIGHT_HOLD"}))
        self.assertEqual(duration, 10.0)
        states, duration = run_single_experiment.completion_requirement(
            "touchdown", 10.0
        )
        self.assertEqual(states, frozenset({"TOUCHDOWN_HOLD"}))
        self.assertEqual(duration, 11.0)

    def test_p8c4_staged_commands_are_strict_and_mutually_separated(self) -> None:
        common = (
            WORKSPACE_DIR,
            "tilt_roll_pos_2deg",
            1,
            Path("/tmp/p8c4-bag"),
            "close-range",
            False,
            "PREDICTED_POSITION_VELOCITY_FF",
        )
        safe_altitude = run_single_experiment.build_start_command(
            *common,
            experiment_profile="safe-altitude",
            terminal_stabilization_mode="shadow",
        )
        self.assertIn("--terminal-contact-stabilization-shadow", safe_altitude)
        self.assertNotIn("--enable-relative-descent", safe_altitude)
        self.assertNotIn("--enable-final-descent", safe_altitude)

        safe_descent = run_single_experiment.build_start_command(
            *common,
            experiment_profile="safe-descent",
            terminal_stabilization_mode="shadow",
        )
        self.assertIn("--enable-relative-descent", safe_descent)
        self.assertNotIn("--enable-final-descent", safe_descent)

        rehearsal = run_single_experiment.build_start_command(
            *common,
            experiment_profile="rehearsal",
            terminal_stabilization_mode="rehearsal",
        )
        self.assertIn("--terminal-contact-stabilization-rehearsal", rehearsal)
        self.assertNotIn("--enable-final-descent", rehearsal)

        touchdown = run_single_experiment.build_start_command(
            *common,
            experiment_profile="touchdown",
            terminal_stabilization_mode="active",
        )
        self.assertIn("--enable-terminal-contact-stabilization", touchdown)
        self.assertIn("--enable-final-descent", touchdown)

    def test_legacy_p6b_contact_gate_uses_confirmed_time_not_preconfirm_minimum(self) -> None:
        result = evaluate_p6b_touchdown.contact_clearance_gate(
            confirmed_clearance_m=0.0003609528735845851,
            maximum_contact_clearance_m=0.03,
            maximum_contact_penetration_m=0.05,
        )
        self.assertTrue(result)
        self.assertFalse(
            evaluate_p6b_touchdown.contact_clearance_gate(
                confirmed_clearance_m=-0.05014470920585659,
                maximum_contact_clearance_m=0.03,
                maximum_contact_penetration_m=0.05,
            )
        )

    def test_non_touchdown_evaluator_accepts_p8c_final_result(self) -> None:
        self.assertTrue(
            run_single_experiment.evaluators_passed(
                {"final_result": "PASS"},
                None,
                experiment_profile="safe-descent",
            )
        )
        self.assertFalse(
            run_single_experiment.evaluators_passed(
                {"final_result": "FAIL"},
                None,
                experiment_profile="safe-descent",
            )
        )

    def test_p8c3_evaluator_selection_and_frozen_command(self) -> None:
        evaluator = run_single_experiment.evaluator_path_for_scenario(
            WORKSPACE_DIR,
            "tilt_roll_pos_2deg",
            p8c3_touchdown=True,
        )
        self.assertEqual(evaluator.name, "evaluate_p8c_tilted_deck.py")
        command = run_single_experiment.build_start_command(
            WORKSPACE_DIR,
            "tilt_roll_pos_2deg",
            1,
            Path("/tmp/p8c3-bag"),
            "close-range",
            False,
            "PREDICTED_POSITION_VELOCITY_FF",
        )
        self.assertIn("--descent-test-height", command)
        self.assertEqual(command[command.index("--descent-test-height") + 1], "0.50")
        self.assertIn("--enable-relative-descent", command)
        self.assertIn("--enable-final-descent", command)

    def test_p8c3_tilt_dry_run_is_noninteractive_and_strict(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(
                scenario="tilt_roll_pos_2deg",
                seed=1,
                episode_timeout=180.0,
                startup_timeout=60.0,
                touchdown_hold=10.0,
                output_directory=Path(directory),
                batch_id="p8c3-dry",
                episode_id="p8c3-dry-roll",
                camera_model="close-range",
                tracking_mode="PREDICTED_POSITION_VELOCITY_FF",
                record_camera_debug=False,
                p8c3_touchdown=True,
                dry_run=True,
                workspace_dir=WORKSPACE_DIR,
            )
            with mock.patch.object(run_single_experiment.subprocess, "Popen") as popen:
                result = run_single_experiment.run_episode(args)
            popen.assert_not_called()
            self.assertTrue(result["p8c3_touchdown"])
            self.assertIn("tilt_roll_pos_2deg", result["command"])
            self.assertIn("--descent-test-height", result["command"])

    def test_heave_dry_run_uses_graded_scenario(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(
                scenario="heave_h1",
                seed=101,
                episode_timeout=180.0,
                startup_timeout=60.0,
                touchdown_hold=10.0,
                output_directory=Path(directory),
                batch_id="p8a-dry",
                episode_id="p8a-dry-heave-h1",
                camera_model="close-range",
                record_camera_debug=False,
                dry_run=True,
                workspace_dir=WORKSPACE_DIR,
            )
            with mock.patch.object(run_single_experiment.subprocess, "Popen") as popen:
                result = run_single_experiment.run_episode(args)
            popen.assert_not_called()
            self.assertIn("heave_h1", result["command"])
            self.assertIn("--enable-final-descent", result["command"])

    def test_mpc_dry_run_passes_explicit_tracking_mode(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(
                scenario="heave_h1",
                seed=101,
                episode_timeout=180.0,
                startup_timeout=60.0,
                touchdown_hold=10.0,
                output_directory=Path(directory),
                batch_id="p8b-dry",
                episode_id="p8b-dry-heave-h1",
                camera_model="close-range",
                tracking_mode="RELATIVE_MPC",
                record_camera_debug=False,
                dry_run=True,
                workspace_dir=WORKSPACE_DIR,
            )
            with mock.patch.object(run_single_experiment.subprocess, "Popen") as popen:
                result = run_single_experiment.run_episode(args)
            popen.assert_not_called()
            command = result["command"]
            self.assertIn("--tracking-mode", command)
            self.assertEqual(command[command.index("--tracking-mode") + 1], "RELATIVE_MPC")

    def test_dry_run_does_not_start_processes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            args = SimpleNamespace(
                scenario="static",
                seed=1,
                episode_timeout=30.0,
                startup_timeout=5.0,
                touchdown_hold=10.0,
                output_directory=Path(directory),
                batch_id="dry-batch",
                episode_id="dry-episode",
                camera_model="close-range",
                record_camera_debug=False,
                dry_run=True,
                workspace_dir=WORKSPACE_DIR,
            )
            with mock.patch.object(run_single_experiment.subprocess, "Popen") as popen:
                result = run_single_experiment.run_episode(args)
            popen.assert_not_called()
            self.assertTrue(result["dry_run"])
            self.assertFalse((Path(directory) / "dry-episode").exists())

    def test_p9_method_config_and_episode_id(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config_path = root / "p9.yaml"
            config_path.write_text(
                textwrap.dedent(
                    """
                    name: p9-test
                    output_root: results
                    experiments:
                      - method: B1
                        scenario: constant02
                        profile: safe-altitude
                        repetitions: 2
                        seeds: [11, 12]
                      - method: B2
                        scenario: static
                        profile: safe-altitude
                        repetitions: 1
                        seeds: [21]
                    """
                ),
                encoding="utf-8",
            )
            config = load_batch_config(config_path, workspace_dir=root)
            self.assertEqual(config.experiments[0].prediction_horizon_s, 0.0)
            self.assertEqual(config.experiments[1].velocity_feedforward_gain, 0.0)
            plan = run_batch_experiments.expanded_plan(config, "p9-fixed")
            self.assertIn("B1_constant02_safe_altitude", plan[0]["episode_id"])
            self.assertEqual(plan[0]["method"], "B1")

    def test_p9_duplicate_combination_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.yaml"
            path.write_text(
                textwrap.dedent(
                    """
                    experiments:
                      - method: B1
                        scenario: static
                        profile: safe-altitude
                        repetitions: 1
                        seeds: [1]
                      - method: B1
                        scenario: static
                        profile: safe-altitude
                        repetitions: 1
                        seeds: [2]
                    """
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate"):
                load_batch_config(path, workspace_dir=Path(directory))

    def test_p9_incompatible_matrix_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            path.write_text(
                textwrap.dedent(
                    """
                    experiments:
                      - method: B5
                        scenario: constant02
                        profile: touchdown
                        repetitions: 1
                        seeds: [1]
                    """
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "not safety-authorized"):
                load_batch_config(path, workspace_dir=Path(directory))

    def test_p9_negative_and_dynamic_tilt_remain_closed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            for scenario in ("tilt_roll_neg_2deg", "rollpitch", "combined"):
                path = Path(directory) / f"{scenario}.yaml"
                path.write_text(
                    textwrap.dedent(
                        f"""
                        experiments:
                          - method: B5
                            scenario: {scenario}
                            profile: touchdown
                            repetitions: 1
                            seeds: [1]
                        """
                    ),
                    encoding="utf-8",
                )
                with self.assertRaises(ValueError):
                    load_batch_config(path, workspace_dir=Path(directory))

    def test_p9_b3_b4_and_b5_frozen_parameters(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "methods.yaml"
            path.write_text(
                textwrap.dedent(
                    """
                    experiments:
                      - method: B3
                        scenario: sinusoidal
                        profile: safe-altitude
                        repetitions: 1
                        seeds: [1]
                      - method: B4
                        scenario: heave_h1
                        profile: touchdown
                        repetitions: 1
                        seeds: [2]
                      - method: B5
                        scenario: tilt_pitch_pos_2deg
                        profile: touchdown
                        repetitions: 1
                        seeds: [3]
                    """
                ),
                encoding="utf-8",
            )
            config = load_batch_config(path, workspace_dir=Path(directory))
            self.assertEqual(config.experiments[0].tracking_mode, "RELATIVE_MPC")
            self.assertTrue(config.experiments[1].vertical_velocity_feedforward_enabled)
            self.assertEqual(config.experiments[2].terminal_stabilization_mode, "active")

    def test_p9_method_identity_cannot_be_redefined(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "redefine.yaml"
            path.write_text(
                textwrap.dedent(
                    """
                    experiments:
                      - method: B1
                        scenario: static
                        profile: safe-altitude
                        repetitions: 1
                        seeds: [1]
                        prediction_horizon_s: 0.1
                    """
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "cannot override"):
                load_batch_config(path, workspace_dir=Path(directory))

    def test_p9_not_applicable_is_not_executed_or_failed(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "na.yaml"
            path.write_text(
                textwrap.dedent(
                    """
                    experiments:
                      - method: B1
                        scenario: constant
                        profile: touchdown
                        applicability: NOT_APPLICABLE
                        repetitions: 2
                        seeds: [1, 2]
                    """
                ),
                encoding="utf-8",
            )
            config = load_batch_config(path, workspace_dir=Path(directory))
            self.assertEqual(config.experiments[0].applicability, "NOT_APPLICABLE")
            self.assertEqual(run_batch_experiments.expanded_plan(config, "na"), [])

    def test_p9_evaluator_auto_routing(self) -> None:
        self.assertEqual(
            run_single_experiment.evaluator_path_for_scenario(
                WORKSPACE_DIR,
                "sinusoidal",
                experiment_profile="safe-altitude",
            ).name,
            "evaluate_p4_bag.py",
        )
        self.assertEqual(
            run_single_experiment.evaluator_path_for_scenario(
                WORKSPACE_DIR,
                "constant02",
                experiment_profile="safe-descent",
            ).name,
            "evaluate_p5b_bag.py",
        )
        self.assertEqual(
            run_single_experiment.evaluator_path_for_scenario(
                WORKSPACE_DIR,
                "tilt_roll_pos_2deg",
                use_p8c_evaluator=True,
                experiment_profile="touchdown",
            ).name,
            "evaluate_p8c_tilted_deck.py",
        )

    def test_p4_safe_altitude_evaluator_does_not_receive_p6b_flag(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with mock.patch.object(
                run_single_experiment,
                "_execute_evaluator",
                return_value=({"horizontal_position_rmse_m": 0.1}, None),
            ) as execute:
                run_single_experiment.run_evaluator(
                    WORKSPACE_DIR,
                    "constant02",
                    1,
                    root / "bag",
                    root,
                    mock.mock_open()(),
                    experiment_profile="safe-altitude",
                )
            command = execute.call_args.args[0]
            self.assertIn("evaluate_p4_bag.py", command[1])
            self.assertNotIn("--require-moving-deck", command)

    def test_p9_start_command_includes_method_parameters(self) -> None:
        command = run_single_experiment.build_start_command(
            WORKSPACE_DIR,
            "constant02",
            1,
            Path("/tmp/p9-bag"),
            "close-range",
            False,
            "PREDICTED_POSITION_VELOCITY_FF",
            "safe-altitude",
            "disabled",
            0.0,
            0.0,
            False,
            1.0,
            0.6,
        )
        self.assertEqual(command[command.index("--prediction-horizon") + 1], "0.0")
        self.assertEqual(command[command.index("--velocity-ff-gain") + 1], "0.0")
        self.assertIn("--disable-vertical-ff", command)
        self.assertNotIn("--enable-final-descent", command)

    def test_p9_resume_fingerprint_detects_code_change(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            episode_dir = Path(directory)
            evaluation = episode_dir / "evaluation.json"
            evaluation.write_text(json.dumps({"positive_touchdown_passed": True}))
            (episode_dir / "manifest.json").write_text(
                json.dumps(
                    {
                        "completed": True,
                        "success": True,
                        "failure_reason": "NONE",
                        "evaluation_path": str(evaluation),
                        "git_commit": "old",
                        "dirty_worktree": False,
                    }
                )
            )
            self.assertTrue(episode_result_complete(episode_dir))
            self.assertFalse(
                episode_result_complete(
                    episode_dir,
                    expected_git_commit="new",
                    expected_dirty_worktree=False,
                )
            )

    def test_p9_safe_altitude_primary_gate(self) -> None:
        self.assertTrue(
            run_single_experiment.evaluators_passed(
                {
                    "horizontal_position_rmse_m": 0.04,
                    "maximum_horizontal_error_m": 0.10,
                    "gnss_recovery_count": 0,
                    "mpc_non_solved_status_count": 0,
                    "mpc_deadline_miss_count": 0,
                },
                None,
                experiment_profile="safe-altitude",
                scenario="constant02",
            )
        )
        self.assertFalse(
            run_single_experiment.evaluators_passed(
                {
                    "horizontal_position_rmse_m": 0.20,
                    "maximum_horizontal_error_m": 0.40,
                    "gnss_recovery_count": 0,
                },
                None,
                experiment_profile="safe-altitude",
                scenario="constant02",
            )
        )

    def test_p9_aggregate_groups_and_nonfinite_values(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            batch_dir = Path(directory)
            for index, method in enumerate(("B0", "B3"), start=1):
                episode_dir = batch_dir / f"episode-{index}"
                episode_dir.mkdir()
                evaluation_path = episode_dir / "evaluation.json"
                evaluation_path.write_text(
                    json.dumps(
                        {
                            "positive_touchdown_passed": True,
                            "horizontal_error_rmse_m": 0.1 * index,
                            "landing_time_s": 5.0 + index,
                            "nav_land_commands": 0,
                            "disarm_commands": 0,
                            "unused_nan": float("nan"),
                        }
                    )
                )
                (episode_dir / "manifest.json").write_text(
                    json.dumps(
                        {
                            "episode_id": episode_dir.name,
                            "method": method,
                            "scenario": "constant02",
                            "profile": "touchdown",
                            "seed": index,
                            "completed": True,
                            "success": True,
                            "failure_reason": "NONE",
                            "evaluation_path": str(evaluation_path),
                        }
                    )
                )
            summary = aggregate_results.aggregate(
                batch_dir, generate_plot_files=False
            )
            self.assertEqual(summary["overall"]["success_count"], 2)
            self.assertIn("B0", summary["by_method"])
            self.assertIn("B3|constant02", summary["by_method_and_scenario"])
            self.assertTrue((batch_dir / "by_method.csv").is_file())
            self.assertTrue((batch_dir / "P9_RESULTS_SUMMARY.md").is_file())
            self.assertEqual(summary["safety"]["nav_land_count"], 0)

    def test_aggregate_ignores_archived_episode_attempts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            batch_dir = Path(directory)
            active = batch_dir / "episode-1"
            archived = batch_dir / "episode-1_failed_attempt01"
            for episode_dir, success in ((active, True), (archived, False)):
                episode_dir.mkdir()
                evaluation_path = episode_dir / "evaluation.json"
                evaluation_path.write_text(
                    json.dumps({"positive_touchdown_passed": success}),
                    encoding="utf-8",
                )
                (episode_dir / "manifest.json").write_text(
                    json.dumps(
                        {
                            "episode_id": episode_dir.name,
                            "method": "B0",
                            "scenario": "static",
                            "profile": "touchdown",
                            "seed": 1,
                            "completed": True,
                            "success": success,
                            "failure_reason": "NONE" if success else "STARTUP_FAILURE",
                            "evaluation_path": str(evaluation_path),
                        }
                    ),
                    encoding="utf-8",
                )
            records, issues = aggregate_results.collect_records(batch_dir)
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0]["manifest"]["episode_id"], "episode-1")
            self.assertEqual(issues, [])

    def test_metric_alias_and_null_handling(self) -> None:
        self.assertEqual(
            metric_value({"horizontal_position_rmse_m": 0.25}, "horizontal_error_rmse_m"),
            0.25,
        )
        self.assertIsNone(metric_value({"horizontal_error_rmse_m": None}, "horizontal_error_rmse_m"))
        self.assertIsNone(metric_value({"horizontal_error_rmse_m": float("inf")}, "horizontal_error_rmse_m"))
        summary = summarize_values([1.0, float("nan"), 3.0])
        self.assertEqual(summary["count"], 2)
        self.assertEqual(summary["min"], 1.0)
        self.assertEqual(summary["max"], 3.0)

    def test_metric_value_reads_p8c_nested_metrics(self) -> None:
        evaluation = {
            "p8c3_touchdown_metrics": {
                "horizontal_error_m": {"rmse": 0.031, "max_abs": 0.072},
                "touchdown_normal_relative_velocity_mps": -0.012,
                "normal_rmse_deg": 0.28,
                "normal_p95_deg": 0.61,
                "post_touchdown_tangential_slip_m": 0.056,
                "hold_tangential_velocity_p95_mps": 0.025,
                "attitude_divergence_delta_deg": 0.0,
            },
            "p8c4_terminal_stabilization_metrics": {
                "attitude_tracking_error_deg": {"rmse": 0.14, "p95": 0.10},
                "desired_tilt_deg": {"max_abs": 2.06},
                "command_slew_degps": {"max_abs": 4.16},
                "combined_acceleration_norm_mps2": {"max_abs": 0.35},
                "fallback_count_after_activation": 0,
                "activation_sample_count": 229,
            },
        }
        expected = {
            "horizontal_error_rmse_m": 0.031,
            "horizontal_error_max_m": 0.072,
            "touchdown_vertical_speed_mps": 0.012,
            "normal_tracking_error_rmse_deg": 0.14,
            "normal_tracking_error_p95_deg": 0.10,
            "touchdown_slip_m": 0.056,
            "hold_tangential_velocity_p95_mps": 0.025,
            "attitude_divergence_increment_deg": 0.0,
            "terminal_command_tilt_max_deg": 2.06,
            "terminal_command_tilt_slew_p100_degps": 4.16,
            "combined_horizontal_acceleration_max_mps2": 0.35,
            "fallback_count": 0.0,
            "terminal_stabilization_activation_count": 229.0,
        }
        for field, value in expected.items():
            self.assertEqual(metric_value(evaluation, field), value)

    def test_reevaluation_refreshes_parent_batch_counts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            batch_dir = Path(directory)
            episodes = []
            for index, success in enumerate((True, False), start=1):
                episode_id = f"episode-{index}"
                episodes.append({"episode_id": episode_id})
                episode_dir = batch_dir / episode_id
                episode_dir.mkdir()
                (episode_dir / "manifest.json").write_text(
                    json.dumps(
                        {
                            "completed": True,
                            "success": success,
                            "failure_reason": "NONE" if success else "SAFETY_GATE_FAILURE",
                        }
                    ),
                    encoding="utf-8",
                )
            (batch_dir / "batch_manifest.json").write_text(
                json.dumps(
                    {
                        "planned_episodes": 2,
                        "episodes": episodes,
                        "completed": False,
                    }
                ),
                encoding="utf-8",
            )
            reevaluate_experiment.refresh_parent_batch_manifest(batch_dir / "episode-1")
            refreshed = json.loads(
                (batch_dir / "batch_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(refreshed["completed_episodes"], 2)
            self.assertEqual(refreshed["successful_episodes"], 1)
            self.assertEqual(refreshed["failed_episodes"], 1)
            self.assertTrue(refreshed["completed"])

    def test_reevaluation_archives_previous_assessment_files(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            episode_dir = Path(directory)
            for name in ("evaluation.json", "evaluation.txt"):
                (episode_dir / name).write_text(name, encoding="utf-8")
            archive_dir = reevaluate_experiment.archive_evaluation_files(episode_dir)
            self.assertIsNotNone(archive_dir)
            assert archive_dir is not None
            self.assertTrue((archive_dir / "evaluation.json").is_file())
            self.assertTrue((archive_dir / "evaluation.txt").is_file())
            self.assertFalse((episode_dir / "evaluation.json").exists())
            self.assertEqual(
                reevaluate_experiment.archive_evaluation_files(episode_dir), None
            )

    def test_evaluator_json_parser_accepts_ros_prefix_and_nested_json(self) -> None:
        parsed = run_single_experiment.parse_evaluator_json_output(
            "[INFO] opened bag\n"
            "{\"horizontal_position_rmse_m\": 0.1, "
            "\"nested\": {\"CONFIRMED\": {\"count\": 12}}}\n"
        )
        self.assertEqual(parsed["horizontal_position_rmse_m"], 0.1)
        self.assertEqual(parsed["nested"]["CONFIRMED"]["count"], 12)

    def test_evaluator_json_parser_ignores_log_braces_and_allows_whitespace(self) -> None:
        parsed = run_single_experiment.parse_evaluator_json_output(
            "[INFO] waiting for state {READY}\n"
            "[WARN] ignored diagnostic={}\n"
            "{\"result\": {\"success\": true}}\n\t  "
        )
        self.assertTrue(parsed["result"]["success"])

    def test_evaluator_json_parser_rejects_garbage_or_missing_json(self) -> None:
        with self.assertRaises(ValueError):
            run_single_experiment.parse_evaluator_json_output(
                "[INFO] opened bag\n{\"success\": true}\ntrailing garbage"
            )
        with self.assertRaises(ValueError):
            run_single_experiment.parse_evaluator_json_output(
                "[INFO] no json, only a brace { in the log"
            )

    def test_stale_process_pid_parser_ignores_invalid_and_duplicates(self) -> None:
        self.assertEqual(
            run_single_experiment.stale_process_pids(
                ["123 MicroXRCEAgent udp4", "bad line", "123 duplicate", "456 gz sim"]
            ),
            [123, 456],
        )

    def test_frozen_applicability_examples(self) -> None:
        self.assertTrue(combination_is_applicable("B4", "heave_h1", "touchdown"))
        self.assertTrue(
            combination_is_applicable(
                "B5", "tilt_roll_pos_2deg", "touchdown"
            )
        )
        self.assertFalse(combination_is_applicable("B4", "heave_h2", "touchdown"))
        self.assertFalse(combination_is_applicable("B5", "constant02", "touchdown"))


if __name__ == "__main__":
    unittest.main()
