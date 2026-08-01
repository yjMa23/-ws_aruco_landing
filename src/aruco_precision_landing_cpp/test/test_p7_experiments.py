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
import run_batch_experiments  # noqa: E402
import run_single_experiment  # noqa: E402
from p7_experiment_utils import (  # noqa: E402
    classify_failure,
    episode_result_complete,
    expand_seeds,
    load_batch_config,
    make_batch_id,
    make_episode_id,
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

    def test_seed_expansion_rejects_mismatch(self) -> None:
        with self.assertRaises(ValueError):
            expand_seeds(3, [1, 2])

    def test_episode_ids_are_unique(self) -> None:
        batch_id = make_batch_id("smoke", datetime(2026, 7, 29, 12, 0, 0))
        first = make_episode_id(batch_id, "static", 1, 1)
        second = make_episode_id(batch_id, "static", 2, 2)
        third = make_episode_id(batch_id, "constant02", 1, 1)
        self.assertEqual(len({first, second, third}), 3)

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


if __name__ == "__main__":
    unittest.main()
