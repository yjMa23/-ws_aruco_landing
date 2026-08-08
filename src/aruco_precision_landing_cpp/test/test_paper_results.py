#!/usr/bin/env python3
"""paper finalization 论文统计与证据归档回归测试。"""

from __future__ import annotations

import importlib.util
import math
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = REPO_ROOT / "scripts" / "finalize_paper_results.py"
SPEC = importlib.util.spec_from_file_location("finalize_paper_results", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class WilsonIntervalTest(unittest.TestCase):
    def test_perfect_observations_have_non_degenerate_intervals(self) -> None:
        for executed in (10, 20, 40):
            lower, upper = MODULE.wilson_interval(executed, executed)
            self.assertGreater(lower, 0.0)
            self.assertLess(lower, 1.0)
            self.assertLessEqual(upper, 1.0)
            self.assertAlmostEqual(upper, 1.0, places=12)

    def test_zero_partial_and_illegal_counts(self) -> None:
        lower, upper = MODULE.wilson_interval(0, 10)
        self.assertAlmostEqual(lower, 0.0, places=12)
        self.assertGreater(upper, 0.0)
        lower, upper = MODULE.wilson_interval(4, 10)
        self.assertLess(lower, 0.4)
        self.assertGreater(upper, 0.4)
        for success, executed in ((-1, 10), (11, 10), (0, 0)):
            with self.assertRaises(ValueError):
                MODULE.wilson_interval(success, executed)
        with self.assertRaises(ValueError):
            MODULE.wilson_interval(1.5, 10)


class BootstrapTest(unittest.TestCase):
    def test_mean_bootstrap_is_reproducible(self) -> None:
        first = MODULE.bootstrap_mean_ci([1.0, 2.0, 4.0, 8.0], repetitions=500, seed=17)
        second = MODULE.bootstrap_mean_ci([1.0, 2.0, 4.0, 8.0], repetitions=500, seed=17)
        self.assertEqual(first, second)

    def test_nan_inf_empty_and_single_sample(self) -> None:
        empty = MODULE.summarize_continuous([], seed=1)
        self.assertEqual(empty["count"], 0)
        self.assertIsNone(empty["mean"])
        nonfinite = MODULE.summarize_continuous([math.nan, math.inf, -math.inf], seed=1)
        self.assertEqual(nonfinite["count"], 0)
        single = MODULE.summarize_continuous([2.5], seed=1)
        self.assertEqual(single["count"], 1)
        self.assertEqual(single["stddev"], 0.0)
        self.assertIsNone(single["mean_ci95_lower"])
        self.assertEqual(single["ci_unavailable_reason"], "fewer_than_2_finite_values")

    def test_independent_two_sample_difference(self) -> None:
        first = MODULE.bootstrap_difference_ci(
            [3.0, 4.0, 5.0, 6.0], [0.5, 1.0, 1.5], repetitions=1000, seed=23
        )
        second = MODULE.bootstrap_difference_ci(
            [3.0, 4.0, 5.0, 6.0], [0.5, 1.0, 1.5], repetitions=1000, seed=23
        )
        self.assertEqual(first, second)
        self.assertGreater(first[0], 0.0)
        self.assertGreater(first[1], 0.0)


class StatisticalSeparationTest(unittest.TestCase):
    @staticmethod
    def row(
        method: str,
        scenario: str,
        profile: str,
        *,
        success: bool = True,
        applicability: str = "APPLICABLE",
        value: float = 0.1,
    ) -> dict[str, object]:
        return {
            "method": method,
            "scenario": scenario,
            "profile": profile,
            "success_bool": success,
            "applicability": applicability,
            "horizontal_error_rmse_m": value,
        }

    def test_not_applicable_does_not_enter_denominator(self) -> None:
        rows = [
            self.row("B0", "constant02", "safe-altitude", success=True),
            self.row("B2", "constant02", "safe-altitude", success=False, applicability="NOT_APPLICABLE"),
        ]
        filtered = MODULE.filter_executed_rows(rows)
        self.assertEqual(len(filtered), 1)
        self.assertEqual(filtered[0]["method"], "B0")

    def test_smoke_failure_is_not_mixed_into_formal_success(self) -> None:
        formal = [self.row("B0", "static", "touchdown", success=True)]
        smoke = [self.row("B2", "constant02", "safe-altitude", success=False)]
        intervals = MODULE.compute_success_intervals(formal, smoke)
        formal_overall = next(
            row for row in intervals if row["dataset"] == "formal" and row["level"] == "overall"
        )
        smoke_overall = next(
            row for row in intervals if row["dataset"] == "smoke" and row["level"] == "overall"
        )
        self.assertEqual((formal_overall["success"], formal_overall["executed"]), (1, 1))
        self.assertEqual((smoke_overall["success"], smoke_overall["executed"]), (0, 1))

    def test_profile_filter_prevents_baseline_touchdown_contamination(self) -> None:
        rows = [
            self.row("B0", "constant02", "touchdown", value=9.0),
            self.row("B0", "constant02", "safe-altitude", value=1.0),
            self.row("B0", "constant02", "safe-altitude", value=2.0),
        ]
        values = MODULE.metric_values(
            rows,
            "B0",
            "constant02",
            "horizontal_error_rmse_m",
            profile="safe-altitude",
        )
        self.assertEqual(values, [1.0, 2.0])

    def test_method_comparisons_use_unpaired_safe_altitude_samples(self) -> None:
        rows = []
        for method, scenario, values in (
            ("B0", "constant02", [1.0, 1.2]),
            ("B1", "constant02", [1.4, 1.6]),
            ("B3", "constant02", [0.7, 0.9]),
            ("B0", "sinusoidal", [3.0, 3.2]),
            ("B3", "sinusoidal", [2.0, 2.2]),
        ):
            rows.extend(
                self.row(method, scenario, "safe-altitude", value=value) for value in values
            )
        rows.append(self.row("B0", "constant02", "touchdown", value=100.0))
        comparisons = MODULE.compute_method_comparisons(rows)
        self.assertEqual(len(comparisons), 3)
        for row in comparisons:
            self.assertEqual(row["sample_count_a"], 2)
            self.assertEqual(row["sample_count_b"], 2)
        b1 = next(row for row in comparisons if row["method_a"] == "B1")
        self.assertAlmostEqual(b1["mean_b"], 1.1)


class ValidationFailureTest(unittest.TestCase):
    def test_wrong_commit_is_rejected(self) -> None:
        batch = {
            "dataset": "baseline",
            "rows": [{"git_commit": "wrong", "dirty_bool": False}],
            "manifest": {
                "git_commit_at_start": MODULE.FORMAL_SIMULATION_COMMIT,
                "dirty_worktree_at_start": False,
            },
        }
        with self.assertRaisesRegex(ValueError, "simulation commit mismatch"):
            MODULE.validate_formal_batch_commit(batch)

    def test_missing_file_and_corrupt_json_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaisesRegex(ValueError, "missing required files"):
                MODULE.validate_required_files(root)
            corrupt = root / "corrupt.json"
            corrupt.write_text("{not-json", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "corrupt JSON"):
                MODULE.read_json_strict(corrupt)

    def test_historical_excluded_batch_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "excluded historical batches"):
            MODULE.validate_input_paths_not_excluded(
                [REPO_ROOT / "results" / "paper_baseline_20x20_20260803"], REPO_ROOT
            )


class OutputSafetyTest(unittest.TestCase):
    def test_latex_special_characters_are_escaped(self) -> None:
        escaped = MODULE.latex_escape("B_5 & 100% #1 $x$ {ok} ~ ^")
        for token in (r"\_", r"\&", r"\%", r"\#", r"\$", r"\{", r"\}"):
            self.assertIn(token, escaped)

    def test_output_must_not_contain_home_j(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "safe.txt").write_text("results/paper_evaluation", encoding="utf-8")
            MODULE.assert_no_absolute_home(root)
            (root / "bad.txt").write_text("/home/j/ws_aruco_landing", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "hard-coded /home/j"):
                MODULE.assert_no_absolute_home(root)

    def test_sha256_and_aggregate_are_reproducible(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "a.json"
            second = root / "b.json"
            first.write_text('{"a": 1}\n', encoding="utf-8")
            second.write_text('{"b": 2}\n', encoding="utf-8")
            self.assertEqual(MODULE.sha256_file(first), MODULE.sha256_file(first))
            aggregate_a = MODULE.aggregate_file_hash([second, first], root)
            aggregate_b = MODULE.aggregate_file_hash([first, second], root)
            self.assertEqual(aggregate_a, aggregate_b)


if __name__ == "__main__":
    unittest.main(verbosity=2)
