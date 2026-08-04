#!/usr/bin/env python3
"""冻结 P9 输入并生成论文统计、表格、图表与可复现实验证据清单。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import random
import shutil
import statistics
import subprocess
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from statistics import NormalDist
from typing import Any, Callable, Iterable, Mapping, Sequence

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from p7_experiment_utils import METRIC_FIELDS  # noqa: E402

FORMAL_SIMULATION_COMMIT = "71af1cc897136265a999c83dd6034bf156a32a50"
AGGREGATION_FIX_COMMIT = "fc979fafc37f05fe4ae690e884153482a14d3c07"
P9_FINAL_DOCUMENT_COMMIT = "b20c8c9186e5869242417bd6c9539f1c0d97f54f"
BOOTSTRAP_REPETITIONS = 10_000
BOOTSTRAP_SEED = 20_260_804
CONFIDENCE_LEVEL = 0.95
OUTPUT_VERSION = "p9_paper_results_v0.1"

REQUIRED_BATCH_FILES = (
    "batch_manifest.json",
    "summary.json",
    "episodes.csv",
    "experiment_matrix.csv",
)
ROOT_STRUCTURED_FILES = (
    "batch_manifest.json",
    "summary.json",
    "summary.csv",
    "episodes.csv",
    "failures.csv",
    "by_method.csv",
    "by_scenario.csv",
    "by_method_scenario.csv",
    "experiment_matrix.csv",
)
EPISODE_STRUCTURED_FILES = (
    "manifest.json",
    "evaluation.json",
    "method_parameters.yaml",
    "controller_config.yaml",
    "scenario_config.yaml",
)
EXCLUDED_BATCHES = {
    "results/p9_baseline_20x20_20260803": "interrupted pre-freeze batch; only 4/40 completed",
    "results/p9_baseline_20x20_20260803_a9d011d": "batch orchestration contamination evidence",
    "results/p9_baseline_20x20_20260803_a9d011d_clean1": "complete pre-clock-fix baseline on superseded simulation commit",
    "results/p9_ablation_20260804_a9d011d": "SYSTEM_TIME/ROS_TIME clock-source defect evidence",
}
CLOSED_COMBINATIONS = {
    ("B2", "constant02", "safe-altitude"),
    ("B4", "heave_h1", "touchdown"),
    ("B5", "tilt_pitch_pos_2deg", "touchdown"),
}

SUCCESS_CSV_FIELDS = (
    "dataset",
    "level",
    "group",
    "method",
    "scenario",
    "profile",
    "success",
    "executed",
    "point_estimate",
    "ci95_lower",
    "ci95_upper",
    "interval_method",
)
CONTINUOUS_CSV_FIELDS = (
    "dataset",
    "level",
    "group",
    "method",
    "scenario",
    "profile",
    "metric",
    "count",
    "mean",
    "stddev",
    "median",
    "p95",
    "min",
    "max",
    "mean_ci95_lower",
    "mean_ci95_upper",
    "bootstrap_method",
    "bootstrap_repetitions",
    "bootstrap_seed",
    "confidence_level",
    "ci_unavailable_reason",
)
COMPARISON_CSV_FIELDS = (
    "method_a",
    "method_b",
    "scenario",
    "metric",
    "mean_a",
    "mean_b",
    "absolute_difference",
    "relative_difference_percent",
    "bootstrap_difference_ci95_lower",
    "bootstrap_difference_ci95_upper",
    "sample_count_a",
    "sample_count_b",
    "bootstrap_method",
    "bootstrap_repetitions",
    "bootstrap_seed",
    "confidence_level",
    "interpretation",
)

METRIC_LABELS = {
    "landing_time_s": "Landing time (s)",
    "horizontal_error_rmse_m": "Horizontal RMSE (m)",
    "horizontal_error_max_m": "Horizontal max error (m)",
    "touchdown_vertical_speed_mps": "Touchdown vertical speed (m/s)",
    "candidate_to_confirm_delay_s": "Candidate-to-confirm delay (s)",
    "hold_duration_s": "Touchdown hold duration (s)",
    "recovery_count": "Recovery count",
    "marker_switch_count": "Marker switch count",
    "detach_count": "Detach count",
    "secondary_contact_count": "Secondary contact count",
    "candidate_repeat_count": "Candidate repeat count",
    "deck_vertical_span_final_m": "Final deck vertical span (m)",
    "hold_relative_height_span_m": "Hold relative-height span (m)",
    "hold_relative_vertical_velocity_p95_mps": "Hold relative vertical velocity P95 (m/s)",
    "normal_tracking_error_rmse_deg": "Normal tracking RMSE (deg)",
    "normal_tracking_error_p95_deg": "Normal tracking P95 (deg)",
    "terminal_command_tilt_max_deg": "Terminal command tilt max (deg)",
    "terminal_command_tilt_slew_p100_degps": "Terminal tilt slew max (deg/s)",
    "combined_horizontal_acceleration_max_mps2": "Horizontal acceleration max (m/s^2)",
    "touchdown_slip_m": "Touchdown slip (m)",
    "hold_tangential_velocity_p95_mps": "Hold tangential velocity P95 (m/s)",
    "attitude_divergence_increment_deg": "Attitude divergence increment (deg)",
    "fallback_count": "Fallback count",
    "terminal_stabilization_activation_count": "Terminal stabilization activations",
    "solver_success_rate": "Solver success rate",
    "solver_failure_count": "Solver failure count",
    "solve_time_mean_ms": "MPC solve time mean (ms)",
    "solve_time_p95_ms": "MPC solve time P95 (ms)",
    "constraint_violation_count": "Constraint violation count",
}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--smoke", required=True, type=Path)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--ablation", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def frozen_source_timestamp(batches: Sequence[Mapping[str, Any]]) -> str:
    timestamps: list[datetime] = []
    for batch in batches:
        raw = batch["manifest"].get("finished_at") or batch["manifest"].get("updated_at")
        if raw is None:
            raise ValueError(f"{batch['dataset']} manifest lacks finished_at/updated_at")
        try:
            timestamps.append(datetime.fromisoformat(str(raw).replace("Z", "+00:00")))
        except ValueError as error:
            raise ValueError(f"invalid frozen batch timestamp for {batch['dataset']}: {raw}") from error
    return max(timestamps).astimezone(timezone.utc).replace(microsecond=0).isoformat()


def read_json_strict(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"missing JSON file: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"corrupt JSON file {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise ValueError(f"missing CSV file: {path}")
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            return list(csv.DictReader(handle))
    except (OSError, csv.Error) as error:
        raise ValueError(f"corrupt CSV file {path}: {error}") from error


def parse_bool(value: Any, label: str) -> bool:
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in {"true", "1", "yes"}:
        return True
    if normalized in {"false", "0", "no"}:
        return False
    raise ValueError(f"{label} must be boolean, got {value!r}")


def finite_float(value: Any) -> float | None:
    if value is None or str(value).strip() == "":
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        raise ValueError("percentile requires at least one value")
    if not 0.0 <= probability <= 1.0:
        raise ValueError("probability must be within [0, 1]")
    ordered = sorted(float(value) for value in values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def wilson_interval(success: int, executed: int, confidence: float = CONFIDENCE_LEVEL) -> tuple[float, float]:
    if not isinstance(success, int) or not isinstance(executed, int):
        raise ValueError("success and executed must be integers")
    if executed <= 0:
        raise ValueError("executed must be positive")
    if success < 0 or success > executed:
        raise ValueError("success must satisfy 0 <= success <= executed")
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must be within (0, 1)")
    estimate = success / executed
    z = NormalDist().inv_cdf(0.5 + confidence / 2.0)
    z2 = z * z
    denominator = 1.0 + z2 / executed
    center = (estimate + z2 / (2.0 * executed)) / denominator
    radius = z * math.sqrt(
        estimate * (1.0 - estimate) / executed + z2 / (4.0 * executed * executed)
    ) / denominator
    return max(0.0, center - radius), min(1.0, center + radius)


def deterministic_seed(*parts: Any) -> int:
    payload = "|".join(str(part) for part in (BOOTSTRAP_SEED, *parts)).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")


def bootstrap_mean_ci(
    values: Sequence[float],
    *,
    repetitions: int = BOOTSTRAP_REPETITIONS,
    seed: int = BOOTSTRAP_SEED,
    confidence: float = CONFIDENCE_LEVEL,
) -> tuple[float | None, float | None, str | None]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if len(finite) < 2:
        return None, None, "fewer_than_2_finite_values"
    if repetitions <= 0:
        raise ValueError("bootstrap repetitions must be positive")
    if not 0.0 < confidence < 1.0:
        raise ValueError("confidence must be within (0, 1)")
    if min(finite) == max(finite):
        return finite[0], finite[0], None
    rng = random.Random(seed)
    count = len(finite)
    means = [
        sum(finite[rng.randrange(count)] for _ in range(count)) / count
        for _ in range(repetitions)
    ]
    alpha = (1.0 - confidence) / 2.0
    return percentile(means, alpha), percentile(means, 1.0 - alpha), None


def bootstrap_difference_ci(
    values_a: Sequence[float],
    values_b: Sequence[float],
    *,
    repetitions: int = BOOTSTRAP_REPETITIONS,
    seed: int = BOOTSTRAP_SEED,
    confidence: float = CONFIDENCE_LEVEL,
) -> tuple[float | None, float | None, str | None]:
    finite_a = [float(value) for value in values_a if math.isfinite(float(value))]
    finite_b = [float(value) for value in values_b if math.isfinite(float(value))]
    if len(finite_a) < 2 or len(finite_b) < 2:
        return None, None, "fewer_than_2_finite_values_in_one_or_both_samples"
    if repetitions <= 0:
        raise ValueError("bootstrap repetitions must be positive")
    rng = random.Random(seed)
    count_a = len(finite_a)
    count_b = len(finite_b)
    differences: list[float] = []
    for _ in range(repetitions):
        mean_a = sum(finite_a[rng.randrange(count_a)] for _ in range(count_a)) / count_a
        mean_b = sum(finite_b[rng.randrange(count_b)] for _ in range(count_b)) / count_b
        differences.append(mean_a - mean_b)
    alpha = (1.0 - confidence) / 2.0
    return percentile(differences, alpha), percentile(differences, 1.0 - alpha), None


def summarize_continuous(values: Sequence[float], *, seed: int) -> dict[str, Any]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0,
            "mean": None,
            "stddev": None,
            "median": None,
            "p95": None,
            "min": None,
            "max": None,
            "mean_ci95_lower": None,
            "mean_ci95_upper": None,
            "ci_unavailable_reason": "no_finite_values",
        }
    lower, upper, reason = bootstrap_mean_ci(finite, seed=seed)
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "stddev": statistics.pstdev(finite),
        "median": statistics.median(finite),
        "p95": percentile(finite, 0.95),
        "min": min(finite),
        "max": max(finite),
        "mean_ci95_lower": lower,
        "mean_ci95_upper": upper,
        "ci_unavailable_reason": reason,
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def aggregate_file_hash(paths: Iterable[Path], root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix().encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        digest.update(sha256_file(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def latex_escape(value: Any) -> str:
    text = str(value)
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(character, character) for character in text)


def repository_relative(path: Path, repo_root: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return path.name


def resolve_manifest_path(raw_path: Any, repo_root: Path) -> str | None:
    if raw_path is None or str(raw_path).strip() == "":
        return None
    path = Path(str(raw_path))
    if path.is_absolute():
        try:
            return path.resolve().relative_to(repo_root.resolve()).as_posix()
        except ValueError:
            return path.name
    return path.as_posix()


def validate_required_files(batch_dir: Path) -> None:
    if not batch_dir.is_dir():
        raise ValueError(f"batch directory does not exist: {batch_dir}")
    missing = [name for name in REQUIRED_BATCH_FILES if not (batch_dir / name).is_file()]
    if missing:
        raise ValueError(f"batch {batch_dir} is missing required files: {', '.join(missing)}")


def load_batch(batch_dir: Path, dataset: str) -> dict[str, Any]:
    validate_required_files(batch_dir)
    manifest = read_json_strict(batch_dir / "batch_manifest.json")
    summary = read_json_strict(batch_dir / "summary.json")
    matrix = read_csv_rows(batch_dir / "experiment_matrix.csv")
    rows = read_csv_rows(batch_dir / "episodes.csv")
    matrix_map: dict[tuple[str, str, str], str] = {}
    for row in matrix:
        key = (row.get("method", ""), row.get("scenario", ""), row.get("profile", ""))
        matrix_map[key] = row.get("applicability", "")
    seen_ids: set[str] = set()
    normalized_rows: list[dict[str, Any]] = []
    for row in rows:
        episode_id = row.get("episode_id", "")
        if not episode_id or episode_id in seen_ids:
            raise ValueError(f"duplicate or empty episode_id in {batch_dir}: {episode_id!r}")
        seen_ids.add(episode_id)
        key = (row.get("method", ""), row.get("scenario", ""), row.get("profile", ""))
        applicability = matrix_map.get(key)
        if applicability != "APPLICABLE":
            raise ValueError(f"executed episode has non-applicable or missing matrix entry: {key}")
        success = parse_bool(row.get("success"), f"{episode_id}.success")
        dirty = parse_bool(row.get("dirty_worktree"), f"{episode_id}.dirty_worktree")
        episode_dir = batch_dir / episode_id
        episode_manifest_path = episode_dir / "manifest.json"
        evaluation_path = episode_dir / "evaluation.json"
        episode_manifest = read_json_strict(episode_manifest_path)
        read_json_strict(evaluation_path)
        if episode_manifest.get("completed") is not True:
            raise ValueError(f"episode manifest is not completed: {episode_manifest_path}")
        if bool(episode_manifest.get("success")) != success:
            raise ValueError(f"episode success mismatch between CSV and manifest: {episode_id}")
        normalized: dict[str, Any] = dict(row)
        normalized.update(
            {
                "dataset": dataset,
                "applicability": applicability,
                "success_bool": success,
                "dirty_bool": dirty,
                "episode_directory": episode_dir,
            }
        )
        normalized_rows.append(normalized)
    for path in batch_dir.rglob("evaluation.json"):
        read_json_strict(path)
    for path in batch_dir.rglob("manifest.json"):
        read_json_strict(path)
    return {
        "dataset": dataset,
        "path": batch_dir,
        "manifest": manifest,
        "summary": summary,
        "matrix": matrix,
        "rows": normalized_rows,
    }


def validate_repo_commits(repo_root: Path) -> None:
    for commit in (FORMAL_SIMULATION_COMMIT, AGGREGATION_FIX_COMMIT, P9_FINAL_DOCUMENT_COMMIT):
        result = subprocess.run(
            ["git", "cat-file", "-e", f"{commit}^{{commit}}"],
            cwd=repo_root,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if result.returncode != 0:
            raise ValueError(f"required commit is absent from repository: {commit}")
    ancestry = subprocess.run(
        ["git", "merge-base", "--is-ancestor", P9_FINAL_DOCUMENT_COMMIT, "HEAD"],
        cwd=repo_root,
        check=False,
    )
    if ancestry.returncode != 0:
        raise ValueError("HEAD does not contain the frozen P9 final document commit")


def validate_formal_batch_commit(batch: Mapping[str, Any]) -> None:
    commits = {row.get("git_commit") for row in batch["rows"]}
    if commits != {FORMAL_SIMULATION_COMMIT}:
        raise ValueError(f"{batch['dataset']} simulation commit mismatch: {sorted(commits)}")
    if any(row["dirty_bool"] for row in batch["rows"]):
        raise ValueError(f"{batch['dataset']} contains dirty-worktree episodes")
    if batch["manifest"].get("git_commit_at_start") != FORMAL_SIMULATION_COMMIT:
        raise ValueError(f"{batch['dataset']} batch manifest commit mismatch")
    if batch["manifest"].get("dirty_worktree_at_start") is not False:
        raise ValueError(f"{batch['dataset']} batch started from a dirty worktree")


def validate_input_paths_not_excluded(
    batch_paths: Sequence[Path], repo_root: Path
) -> None:
    explicit_inputs = {repository_relative(path, repo_root) for path in batch_paths}
    contamination = explicit_inputs.intersection(EXCLUDED_BATCHES)
    if contamination:
        raise ValueError(
            f"excluded historical batches were supplied as P10 inputs: {sorted(contamination)}"
        )


def _summary_counts(batch: Mapping[str, Any]) -> tuple[int, int, int]:
    rows = batch["rows"]
    executed = len(rows)
    success = sum(bool(row["success_bool"]) for row in rows)
    return executed, success, executed - success


def validate_batch_counts(batch: Mapping[str, Any], expected: tuple[int, int, int]) -> None:
    executed, success, failure = _summary_counts(batch)
    if (executed, success, failure) != expected:
        raise ValueError(
            f"{batch['dataset']} expected executed/success/failure={expected}, "
            f"got {(executed, success, failure)}"
        )
    manifest = batch["manifest"]
    if manifest.get("completed") is not True:
        raise ValueError(f"{batch['dataset']} batch manifest is not completed")
    if int(manifest.get("planned_episodes", -1)) != executed:
        raise ValueError(f"{batch['dataset']} planned_episodes mismatch")
    if int(manifest.get("completed_episodes", -1)) != executed:
        raise ValueError(f"{batch['dataset']} completed_episodes mismatch")
    overall = batch["summary"].get("overall", {})
    if int(overall.get("total_experiments", -1)) != executed:
        raise ValueError(f"{batch['dataset']} summary total mismatch")
    if int(overall.get("success_count", -1)) != success:
        raise ValueError(f"{batch['dataset']} summary success mismatch")
    if int(overall.get("failure_count", -1)) != failure:
        raise ValueError(f"{batch['dataset']} summary failure mismatch")


def validate_input_consistency(
    smoke: Mapping[str, Any],
    baseline: Mapping[str, Any],
    ablation: Mapping[str, Any],
    repo_root: Path,
) -> dict[str, Any]:
    validate_repo_commits(repo_root)
    validate_batch_counts(smoke, (27, 20, 7))
    validate_batch_counts(baseline, (40, 40, 0))
    validate_batch_counts(ablation, (60, 60, 0))

    smoke_failures = [row for row in smoke["rows"] if not row["success_bool"]]
    if {row.get("failure_reason") for row in smoke_failures} != {"SAFETY_GATE_FAILURE"}:
        raise ValueError("all smoke failures must be SAFETY_GATE_FAILURE")
    failed_combinations = {
        (row.get("method"), row.get("scenario"), row.get("profile"))
        for row in smoke_failures
    }
    if failed_combinations != CLOSED_COMBINATIONS:
        raise ValueError(
            f"smoke closed combinations mismatch: expected {sorted(CLOSED_COMBINATIONS)}, "
            f"got {sorted(failed_combinations)}"
        )

    formal_na_rows = [row for row in ablation["matrix"] if row.get("applicability") == "NOT_APPLICABLE"]
    na_slots = sum(int(row.get("repetitions", "0")) for row in formal_na_rows)
    na_combinations = {
        (row.get("method"), row.get("scenario"), row.get("profile"))
        for row in formal_na_rows
    }
    if na_slots != 30 or na_combinations != CLOSED_COMBINATIONS:
        raise ValueError(
            f"formal NOT_APPLICABLE matrix mismatch: slots={na_slots}, combinations={sorted(na_combinations)}"
        )

    for batch in (baseline, ablation):
        validate_formal_batch_commit(batch)

    all_rows = [*smoke["rows"], *baseline["rows"], *ablation["rows"]]
    nav_land = sum(int(float(row.get("nav_land_count") or 0)) for row in all_rows)
    disarm = sum(int(float(row.get("disarm_count") or 0)) for row in all_rows)
    if nav_land != 0 or disarm != 0:
        raise ValueError(f"NAV_LAND / Disarm must be 0 / 0, got {nav_land} / {disarm}")

    validate_input_paths_not_excluded(
        [smoke["path"], baseline["path"], ablation["path"]], repo_root
    )

    return {
        "status": "PASS",
        "smoke": {"executed": 27, "success": 20, "failure": 7},
        "baseline": {"executed": 40, "success": 40, "failure": 0},
        "ablation": {"executed": 60, "success": 60, "failure": 0},
        "formal_not_applicable_slots": 30,
        "closed_combinations": ["/".join(item) for item in sorted(CLOSED_COMBINATIONS)],
        "formal_simulation_commit": FORMAL_SIMULATION_COMMIT,
        "aggregation_fix_commit": AGGREGATION_FIX_COMMIT,
        "p9_final_document_commit": P9_FINAL_DOCUMENT_COMMIT,
        "nav_land_count": nav_land,
        "disarm_count": disarm,
        "excluded_batches_used": [],
    }


def filter_executed_rows(rows: Iterable[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    return [row for row in rows if row.get("applicability") == "APPLICABLE"]


def _group_rows(
    rows: Sequence[Mapping[str, Any]],
    key_fn: Callable[[Mapping[str, Any]], tuple[str, ...]],
) -> dict[tuple[str, ...], list[Mapping[str, Any]]]:
    groups: dict[tuple[str, ...], list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[key_fn(row)].append(row)
    return dict(sorted(groups.items()))


def _success_row(
    dataset: str,
    level: str,
    group: str,
    rows: Sequence[Mapping[str, Any]],
    *,
    method: str = "",
    scenario: str = "",
    profile: str = "",
) -> dict[str, Any]:
    executed = len(rows)
    if executed <= 0:
        raise ValueError(f"success interval group is empty: {dataset}/{level}/{group}")
    success = sum(bool(row["success_bool"]) for row in rows)
    lower, upper = wilson_interval(success, executed)
    return {
        "dataset": dataset,
        "level": level,
        "group": group,
        "method": method,
        "scenario": scenario,
        "profile": profile,
        "success": success,
        "executed": executed,
        "point_estimate": success / executed,
        "ci95_lower": lower,
        "ci95_upper": upper,
        "interval_method": "wilson",
    }


def compute_success_intervals(
    formal_rows: Sequence[Mapping[str, Any]], smoke_rows: Sequence[Mapping[str, Any]]
) -> list[dict[str, Any]]:
    formal = filter_executed_rows(formal_rows)
    smoke = filter_executed_rows(smoke_rows)
    output = [_success_row("formal", "overall", "all_formal_executed", formal)]
    for (method,), group in _group_rows(formal, lambda row: (str(row["method"]),)).items():
        output.append(_success_row("formal", "by_method", method, group, method=method))
    for (scenario,), group in _group_rows(formal, lambda row: (str(row["scenario"]),)).items():
        output.append(_success_row("formal", "by_scenario", scenario, group, scenario=scenario))
    for (method, scenario), group in _group_rows(
        formal, lambda row: (str(row["method"]), str(row["scenario"]))
    ).items():
        output.append(
            _success_row(
                "formal",
                "by_method_and_scenario",
                f"{method}/{scenario}",
                group,
                method=method,
                scenario=scenario,
            )
        )
    for (method, scenario, profile), group in _group_rows(
        formal,
        lambda row: (str(row["method"]), str(row["scenario"]), str(row["profile"])),
    ).items():
        output.append(
            _success_row(
                "formal",
                "by_method_scenario_profile",
                f"{method}/{scenario}/{profile}",
                group,
                method=method,
                scenario=scenario,
                profile=profile,
            )
        )
    output.append(_success_row("smoke", "overall", "all_smoke_executed", smoke))
    for (method, scenario, profile), group in _group_rows(
        smoke,
        lambda row: (str(row["method"]), str(row["scenario"]), str(row["profile"])),
    ).items():
        output.append(
            _success_row(
                "smoke",
                "by_method_scenario_profile",
                f"{method}/{scenario}/{profile}",
                group,
                method=method,
                scenario=scenario,
                profile=profile,
            )
        )
    return output


def _continuous_rows_for_groups(
    dataset: str,
    level: str,
    groups: Mapping[tuple[str, ...], Sequence[Mapping[str, Any]]],
    group_formatter: Callable[[tuple[str, ...]], str],
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    for key, rows in groups.items():
        successful = [row for row in rows if row["success_bool"]]
        method = key[0] if level in {"by_method", "by_method_and_scenario", "by_method_scenario_profile"} else ""
        scenario = ""
        profile = ""
        if level == "by_scenario":
            scenario = key[0]
        elif level == "by_method_and_scenario":
            scenario = key[1]
        elif level == "by_method_scenario_profile":
            scenario = key[1]
            profile = key[2]
        group_name = group_formatter(key)
        for metric in METRIC_FIELDS:
            values = [
                number
                for row in successful
                if (number := finite_float(row.get(metric))) is not None
            ]
            statistics_row = summarize_continuous(
                values,
                seed=deterministic_seed(dataset, level, group_name, metric),
            )
            output.append(
                {
                    "dataset": dataset,
                    "level": level,
                    "group": group_name,
                    "method": method,
                    "scenario": scenario,
                    "profile": profile,
                    "metric": metric,
                    **statistics_row,
                    "bootstrap_method": "nonparametric_percentile",
                    "bootstrap_repetitions": BOOTSTRAP_REPETITIONS,
                    "bootstrap_seed": BOOTSTRAP_SEED,
                    "confidence_level": CONFIDENCE_LEVEL,
                }
            )
    return output


def compute_continuous_intervals(
    formal_rows: Sequence[Mapping[str, Any]], smoke_rows: Sequence[Mapping[str, Any]]
) -> list[dict[str, Any]]:
    formal = filter_executed_rows(formal_rows)
    smoke = filter_executed_rows(smoke_rows)
    output: list[dict[str, Any]] = []
    output.extend(
        _continuous_rows_for_groups(
            "formal", "overall", {("all_formal_executed",): formal}, lambda key: key[0]
        )
    )
    output.extend(
        _continuous_rows_for_groups(
            "formal", "by_method", _group_rows(formal, lambda row: (str(row["method"]),)), lambda key: key[0]
        )
    )
    output.extend(
        _continuous_rows_for_groups(
            "formal", "by_scenario", _group_rows(formal, lambda row: (str(row["scenario"]),)), lambda key: key[0]
        )
    )
    output.extend(
        _continuous_rows_for_groups(
            "formal",
            "by_method_and_scenario",
            _group_rows(formal, lambda row: (str(row["method"]), str(row["scenario"]))),
            lambda key: "/".join(key),
        )
    )
    output.extend(
        _continuous_rows_for_groups(
            "formal",
            "by_method_scenario_profile",
            _group_rows(
                formal,
                lambda row: (str(row["method"]), str(row["scenario"]), str(row["profile"])),
            ),
            lambda key: "/".join(key),
        )
    )
    output.extend(
        _continuous_rows_for_groups(
            "smoke",
            "by_method_scenario_profile",
            _group_rows(
                smoke,
                lambda row: (str(row["method"]), str(row["scenario"]), str(row["profile"])),
            ),
            lambda key: "/".join(key),
        )
    )
    return output


def metric_values(
    rows: Sequence[Mapping[str, Any]],
    method: str,
    scenario: str,
    metric: str,
    *,
    profile: str | None = None,
) -> list[float]:
    return [
        value
        for row in rows
        if row.get("method") == method
        and row.get("scenario") == scenario
        and (profile is None or row.get("profile") == profile)
        and row.get("applicability") == "APPLICABLE"
        and row.get("success_bool") is True
        and (value := finite_float(row.get(metric))) is not None
    ]


def compute_method_comparisons(formal_rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    specifications = (
        ("B1", "B0", "constant02", "horizontal_error_rmse_m"),
        ("B3", "B0", "constant02", "horizontal_error_rmse_m"),
        ("B3", "B0", "sinusoidal", "horizontal_error_rmse_m"),
    )
    output: list[dict[str, Any]] = []
    for method_a, method_b, scenario, metric in specifications:
        values_a = metric_values(
            formal_rows, method_a, scenario, metric, profile="safe-altitude"
        )
        values_b = metric_values(
            formal_rows, method_b, scenario, metric, profile="safe-altitude"
        )
        if not values_a or not values_b:
            raise ValueError(f"missing formal values for comparison {method_a}-{method_b}/{scenario}/{metric}")
        mean_a = statistics.fmean(values_a)
        mean_b = statistics.fmean(values_b)
        difference = mean_a - mean_b
        relative = None if mean_b == 0.0 else difference / abs(mean_b) * 100.0
        lower, upper, reason = bootstrap_difference_ci(
            values_a,
            values_b,
            seed=deterministic_seed("comparison", method_a, method_b, scenario, metric),
        )
        if reason:
            interpretation = f"confidence interval unavailable: {reason}"
        elif lower is not None and upper is not None and (lower > 0.0 or upper < 0.0):
            interpretation = "差异方向得到当前样本支持"
        else:
            interpretation = "当前样本不足以确认差异方向"
        output.append(
            {
                "method_a": method_a,
                "method_b": method_b,
                "scenario": scenario,
                "metric": metric,
                "mean_a": mean_a,
                "mean_b": mean_b,
                "absolute_difference": difference,
                "relative_difference_percent": relative,
                "bootstrap_difference_ci95_lower": lower,
                "bootstrap_difference_ci95_upper": upper,
                "sample_count_a": len(values_a),
                "sample_count_b": len(values_b),
                "bootstrap_method": "independent_two_sample_nonparametric_percentile",
                "bootstrap_repetitions": BOOTSTRAP_REPETITIONS,
                "bootstrap_seed": BOOTSTRAP_SEED,
                "confidence_level": CONFIDENCE_LEVEL,
                "interpretation": interpretation,
            }
        )
    return output


def write_csv(path: Path, fieldnames: Sequence[str], rows: Sequence[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def format_number(value: Any, digits: int = 4) -> str:
    if value is None or value == "":
        return "—"
    if isinstance(value, (float, int)):
        return f"{float(value):.{digits}f}"
    return str(value)


def format_ci(lower: Any, upper: Any, digits: int = 3) -> str:
    if lower is None or upper is None:
        return "—"
    return f"[{float(lower):.{digits}f}, {float(upper):.{digits}f}]"


def success_lookup(rows: Sequence[Mapping[str, Any]]) -> dict[tuple[str, str, str, str], Mapping[str, Any]]:
    return {
        (str(row["dataset"]), str(row["method"]), str(row["scenario"]), str(row["profile"])): row
        for row in rows
        if row["level"] == "by_method_scenario_profile"
    }


def continuous_lookup(rows: Sequence[Mapping[str, Any]]) -> dict[tuple[str, str, str, str, str], Mapping[str, Any]]:
    return {
        (
            str(row["dataset"]),
            str(row["method"]),
            str(row["scenario"]),
            str(row["profile"]),
            str(row["metric"]),
        ): row
        for row in rows
        if row["level"] == "by_method_scenario_profile"
    }


def table_cell_metric(
    lookup: Mapping[tuple[str, str, str, str, str], Mapping[str, Any]],
    dataset: str,
    method: str,
    scenario: str,
    profile: str,
    metric: str,
) -> tuple[str, str, str]:
    row = lookup.get((dataset, method, scenario, profile, metric))
    if row is None or int(row["count"]) == 0:
        return "—", "—", "—"
    return (
        format_number(row["mean"]),
        format_ci(row["mean_ci95_lower"], row["mean_ci95_upper"]),
        format_number(row["p95"]),
    )


def write_table_triplet(
    tables_dir: Path,
    name: str,
    columns: Sequence[str],
    rows: Sequence[Mapping[str, Any]],
) -> None:
    write_csv(tables_dir / f"{name}.csv", columns, rows)
    markdown = [
        "| " + " | ".join(columns) + " |",
        "| " + " | ".join("---" if index == 0 else "---:" for index in range(len(columns))) + " |",
    ]
    for row in rows:
        markdown.append("| " + " | ".join(str(row.get(column, "")) for column in columns) + " |")
    (tables_dir / f"{name}.md").write_text("\n".join(markdown) + "\n", encoding="utf-8")

    latex_lines = [
        r"\begin{tabular}{" + "l" + "r" * (len(columns) - 1) + "}",
        r"\toprule",
        " & ".join(latex_escape(column) for column in columns) + r" \\",
        r"\midrule",
    ]
    for row in rows:
        latex_lines.append(" & ".join(latex_escape(row.get(column, "")) for column in columns) + r" \\")
    latex_lines.extend([r"\bottomrule", r"\end{tabular}"])
    (tables_dir / f"{name}.tex").write_text("\n".join(latex_lines) + "\n", encoding="utf-8")


def build_paper_tables(
    output_dir: Path,
    success_rows: Sequence[Mapping[str, Any]],
    continuous_rows: Sequence[Mapping[str, Any]],
    smoke_rows: Sequence[Mapping[str, Any]],
    ablation_matrix: Sequence[Mapping[str, Any]],
) -> list[str]:
    tables_dir = output_dir / "tables"
    tables_dir.mkdir(parents=True, exist_ok=True)
    success = success_lookup(success_rows)
    continuous = continuous_lookup(continuous_rows)

    def result_row(method: str, scenario: str, profile: str, dataset: str = "formal") -> dict[str, Any]:
        success_row = success[(dataset, method, scenario, profile)]
        rmse_mean, rmse_ci, rmse_p95 = table_cell_metric(
            continuous, dataset, method, scenario, profile, "horizontal_error_rmse_m"
        )
        landing_mean, landing_ci, _ = table_cell_metric(
            continuous, dataset, method, scenario, profile, "landing_time_s"
        )
        return {
            "Method": method,
            "Scenario": scenario,
            "Profile": profile,
            "N": success_row["executed"],
            "Success": f"{success_row['success']}/{success_row['executed']}",
            "Success 95% CI": format_ci(success_row["ci95_lower"], success_row["ci95_upper"]),
            "Horizontal RMSE mean (m)": rmse_mean,
            "Mean 95% CI (m)": rmse_ci,
            "Horizontal RMSE P95 (m)": rmse_p95,
            "Landing time mean (s)": landing_mean,
            "Landing time mean 95% CI (s)": landing_ci,
        }

    baseline_columns = (
        "Method", "Scenario", "Profile", "N", "Success", "Success 95% CI",
        "Horizontal RMSE mean (m)", "Mean 95% CI (m)", "Horizontal RMSE P95 (m)",
        "Landing time mean (s)", "Landing time mean 95% CI (s)",
    )
    write_table_triplet(
        tables_dir,
        "baseline_static_constant02",
        baseline_columns,
        [result_row("B0", "static", "touchdown"), result_row("B0", "constant02", "touchdown")],
    )

    ablation_columns = (
        "Method", "Scenario", "Profile", "Applicability", "N", "Success", "Success 95% CI",
        "Horizontal RMSE mean (m)", "Mean 95% CI (m)", "Horizontal RMSE P95 (m)",
    )
    constant_rows: list[dict[str, Any]] = []
    for method in ("B0", "B1", "B2", "B3"):
        if method == "B2":
            constant_rows.append({
                "Method": method,
                "Scenario": "constant02",
                "Profile": "safe-altitude",
                "Applicability": "NOT_APPLICABLE",
                "N": "—",
                "Success": "—",
                "Success 95% CI": "—",
                "Horizontal RMSE mean (m)": "—",
                "Mean 95% CI (m)": "—",
                "Horizontal RMSE P95 (m)": "—",
            })
        else:
            row = result_row(method, "constant02", "safe-altitude")
            row["Applicability"] = "APPLICABLE"
            constant_rows.append(row)
    write_table_triplet(tables_dir, "ablation_constant02", ablation_columns, constant_rows)

    sinusoidal_rows = []
    for method in ("B0", "B3"):
        row = result_row(method, "sinusoidal", "safe-altitude")
        row["Applicability"] = "APPLICABLE"
        sinusoidal_rows.append(row)
    write_table_triplet(tables_dir, "ablation_sinusoidal", ablation_columns, sinusoidal_rows)

    fixed = result_row("B5", "tilt_roll_pos_2deg", "touchdown")
    for metric, heading in (
        ("touchdown_slip_m", "Touchdown slip mean (m)"),
        ("normal_tracking_error_p95_deg", "Normal tracking P95 mean (deg)"),
        ("hold_tangential_velocity_p95_mps", "Hold tangential velocity P95 mean (m/s)"),
    ):
        mean, ci, p95 = table_cell_metric(
            continuous, "formal", "B5", "tilt_roll_pos_2deg", "touchdown", metric
        )
        fixed[heading] = mean
        fixed[f"{heading} 95% CI"] = ci
        fixed[f"{heading} sample P95"] = p95
    fixed_columns = tuple(fixed.keys())
    write_table_triplet(tables_dir, "fixed_t1_roll_touchdown", fixed_columns, [fixed])

    formal_matrix_status = {
        (row.get("method"), row.get("scenario"), row.get("profile")): row.get("applicability")
        for row in ablation_matrix
    }
    smoke_grouped = _group_rows(
        smoke_rows,
        lambda row: (str(row["method"]), str(row["scenario"]), str(row["profile"])),
    )
    smoke_table_rows = []
    for (method, scenario, profile), group in smoke_grouped.items():
        successes = sum(bool(row["success_bool"]) for row in group)
        failures = len(group) - successes
        failure_reasons = Counter(row.get("failure_reason") for row in group if not row["success_bool"])
        formal_status = formal_matrix_status.get((method, scenario, profile), "NOT_IN_FORMAL_MATRIX")
        smoke_table_rows.append({
            "Method": method,
            "Scenario": scenario,
            "Profile": profile,
            "Smoke N": len(group),
            "Smoke success": f"{successes}/{len(group)}",
            "Smoke failures": failures,
            "Failure reason": "; ".join(f"{key}:{value}" for key, value in sorted(failure_reasons.items())) or "NONE",
            "Formal applicability": formal_status,
        })
    smoke_columns = (
        "Method", "Scenario", "Profile", "Smoke N", "Smoke success", "Smoke failures",
        "Failure reason", "Formal applicability",
    )
    write_table_triplet(tables_dir, "smoke_safety_gate", smoke_columns, smoke_table_rows)

    safety_rows = [
        {"Item": "Formal executed episodes", "Value": 100, "Interpretation": "baseline 40 + ablation 60"},
        {"Item": "Formal NOT_APPLICABLE slots", "Value": 30, "Interpretation": "not executed and not failures"},
        {"Item": "Smoke executed episodes", "Value": 27, "Interpretation": "separate safety-gate evidence"},
        {"Item": "Smoke SAFETY_GATE_FAILURE", "Value": 7, "Interpretation": "not mixed into formal success rate"},
        {"Item": "NAV_LAND commands", "Value": 0, "Interpretation": "disabled"},
        {"Item": "Automatic Disarm commands", "Value": 0, "Interpretation": "disabled"},
    ]
    write_table_triplet(
        tables_dir,
        "safety_and_applicability",
        ("Item", "Value", "Interpretation"),
        safety_rows,
    )
    return [
        "baseline_static_constant02",
        "ablation_constant02",
        "ablation_sinusoidal",
        "fixed_t1_roll_touchdown",
        "smoke_safety_gate",
        "safety_and_applicability",
    ]


def save_figure(figure: Any, plots_dir: Path, name: str) -> list[str]:
    plots_dir.mkdir(parents=True, exist_ok=True)
    generated: list[str] = []
    for extension in ("png", "pdf", "svg"):
        path = plots_dir / f"{name}.{extension}"
        metadata: dict[str, Any] | None = None
        if extension == "pdf":
            metadata = {"Creator": "finalize_p9_paper_results.py", "CreationDate": None, "ModDate": None}
        elif extension == "svg":
            metadata = {"Creator": "finalize_p9_paper_results.py", "Date": None}
        figure.savefig(path, dpi=300 if extension == "png" else None, bbox_inches="tight", metadata=metadata)
        if extension == "svg":
            normalized = "\n".join(
                line.rstrip() for line in path.read_text(encoding="utf-8").splitlines()
            ) + "\n"
            path.write_text(normalized, encoding="utf-8")
        generated.append(path.relative_to(plots_dir.parent).as_posix())
    return generated


def generate_plots(
    output_dir: Path,
    formal_rows: Sequence[Mapping[str, Any]],
    smoke_rows: Sequence[Mapping[str, Any]],
    success_rows: Sequence[Mapping[str, Any]],
    ablation_matrix: Sequence[Mapping[str, Any]],
) -> list[str]:
    try:
        import matplotlib as mpl
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError(f"matplotlib is required for P10 paper plots: {error}") from error

    mpl.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
        "legend.fontsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "svg.hashsalt": "p9-paper-results-v0.1",
    })
    plots_dir = output_dir / "plots"
    generated: list[str] = []

    def boxplot(name: str, title: str, labels: Sequence[str], values: Sequence[Sequence[float]], ylabel: str) -> None:
        if any(not group for group in values):
            raise ValueError(f"plot {name} has an empty data group")
        figure, axis = plt.subplots(figsize=(7.2, 4.6))
        axis.boxplot(values, labels=[f"{label}\n(n={len(group)})" for label, group in zip(labels, values)], showmeans=True)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.set_ylim(bottom=0.0)
        axis.grid(axis="y", alpha=0.25)
        figure.tight_layout()
        generated.extend(save_figure(figure, plots_dir, name))
        plt.close(figure)

    boxplot(
        "baseline_horizontal_rmse_distribution",
        "Baseline horizontal RMSE distribution",
        ("Static", "Constant 0.2 m/s"),
        (
            metric_values(
                formal_rows,
                "B0",
                "static",
                "horizontal_error_rmse_m",
                profile="touchdown",
            ),
            [
                value for row in formal_rows
                if row.get("method") == "B0" and row.get("scenario") == "constant02"
                and row.get("profile") == "touchdown" and row.get("success_bool")
                and (value := finite_float(row.get("horizontal_error_rmse_m"))) is not None
            ],
        ),
        "Horizontal RMSE (m)",
    )
    boxplot(
        "constant02_b0_b1_b3_horizontal_rmse",
        "Constant 0.2 m/s method comparison",
        ("B0", "B1", "B3"),
        tuple(
            metric_values(
                formal_rows,
                method,
                "constant02",
                "horizontal_error_rmse_m",
                profile="safe-altitude",
            )
            for method in ("B0", "B1", "B3")
        ),
        "Horizontal RMSE (m)",
    )
    boxplot(
        "sinusoidal_b0_b3_horizontal_rmse",
        "Sinusoidal method comparison",
        ("B0", "B3"),
        tuple(
            metric_values(
                formal_rows,
                method,
                "sinusoidal",
                "horizontal_error_rmse_m",
                profile="safe-altitude",
            )
            for method in ("B0", "B3")
        ),
        "Horizontal RMSE (m)",
    )

    combo_success = [
        row for row in success_rows
        if row["dataset"] == "formal" and row["level"] == "by_method_scenario_profile"
    ]
    figure, axis = plt.subplots(figsize=(10.5, 5.2))
    positions = list(range(len(combo_success)))
    estimates = [float(row["point_estimate"]) for row in combo_success]
    lower_errors = [estimate - float(row["ci95_lower"]) for estimate, row in zip(estimates, combo_success)]
    upper_errors = [float(row["ci95_upper"]) - estimate for estimate, row in zip(estimates, combo_success)]
    labels = [f"{row['method']}\n{row['scenario']}\n{row['profile']}\n(n={row['executed']})" for row in combo_success]
    axis.errorbar(positions, estimates, yerr=[lower_errors, upper_errors], fmt="o", capsize=4)
    axis.set_xticks(positions, labels, rotation=25, ha="right")
    axis.set_ylim(0.0, 1.05)
    axis.set_ylabel("Observed success rate")
    axis.set_title("Formal combination success rate with Wilson 95% CI")
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    generated.extend(save_figure(figure, plots_dir, "formal_success_rate_wilson_ci"))
    plt.close(figure)

    boxplot(
        "mpc_solve_time",
        "MPC solve-time distribution",
        ("B3 constant02", "B3 sinusoidal"),
        (
            metric_values(
                formal_rows,
                "B3",
                "constant02",
                "solve_time_mean_ms",
                profile="safe-altitude",
            ),
            metric_values(
                formal_rows,
                "B3",
                "sinusoidal",
                "solve_time_mean_ms",
                profile="safe-altitude",
            ),
        ),
        "Episode mean solve time (ms)",
    )

    fixed_rows = [
        row for row in formal_rows
        if row.get("method") == "B5" and row.get("scenario") == "tilt_roll_pos_2deg"
        and row.get("profile") == "touchdown" and row.get("success_bool")
    ]
    slip = [value for row in fixed_rows if (value := finite_float(row.get("touchdown_slip_m"))) is not None]
    attitude = [value for row in fixed_rows if (value := finite_float(row.get("normal_tracking_error_p95_deg"))) is not None]
    figure, axes = plt.subplots(1, 2, figsize=(8.0, 4.4))
    axes[0].boxplot([slip], labels=[f"Slip\n(n={len(slip)})"], showmeans=True)
    axes[0].set_ylabel("Touchdown slip (m)")
    axes[0].set_ylim(bottom=0.0)
    axes[0].grid(axis="y", alpha=0.25)
    axes[1].boxplot([attitude], labels=[f"Normal error P95\n(n={len(attitude)})"], showmeans=True)
    axes[1].set_ylabel("Normal tracking error (deg)")
    axes[1].set_ylim(bottom=0.0)
    axes[1].grid(axis="y", alpha=0.25)
    figure.suptitle("Fixed T1 roll touchdown: slip and attitude tracking")
    figure.tight_layout()
    generated.extend(save_figure(figure, plots_dir, "fixed_t1_slip_and_attitude_tracking"))
    plt.close(figure)

    smoke_groups = _group_rows(
        smoke_rows,
        lambda row: (str(row["method"]), str(row["scenario"]), str(row["profile"])),
    )
    formal_status = {
        (row.get("method"), row.get("scenario"), row.get("profile")): row.get("applicability")
        for row in ablation_matrix
    }
    labels = [f"{method}/{scenario}/{profile}" for method, scenario, profile in smoke_groups]
    successes = [sum(bool(row["success_bool"]) for row in group) for group in smoke_groups.values()]
    failures = [len(group) - success for group, success in zip(smoke_groups.values(), successes)]
    figure, axis = plt.subplots(figsize=(10.2, 5.4))
    positions = list(range(len(labels)))
    axis.bar(positions, successes, label="Smoke success")
    axis.bar(positions, failures, bottom=successes, label="SAFETY_GATE_FAILURE")
    axis.set_xticks(positions, labels, rotation=35, ha="right")
    axis.set_ylim(0.0, max(len(group) for group in smoke_groups.values()) + 1.2)
    axis.set_ylabel("Executed smoke episodes")
    axis.set_title("Smoke safety gates and formal applicability")
    for position, key in enumerate(smoke_groups):
        status = formal_status.get(key, "not in formal matrix")
        axis.text(position, successes[position] + failures[position] + 0.12, "formal: N/A" if status == "NOT_APPLICABLE" else "formal: allowed", ha="center", va="bottom", fontsize=8, rotation=90)
    axis.legend()
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    generated.extend(save_figure(figure, plots_dir, "smoke_safety_gate_and_applicability"))
    plt.close(figure)
    return generated


def batch_provenance(batch: Mapping[str, Any], repo_root: Path) -> dict[str, Any]:
    batch_dir: Path = batch["path"]
    rows = batch["rows"]
    manifest = batch["manifest"]
    all_files = [path for path in batch_dir.rglob("*") if path.is_file()]
    evaluation_files = list(batch_dir.rglob("evaluation.json"))
    bag_dirs = [path for path in batch_dir.rglob("bag") if path.is_dir()]
    excluded = [
        {
            "method": row.get("method"),
            "scenario": row.get("scenario"),
            "profile": row.get("profile"),
            "repetitions": int(row.get("repetitions", "0")),
        }
        for row in batch["matrix"]
        if row.get("applicability") == "NOT_APPLICABLE"
    ]
    executed, success, failure = _summary_counts(batch)
    commits = sorted({str(row.get("git_commit")) for row in rows})
    dirty_states = sorted({bool(row.get("dirty_bool")) for row in rows})
    return {
        "batch_id": manifest.get("batch_id", batch_dir.name),
        "batch_path": repository_relative(batch_dir, repo_root),
        "git_commit": commits[0] if len(commits) == 1 else commits,
        "dirty_state": dirty_states[0] if len(dirty_states) == 1 else dirty_states,
        "config_path": resolve_manifest_path(manifest.get("config_path"), repo_root),
        "planned": int(manifest.get("planned_episodes", executed)),
        "completed": int(manifest.get("completed_episodes", executed)),
        "success": success,
        "failure": failure,
        "excluded_combinations": excluded,
        "manifest_sha256": sha256_file(batch_dir / "batch_manifest.json"),
        "summary_json_sha256": sha256_file(batch_dir / "summary.json"),
        "episodes_csv_sha256": sha256_file(batch_dir / "episodes.csv"),
        "experiment_matrix_csv_sha256": sha256_file(batch_dir / "experiment_matrix.csv"),
        "all_evaluation_json_count": len(evaluation_files),
        "all_evaluation_json_aggregate_sha256": aggregate_file_hash(evaluation_files, batch_dir),
        "statistical_episode_count": len(rows),
        "bag_directory_count": len(bag_dirs),
        "total_file_count": len(all_files),
        "total_bytes": sum(path.stat().st_size for path in all_files),
    }


def write_data_manifest(output_dir: Path, batches: Sequence[Mapping[str, Any]], repo_root: Path) -> dict[str, Any]:
    candidates: set[Path] = set()
    for batch in batches:
        batch_dir: Path = batch["path"]
        for name in ROOT_STRUCTURED_FILES:
            path = batch_dir / name
            if path.is_file():
                candidates.add(path)
        for name in EPISODE_STRUCTURED_FILES:
            candidates.update(path for path in batch_dir.rglob(name) if path.is_file())
    lines: list[str] = []
    for path in sorted(candidates, key=lambda item: repository_relative(item, repo_root)):
        lines.append(f"{sha256_file(path)}  {repository_relative(path, repo_root)}")
    manifest_path = output_dir / "DATA_MANIFEST.sha256"
    manifest_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "entry_count": len(lines),
        "manifest_sha256": sha256_file(manifest_path),
        "scope": "small structured files only; Bag SQLite payloads are counted by directory and size but not hashed",
    }


def build_markdown_summary(
    validation: Mapping[str, Any],
    success_rows: Sequence[Mapping[str, Any]],
    continuous_rows: Sequence[Mapping[str, Any]],
    comparisons: Sequence[Mapping[str, Any]],
) -> str:
    success = success_lookup(success_rows)
    continuous = continuous_lookup(continuous_rows)
    lines = [
        "# P9 Paper Results v0.1",
        "",
        "## Scope",
        "",
        "This report is generated only from the frozen P9 smoke, baseline, and formal-ablation structured evidence. No controller parameter, evaluator threshold, seed, or episode selection was changed, and no new SITL run was performed.",
        "",
        "## Frozen evidence checks",
        "",
        f"- Smoke: `{validation['smoke']['success']}/{validation['smoke']['executed']}`; all 7 failures are `SAFETY_GATE_FAILURE`.",
        f"- Baseline: `{validation['baseline']['success']}/{validation['baseline']['executed']}`.",
        f"- Formal ablation: `{validation['ablation']['success']}/{validation['ablation']['executed']}`.",
        f"- Formal `NOT_APPLICABLE` slots: `{validation['formal_not_applicable_slots']}`.",
        f"- NAV_LAND / automatic Disarm: `{validation['nav_land_count']} / {validation['disarm_count']}`.",
        f"- Formal simulation commit: `{validation['formal_simulation_commit']}`.",
        "",
        "## Formal observed success with Wilson 95% CI",
        "",
        "| Method | Scenario | Profile | Success | Observed rate | Wilson 95% CI |",
        "| --- | --- | --- | ---: | ---: | ---: |",
    ]
    for row in success_rows:
        if row["dataset"] != "formal" or row["level"] != "by_method_scenario_profile":
            continue
        lines.append(
            f"| {row['method']} | {row['scenario']} | {row['profile']} | "
            f"{row['success']}/{row['executed']} | {row['point_estimate']:.1%} | "
            f"{format_ci(row['ci95_lower'], row['ci95_upper'])} |"
        )
    lines.extend([
        "",
        "A `10/10`, `20/20`, or `40/40` observation is a finite-sample result; it does not imply that the true success probability is 100%.",
        "",
        "## Selected continuous metrics with bootstrap mean 95% CI",
        "",
        "| Method | Scenario | Profile | Metric | N | Mean | Mean 95% CI | P95 |",
        "| --- | --- | --- | --- | ---: | ---: | ---: | ---: |",
    ])
    selected = (
        ("B0", "static", "touchdown", "horizontal_error_rmse_m"),
        ("B0", "constant02", "touchdown", "horizontal_error_rmse_m"),
        ("B0", "constant02", "safe-altitude", "horizontal_error_rmse_m"),
        ("B1", "constant02", "safe-altitude", "horizontal_error_rmse_m"),
        ("B3", "constant02", "safe-altitude", "horizontal_error_rmse_m"),
        ("B0", "sinusoidal", "safe-altitude", "horizontal_error_rmse_m"),
        ("B3", "sinusoidal", "safe-altitude", "horizontal_error_rmse_m"),
        ("B3", "constant02", "safe-altitude", "solve_time_mean_ms"),
        ("B3", "sinusoidal", "safe-altitude", "solve_time_mean_ms"),
        ("B5", "tilt_roll_pos_2deg", "touchdown", "touchdown_slip_m"),
        ("B5", "tilt_roll_pos_2deg", "touchdown", "normal_tracking_error_p95_deg"),
    )
    for method, scenario, profile, metric in selected:
        row = continuous[("formal", method, scenario, profile, metric)]
        lines.append(
            f"| {method} | {scenario} | {profile} | {METRIC_LABELS[metric]} | {row['count']} | "
            f"{format_number(row['mean'])} | {format_ci(row['mean_ci95_lower'], row['mean_ci95_upper'])} | "
            f"{format_number(row['p95'])} |"
        )
    lines.extend([
        "",
        "Bootstrap settings: deterministic nonparametric percentile bootstrap, 10000 repetitions, fixed random seed 20260804, confidence level 0.95. Standard deviation keeps the existing P9 population-standard-deviation convention.",
        "",
        "## Independent-sample method differences",
        "",
        "| Comparison | Scenario | Mean A | Mean B | A-B | Difference 95% CI | Relative difference | Interpretation |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ])
    for row in comparisons:
        lines.append(
            f"| {row['method_a']} - {row['method_b']} | {row['scenario']} | "
            f"{format_number(row['mean_a'])} | {format_number(row['mean_b'])} | "
            f"{format_number(row['absolute_difference'])} | "
            f"{format_ci(row['bootstrap_difference_ci95_lower'], row['bootstrap_difference_ci95_upper'])} | "
            f"{format_number(row['relative_difference_percent'], 2)}% | {row['interpretation']} |"
        )
    lines.extend([
        "",
        "Different methods are treated as independent samples; seed numbers are not assumed to define paired observations.",
        "",
        "## Safety-gate and applicability interpretation",
        "",
        "The 7 smoke failures are preserved as method-capability safety-gate evidence. B2 constant02, B4 heave_h1, and B5 pitch +2 deg are closed for formal execution and represented by 30 `NOT_APPLICABLE` slots. They are not encoded as zero success, not counted as formal failures, and not hidden.",
        "",
        "B5 roll +2 deg is the only fixed-T1 formal touchdown combination. Its result cannot be extrapolated to negative tilt, dynamic roll/pitch, rollpitch, or combined motion. P8C-4 remains an Offboard-position terminal stabilization design, not PX4 attitude-setpoint control.",
        "",
        "Ground Truth is isolated to offline evaluation. NAV_LAND and automatic Disarm remain disabled with observed counts 0 / 0.",
        "",
        "## Limitations",
        "",
        "The formal combinations have 10 or 20 observations each, all from deterministic frozen scenario/seed sets in SITL. Wilson and bootstrap intervals quantify finite-sample uncertainty but do not cover simulator-to-real transfer, untested sea states, negative or dynamic deck attitude, or distribution shift. Bag payloads are not individually hashed in order to avoid rereading large SQLite files; structured evidence is fully hashed and Bag directories, file counts, and byte totals are recorded in provenance.",
    ])
    return "\n".join(lines) + "\n"


def assert_no_absolute_home(output_dir: Path) -> None:
    forbidden = b"/home/j"
    offenders = []
    for path in output_dir.rglob("*"):
        if path.is_file() and forbidden in path.read_bytes():
            offenders.append(path.relative_to(output_dir).as_posix())
    if offenders:
        raise ValueError(f"generated output contains hard-coded /home/j paths: {offenders}")


def main() -> int:
    arguments = parse_arguments()
    repo_root = SCRIPT_DIR.parent.resolve()
    smoke_path = arguments.smoke.resolve()
    baseline_path = arguments.baseline.resolve()
    ablation_path = arguments.ablation.resolve()
    output_dir = arguments.output.resolve()
    input_paths = (smoke_path, baseline_path, ablation_path)
    if any(output_dir == path or output_dir.is_relative_to(path) for path in input_paths):
        raise ValueError("output directory must not equal or be nested under an input batch")

    smoke = load_batch(smoke_path, "smoke")
    baseline = load_batch(baseline_path, "baseline")
    ablation = load_batch(ablation_path, "ablation")
    validation = validate_input_consistency(smoke, baseline, ablation, repo_root)
    batches = (smoke, baseline, ablation)
    source_frozen_at = frozen_source_timestamp(batches)

    formal_rows = [*baseline["rows"], *ablation["rows"]]
    smoke_rows = smoke["rows"]
    success_rows = compute_success_intervals(formal_rows, smoke_rows)
    continuous_rows = compute_continuous_intervals(formal_rows, smoke_rows)
    comparisons = compute_method_comparisons(formal_rows)

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "success_rate_confidence_intervals.csv", SUCCESS_CSV_FIELDS, success_rows)
    write_csv(
        output_dir / "continuous_metric_confidence_intervals.csv",
        CONTINUOUS_CSV_FIELDS,
        continuous_rows,
    )
    write_csv(output_dir / "method_comparisons.csv", COMPARISON_CSV_FIELDS, comparisons)

    tables = build_paper_tables(
        output_dir,
        success_rows,
        continuous_rows,
        smoke_rows,
        ablation["matrix"],
    )
    plots = generate_plots(
        output_dir,
        formal_rows,
        smoke_rows,
        success_rows,
        ablation["matrix"],
    )

    provenance = {
        "schema_version": "1.0",
        "generated_at": source_frozen_at,
        "output_version": OUTPUT_VERSION,
        "input_policy": "frozen explicit P9 structured evidence only",
        "batches": [batch_provenance(batch, repo_root) for batch in (smoke, baseline, ablation)],
        "excluded_historical_batches": [
            {"batch_path": path, "reason": reason} for path, reason in EXCLUDED_BATCHES.items()
        ],
        "bag_hashing_limit": "Bag directories and total bytes are recorded, but large Bag SQLite payloads are not individually hashed.",
    }
    data_manifest = write_data_manifest(output_dir, batches, repo_root)
    provenance["data_manifest"] = data_manifest
    write_json(output_dir / "data_provenance.json", provenance)

    markdown = build_markdown_summary(validation, success_rows, continuous_rows, comparisons)
    (output_dir / "P9_PAPER_RESULTS.md").write_text(markdown, encoding="utf-8")
    paper_summary = {
        "schema_version": "1.0",
        "generated_at": source_frozen_at,
        "validation": validation,
        "statistics": {
            "success_interval_method": "wilson",
            "continuous_mean_interval_method": "deterministic nonparametric percentile bootstrap",
            "comparison_interval_method": "independent two-sample deterministic nonparametric percentile bootstrap",
            "bootstrap_repetitions": BOOTSTRAP_REPETITIONS,
            "bootstrap_seed": BOOTSTRAP_SEED,
            "confidence_level": CONFIDENCE_LEVEL,
            "population_standard_deviation": True,
        },
        "success_rate_confidence_intervals": success_rows,
        "method_comparisons": comparisons,
        "tables": tables,
        "plots": plots,
        "provenance_manifest": data_manifest,
    }
    write_json(output_dir / "paper_summary.json", paper_summary)
    assert_no_absolute_home(output_dir)
    print(json.dumps({
        "status": "PASS",
        "output": repository_relative(output_dir, repo_root),
        "formal_executed": 100,
        "smoke_executed": 27,
        "not_applicable_slots": 30,
        "tables": len(tables),
        "plot_files": len(plots),
        "data_manifest_entries": data_manifest["entry_count"],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError) as error:
        print(f"P10 finalization failed: {error}", file=sys.stderr)
        raise SystemExit(2)
