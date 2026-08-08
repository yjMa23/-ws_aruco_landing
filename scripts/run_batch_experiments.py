#!/usr/bin/env python3
"""顺序运行批量实验。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

from experiment_utils import (
    SUPPORTED_METHODS,
    SUPPORTED_SCENARIOS,
    atomic_write_json,
    episode_result_complete,
    load_batch_config,
    make_batch_id,
    make_episode_id,
    read_json,
    utc_now_iso,
    write_csv,
)

EPISODE_CSV_FIELDS = (
    "episode_id",
    "batch_id",
    "method",
    "scenario",
    "profile",
    "repetition",
    "seed",
    "applicability",
    "success",
    "failure_reason",
    "duration_s",
    "git_commit",
    "dirty_worktree",
    "bag_path",
    "evaluation_path",
)
MATRIX_CSV_FIELDS = (
    "method",
    "scenario",
    "profile",
    "applicability",
    "repetitions",
    "seeds",
    "tracking_mode",
    "prediction_horizon_s",
    "velocity_feedforward_gain",
    "vertical_velocity_feedforward_enabled",
    "terminal_stabilization_mode",
    "evaluator",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a sequential experiment batch.")
    parser.add_argument("config", type=Path, help="Experiment YAML configuration")
    parser.add_argument("--batch-id", help="Explicit batch ID; required to resume a known batch")
    parser.add_argument("--resume", action="store_true", help="Resume a known batch")
    parser.add_argument(
        "--retry-failures",
        action="store_true",
        help="with --resume, archive completed failed attempts and retry them once",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the expanded plan only")
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
    )
    parser.add_argument(
        "--single-runner",
        type=Path,
        default=Path(__file__).resolve().parent / "run_single_experiment.py",
        help="Single-episode runner; exposed for tests",
    )
    return parser.parse_args()


def git_state(workspace_dir: Path) -> tuple[str, bool]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=workspace_dir, check=True,
        text=True, capture_output=True
    ).stdout.strip()
    dirty = bool(subprocess.run(
        ["git", "status", "--porcelain"], cwd=workspace_dir, check=True,
        text=True, capture_output=True
    ).stdout.strip())
    return commit, dirty


def expanded_plan(config: Any, batch_id: str) -> list[dict[str, Any]]:
    plan: list[dict[str, Any]] = []
    for spec in config.experiments:
        if spec.applicability == "NOT_APPLICABLE":
            continue
        for repetition, seed in enumerate(spec.seeds, start=1):
            plan.append(
                {
                    "episode_id": make_episode_id(
                        batch_id,
                        spec.scenario,
                        repetition,
                        seed,
                        method=spec.method,
                        profile=spec.profile,
                    ),
                    "batch_id": batch_id,
                    "method": spec.method,
                    "scenario": spec.scenario,
                    "profile": spec.profile,
                    "repetition": repetition,
                    "seed": seed,
                    "applicability": spec.applicability,
                    "tracking_mode": spec.tracking_mode,
                    "prediction_horizon_s": spec.prediction_horizon_s,
                    "velocity_feedforward_gain": spec.velocity_feedforward_gain,
                    "vertical_velocity_feedforward_enabled": (
                        spec.vertical_velocity_feedforward_enabled
                    ),
                    "vertical_velocity_feedforward_gain": (
                        spec.vertical_velocity_feedforward_gain
                    ),
                    "vertical_velocity_feedforward_max_mps": (
                        spec.vertical_velocity_feedforward_max_mps
                    ),
                    "terminal_stabilization_mode": (
                        spec.terminal_stabilization_mode
                    ),
                    "evaluator": spec.evaluator,
                    "record_camera_debug": spec.record_camera_debug,
                    "parameter_overrides": dict(spec.parameter_overrides),
                }
            )
    return plan


def matrix_rows(config: Any) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for spec in config.experiments:
        rows.append(
            {
                "method": spec.method,
                "scenario": spec.scenario,
                "profile": spec.profile,
                "applicability": spec.applicability,
                "repetitions": spec.repetitions,
                "seeds": ";".join(str(seed) for seed in spec.seeds),
                "tracking_mode": spec.tracking_mode,
                "prediction_horizon_s": spec.prediction_horizon_s,
                "velocity_feedforward_gain": spec.velocity_feedforward_gain,
                "vertical_velocity_feedforward_enabled": (
                    spec.vertical_velocity_feedforward_enabled
                ),
                "terminal_stabilization_mode": spec.terminal_stabilization_mode,
                "evaluator": spec.evaluator,
            }
        )
    return rows


def build_single_command(
    *,
    single_runner: Path,
    workspace_dir: Path,
    batch_dir: Path,
    config: Any,
    episode: dict[str, Any],
    retry_existing_failure: bool = False,
    rerun_after_code_change: bool = False,
) -> list[str]:
    command = [
        sys.executable,
        str(single_runner),
        "--method",
        episode["method"],
        "--scenario",
        episode["scenario"],
        "--seed",
        str(episode["seed"]),
        "--episode-timeout",
        str(config.episode_timeout_s),
        "--startup-timeout",
        str(config.startup_timeout_s),
        "--touchdown-hold",
        str(config.touchdown_hold_s),
        "--output-directory",
        str(batch_dir),
        "--batch-id",
        episode["batch_id"],
        "--episode-id",
        episode["episode_id"],
        "--camera-model",
        config.camera_model,
        "--tracking-mode",
        episode["tracking_mode"],
        "--prediction-horizon",
        str(episode["prediction_horizon_s"]),
        "--velocity-ff-gain",
        str(episode["velocity_feedforward_gain"]),
        "--vertical-ff-gain",
        str(episode["vertical_velocity_feedforward_gain"]),
        "--vertical-ff-max",
        str(episode["vertical_velocity_feedforward_max_mps"]),
        "--experiment-profile",
        episode["profile"],
        "--terminal-stabilization-mode",
        episode["terminal_stabilization_mode"],
        "--evaluator",
        episode["evaluator"],
        "--success-bag-policy",
        config.success_bag_policy,
        "--failure-bag-policy",
        config.failure_bag_policy,
        "--workspace-dir",
        str(workspace_dir),
    ]
    command.append(
        "--enable-vertical-ff"
        if episode["vertical_velocity_feedforward_enabled"]
        else "--disable-vertical-ff"
    )
    if episode["record_camera_debug"]:
        command.append("--record-camera-debug")
    if retry_existing_failure:
        command.append("--retry-existing-failure")
    if rerun_after_code_change:
        command.append("--rerun-after-code-change")
    return command


def load_episode_rows(batch_dir: Path, plan: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for episode in plan:
        manifest_path = batch_dir / episode["episode_id"] / "manifest.json"
        if not manifest_path.is_file():
            continue
        try:
            manifest = read_json(manifest_path)
        except ValueError:
            rows.append(
                {
                    **episode,
                    "success": False,
                    "failure_reason": "EVALUATION_ERROR",
                }
            )
            continue
        rows.append({**episode, **manifest})
    return rows


def write_progress(
    batch_dir: Path,
    batch_manifest: dict[str, Any],
    plan: list[dict[str, Any]],
) -> None:
    rows = load_episode_rows(batch_dir, plan)
    batch_manifest["completed_episodes"] = sum(
        1 for row in rows if row.get("completed") is True
    )
    batch_manifest["successful_episodes"] = sum(
        1 for row in rows if row.get("success") is True
    )
    batch_manifest["failed_episodes"] = sum(
        1
        for row in rows
        if row.get("completed") is True and row.get("success") is False
    )
    batch_manifest["updated_at"] = utc_now_iso()
    atomic_write_json(batch_dir / "batch_manifest.json", batch_manifest)
    write_csv(batch_dir / "episodes.csv", EPISODE_CSV_FIELDS, rows)


def run_batch(args: argparse.Namespace) -> dict[str, Any]:
    workspace_dir = args.workspace_dir.expanduser().resolve()
    config_path = args.config.expanduser().resolve()
    config = load_batch_config(config_path, workspace_dir=workspace_dir)
    retry_failures_requested = bool(getattr(args, "retry_failures", False))
    if args.resume and not args.batch_id:
        raise ValueError("--resume requires --batch-id so episode IDs remain deterministic")
    if retry_failures_requested and not args.resume:
        raise ValueError("--retry-failures requires --resume")
    batch_id = args.batch_id or make_batch_id(config.name)
    batch_dir = config.output_root / batch_id
    plan = expanded_plan(config, batch_id)
    matrix = matrix_rows(config)

    if args.dry_run:
        dry_run_result = {
            "dry_run": True,
            "batch_id": batch_id,
            "batch_directory": str(batch_dir),
            "config": {
                **asdict(config),
                "output_root": str(config.output_root),
            },
            "experiment_matrix": matrix,
            "episodes": plan,
        }
        print(json.dumps(dry_run_result, ensure_ascii=False, indent=2, default=str))
        return dry_run_result

    if batch_dir.exists() and not args.resume:
        raise ValueError(
            f"batch directory already exists: {batch_dir}; use --resume --batch-id {batch_id}"
        )
    batch_dir.mkdir(parents=True, exist_ok=True)
    current_commit, current_dirty = git_state(workspace_dir)
    batch_manifest: dict[str, Any] = {
        "batch_id": batch_id,
        "name": config.name,
        "config_path": str(config_path),
        "output_directory": str(batch_dir),
        "created_at": utc_now_iso(),
        "updated_at": utc_now_iso(),
        "resume": bool(args.resume),
        "retry_failures": retry_failures_requested,
        "sequential": True,
        "git_commit_at_start": current_commit,
        "dirty_worktree_at_start": current_dirty,
        "planned_episodes": len(plan),
        "not_applicable_combinations": sum(
            row["applicability"] == "NOT_APPLICABLE" for row in matrix
        ),
        "completed_episodes": 0,
        "successful_episodes": 0,
        "failed_episodes": 0,
        "supported_methods": list(SUPPORTED_METHODS),
        "supported_scenarios": list(SUPPORTED_SCENARIOS),
        "experiment_matrix": matrix,
        "episodes": plan,
        "completed": False,
    }
    write_csv(batch_dir / "experiment_matrix.csv", MATRIX_CSV_FIELDS, matrix)
    write_progress(batch_dir, batch_manifest, plan)

    for episode in plan:
        episode_dir = batch_dir / episode["episode_id"]
        retry_existing_failure = False
        rerun_after_code_change = False
        if args.resume and episode_dir.exists():
            manifest_path = episode_dir / "manifest.json"
            previous: dict[str, Any] | None = None
            if manifest_path.is_file():
                try:
                    previous = read_json(manifest_path)
                except ValueError:
                    previous = None
            same_fingerprint = bool(
                previous
                and previous.get("git_commit") == current_commit
                and previous.get("dirty_worktree") is current_dirty
            )
            if episode_result_complete(
                episode_dir,
                expected_git_commit=current_commit,
                expected_dirty_worktree=current_dirty,
            ):
                if previous and previous.get("success") is False and retry_failures_requested:
                    retry_existing_failure = True
                else:
                    print(f"[resume] skip completed {episode['episode_id']}")
                    continue
            elif previous is not None and previous.get("completed") is True:
                if same_fingerprint and previous.get("success") is False:
                    if retry_failures_requested:
                        retry_existing_failure = True
                    else:
                        print(f"[resume] keep failed {episode['episode_id']}")
                        continue
                else:
                    rerun_after_code_change = True
            elif episode_dir.exists():
                rerun_after_code_change = True

        command = build_single_command(
            single_runner=args.single_runner.expanduser().resolve(),
            workspace_dir=workspace_dir,
            batch_dir=batch_dir,
            config=config,
            episode=episode,
            retry_existing_failure=retry_existing_failure,
            rerun_after_code_change=rerun_after_code_change,
        )
        print(f"[run] {episode['episode_id']}")
        result = subprocess.run(command, cwd=workspace_dir, check=False)
        if result.returncode != 0 and not (episode_dir / "manifest.json").is_file():
            episode_dir.mkdir(parents=True, exist_ok=True)
            fallback_manifest = {
                **episode,
                "git_commit": current_commit,
                "dirty_worktree": current_dirty,
                "start_wall_time": None,
                "end_wall_time": utc_now_iso(),
                "duration_s": None,
                "camera_model": config.camera_model,
                "start_command": command,
                "exit_code": result.returncode,
                "success": False,
                "failure_reason": "PROCESS_EXITED",
                "failure_detail": "single runner exited without writing manifest",
                "bag_path": None,
                "evaluation_path": None,
                "completed": True,
            }
            atomic_write_json(episode_dir / "manifest.json", fallback_manifest)
        write_progress(batch_dir, batch_manifest, plan)

    rows = load_episode_rows(batch_dir, plan)
    completed = sum(1 for row in rows if row.get("completed") is True)
    batch_manifest["completed"] = completed == len(plan)
    batch_manifest["finished_at"] = utc_now_iso()
    write_progress(batch_dir, batch_manifest, plan)
    print(json.dumps(batch_manifest, ensure_ascii=False, indent=2))
    return batch_manifest


def main() -> int:
    args = parse_arguments()
    try:
        run_batch(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
