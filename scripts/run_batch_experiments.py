#!/usr/bin/env python3
"""顺序运行 P7/P8A 触地批量实验。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any

from p7_experiment_utils import (
    FAILURE_TYPES,
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
    "scenario",
    "seed",
    "success",
    "failure_reason",
    "duration_s",
    "git_commit",
    "dirty_worktree",
    "bag_path",
    "evaluation_path",
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a sequential touchdown experiment batch.")
    parser.add_argument("config", type=Path, help="Experiment YAML configuration")
    parser.add_argument("--batch-id", help="Explicit batch ID; required to resume a known batch")
    parser.add_argument("--resume", action="store_true", help="Skip completed episodes")
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


def expanded_plan(config: Any, batch_id: str) -> list[dict[str, Any]]:
    plan: list[dict[str, Any]] = []
    for scenario_spec in config.scenarios:
        for repetition, seed in enumerate(scenario_spec.seeds, start=1):
            plan.append(
                {
                    "episode_id": make_episode_id(
                        batch_id, scenario_spec.scenario, repetition, seed
                    ),
                    "batch_id": batch_id,
                    "scenario": scenario_spec.scenario,
                    "repetition": repetition,
                    "seed": seed,
                }
            )
    return plan


def build_single_command(
    *,
    single_runner: Path,
    workspace_dir: Path,
    batch_dir: Path,
    config: Any,
    episode: dict[str, Any],
) -> list[str]:
    command = [
        sys.executable,
        str(single_runner),
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
        "--workspace-dir",
        str(workspace_dir),
    ]
    if config.record_camera_debug:
        command.append("--record-camera-debug")
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
        rows.append(manifest)
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
    if args.resume and not args.batch_id:
        raise ValueError("--resume requires --batch-id so episode IDs remain deterministic")
    batch_id = args.batch_id or make_batch_id(config.name)
    batch_dir = config.output_root / batch_id
    plan = expanded_plan(config, batch_id)

    if args.dry_run:
        dry_run_result = {
            "dry_run": True,
            "batch_id": batch_id,
            "batch_directory": str(batch_dir),
            "config": {
                **asdict(config),
                "output_root": str(config.output_root),
            },
            "episodes": plan,
        }
        print(json.dumps(dry_run_result, ensure_ascii=False, indent=2, default=str))
        return dry_run_result

    if batch_dir.exists() and not args.resume:
        raise ValueError(
            f"batch directory already exists: {batch_dir}; use --resume --batch-id {batch_id}"
        )
    batch_dir.mkdir(parents=True, exist_ok=True)
    batch_manifest: dict[str, Any] = {
        "batch_id": batch_id,
        "name": config.name,
        "config_path": str(config_path),
        "output_directory": str(batch_dir),
        "created_at": utc_now_iso(),
        "updated_at": utc_now_iso(),
        "resume": bool(args.resume),
        "sequential": True,
        "planned_episodes": len(plan),
        "completed_episodes": 0,
        "successful_episodes": 0,
        "failed_episodes": 0,
        "supported_scenarios": list(SUPPORTED_SCENARIOS),
        "episodes": plan,
        "completed": False,
    }
    write_progress(batch_dir, batch_manifest, plan)

    for episode in plan:
        episode_dir = batch_dir / episode["episode_id"]
        if args.resume and episode_result_complete(episode_dir):
            print(f"[resume] skip completed {episode['episode_id']}")
            continue
        command = build_single_command(
            single_runner=args.single_runner.expanduser().resolve(),
            workspace_dir=workspace_dir,
            batch_dir=batch_dir,
            config=config,
            episode=episode,
        )
        print(f"[run] {episode['episode_id']}")
        result = subprocess.run(command, cwd=workspace_dir, check=False)
        if result.returncode != 0 and not (episode_dir / "manifest.json").is_file():
            episode_dir.mkdir(parents=True, exist_ok=True)
            fallback_manifest = {
                **episode,
                "git_commit": None,
                "dirty_worktree": None,
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
