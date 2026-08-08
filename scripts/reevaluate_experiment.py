#!/usr/bin/env python3
"""Re-run only the offline evaluator for a completed paper evaluation episode.

This preserves the original Bag, logs, manifest failure evidence and previous
assessment files. It is intended for evaluator-only fixes; it never starts SITL.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

from experiment_utils import atomic_write_json, read_json, utc_now_iso
from run_single_experiment import evaluators_passed, git_state, run_evaluator


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Re-evaluate one completed episode Bag.")
    parser.add_argument("episode_directory", type=Path)
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="allow re-evaluation when the prior failure was not EVALUATION_ERROR",
    )
    return parser.parse_args()


def archive_evaluation_files(episode_dir: Path) -> Path | None:
    candidates = (
        "evaluation.json",
        "evaluation.txt",
        "legacy_evaluation.json",
        "legacy_evaluation.txt",
    )
    existing = [episode_dir / name for name in candidates if (episode_dir / name).exists()]
    if not existing:
        return None
    index = 1
    while True:
        archive_dir = episode_dir / f"evaluation_attempt{index:02d}"
        if not archive_dir.exists():
            break
        index += 1
    archive_dir.mkdir()
    for source in existing:
        shutil.move(str(source), archive_dir / source.name)
    return archive_dir


def refresh_parent_batch_manifest(episode_dir: Path) -> None:
    batch_manifest_path = episode_dir.parent / "batch_manifest.json"
    if not batch_manifest_path.is_file():
        return
    batch_manifest = read_json(batch_manifest_path)
    plan = batch_manifest.get("episodes")
    if not isinstance(plan, list):
        return
    manifests: list[dict[str, Any]] = []
    for entry in plan:
        if not isinstance(entry, dict) or not entry.get("episode_id"):
            continue
        manifest_path = episode_dir.parent / str(entry["episode_id"]) / "manifest.json"
        if not manifest_path.is_file():
            continue
        try:
            manifests.append(read_json(manifest_path))
        except ValueError:
            continue
    completed = sum(item.get("completed") is True for item in manifests)
    successful = sum(item.get("success") is True for item in manifests)
    failed = sum(
        item.get("completed") is True and item.get("success") is False
        for item in manifests
    )
    batch_manifest.update(
        {
            "completed_episodes": completed,
            "successful_episodes": successful,
            "failed_episodes": failed,
            "completed": completed == int(batch_manifest.get("planned_episodes", len(plan))),
            "updated_at": utc_now_iso(),
        }
    )
    atomic_write_json(batch_manifest_path, batch_manifest)


def re_evaluate_episode(
    episode_dir: Path,
    workspace_dir: Path,
    *,
    force: bool = False,
) -> dict[str, Any]:
    episode_dir = episode_dir.expanduser().resolve()
    workspace_dir = workspace_dir.expanduser().resolve()
    manifest_path = episode_dir / "manifest.json"
    manifest = read_json(manifest_path)
    if manifest.get("completed") is not True:
        raise ValueError("episode is not completed")
    prior_failure = str(manifest.get("failure_reason", "UNKNOWN"))
    if prior_failure != "EVALUATION_ERROR" and not force:
        raise ValueError(
            "offline-only re-evaluation is restricted to EVALUATION_ERROR; use --force explicitly"
        )
    bag_path = Path(str(manifest.get("bag_path", episode_dir / "bag")))
    if not bag_path.is_dir():
        raise ValueError(f"episode Bag does not exist: {bag_path}")
    scenario = str(manifest.get("scenario"))
    seed = int(manifest.get("seed"))
    profile = str(manifest.get("profile", manifest.get("experiment_profile", "touchdown")))
    terminal_mode = str(manifest.get("terminal_stabilization_mode", "disabled"))
    evaluator_route = str(manifest.get("evaluator", "auto"))
    tilted_deck_touchdown = bool(manifest.get("tilted_deck_touchdown", False))

    archived = archive_evaluation_files(episode_dir)
    run_log_path = episode_dir / "run.log"
    with run_log_path.open("a", encoding="utf-8") as run_log:
        run_log.write("\n===== OFFLINE EVALUATOR REVALIDATION =====\n")
        evaluation, legacy_evaluation, error = run_evaluator(
            workspace_dir,
            scenario,
            seed,
            bag_path,
            episode_dir,
            run_log,
            tilted_deck_touchdown=tilted_deck_touchdown,
            experiment_profile=profile,
            terminal_stabilization_mode=terminal_mode,
            evaluator_route=evaluator_route,
        )
    passed = evaluators_passed(
        evaluation,
        legacy_evaluation,
        experiment_profile=profile,
        scenario=scenario,
    )
    evaluation_commit, evaluation_dirty = git_state(workspace_dir)
    history = list(manifest.get("evaluation_revalidation_history", []))
    history.append(
        {
            "at": utc_now_iso(),
            "prior_failure_reason": prior_failure,
            "prior_failure_detail": manifest.get("failure_detail"),
            "archived_evaluation_directory": str(archived) if archived else None,
            "evaluation_code_commit": evaluation_commit,
            "evaluation_code_dirty": evaluation_dirty,
            "passed": passed,
            "error": error,
        }
    )
    manifest.update(
        {
            "evaluation_path": (
                str(episode_dir / "evaluation.json") if evaluation is not None else None
            ),
            "legacy_evaluation_path": (
                str(episode_dir / "legacy_evaluation.json")
                if legacy_evaluation is not None
                else None
            ),
            "success": bool(passed and error is None),
            "failure_reason": (
                "NONE" if passed and error is None
                else "EVALUATION_ERROR" if error is not None
                else "SAFETY_GATE_FAILURE"
            ),
            "failure_detail": (
                None if passed and error is None
                else f"offline evaluator revalidation failed: {error or 'hard gate'}"
            ),
            "evaluation_code_commit": evaluation_commit,
            "evaluation_code_dirty": evaluation_dirty,
            "evaluation_revalidated_at": utc_now_iso(),
            "evaluation_revalidation_history": history,
            "completed": True,
        }
    )
    atomic_write_json(manifest_path, manifest)
    refresh_parent_batch_manifest(episode_dir)
    return manifest


def main() -> int:
    args = parse_arguments()
    try:
        manifest = re_evaluate_episode(
            args.episode_directory,
            args.workspace_dir,
            force=args.force,
        )
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0 if manifest.get("success") is True else 2


if __name__ == "__main__":
    raise SystemExit(main())
