#!/usr/bin/env python3
"""顺序运行冻结的 4 场景 × 3 seed 甲板 6-DoF shadow SITL 矩阵。"""

from __future__ import annotations

import argparse
import json
import queue
import shutil
import subprocess
import sys
import time
from pathlib import Path

from evaluate_deck_motion_shadow import evaluate
from run_single_experiment import (
    RosStateMonitor,
    cleanup_stale_processes,
    stale_processes,
    terminate_process_group,
)

SCENARIOS = ("static", "rollpitch", "combined", "rigid_body_motion")
SEEDS = (1, 2, 3)


def start_command(
    workspace: Path,
    scenario: str,
    seed: int,
    bag: Path,
    environment: str = "legacy",
) -> list[str]:
    if environment not in {"legacy", "marine"}:
        raise ValueError(f"unsupported environment: {environment}")
    return [
        str(workspace / "scripts" / "start_sitl.sh"),
        "--environment", environment,
        "--scenario", scenario,
        "--seed", str(seed),
        "--headless",
        "--auto-confirm-controller",
        "--tracking-mode", "PREDICTED_POSITION_VELOCITY_FF",
        "--rendezvous-altitude", "7.0",
        "--bag-output", str(bag),
        "--record",
    ]


def run_episode(
    workspace: Path,
    output: Path,
    scenario: str,
    seed: int,
    environment: str,
    startup_timeout_s: float,
    record_duration_s: float,
) -> dict[str, object]:
    episode_id = f"{scenario}_s{seed}"
    episode_dir = output / episode_id
    if episode_dir.exists():
        raise FileExistsError(f"episode already exists: {episode_dir}")
    episode_dir.mkdir(parents=True)
    scenario_file = {
        "static": "static.yaml",
        "rollpitch": "roll_pitch.yaml",
        "combined": "combined.yaml",
        "rigid_body_motion": "rigid_body_motion.yaml",
    }[scenario]
    shutil.copyfile(
        workspace / "src" / "aruco_precision_landing_cpp" / "config" / "px4_aruco_landing.yaml",
        episode_dir / "controller_config.yaml",
    )
    shutil.copyfile(
        workspace / "src" / "moving_deck_sim" / "config" / scenario_file,
        episode_dir / "scenario_config.yaml",
    )
    if environment == "marine":
        shutil.copyfile(
            workspace / "src" / "aruco_detector" / "config"
            / "aruco_detector_marine.yaml",
            episode_dir / "detector_config.yaml",
        )
    bag = episode_dir / "bag"
    command = start_command(workspace, scenario, seed, bag, environment)
    manifest: dict[str, object] = {
        "episode_id": episode_id,
        "scenario": scenario,
        "seed": seed,
        "environment": environment,
        "command": command,
        "success": False,
    }
    events: queue.Queue[tuple[str, str]] = queue.Queue()
    with (episode_dir / "run.log").open("w", encoding="utf-8") as log:
        monitor = RosStateMonitor(events, log)
        monitor.start()
        process = subprocess.Popen(
            command,
            cwd=workspace,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        tracking_start: float | None = None
        deadline = time.monotonic() + startup_timeout_s
        failure = ""
        try:
            while True:
                now = time.monotonic()
                if process.poll() is not None:
                    failure = f"SITL exited with status {process.returncode}"
                    break
                if tracking_start is None and now > deadline:
                    failure = "startup timeout before safe-altitude tracking"
                    break
                try:
                    kind, value = events.get(timeout=0.2)
                except queue.Empty:
                    kind, value = "", ""
                if kind == "monitor_error":
                    failure = value
                    break
                if kind == "state":
                    if value in {"DESCEND", "TEST_HEIGHT_HOLD", "FINAL_DESCENT", "TOUCHDOWN_CANDIDATE_HOLD", "TOUCHDOWN_HOLD"}:
                        failure = f"unsafe state entered: {value}"
                        break
                    if value == "ABORT":
                        failure = "controller entered ABORT"
                        break
                    if value in {"TRACK_TARGET", "WAIT_LANDING_WINDOW"} and tracking_start is None:
                        tracking_start = now
                if tracking_start is not None and now - tracking_start >= record_duration_s:
                    break
        finally:
            terminate_process_group(process)
            monitor.stop()

    residuals = cleanup_stale_processes()
    if residuals:
        failure = failure or "residual SITL processes remain"
        manifest["residual_processes"] = residuals
    if failure:
        manifest["failure"] = failure
        if failure.startswith("unsafe state entered"):
            manifest["safety_passed"] = False
    elif not bag.exists():
        manifest["failure"] = "rosbag was not created"
    else:
        try:
            evaluation = evaluate(bag, planar_board=environment == "marine")
            (episode_dir / "evaluation.json").write_text(
                json.dumps(evaluation, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            if environment == "legacy":
                manifest["evaluation_passed"] = evaluation["passed"]
                manifest["success"] = bool(evaluation["passed"])
                if not evaluation["passed"]:
                    manifest["failure"] = "one or more frozen hard gates failed"
            else:
                metrics = evaluation["planar_board_metrics"]
                manifest["shadow_full_hard_gates_passed"] = evaluation["passed"]
                manifest["safety_passed"] = evaluation["safety_passed"]
                manifest["planar_board_passed"] = evaluation["planar_board_passed"]
                manifest["observed_multi_marker_counts"] = metrics[
                    "observed_multi_marker_counts"
                ]
                manifest["single_marker_frame_count"] = metrics[
                    "single_marker_frame_count"
                ]
                manifest["far_single_frame_count"] = metrics[
                    "far_single_frame_count"
                ]
                manifest["evaluation_passed"] = bool(
                    evaluation["safety_passed"] and evaluation["planar_board_passed"]
                )
                manifest["success"] = manifest["evaluation_passed"]
                if not evaluation["safety_passed"]:
                    manifest["failure"] = "one or more safety gates failed"
                elif not evaluation["planar_board_passed"]:
                    manifest["failure"] = "one or more planar-board gates failed"
        except Exception as error:  # noqa: BLE001 - persist evaluator failures per seed
            manifest["failure"] = f"evaluation error: {error!r}"
    (episode_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--environment", choices=("legacy", "marine"), default="legacy")
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    parser.add_argument("--record-duration", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--workspace", type=Path, default=Path(__file__).resolve().parent.parent
    )
    args = parser.parse_args()
    if args.startup_timeout <= 0.0 or args.record_duration <= 0.0:
        parser.error("timeouts must be positive")
    matrix = [
        {
            "scenario": scenario,
            "seed": seed,
            "command": start_command(
                args.workspace,
                scenario,
                seed,
                args.output / f"{scenario}_s{seed}" / "bag",
                args.environment,
            ),
        }
        for scenario in SCENARIOS
        for seed in SEEDS
    ]
    if args.dry_run:
        dry_run: dict[str, object] = {"episodes": matrix}
        if args.environment == "marine":
            dry_run.update({
                "environment": args.environment,
                "evaluation_mode": "planar_board",
            })
        print(json.dumps(dry_run, indent=2, ensure_ascii=False))
        return 0
    if stale_processes():
        raise RuntimeError("refusing to start while matching PX4/Gazebo/landing processes exist")
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for item in matrix:
        result = run_episode(
            args.workspace,
            args.output,
            item["scenario"],
            item["seed"],
            args.environment,
            args.startup_timeout,
            args.record_duration,
        )
        results.append(result)
        if args.environment == "marine" and result.get("safety_passed") is False:
            break
    aggregate = {
        "environment": args.environment,
        "scenarios": list(SCENARIOS),
        "seeds": list(SEEDS),
        "episodes": results,
    }
    if args.environment == "legacy":
        aggregate["passed"] = len(results) == len(matrix) and all(
            result["success"] for result in results
        )
    else:
        observed_multi_marker_counts = sorted({
            marker_count
            for result in results
            for marker_count in result.get("observed_multi_marker_counts", [])
        })
        single_marker_frame_count = sum(
            int(result.get("single_marker_frame_count", 0)) for result in results
        )
        far_single_frame_count = sum(
            int(result.get("far_single_frame_count", 0)) for result in results
        )
        matrix_gates = {
            "all_twelve_episodes_completed": len(results) == len(matrix),
            "all_episode_safety_gates_passed": (
                len(results) == len(matrix)
                and all(result.get("safety_passed") is True for result in results)
            ),
            "all_episode_planar_board_gates_passed": (
                len(results) == len(matrix)
                and all(result.get("planar_board_passed") is True for result in results)
            ),
            "observed_multi_marker_counts_2_3_4": (
                {2, 3, 4} <= set(observed_multi_marker_counts)
            ),
            "single_marker_frames_routed_to_far_single": (
                single_marker_frame_count == far_single_frame_count
            ),
        }
        aggregate.update(
            {
                "safety_passed_count": sum(
                    result.get("safety_passed") is True for result in results
                ),
                "planar_board_passed_count": sum(
                    result.get("planar_board_passed") is True for result in results
                ),
                "shadow_full_hard_gates_passed_count": sum(
                    result.get("shadow_full_hard_gates_passed") is True
                    for result in results
                ),
                "matrix_planar_board_metrics": {
                    "observed_multi_marker_counts": observed_multi_marker_counts,
                    "single_marker_frame_count": single_marker_frame_count,
                    "far_single_frame_count": far_single_frame_count,
                },
                "matrix_planar_board_gates": matrix_gates,
                "full_shadow_passed": (
                    len(results) == len(matrix)
                    and all(
                        result.get("shadow_full_hard_gates_passed") is True
                        for result in results
                    )
                ),
                "passed": all(matrix_gates.values()),
            }
        )
    (args.output / "manifest.json").write_text(
        json.dumps(aggregate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(aggregate, indent=2, ensure_ascii=False))
    return 0 if aggregate["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
