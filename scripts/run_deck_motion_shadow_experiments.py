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
    elif not bag.exists():
        manifest["failure"] = "rosbag was not created"
    else:
        try:
            evaluation = evaluate(bag)
            (episode_dir / "evaluation.json").write_text(
                json.dumps(evaluation, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                encoding="utf-8",
            )
            manifest["evaluation_passed"] = evaluation["passed"]
            manifest["success"] = bool(evaluation["passed"])
            if not evaluation["passed"]:
                manifest["failure"] = "one or more frozen hard gates failed"
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
        print(json.dumps({"episodes": matrix}, indent=2, ensure_ascii=False))
        return 0
    if stale_processes():
        raise RuntimeError("refusing to start while matching PX4/Gazebo/landing processes exist")
    args.output.mkdir(parents=True, exist_ok=False)
    results = [
        run_episode(
            args.workspace,
            args.output,
            item["scenario"],
            item["seed"],
            args.environment,
            args.startup_timeout,
            args.record_duration,
        )
        for item in matrix
    ]
    aggregate = {
        "environment": args.environment,
        "scenarios": list(SCENARIOS),
        "seeds": list(SEEDS),
        "episodes": results,
        "passed": all(result["success"] for result in results),
    }
    (args.output / "manifest.json").write_text(
        json.dumps(aggregate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(aggregate, indent=2, ensure_ascii=False))
    return 0 if aggregate["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
