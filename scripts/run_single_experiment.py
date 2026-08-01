#!/usr/bin/env python3
"""运行一轮 P7/P8A SITL 实验并保存结构化结果。"""

from __future__ import annotations

import argparse
import json
import os
import queue
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

import yaml

from p7_experiment_utils import (
    FAILURE_TYPES,
    SUPPORTED_SCENARIOS,
    atomic_write_json,
    classify_failure,
    make_batch_id,
    make_episode_id,
    read_evaluation_json,
    utc_now_iso,
)

STALE_PATTERN = (
    r"MicroXRCEAgent|(^|/)px4( |$)|gz sim|moving_deck_controller|"
    r"deck_gnss_simulator|parameter_bridge.*world/aruco|aruco_detector_node|"
    r"px4_aruco_landing_node"
)
TRACKING_MODES = (
    "PREDICTED_POSITION_VELOCITY_FF",
    "RELATIVE_MPC",
)

KNOWN_STATES = {
    "INIT",
    "WAIT_FOR_PX4",
    "OFFBOARD_PRE_STREAM",
    "ARM_AND_TAKEOFF",
    "WAIT_DECK_GNSS",
    "RENDEZVOUS_GNSS",
    "ACQUIRE_ARUCO",
    "VISUAL_HANDOVER",
    "TRACK_TARGET",
    "WAIT_LANDING_WINDOW",
    "DESCEND",
    "TEST_HEIGHT_HOLD",
    "FINAL_DESCENT",
    "TOUCHDOWN_CANDIDATE_HOLD",
    "TOUCHDOWN_HOLD",
    "RECOVER_TO_GNSS",
    "RECOVER_CLIMB",
    "ABORT",
    "DONE",
}


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one P7/P8A touchdown SITL episode."
    )
    parser.add_argument("--scenario", choices=SUPPORTED_SCENARIOS, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--episode-timeout", type=float, default=600.0)
    parser.add_argument("--startup-timeout", type=float, default=120.0)
    parser.add_argument("--touchdown-hold", type=float, default=10.0)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--batch-id")
    parser.add_argument("--episode-id")
    parser.add_argument(
        "--camera-model", choices=("close-range", "px4-default"), default="close-range"
    )
    parser.add_argument(
        "--tracking-mode",
        choices=TRACKING_MODES,
        default="PREDICTED_POSITION_VELOCITY_FF",
    )
    parser.add_argument("--record-camera-debug", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--workspace-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent,
    )
    return parser.parse_args()


def git_state(workspace_dir: Path) -> tuple[str, bool]:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=workspace_dir,
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()
    dirty = bool(
        subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=workspace_dir,
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()
    )
    return commit, dirty


def stale_processes() -> list[str]:
    result = subprocess.run(
        ["pgrep", "-af", STALE_PATTERN],
        text=True,
        capture_output=True,
        check=False,
    )
    return [line for line in result.stdout.splitlines() if line.strip()]


def build_start_command(
    workspace_dir: Path,
    scenario: str,
    seed: int,
    bag_path: Path,
    camera_model: str,
    record_camera_debug: bool,
    tracking_mode: str = "PREDICTED_POSITION_VELOCITY_FF",
) -> list[str]:
    command = [
        str(workspace_dir / "scripts" / "start_sitl.sh"),
        "--scenario",
        scenario,
        "--headless",
        "--bag-output",
        str(bag_path),
        "--seed",
        str(seed),
        "--auto-confirm-controller",
        "--camera-model",
        camera_model,
        "--tracking-mode",
        tracking_mode,
        "--enable-relative-descent",
        "--enable-final-descent",
    ]
    command.append("--record-camera-debug" if record_camera_debug else "--record")
    return command


class RosStateMonitor:
    """使用独立 rclpy Context 订阅状态，避免依赖不稳定的 ROS CLI daemon。"""

    def __init__(
        self,
        events: queue.Queue[tuple[str, str]],
        run_log: TextIO,
    ) -> None:
        self._events = events
        self._run_log = run_log
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._node: Any = None

    def start(self) -> None:
        try:
            import rclpy
            from std_msgs.msg import String
        except ImportError as error:
            raise RuntimeError(
                "ROS 2 Python modules are unavailable; source ROS Humble first"
            ) from error

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(f"p7_state_monitor_{os.getpid()}")

        def callback(message: String) -> None:
            state = str(message.data)
            self._run_log.write(state + "\n---\n")
            self._run_log.flush()
            self._events.put(("state", state))

        self._node.create_subscription(String, "/landing/state", callback, 10)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        import rclpy

        try:
            while not self._stop_event.is_set() and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.2)
        except Exception as error:  # noqa: BLE001 - propagate ROS runtime failures via queue
            self._events.put(("monitor_error", repr(error)))

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._node is not None:
            self._node.destroy_node()
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except ImportError:
            pass


def _stream_lines(
    stream: TextIO,
    destination: TextIO,
    events: queue.Queue[tuple[str, str]],
    kind: str,
) -> None:
    for line in iter(stream.readline, ""):
        destination.write(line)
        destination.flush()
        events.put((kind, line.rstrip("\n")))


def parse_state_line(line: str) -> str | None:
    value = line.strip().strip("'\"")
    return value if value in KNOWN_STATES else None


def terminate_process_group(process: subprocess.Popen[str], timeout_s: float = 20.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=5.0)


def snapshot_configs(
    workspace_dir: Path,
    episode_dir: Path,
    scenario: str,
    seed: int,
    tracking_mode: str,
) -> None:
    controller_source = (
        workspace_dir
        / "src"
        / "aruco_precision_landing_cpp"
        / "config"
        / "px4_aruco_landing.yaml"
    )
    shutil.copyfile(controller_source, episode_dir / "controller_config.yaml")
    scenario_filename = {
        "static": "static.yaml",
        "constant02": "constant_velocity_0p2.yaml",
        "heave_h1": "heave_h1.yaml",
        "heave_h2": "heave_h2.yaml",
        "heave_h3": "heave_h3.yaml",
    }[scenario]
    scenario_source = (
        workspace_dir / "src" / "moving_deck_sim" / "config" / scenario_filename
    )
    gnss_source = (
        workspace_dir / "src" / "moving_deck_sim" / "config" / "gnss_ideal.yaml"
    )
    scenario_data = yaml.safe_load(scenario_source.read_text(encoding="utf-8"))
    gnss_data = yaml.safe_load(gnss_source.read_text(encoding="utf-8"))
    scenario_data["moving_deck_controller"]["ros__parameters"]["random_seed"] = seed
    gnss_data["deck_gnss_simulator"]["ros__parameters"]["random_seed"] = seed
    snapshot = {
        "touchdown_episode": {
            "scenario": scenario,
            "seed": seed,
            "tracking_mode": tracking_mode,
        },
        **scenario_data,
        **gnss_data,
    }
    (episode_dir / "scenario_config.yaml").write_text(
        yaml.safe_dump(snapshot, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )


def evaluator_path_for_scenario(workspace_dir: Path, scenario: str) -> Path:
    """根据场景选择复用的 P6B 或 P8A 离线评测器。"""

    filename = (
        "evaluate_p8a_heave_touchdown.py"
        if scenario.startswith("heave_h")
        else "evaluate_p6b_touchdown.py"
    )
    return workspace_dir / "scripts" / filename


def run_evaluator(
    workspace_dir: Path,
    scenario: str,
    bag_path: Path,
    episode_dir: Path,
    run_log: TextIO,
) -> tuple[dict[str, Any] | None, str | None]:
    evaluator = evaluator_path_for_scenario(workspace_dir, scenario)
    base_command = [sys.executable, str(evaluator), str(bag_path)]
    if scenario == "constant02":
        base_command.append("--require-moving-deck")
    human = subprocess.run(base_command, text=True, capture_output=True, check=False)
    human_output = human.stdout + human.stderr
    (episode_dir / "evaluation.txt").write_text(human_output, encoding="utf-8")
    run_log.write(f"\n===== {evaluator.stem} =====\n")
    run_log.write(human_output)
    run_log.flush()

    json_result = subprocess.run(
        [*base_command, "--json"], text=True, capture_output=True, check=False
    )
    if json_result.returncode not in (0, 2):
        return None, json_result.stderr.strip() or "evaluator failed"
    try:
        evaluation = json.loads(json_result.stdout)
        if not isinstance(evaluation, dict):
            raise ValueError("evaluation root is not an object")
        atomic_write_json(episode_dir / "evaluation.json", evaluation)
        evaluation = read_evaluation_json(episode_dir / "evaluation.json")
    except (json.JSONDecodeError, ValueError) as error:
        return None, str(error)
    return evaluation, None


def run_episode(args: argparse.Namespace) -> dict[str, Any]:
    workspace_dir = args.workspace_dir.expanduser().resolve()
    if args.seed < 0 or args.seed > 0xFFFFFFFF:
        raise ValueError("seed must fit in uint32")
    if args.episode_timeout <= 0.0 or args.startup_timeout <= 0.0:
        raise ValueError("timeouts must be positive")
    if args.touchdown_hold < 10.0:
        raise ValueError("touchdown hold must be at least 10 seconds")

    batch_id = args.batch_id or make_batch_id("p7_single")
    episode_id = args.episode_id or make_episode_id(
        batch_id, args.scenario, 1, args.seed
    )
    output_root = args.output_directory.expanduser().resolve()
    episode_dir = output_root / episode_id
    bag_path = episode_dir / "bag"
    tracking_mode = getattr(
        args, "tracking_mode", "PREDICTED_POSITION_VELOCITY_FF"
    )
    if tracking_mode not in TRACKING_MODES:
        raise ValueError(f"unsupported tracking mode: {tracking_mode}")
    start_command = build_start_command(
        workspace_dir,
        args.scenario,
        args.seed,
        bag_path,
        args.camera_model,
        args.record_camera_debug,
        tracking_mode,
    )
    if args.dry_run:
        result = {
            "dry_run": True,
            "batch_id": batch_id,
            "episode_id": episode_id,
            "episode_directory": str(episode_dir),
            "scenario": args.scenario,
            "seed": args.seed,
            "command": start_command,
        }
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return result
    if episode_dir.exists():
        raise ValueError(f"episode directory already exists: {episode_dir}")
    episode_dir.mkdir(parents=True)

    git_commit, dirty_worktree = git_state(workspace_dir)
    start_wall_time = utc_now_iso()
    start_monotonic = time.monotonic()
    manifest: dict[str, Any] = {
        "episode_id": episode_id,
        "batch_id": batch_id,
        "scenario": args.scenario,
        "seed": args.seed,
        "git_commit": git_commit,
        "dirty_worktree": dirty_worktree,
        "start_wall_time": start_wall_time,
        "end_wall_time": None,
        "duration_s": None,
        "camera_model": args.camera_model,
        "tracking_mode": tracking_mode,
        "record_camera_debug": args.record_camera_debug,
        "start_command": start_command,
        "exit_code": None,
        "success": False,
        "failure_reason": "UNKNOWN",
        "failure_detail": None,
        "bag_path": str(bag_path),
        "evaluation_path": None,
        "state_sequence": [],
        "completed": False,
    }
    atomic_write_json(episode_dir / "manifest.json", manifest)
    snapshot_configs(
        workspace_dir, episode_dir, args.scenario, args.seed, tracking_mode
    )

    preexisting = stale_processes()
    if preexisting:
        detail = "existing SITL processes: " + " | ".join(preexisting)
        (episode_dir / "run.log").write_text(detail + "\n", encoding="utf-8")
        manifest["failure_reason"] = "STARTUP_FAILURE"
        manifest["failure_detail"] = detail
        manifest["end_wall_time"] = utc_now_iso()
        manifest["duration_s"] = time.monotonic() - start_monotonic
        manifest["completed"] = True
        atomic_write_json(episode_dir / "manifest.json", manifest)
        print(json.dumps(manifest, ensure_ascii=False, indent=2))
        return manifest

    events: queue.Queue[tuple[str, str]] = queue.Queue()
    start_process: subprocess.Popen[str] | None = None
    state_monitor: RosStateMonitor | None = None
    event = "UNKNOWN"
    event_detail: str | None = None
    state_sequence: list[str] = []
    current_state: str | None = None
    hold_start_monotonic: float | None = None
    episode_start_monotonic: float | None = None

    with (episode_dir / "run.log").open("w", encoding="utf-8") as run_log:
        run_log.write("Command: " + shlex.join(start_command) + "\n")
        run_log.flush()
        try:
            start_process = subprocess.Popen(
                start_command,
                cwd=workspace_dir,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            assert start_process.stdout is not None
            log_thread = threading.Thread(
                target=_stream_lines,
                args=(start_process.stdout, run_log, events, "log"),
                daemon=True,
            )
            log_thread.start()
            state_monitor = RosStateMonitor(events, run_log)
            state_monitor.start()

            while True:
                now = time.monotonic()
                startup_elapsed = now - start_monotonic
                episode_elapsed = (
                    now - episode_start_monotonic
                    if episode_start_monotonic is not None
                    else 0.0
                )
                try:
                    kind, line = events.get(timeout=0.2)
                except queue.Empty:
                    kind, line = "", ""
                if kind == "monitor_error":
                    event = "STARTUP_FAILURE" if not state_sequence else "PROCESS_EXITED"
                    event_detail = f"ROS state monitor failed: {line}"
                    break
                if kind == "state":
                    state = parse_state_line(line)
                    if state is not None:
                        if episode_start_monotonic is None:
                            episode_start_monotonic = now
                        if not state_sequence or state_sequence[-1] != state:
                            state_sequence.append(state)
                        if state != current_state:
                            current_state = state
                            hold_start_monotonic = (
                                now if state == "TOUCHDOWN_HOLD" else None
                            )
                        if state == "ABORT":
                            event = "PX4_ABORT"
                            event_detail = "controller entered ABORT"
                            break
                if current_state == "TOUCHDOWN_HOLD" and hold_start_monotonic is not None:
                    if now - hold_start_monotonic >= args.touchdown_hold:
                        event = "NONE"
                        event_detail = (
                            f"TOUCHDOWN_HOLD sustained for {args.touchdown_hold:.1f} s"
                        )
                        break
                if start_process.poll() is not None:
                    event = "PROCESS_EXITED"
                    event_detail = f"start_sitl exited with {start_process.returncode}"
                    break
                if not state_sequence and startup_elapsed >= args.startup_timeout:
                    event = "PX4_TIMEOUT"
                    event_detail = "no landing state received before startup timeout"
                    break
                if (
                    episode_start_monotonic is not None
                    and episode_elapsed >= args.episode_timeout
                ):
                    event = "EPISODE_TIMEOUT"
                    event_detail = "episode timeout reached"
                    break
        except (OSError, subprocess.SubprocessError) as error:
            event = "STARTUP_FAILURE"
            event_detail = str(error)
        finally:
            if start_process is not None:
                terminate_process_group(start_process)
            if state_monitor is not None:
                state_monitor.stop()

        cleanup_residuals = stale_processes()
        cleanup_ok = not cleanup_residuals
        evaluation: dict[str, Any] | None = None
        evaluation_error: str | None = None
        if bag_path.is_dir():
            evaluation, evaluation_error = run_evaluator(
                workspace_dir, args.scenario, bag_path, episode_dir, run_log
            )
            if evaluation is not None:
                manifest["evaluation_path"] = str(episode_dir / "evaluation.json")
        elif event == "NONE":
            evaluation_error = "bag directory was not created"

        success = bool(
            event == "NONE"
            and evaluation is not None
            and evaluation.get("positive_touchdown_passed") is True
            and cleanup_ok
        )
        log_text = (episode_dir / "run.log").read_text(
            encoding="utf-8", errors="replace"
        )
        failure_reason = classify_failure(
            success=success,
            event=("EVALUATION_ERROR" if evaluation_error and event == "NONE" else event),
            state_sequence=state_sequence,
            evaluation=evaluation,
            log_text=log_text,
            cleanup_ok=cleanup_ok,
        )
        if failure_reason not in FAILURE_TYPES:
            failure_reason = "UNKNOWN"
        if evaluation_error:
            event_detail = (
                f"{event_detail}; evaluator: {evaluation_error}"
                if event_detail
                else f"evaluator: {evaluation_error}"
            )
        if cleanup_residuals:
            residual_detail = "cleanup residuals: " + " | ".join(cleanup_residuals)
            event_detail = f"{event_detail}; {residual_detail}" if event_detail else residual_detail

    end_monotonic = time.monotonic()
    manifest.update(
        {
            "end_wall_time": utc_now_iso(),
            "duration_s": end_monotonic - start_monotonic,
            "exit_code": start_process.returncode if start_process is not None else None,
            "success": success,
            "failure_reason": failure_reason,
            "failure_detail": event_detail,
            "state_sequence": state_sequence,
            "completed": True,
        }
    )
    atomic_write_json(episode_dir / "manifest.json", manifest)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return manifest


def main() -> int:
    args = parse_arguments()
    try:
        run_episode(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
