#!/usr/bin/env python3
"""运行一轮 P7/P8A SITL 实验并保存结构化结果。"""

from __future__ import annotations

import argparse
import json
import math
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
    SUPPORTED_BAG_POLICIES,
    SUPPORTED_EVALUATORS,
    SUPPORTED_METHODS,
    SUPPORTED_SCENARIOS,
    atomic_write_json,
    classify_failure,
    combination_is_applicable,
    make_batch_id,
    make_episode_id,
    read_evaluation_json,
    utc_now_iso,
)

STALE_PATTERN = (
    r"MicroXRCEAgent|(^|/)px4( |$)|gz sim|moving_deck_controller|"
    r"deck_gnss_simulator|parameter_bridge|aruco_detector_node|"
    r"px4_aruco_landing_node|mavlink_gcs_heartbeat.py"
)
TRACKING_MODES = (
    "PREDICTED_POSITION_VELOCITY_FF",
    "RELATIVE_MPC",
)
EXPERIMENT_PROFILES = (
    "touchdown",
    "safe-altitude",
    "safe-descent",
    "rehearsal",
)
TERMINAL_STABILIZATION_MODES = (
    "disabled",
    "shadow",
    "rehearsal",
    "active",
)
POST_HOLD_RECORDING_MARGIN_S = 1.0
SAFE_ALTITUDE_RECORDING_DURATION_S = 10.0
SAFE_DESCENT_RECORDING_DURATION_S = 9.0
REHEARSAL_RECORDING_DURATION_S = 10.0

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


def required_hold_recording_duration_s(required_hold_s: float) -> float:
    """在硬门所需 hold 之后额外录制停机隔离余量。"""

    if required_hold_s < 0.0:
        raise ValueError("required hold duration must be non-negative")
    return required_hold_s + POST_HOLD_RECORDING_MARGIN_S


def completion_requirement(
    experiment_profile: str, touchdown_hold_s: float
) -> tuple[frozenset[str], float]:
    """返回无人值守 profile 的完成状态集合和连续录制时长。"""

    if experiment_profile == "touchdown":
        return (
            frozenset({"TOUCHDOWN_HOLD"}),
            required_hold_recording_duration_s(touchdown_hold_s),
        )
    if experiment_profile == "safe-altitude":
        return frozenset({"WAIT_LANDING_WINDOW"}), SAFE_ALTITUDE_RECORDING_DURATION_S
    if experiment_profile == "safe-descent":
        return frozenset({"TEST_HEIGHT_HOLD"}), SAFE_DESCENT_RECORDING_DURATION_S
    if experiment_profile == "rehearsal":
        return frozenset({"TEST_HEIGHT_HOLD"}), REHEARSAL_RECORDING_DURATION_S
    raise ValueError(f"unsupported experiment profile: {experiment_profile}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one P7/P8A touchdown SITL episode."
    )
    parser.add_argument("--method", choices=SUPPORTED_METHODS, default="B0")
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
    parser.add_argument("--prediction-horizon", type=float, default=0.10)
    parser.add_argument("--velocity-ff-gain", type=float, default=1.0)
    vertical_group = parser.add_mutually_exclusive_group()
    vertical_group.add_argument(
        "--enable-vertical-ff",
        dest="vertical_ff_enabled",
        action="store_true",
    )
    vertical_group.add_argument(
        "--disable-vertical-ff",
        dest="vertical_ff_enabled",
        action="store_false",
    )
    parser.set_defaults(vertical_ff_enabled=True)
    parser.add_argument("--vertical-ff-gain", type=float, default=1.0)
    parser.add_argument("--vertical-ff-max", type=float, default=0.60)
    parser.add_argument(
        "--experiment-profile",
        choices=EXPERIMENT_PROFILES,
        default="touchdown",
        help="state-driven completion profile for staged P8C-4 validation",
    )
    parser.add_argument(
        "--terminal-stabilization-mode",
        choices=TERMINAL_STABILIZATION_MODES,
        default="disabled",
        help="P8C-4 terminal stabilization mode represented by this episode",
    )
    parser.add_argument(
        "--evaluator",
        choices=SUPPORTED_EVALUATORS,
        default="auto",
        help="explicit evaluator route or automatic selection",
    )
    parser.add_argument(
        "--success-bag-policy",
        choices=SUPPORTED_BAG_POLICIES,
        default="lightweight",
    )
    parser.add_argument(
        "--failure-bag-policy",
        choices=SUPPORTED_BAG_POLICIES,
        default="diagnostic",
    )
    parser.add_argument("--record-camera-debug", action="store_true")
    parser.add_argument(
        "--p8c3-touchdown",
        action="store_true",
        help=(
            "run the strict P8C-3 evaluator; valid for static, constant02, and "
            "positive fixed +2 degree tilt profiles"
        ),
    )
    parser.add_argument(
        "--retry-existing-failure",
        action="store_true",
        help="archive a completed failed episode directory and retry with the requested ID",
    )
    parser.add_argument(
        "--rerun-after-code-change",
        action="store_true",
        help=(
            "archive any completed prior attempt as superseded evidence, then rerun "
            "the same final episode ID after an implementation change"
        ),
    )
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


def archive_completed_episode(
    episode_dir: Path,
    *,
    require_failure: bool,
) -> Path:
    """归档已完成轮次，保留 Bag、评测和日志后允许同一最终 ID 重跑。"""

    manifest_path = episode_dir / "manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(
            f"cannot retry existing episode without a readable manifest: {episode_dir}"
        ) from error
    if not isinstance(manifest, dict):
        raise ValueError(f"existing episode manifest is not an object: {manifest_path}")
    completed = manifest.get("completed") is True
    if require_failure and not completed:
        raise ValueError("existing episode must be completed before it can be archived")
    if require_failure and manifest.get("success") is not False:
        raise ValueError(
            "--retry-existing-failure requires a completed failed episode"
        )
    if not completed:
        suffix = "interrupted_attempt"
    elif manifest.get("success") is False:
        suffix = "failed_attempt"
    else:
        suffix = "superseded_attempt"
    attempt = 1
    while True:
        archive = episode_dir.with_name(
            f"{episode_dir.name}_{suffix}{attempt:02d}"
        )
        if not archive.exists():
            break
        attempt += 1
    shutil.move(str(episode_dir), str(archive))
    return archive


def archive_failed_episode(episode_dir: Path) -> Path:
    """兼容旧调用：仅归档已完成失败轮次。"""

    return archive_completed_episode(episode_dir, require_failure=True)


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
    experiment_profile: str = "touchdown",
    terminal_stabilization_mode: str = "disabled",
    prediction_horizon_s: float = 0.10,
    velocity_feedforward_gain: float = 1.0,
    vertical_velocity_feedforward_enabled: bool = True,
    vertical_velocity_feedforward_gain: float = 1.0,
    vertical_velocity_feedforward_max_mps: float = 0.60,
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
        "--prediction-horizon",
        str(prediction_horizon_s),
        "--velocity-ff-gain",
        str(velocity_feedforward_gain),
        "--vertical-ff-gain",
        str(vertical_velocity_feedforward_gain),
        "--vertical-ff-max",
        str(vertical_velocity_feedforward_max_mps),
    ]
    command.append(
        "--enable-vertical-ff"
        if vertical_velocity_feedforward_enabled
        else "--disable-vertical-ff"
    )
    if experiment_profile in {"safe-descent", "rehearsal", "touchdown"}:
        command.extend(
            ["--enable-relative-descent", "--descent-test-height", "0.50"]
        )
    if experiment_profile == "touchdown":
        command.append("--enable-final-descent")

    terminal_flag = {
        "disabled": None,
        "shadow": "--terminal-contact-stabilization-shadow",
        "rehearsal": "--terminal-contact-stabilization-rehearsal",
        "active": "--enable-terminal-contact-stabilization",
    }.get(terminal_stabilization_mode)
    if terminal_stabilization_mode not in TERMINAL_STABILIZATION_MODES:
        raise ValueError(
            f"unsupported terminal stabilization mode: {terminal_stabilization_mode}"
        )
    if terminal_flag is not None:
        command.append(terminal_flag)
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


def recovery_after_terminal_phase(
    state_sequence: list[str], current_state: str
) -> bool:
    """触地候选或 hold 后进入恢复即为本轮硬失败，不再自动二次降落。"""

    return current_state in {"RECOVER_TO_GNSS", "RECOVER_CLIMB"} and any(
        state in {"TOUCHDOWN_CANDIDATE_HOLD", "TOUCHDOWN_HOLD"}
        for state in state_sequence
    )


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
    experiment_profile: str = "touchdown",
    terminal_stabilization_mode: str = "disabled",
    *,
    method: str = "B0",
    prediction_horizon_s: float = 0.10,
    velocity_feedforward_gain: float = 1.0,
    vertical_velocity_feedforward_enabled: bool = True,
    vertical_velocity_feedforward_gain: float = 1.0,
    vertical_velocity_feedforward_max_mps: float = 0.60,
    evaluator: str = "auto",
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
        "constant": "constant_velocity.yaml",
        "sinusoidal": "sinusoidal_xy.yaml",
        "heave_h1": "heave_h1.yaml",
        "heave_h2": "heave_h2.yaml",
        "heave_h3": "heave_h3.yaml",
        "tilt_roll_pos_2deg": "tilt_roll_pos_2deg.yaml",
        "tilt_pitch_pos_2deg": "tilt_pitch_pos_2deg.yaml",
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
        "experiment_episode": {
            "method": method,
            "scenario": scenario,
            "seed": seed,
            "tracking_mode": tracking_mode,
            "prediction_horizon_s": prediction_horizon_s,
            "velocity_feedforward_gain": velocity_feedforward_gain,
            "vertical_velocity_feedforward_enabled": vertical_velocity_feedforward_enabled,
            "vertical_velocity_feedforward_gain": vertical_velocity_feedforward_gain,
            "vertical_velocity_feedforward_max_mps": vertical_velocity_feedforward_max_mps,
            "experiment_profile": experiment_profile,
            "terminal_stabilization_mode": terminal_stabilization_mode,
            "evaluator": evaluator,
        },
        **scenario_data,
        **gnss_data,
    }
    (episode_dir / "scenario_config.yaml").write_text(
        yaml.safe_dump(snapshot, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    method_snapshot = {
        "method": method,
        "tracking_mode": tracking_mode,
        "prediction_horizon_s": prediction_horizon_s,
        "velocity_feedforward_gain": velocity_feedforward_gain,
        "vertical_velocity_feedforward_enabled": vertical_velocity_feedforward_enabled,
        "vertical_velocity_feedforward_gain": vertical_velocity_feedforward_gain,
        "vertical_velocity_feedforward_max_mps": vertical_velocity_feedforward_max_mps,
        "experiment_profile": experiment_profile,
        "terminal_stabilization_mode": terminal_stabilization_mode,
        "evaluator": evaluator,
    }
    (episode_dir / "method_parameters.yaml").write_text(
        yaml.safe_dump(method_snapshot, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def evaluator_path_for_scenario(
    workspace_dir: Path,
    scenario: str,
    *,
    p8c3_touchdown: bool = False,
    use_p8c_evaluator: bool = False,
    experiment_profile: str = "touchdown",
    evaluator: str = "auto",
) -> Path:
    """根据方法、场景和 profile 自动选择 P4/P5B/P6B/P8A/P8C evaluator。"""

    explicit = {
        "p4": "evaluate_p4_bag.py",
        "p5b": "evaluate_p5b_bag.py",
        "p6b": "evaluate_p6b_touchdown.py",
        "p8a": "evaluate_p8a_heave_touchdown.py",
        "p8c": "evaluate_p8c_tilted_deck.py",
    }
    if evaluator != "auto":
        if evaluator not in explicit:
            raise ValueError(f"unsupported evaluator route: {evaluator}")
        filename = explicit[evaluator]
    elif p8c3_touchdown or use_p8c_evaluator:
        filename = "evaluate_p8c_tilted_deck.py"
    elif experiment_profile == "safe-altitude":
        filename = "evaluate_p4_bag.py"
    elif experiment_profile in {"safe-descent", "rehearsal"}:
        filename = "evaluate_p5b_bag.py"
    elif scenario.startswith("heave_h"):
        filename = "evaluate_p8a_heave_touchdown.py"
    else:
        filename = "evaluate_p6b_touchdown.py"
    return workspace_dir / "scripts" / filename


def _execute_evaluator(
    command: list[str],
    *,
    text_path: Path,
    json_path: Path,
    run_log: TextIO,
    label: str,
) -> tuple[dict[str, Any] | None, str | None]:
    """运行一个离线评测器并保存人类可读与 JSON 输出。"""

    human = subprocess.run(command, text=True, capture_output=True, check=False)
    human_output = human.stdout + human.stderr
    text_path.write_text(human_output, encoding="utf-8")
    run_log.write(f"\n===== {label} =====\n")
    run_log.write(human_output)
    run_log.flush()
    json_result = subprocess.run(
        [*command, "--json"], text=True, capture_output=True, check=False
    )
    if json_result.returncode not in (0, 2):
        return None, json_result.stderr.strip() or f"{label} failed"
    try:
        evaluation = json.loads(json_result.stdout)
        if not isinstance(evaluation, dict):
            raise ValueError("evaluation root is not an object")
        atomic_write_json(json_path, evaluation)
        evaluation = read_evaluation_json(
            json_path,
            require_touchdown_field=(
                "touchdown" in label or label in {
                    "evaluate_p6b_touchdown", "evaluate_p8a_heave_touchdown"
                }
            ),
        )
    except (json.JSONDecodeError, ValueError) as error:
        return None, str(error)
    return evaluation, None


def run_evaluator(
    workspace_dir: Path,
    scenario: str,
    seed: int,
    bag_path: Path,
    episode_dir: Path,
    run_log: TextIO,
    *,
    p8c3_touchdown: bool = False,
    experiment_profile: str = "touchdown",
    terminal_stabilization_mode: str = "disabled",
    evaluator_route: str = "auto",
) -> tuple[dict[str, Any] | None, dict[str, Any] | None, str | None]:
    """运行主评测；static/constant02 触地同时执行旧 P6B 回归。"""

    use_p8c_evaluator = bool(
        p8c3_touchdown or terminal_stabilization_mode != "disabled"
    )
    evaluator = evaluator_path_for_scenario(
        workspace_dir,
        scenario,
        p8c3_touchdown=p8c3_touchdown,
        use_p8c_evaluator=use_p8c_evaluator,
        experiment_profile=experiment_profile,
        evaluator=evaluator_route,
    )
    base_command = [sys.executable, str(evaluator), str(bag_path)]
    if evaluator.name == "evaluate_p8c_tilted_deck.py":
        base_command.extend(["--scenario", scenario, "--seed", str(seed)])
        if experiment_profile in {"safe-descent", "rehearsal"}:
            base_command.append("--p8c2-safe-descent")
        if p8c3_touchdown or experiment_profile == "touchdown":
            base_command.append("--p8c3-touchdown")
        if terminal_stabilization_mode != "disabled":
            base_command.extend(
                [
                    "--p8c4-stabilization",
                    "--p8c4-mode",
                    terminal_stabilization_mode,
                ]
            )
    elif scenario == "constant02":
        base_command.append("--require-moving-deck")
    evaluation, error = _execute_evaluator(
        base_command,
        text_path=episode_dir / "evaluation.txt",
        json_path=episode_dir / "evaluation.json",
        run_log=run_log,
        label=evaluator.stem,
    )
    if error is not None:
        return evaluation, None, error

    legacy_evaluation: dict[str, Any] | None = None
    if (
        experiment_profile == "touchdown"
        and scenario in {"static", "constant02"}
        and evaluator.name != "evaluate_p6b_touchdown.py"
    ):
        legacy = evaluator_path_for_scenario(
            workspace_dir, scenario, experiment_profile="touchdown", evaluator="p6b"
        )
        legacy_command = [sys.executable, str(legacy), str(bag_path)]
        if scenario == "constant02":
            legacy_command.append("--require-moving-deck")
        legacy_evaluation, legacy_error = _execute_evaluator(
            legacy_command,
            text_path=episode_dir / "legacy_evaluation.txt",
            json_path=episode_dir / "legacy_evaluation.json",
            run_log=run_log,
            label=legacy.stem,
        )
        if legacy_error is not None:
            return evaluation, legacy_evaluation, legacy_error
    return evaluation, legacy_evaluation, None


def evaluators_passed(
    evaluation: dict[str, Any] | None,
    legacy_evaluation: dict[str, Any] | None,
    *,
    experiment_profile: str = "touchdown",
    scenario: str = "static",
) -> bool:
    """主 evaluator 与可选 legacy evaluator 必须同时通过冻结门。"""

    if evaluation is None:
        return False
    if "positive_touchdown_passed" in evaluation:
        main_passed = evaluation.get("positive_touchdown_passed") is True
    elif "final_result" in evaluation:
        main_passed = evaluation.get("final_result") == "PASS"
    elif "horizontal_position_rmse_m" in evaluation:
        # Frozen safe-altitude gates are deliberately looser than historical
        # nominal results and are not used to authorize touchdown.
        limits = {
            "static": (0.10, 0.25),
            "constant02": (0.15, 0.30),
            "constant": (0.20, 0.45),
            "sinusoidal": (0.50, 0.80),
            "heave_h1": (0.20, 0.45),
            "heave_h2": (0.25, 0.55),
        }
        rmse_limit, max_limit = limits.get(scenario, (0.20, 0.45))
        try:
            rmse_value = float(evaluation["horizontal_position_rmse_m"])
            max_value = float(evaluation["maximum_horizontal_error_m"])
        except (KeyError, TypeError, ValueError):
            main_passed = False
        else:
            main_passed = bool(
                math.isfinite(rmse_value)
                and math.isfinite(max_value)
                and rmse_value <= rmse_limit
                and max_value <= max_limit
                and int(evaluation.get("gnss_recovery_count", 0)) == 0
                and int(evaluation.get("mpc_non_solved_status_count", 0)) == 0
                and int(evaluation.get("mpc_deadline_miss_count", 0)) == 0
            )
    elif "test_height_hold_reached" in evaluation:
        main_passed = bool(
            evaluation.get("test_height_hold_reached") is True
            and not evaluation.get("forbidden_states_seen")
            and int(evaluation.get("nav_land_command_count", 0)) == 0
            and int(evaluation.get("disarm_command_count", 0)) == 0
            and int(evaluation.get("reference_decreases_with_closed_window_count", 0)) == 0
            and float(evaluation.get("maximum_target_z_step_m", float("inf"))) <= 0.15
        )
    else:
        main_passed = False
    return bool(
        main_passed
        and (
            legacy_evaluation is None
            or legacy_evaluation.get("positive_touchdown_passed") is True
        )
    )


def run_episode(args: argparse.Namespace) -> dict[str, Any]:
    workspace_dir = args.workspace_dir.expanduser().resolve()
    if args.seed < 0 or args.seed > 0xFFFFFFFF:
        raise ValueError("seed must fit in uint32")
    if args.episode_timeout <= 0.0 or args.startup_timeout <= 0.0:
        raise ValueError("timeouts must be positive")
    method_was_explicit = hasattr(args, "method")
    method = str(getattr(args, "method", "B0")).upper()
    if method not in SUPPORTED_METHODS:
        raise ValueError(f"unsupported method: {method}")
    experiment_profile = str(getattr(args, "experiment_profile", "touchdown"))
    terminal_stabilization_mode = str(
        getattr(args, "terminal_stabilization_mode", "disabled")
    )
    evaluator_route = str(getattr(args, "evaluator", "auto"))
    if evaluator_route not in SUPPORTED_EVALUATORS:
        raise ValueError(f"unsupported evaluator route: {evaluator_route}")
    success_bag_policy = str(
        getattr(args, "success_bag_policy", "lightweight")
    )
    failure_bag_policy = str(
        getattr(args, "failure_bag_policy", "diagnostic")
    )
    if success_bag_policy not in SUPPORTED_BAG_POLICIES:
        raise ValueError("unsupported success bag policy")
    if failure_bag_policy not in SUPPORTED_BAG_POLICIES:
        raise ValueError("unsupported failure bag policy")
    prediction_horizon_s = float(getattr(args, "prediction_horizon", 0.10))
    velocity_feedforward_gain = float(getattr(args, "velocity_ff_gain", 1.0))
    vertical_ff_enabled = bool(getattr(args, "vertical_ff_enabled", True))
    vertical_ff_gain = float(getattr(args, "vertical_ff_gain", 1.0))
    vertical_ff_max = float(getattr(args, "vertical_ff_max", 0.60))
    finite_parameters = (
        prediction_horizon_s,
        velocity_feedforward_gain,
        vertical_ff_gain,
        vertical_ff_max,
    )
    if not all(math.isfinite(value) for value in finite_parameters):
        raise ValueError("method parameters must be finite")
    if not 0.0 <= prediction_horizon_s <= 0.50:
        raise ValueError("prediction horizon must be within [0, 0.50]")
    if not 0.0 <= velocity_feedforward_gain <= 5.0:
        raise ValueError("velocity feedforward gain must be within [0, 5]")
    if not 0.0 <= vertical_ff_gain <= 3.0 or not 0.0 < vertical_ff_max <= 2.0:
        raise ValueError("vertical feedforward parameters are out of range")
    if experiment_profile not in EXPERIMENT_PROFILES:
        raise ValueError(f"unsupported experiment profile: {experiment_profile}")
    if terminal_stabilization_mode not in TERMINAL_STABILIZATION_MODES:
        raise ValueError(
            f"unsupported terminal stabilization mode: {terminal_stabilization_mode}"
        )
    if experiment_profile == "touchdown" and args.touchdown_hold < 10.0:
        raise ValueError("touchdown hold must be at least 10 seconds")
    p8c3_touchdown = bool(getattr(args, "p8c3_touchdown", False))
    if p8c3_touchdown and args.scenario not in {
        "static",
        "constant02",
        "tilt_roll_pos_2deg",
        "tilt_pitch_pos_2deg",
    }:
        raise ValueError("P8C-3 mode supports static, constant02, and positive fixed +2 degree tilt only")
    positive_tilt = args.scenario in {
        "tilt_roll_pos_2deg",
        "tilt_pitch_pos_2deg",
    }
    if terminal_stabilization_mode != "disabled" and not positive_tilt:
        raise ValueError(
            "P8C-4 terminal stabilization automation supports positive fixed +2 degree tilt only"
        )
    valid_profile_mode_pairs = {
        ("safe-altitude", "shadow"),
        ("safe-descent", "shadow"),
        ("rehearsal", "rehearsal"),
        ("touchdown", "active"),
    }
    if terminal_stabilization_mode != "disabled" and (
        experiment_profile,
        terminal_stabilization_mode,
    ) not in valid_profile_mode_pairs:
        raise ValueError(
            "terminal stabilization mode does not match the staged experiment profile"
        )
    if terminal_stabilization_mode == "active":
        p8c3_touchdown = True

    completion_states, completion_duration_s = completion_requirement(
        experiment_profile, args.touchdown_hold
    )

    batch_id = args.batch_id or make_batch_id("p9_single")
    episode_id = args.episode_id or make_episode_id(
        batch_id,
        args.scenario,
        1,
        args.seed,
        method=method if method_was_explicit else None,
        profile=experiment_profile if method_was_explicit else None,
    )
    output_root = args.output_directory.expanduser().resolve()
    episode_dir = output_root / episode_id
    bag_path = episode_dir / "bag"
    tracking_mode = getattr(
        args, "tracking_mode", "PREDICTED_POSITION_VELOCITY_FF"
    )
    if tracking_mode not in TRACKING_MODES:
        raise ValueError(f"unsupported tracking mode: {tracking_mode}")
    if method_was_explicit:
        if not combination_is_applicable(method, args.scenario, experiment_profile):
            raise ValueError(
                f"method/scenario/profile is not safety-authorized: "
                f"{method}/{args.scenario}/{experiment_profile}"
            )
        expected_tracking = (
            "RELATIVE_MPC" if method in {"B3", "B4"}
            else "PREDICTED_POSITION_VELOCITY_FF"
        )
        if tracking_mode != expected_tracking:
            raise ValueError(f"{method} requires tracking mode {expected_tracking}")
        if method == "B1" and not math.isclose(prediction_horizon_s, 0.0):
            raise ValueError("B1 requires prediction horizon 0.0")
        if method != "B1" and not math.isclose(prediction_horizon_s, 0.10):
            raise ValueError(f"{method} requires prediction horizon 0.10")
        if method == "B2" and not math.isclose(velocity_feedforward_gain, 0.0):
            raise ValueError("B2 requires velocity feedforward gain 0.0")
        if method != "B2" and not math.isclose(velocity_feedforward_gain, 1.0):
            raise ValueError(f"{method} requires velocity feedforward gain 1.0")
        if method == "B4" and not vertical_ff_enabled:
            raise ValueError("B4 requires validated vertical feedforward")
        if method == "B5" and terminal_stabilization_mode == "disabled":
            raise ValueError("B5 requires staged terminal stabilization")
        if method != "B5" and terminal_stabilization_mode != "disabled":
            raise ValueError("terminal stabilization is only authorized for B5")
    start_command = build_start_command(
        workspace_dir,
        args.scenario,
        args.seed,
        bag_path,
        args.camera_model,
        args.record_camera_debug,
        tracking_mode,
        experiment_profile,
        terminal_stabilization_mode,
        prediction_horizon_s,
        velocity_feedforward_gain,
        vertical_ff_enabled,
        vertical_ff_gain,
        vertical_ff_max,
    )
    if args.dry_run:
        result = {
            "dry_run": True,
            "batch_id": batch_id,
            "episode_id": episode_id,
            "episode_directory": str(episode_dir),
            "method": method,
            "scenario": args.scenario,
            "seed": args.seed,
            "command": start_command,
            "tracking_mode": tracking_mode,
            "prediction_horizon_s": prediction_horizon_s,
            "velocity_feedforward_gain": velocity_feedforward_gain,
            "vertical_velocity_feedforward_enabled": vertical_ff_enabled,
            "vertical_velocity_feedforward_gain": vertical_ff_gain,
            "vertical_velocity_feedforward_max_mps": vertical_ff_max,
            "evaluator": evaluator_route,
            "p8c3_touchdown": p8c3_touchdown,
            "experiment_profile": experiment_profile,
            "terminal_stabilization_mode": terminal_stabilization_mode,
            "completion_states": sorted(completion_states),
            "completion_duration_s": completion_duration_s,
        }
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return result
    archived_previous_attempt: Path | None = None
    if episode_dir.exists():
        retry_failure = bool(getattr(args, "retry_existing_failure", False))
        rerun_after_change = bool(getattr(args, "rerun_after_code_change", False))
        if retry_failure and rerun_after_change:
            raise ValueError(
                "--retry-existing-failure and --rerun-after-code-change are mutually exclusive"
            )
        if retry_failure:
            archived_previous_attempt = archive_completed_episode(
                episode_dir, require_failure=True
            )
        elif rerun_after_change:
            archived_previous_attempt = archive_completed_episode(
                episode_dir, require_failure=False
            )
        else:
            raise ValueError(f"episode directory already exists: {episode_dir}")
    episode_dir.mkdir(parents=True)

    git_commit, dirty_worktree = git_state(workspace_dir)
    start_wall_time = utc_now_iso()
    start_monotonic = time.monotonic()
    manifest: dict[str, Any] = {
        "episode_id": episode_id,
        "batch_id": batch_id,
        "method": method,
        "scenario": args.scenario,
        "profile": experiment_profile,
        "seed": args.seed,
        "git_commit": git_commit,
        "dirty_worktree": dirty_worktree,
        "start_wall_time": start_wall_time,
        "end_wall_time": None,
        "duration_s": None,
        "camera_model": args.camera_model,
        "tracking_mode": tracking_mode,
        "prediction_horizon_s": prediction_horizon_s,
        "velocity_feedforward_gain": velocity_feedforward_gain,
        "vertical_velocity_feedforward_enabled": vertical_ff_enabled,
        "vertical_velocity_feedforward_gain": vertical_ff_gain,
        "vertical_velocity_feedforward_max_mps": vertical_ff_max,
        "evaluator": evaluator_route,
        "success_bag_policy": success_bag_policy,
        "failure_bag_policy": failure_bag_policy,
        "record_camera_debug": args.record_camera_debug,
        "p8c3_touchdown": p8c3_touchdown,
        "experiment_profile": experiment_profile,
        "terminal_stabilization_mode": terminal_stabilization_mode,
        "completion_states": sorted(completion_states),
        "completion_duration_s": completion_duration_s,
        "required_touchdown_hold_s": args.touchdown_hold,
        "post_hold_recording_margin_s": POST_HOLD_RECORDING_MARGIN_S,
        "start_command": start_command,
        "exit_code": None,
        "success": False,
        "failure_reason": "UNKNOWN",
        "failure_detail": None,
        "bag_path": str(bag_path),
        "evaluation_path": None,
        "legacy_evaluation_path": None,
        "state_sequence": [],
        "completed": False,
        "archived_previous_attempt": (
            str(archived_previous_attempt)
            if archived_previous_attempt is not None
            else None
        ),
    }
    atomic_write_json(episode_dir / "manifest.json", manifest)
    (episode_dir / "command.txt").write_text(
        shlex.join(start_command) + "\n", encoding="utf-8"
    )
    (episode_dir / "run_metadata.txt").write_text(
        "\n".join(
            (
                f"method={method}",
                f"scenario={args.scenario}",
                f"profile={experiment_profile}",
                f"seed={args.seed}",
                f"git_commit={git_commit}",
                f"git_dirty={str(dirty_worktree).lower()}",
                f"camera_model={args.camera_model}",
                f"tracking_mode={tracking_mode}",
                f"prediction_horizon_s={prediction_horizon_s}",
                f"velocity_feedforward_gain={velocity_feedforward_gain}",
                f"vertical_velocity_feedforward_enabled={str(vertical_ff_enabled).lower()}",
                f"vertical_velocity_feedforward_gain={vertical_ff_gain}",
                f"vertical_velocity_feedforward_max_mps={vertical_ff_max}",
                f"evaluator={evaluator_route}",
                f"success_bag_policy={success_bag_policy}",
                f"failure_bag_policy={failure_bag_policy}",
                f"p8c3_touchdown={str(p8c3_touchdown).lower()}",
                f"experiment_profile={experiment_profile}",
                f"terminal_stabilization_mode={terminal_stabilization_mode}",
                "relative_descent=" + str(
                    experiment_profile in {"safe-descent", "rehearsal", "touchdown"}
                ).lower(),
                "descent_test_height_m=0.50",
                "final_descent=" + str(experiment_profile == "touchdown").lower(),
                f"touchdown_hold_required_s={args.touchdown_hold}",
                f"post_hold_recording_margin_s={POST_HOLD_RECORDING_MARGIN_S}",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    snapshot_configs(
        workspace_dir,
        episode_dir,
        args.scenario,
        args.seed,
        tracking_mode,
        experiment_profile,
        terminal_stabilization_mode,
        method=method,
        prediction_horizon_s=prediction_horizon_s,
        velocity_feedforward_gain=velocity_feedforward_gain,
        vertical_velocity_feedforward_enabled=vertical_ff_enabled,
        vertical_velocity_feedforward_gain=vertical_ff_gain,
        vertical_velocity_feedforward_max_mps=vertical_ff_max,
        evaluator=evaluator_route,
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
    completion_start_monotonic: float | None = None
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
                            completion_start_monotonic = (
                                now if state in completion_states else None
                            )
                        if state == "ABORT":
                            event = "PX4_ABORT"
                            event_detail = "controller entered ABORT"
                            break
                        if recovery_after_terminal_phase(state_sequence, state):
                            event = "RECOVERY_LIMIT"
                            event_detail = (
                                "controller entered recovery after touchdown candidate/hold"
                            )
                            break
                if (
                    current_state in completion_states
                    and completion_start_monotonic is not None
                    and now - completion_start_monotonic >= completion_duration_s
                ):
                    event = "NONE"
                    event_detail = (
                        f"{current_state} sustained for {completion_duration_s:.1f} s "
                        f"under {experiment_profile} completion profile"
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
        legacy_evaluation: dict[str, Any] | None = None
        evaluation_error: str | None = None
        if bag_path.is_dir():
            evaluation, legacy_evaluation, evaluation_error = run_evaluator(
                workspace_dir,
                args.scenario,
                args.seed,
                bag_path,
                episode_dir,
                run_log,
                p8c3_touchdown=p8c3_touchdown,
                experiment_profile=experiment_profile,
                terminal_stabilization_mode=terminal_stabilization_mode,
                evaluator_route=evaluator_route,
            )
            if evaluation is not None:
                manifest["evaluation_path"] = str(episode_dir / "evaluation.json")
            if legacy_evaluation is not None:
                manifest["legacy_evaluation_path"] = str(
                    episode_dir / "legacy_evaluation.json"
                )
        elif event == "NONE":
            evaluation_error = "bag directory was not created"

        evaluator_passed = evaluators_passed(
            evaluation,
            legacy_evaluation,
            experiment_profile=experiment_profile,
            scenario=args.scenario,
        )
        success = bool(event == "NONE" and evaluator_passed and cleanup_ok)
        log_text = (episode_dir / "run.log").read_text(
            encoding="utf-8", errors="replace"
        )
        classification_event = event
        if event == "NONE" and evaluation_error:
            classification_event = "EVALUATION_ERROR"
        elif event == "NONE" and not evaluator_passed:
            classification_event = "SAFETY_GATE_FAILURE"
        failure_reason = classify_failure(
            success=success,
            event=classification_event,
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
        (episode_dir / "process_cleanup.txt").write_text(
            (
                "cleanup=PASS\nresidual_processes=0\n"
                if cleanup_ok
                else "cleanup=FAIL\n" + "\n".join(cleanup_residuals) + "\n"
            ),
            encoding="utf-8",
        )

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
        manifest = run_episode(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 2
    return 0 if manifest.get("dry_run") is True or manifest.get("success") is True else 2


if __name__ == "__main__":
    raise SystemExit(main())
