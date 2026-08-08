#!/usr/bin/env python3
"""批量实验配置、适用矩阵、结果分类与统计公共工具。"""

from __future__ import annotations

import csv
import json
import math
import os
import statistics
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml

SUPPORTED_SCENARIOS = (
    "static",
    "constant02",
    "constant",
    "sinusoidal",
    "heave_h1",
    "heave_h2",
    "heave_h3",
    "tilt_roll_pos_2deg",
    "tilt_pitch_pos_2deg",
)
FORBIDDEN_EVALUATION_SCENARIOS = (
    "tilt_roll_neg_2deg",
    "tilt_pitch_neg_2deg",
    "rollpitch",
    "combined",
)
SUPPORTED_METHODS = ("B0", "B1", "B2", "B3", "B4", "B5")
SUPPORTED_PROFILES = ("safe-altitude", "safe-descent", "rehearsal", "touchdown")
SUPPORTED_EVALUATORS = ("auto", "horizontal_tracking", "relative_descent", "final_descent_touchdown", "heave_touchdown", "tilted_deck")
SUPPORTED_APPLICABILITY = ("APPLICABLE", "NOT_APPLICABLE")
SUPPORTED_TERMINAL_MODES = ("disabled", "shadow", "rehearsal", "active")
SUPPORTED_TRACKING_MODES = (
    "PREDICTED_POSITION_VELOCITY_FF",
    "RELATIVE_MPC",
)
SUPPORTED_BAG_POLICIES = ("lightweight", "diagnostic")

FAILURE_TYPES = (
    "NONE",
    "STARTUP_FAILURE",
    "PX4_TIMEOUT",
    "PROCESS_EXITED",
    "ARUCO_NOT_ACQUIRED",
    "VISION_LOST",
    "LANDING_WINDOW_TIMEOUT",
    "TRACKING_DIVERGED",
    "RECOVERY_LIMIT",
    "TOUCHDOWN_NOT_CONFIRMED",
    "SAFETY_GATE_FAILURE",
    "PX4_ABORT",
    "EPISODE_TIMEOUT",
    "EVALUATION_ERROR",
    "CLEANUP_FAILURE",
    "CONFIGURATION_ERROR",
    "UNKNOWN",
)

METRIC_FIELDS = (
    "landing_time_s",
    "horizontal_error_rmse_m",
    "horizontal_error_max_m",
    "touchdown_vertical_speed_mps",
    "candidate_to_confirm_delay_s",
    "hold_duration_s",
    "recovery_count",
    "marker_switch_count",
    "detach_count",
    "secondary_contact_count",
    "candidate_repeat_count",
    "deck_vertical_span_final_m",
    "hold_relative_height_span_m",
    "hold_relative_vertical_velocity_p95_mps",
    "normal_tracking_error_rmse_deg",
    "normal_tracking_error_p95_deg",
    "terminal_command_tilt_max_deg",
    "terminal_command_tilt_slew_p100_degps",
    "combined_horizontal_acceleration_max_mps2",
    "touchdown_slip_m",
    "hold_tangential_velocity_p95_mps",
    "attitude_divergence_increment_deg",
    "fallback_count",
    "terminal_stabilization_activation_count",
    "solver_success_rate",
    "solver_failure_count",
    "solve_time_mean_ms",
    "solve_time_p95_ms",
    "constraint_violation_count",
)

# Frozen paper evaluation safety authorization. Absence means NOT_APPLICABLE.
APPLICABLE_COMBINATIONS = frozenset(
    {
        # B0 complete adaptive rule-based tracking.
        *(('B0', scenario, 'safe-altitude') for scenario in (
            'static', 'constant02', 'constant', 'sinusoidal', 'heave_h1', 'heave_h2'
        )),
        *(('B0', scenario, 'safe-descent') for scenario in (
            'static', 'constant02', 'heave_h1', 'heave_h2'
        )),
        *(('B0', scenario, 'touchdown') for scenario in (
            'static', 'constant02', 'heave_h1', 'heave_h2'
        )),
        # adaptive rule-based tracking ablations are authorized only at safe altitude.
        ('B1', 'static', 'safe-altitude'),
        ('B1', 'constant02', 'safe-altitude'),
        ('B2', 'static', 'safe-altitude'),
        ('B2', 'constant02', 'safe-altitude'),
        # Horizontal relative MPC authorization from relative MPC.
        ('B3', 'static', 'safe-altitude'),
        ('B3', 'constant02', 'safe-altitude'),
        ('B3', 'constant', 'safe-altitude'),
        ('B3', 'sinusoidal', 'safe-altitude'),
        ('B3', 'constant02', 'safe-descent'),
        ('B3', 'constant02', 'touchdown'),
        # MPC + validated H1 heave semantics.
        ('B4', 'heave_h1', 'safe-altitude'),
        ('B4', 'heave_h1', 'safe-descent'),
        ('B4', 'heave_h1', 'touchdown'),
        # terminal contact stabilization fixed positive T1 staged authorization.
        *(('B5', scenario, profile) for scenario in (
            'tilt_roll_pos_2deg', 'tilt_pitch_pos_2deg'
        ) for profile in SUPPORTED_PROFILES),
    }
)


@dataclass(frozen=True)
class ScenarioSpec:
    """Scenario view used by simple touchdown batch configs."""

    scenario: str
    repetitions: int
    seeds: tuple[int, ...]


@dataclass(frozen=True)
class ExperimentSpec:
    """A frozen paper evaluation method/scenario/profile combination."""

    method: str
    scenario: str
    profile: str
    repetitions: int
    seeds: tuple[int, ...]
    tracking_mode: str
    prediction_horizon_s: float
    velocity_feedforward_gain: float
    vertical_velocity_feedforward_enabled: bool
    vertical_velocity_feedforward_gain: float
    vertical_velocity_feedforward_max_mps: float
    terminal_stabilization_mode: str
    evaluator: str
    applicability: str
    record_camera_debug: bool
    parameter_overrides: Mapping[str, Any]


@dataclass(frozen=True)
class BatchConfig:
    """Unified batch experiment configuration."""

    name: str
    output_root: Path
    episode_timeout_s: float
    startup_timeout_s: float
    touchdown_hold_s: float
    camera_model: str
    record_camera_debug: bool
    scenarios: tuple[ScenarioSpec, ...]
    experiments: tuple[ExperimentSpec, ...]
    success_bag_policy: str = "lightweight"
    failure_bag_policy: str = "diagnostic"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        temporary_path = Path(handle.name)
    os.replace(temporary_path, path)


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Mapping[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def expand_seeds(repetitions: int, seeds: Sequence[int] | None) -> tuple[int, ...]:
    if repetitions <= 0:
        raise ValueError("repetitions must be positive")
    normalized = tuple(int(seed) for seed in (seeds or ()))
    if not normalized:
        normalized = tuple(range(1, repetitions + 1))
    elif len(normalized) == 1 and repetitions > 1:
        normalized = tuple(normalized[0] + offset for offset in range(repetitions))
    elif len(normalized) != repetitions:
        raise ValueError("seeds must contain one value or exactly repetitions values")
    if any(seed < 0 or seed > 0xFFFFFFFF for seed in normalized):
        raise ValueError("seeds must fit in uint32")
    if len(set(normalized)) != len(normalized):
        raise ValueError("seeds must be unique within an experiment")
    return normalized


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def _finite_float(value: Any, label: str, *, minimum: float | None = None,
                  maximum: float | None = None, positive: bool = False) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must be numeric") from error
    if not math.isfinite(numeric):
        raise ValueError(f"{label} must be finite")
    if positive and numeric <= 0.0:
        raise ValueError(f"{label} must be positive")
    if minimum is not None and numeric < minimum:
        raise ValueError(f"{label} must be >= {minimum}")
    if maximum is not None and numeric > maximum:
        raise ValueError(f"{label} must be <= {maximum}")
    return numeric


def method_defaults(method: str, profile: str) -> dict[str, Any]:
    if method not in SUPPORTED_METHODS:
        raise ValueError(f"unknown method '{method}'")
    if profile not in SUPPORTED_PROFILES:
        raise ValueError(f"illegal profile '{profile}'")
    result: dict[str, Any] = {
        "tracking_mode": "PREDICTED_POSITION_VELOCITY_FF",
        "prediction_horizon_s": 0.10,
        "velocity_feedforward_gain": 1.0,
        "vertical_velocity_feedforward_enabled": True,
        "vertical_velocity_feedforward_gain": 1.0,
        "vertical_velocity_feedforward_max_mps": 0.60,
        "terminal_stabilization_mode": "disabled",
    }
    if method == "B1":
        result["prediction_horizon_s"] = 0.0
    elif method == "B2":
        result["velocity_feedforward_gain"] = 0.0
    elif method in {"B3", "B4"}:
        result["tracking_mode"] = "RELATIVE_MPC"
    elif method == "B5":
        result["terminal_stabilization_mode"] = {
            "safe-altitude": "shadow",
            "safe-descent": "shadow",
            "rehearsal": "rehearsal",
            "touchdown": "active",
        }[profile]
    return result


def combination_is_applicable(method: str, scenario: str, profile: str) -> bool:
    return (method, scenario, profile) in APPLICABLE_COMBINATIONS


def _validate_parameter_overrides(value: Any, label: str) -> Mapping[str, Any]:
    if value is None:
        return {}
    overrides = _require_mapping(value, label)
    supported = {
        "prediction_horizon_s",
        "velocity_feedforward_gain",
        "vertical_velocity_feedforward_enabled",
        "vertical_velocity_feedforward_gain",
        "vertical_velocity_feedforward_max_mps",
    }
    unknown = sorted(set(overrides) - supported)
    if unknown:
        raise ValueError(f"unsupported parameter_overrides keys: {unknown}")
    for key, raw in overrides.items():
        if key == "vertical_velocity_feedforward_enabled":
            if not isinstance(raw, bool):
                raise ValueError(f"{label}.{key} must be boolean")
        else:
            _finite_float(raw, f"{label}.{key}")
    return dict(overrides)


def _parse_experiment(
    entry: Mapping[str, Any],
    *,
    index: int,
    default_record_debug: bool,
) -> ExperimentSpec:
    label = f"experiments[{index}]"
    method = str(entry.get("method", "")).strip().upper()
    scenario = str(entry.get("scenario", "")).strip()
    profile = str(entry.get("profile", "touchdown")).strip()
    if method not in SUPPORTED_METHODS:
        raise ValueError(f"unknown method '{method}' in {label}")
    if scenario in FORBIDDEN_EVALUATION_SCENARIOS:
        raise ValueError(f"forbidden negative/dynamic tilt scenario '{scenario}' in {label}")
    if scenario not in SUPPORTED_SCENARIOS:
        raise ValueError(f"unknown scenario '{scenario}' in {label}")
    if profile not in SUPPORTED_PROFILES:
        raise ValueError(f"illegal profile '{profile}' in {label}")
    repetitions = int(entry.get("repetitions", 0))
    seeds_value = entry.get("seeds")
    if seeds_value is not None and not isinstance(seeds_value, list):
        raise ValueError(f"seeds for {label} must be a list")
    seeds = expand_seeds(repetitions, seeds_value)
    applicability = str(entry.get("applicability", "APPLICABLE")).strip().upper()
    if applicability not in SUPPORTED_APPLICABILITY:
        raise ValueError(f"invalid applicability '{applicability}' in {label}")
    authorized = combination_is_applicable(method, scenario, profile)
    if applicability == "APPLICABLE" and not authorized:
        raise ValueError(
            f"method/scenario/profile is not safety-authorized: {method}/{scenario}/{profile}"
        )
    if applicability == "NOT_APPLICABLE" and authorized:
        # Explicit NA is allowed for a deliberately documented matrix row.
        pass

    defaults = method_defaults(method, profile)
    parameter_overrides = _validate_parameter_overrides(
        entry.get("parameter_overrides"), f"{label}.parameter_overrides"
    )
    values = {**defaults, **parameter_overrides}
    for key in defaults:
        if key in entry:
            values[key] = entry[key]

    tracking_mode = str(values["tracking_mode"])
    if tracking_mode not in SUPPORTED_TRACKING_MODES:
        raise ValueError(f"illegal tracking mode '{tracking_mode}' in {label}")
    prediction_horizon_s = _finite_float(
        values["prediction_horizon_s"], f"{label}.prediction_horizon_s",
        minimum=0.0, maximum=0.50
    )
    velocity_feedforward_gain = _finite_float(
        values["velocity_feedforward_gain"], f"{label}.velocity_feedforward_gain",
        minimum=0.0, maximum=5.0
    )
    vertical_enabled = values["vertical_velocity_feedforward_enabled"]
    if not isinstance(vertical_enabled, bool):
        raise ValueError(f"{label}.vertical_velocity_feedforward_enabled must be boolean")
    vertical_gain = _finite_float(
        values["vertical_velocity_feedforward_gain"],
        f"{label}.vertical_velocity_feedforward_gain", minimum=0.0, maximum=3.0
    )
    vertical_max = _finite_float(
        values["vertical_velocity_feedforward_max_mps"],
        f"{label}.vertical_velocity_feedforward_max_mps", positive=True, maximum=2.0
    )
    terminal_mode = str(values["terminal_stabilization_mode"])
    if terminal_mode not in SUPPORTED_TERMINAL_MODES:
        raise ValueError(f"illegal terminal stabilization mode '{terminal_mode}' in {label}")
    evaluator = str(entry.get("evaluator", "auto"))
    if evaluator not in SUPPORTED_EVALUATORS:
        raise ValueError(f"unknown evaluator '{evaluator}' in {label}")

    # Method identity is frozen: explicit values may restate but not redefine it.
    expected = method_defaults(method, profile)
    identity_values = {
        "tracking_mode": tracking_mode,
        "prediction_horizon_s": prediction_horizon_s,
        "velocity_feedforward_gain": velocity_feedforward_gain,
        "terminal_stabilization_mode": terminal_mode,
    }
    for key, actual in identity_values.items():
        expected_value = expected[key]
        if isinstance(expected_value, float):
            if not math.isclose(float(actual), expected_value, abs_tol=1.0e-12):
                raise ValueError(f"{method} cannot override frozen {key}")
        elif actual != expected_value:
            raise ValueError(f"{method} cannot override frozen {key}")
    if method == "B4" and not vertical_enabled:
        raise ValueError("B4 requires validated vertical velocity feedforward")
    if method == "B5" and scenario not in {
        "tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"
    }:
        raise ValueError("B5 is restricted to fixed positive T1 scenarios")
    if method != "B5" and terminal_mode != "disabled":
        raise ValueError("terminal stabilization is only valid for B5")

    return ExperimentSpec(
        method=method,
        scenario=scenario,
        profile=profile,
        repetitions=repetitions,
        seeds=seeds,
        tracking_mode=tracking_mode,
        prediction_horizon_s=prediction_horizon_s,
        velocity_feedforward_gain=velocity_feedforward_gain,
        vertical_velocity_feedforward_enabled=vertical_enabled,
        vertical_velocity_feedforward_gain=vertical_gain,
        vertical_velocity_feedforward_max_mps=vertical_max,
        terminal_stabilization_mode=terminal_mode,
        evaluator=evaluator,
        applicability=applicability,
        record_camera_debug=bool(entry.get("record_camera_debug", default_record_debug)),
        parameter_overrides=parameter_overrides,
    )


def load_batch_config(path: Path, workspace_dir: Path | None = None) -> BatchConfig:
    """Read and validate an experiment YAML."""

    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"failed to read YAML config: {error}") from error
    root = _require_mapping(raw, "config")
    name = str(root.get("name", path.stem)).strip()
    if not name:
        raise ValueError("name must not be empty")
    output_root_raw = Path(str(root.get("output_root", "results"))).expanduser()
    if not output_root_raw.is_absolute():
        base = workspace_dir if workspace_dir is not None else path.parent
        output_root_raw = (base / output_root_raw).resolve()
    episode_timeout_s = _finite_float(
        root.get("episode_timeout_s", 600.0), "episode_timeout_s", positive=True
    )
    startup_timeout_s = _finite_float(
        root.get("startup_timeout_s", 120.0), "startup_timeout_s", positive=True
    )
    touchdown_hold_s = _finite_float(
        root.get("touchdown_hold_s", 10.0), "touchdown_hold_s", minimum=10.0
    )
    camera_model = str(root.get("camera_model", "close-range"))
    if camera_model not in ("close-range", "px4-default"):
        raise ValueError("camera_model must be close-range or px4-default")
    record_camera_debug = bool(root.get("record_camera_debug", False))
    success_bag_policy = str(root.get("success_bag_policy", "lightweight"))
    failure_bag_policy = str(root.get("failure_bag_policy", "diagnostic"))
    if success_bag_policy not in SUPPORTED_BAG_POLICIES:
        raise ValueError("invalid success_bag_policy")
    if failure_bag_policy not in SUPPORTED_BAG_POLICIES:
        raise ValueError("invalid failure_bag_policy")

    experiments: list[ExperimentSpec] = []
    legacy_scenarios: list[ScenarioSpec] = []
    experiments_raw = root.get("experiments")
    scenarios_raw = root.get("scenarios")
    if experiments_raw is not None and scenarios_raw is not None:
        raise ValueError("config must use either experiments or legacy scenarios, not both")
    if experiments_raw is not None:
        if not isinstance(experiments_raw, list) or not experiments_raw:
            raise ValueError("experiments must be a non-empty list")
        seen: set[tuple[str, str, str]] = set()
        for index, item in enumerate(experiments_raw):
            entry = _require_mapping(item, f"experiments[{index}]")
            spec = _parse_experiment(
                entry, index=index, default_record_debug=record_camera_debug
            )
            key = (spec.method, spec.scenario, spec.profile)
            if key in seen:
                raise ValueError(f"duplicate method/scenario/profile combination: {key}")
            seen.add(key)
            experiments.append(spec)
            legacy_scenarios.append(
                ScenarioSpec(spec.scenario, spec.repetitions, spec.seeds)
            )
    else:
        if not isinstance(scenarios_raw, list) or not scenarios_raw:
            raise ValueError("scenarios must be a non-empty list")
        seen_scenarios: set[str] = set()
        for index, item in enumerate(scenarios_raw):
            entry = _require_mapping(item, f"scenarios[{index}]")
            scenario = str(entry.get("scenario", ""))
            if scenario not in SUPPORTED_SCENARIOS:
                raise ValueError(
                    f"unsupported scenario '{scenario}'; automation supports {SUPPORTED_SCENARIOS}"
                )
            if scenario in seen_scenarios:
                raise ValueError(f"duplicate scenario '{scenario}'")
            seen_scenarios.add(scenario)
            repetitions = int(entry.get("repetitions", 0))
            seeds_value = entry.get("seeds")
            if seeds_value is not None and not isinstance(seeds_value, list):
                raise ValueError(f"seeds for {scenario} must be a list")
            seeds = expand_seeds(repetitions, seeds_value)
            legacy_scenarios.append(ScenarioSpec(scenario, repetitions, seeds))
            synthetic = {
                "method": (
                    "B5"
                    if scenario in {"tilt_roll_pos_2deg", "tilt_pitch_pos_2deg"}
                    else "B0"
                ),
                "scenario": scenario,
                "profile": "touchdown",
                "repetitions": repetitions,
                "seeds": list(seeds),
                "record_camera_debug": record_camera_debug,
            }
            experiments.append(
                _parse_experiment(
                    synthetic, index=index, default_record_debug=record_camera_debug
                )
            )
    return BatchConfig(
        name=name,
        output_root=output_root_raw,
        episode_timeout_s=episode_timeout_s,
        startup_timeout_s=startup_timeout_s,
        touchdown_hold_s=touchdown_hold_s,
        camera_model=camera_model,
        record_camera_debug=record_camera_debug,
        scenarios=tuple(legacy_scenarios),
        experiments=tuple(experiments),
        success_bag_policy=success_bag_policy,
        failure_bag_policy=failure_bag_policy,
    )


def make_batch_id(name: str, now: datetime | None = None) -> str:
    current = now or datetime.now()
    safe_name = "".join(
        character if character.isalnum() or character in "-_" else "-"
        for character in name
    )
    safe_name = safe_name.strip("-") or "paper_evaluation"
    return f"{safe_name}_{current.strftime('%Y%m%d_%H%M%S')}"


def make_episode_id(
    batch_id: str,
    scenario: str,
    repetition: int,
    seed: int,
    method: str | None = None,
    profile: str | None = None,
) -> str:
    if scenario not in SUPPORTED_SCENARIOS:
        raise ValueError(f"unsupported scenario: {scenario}")
    if repetition <= 0:
        raise ValueError("repetition must be positive")
    if method is None and profile is None:
        return f"{batch_id}_{scenario}_r{repetition:03d}_s{seed:010d}"
    normalized_method = str(method or "B0").upper()
    normalized_profile = str(profile or "touchdown")
    if normalized_method not in SUPPORTED_METHODS:
        raise ValueError(f"unsupported method: {normalized_method}")
    if normalized_profile not in SUPPORTED_PROFILES:
        raise ValueError(f"unsupported profile: {normalized_profile}")
    profile_token = normalized_profile.replace("-", "_")
    return (
        f"{batch_id}_{normalized_method}_{scenario}_{profile_token}_"
        f"r{repetition:03d}_s{seed:010d}"
    )


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"failed to read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def read_evaluation_json(
    path: Path, *, require_touchdown_field: bool = True
) -> dict[str, Any]:
    result = read_json(path)
    if require_touchdown_field:
        if "positive_touchdown_passed" not in result:
            raise ValueError("evaluation JSON is missing positive_touchdown_passed")
        if not isinstance(result["positive_touchdown_passed"], bool):
            raise ValueError("positive_touchdown_passed must be boolean")
    return result


def episode_result_complete(
    episode_dir: Path,
    *,
    expected_git_commit: str | None = None,
    expected_dirty_worktree: bool | None = None,
) -> bool:
    manifest_path = episode_dir / "manifest.json"
    if not manifest_path.is_file():
        return False
    try:
        manifest = read_json(manifest_path)
    except ValueError:
        return False
    if manifest.get("completed") is not True:
        return False
    if not isinstance(manifest.get("success"), bool):
        return False
    if manifest.get("failure_reason") not in FAILURE_TYPES:
        return False
    if expected_git_commit is not None and manifest.get("git_commit") != expected_git_commit:
        return False
    if (
        expected_dirty_worktree is not None
        and manifest.get("dirty_worktree") is not expected_dirty_worktree
    ):
        return False
    evaluation_path = manifest.get("evaluation_path")
    if evaluation_path and not Path(str(evaluation_path)).is_file():
        return False
    return True


def classify_failure(
    *,
    success: bool,
    event: str | None = None,
    state_sequence: Sequence[str] = (),
    evaluation: Mapping[str, Any] | None = None,
    log_text: str = "",
    cleanup_ok: bool = True,
) -> str:
    if success:
        return "NONE" if cleanup_ok else "CLEANUP_FAILURE"
    if not cleanup_ok:
        return "CLEANUP_FAILURE"
    normalized_event = (event or "").upper()
    states = tuple(str(state) for state in state_sequence)
    lower_log = log_text.lower()
    if "ABORT" in states:
        return "PX4_ABORT"
    if normalized_event == "EPISODE_TIMEOUT":
        if any(state in ("FINAL_DESCENT", "TOUCHDOWN_CANDIDATE_HOLD") for state in states):
            return "TOUCHDOWN_NOT_CONFIRMED"
        return "EPISODE_TIMEOUT"
    if normalized_event in FAILURE_TYPES and normalized_event != "NONE":
        return normalized_event
    if "safety gate" in lower_log or "hard gate" in lower_log:
        return "SAFETY_GATE_FAILURE"
    if "px4 abort" in lower_log or "controller entered abort" in lower_log:
        return "PX4_ABORT"
    keyword_types = (
        ("recovery limit", "RECOVERY_LIMIT"),
        ("tracking diverged", "TRACKING_DIVERGED"),
        ("landing window timeout", "LANDING_WINDOW_TIMEOUT"),
        ("aruco not acquired", "ARUCO_NOT_ACQUIRED"),
        ("vision lost", "VISION_LOST"),
    )
    for keyword, failure_type in keyword_types:
        if keyword in lower_log:
            return failure_type
    if evaluation is not None:
        if evaluation.get("positive_touchdown_passed") is True:
            return "NONE"
        if "positive_touchdown_passed" in evaluation:
            if evaluation.get("confirmed_start_s") is None:
                return "TOUCHDOWN_NOT_CONFIRMED"
            return "SAFETY_GATE_FAILURE"
        if evaluation.get("final_result") == "FAIL":
            return "SAFETY_GATE_FAILURE"
        return "EVALUATION_ERROR"
    return "UNKNOWN"


def percentile(values: Sequence[float], probability: float) -> float | None:
    if not values:
        return None
    if probability < 0.0 or probability > 1.0:
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


def summarize_values(values: Sequence[float]) -> dict[str, Any]:
    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0,
            "mean": None,
            "stddev": None,
            "median": None,
            "p90": None,
            "p95": None,
            "min": None,
            "max": None,
        }
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "stddev": statistics.pstdev(finite),
        "median": statistics.median(finite),
        "p90": percentile(finite, 0.90),
        "p95": percentile(finite, 0.95),
        "min": min(finite),
        "max": max(finite),
    }


METRIC_ALIASES: Mapping[str, tuple[str, ...]] = {
    "landing_time_s": ("landing_time_s", "touchdown_confirmed_time_s", "descent_interval_s"),
    "horizontal_error_rmse_m": (
        "horizontal_error_rmse_m", "horizontal_position_rmse_m"
    ),
    "horizontal_error_max_m": (
        "horizontal_error_max_m", "maximum_horizontal_error_m"
    ),
    "touchdown_vertical_speed_mps": (
        "touchdown_vertical_speed_mps", "touchdown_relative_vertical_velocity_mps",
        "touchdown_normal_relative_velocity_mps"
    ),
    "candidate_to_confirm_delay_s": ("candidate_to_confirm_delay_s", "candidate_duration_s"),
    "hold_duration_s": ("hold_duration_s", "touchdown_hold_duration_s"),
    "recovery_count": ("recovery_count", "gnss_recovery_count"),
    "marker_switch_count": ("marker_switch_count",),
    "detach_count": ("detach_count",),
    "secondary_contact_count": ("secondary_contact_count",),
    "candidate_repeat_count": ("candidate_repeat_count",),
    "deck_vertical_span_final_m": ("deck_vertical_span_final_m",),
    "hold_relative_height_span_m": ("hold_relative_height_span_m",),
    "hold_relative_vertical_velocity_p95_mps": (
        "hold_relative_vertical_velocity_p95_mps", "relative_vertical_velocity_p95_mps"
    ),
    "normal_tracking_error_rmse_deg": (
        "normal_tracking_error_rmse_deg", "attitude_tracking_rmse_deg", "normal_rmse_deg"
    ),
    "normal_tracking_error_p95_deg": (
        "normal_tracking_error_p95_deg", "attitude_tracking_p95_deg", "normal_p95_deg"
    ),
    "terminal_command_tilt_max_deg": (
        "terminal_command_tilt_max_deg", "command_tilt_max_deg"
    ),
    "terminal_command_tilt_slew_p100_degps": (
        "terminal_command_tilt_slew_p100_degps", "command_slew_max_degps"
    ),
    "combined_horizontal_acceleration_max_mps2": (
        "combined_horizontal_acceleration_max_mps2", "maximum_horizontal_acceleration_mps2"
    ),
    "touchdown_slip_m": (
        "touchdown_slip_m", "post_touchdown_slip_m", "post_touchdown_tangential_slip_m"
    ),
    "hold_tangential_velocity_p95_mps": ("hold_tangential_velocity_p95_mps",),
    "attitude_divergence_increment_deg": (
        "attitude_divergence_increment_deg", "attitude_divergence_deg",
        "attitude_divergence_delta_deg"
    ),
    "fallback_count": ("fallback_count", "mpc_fallback_count", "fallback_after_activation"),
    "terminal_stabilization_activation_count": (
        "terminal_stabilization_activation_count", "stabilization_activation_count"
    ),
    "solver_success_rate": ("solver_success_rate", "mpc_solver_success_rate"),
    "solver_failure_count": (
        "solver_failure_count", "mpc_non_solved_status_count"
    ),
    "solve_time_mean_ms": ("solve_time_mean_ms", "mpc_solve_time_mean_ms"),
    "solve_time_p95_ms": ("solve_time_p95_ms", "mpc_solve_time_p95_ms"),
    "constraint_violation_count": (
        "constraint_violation_count", "mpc_constraint_violation_count",
        "mpc_deadline_miss_count"
    ),
}

# tilted-deck evaluator 保留论文级嵌套结构；paper evaluation 聚合显式映射需要进入统一表格的标量，
# 避免递归搜索同名字段时误取 Ground Truth 或 observation-only 数据。
METRIC_PATHS: Mapping[str, tuple[tuple[str, ...], ...]] = {
    "horizontal_error_rmse_m": (
        ("tilted_deck_touchdown_metrics", "horizontal_error_m", "rmse"),
    ),
    "horizontal_error_max_m": (
        ("tilted_deck_touchdown_metrics", "horizontal_error_m", "max_abs"),
    ),
    "touchdown_vertical_speed_mps": (
        ("tilted_deck_touchdown_metrics", "touchdown_normal_relative_velocity_mps"),
    ),
    "candidate_to_confirm_delay_s": (
        ("tilted_deck_touchdown_metrics", "candidate_duration_s"),
    ),
    "hold_duration_s": (("tilted_deck_touchdown_metrics", "hold_duration_s"),),
    "recovery_count": (("tilted_deck_touchdown_metrics", "recovery_count"),),
    "marker_switch_count": (("tilted_deck_touchdown_metrics", "marker_switch_count"),),
    "detach_count": (("tilted_deck_touchdown_metrics", "detach_count"),),
    "secondary_contact_count": (
        ("tilted_deck_touchdown_metrics", "secondary_contact_count"),
    ),
    "candidate_repeat_count": (
        ("tilted_deck_touchdown_metrics", "candidate_repeat_count"),
    ),
    "normal_tracking_error_rmse_deg": (
        (
            "terminal_stabilization_metrics",
            "attitude_tracking_error_deg",
            "rmse",
        ),
        ("tilted_deck_touchdown_metrics", "normal_rmse_deg"),
    ),
    "normal_tracking_error_p95_deg": (
        (
            "terminal_stabilization_metrics",
            "attitude_tracking_error_deg",
            "p95",
        ),
        ("tilted_deck_touchdown_metrics", "normal_p95_deg"),
    ),
    "terminal_command_tilt_max_deg": (
        ("terminal_stabilization_metrics", "desired_tilt_deg", "max_abs"),
    ),
    "terminal_command_tilt_slew_p100_degps": (
        ("terminal_stabilization_metrics", "command_slew_degps", "max_abs"),
    ),
    "combined_horizontal_acceleration_max_mps2": (
        (
            "terminal_stabilization_metrics",
            "combined_acceleration_norm_mps2",
            "max_abs",
        ),
    ),
    "touchdown_slip_m": (
        ("tilted_deck_touchdown_metrics", "post_touchdown_tangential_slip_m"),
    ),
    "hold_tangential_velocity_p95_mps": (
        ("tilted_deck_touchdown_metrics", "hold_tangential_velocity_p95_mps"),
    ),
    "attitude_divergence_increment_deg": (
        ("tilted_deck_touchdown_metrics", "attitude_divergence_delta_deg"),
    ),
    "fallback_count": (
        ("terminal_stabilization_metrics", "fallback_count_after_activation"),
        ("terminal_stabilization_metrics", "fallback_count"),
    ),
    "terminal_stabilization_activation_count": (
        ("terminal_stabilization_metrics", "activation_sample_count"),
    ),
}


def _nested_value(evaluation: Mapping[str, Any], path: tuple[str, ...]) -> Any:
    value: Any = evaluation
    for key in path:
        if not isinstance(value, Mapping):
            return None
        value = value.get(key)
    return value


def _finite_metric(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None


def metric_value(evaluation: Mapping[str, Any], field: str) -> float | None:
    for candidate in METRIC_ALIASES.get(field, (field,)):
        numeric = _finite_metric(evaluation.get(candidate))
        if numeric is not None:
            return abs(numeric) if field == "touchdown_vertical_speed_mps" else numeric

    for path in METRIC_PATHS.get(field, ()):
        numeric = _finite_metric(_nested_value(evaluation, path))
        if numeric is not None:
            return abs(numeric) if field == "touchdown_vertical_speed_mps" else numeric
    return None
