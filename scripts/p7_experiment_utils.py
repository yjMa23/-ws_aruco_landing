#!/usr/bin/env python3
"""P7 批量实验配置、结果分类与统计公共工具。"""

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
    "heave_h1",
    "heave_h2",
    "heave_h3",
    "tilt_roll_pos_2deg",
    "tilt_pitch_pos_2deg",
)
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
    "PX4_ABORT",
    "EPISODE_TIMEOUT",
    "EVALUATION_ERROR",
    "CLEANUP_FAILURE",
    "UNKNOWN",
)
METRIC_FIELDS = (
    "landing_time_s",
    "horizontal_error_rmse_m",
    "horizontal_error_max_m",
    "touchdown_vertical_speed_mps",
    "candidate_to_confirm_delay_s",
    "recovery_count",
    "marker_switch_count",
    "hold_duration_s",
    "deck_vertical_span_final_m",
    "hold_relative_height_span_m",
    "hold_relative_vertical_velocity_p95_mps",
    "detach_count",
    "secondary_contact_count",
    "candidate_repeat_count",
)


@dataclass(frozen=True)
class ScenarioSpec:
    """单个场景的重复次数和展开后的随机种子。"""

    scenario: str
    repetitions: int
    seeds: tuple[int, ...]


@dataclass(frozen=True)
class BatchConfig:
    """P7 批量实验配置。"""

    name: str
    output_root: Path
    episode_timeout_s: float
    startup_timeout_s: float
    touchdown_hold_s: float
    camera_model: str
    record_camera_debug: bool
    scenarios: tuple[ScenarioSpec, ...]


def utc_now_iso() -> str:
    """返回带时区的 UTC ISO 时间。"""

    return datetime.now(timezone.utc).isoformat()


def atomic_write_json(path: Path, data: Mapping[str, Any]) -> None:
    """原子写入 JSON，避免中途中断留下半文件。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", encoding="utf-8", dir=path.parent, delete=False
    ) as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2, sort_keys=True)
        handle.write("\n")
        temporary_path = Path(handle.name)
    os.replace(temporary_path, path)


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Mapping[str, Any]]) -> None:
    """覆盖写入 CSV。"""

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def expand_seeds(repetitions: int, seeds: Sequence[int] | None) -> tuple[int, ...]:
    """按重复次数展开种子。

    未提供种子时使用 1..N；只提供一个种子时从该值连续递增；否则要求数量
    与 repetitions 完全一致。
    """

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
        raise ValueError("seeds must be unique within a scenario")
    return normalized


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    return value


def load_batch_config(path: Path, workspace_dir: Path | None = None) -> BatchConfig:
    """读取并校验 P7 YAML 配置。"""

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
    episode_timeout_s = float(root.get("episode_timeout_s", 600.0))
    startup_timeout_s = float(root.get("startup_timeout_s", 120.0))
    touchdown_hold_s = float(root.get("touchdown_hold_s", 10.0))
    if episode_timeout_s <= 0.0 or startup_timeout_s <= 0.0 or touchdown_hold_s < 10.0:
        raise ValueError(
            "episode/startup timeouts must be positive and touchdown_hold_s must be >= 10"
        )
    camera_model = str(root.get("camera_model", "close-range"))
    if camera_model not in ("close-range", "px4-default"):
        raise ValueError("camera_model must be close-range or px4-default")
    record_camera_debug = bool(root.get("record_camera_debug", False))
    scenarios_raw = root.get("scenarios")
    if not isinstance(scenarios_raw, list) or not scenarios_raw:
        raise ValueError("scenarios must be a non-empty list")
    scenarios: list[ScenarioSpec] = []
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
        scenarios.append(ScenarioSpec(scenario, repetitions, seeds))
    return BatchConfig(
        name=name,
        output_root=output_root_raw,
        episode_timeout_s=episode_timeout_s,
        startup_timeout_s=startup_timeout_s,
        touchdown_hold_s=touchdown_hold_s,
        camera_model=camera_model,
        record_camera_debug=record_camera_debug,
        scenarios=tuple(scenarios),
    )


def make_batch_id(name: str, now: datetime | None = None) -> str:
    """生成可排序且可读的批次 ID。"""

    current = now or datetime.now()
    safe_name = "".join(character if character.isalnum() or character in "-_" else "-" for character in name)
    safe_name = safe_name.strip("-") or "p7"
    return f"{safe_name}_{current.strftime('%Y%m%d_%H%M%S')}"


def make_episode_id(batch_id: str, scenario: str, repetition: int, seed: int) -> str:
    """生成批次内确定且跨批次唯一的 episode ID。"""

    if scenario not in SUPPORTED_SCENARIOS:
        raise ValueError(f"unsupported scenario: {scenario}")
    if repetition <= 0:
        raise ValueError("repetition must be positive")
    return f"{batch_id}_{scenario}_r{repetition:03d}_s{seed:010d}"


def read_json(path: Path) -> dict[str, Any]:
    """读取 JSON 对象，损坏或非对象时抛出 ValueError。"""

    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"failed to read JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def read_evaluation_json(path: Path) -> dict[str, Any]:
    """读取评测结果并验证关键字段。"""

    result = read_json(path)
    if "positive_touchdown_passed" not in result:
        raise ValueError("evaluation JSON is missing positive_touchdown_passed")
    if not isinstance(result["positive_touchdown_passed"], bool):
        raise ValueError("positive_touchdown_passed must be boolean")
    return result


def episode_result_complete(episode_dir: Path) -> bool:
    """判断 episode 是否已有可用于 resume 的完整结构化结果。"""

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
    failure_reason = manifest.get("failure_reason")
    if failure_reason not in FAILURE_TYPES:
        return False
    evaluation_path = manifest.get("evaluation_path")
    if evaluation_path:
        return Path(str(evaluation_path)).is_file()
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
    """根据运行事件、状态序列、日志与评测结果统一分类失败。"""

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
        if evaluation.get("confirmed_start_s") is None:
            return "TOUCHDOWN_NOT_CONFIRMED"
        return "EVALUATION_ERROR"
    return "UNKNOWN"


def percentile(values: Sequence[float], probability: float) -> float | None:
    """使用线性插值计算百分位。"""

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
    """输出 mean/std/median/P90/P95；空输入返回 null 指标。"""

    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0,
            "mean": None,
            "stddev": None,
            "median": None,
            "p90": None,
            "p95": None,
        }
    return {
        "count": len(finite),
        "mean": statistics.fmean(finite),
        "stddev": statistics.pstdev(finite),
        "median": statistics.median(finite),
        "p90": percentile(finite, 0.90),
        "p95": percentile(finite, 0.95),
    }


def metric_value(evaluation: Mapping[str, Any], field: str) -> float | None:
    """从 evaluator JSON 读取聚合指标；非法值返回 None。"""

    value = evaluation.get(field)
    if value is None:
        return None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None
