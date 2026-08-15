#!/usr/bin/env python3
"""基于固定 Bag 分解 0.5 s Future Twist 因果误差；Ground Truth 只用于离线评分。"""

from __future__ import annotations

import argparse
import bisect
from collections import defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Optional, Sequence

from evaluate_deck_motion_shadow import (
    GROUND_TRUTH_TOPIC,
    Q_NED_ENU,
    STATE_TOPIC,
    TRAJECTORY_TOPIC,
    derivative_truth,
    duration_seconds,
    interpolate_truth,
    load_ros_modules,
    normalize_quaternion_or_none,
    quaternion_multiply,
    quaternion_rotate,
    stamp_seconds,
)


PREDICTION_HORIZON_S = 0.50
POINT_TIME_TOLERANCE_S = 1.0e-9
STATE_TRAJECTORY_PAIR_TOLERANCE_S = 0.01

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class TwistTruth:
    time_s: float
    linear_velocity: Vector3
    angular_velocity: Vector3


@dataclass(frozen=True)
class TrajectoryRecord:
    bag_time_s: float
    publish_time_s: float
    state_sample_time_s: float
    target_time_s: float
    origin_linear_velocity: Vector3
    origin_angular_velocity: Vector3
    linear_acceleration: Vector3
    angular_acceleration: Vector3
    published_future_linear_velocity: Vector3
    published_future_angular_velocity: Vector3


@dataclass(frozen=True)
class ErrorDecomposition:
    current: Vector3
    acceleration: Vector3
    model_residual: Vector3
    final: Vector3
    closure: Vector3


@dataclass(frozen=True)
class FrameDiagnostic:
    episode: str
    scenario: str
    observation_age_s: float
    target_time_s: float
    actual_horizon_s: float
    state_sample_to_target_horizon_s: float
    linear: ErrorDecomposition
    angular: ErrorDecomposition
    ca_linear_error: Vector3
    cv_linear_error: Vector3
    ca_angular_error: Vector3
    cv_angular_error: Vector3


def finite_vector(values: Sequence[float]) -> bool:
    return len(values) == 3 and all(math.isfinite(value) for value in values)


def add(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return tuple(x + y for x, y in zip(a, b))  # type: ignore[return-value]


def subtract(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return tuple(x - y for x, y in zip(a, b))  # type: ignore[return-value]


def scale(values: Sequence[float], factor: float) -> Vector3:
    return tuple(factor * value for value in values)  # type: ignore[return-value]


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def percentile(values: Sequence[float], probability: float) -> Optional[float]:
    clean = sorted(value for value in values if math.isfinite(value))
    if not clean:
        return None
    position = probability * (len(clean) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return clean[lower]
    return clean[lower] + (position - lower) * (clean[upper] - clean[lower])


def metric_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return {"count": 0, "rmse": None, "p95": None, "max": None}
    return {
        "count": len(clean),
        "rmse": math.sqrt(sum(value * value for value in clean) / len(clean)),
        "p95": percentile(clean, 0.95),
        "max": max(clean),
    }


def distribution_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    clean = [value for value in values if math.isfinite(value)]
    if not clean:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    return {
        "count": len(clean),
        "min": min(clean),
        "median": percentile(clean, 0.50),
        "p95": percentile(clean, 0.95),
        "max": max(clean),
    }


def pearson_correlation(a: Sequence[float], b: Sequence[float]) -> Optional[float]:
    pairs = [(x, y) for x, y in zip(a, b) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 2:
        return None
    xs, ys = zip(*pairs)
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    centered_x = [value - mean_x for value in xs]
    centered_y = [value - mean_y for value in ys]
    denominator = math.sqrt(
        sum(value * value for value in centered_x)
        * sum(value * value for value in centered_y)
    )
    if denominator <= 1.0e-15:
        return None
    return sum(x * y for x, y in zip(centered_x, centered_y)) / denominator


def time_semantics(
    state_sample_time_s: float,
    publish_time_s: float,
    relative_horizon_s: float,
) -> dict[str, float]:
    if not all(
        math.isfinite(value)
        for value in (state_sample_time_s, publish_time_s, relative_horizon_s)
    ):
        raise ValueError("time semantics require finite timestamps")
    target_time_s = publish_time_s + relative_horizon_s
    return {
        "observation_age_s": publish_time_s - state_sample_time_s,
        "requested_horizon_s": relative_horizon_s,
        "actual_target_time_s": target_time_s,
        "state_sample_to_target_horizon_s": target_time_s - state_sample_time_s,
    }


def decompose_error(
    estimated_current: Vector3,
    estimated_acceleration: Vector3,
    truth_current: Vector3,
    truth_acceleration: Vector3,
    truth_future: Vector3,
    published_future: Vector3,
    horizon_s: float = PREDICTION_HORIZON_S,
) -> ErrorDecomposition:
    """分解 published future error；closure 同时验证生产 CA 外推与分解公式。"""

    vectors = (
        estimated_current,
        estimated_acceleration,
        truth_current,
        truth_acceleration,
        truth_future,
        published_future,
    )
    if not math.isfinite(horizon_s) or horizon_s < 0.0 or not all(
        finite_vector(vector) for vector in vectors
    ):
        raise ValueError("error decomposition requires finite vectors and horizon")
    current = subtract(estimated_current, truth_current)
    acceleration = scale(subtract(estimated_acceleration, truth_acceleration), horizon_s)
    model_residual = subtract(
        add(truth_current, scale(truth_acceleration, horizon_s)), truth_future
    )
    final = subtract(published_future, truth_future)
    closure = subtract(final, add(add(current, acceleration), model_residual))
    return ErrorDecomposition(current, acceleration, model_residual, final, closure)


def counterfactual_predictions(
    estimated_current: Vector3,
    estimated_acceleration: Vector3,
    horizon_s: float = PREDICTION_HORIZON_S,
) -> tuple[Vector3, Vector3]:
    """只用 causal origin estimate 构造 CA 与 CV，不读取 Ground Truth。"""

    if not math.isfinite(horizon_s) or horizon_s < 0.0:
        raise ValueError("counterfactual horizon must be finite and nonnegative")
    if not finite_vector(estimated_current) or not finite_vector(estimated_acceleration):
        raise ValueError("counterfactual inputs must be finite")
    ca = add(estimated_current, scale(estimated_acceleration, horizon_s))
    cv = estimated_current
    return ca, cv


def nearest_state_sample_time(
    state_stamps: Sequence[tuple[float, float]],
    trajectory_bag_time_s: float,
    tolerance_s: float = STATE_TRAJECTORY_PAIR_TOLERANCE_S,
) -> Optional[float]:
    """按 rosbag receipt time 最近邻配对同一 publish tick，允许跨 topic 到达顺序反转。"""

    if not state_stamps or not math.isfinite(trajectory_bag_time_s):
        return None
    bag_times = [sample[0] for sample in state_stamps]
    index = bisect.bisect_left(bag_times, trajectory_bag_time_s)
    candidates = []
    if index < len(state_stamps):
        candidates.append(state_stamps[index])
    if index > 0:
        candidates.append(state_stamps[index - 1])
    if not candidates:
        return None
    nearest = min(candidates, key=lambda sample: abs(sample[0] - trajectory_bag_time_s))
    if abs(nearest[0] - trajectory_bag_time_s) > tolerance_s:
        return None
    return nearest[1]


def scenario_from_episode(episode: str) -> str:
    if "_s" not in episode:
        raise ValueError(f"episode name lacks seed suffix: {episode}")
    return episode.rsplit("_s", 1)[0]


def _truth_as_rigid(samples: Sequence[TwistTruth]) -> list[Any]:
    """复用 evaluator 的插值/差分函数，仅填充它需要的 twist 字段。"""

    from evaluate_deck_motion_shadow import RigidState

    identity = (1.0, 0.0, 0.0, 0.0)
    zero = (0.0, 0.0, 0.0)
    return [
        RigidState(sample.time_s, zero, identity, sample.linear_velocity, sample.angular_velocity)
        for sample in samples
    ]


def load_episode(bag_path: Path) -> tuple[list[TwistTruth], list[TrajectoryRecord], dict[str, int]]:
    rosbag2_py, deserialize_message, get_message, StorageOptions, ConverterOptions = load_ros_modules()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = {GROUND_TRUTH_TOPIC, STATE_TOPIC, TRAJECTORY_TOPIC}
    missing = required - topic_types.keys()
    if missing:
        raise RuntimeError("bag is missing required topics: " + ", ".join(sorted(missing)))
    message_types = {topic: get_message(topic_types[topic]) for topic in required}

    truth: list[TwistTruth] = []
    state_stamps: list[tuple[float, float]] = []
    raw_trajectories: list[tuple[float, float, Any, Any]] = []
    counts = {"invalid_ground_truth": 0, "invalid_trajectory": 0, "unpaired_trajectory": 0}

    while reader.has_next():
        topic, serialized, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        bag_time_s = bag_timestamp_ns * 1.0e-9
        if topic == GROUND_TRUTH_TOPIC:
            pose = message.pose.pose
            twist = message.twist.twist
            q_enu_body = normalize_quaternion_or_none(
                (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            )
            raw_values = (
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            )
            if q_enu_body is None or not all(math.isfinite(value) for value in raw_values):
                counts["invalid_ground_truth"] += 1
                continue
            q_ned_body = quaternion_multiply(Q_NED_ENU, q_enu_body)
            linear: Vector3 = (twist.linear.y, twist.linear.x, -twist.linear.z)
            angular = quaternion_rotate(
                q_ned_body, (twist.angular.x, twist.angular.y, twist.angular.z)
            )
            truth.append(
                TwistTruth(
                    stamp_seconds(message.header.stamp),
                    linear,
                    tuple(angular),  # type: ignore[arg-type]
                )
            )
        elif topic == STATE_TOPIC:
            state_stamps.append((bag_time_s, stamp_seconds(message.header.stamp)))
        elif topic == TRAJECTORY_TOPIC:
            if message.header.frame_id != "uav_origin_ned":
                counts["invalid_trajectory"] += 1
                continue
            origin = None
            future = None
            for point in message.points:
                relative_time_s = duration_seconds(point.time_from_start)
                if abs(relative_time_s) <= POINT_TIME_TOLERANCE_S:
                    origin = point
                elif abs(relative_time_s - PREDICTION_HORIZON_S) <= POINT_TIME_TOLERANCE_S:
                    future = point
            if origin is None or future is None:
                counts["invalid_trajectory"] += 1
                continue
            raw_trajectories.append(
                (bag_time_s, stamp_seconds(message.header.stamp), origin, future)
            )

    truth.sort(key=lambda sample: sample.time_s)
    state_stamps.sort()
    records: list[TrajectoryRecord] = []
    for bag_time_s, publish_time_s, origin, future in raw_trajectories:
        state_sample_time_s = nearest_state_sample_time(state_stamps, bag_time_s)
        if state_sample_time_s is None:
            counts["unpaired_trajectory"] += 1
            continue
        if (
            len(origin.velocities) != 1
            or len(origin.accelerations) != 1
            or len(future.velocities) != 1
        ):
            counts["invalid_trajectory"] += 1
            continue
        origin_velocity = origin.velocities[0]
        origin_acceleration = origin.accelerations[0]
        future_velocity = future.velocities[0]
        values = (
            origin_velocity.linear.x, origin_velocity.linear.y, origin_velocity.linear.z,
            origin_velocity.angular.x, origin_velocity.angular.y, origin_velocity.angular.z,
            origin_acceleration.linear.x, origin_acceleration.linear.y, origin_acceleration.linear.z,
            origin_acceleration.angular.x, origin_acceleration.angular.y, origin_acceleration.angular.z,
            future_velocity.linear.x, future_velocity.linear.y, future_velocity.linear.z,
            future_velocity.angular.x, future_velocity.angular.y, future_velocity.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            counts["invalid_trajectory"] += 1
            continue
        records.append(
            TrajectoryRecord(
                bag_time_s=bag_time_s,
                publish_time_s=publish_time_s,
                state_sample_time_s=state_sample_time_s,
                target_time_s=publish_time_s + PREDICTION_HORIZON_S,
                origin_linear_velocity=tuple(values[0:3]),  # type: ignore[arg-type]
                origin_angular_velocity=tuple(values[3:6]),  # type: ignore[arg-type]
                linear_acceleration=tuple(values[6:9]),  # type: ignore[arg-type]
                angular_acceleration=tuple(values[9:12]),  # type: ignore[arg-type]
                published_future_linear_velocity=tuple(values[12:15]),  # type: ignore[arg-type]
                published_future_angular_velocity=tuple(values[15:18]),  # type: ignore[arg-type]
            )
        )
    return truth, records, counts


def build_frame_diagnostics(
    episode: str,
    truth: Sequence[TwistTruth],
    records: Sequence[TrajectoryRecord],
) -> tuple[list[FrameDiagnostic], int]:
    scenario = scenario_from_episode(episode)
    rigid_truth = _truth_as_rigid(truth)
    frames: list[FrameDiagnostic] = []
    excluded_without_truth = 0
    for record in records:
        truth_current_state = interpolate_truth(rigid_truth, record.publish_time_s)
        truth_future_state = interpolate_truth(rigid_truth, record.target_time_s)
        derivatives = derivative_truth(rigid_truth, record.publish_time_s)
        if truth_current_state is None or truth_future_state is None or derivatives is None:
            excluded_without_truth += 1
            continue
        truth_current_linear = tuple(truth_current_state.linear_velocity)
        truth_current_angular = tuple(truth_current_state.angular_velocity)
        truth_future_linear = tuple(truth_future_state.linear_velocity)
        truth_future_angular = tuple(truth_future_state.angular_velocity)
        truth_linear_acceleration = tuple(derivatives[0])
        truth_angular_acceleration = tuple(derivatives[1])

        linear = decompose_error(
            record.origin_linear_velocity,
            record.linear_acceleration,
            truth_current_linear,
            truth_linear_acceleration,
            truth_future_linear,
            record.published_future_linear_velocity,
        )
        angular = decompose_error(
            record.origin_angular_velocity,
            record.angular_acceleration,
            truth_current_angular,
            truth_angular_acceleration,
            truth_future_angular,
            record.published_future_angular_velocity,
        )
        ca_linear, cv_linear = counterfactual_predictions(
            record.origin_linear_velocity, record.linear_acceleration
        )
        ca_angular, cv_angular = counterfactual_predictions(
            record.origin_angular_velocity, record.angular_acceleration
        )
        semantics = time_semantics(
            record.state_sample_time_s, record.publish_time_s, PREDICTION_HORIZON_S
        )
        frames.append(
            FrameDiagnostic(
                episode=episode,
                scenario=scenario,
                observation_age_s=semantics["observation_age_s"],
                target_time_s=semantics["actual_target_time_s"],
                actual_horizon_s=record.target_time_s - record.publish_time_s,
                state_sample_to_target_horizon_s=semantics[
                    "state_sample_to_target_horizon_s"
                ],
                linear=linear,
                angular=angular,
                ca_linear_error=subtract(ca_linear, truth_future_linear),
                cv_linear_error=subtract(cv_linear, truth_future_linear),
                ca_angular_error=subtract(ca_angular, truth_future_angular),
                cv_angular_error=subtract(cv_angular, truth_future_angular),
            )
        )
    return frames, excluded_without_truth


def _linear_scalar(vector: Vector3, channel: str) -> float:
    if channel == "horizontal":
        return math.hypot(vector[0], vector[1])
    if channel == "vertical":
        return abs(vector[2])
    raise KeyError(channel)


def _angular_scalar(vector: Vector3, channel: str) -> float:
    factor = 180.0 / math.pi
    if channel == "roll":
        return abs(vector[0]) * factor
    if channel == "pitch":
        return abs(vector[1]) * factor
    if channel == "yaw":
        return abs(vector[2]) * factor
    if channel == "roll_pitch":
        return math.hypot(vector[0], vector[1]) * factor
    if channel == "vector":
        return norm(vector) * factor
    raise KeyError(channel)


def _component_metrics(
    frames: Sequence[FrameDiagnostic],
    attribute: str,
    channel: str,
    angular: bool,
) -> dict[str, Any]:
    decompositions = [getattr(frame, attribute) for frame in frames]
    scalar = _angular_scalar if angular else _linear_scalar
    values = {
        "current": [scalar(item.current, channel) for item in decompositions],
        "acceleration_estimation_contribution": [
            scalar(item.acceleration, channel) for item in decompositions
        ],
        "constant_acceleration_model_residual": [
            scalar(item.model_residual, channel) for item in decompositions
        ],
        "final_future_error": [scalar(item.final, channel) for item in decompositions],
        "closure_residual": [scalar(item.closure, channel) for item in decompositions],
    }
    final = values["final_future_error"]
    return {
        "components": {name: metric_summary(samples) for name, samples in values.items()},
        "correlation_with_final_error": {
            "current": pearson_correlation(values["current"], final),
            "acceleration_estimation_contribution": pearson_correlation(
                values["acceleration_estimation_contribution"], final
            ),
            "constant_acceleration_model_residual": pearson_correlation(
                values["constant_acceleration_model_residual"], final
            ),
        },
    }


def _counterfactual_metrics(
    frames: Sequence[FrameDiagnostic], channel: str, angular: bool
) -> dict[str, Any]:
    scalar = _angular_scalar if angular else _linear_scalar
    ca_attr = "ca_angular_error" if angular else "ca_linear_error"
    cv_attr = "cv_angular_error" if angular else "cv_linear_error"
    ca = [scalar(getattr(frame, ca_attr), channel) for frame in frames]
    cv = [scalar(getattr(frame, cv_attr), channel) for frame in frames]
    ca_summary = metric_summary(ca)
    cv_summary = metric_summary(cv)
    count = len(ca)
    ca_better = sum(a < b - 1.0e-12 for a, b in zip(ca, cv))
    cv_better = sum(b < a - 1.0e-12 for a, b in zip(ca, cv))
    ties = count - ca_better - cv_better

    def delta(key: str) -> Optional[float]:
        a = ca_summary[key]
        b = cv_summary[key]
        return None if a is None or b is None else float(a) - float(b)

    return {
        "ca": ca_summary,
        "cv": cv_summary,
        "ca_minus_cv": {"rmse": delta("rmse"), "p95": delta("p95")},
        "ca_better_frame_ratio": ca_better / count if count else None,
        "cv_better_frame_ratio": cv_better / count if count else None,
        "tie_frame_ratio": ties / count if count else None,
    }


def aggregate_frames(frames: Sequence[FrameDiagnostic]) -> dict[str, Any]:
    ages = [frame.observation_age_s for frame in frames]
    target_offsets = [frame.actual_horizon_s for frame in frames]
    sample_to_target = [frame.state_sample_to_target_horizon_s for frame in frames]
    targets = [frame.target_time_s for frame in frames]
    result: dict[str, Any] = {
        "frame_count": len(frames),
        "time_semantics": {
            "requested_horizon_s": PREDICTION_HORIZON_S,
            "observation_age_s": distribution_summary(ages),
            "actual_target_offset_s": distribution_summary(target_offsets),
            "actual_target_time_s": distribution_summary(targets),
            "state_sample_to_target_horizon_s": distribution_summary(sample_to_target),
        },
        "linear": {
            channel: _component_metrics(frames, "linear", channel, False)
            for channel in ("horizontal", "vertical")
        },
        "angular": {
            channel: _component_metrics(frames, "angular", channel, True)
            for channel in ("roll", "pitch", "yaw", "roll_pitch", "vector")
        },
        "counterfactual": {
            "horizontal_velocity": _counterfactual_metrics(frames, "horizontal", False),
            "vertical_velocity": _counterfactual_metrics(frames, "vertical", False),
            "angular_velocity": _counterfactual_metrics(frames, "vector", True),
        },
    }
    result["time_semantics"]["observation_age_correlation_with_final_error"] = {
        "horizontal_velocity": pearson_correlation(
            ages, [_linear_scalar(frame.linear.final, "horizontal") for frame in frames]
        ),
        "vertical_velocity": pearson_correlation(
            ages, [_linear_scalar(frame.linear.final, "vertical") for frame in frames]
        ),
        "angular_velocity": pearson_correlation(
            ages, [_angular_scalar(frame.angular.final, "vector") for frame in frames]
        ),
    }
    return result


def aggregate_by_scenario(frames: Sequence[FrameDiagnostic]) -> dict[str, Any]:
    grouped: dict[str, list[FrameDiagnostic]] = defaultdict(list)
    for frame in frames:
        grouped[frame.scenario].append(frame)
    return {
        scenario: aggregate_frames(group)
        for scenario, group in sorted(grouped.items())
    }


def analyze_matrix(matrix_dir: Path) -> dict[str, Any]:
    expected = [
        f"{scenario}_s{seed}"
        for scenario in ("static", "rollpitch", "combined", "rigid_body_motion")
        for seed in (1, 2, 3)
    ]
    missing = [episode for episode in expected if not (matrix_dir / episode / "bag").is_dir()]
    if missing:
        raise RuntimeError("fixed matrix is incomplete: " + ", ".join(missing))

    episode_frames: dict[str, list[FrameDiagnostic]] = {}
    episode_counts: dict[str, dict[str, int]] = {}
    all_frames: list[FrameDiagnostic] = []
    for episode in expected:
        truth, records, counts = load_episode(matrix_dir / episode / "bag")
        frames, excluded_without_truth = build_frame_diagnostics(episode, truth, records)
        counts = dict(counts)
        counts["trajectory_records"] = len(records)
        counts["diagnostic_frames"] = len(frames)
        counts["excluded_without_truth"] = excluded_without_truth
        episode_counts[episode] = counts
        episode_frames[episode] = frames
        all_frames.extend(frames)

    result = {
        "matrix_dir": str(matrix_dir),
        "causal_boundary": (
            "CA/CV predictions use only trajectory origin twist and acceleration already published "
            "at that origin; Ground Truth is read only after candidate generation for offline scoring"
        ),
        "episodes": {
            episode: {
                "counts": episode_counts[episode],
                "aggregate": aggregate_frames(frames),
            }
            for episode, frames in episode_frames.items()
        },
        "scenarios": aggregate_by_scenario(all_frames),
        "global": aggregate_frames(all_frames),
    }
    # 最终编码使用 allow_nan=False；这里先显式验证，避免任何 NaN/Inf 悄悄进入结果。
    json.dumps(result, allow_nan=False)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze_matrix(args.matrix_dir)
    encoded = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
