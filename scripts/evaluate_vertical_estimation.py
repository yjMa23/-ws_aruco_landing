#!/usr/bin/env python3
"""离线评测 vertical estimation 低高度垂直状态估计误差。

Ground Truth 仅在本脚本中用于误差统计，不进入控制节点。
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence

from evaluate_horizontal_tracking import (
    GeodeticPosition,
    TimedVector,
    finite_values,
    interpolate,
    world_enu_to_local_ned,
)

STATE_TOPIC = "/landing/state"
MARKER_TOPIC = "/landing/marker_pose_ned"
ESTIMATED_TOPIC = "/landing/estimated_deck_odometry"
PREDICTED_TOPIC = "/landing/predicted_deck_pose"
VERTICAL_STATE_TOPIC = "/landing/vertical_state"
RAW_RELATIVE_HEIGHT_TOPIC = "/landing/raw_relative_height"
RELATIVE_VERTICAL_VELOCITY_TOPIC = "/landing/relative_vertical_velocity"
RELATIVE_HEIGHT_TOPIC = "/landing/relative_height"
REFERENCE_TOPIC = "/landing/relative_height_reference"
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"

DESCENT_ANALYSIS_STATES = {"DESCEND", "TEST_HEIGHT_HOLD", "RECOVER_CLIMB"}
SAFE_HEIGHT_ANALYSIS_STATES = {"WAIT_LANDING_WINDOW"}

OPTIONAL_TOPICS = {
    VERTICAL_STATE_TOPIC,
    RAW_RELATIVE_HEIGHT_TOPIC,
    RELATIVE_VERTICAL_VELOCITY_TOPIC,
}

REQUIRED_TOPICS = {
    STATE_TOPIC,
    MARKER_TOPIC,
    ESTIMATED_TOPIC,
    PREDICTED_TOPIC,
    RELATIVE_HEIGHT_TOPIC,
    REFERENCE_TOPIC,
    LOCAL_POSITION_TOPIC,
    GROUND_TRUTH_TOPIC,
}


@dataclass(frozen=True)
class ErrorStatistics:
    count: int
    bias: float
    rmse: float
    standard_deviation: float
    p95_absolute_error: float
    maximum_absolute_error: float
    correction_to_add: float


def resolve_bag_uri(path: Path) -> Path:
    if path.is_dir():
        return path
    if path.is_file() and path.suffix == ".db3":
        return path.parent
    raise FileNotFoundError(f"unsupported rosbag path: {path}")


def load_ros_modules() -> tuple[Any, Any, Any, Any, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from rosbag2_py import ConverterOptions, StorageOptions
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Source ROS Humble and px4_msgs first."
        ) from error
    return rosbag2_py, deserialize_message, get_message, StorageOptions, ConverterOptions


def percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        raise ValueError("cannot calculate a percentile from an empty sequence")
    if not 0.0 <= probability <= 1.0:
        raise ValueError("probability must be within [0, 1]")
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    alpha = position - lower
    return ordered[lower] + alpha * (ordered[upper] - ordered[lower])


def error_statistics(errors: Sequence[float]) -> ErrorStatistics:
    if not errors:
        raise ValueError("cannot calculate statistics from an empty sequence")
    bias = statistics.fmean(errors)
    absolute_errors = [abs(value) for value in errors]
    return ErrorStatistics(
        count=len(errors),
        bias=bias,
        rmse=math.sqrt(statistics.fmean(value * value for value in errors)),
        standard_deviation=statistics.pstdev(errors) if len(errors) > 1 else 0.0,
        p95_absolute_error=percentile(absolute_errors, 0.95),
        maximum_absolute_error=max(absolute_errors),
        correction_to_add=-bias,
    )


def statistics_to_dict(result: ErrorStatistics) -> dict[str, float | int]:
    return {
        "count": result.count,
        "bias_m": result.bias,
        "rmse_m": result.rmse,
        "standard_deviation_m": result.standard_deviation,
        "p95_absolute_error_m": result.p95_absolute_error,
        "maximum_absolute_error_m": result.maximum_absolute_error,
        "correction_to_add_m": result.correction_to_add,
    }


def active_intervals(
    states: Sequence[tuple[float, str]],
    final_time_s: float,
    active_states: set[str],
) -> list[tuple[float, float]]:
    intervals: list[tuple[float, float]] = []
    start: Optional[float] = None
    current_state: Optional[str] = None
    for time_s, state in states:
        if state == current_state:
            continue
        was_active = current_state in active_states
        is_active = state in active_states
        if is_active and not was_active:
            start = time_s
        elif was_active and not is_active and start is not None:
            intervals.append((start, time_s))
            start = None
        current_state = state
    if start is not None:
        intervals.append((start, final_time_s))
    return intervals


def time_in_intervals(time_s: float, intervals: Sequence[tuple[float, float]]) -> bool:
    return any(start <= time_s <= end for start, end in intervals)


def sample_at(
    samples: Sequence[TimedVector], query_time_s: float
) -> Optional[tuple[float, ...]]:
    return interpolate(samples, query_time_s)


def estimate_delay_s(
    signal: Sequence[TimedVector],
    reference: Sequence[TimedVector],
    start_s: float,
    end_s: float,
    sample_period_s: float,
    maximum_lag_s: float,
) -> Optional[float]:
    if end_s - start_s < max(4.0, 4.0 * maximum_lag_s):
        return None
    times: list[float] = []
    time_s = start_s
    while time_s <= end_s:
        if sample_at(signal, time_s) is not None and sample_at(reference, time_s) is not None:
            times.append(time_s)
        time_s += sample_period_s
    if len(times) < 40:
        return None

    reference_values = [sample_at(reference, time_s)[0] for time_s in times]  # type: ignore[index]
    if statistics.pstdev(reference_values) < 1.0e-3:
        return None

    maximum_lag_samples = int(round(maximum_lag_s / sample_period_s))
    best_lag_samples: Optional[int] = None
    best_correlation = -math.inf
    for lag_samples in range(-maximum_lag_samples, maximum_lag_samples + 1):
        paired_signal: list[float] = []
        paired_reference: list[float] = []
        lag_s = lag_samples * sample_period_s
        for time_s in times:
            signal_value = sample_at(signal, time_s + lag_s)
            reference_value = sample_at(reference, time_s)
            if signal_value is None or reference_value is None:
                continue
            paired_signal.append(signal_value[0])
            paired_reference.append(reference_value[0])
        if len(paired_signal) < 30:
            continue
        signal_mean = statistics.fmean(paired_signal)
        reference_mean = statistics.fmean(paired_reference)
        numerator = sum(
            (signal_value - signal_mean) * (reference_value - reference_mean)
            for signal_value, reference_value in zip(paired_signal, paired_reference)
        )
        signal_energy = sum((value - signal_mean) ** 2 for value in paired_signal)
        reference_energy = sum((value - reference_mean) ** 2 for value in paired_reference)
        denominator = math.sqrt(signal_energy * reference_energy)
        if denominator <= 1.0e-12:
            continue
        correlation = numerator / denominator
        if correlation > best_correlation:
            best_correlation = correlation
            best_lag_samples = lag_samples

    if best_lag_samples is None:
        return None
    # 正值表示待测信号需要向未来移动后才能与真值对齐，即待测信号相对真值滞后。
    return best_lag_samples * sample_period_s


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    (
        rosbag2_py,
        deserialize_message,
        get_message,
        storage_options_type,
        converter_options_type,
    ) = load_ros_modules()
    bag_uri = resolve_bag_uri(args.bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        storage_options_type(uri=str(bag_uri), storage_id="sqlite3"),
        converter_options_type(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing_topics = REQUIRED_TOPICS - topic_types.keys()
    if missing_topics:
        raise RuntimeError(
            "bag is missing required topics: " + ", ".join(sorted(missing_topics))
        )
    selected_topics = REQUIRED_TOPICS | (OPTIONAL_TOPICS & topic_types.keys())
    message_types = {topic: get_message(topic_types[topic]) for topic in selected_topics}

    states: list[tuple[float, str]] = []
    marker_z: list[TimedVector] = []
    estimated_z: list[TimedVector] = []
    estimated_vz: list[TimedVector] = []
    predicted_z: list[TimedVector] = []
    vertical_state_z: list[TimedVector] = []
    vertical_state_vz: list[TimedVector] = []
    raw_relative_height: list[TimedVector] = []
    published_relative_vertical_velocity: list[TimedVector] = []
    published_relative_height: list[TimedVector] = []
    reference_height: list[TimedVector] = []
    local_position_messages: list[tuple[float, Any]] = []
    ground_truth_world: list[TimedVector] = []
    final_time_s = 0.0

    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        time_s = timestamp_ns * 1.0e-9
        final_time_s = max(final_time_s, time_s)
        if topic not in message_types:
            continue
        message = deserialize_message(serialized_data, message_types[topic])
        if topic == STATE_TOPIC:
            states.append((time_s, str(message.data)))
        elif topic == MARKER_TOPIC:
            value = float(message.pose.position.z)
            if math.isfinite(value):
                marker_z.append(TimedVector(time_s, (value,)))
        elif topic == ESTIMATED_TOPIC:
            position = float(message.pose.pose.position.z)
            velocity = float(message.twist.twist.linear.z)
            if math.isfinite(position):
                estimated_z.append(TimedVector(time_s, (position,)))
            if math.isfinite(velocity):
                estimated_vz.append(TimedVector(time_s, (velocity,)))
        elif topic == PREDICTED_TOPIC:
            value = float(message.pose.position.z)
            if math.isfinite(value):
                predicted_z.append(TimedVector(time_s, (value,)))
        elif topic == VERTICAL_STATE_TOPIC:
            position = float(message.pose.pose.position.z)
            velocity = float(message.twist.twist.linear.z)
            if math.isfinite(position):
                vertical_state_z.append(TimedVector(time_s, (position,)))
            if math.isfinite(velocity):
                vertical_state_vz.append(TimedVector(time_s, (velocity,)))
        elif topic == RAW_RELATIVE_HEIGHT_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                raw_relative_height.append(TimedVector(time_s, (value,)))
        elif topic == RELATIVE_VERTICAL_VELOCITY_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                published_relative_vertical_velocity.append(
                    TimedVector(time_s, (value,))
                )
        elif topic == RELATIVE_HEIGHT_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                published_relative_height.append(TimedVector(time_s, (value,)))
        elif topic == REFERENCE_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                reference_height.append(TimedVector(time_s, (value,)))
        elif topic == LOCAL_POSITION_TOPIC:
            local_position_messages.append((time_s, message))
        elif topic == GROUND_TRUTH_TOPIC:
            position = message.pose.pose.position
            velocity = message.twist.twist.linear
            values = (
                float(position.x),
                float(position.y),
                float(position.z),
                float(velocity.x),
                float(velocity.y),
                float(velocity.z),
            )
            if finite_values(values):
                ground_truth_world.append(TimedVector(time_s, values))

    intervals = active_intervals(states, final_time_s, DESCENT_ANALYSIS_STATES)
    analysis_mode = "descent"
    if not intervals:
        intervals = active_intervals(states, final_time_s, SAFE_HEIGHT_ANALYSIS_STATES)
        analysis_mode = "safe_height_wait"
    if not intervals:
        raise RuntimeError(
            "bag never entered a relative descent/recovery state or WAIT_LANDING_WINDOW"
        )
    analysis_start_s = intervals[0][0] + args.discard_seconds
    analysis_end_s = intervals[-1][1]
    intervals[0] = (analysis_start_s, intervals[0][1])
    intervals = [(start, end) for start, end in intervals if end > start]
    if not intervals:
        raise RuntimeError("no analysis interval remains after transition discard")

    valid_reference_message = next(
        (
            message
            for time_s, message in local_position_messages
            if time_in_intervals(time_s, intervals)
            and bool(message.xy_global)
            and bool(message.z_global)
            and finite_values((message.ref_lat, message.ref_lon, message.ref_alt))
        ),
        None,
    )
    if valid_reference_message is None:
        raise RuntimeError("no valid PX4 local geodetic reference is available")

    local_origin = GeodeticPosition(
        float(valid_reference_message.ref_lat),
        float(valid_reference_message.ref_lon),
        float(valid_reference_message.ref_alt),
    )
    world_origin = GeodeticPosition(
        args.world_origin_latitude,
        args.world_origin_longitude,
        args.world_origin_altitude,
    )
    ground_truth_ned = [
        TimedVector(
            sample.time_s,
            world_enu_to_local_ned(sample.values[:3], world_origin, local_origin)
            + (sample.values[4], sample.values[3], -sample.values[5]),
        )
        for sample in ground_truth_world
    ]
    ground_truth_deck_z = [
        TimedVector(sample.time_s, (sample.values[2],)) for sample in ground_truth_ned
    ]
    ground_truth_deck_vz = [
        TimedVector(sample.time_s, (sample.values[5],)) for sample in ground_truth_ned
    ]

    uav_z: list[TimedVector] = []
    uav_vz: list[TimedVector] = []
    ground_truth_relative_height: list[TimedVector] = []
    ground_truth_relative_velocity: list[TimedVector] = []
    for time_s, message in local_position_messages:
        if not time_in_intervals(time_s, intervals):
            continue
        if bool(message.z_valid) and math.isfinite(message.z):
            uav_z.append(TimedVector(time_s, (float(message.z),)))
            deck_z_value = sample_at(ground_truth_deck_z, time_s)
            if deck_z_value is not None:
                ground_truth_relative_height.append(
                    TimedVector(time_s, (deck_z_value[0] - float(message.z),))
                )
        if math.isfinite(message.vz):
            uav_vz.append(TimedVector(time_s, (float(message.vz),)))
            deck_vz_value = sample_at(ground_truth_deck_vz, time_s)
            if deck_vz_value is not None:
                ground_truth_relative_velocity.append(
                    TimedVector(time_s, (deck_vz_value[0] - float(message.vz),))
                )

    def errors_against_ground_truth(samples: Sequence[TimedVector]) -> list[float]:
        errors: list[float] = []
        for sample in samples:
            if not time_in_intervals(sample.time_s, intervals):
                continue
            truth = sample_at(ground_truth_deck_z, sample.time_s)
            if truth is not None:
                errors.append(sample.values[0] - truth[0])
        return errors

    marker_errors = errors_against_ground_truth(marker_z)
    estimated_errors = errors_against_ground_truth(estimated_z)
    predicted_errors = errors_against_ground_truth(predicted_z)
    vertical_state_errors = errors_against_ground_truth(vertical_state_z)
    if not marker_errors or not estimated_errors or not predicted_errors:
        raise RuntimeError("insufficient aligned vertical deck-position samples")

    raw_relative_height_errors: list[float] = []
    for sample in raw_relative_height:
        if not time_in_intervals(sample.time_s, intervals):
            continue
        truth = sample_at(ground_truth_relative_height, sample.time_s)
        if truth is not None:
            raw_relative_height_errors.append(sample.values[0] - truth[0])

    relative_height_samples = list(published_relative_height)
    if not relative_height_samples:
        for sample in predicted_z:
            if not time_in_intervals(sample.time_s, intervals):
                continue
            vehicle_z = sample_at(uav_z, sample.time_s)
            if vehicle_z is not None:
                relative_height_samples.append(
                    TimedVector(sample.time_s, (sample.values[0] - vehicle_z[0],))
                )

    relative_height_errors: list[float] = []
    binned_errors: dict[str, list[float]] = {
        "above_2m": [],
        "1m_to_2m": [],
        "0p5m_to_1m": [],
        "below_0p5m": [],
    }
    for sample in relative_height_samples:
        if not time_in_intervals(sample.time_s, intervals):
            continue
        truth = sample_at(ground_truth_relative_height, sample.time_s)
        if truth is None:
            continue
        error = sample.values[0] - truth[0]
        relative_height_errors.append(error)
        actual_height = truth[0]
        if actual_height > 2.0:
            binned_errors["above_2m"].append(error)
        elif actual_height >= 1.0:
            binned_errors["1m_to_2m"].append(error)
        elif actual_height >= 0.5:
            binned_errors["0p5m_to_1m"].append(error)
        else:
            binned_errors["below_0p5m"].append(error)
    if not relative_height_errors:
        raise RuntimeError("insufficient aligned relative-height samples")

    relative_velocity_errors: list[float] = []
    for sample in estimated_vz:
        if not time_in_intervals(sample.time_s, intervals):
            continue
        uav_velocity = sample_at(uav_vz, sample.time_s)
        truth = sample_at(ground_truth_relative_velocity, sample.time_s)
        if uav_velocity is None or truth is None:
            continue
        estimated_relative_velocity = sample.values[0] - uav_velocity[0]
        relative_velocity_errors.append(estimated_relative_velocity - truth[0])

    vertical_relative_velocity_errors: list[float] = []
    if published_relative_vertical_velocity:
        for sample in published_relative_vertical_velocity:
            if not time_in_intervals(sample.time_s, intervals):
                continue
            truth = sample_at(ground_truth_relative_velocity, sample.time_s)
            if truth is not None:
                vertical_relative_velocity_errors.append(sample.values[0] - truth[0])
    else:
        for sample in vertical_state_vz:
            if not time_in_intervals(sample.time_s, intervals):
                continue
            uav_velocity = sample_at(uav_vz, sample.time_s)
            truth = sample_at(ground_truth_relative_velocity, sample.time_s)
            if uav_velocity is None or truth is None:
                continue
            vertical_relative_velocity_errors.append(
                sample.values[0] - uav_velocity[0] - truth[0]
            )

    contiguous_start_s, contiguous_end_s = max(
        intervals, key=lambda interval: interval[1] - interval[0]
    )
    delays = {
        "marker_z_delay_s": estimate_delay_s(
            marker_z,
            ground_truth_deck_z,
            contiguous_start_s,
            contiguous_end_s,
            args.delay_sample_period,
            args.maximum_delay,
        ),
        "estimated_z_delay_s": estimate_delay_s(
            estimated_z,
            ground_truth_deck_z,
            contiguous_start_s,
            contiguous_end_s,
            args.delay_sample_period,
            args.maximum_delay,
        ),
        "predicted_z_delay_s": estimate_delay_s(
            predicted_z,
            ground_truth_deck_z,
            contiguous_start_s,
            contiguous_end_s,
            args.delay_sample_period,
            args.maximum_delay,
        ),
        "vertical_state_z_delay_s": estimate_delay_s(
            vertical_state_z,
            ground_truth_deck_z,
            contiguous_start_s,
            contiguous_end_s,
            args.delay_sample_period,
            args.maximum_delay,
        ) if vertical_state_z else None,
    }

    return {
        "bag": str(bag_uri),
        "analysis_mode": analysis_mode,
        "analysis_duration_s": sum(end - start for start, end in intervals),
        "analysis_intervals": intervals,
        "marker_deck_z": statistics_to_dict(error_statistics(marker_errors)),
        "estimated_deck_z": statistics_to_dict(error_statistics(estimated_errors)),
        "predicted_deck_z": statistics_to_dict(error_statistics(predicted_errors)),
        "vertical_state_deck_z": (
            statistics_to_dict(error_statistics(vertical_state_errors))
            if vertical_state_errors
            else None
        ),
        "raw_relative_height": (
            statistics_to_dict(error_statistics(raw_relative_height_errors))
            if raw_relative_height_errors
            else None
        ),
        "relative_height": statistics_to_dict(error_statistics(relative_height_errors)),
        "relative_vertical_velocity": (
            statistics_to_dict(error_statistics(relative_velocity_errors))
            if relative_velocity_errors
            else None
        ),
        "vertical_state_relative_velocity": (
            statistics_to_dict(error_statistics(vertical_relative_velocity_errors))
            if vertical_relative_velocity_errors
            else None
        ),
        "relative_height_bins": {
            name: statistics_to_dict(error_statistics(errors)) if errors else None
            for name, errors in binned_errors.items()
        },
        "delay_estimates": delays,
        "reference_height_min_m": min(
            (
                sample.values[0]
                for sample in reference_height
                if time_in_intervals(sample.time_s, intervals)
            ),
            default=math.nan,
        ),
        "ground_truth_relative_height_min_m": min(
            (sample.values[0] for sample in ground_truth_relative_height),
            default=math.nan,
        ),
    }


def print_statistics(name: str, result: Optional[dict[str, Any]]) -> None:
    if result is None:
        print(f"{name}: unavailable")
        return
    print(
        f"{name}: bias {result['bias_m']:+.4f} m, "
        f"RMSE {result['rmse_m']:.4f} m, std {result['standard_deviation_m']:.4f} m, "
        f"P95 |error| {result['p95_absolute_error_m']:.4f} m, "
        f"max |error| {result['maximum_absolute_error_m']:.4f} m, "
        f"correction {result['correction_to_add_m']:+.4f} m, n={result['count']}"
    )


def print_human_readable(result: dict[str, Any]) -> None:
    print(f"Bag: {result['bag']}")
    print(
        f"Analysis mode / duration: {result['analysis_mode']} / "
        f"{result['analysis_duration_s']:.3f} s"
    )
    print_statistics("Marker deck z", result["marker_deck_z"])
    print_statistics("Estimated deck z", result["estimated_deck_z"])
    print_statistics("Predicted deck z", result["predicted_deck_z"])
    print_statistics("estimated deck z", result["vertical_state_deck_z"])
    print_statistics("Raw relative height", result["raw_relative_height"])
    print_statistics("Relative height", result["relative_height"])
    print_statistics("Legacy relative vertical velocity", result["relative_vertical_velocity"])
    print_statistics(
        "estimated relative velocity",
        result["vertical_state_relative_velocity"],
    )
    print("Relative-height bins:")
    for name, statistics_result in result["relative_height_bins"].items():
        print_statistics(f"  {name}", statistics_result)
    print("Delay estimates (positive means signal lags Ground Truth):")
    for name, value in result["delay_estimates"].items():
        print(f"  {name}: {'unavailable' if value is None else f'{value:+.3f} s'}")
    print(
        "Reference / Ground Truth minimum height: "
        f"{result['reference_height_min_m']:.4f} / "
        f"{result['ground_truth_relative_height_min_m']:.4f} m"
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate low-altitude vertical estimation errors from a relative descent ROS 2 bag."
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
    parser.add_argument(
        "--discard-seconds",
        type=float,
        default=1.0,
        help="discard time after first entering a descent state (default: 1.0)",
    )
    parser.add_argument(
        "--delay-sample-period",
        type=float,
        default=0.05,
        help="resampling period for delay estimation (default: 0.05 s)",
    )
    parser.add_argument(
        "--maximum-delay",
        type=float,
        default=1.0,
        help="maximum absolute delay searched by cross-correlation (default: 1.0 s)",
    )
    parser.add_argument(
        "--world-origin-latitude", type=float, default=47.397971057728974
    )
    parser.add_argument(
        "--world-origin-longitude", type=float, default=8.546163739800146
    )
    parser.add_argument("--world-origin-altitude", type=float, default=0.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    numeric_values = (
        args.discard_seconds,
        args.delay_sample_period,
        args.maximum_delay,
        args.world_origin_latitude,
        args.world_origin_longitude,
        args.world_origin_altitude,
    )
    if not finite_values(numeric_values):
        parser.error("all numeric arguments must be finite")
    if args.discard_seconds < 0.0:
        parser.error("--discard-seconds must be non-negative")
    if args.delay_sample_period <= 0.0:
        parser.error("--delay-sample-period must be positive")
    if args.maximum_delay <= 0.0:
        parser.error("--maximum-delay must be positive")
    return args


def main() -> int:
    args = parse_arguments()
    try:
        result = evaluate(args)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_human_readable(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
