#!/usr/bin/env python3
"""离线评测 P5B 相对甲板高度分阶段下降 rosbag。"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any, Optional, Sequence

from evaluate_p4_bag import (
    GeodeticPosition,
    TimedVector,
    finite_values,
    interpolate,
    world_enu_to_local_ned,
)


STATE_TOPIC = "/landing/state"
WINDOW_OPEN_TOPIC = "/landing/window_open"
RELATIVE_HEIGHT_TOPIC = "/landing/relative_height"
REFERENCE_TOPIC = "/landing/relative_height_reference"
PHASE_TOPIC = "/landing/descent_phase"
TARGET_TOPIC = "/landing/target_pose"
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"

DESCENT_STATES = {"DESCEND", "TEST_HEIGHT_HOLD", "RECOVER_CLIMB"}
FORBIDDEN_STATES = {
    "CENTER_ABOVE_MARKER",
    "DESCEND_WITH_TRACKING",
    "FINAL_LAND",
    "DONE",
    "ABORT",
}
NAV_LAND_COMMAND = 21
ARM_DISARM_COMMAND = 400

REQUIRED_TOPICS = {
    STATE_TOPIC,
    WINDOW_OPEN_TOPIC,
    RELATIVE_HEIGHT_TOPIC,
    REFERENCE_TOPIC,
    PHASE_TOPIC,
    TARGET_TOPIC,
    LOCAL_POSITION_TOPIC,
    GROUND_TRUTH_TOPIC,
    VEHICLE_COMMAND_TOPIC,
}


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


def nearest_boolean(
    samples: Sequence[tuple[float, bool]], query_time_s: float, tolerance_s: float = 0.03
) -> Optional[bool]:
    if not samples:
        return None
    times = [sample[0] for sample in samples]
    upper_index = bisect.bisect_left(times, query_time_s)
    candidate_indices = [
        index
        for index in (upper_index - 1, upper_index)
        if 0 <= index < len(samples)
    ]
    if not candidate_indices:
        return None
    best_index = min(
        candidate_indices, key=lambda index: abs(samples[index][0] - query_time_s)
    )
    if abs(samples[best_index][0] - query_time_s) > tolerance_s:
        return None
    return samples[best_index][1]


def state_sequence(samples: Sequence[tuple[float, str]]) -> list[str]:
    sequence: list[str] = []
    for _, state in samples:
        if not sequence or sequence[-1] != state:
            sequence.append(state)
    return sequence


def rmse(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("cannot compute RMSE from an empty sequence")
    return math.sqrt(sum(value * value for value in values) / len(values))


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
    missing = REQUIRED_TOPICS - topic_types.keys()
    if missing:
        raise RuntimeError("bag is missing required topics: " + ", ".join(sorted(missing)))
    message_types = {topic: get_message(topic_types[topic]) for topic in REQUIRED_TOPICS}

    state_samples: list[tuple[float, str]] = []
    window_samples: list[tuple[float, bool]] = []
    estimated_height_samples: list[TimedVector] = []
    reference_samples: list[TimedVector] = []
    phase_samples: list[tuple[float, str]] = []
    target_samples: list[TimedVector] = []
    local_position_samples: list[tuple[float, Any]] = []
    ground_truth_world: list[TimedVector] = []
    command_samples: list[tuple[float, int, float]] = []

    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        time_s = timestamp_ns * 1.0e-9
        message = deserialize_message(serialized_data, message_types[topic])
        if topic == STATE_TOPIC:
            state_samples.append((time_s, str(message.data)))
        elif topic == WINDOW_OPEN_TOPIC:
            window_samples.append((time_s, bool(message.data)))
        elif topic == RELATIVE_HEIGHT_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                estimated_height_samples.append(TimedVector(time_s, (value,)))
        elif topic == REFERENCE_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                reference_samples.append(TimedVector(time_s, (value,)))
        elif topic == PHASE_TOPIC:
            phase_samples.append((time_s, str(message.data)))
        elif topic == TARGET_TOPIC:
            z = float(message.pose.position.z)
            if math.isfinite(z):
                target_samples.append(TimedVector(time_s, (z,)))
        elif topic == LOCAL_POSITION_TOPIC:
            local_position_samples.append((time_s, message))
        elif topic == GROUND_TRUTH_TOPIC:
            position = message.pose.pose.position
            values = (float(position.x), float(position.y), float(position.z))
            if finite_values(values):
                ground_truth_world.append(TimedVector(time_s, values))
        elif topic == VEHICLE_COMMAND_TOPIC:
            command_samples.append((time_s, int(message.command), float(message.param1)))

    descent_times = [time_s for time_s, state in state_samples if state in DESCENT_STATES]
    if not descent_times:
        raise RuntimeError("bag never entered a P5B descent state")
    descent_start_s = descent_times[0]
    final_time_s = max(
        state_samples[-1][0],
        reference_samples[-1].time_s if reference_samples else descent_start_s,
    )

    if not reference_samples or not estimated_height_samples:
        raise RuntimeError("relative-height debug topics contain no valid samples")

    reference_after_start = [sample for sample in reference_samples if sample.time_s >= descent_start_s]
    estimated_after_start = [
        sample for sample in estimated_height_samples if sample.time_s >= descent_start_s
    ]
    target_after_start = [sample for sample in target_samples if sample.time_s >= descent_start_s]
    if not reference_after_start or not estimated_after_start or not target_after_start:
        raise RuntimeError("descent interval contains incomplete debug data")

    valid_reference_message = next(
        (
            message
            for time_s, message in local_position_samples
            if time_s >= descent_start_s
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
            world_enu_to_local_ned(sample.values, world_origin, local_origin),
        )
        for sample in ground_truth_world
    ]

    actual_height_samples: list[TimedVector] = []
    actual_reference_errors: list[float] = []
    for time_s, message in local_position_samples:
        if time_s < descent_start_s or not bool(message.z_valid) or not math.isfinite(message.z):
            continue
        deck_position = interpolate(ground_truth_ned, time_s)
        reference = interpolate(reference_after_start, time_s)
        if deck_position is None or reference is None:
            continue
        actual_height = deck_position[2] - float(message.z)
        if not math.isfinite(actual_height):
            continue
        actual_height_samples.append(TimedVector(time_s, (actual_height,)))
        actual_reference_errors.append(actual_height - reference[0])

    estimated_reference_errors: list[float] = []
    for sample in estimated_after_start:
        reference = interpolate(reference_after_start, sample.time_s)
        if reference is not None:
            estimated_reference_errors.append(sample.values[0] - reference[0])

    reference_decreases_with_closed_window = 0
    reference_rates: dict[str, list[float]] = {"fast": [], "medium": [], "slow": []}
    for previous, current in zip(reference_after_start, reference_after_start[1:]):
        dt_s = current.time_s - previous.time_s
        delta_m = current.values[0] - previous.values[0]
        if dt_s <= 0.0:
            continue
        window_open = nearest_boolean(window_samples, current.time_s)
        if delta_m < -1.0e-6 and window_open is False:
            reference_decreases_with_closed_window += 1
        if delta_m < -1.0e-6:
            rate_mps = -delta_m / dt_s
            if previous.values[0] > args.fast_height_threshold:
                reference_rates["fast"].append(rate_mps)
            elif previous.values[0] > args.slow_height_threshold:
                reference_rates["medium"].append(rate_mps)
            else:
                reference_rates["slow"].append(rate_mps)

    target_steps = [
        abs(current.values[0] - previous.values[0])
        for previous, current in zip(target_after_start, target_after_start[1:])
    ]

    states = state_sequence(state_samples)
    forbidden_states_seen = sorted(set(states) & FORBIDDEN_STATES)
    nav_land_count = sum(1 for _, command, _ in command_samples if command == NAV_LAND_COMMAND)
    disarm_count = sum(
        1
        for _, command, param1 in command_samples
        if command == ARM_DISARM_COMMAND and param1 < 0.5
    )

    actual_heights = [sample.values[0] for sample in actual_height_samples]
    references = [sample.values[0] for sample in reference_after_start]
    estimated_heights = [sample.values[0] for sample in estimated_after_start]

    def median_or_none(values: Sequence[float]) -> Optional[float]:
        return statistics.median(values) if values else None

    result: dict[str, Any] = {
        "bag": str(bag_uri),
        "state_sequence": states,
        "descent_start_time_s": descent_start_s,
        "descent_interval_s": max(0.0, final_time_s - descent_start_s),
        "minimum_estimated_relative_height_m": min(estimated_heights),
        "maximum_estimated_relative_height_m": max(estimated_heights),
        "minimum_reference_height_m": min(references),
        "maximum_reference_height_m": max(references),
        "minimum_actual_relative_height_m": min(actual_heights) if actual_heights else None,
        "maximum_actual_relative_height_m": max(actual_heights) if actual_heights else None,
        "estimated_reference_tracking_rmse_m": rmse(estimated_reference_errors),
        "actual_reference_tracking_rmse_m": (
            rmse(actual_reference_errors) if actual_reference_errors else None
        ),
        "fast_reference_rate_mps_median": median_or_none(reference_rates["fast"]),
        "medium_reference_rate_mps_median": median_or_none(reference_rates["medium"]),
        "slow_reference_rate_mps_median": median_or_none(reference_rates["slow"]),
        "reference_decreases_with_closed_window_count": (
            reference_decreases_with_closed_window
        ),
        "maximum_target_z_step_m": max(target_steps, default=0.0),
        "nav_land_command_count": nav_land_count,
        "disarm_command_count": disarm_count,
        "forbidden_states_seen": forbidden_states_seen,
        "test_height_hold_reached": "TEST_HEIGHT_HOLD" in states,
        "recovery_climb_seen": "RECOVER_CLIMB" in states,
        "relative_height_sample_count": len(estimated_after_start),
        "actual_height_sample_count": len(actual_height_samples),
    }
    return result


def print_human(result: dict[str, Any]) -> None:
    print(f"Bag: {result['bag']}")
    print(f"State sequence: {' -> '.join(result['state_sequence'])}")
    print(f"Descent interval: {result['descent_interval_s']:.3f} s")
    print(
        "Estimated relative height min / max: "
        f"{result['minimum_estimated_relative_height_m']:.4f} / "
        f"{result['maximum_estimated_relative_height_m']:.4f} m"
    )
    print(
        "Reference height min / max: "
        f"{result['minimum_reference_height_m']:.4f} / "
        f"{result['maximum_reference_height_m']:.4f} m"
    )
    if result["minimum_actual_relative_height_m"] is not None:
        print(
            "Ground-truth relative height min / max: "
            f"{result['minimum_actual_relative_height_m']:.4f} / "
            f"{result['maximum_actual_relative_height_m']:.4f} m"
        )
    print(
        "Estimated / actual reference tracking RMSE: "
        f"{result['estimated_reference_tracking_rmse_m']:.4f} / "
        f"{result['actual_reference_tracking_rmse_m']:.4f} m"
        if result["actual_reference_tracking_rmse_m"] is not None
        else f"Estimated reference tracking RMSE: {result['estimated_reference_tracking_rmse_m']:.4f} m"
    )
    print(
        "Reference-rate medians fast / medium / slow: "
        f"{result['fast_reference_rate_mps_median']} / "
        f"{result['medium_reference_rate_mps_median']} / "
        f"{result['slow_reference_rate_mps_median']} m/s"
    )
    print(
        "Reference decreases while window closed: "
        f"{result['reference_decreases_with_closed_window_count']}"
    )
    print(f"Maximum target-z step: {result['maximum_target_z_step_m']:.5f} m")
    print(
        "NAV_LAND / Disarm commands: "
        f"{result['nav_land_command_count']} / {result['disarm_command_count']}"
    )
    print(f"Forbidden states seen: {result['forbidden_states_seen'] or 'none'}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate P5B relative-height descent metrics from a ROS 2 bag."
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
    parser.add_argument("--fast-height-threshold", type=float, default=2.0)
    parser.add_argument("--slow-height-threshold", type=float, default=0.8)
    parser.add_argument(
        "--world-origin-latitude", type=float, default=47.397971057728974
    )
    parser.add_argument(
        "--world-origin-longitude", type=float, default=8.546163739800146
    )
    parser.add_argument("--world-origin-altitude", type=float, default=0.0)
    parser.add_argument("--json", action="store_true", help="print JSON output")
    return parser.parse_args()


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
        print_human(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
