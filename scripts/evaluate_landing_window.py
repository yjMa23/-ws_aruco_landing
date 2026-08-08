#!/usr/bin/env python3
"""离线评测 landing window 甲板姿态估计与规则式着陆窗口 rosbag。"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Optional, Sequence


STATE_TOPIC = "/landing/state"
WINDOW_OPEN_TOPIC = "/landing/window_open"
WINDOW_REASONS_TOPIC = "/landing/window_reject_reasons"
WINDOW_DURATION_TOPIC = "/landing/window_satisfied_duration"
ATTITUDE_TOPIC = "/landing/estimated_deck_attitude"
TARGET_TOPIC = "/landing/target_pose"
GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"

WAIT_STATE = "WAIT_LANDING_WINDOW"
FORBIDDEN_STATES = {
    "CENTER_ABOVE_MARKER",
    "DESCEND_WITH_TRACKING",
    "FINAL_LAND",
    "DONE",
}

REQUIRED_TOPICS = {
    STATE_TOPIC,
    WINDOW_OPEN_TOPIC,
    WINDOW_REASONS_TOPIC,
    WINDOW_DURATION_TOPIC,
    ATTITUDE_TOPIC,
    TARGET_TOPIC,
}

REJECT_REASON_NAMES = {
    1 << 0: "visual_unavailable",
    1 << 1: "visual_too_old",
    1 << 2: "estimate_invalid",
    1 << 3: "prediction_invalid",
    1 << 4: "horizontal_error",
    1 << 5: "relative_speed",
    1 << 6: "deck_tilt",
    1 << 7: "relative_height",
    1 << 8: "invalid_time",
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
            "ROS 2 Python modules are unavailable. Source ROS Humble and the workspace first."
        ) from error
    return rosbag2_py, deserialize_message, get_message, StorageOptions, ConverterOptions


def quaternion_total_tilt_rad(quaternion: Sequence[float]) -> float:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("invalid quaternion")
    w, x, y, z = (value / norm for value in quaternion)
    # R_zz is the world-up component of the body's +Z axis. Total plane tilt is yaw independent.
    r_zz = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, r_zz)))


def interpolate_scalar(
    samples: Sequence[tuple[float, float]], query_time_s: float
) -> Optional[float]:
    if not samples or query_time_s < samples[0][0] or query_time_s > samples[-1][0]:
        return None
    times = [sample[0] for sample in samples]
    upper_index = bisect.bisect_left(times, query_time_s)
    if upper_index == 0:
        return samples[0][1]
    if upper_index == len(samples):
        return samples[-1][1]
    upper_time, upper_value = samples[upper_index]
    lower_time, lower_value = samples[upper_index - 1]
    if query_time_s == upper_time:
        return upper_value
    duration_s = upper_time - lower_time
    if duration_s <= 0.0:
        return None
    alpha = (query_time_s - lower_time) / duration_s
    return lower_value + alpha * (upper_value - lower_value)


def decode_reasons(mask: int) -> list[str]:
    if mask == 0:
        return ["none"]
    names = [name for bit, name in REJECT_REASON_NAMES.items() if mask & bit]
    unknown = mask & ~sum(REJECT_REASON_NAMES.keys())
    if unknown:
        names.append(f"unknown_0x{unknown:x}")
    return names


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

    selected_topics = REQUIRED_TOPICS | ({GROUND_TRUTH_TOPIC} & topic_types.keys())
    message_types = {topic: get_message(topic_types[topic]) for topic in selected_topics}

    state_samples: list[tuple[float, str]] = []
    window_samples: list[tuple[float, bool]] = []
    reason_samples: list[tuple[float, int]] = []
    duration_samples: list[tuple[float, float]] = []
    attitude_samples: list[tuple[float, float, float, float]] = []
    target_samples: list[tuple[float, float]] = []
    ground_truth_tilt_samples: list[tuple[float, float]] = []

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
        elif topic == WINDOW_REASONS_TOPIC:
            reason_samples.append((time_s, int(message.data)))
        elif topic == WINDOW_DURATION_TOPIC:
            duration_samples.append((time_s, float(message.data)))
        elif topic == ATTITUDE_TOPIC:
            values = (float(message.vector.x), float(message.vector.y), float(message.vector.z))
            if all(math.isfinite(value) for value in values):
                attitude_samples.append((time_s, *values))
        elif topic == TARGET_TOPIC:
            target_z = float(message.pose.position.z)
            if math.isfinite(target_z):
                target_samples.append((time_s, target_z))
        elif topic == GROUND_TRUTH_TOPIC:
            q = message.pose.pose.orientation
            try:
                tilt = quaternion_total_tilt_rad((q.w, q.x, q.y, q.z))
            except ValueError:
                continue
            ground_truth_tilt_samples.append((time_s, tilt))

    wait_times = [time_s for time_s, state in state_samples if state == WAIT_STATE]
    if not wait_times:
        raise RuntimeError(f"bag never entered {WAIT_STATE}")
    wait_start_s = wait_times[0]
    wait_end_s = max(time_s for time_s, state in state_samples if state == WAIT_STATE)

    forbidden_states_seen = sorted({state for _, state in state_samples if state in FORBIDDEN_STATES})
    open_after_wait = [(time_s, value) for time_s, value in window_samples if time_s >= wait_start_s]
    first_open_s = next((time_s for time_s, value in open_after_wait if value), None)
    opening_delay_s = None if first_open_s is None else first_open_s - wait_start_s
    satisfied_duration_at_first_open_s = (
        interpolate_scalar(duration_samples, first_open_s)
        if first_open_s is not None
        else None
    )
    open_count = sum(1 for _, value in open_after_wait if value)
    open_fraction = open_count / len(open_after_wait) if open_after_wait else 0.0
    open_transition_count = 0
    close_transition_count = 0
    previous_open: Optional[bool] = None
    for _, is_open in open_after_wait:
        if previous_open is False and is_open:
            open_transition_count += 1
        elif previous_open is True and not is_open:
            close_transition_count += 1
        previous_open = is_open

    wait_reasons = [mask for time_s, mask in reason_samples if time_s >= wait_start_s]
    reason_mask_counts = Counter(wait_reasons)
    reason_name_counts: Counter[str] = Counter()
    for mask in wait_reasons:
        for name in decode_reasons(mask):
            reason_name_counts[name] += 1

    wait_durations = [value for time_s, value in duration_samples if time_s >= wait_start_s]
    wait_attitudes = [sample for sample in attitude_samples if sample[0] >= wait_start_s]
    wait_targets = [z for time_s, z in target_samples if time_s >= wait_start_s]

    if not wait_targets:
        raise RuntimeError("no target-pose samples were found during WAIT_LANDING_WINDOW")
    if not wait_attitudes:
        raise RuntimeError("no estimated-deck-attitude samples were found during WAIT_LANDING_WINDOW")

    tilt_errors: list[float] = []
    for time_s, _, _, estimated_tilt in wait_attitudes:
        ground_truth_tilt = interpolate_scalar(ground_truth_tilt_samples, time_s)
        if ground_truth_tilt is not None:
            tilt_errors.append(estimated_tilt - ground_truth_tilt)

    result: dict[str, Any] = {
        "bag": str(bag_uri),
        "wait_start_time_s": wait_start_s,
        "wait_duration_s": max(0.0, wait_end_s - wait_start_s),
        "first_window_open_time_s": first_open_s,
        "window_opening_delay_s": opening_delay_s,
        "satisfied_duration_at_first_open_s": satisfied_duration_at_first_open_s,
        "window_ever_opened": first_open_s is not None,
        "window_open_sample_fraction": open_fraction,
        "window_open_transition_count": open_transition_count,
        "window_close_transition_count": close_transition_count,
        "maximum_satisfied_duration_s": max(wait_durations, default=0.0),
        "target_z_min_m": min(wait_targets),
        "target_z_max_m": max(wait_targets),
        "target_z_span_m": max(wait_targets) - min(wait_targets),
        "maximum_absolute_estimated_roll_deg": max(
            abs(math.degrees(sample[1])) for sample in wait_attitudes
        ),
        "maximum_absolute_estimated_pitch_deg": max(
            abs(math.degrees(sample[2])) for sample in wait_attitudes
        ),
        "maximum_estimated_tilt_deg": max(
            math.degrees(sample[3]) for sample in wait_attitudes
        ),
        "estimated_tilt_rmse_deg": (
            math.degrees(math.sqrt(sum(error * error for error in tilt_errors) / len(tilt_errors)))
            if tilt_errors
            else None
        ),
        "forbidden_states_seen": forbidden_states_seen,
        "reject_mask_counts": {str(mask): count for mask, count in sorted(reason_mask_counts.items())},
        "reject_reason_counts": dict(sorted(reason_name_counts.items())),
        "wait_state_sample_count": len(wait_times),
        "window_sample_count": len(open_after_wait),
        "attitude_sample_count": len(wait_attitudes),
    }
    return result


def print_human(result: dict[str, Any]) -> None:
    print(f"Bag: {result['bag']}")
    print(f"WAIT_LANDING_WINDOW duration: {result['wait_duration_s']:.3f} s")
    if result["window_ever_opened"]:
        print(f"First window opening delay: {result['window_opening_delay_s']:.3f} s")
        print(
            "Satisfied duration at first opening: "
            f"{result['satisfied_duration_at_first_open_s']:.3f} s"
        )
    else:
        print("Window never opened")
    print(f"Window-open sample fraction: {result['window_open_sample_fraction']:.3f}")
    print(
        "Window open / close transitions: "
        f"{result['window_open_transition_count']} / "
        f"{result['window_close_transition_count']}"
    )
    print(f"Maximum satisfied duration: {result['maximum_satisfied_duration_s']:.3f} s")
    print(
        "Target z min / max / span: "
        f"{result['target_z_min_m']:.4f} / {result['target_z_max_m']:.4f} / "
        f"{result['target_z_span_m']:.6f} m"
    )
    print(
        "Maximum estimated |roll| / |pitch| / tilt: "
        f"{result['maximum_absolute_estimated_roll_deg']:.3f} / "
        f"{result['maximum_absolute_estimated_pitch_deg']:.3f} / "
        f"{result['maximum_estimated_tilt_deg']:.3f} deg"
    )
    if result["estimated_tilt_rmse_deg"] is not None:
        print(f"Estimated tilt RMSE: {result['estimated_tilt_rmse_deg']:.3f} deg")
    print(f"Forbidden states seen: {result['forbidden_states_seen'] or 'none'}")
    print(f"Reject mask counts: {result['reject_mask_counts']}")
    print(f"Reject reason counts: {result['reject_reason_counts']}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate landing window landing-window and deck-attitude metrics from a ROS 2 bag."
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
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
