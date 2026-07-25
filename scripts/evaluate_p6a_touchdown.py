#!/usr/bin/env python3
"""离线评测 P6A 多源触地候选与确认调试话题。"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path
from typing import Any


STATUS_TOPIC = "/landing/touchdown_status"
EVIDENCE_TOPIC = "/landing/touchdown_evidence"
DURATION_TOPIC = "/landing/touchdown_candidate_duration"
CONFIRMED_TOPIC = "/landing/touchdown_confirmed"
STATE_TOPIC = "/landing/state"
LAND_DETECTED_TOPIC = "/fmu/out/vehicle_land_detected"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"

REQUIRED_TOPICS = {
    STATUS_TOPIC,
    EVIDENCE_TOPIC,
    DURATION_TOPIC,
    CONFIRMED_TOPIC,
    STATE_TOPIC,
    LAND_DETECTED_TOPIC,
    VEHICLE_COMMAND_TOPIC,
}

NAV_LAND_COMMAND = 21
ARM_DISARM_COMMAND = 400
ALLOWED_P6A_STATE = "TEST_HEIGHT_HOLD"


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


def evaluate(path: Path) -> dict[str, Any]:
    (
        rosbag2_py,
        deserialize_message,
        get_message,
        storage_options_type,
        converter_options_type,
    ) = load_ros_modules()
    bag_uri = resolve_bag_uri(path)

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

    status_counts: collections.Counter[str] = collections.Counter()
    status_by_state: dict[str, collections.Counter[str]] = collections.defaultdict(
        collections.Counter
    )
    evidence_counts: collections.Counter[int] = collections.Counter()
    px4_true_counts: collections.Counter[str] = collections.Counter()
    state_sequence: list[str] = []
    maximum_candidate_duration_s = 0.0
    candidate_samples = 0
    confirmed_true_samples = 0
    candidate_outside_allowed_state = 0
    confirmed_outside_allowed_state = 0
    nav_land_commands = 0
    disarm_commands = 0
    current_state = "UNKNOWN"

    while reader.has_next():
        topic, serialized_data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized_data, message_types[topic])

        if topic == STATE_TOPIC:
            current_state = str(message.data)
            if not state_sequence or state_sequence[-1] != current_state:
                state_sequence.append(current_state)
        elif topic == STATUS_TOPIC:
            status = str(message.data)
            status_counts[status] += 1
            status_by_state[current_state][status] += 1
            if status == "CANDIDATE":
                candidate_samples += 1
                if current_state != ALLOWED_P6A_STATE:
                    candidate_outside_allowed_state += 1
            elif status == "CONFIRMED" and current_state != ALLOWED_P6A_STATE:
                confirmed_outside_allowed_state += 1
        elif topic == EVIDENCE_TOPIC:
            evidence_counts[int(message.data)] += 1
        elif topic == DURATION_TOPIC:
            duration_s = float(message.data)
            if math.isfinite(duration_s):
                maximum_candidate_duration_s = max(
                    maximum_candidate_duration_s, duration_s
                )
        elif topic == CONFIRMED_TOPIC:
            confirmed_true_samples += int(bool(message.data))
        elif topic == LAND_DETECTED_TOPIC:
            for field in (
                "freefall",
                "ground_contact",
                "maybe_landed",
                "landed",
                "at_rest",
                "has_low_throttle",
                "vertical_movement",
                "horizontal_movement",
                "rotational_movement",
                "close_to_ground_or_skipped_check",
            ):
                px4_true_counts[field] += int(bool(getattr(message, field)))
        elif topic == VEHICLE_COMMAND_TOPIC:
            command = int(message.command)
            if command == NAV_LAND_COMMAND:
                nav_land_commands += 1
            elif command == ARM_DISARM_COMMAND and float(message.param1) < 0.5:
                disarm_commands += 1

    return {
        "bag": str(bag_uri),
        "state_sequence": state_sequence,
        "touchdown_status_counts": dict(status_counts),
        "touchdown_status_by_state": {
            state: dict(counts) for state, counts in status_by_state.items()
        },
        "evidence_mask_counts": {str(mask): count for mask, count in evidence_counts.items()},
        "maximum_candidate_duration_s": maximum_candidate_duration_s,
        "candidate_samples": candidate_samples,
        "confirmed_true_samples": confirmed_true_samples,
        "candidate_outside_allowed_state": candidate_outside_allowed_state,
        "confirmed_outside_allowed_state": confirmed_outside_allowed_state,
        "px4_land_flag_true_counts": dict(px4_true_counts),
        "nav_land_commands": nav_land_commands,
        "disarm_commands": disarm_commands,
        "negative_touchdown_test_passed": (
            status_counts.get("CONFIRMED", 0) == 0
            and confirmed_true_samples == 0
            and candidate_outside_allowed_state == 0
            and confirmed_outside_allowed_state == 0
            and nav_land_commands == 0
            and disarm_commands == 0
        ),
    }


def print_human_readable(result: dict[str, Any]) -> None:
    print(f"Bag: {result['bag']}")
    print("State sequence: " + " -> ".join(result["state_sequence"]))
    print(f"Touchdown status counts: {result['touchdown_status_counts']}")
    print(f"Touchdown status by state: {result['touchdown_status_by_state']}")
    print(f"Evidence masks: {result['evidence_mask_counts']}")
    print(
        "Candidate samples / max duration: "
        f"{result['candidate_samples']} / "
        f"{result['maximum_candidate_duration_s']:.3f} s"
    )
    print(
        "Confirmed true / outside-state candidate / outside-state confirmed: "
        f"{result['confirmed_true_samples']} / "
        f"{result['candidate_outside_allowed_state']} / "
        f"{result['confirmed_outside_allowed_state']}"
    )
    print(f"PX4 land flag true counts: {result['px4_land_flag_true_counts']}")
    print(
        "NAV_LAND / Disarm commands: "
        f"{result['nav_land_commands']} / {result['disarm_commands']}"
    )
    print(
        "P6A negative touchdown test: "
        + ("PASS" if result["negative_touchdown_test_passed"] else "FAIL")
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate P6A touchdown evidence and false confirmations."
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
    parser.add_argument("--json", action="store_true", help="print JSON output")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        result = evaluate(args.bag)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print_human_readable(result)
    return 0 if result["negative_touchdown_test_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
