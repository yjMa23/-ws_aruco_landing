#!/usr/bin/env python3
"""离线评测 heave touchdown 升沉甲板最终下降、真实接触与相对保持。"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path
from typing import Any, Optional, Sequence

from evaluate_horizontal_tracking import (
    GeodeticPosition,
    TimedVector,
    finite_values,
    interpolate,
    world_enu_to_local_ned,
)
from evaluate_final_descent_touchdown import (
    evaluate as evaluate_final_descent,
    load_ros_modules,
    resolve_bag_uri,
)

STATE_TOPIC = "/landing/state"
TOUCHDOWN_STATUS_TOPIC = "/landing/touchdown_status"
VERTICAL_STATE_TOPIC = "/landing/vertical_state"
UAV_VERTICAL_VELOCITY_TOPIC = "/landing/uav_vertical_velocity"
RELATIVE_VERTICAL_VELOCITY_TOPIC = "/landing/relative_vertical_velocity"
HOLD_REFERENCE_TOPIC = "/landing/touchdown_hold_relative_height_reference"
HOLD_TARGET_TOPIC = "/landing/touchdown_hold_vertical_target"
HOLD_MODE_TOPIC = "/landing/touchdown_hold_mode"
HOLD_REASON_TOPIC = "/landing/touchdown_hold_reason"
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"

REQUIRED_HEAVE_TOUCHDOWN_TOPICS = {
    STATE_TOPIC,
    TOUCHDOWN_STATUS_TOPIC,
    VERTICAL_STATE_TOPIC,
    UAV_VERTICAL_VELOCITY_TOPIC,
    RELATIVE_VERTICAL_VELOCITY_TOPIC,
    HOLD_REFERENCE_TOPIC,
    HOLD_TARGET_TOPIC,
    HOLD_MODE_TOPIC,
    HOLD_REASON_TOPIC,
    LOCAL_POSITION_TOPIC,
    GROUND_TRUTH_TOPIC,
}


def percentile(values: Sequence[float], quantile: float) -> float:
    """使用线性插值计算有限样本分位数。"""

    finite = sorted(float(value) for value in values if math.isfinite(value))
    if not finite:
        return math.nan
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be within [0, 1]")
    if len(finite) == 1:
        return finite[0]
    position = quantile * (len(finite) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return finite[lower]
    alpha = position - lower
    return finite[lower] * (1.0 - alpha) + finite[upper] * alpha


def transition_entry_count(values: Sequence[str], target: str) -> int:
    """统计压缩状态序列进入 target 的次数。"""

    count = 0
    previous: Optional[str] = None
    for value in values:
        if value == target and previous != target:
            count += 1
        previous = value
    return count


def contact_transition_counts(
    clearances_m: Sequence[float],
    *,
    contact_enter_m: float,
    detached_enter_m: float,
) -> tuple[int, int]:
    """使用接触/离板迟滞统计离板和二次接触次数。"""

    if not math.isfinite(contact_enter_m) or not math.isfinite(detached_enter_m):
        raise ValueError("contact thresholds must be finite")
    if detached_enter_m <= contact_enter_m:
        raise ValueError("detached_enter_m must exceed contact_enter_m")
    finite = [float(value) for value in clearances_m if math.isfinite(value)]
    if not finite:
        return 0, 0

    in_contact = finite[0] < detached_enter_m
    detachments = 0
    secondary_contacts = 0
    for clearance_m in finite[1:]:
        if in_contact and clearance_m >= detached_enter_m:
            in_contact = False
            detachments += 1
        elif not in_contact and clearance_m <= contact_enter_m:
            in_contact = True
            secondary_contacts += 1
    return detachments, secondary_contacts


def _first_time(samples: Sequence[tuple[float, str]], value: str) -> Optional[float]:
    return next((time_s for time_s, sample in samples if sample == value), None)


def _values_after(samples: Sequence[TimedVector], start_s: Optional[float]) -> list[TimedVector]:
    if start_s is None:
        return []
    return [sample for sample in samples if sample.time_s >= start_s]


def _span(samples: Sequence[TimedVector]) -> float:
    values = [sample.values[0] for sample in samples if math.isfinite(sample.values[0])]
    return max(values) - min(values) if values else math.nan


def _interpolated_scalar(samples: Sequence[TimedVector], time_s: Optional[float]) -> float:
    if time_s is None:
        return math.nan
    value = interpolate(samples, time_s)
    return value[0] if value is not None and math.isfinite(value[0]) else math.nan


def _load_heave_touchdown_samples(args: argparse.Namespace) -> dict[str, Any]:
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
    missing = REQUIRED_HEAVE_TOUCHDOWN_TOPICS - topic_types.keys()
    if missing:
        raise RuntimeError(
            "bag is missing required heave touchdown topics: " + ", ".join(sorted(missing))
        )
    message_types = {
        topic: get_message(topic_types[topic]) for topic in REQUIRED_HEAVE_TOUCHDOWN_TOPICS
    }

    states: list[tuple[float, str]] = []
    touchdown_status: list[tuple[float, str]] = []
    vertical_state: list[TimedVector] = []
    uav_vertical_velocity: list[TimedVector] = []
    relative_vertical_velocity: list[TimedVector] = []
    hold_reference: list[TimedVector] = []
    hold_target: list[TimedVector] = []
    hold_mode: list[tuple[float, str]] = []
    hold_reason: list[tuple[float, str]] = []
    local_position: list[tuple[float, Any]] = []
    ground_truth_position_world: list[TimedVector] = []
    ground_truth_velocity_ned: list[TimedVector] = []

    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        time_s = timestamp_ns * 1.0e-9
        message = deserialize_message(serialized_data, message_types[topic])
        if topic == STATE_TOPIC:
            states.append((time_s, str(message.data)))
        elif topic == TOUCHDOWN_STATUS_TOPIC:
            touchdown_status.append((time_s, str(message.data)))
        elif topic == VERTICAL_STATE_TOPIC:
            values = (
                float(message.pose.pose.position.z),
                float(message.twist.twist.linear.z),
            )
            if finite_values(values):
                vertical_state.append(TimedVector(time_s, values))
        elif topic == UAV_VERTICAL_VELOCITY_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                uav_vertical_velocity.append(TimedVector(time_s, (value,)))
        elif topic == RELATIVE_VERTICAL_VELOCITY_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                relative_vertical_velocity.append(TimedVector(time_s, (value,)))
        elif topic == HOLD_REFERENCE_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                hold_reference.append(TimedVector(time_s, (value,)))
        elif topic == HOLD_TARGET_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                hold_target.append(TimedVector(time_s, (value,)))
        elif topic == HOLD_MODE_TOPIC:
            hold_mode.append((time_s, str(message.data)))
        elif topic == HOLD_REASON_TOPIC:
            hold_reason.append((time_s, str(message.data)))
        elif topic == LOCAL_POSITION_TOPIC:
            local_position.append((time_s, message))
        elif topic == GROUND_TRUTH_TOPIC:
            position = message.pose.pose.position
            position_values = (float(position.x), float(position.y), float(position.z))
            if finite_values(position_values):
                ground_truth_position_world.append(TimedVector(time_s, position_values))
            velocity_z_enu_mps = float(message.twist.twist.linear.z)
            if math.isfinite(velocity_z_enu_mps):
                ground_truth_velocity_ned.append(
                    TimedVector(time_s, (-velocity_z_enu_mps,))
                )

    final_start_s = _first_time(states, "FINAL_DESCENT")
    confirmed_start_s = _first_time(touchdown_status, "CONFIRMED")
    hold_start_s = _first_time(states, "TOUCHDOWN_HOLD")
    if final_start_s is None:
        raise RuntimeError("bag never entered FINAL_DESCENT")

    reference_message = next(
        (
            message
            for time_s, message in local_position
            if time_s >= final_start_s
            and bool(message.xy_global)
            and bool(message.z_global)
            and finite_values((message.ref_lat, message.ref_lon, message.ref_alt))
        ),
        None,
    )
    if reference_message is None:
        raise RuntimeError("no valid PX4 local geodetic reference is available")
    local_origin = GeodeticPosition(
        float(reference_message.ref_lat),
        float(reference_message.ref_lon),
        float(reference_message.ref_alt),
    )
    world_origin = GeodeticPosition(
        args.world_origin_latitude,
        args.world_origin_longitude,
        args.world_origin_altitude,
    )
    ground_truth_position_ned = [
        TimedVector(
            sample.time_s,
            world_enu_to_local_ned(sample.values, world_origin, local_origin),
        )
        for sample in ground_truth_position_world
    ]

    actual_relative_height: list[TimedVector] = []
    contact_clearance: list[TimedVector] = []
    for time_s, message in local_position:
        if (
            time_s < final_start_s
            or not bool(message.z_valid)
            or not math.isfinite(float(message.z))
        ):
            continue
        deck_position = interpolate(ground_truth_position_ned, time_s)
        if deck_position is None:
            continue
        relative_height_m = deck_position[2] - float(message.z)
        actual_relative_height.append(TimedVector(time_s, (relative_height_m,)))
        contact_clearance.append(
            TimedVector(
                time_s,
                (relative_height_m - args.vehicle_reference_to_contact_m,),
            )
        )

    return {
        "states": states,
        "touchdown_status": touchdown_status,
        "vertical_state": vertical_state,
        "uav_vertical_velocity": uav_vertical_velocity,
        "relative_vertical_velocity": relative_vertical_velocity,
        "hold_reference": hold_reference,
        "hold_target": hold_target,
        "hold_mode": hold_mode,
        "hold_reason": hold_reason,
        "ground_truth_position_ned": ground_truth_position_ned,
        "ground_truth_velocity_ned": ground_truth_velocity_ned,
        "actual_relative_height": actual_relative_height,
        "contact_clearance": contact_clearance,
        "final_start_s": final_start_s,
        "confirmed_start_s": confirmed_start_s,
        "hold_start_s": hold_start_s,
    }


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    base_args = argparse.Namespace(**vars(args))
    base_args.require_moving_deck = False
    base_args.minimum_deck_displacement_m = 0.0
    # heave touchdown 要求世界系 z 目标随甲板运动，因此只在 final descent 子评测中关闭“目标冻结”门槛。
    base_args.maximum_hold_target_span_m = math.inf
    base_args.maximum_hold_z_velocity_mps = math.inf
    base_result = evaluate_final_descent(base_args)
    samples = _load_heave_touchdown_samples(args)

    final_start_s = samples["final_start_s"]
    confirmed_start_s = samples["confirmed_start_s"]
    hold_start_s = samples["hold_start_s"]
    hold_measurement_start_s = (
        hold_start_s + args.hold_settling_time_s if hold_start_s is not None else None
    )

    deck_z_final = _values_after(samples["ground_truth_position_ned"], final_start_s)
    deck_z_scalar = [TimedVector(sample.time_s, (sample.values[2],)) for sample in deck_z_final]
    hold_heights = _values_after(samples["actual_relative_height"], hold_measurement_start_s)
    hold_clearances = _values_after(samples["contact_clearance"], hold_measurement_start_s)
    hold_relative_velocity = _values_after(
        samples["relative_vertical_velocity"], hold_measurement_start_s
    )
    hold_reference = _values_after(samples["hold_reference"], hold_measurement_start_s)
    hold_target = _values_after(samples["hold_target"], hold_measurement_start_s)
    clearance_values = [sample.values[0] for sample in hold_clearances]
    detachments, secondary_contacts = contact_transition_counts(
        clearance_values,
        contact_enter_m=args.contact_enter_clearance_m,
        detached_enter_m=args.detached_enter_clearance_m,
    )

    vertical_state_velocity = [
        TimedVector(sample.time_s, (sample.values[1],))
        for sample in samples["vertical_state"]
    ]
    touchdown_deck_vertical_velocity_mps = _interpolated_scalar(
        vertical_state_velocity, confirmed_start_s
    )
    touchdown_gt_deck_vertical_velocity_mps = _interpolated_scalar(
        samples["ground_truth_velocity_ned"], confirmed_start_s
    )
    touchdown_uav_vertical_velocity_mps = _interpolated_scalar(
        samples["uav_vertical_velocity"], confirmed_start_s
    )
    touchdown_relative_vertical_velocity_mps = _interpolated_scalar(
        samples["relative_vertical_velocity"], confirmed_start_s
    )
    touchdown_contact_clearance_m = _interpolated_scalar(
        samples["contact_clearance"], confirmed_start_s
    )
    candidate_contact_clearance_m = _interpolated_scalar(
        samples["contact_clearance"], base_result.get("candidate_start_s")
    )

    status_values = [value for _, value in samples["touchdown_status"]]
    candidate_entry_count = transition_entry_count(status_values, "CANDIDATE")
    hold_modes = [
        value
        for time_s, value in samples["hold_mode"]
        if hold_start_s is not None and time_s >= hold_start_s
    ]
    hold_reasons = [
        value
        for time_s, value in samples["hold_reason"]
        if hold_start_s is not None and time_s >= hold_start_s
    ]

    deck_vertical_span_m = _span(deck_z_scalar)
    hold_relative_height_span_m = _span(hold_heights)
    hold_relative_velocity_p95_mps = percentile(
        [abs(sample.values[0]) for sample in hold_relative_velocity], 0.95
    )
    hold_clearance_min_m = min(clearance_values, default=math.nan)
    hold_clearance_max_m = max(clearance_values, default=math.nan)
    hold_reference_span_m = _span(hold_reference)
    hold_target_span_m = _span(hold_target)
    relative_deck_hold_observed = "RELATIVE_DECK_HOLD" in hold_modes

    heave_touchdown_passed = (
        base_result.get("candidate_start_s") is not None
        and base_result.get("confirmed_start_s") is not None
        and base_result.get("touchdown_hold_start_s") is not None
        and base_result.get("rate_profile_passed") is True
        and base_result.get("horizontal_tracking_passed") is True
        and base_result.get("physical_contact_passed") is True
        and float(base_result.get("hold_duration_s", 0.0)) >= args.minimum_hold_duration_s
        and math.isfinite(deck_vertical_span_m)
        and deck_vertical_span_m >= args.minimum_deck_vertical_span_m
        and math.isfinite(touchdown_relative_vertical_velocity_mps)
        and abs(touchdown_relative_vertical_velocity_mps)
        <= args.maximum_touchdown_relative_vertical_speed_mps
        and math.isfinite(hold_relative_height_span_m)
        and hold_relative_height_span_m <= args.maximum_hold_relative_height_span_m
        and math.isfinite(hold_relative_velocity_p95_mps)
        and hold_relative_velocity_p95_mps
        <= args.maximum_hold_relative_vertical_velocity_p95_mps
        and math.isfinite(hold_clearance_max_m)
        and hold_clearance_max_m <= args.detached_enter_clearance_m
        and detachments == 0
        and secondary_contacts == 0
        and candidate_entry_count <= args.maximum_candidate_entries
        and relative_deck_hold_observed
        and base_result.get("recovery_count") == 0
        and base_result.get("nav_land_commands") == 0
        and base_result.get("disarm_commands") == 0
    )

    result = dict(base_result)
    result.update(
        {
            "deck_vertical_span_final_m": deck_vertical_span_m,
            "touchdown_deck_vertical_velocity_mps": (
                touchdown_deck_vertical_velocity_mps
                if math.isfinite(touchdown_deck_vertical_velocity_mps)
                else None
            ),
            "touchdown_ground_truth_deck_vertical_velocity_mps": (
                touchdown_gt_deck_vertical_velocity_mps
                if math.isfinite(touchdown_gt_deck_vertical_velocity_mps)
                else None
            ),
            "touchdown_uav_vertical_velocity_mps": (
                touchdown_uav_vertical_velocity_mps
                if math.isfinite(touchdown_uav_vertical_velocity_mps)
                else None
            ),
            "touchdown_relative_vertical_velocity_mps": (
                touchdown_relative_vertical_velocity_mps
                if math.isfinite(touchdown_relative_vertical_velocity_mps)
                else None
            ),
            "candidate_contact_clearance_m": (
                candidate_contact_clearance_m
                if math.isfinite(candidate_contact_clearance_m)
                else None
            ),
            "touchdown_contact_clearance_m": (
                touchdown_contact_clearance_m
                if math.isfinite(touchdown_contact_clearance_m)
                else None
            ),
            "contact_clearance_min_after_hold_m": (
                hold_clearance_min_m if math.isfinite(hold_clearance_min_m) else None
            ),
            "contact_clearance_max_after_hold_m": (
                hold_clearance_max_m if math.isfinite(hold_clearance_max_m) else None
            ),
            "hold_relative_height_span_m": (
                hold_relative_height_span_m
                if math.isfinite(hold_relative_height_span_m)
                else None
            ),
            "hold_relative_vertical_velocity_p95_mps": (
                hold_relative_velocity_p95_mps
                if math.isfinite(hold_relative_velocity_p95_mps)
                else None
            ),
            "hold_relative_height_reference_span_m": (
                hold_reference_span_m if math.isfinite(hold_reference_span_m) else None
            ),
            "hold_vertical_target_span_m": (
                hold_target_span_m if math.isfinite(hold_target_span_m) else None
            ),
            "detach_count": detachments,
            "secondary_contact_count": secondary_contacts,
            "candidate_entry_count": candidate_entry_count,
            "candidate_repeat_count": max(0, candidate_entry_count - 1),
            "touchdown_hold_mode_counts": dict(collections.Counter(hold_modes)),
            "touchdown_hold_reason_counts": dict(collections.Counter(hold_reasons)),
            "relative_deck_hold_observed": relative_deck_hold_observed,
            "heave_touchdown_passed": heave_touchdown_passed,
            # 保持与现有单轮/批量实验读取接口兼容。
            "positive_touchdown_passed": heave_touchdown_passed,
        }
    )
    return result


def print_human_readable(result: dict[str, Any]) -> None:
    print(f"Bag: {result['bag']}")
    print("State sequence: " + " -> ".join(result["state_sequence"]))
    print(
        "Final deck vertical span: "
        f"{result['deck_vertical_span_final_m']} m"
    )
    print(
        "Touchdown deck / UAV / relative vertical velocity: "
        f"{result['touchdown_deck_vertical_velocity_mps']} / "
        f"{result['touchdown_uav_vertical_velocity_mps']} / "
        f"{result['touchdown_relative_vertical_velocity_mps']} m/s"
    )
    print(
        "Hold relative-height span / relative-vz P95: "
        f"{result['hold_relative_height_span_m']} m / "
        f"{result['hold_relative_vertical_velocity_p95_mps']} m/s"
    )
    print(
        "Candidate / confirmed contact clearance: "
        f"{result['candidate_contact_clearance_m']} / "
        f"{result['touchdown_contact_clearance_m']} m"
    )
    print(
        "Hold clearance min / max: "
        f"{result['contact_clearance_min_after_hold_m']} / "
        f"{result['contact_clearance_max_after_hold_m']} m"
    )
    print(
        "Detach / secondary contact / candidate repeats / recovery: "
        f"{result['detach_count']} / {result['secondary_contact_count']} / "
        f"{result['candidate_repeat_count']} / {result['recovery_count']}"
    )
    print(
        "Hold modes / reasons: "
        f"{result['touchdown_hold_mode_counts']} / "
        f"{result['touchdown_hold_reason_counts']}"
    )
    print(
        "NAV_LAND / Disarm: "
        f"{result['nav_land_commands']} / {result['disarm_commands']}"
    )
    print(
        "heave touchdown heave touchdown test: "
        + ("PASS" if result["heave_touchdown_passed"] else "FAIL")
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate heave touchdown heave touchdown and relative-height hold."
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
    parser.add_argument("--json", action="store_true", help="print JSON output")
    parser.add_argument("--reference-tolerance-m", type=float, default=1.0e-4)
    parser.add_argument("--minimum-hold-duration-s", type=float, default=10.0)
    parser.add_argument("--hold-settling-time-s", type=float, default=0.50)
    parser.add_argument("--vehicle-reference-to-contact-m", type=float, default=0.227)
    parser.add_argument("--maximum-contact-clearance-m", type=float, default=0.03)
    parser.add_argument("--maximum-contact-penetration-m", type=float, default=0.05)
    parser.add_argument("--slowdown-height-m", type=float, default=0.25)
    parser.add_argument("--maximum-approach-rate-mps", type=float, default=0.20)
    parser.add_argument("--maximum-contact-rate-mps", type=float, default=0.05)
    parser.add_argument("--maximum-horizontal-error-m", type=float, default=0.30)
    parser.add_argument("--minimum-deck-vertical-span-m", type=float, default=0.10)
    parser.add_argument(
        "--maximum-touchdown-relative-vertical-speed-mps",
        type=float,
        default=0.12,
    )
    parser.add_argument(
        "--maximum-hold-relative-height-span-m", type=float, default=0.08
    )
    parser.add_argument(
        "--maximum-hold-relative-vertical-velocity-p95-mps",
        type=float,
        default=0.12,
    )
    parser.add_argument("--contact-enter-clearance-m", type=float, default=0.03)
    parser.add_argument("--detached-enter-clearance-m", type=float, default=0.05)
    parser.add_argument(
        "--maximum-candidate-entries",
        type=int,
        default=2,
        help=(
            "maximum pre-confirmation candidate entries; one re-entry is allowed for "
            "PX4 contact evidence handing over to the terminal-stall path"
        ),
    )
    parser.add_argument(
        "--world-origin-latitude", type=float, default=47.397971057728974
    )
    parser.add_argument(
        "--world-origin-longitude", type=float, default=8.546163739800146
    )
    parser.add_argument("--world-origin-altitude", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        result = evaluate(args)
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print_human_readable(result)
    return 0 if result["heave_touchdown_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
