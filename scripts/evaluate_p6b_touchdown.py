#!/usr/bin/env python3
"""离线评测 P6B 最终下降、触地候选、确认与保持。"""

from __future__ import annotations

import argparse
import bisect
import collections
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
FINAL_PHASE_TOPIC = "/landing/final_descent_phase"
TOUCHDOWN_STATUS_TOPIC = "/landing/touchdown_status"
TOUCHDOWN_EVIDENCE_TOPIC = "/landing/touchdown_evidence"
TOUCHDOWN_DURATION_TOPIC = "/landing/touchdown_candidate_duration"
TOUCHDOWN_CONFIRMED_TOPIC = "/landing/touchdown_confirmed"
REFERENCE_TOPIC = "/landing/relative_height_reference"
RELATIVE_HEIGHT_TOPIC = "/landing/relative_height"
TARGET_TOPIC = "/landing/target_pose"
TRAJECTORY_TOPIC = "/fmu/in/trajectory_setpoint"
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
LAND_DETECTED_TOPIC = "/fmu/out/vehicle_land_detected"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"
GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"
MARKER_ID_TOPIC = "/landing/active_marker_id"
ARUCO_VISIBLE_TOPIC = "/aruco/visible"
RELATIVE_VERTICAL_VELOCITY_TOPIC = "/landing/relative_vertical_velocity"
TOUCHDOWN_HOLD_MODE_TOPIC = "/landing/touchdown_hold_mode"

REQUIRED_TOPICS = {
    STATE_TOPIC,
    FINAL_PHASE_TOPIC,
    TOUCHDOWN_STATUS_TOPIC,
    TOUCHDOWN_EVIDENCE_TOPIC,
    TOUCHDOWN_DURATION_TOPIC,
    TOUCHDOWN_CONFIRMED_TOPIC,
    REFERENCE_TOPIC,
    RELATIVE_HEIGHT_TOPIC,
    TARGET_TOPIC,
    TRAJECTORY_TOPIC,
    LOCAL_POSITION_TOPIC,
    LAND_DETECTED_TOPIC,
    VEHICLE_COMMAND_TOPIC,
    GROUND_TRUTH_TOPIC,
}
OPTIONAL_TOPICS = {
    MARKER_ID_TOPIC,
    ARUCO_VISIBLE_TOPIC,
    RELATIVE_VERTICAL_VELOCITY_TOPIC,
    TOUCHDOWN_HOLD_MODE_TOPIC,
}

FINAL_STATES = {
    "FINAL_DESCENT",
    "TOUCHDOWN_CANDIDATE_HOLD",
    "TOUCHDOWN_HOLD",
}
NAV_LAND_COMMAND = 21
ARM_DISARM_COMMAND = 400


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


def state_sequence(samples: Sequence[tuple[float, str]]) -> list[str]:
    result: list[str] = []
    for _, value in samples:
        if not result or result[-1] != value:
            result.append(value)
    return result


def first_time(samples: Sequence[tuple[float, str]], value: str) -> Optional[float]:
    return next((time_s for time_s, sample in samples if sample == value), None)


def transition_sequence(samples: Sequence[tuple[float, Any]]) -> list[Any]:
    result: list[Any] = []
    for _, value in samples:
        if not result or result[-1] != value:
            result.append(value)
    return result


def visibility_transition_counts(
    samples: Sequence[tuple[float, bool]], start_s: float
) -> tuple[int, int]:
    filtered = [(time_s, visible) for time_s, visible in samples if time_s >= start_s]
    losses = 0
    recoveries = 0
    for (_, previous), (_, current) in zip(filtered, filtered[1:]):
        if previous and not current:
            losses += 1
        elif not previous and current:
            recoveries += 1
    return losses, recoveries


def values_after(samples: Sequence[TimedVector], start_s: Optional[float]) -> list[TimedVector]:
    if start_s is None:
        return []
    return [sample for sample in samples if sample.time_s >= start_s]


def maximum_step(samples: Sequence[TimedVector]) -> float:
    if len(samples) < 2:
        return 0.0
    return max(
        abs(current.values[0] - previous.values[0])
        for previous, current in zip(samples, samples[1:])
    )


def reference_rate_medians(
    samples: Sequence[TimedVector], slowdown_height_m: float, tolerance_m: float
) -> tuple[float, float]:
    approach_rates: list[float] = []
    contact_rates: list[float] = []
    for previous, current in zip(samples, samples[1:]):
        dt_s = current.time_s - previous.time_s
        decrease_m = previous.values[0] - current.values[0]
        if dt_s <= 0.0 or decrease_m <= tolerance_m:
            continue
        rate_mps = decrease_m / dt_s
        if previous.values[0] > slowdown_height_m + tolerance_m:
            approach_rates.append(rate_mps)
        else:
            contact_rates.append(rate_mps)
    return (
        statistics.median(approach_rates) if approach_rates else math.nan,
        statistics.median(contact_rates) if contact_rates else math.nan,
    )


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
    selected_topics = REQUIRED_TOPICS | (OPTIONAL_TOPICS & topic_types.keys())
    message_types = {topic: get_message(topic_types[topic]) for topic in selected_topics}

    states: list[tuple[float, str]] = []
    phases: list[tuple[float, str]] = []
    touchdown_status: list[tuple[float, str]] = []
    touchdown_evidence: list[TimedVector] = []
    touchdown_duration: list[TimedVector] = []
    touchdown_confirmed: list[tuple[float, bool]] = []
    references: list[TimedVector] = []
    estimated_heights: list[TimedVector] = []
    target_z: list[TimedVector] = []
    trajectory_z_velocity: list[TimedVector] = []
    local_position: list[tuple[float, Any]] = []
    land_flags: list[tuple[float, dict[str, bool]]] = []
    commands: list[tuple[int, float]] = []
    ground_truth_world: list[TimedVector] = []
    marker_ids: list[tuple[float, int]] = []
    aruco_visible: list[tuple[float, bool]] = []
    relative_vertical_velocity: list[TimedVector] = []
    touchdown_hold_modes: list[tuple[float, str]] = []

    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        time_s = timestamp_ns * 1.0e-9
        if topic not in message_types:
            continue
        message = deserialize_message(serialized_data, message_types[topic])
        if topic == STATE_TOPIC:
            states.append((time_s, str(message.data)))
        elif topic == FINAL_PHASE_TOPIC:
            phases.append((time_s, str(message.data)))
        elif topic == TOUCHDOWN_STATUS_TOPIC:
            touchdown_status.append((time_s, str(message.data)))
        elif topic == TOUCHDOWN_EVIDENCE_TOPIC:
            touchdown_evidence.append(TimedVector(time_s, (float(message.data),)))
        elif topic == TOUCHDOWN_DURATION_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                touchdown_duration.append(TimedVector(time_s, (value,)))
        elif topic == TOUCHDOWN_CONFIRMED_TOPIC:
            touchdown_confirmed.append((time_s, bool(message.data)))
        elif topic == REFERENCE_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                references.append(TimedVector(time_s, (value,)))
        elif topic == RELATIVE_HEIGHT_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                estimated_heights.append(TimedVector(time_s, (value,)))
        elif topic == TARGET_TOPIC:
            value = float(message.pose.position.z)
            if math.isfinite(value):
                target_z.append(TimedVector(time_s, (value,)))
        elif topic == TRAJECTORY_TOPIC:
            value = float(message.velocity[2])
            if math.isfinite(value):
                trajectory_z_velocity.append(TimedVector(time_s, (value,)))
        elif topic == LOCAL_POSITION_TOPIC:
            local_position.append((time_s, message))
        elif topic == LAND_DETECTED_TOPIC:
            land_flags.append(
                (
                    time_s,
                    {
                        field: bool(getattr(message, field))
                        for field in (
                            "ground_contact",
                            "maybe_landed",
                            "landed",
                            "at_rest",
                            "freefall",
                            "vertical_movement",
                            "horizontal_movement",
                            "rotational_movement",
                        )
                    },
                )
            )
        elif topic == VEHICLE_COMMAND_TOPIC:
            commands.append((int(message.command), float(message.param1)))
        elif topic == GROUND_TRUTH_TOPIC:
            position = message.pose.pose.position
            values = (float(position.x), float(position.y), float(position.z))
            if finite_values(values):
                ground_truth_world.append(TimedVector(time_s, values))
        elif topic == MARKER_ID_TOPIC:
            marker_ids.append((time_s, int(message.data)))
        elif topic == ARUCO_VISIBLE_TOPIC:
            aruco_visible.append((time_s, bool(message.data)))
        elif topic == RELATIVE_VERTICAL_VELOCITY_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                relative_vertical_velocity.append(TimedVector(time_s, (value,)))
        elif topic == TOUCHDOWN_HOLD_MODE_TOPIC:
            touchdown_hold_modes.append((time_s, str(message.data)))

    final_start_s = first_time(states, "FINAL_DESCENT")
    candidate_start_s = first_time(touchdown_status, "CANDIDATE")
    confirmed_start_s = first_time(touchdown_status, "CONFIRMED")
    hold_start_s = first_time(states, "TOUCHDOWN_HOLD")

    if final_start_s is None:
        raise RuntimeError("bag never entered FINAL_DESCENT")

    valid_reference_message = next(
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

    actual_heights: list[TimedVector] = []
    horizontal_errors_m: list[float] = []
    deck_positions_final: list[TimedVector] = []
    for time_s, message in local_position:
        if (
            time_s < final_start_s
            or not bool(message.xy_valid)
            or not bool(message.z_valid)
            or not finite_values((message.x, message.y, message.z))
        ):
            continue
        deck = interpolate(ground_truth_ned, time_s)
        if deck is None:
            continue
        actual_heights.append(TimedVector(time_s, (deck[2] - float(message.z),)))
        horizontal_errors_m.append(
            math.hypot(deck[0] - float(message.x), deck[1] - float(message.y))
        )
        deck_positions_final.append(TimedVector(time_s, (deck[0], deck[1])))

    references_final = values_after(references, final_start_s)
    target_final = values_after(target_z, final_start_s)
    velocity_final = values_after(trajectory_z_velocity, final_start_s)
    actual_final = values_after(actual_heights, final_start_s)
    approach_rate_mps, contact_rate_mps = reference_rate_medians(
        references_final,
        args.slowdown_height_m,
        args.reference_tolerance_m,
    )
    horizontal_rmse_m = (
        math.sqrt(sum(value * value for value in horizontal_errors_m) / len(horizontal_errors_m))
        if horizontal_errors_m
        else math.nan
    )
    horizontal_max_m = max(horizontal_errors_m, default=math.nan)
    deck_horizontal_displacement_m = math.nan
    if deck_positions_final:
        start_xy = deck_positions_final[0].values
        deck_horizontal_displacement_m = max(
            math.hypot(sample.values[0] - start_xy[0], sample.values[1] - start_xy[1])
            for sample in deck_positions_final
        )
    rate_profile_passed = (
        math.isfinite(approach_rate_mps)
        and math.isfinite(contact_rate_mps)
        and approach_rate_mps > contact_rate_mps
        and approach_rate_mps <= args.maximum_approach_rate_mps
        and contact_rate_mps <= args.maximum_contact_rate_mps
    )
    moving_deck_passed = (
        not args.require_moving_deck
        or (
            math.isfinite(deck_horizontal_displacement_m)
            and deck_horizontal_displacement_m >= args.minimum_deck_displacement_m
        )
    )
    horizontal_tracking_passed = (
        math.isfinite(horizontal_max_m)
        and horizontal_max_m <= args.maximum_horizontal_error_m
    )

    reference_decreases_after_candidate = 0
    if candidate_start_s is not None:
        candidate_refs = values_after(references, candidate_start_s)
        for previous, current in zip(candidate_refs, candidate_refs[1:]):
            if current.values[0] < previous.values[0] - args.reference_tolerance_m:
                reference_decreases_after_candidate += 1

    target_span_after_hold = math.nan
    max_abs_z_velocity_after_hold = 0.0
    hold_relative_height_span_m = math.nan
    hold_contact_clearance_min_m = math.nan
    hold_contact_clearance_max_m = math.nan
    hold_duration_s = 0.0
    if hold_start_s is not None:
        hold_targets_all = values_after(target_z, hold_start_s)
        hold_measurement_start_s = hold_start_s + args.hold_settling_time_s
        hold_targets = values_after(target_z, hold_measurement_start_s)
        hold_velocities = values_after(trajectory_z_velocity, hold_measurement_start_s)
        hold_actual_heights = values_after(actual_heights, hold_measurement_start_s)
        if hold_targets_all:
            hold_duration_s = hold_targets_all[-1].time_s - hold_start_s
        if hold_targets:
            target_values = [sample.values[0] for sample in hold_targets]
            target_span_after_hold = max(target_values) - min(target_values)
        if hold_velocities:
            max_abs_z_velocity_after_hold = max(abs(sample.values[0]) for sample in hold_velocities)
        if hold_actual_heights:
            hold_height_values = [sample.values[0] for sample in hold_actual_heights]
            hold_relative_height_span_m = max(hold_height_values) - min(hold_height_values)
            hold_clearances = [
                value - args.vehicle_reference_to_contact_m for value in hold_height_values
            ]
            hold_contact_clearance_min_m = min(hold_clearances)
            hold_contact_clearance_max_m = max(hold_clearances)

    relative_deck_hold_observed = any(
        value == "RELATIVE_DECK_HOLD"
        for time_s, value in touchdown_hold_modes
        if hold_start_s is not None and time_s >= hold_start_s
    )
    frozen_world_hold_passed = (
        math.isfinite(target_span_after_hold)
        and target_span_after_hold <= args.maximum_hold_target_span_m
        and math.isfinite(max_abs_z_velocity_after_hold)
        and max_abs_z_velocity_after_hold <= args.maximum_hold_z_velocity_mps
    )
    relative_deck_hold_passed = (
        math.isfinite(target_span_after_hold)
        and target_span_after_hold <= args.maximum_hold_target_span_m
        and math.isfinite(hold_relative_height_span_m)
        and hold_relative_height_span_m <= args.maximum_hold_relative_height_span_m
        and math.isfinite(hold_contact_clearance_min_m)
        and hold_contact_clearance_min_m >= -args.maximum_contact_penetration_m
        and math.isfinite(hold_contact_clearance_max_m)
        and hold_contact_clearance_max_m <= args.maximum_contact_clearance_m
    )
    hold_vertical_semantics_passed = (
        relative_deck_hold_passed
        if relative_deck_hold_observed
        else frozen_world_hold_passed
    )

    first_land_flag_times = {
        field: next(
            (
                time_s
                for time_s, flags in land_flags
                if time_s >= final_start_s and flags[field]
            ),
            None,
        )
        for field in (
            "ground_contact",
            "maybe_landed",
            "landed",
            "at_rest",
        )
    }

    marker_sequence = transition_sequence(
        [(time_s, marker_id) for time_s, marker_id in marker_ids if time_s >= final_start_s]
    )
    vision_loss_count, vision_recovery_count = visibility_transition_counts(
        aruco_visible, final_start_s
    )
    compressed_states = state_sequence(states)
    recovery_count = sum(
        state in {"RECOVER_TO_GNSS", "RECOVER_CLIMB"} for state in compressed_states
    )
    touchdown_relative_vertical_velocity_mps = math.nan
    if confirmed_start_s is not None:
        interpolated_velocity = interpolate(relative_vertical_velocity, confirmed_start_s)
        if interpolated_velocity is not None:
            touchdown_relative_vertical_velocity_mps = interpolated_velocity[0]

    ground_truth_height_min_m = min(
        (sample.values[0] for sample in actual_final), default=math.nan
    )
    ground_truth_contact_clearance_min_m = (
        ground_truth_height_min_m - args.vehicle_reference_to_contact_m
        if math.isfinite(ground_truth_height_min_m)
        else math.nan
    )
    physical_contact_passed = (
        math.isfinite(ground_truth_contact_clearance_min_m)
        and ground_truth_contact_clearance_min_m <= args.maximum_contact_clearance_m
        and ground_truth_contact_clearance_min_m >= -args.maximum_contact_penetration_m
    )

    status_counts = collections.Counter(value for _, value in touchdown_status)
    phase_counts = collections.Counter(value for _, value in phases)
    nav_land_commands = sum(command == NAV_LAND_COMMAND for command, _ in commands)
    disarm_commands = sum(
        command == ARM_DISARM_COMMAND and param1 < 0.5 for command, param1 in commands
    )

    result = {
        "bag": str(bag_uri),
        "state_sequence": compressed_states,
        "final_phase_counts": dict(phase_counts),
        "touchdown_status_counts": dict(status_counts),
        "final_descent_start_s": final_start_s,
        "candidate_start_s": candidate_start_s,
        "confirmed_start_s": confirmed_start_s,
        "touchdown_hold_start_s": hold_start_s,
        "candidate_to_confirm_delay_s": (
            confirmed_start_s - candidate_start_s
            if candidate_start_s is not None and confirmed_start_s is not None
            else None
        ),
        "landing_time_s": (
            confirmed_start_s - final_start_s if confirmed_start_s is not None else None
        ),
        "touchdown_vertical_speed_mps": (
            abs(touchdown_relative_vertical_velocity_mps)
            if math.isfinite(touchdown_relative_vertical_velocity_mps)
            else None
        ),
        "touchdown_relative_vertical_velocity_mps": (
            touchdown_relative_vertical_velocity_mps
            if math.isfinite(touchdown_relative_vertical_velocity_mps)
            else None
        ),
        "reference_min_m": min((sample.values[0] for sample in references_final), default=math.nan),
        "estimated_height_min_m": min(
            (sample.values[0] for sample in values_after(estimated_heights, final_start_s)),
            default=math.nan,
        ),
        "ground_truth_height_min_m": ground_truth_height_min_m,
        "vehicle_reference_to_contact_m": args.vehicle_reference_to_contact_m,
        "ground_truth_contact_clearance_min_m": ground_truth_contact_clearance_min_m,
        "physical_contact_passed": physical_contact_passed,
        "maximum_target_z_step_m": maximum_step(target_final),
        "approach_reference_rate_mps": approach_rate_mps,
        "contact_reference_rate_mps": contact_rate_mps,
        "rate_profile_passed": rate_profile_passed,
        "horizontal_error_rmse_m": horizontal_rmse_m,
        "horizontal_error_max_m": horizontal_max_m,
        "horizontal_tracking_passed": horizontal_tracking_passed,
        "deck_horizontal_displacement_m": deck_horizontal_displacement_m,
        "moving_deck_required": args.require_moving_deck,
        "moving_deck_passed": moving_deck_passed,
        "maximum_candidate_duration_s": max(
            (sample.values[0] for sample in touchdown_duration), default=0.0
        ),
        "reference_decreases_after_candidate": reference_decreases_after_candidate,
        "hold_duration_s": hold_duration_s,
        "target_z_span_after_hold_m": target_span_after_hold,
        "max_abs_z_velocity_after_hold_mps": max_abs_z_velocity_after_hold,
        "relative_deck_hold_observed": relative_deck_hold_observed,
        "hold_relative_height_span_m": (
            hold_relative_height_span_m
            if math.isfinite(hold_relative_height_span_m)
            else None
        ),
        "hold_contact_clearance_min_m": (
            hold_contact_clearance_min_m
            if math.isfinite(hold_contact_clearance_min_m)
            else None
        ),
        "hold_contact_clearance_max_m": (
            hold_contact_clearance_max_m
            if math.isfinite(hold_contact_clearance_max_m)
            else None
        ),
        "hold_vertical_semantics_passed": hold_vertical_semantics_passed,
        "first_px4_land_flag_times_s": first_land_flag_times,
        "nav_land_commands": nav_land_commands,
        "disarm_commands": disarm_commands,
        "marker_switch_sequence": marker_sequence,
        "marker_switch_count": max(0, len(marker_sequence) - 1),
        "vision_loss_count": vision_loss_count,
        "vision_recovery_count": vision_recovery_count,
        "recovery_count": recovery_count,
        "positive_touchdown_passed": (
            candidate_start_s is not None
            and confirmed_start_s is not None
            and hold_start_s is not None
            and reference_decreases_after_candidate == 0
            and rate_profile_passed
            and horizontal_tracking_passed
            and moving_deck_passed
            and physical_contact_passed
            and hold_duration_s >= args.minimum_hold_duration_s
            and hold_vertical_semantics_passed
            and nav_land_commands == 0
            and disarm_commands == 0
        ),
    }
    return result


def print_human_readable(result: dict[str, Any]) -> None:
    print(f"Bag: {result['bag']}")
    print("State sequence: " + " -> ".join(result["state_sequence"]))
    print(f"Final phase counts: {result['final_phase_counts']}")
    print(f"Touchdown status counts: {result['touchdown_status_counts']}")
    print(
        "Final / candidate / confirmed / hold times: "
        f"{result['final_descent_start_s']} / {result['candidate_start_s']} / "
        f"{result['confirmed_start_s']} / {result['touchdown_hold_start_s']}"
    )
    print(f"Candidate-to-confirm delay: {result['candidate_to_confirm_delay_s']}")
    print(
        "Landing time / touchdown vertical speed: "
        f"{result['landing_time_s']} s / {result['touchdown_vertical_speed_mps']} m/s"
    )
    print(
        "Reference / estimated / Ground Truth minimum height: "
        f"{result['reference_min_m']:.4f} / {result['estimated_height_min_m']:.4f} / "
        f"{result['ground_truth_height_min_m']:.4f} m"
    )
    print(
        "Ground Truth landing-gear clearance: "
        f"{result['ground_truth_contact_clearance_min_m']:.4f} m; "
        f"contact={'PASS' if result['physical_contact_passed'] else 'FAIL'}"
    )
    print(f"Maximum target-z step: {result['maximum_target_z_step_m']:.5f} m")
    print(
        "Approach / contact reference rate: "
        f"{result['approach_reference_rate_mps']} / "
        f"{result['contact_reference_rate_mps']} m/s; "
        f"profile={'PASS' if result['rate_profile_passed'] else 'FAIL'}"
    )
    print(
        "Horizontal RMSE / max / deck displacement: "
        f"{result['horizontal_error_rmse_m']} / {result['horizontal_error_max_m']} / "
        f"{result['deck_horizontal_displacement_m']} m; "
        f"tracking={'PASS' if result['horizontal_tracking_passed'] else 'FAIL'}, "
        f"moving={'PASS' if result['moving_deck_passed'] else 'FAIL'}"
    )
    print(
        "Maximum candidate duration / reference decreases after candidate: "
        f"{result['maximum_candidate_duration_s']:.3f} s / "
        f"{result['reference_decreases_after_candidate']}"
    )
    print(
        "Hold duration / target-z span / max |z velocity|: "
        f"{result['hold_duration_s']:.3f} s / "
        f"{result['target_z_span_after_hold_m']} m / "
        f"{result['max_abs_z_velocity_after_hold_mps']} m/s"
    )
    print(
        "Hold semantics / relative-height span / contact clearance min/max: "
        f"{'RELATIVE_DECK_HOLD' if result['relative_deck_hold_observed'] else 'FROZEN_WORLD_Z'} / "
        f"{result['hold_relative_height_span_m']} m / "
        f"{result['hold_contact_clearance_min_m']} / "
        f"{result['hold_contact_clearance_max_m']} m; "
        f"semantics={'PASS' if result['hold_vertical_semantics_passed'] else 'FAIL'}"
    )
    print(f"First PX4 land flags: {result['first_px4_land_flag_times_s']}")
    print(
        "Marker sequence / switches: "
        f"{result['marker_switch_sequence']} / {result['marker_switch_count']}"
    )
    print(
        "Vision losses / recoveries / recovery states: "
        f"{result['vision_loss_count']} / {result['vision_recovery_count']} / "
        f"{result['recovery_count']}"
    )
    print(
        "NAV_LAND / Disarm commands: "
        f"{result['nav_land_commands']} / {result['disarm_commands']}"
    )
    print(
        "P6B positive touchdown test: "
        + ("PASS" if result["positive_touchdown_passed"] else "FAIL")
    )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate P6B final descent and positive touchdown confirmation."
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
    parser.add_argument("--json", action="store_true", help="print JSON output")
    parser.add_argument("--reference-tolerance-m", type=float, default=1.0e-4)
    parser.add_argument("--minimum-hold-duration-s", type=float, default=10.0)
    parser.add_argument("--maximum-hold-target-span-m", type=float, default=0.05)
    parser.add_argument("--maximum-hold-z-velocity-mps", type=float, default=1.0e-3)
    parser.add_argument(
        "--maximum-hold-relative-height-span-m", type=float, default=0.08
    )
    parser.add_argument("--hold-settling-time-s", type=float, default=0.20)
    parser.add_argument("--vehicle-reference-to-contact-m", type=float, default=0.227)
    parser.add_argument("--maximum-contact-clearance-m", type=float, default=0.03)
    parser.add_argument("--maximum-contact-penetration-m", type=float, default=0.05)
    parser.add_argument("--slowdown-height-m", type=float, default=0.25)
    parser.add_argument("--maximum-approach-rate-mps", type=float, default=0.20)
    parser.add_argument("--maximum-contact-rate-mps", type=float, default=0.05)
    parser.add_argument("--maximum-horizontal-error-m", type=float, default=0.30)
    parser.add_argument("--require-moving-deck", action="store_true")
    parser.add_argument("--minimum-deck-displacement-m", type=float, default=0.25)
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
    return 0 if result["positive_touchdown_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
