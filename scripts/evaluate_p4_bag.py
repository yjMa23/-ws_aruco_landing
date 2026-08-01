#!/usr/bin/env python3
"""离线评测 P4 移动甲板水平跟踪 rosbag。

脚本只读取 rosbag，不向控制链发布数据。Gazebo world ENU Ground Truth 会先通过
WGS84 世界原点转换到 PX4 local NED，再用于计算跟踪误差。
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence


TRACK_STATE = "TRACK_TARGET"
RECOVERY_STATE = "RECOVER_TO_GNSS"

STATE_TOPIC = "/landing/state"
VISIBLE_TOPIC = "/aruco/visible"
GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"
PREDICTED_TOPIC = "/landing/predicted_deck_pose"
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
VEHICLE_ODOMETRY_TOPIC = "/fmu/out/vehicle_odometry"
EFFECTIVE_GAIN_TOPIC = "/landing/effective_relative_velocity_gain"
ESTIMATED_DECK_ACCELERATION_TOPIC = "/landing/estimated_deck_acceleration"
MPC_STATUS_TOPIC = "/landing/relative_mpc/status"
MPC_SOLVE_TIME_TOPIC = "/landing/relative_mpc/solve_time_ms"
MPC_ITERATION_TOPIC = "/landing/relative_mpc/iteration_count"
MPC_OBJECTIVE_TOPIC = "/landing/relative_mpc/objective"
MPC_FALLBACK_COUNT_TOPIC = "/landing/relative_mpc/fallback_count"
MPC_FIRST_CONTROL_TOPIC = "/landing/relative_mpc/first_control"
MPC_ACTIVE_CONSTRAINTS_TOPIC = "/landing/relative_mpc/active_constraints"

REQUIRED_TOPICS = {
    STATE_TOPIC,
    VISIBLE_TOPIC,
    GROUND_TRUTH_TOPIC,
    PREDICTED_TOPIC,
    LOCAL_POSITION_TOPIC,
    VEHICLE_ODOMETRY_TOPIC,
}
OPTIONAL_TOPICS = {
    EFFECTIVE_GAIN_TOPIC,
    ESTIMATED_DECK_ACCELERATION_TOPIC,
    MPC_STATUS_TOPIC,
    MPC_SOLVE_TIME_TOPIC,
    MPC_ITERATION_TOPIC,
    MPC_OBJECTIVE_TOPIC,
    MPC_FALLBACK_COUNT_TOPIC,
    MPC_FIRST_CONTROL_TOPIC,
    MPC_ACTIVE_CONSTRAINTS_TOPIC,
}

WGS84_SEMI_MAJOR_AXIS_M = 6378137.0
WGS84_INVERSE_FLATTENING = 298.257223563
WGS84_FLATTENING = 1.0 / WGS84_INVERSE_FLATTENING
WGS84_SEMI_MINOR_AXIS_M = WGS84_SEMI_MAJOR_AXIS_M * (1.0 - WGS84_FLATTENING)
WGS84_FIRST_ECCENTRICITY_SQUARED = WGS84_FLATTENING * (
    2.0 - WGS84_FLATTENING
)
WGS84_SECOND_ECCENTRICITY_SQUARED = (
    WGS84_SEMI_MAJOR_AXIS_M**2 - WGS84_SEMI_MINOR_AXIS_M**2
) / WGS84_SEMI_MINOR_AXIS_M**2


@dataclass(frozen=True)
class TimedVector:
    time_s: float
    values: tuple[float, ...]


@dataclass(frozen=True)
class GeodeticPosition:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


def finite_values(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def mpc_status_is_intentional_disengagement(status: str) -> bool:
    return status.upper() == "TERMINAL_PHASE_P47"


def mpc_status_is_solver_success(status: str) -> bool:
    return status.lower().startswith("solved")


def wgs84_to_ecef(position: GeodeticPosition) -> tuple[float, float, float]:
    latitude = math.radians(position.latitude_deg)
    longitude = math.radians(position.longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    radius = WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude**2
    )
    return (
        (radius + position.altitude_m) * cos_latitude * cos_longitude,
        (radius + position.altitude_m) * cos_latitude * sin_longitude,
        (
            radius * (1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED)
            + position.altitude_m
        )
        * sin_latitude,
    )


def ecef_to_wgs84(ecef: Sequence[float]) -> GeodeticPosition:
    x, y, z = ecef
    horizontal_radius = math.hypot(x, y)
    if horizontal_radius <= 1.0e-9:
        return GeodeticPosition(
            90.0 if z >= 0.0 else -90.0,
            0.0,
            abs(z) - WGS84_SEMI_MINOR_AXIS_M,
        )

    longitude = math.atan2(y, x)
    theta = math.atan2(
        z * WGS84_SEMI_MAJOR_AXIS_M,
        horizontal_radius * WGS84_SEMI_MINOR_AXIS_M,
    )
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    latitude = math.atan2(
        z
        + WGS84_SECOND_ECCENTRICITY_SQUARED
        * WGS84_SEMI_MINOR_AXIS_M
        * sin_theta**3,
        horizontal_radius
        - WGS84_FIRST_ECCENTRICITY_SQUARED
        * WGS84_SEMI_MAJOR_AXIS_M
        * cos_theta**3,
    )
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    radius = WGS84_SEMI_MAJOR_AXIS_M / math.sqrt(
        1.0 - WGS84_FIRST_ECCENTRICITY_SQUARED * sin_latitude**2
    )
    altitude = horizontal_radius / cos_latitude - radius
    return GeodeticPosition(math.degrees(latitude), math.degrees(longitude), altitude)


def ecef_to_enu_matrix(origin: GeodeticPosition) -> tuple[tuple[float, ...], ...]:
    latitude = math.radians(origin.latitude_deg)
    longitude = math.radians(origin.longitude_deg)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)
    return (
        (-sin_longitude, cos_longitude, 0.0),
        (
            -sin_latitude * cos_longitude,
            -sin_latitude * sin_longitude,
            cos_latitude,
        ),
        (
            cos_latitude * cos_longitude,
            cos_latitude * sin_longitude,
            sin_latitude,
        ),
    )


def matrix_vector_multiply(
    matrix: Sequence[Sequence[float]], vector: Sequence[float]
) -> tuple[float, float, float]:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def transpose(matrix: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))


def local_enu_to_wgs84(
    position_enu: Sequence[float], origin: GeodeticPosition
) -> GeodeticPosition:
    origin_ecef = wgs84_to_ecef(origin)
    enu_to_ecef = transpose(ecef_to_enu_matrix(origin))
    offset_ecef = matrix_vector_multiply(enu_to_ecef, position_enu)
    return ecef_to_wgs84(
        tuple(origin_ecef[index] + offset_ecef[index] for index in range(3))
    )


def wgs84_to_local_enu(
    position: GeodeticPosition, origin: GeodeticPosition
) -> tuple[float, float, float]:
    position_ecef = wgs84_to_ecef(position)
    origin_ecef = wgs84_to_ecef(origin)
    offset_ecef = tuple(
        position_ecef[index] - origin_ecef[index] for index in range(3)
    )
    return matrix_vector_multiply(ecef_to_enu_matrix(origin), offset_ecef)


def world_enu_to_local_ned(
    position_world_enu: Sequence[float],
    world_origin: GeodeticPosition,
    local_origin: GeodeticPosition,
) -> tuple[float, float, float]:
    position_wgs84 = local_enu_to_wgs84(position_world_enu, world_origin)
    local_enu = wgs84_to_local_enu(position_wgs84, local_origin)
    return (local_enu[1], local_enu[0], -local_enu[2])


def interpolate(samples: Sequence[TimedVector], query_time_s: float) -> Optional[tuple[float, ...]]:
    if not samples or query_time_s < samples[0].time_s or query_time_s > samples[-1].time_s:
        return None
    times = [sample.time_s for sample in samples]
    upper_index = bisect.bisect_left(times, query_time_s)
    if upper_index == 0:
        return samples[0].values
    if upper_index == len(samples):
        return samples[-1].values
    upper = samples[upper_index]
    lower = samples[upper_index - 1]
    if query_time_s == upper.time_s:
        return upper.values
    duration = upper.time_s - lower.time_s
    if duration <= 0.0:
        return None
    alpha = (query_time_s - lower.time_s) / duration
    return tuple(
        lower.values[index] + alpha * (upper.values[index] - lower.values[index])
        for index in range(len(lower.values))
    )


def rmse(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("cannot compute RMSE from an empty sequence")
    return math.sqrt(sum(value * value for value in values) / len(values))


def percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        raise ValueError("cannot compute percentile from an empty sequence")
    if not 0.0 <= probability <= 1.0:
        raise ValueError("percentile probability must be within [0, 1]")
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    if lower_index == upper_index:
        return ordered[lower_index]
    alpha = position - lower_index
    return ordered[lower_index] + alpha * (
        ordered[upper_index] - ordered[lower_index]
    )


def quaternion_to_roll_pitch(q: Sequence[float]) -> tuple[float, float]:
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm <= 1.0e-9:
        raise ValueError("invalid quaternion")
    w, x, y, z = (value / norm for value in q)
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch


def resolve_bag_uri(path: Path) -> Path:
    if path.is_dir():
        return path
    if path.is_file() and path.suffix == ".db3":
        return path.parent
    raise FileNotFoundError(f"rosbag path does not exist or is unsupported: {path}")


def load_ros_modules() -> tuple[Any, Any, Any, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from rosbag2_py import ConverterOptions, StorageOptions
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and the px4_msgs workspace before running this script."
        ) from error
    return rosbag2_py, deserialize_message, get_message, (StorageOptions, ConverterOptions)


def evaluate(args: argparse.Namespace) -> dict[str, float | int | str]:
    rosbag2_py, deserialize_message, get_message, option_types = load_ros_modules()
    storage_options_type, converter_options_type = option_types
    bag_uri = resolve_bag_uri(args.bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        storage_options_type(uri=str(bag_uri), storage_id="sqlite3"),
        converter_options_type(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    missing_topics = REQUIRED_TOPICS - topic_types.keys()
    if missing_topics:
        raise RuntimeError(
            "bag is missing required topics: " + ", ".join(sorted(missing_topics))
        )

    selected_topics = REQUIRED_TOPICS | (OPTIONAL_TOPICS & topic_types.keys())
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in selected_topics
        if topic in topic_types
    }

    state_samples: list[tuple[float, str]] = []
    visibility_samples: list[tuple[float, bool]] = []
    ground_truth_world: list[TimedVector] = []
    predicted_positions: list[TimedVector] = []
    local_positions_raw: list[tuple[float, Any]] = []
    vehicle_attitudes: list[TimedVector] = []
    effective_gain_samples: list[TimedVector] = []
    estimated_deck_acceleration_samples: list[TimedVector] = []
    mpc_status_samples: list[tuple[float, str]] = []
    mpc_solve_time_samples: list[TimedVector] = []
    mpc_iteration_samples: list[TimedVector] = []
    mpc_objective_samples: list[TimedVector] = []
    mpc_fallback_count_samples: list[TimedVector] = []
    mpc_first_control_samples: list[TimedVector] = []
    mpc_active_constraint_samples: list[TimedVector] = []
    final_bag_time_s = 0.0

    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        time_s = timestamp_ns * 1.0e-9
        final_bag_time_s = max(final_bag_time_s, time_s)
        if topic not in message_types:
            continue
        message = deserialize_message(serialized_data, message_types[topic])

        if topic == STATE_TOPIC:
            state_samples.append((time_s, str(message.data)))
        elif topic == VISIBLE_TOPIC:
            visibility_samples.append((time_s, bool(message.data)))
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
        elif topic == PREDICTED_TOPIC:
            position = message.pose.position
            values = (float(position.x), float(position.y), float(position.z))
            if finite_values(values):
                predicted_positions.append(TimedVector(time_s, values))
        elif topic == LOCAL_POSITION_TOPIC:
            local_positions_raw.append((time_s, message))
        elif topic == VEHICLE_ODOMETRY_TOPIC:
            quaternion = tuple(float(value) for value in message.q)
            if finite_values(quaternion):
                vehicle_attitudes.append(TimedVector(time_s, quaternion))
        elif topic == EFFECTIVE_GAIN_TOPIC:
            gain = float(message.data)
            if math.isfinite(gain):
                effective_gain_samples.append(TimedVector(time_s, (gain,)))
        elif topic == ESTIMATED_DECK_ACCELERATION_TOPIC:
            acceleration = message.twist.linear
            values = (float(acceleration.x), float(acceleration.y))
            if finite_values(values):
                estimated_deck_acceleration_samples.append(TimedVector(time_s, values))
        elif topic == MPC_STATUS_TOPIC:
            mpc_status_samples.append((time_s, str(message.data)))
        elif topic == MPC_SOLVE_TIME_TOPIC:
            value = float(message.data)
            if math.isfinite(value) and value >= 0.0:
                mpc_solve_time_samples.append(TimedVector(time_s, (value,)))
        elif topic == MPC_ITERATION_TOPIC:
            value = float(message.data)
            if math.isfinite(value) and value >= 0.0:
                mpc_iteration_samples.append(TimedVector(time_s, (value,)))
        elif topic == MPC_OBJECTIVE_TOPIC:
            value = float(message.data)
            if math.isfinite(value):
                mpc_objective_samples.append(TimedVector(time_s, (value,)))
        elif topic == MPC_FALLBACK_COUNT_TOPIC:
            value = float(message.data)
            if math.isfinite(value) and value >= 0.0:
                mpc_fallback_count_samples.append(TimedVector(time_s, (value,)))
        elif topic == MPC_FIRST_CONTROL_TOPIC:
            values = (float(message.vector.x), float(message.vector.y))
            if finite_values(values):
                mpc_first_control_samples.append(TimedVector(time_s, values))
        elif topic == MPC_ACTIVE_CONSTRAINTS_TOPIC:
            value = float(message.data)
            if math.isfinite(value) and value >= 0.0:
                mpc_active_constraint_samples.append(TimedVector(time_s, (value,)))

    track_times = [time_s for time_s, state in state_samples if state == TRACK_STATE]
    if not track_times:
        raise RuntimeError("bag never entered TRACK_TARGET")
    stable_start_s = track_times[0] + args.discard_seconds
    if final_bag_time_s <= stable_start_s:
        raise RuntimeError("no data remains after the configured transition discard time")

    valid_reference_message = next(
        (
            message
            for time_s, message in local_positions_raw
            if time_s >= stable_start_s
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

    horizontal_errors: list[float] = []
    relative_speed_errors: list[float] = []
    horizontal_speeds: list[float] = []
    horizontal_accelerations: list[float] = []
    for time_s, message in local_positions_raw:
        if time_s < stable_start_s or not bool(message.xy_valid):
            continue
        values = (
            float(message.x),
            float(message.y),
            float(message.vx),
            float(message.vy),
            float(message.ax),
            float(message.ay),
        )
        if not finite_values(values[:2]):
            continue
        deck_state = interpolate(ground_truth_ned, time_s)
        if deck_state is None:
            continue
        horizontal_errors.append(
            math.hypot(values[0] - deck_state[0], values[1] - deck_state[1])
        )
        if bool(message.v_xy_valid) and finite_values(values[2:4]):
            relative_speed_errors.append(
                math.hypot(values[2] - deck_state[3], values[3] - deck_state[4])
            )
            horizontal_speeds.append(math.hypot(values[2], values[3]))
        if finite_values(values[4:6]):
            horizontal_accelerations.append(math.hypot(values[4], values[5]))

    prediction_errors: list[float] = []
    for sample in predicted_positions:
        if sample.time_s < stable_start_s:
            continue
        deck_state = interpolate(ground_truth_ned, sample.time_s)
        if deck_state is None:
            continue
        prediction_errors.append(
            math.hypot(sample.values[0] - deck_state[0], sample.values[1] - deck_state[1])
        )

    if not horizontal_errors:
        raise RuntimeError("no aligned vehicle and Ground Truth position samples were found")
    if not relative_speed_errors:
        raise RuntimeError("no aligned relative velocity samples were found")
    if not prediction_errors:
        raise RuntimeError("no aligned prediction samples were found")

    marker_loss_count = 0
    previous_visible: Optional[bool] = None
    for time_s, visible in visibility_samples:
        if time_s < stable_start_s:
            continue
        if previous_visible is True and not visible:
            marker_loss_count += 1
        previous_visible = visible

    recovery_count = 0
    previous_state: Optional[str] = None
    for time_s, state in state_samples:
        if time_s < stable_start_s:
            continue
        if state == RECOVERY_STATE and previous_state != RECOVERY_STATE:
            recovery_count += 1
        previous_state = state

    max_roll_deg = 0.0
    max_pitch_deg = 0.0
    for sample in vehicle_attitudes:
        if sample.time_s < stable_start_s:
            continue
        try:
            roll, pitch = quaternion_to_roll_pitch(sample.values)
        except ValueError:
            continue
        max_roll_deg = max(max_roll_deg, abs(math.degrees(roll)))
        max_pitch_deg = max(max_pitch_deg, abs(math.degrees(pitch)))

    result: dict[str, float | int | str] = {
        "bag": str(bag_uri),
        "stable_start_time_s": stable_start_s,
        "stable_duration_s": final_bag_time_s - stable_start_s,
        "horizontal_position_rmse_m": rmse(horizontal_errors),
        "relative_velocity_rmse_mps": rmse(relative_speed_errors),
        "maximum_horizontal_error_m": max(horizontal_errors),
        "predicted_position_rmse_m": rmse(prediction_errors),
        "marker_loss_count": marker_loss_count,
        "gnss_recovery_count": recovery_count,
        "maximum_horizontal_speed_mps": max(horizontal_speeds, default=0.0),
        "maximum_horizontal_acceleration_mps2": max(
            horizontal_accelerations, default=0.0
        ),
        "maximum_absolute_roll_deg": max_roll_deg,
        "maximum_absolute_pitch_deg": max_pitch_deg,
        "position_sample_count": len(horizontal_errors),
        "relative_velocity_sample_count": len(relative_speed_errors),
        "prediction_sample_count": len(prediction_errors),
    }

    stable_gains = [
        sample.values[0]
        for sample in effective_gain_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_gains:
        result.update(
            {
                "effective_relative_velocity_gain_min": min(stable_gains),
                "effective_relative_velocity_gain_max": max(stable_gains),
                "effective_relative_velocity_gain_mean": sum(stable_gains)
                / len(stable_gains),
                "effective_relative_velocity_gain_sample_count": len(stable_gains),
            }
        )

    stable_acceleration_norms = [
        math.hypot(sample.values[0], sample.values[1])
        for sample in estimated_deck_acceleration_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_acceleration_norms:
        result.update(
            {
                "estimated_deck_acceleration_max_mps2": max(
                    stable_acceleration_norms
                ),
                "estimated_deck_acceleration_rmse_mps2": rmse(
                    stable_acceleration_norms
                ),
                "estimated_deck_acceleration_sample_count": len(
                    stable_acceleration_norms
                ),
            }
        )

    stable_solve_times = [
        sample.values[0]
        for sample in mpc_solve_time_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_solve_times:
        result.update(
            {
                "mpc_solve_time_mean_ms": sum(stable_solve_times)
                / len(stable_solve_times),
                "mpc_solve_time_p95_ms": percentile(stable_solve_times, 0.95),
                "mpc_solve_time_max_ms": max(stable_solve_times),
                "mpc_solve_time_sample_count": len(stable_solve_times),
                "mpc_control_period_ms": args.control_period_ms,
                "mpc_deadline_miss_count": sum(
                    value >= args.control_period_ms for value in stable_solve_times
                ),
            }
        )

    stable_iterations = [
        sample.values[0]
        for sample in mpc_iteration_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_iterations:
        result.update(
            {
                "mpc_iteration_mean": sum(stable_iterations)
                / len(stable_iterations),
                "mpc_iteration_p95": percentile(stable_iterations, 0.95),
                "mpc_iteration_max": max(stable_iterations),
            }
        )

    stable_objectives = [
        sample.values[0]
        for sample in mpc_objective_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_objectives:
        result["mpc_objective_mean"] = sum(stable_objectives) / len(
            stable_objectives
        )

    stable_fallback_counts = [
        sample.values[0]
        for sample in mpc_fallback_count_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_fallback_counts:
        result["mpc_fallback_count"] = int(max(stable_fallback_counts))

    stable_statuses = [
        status for time_s, status in mpc_status_samples if time_s >= stable_start_s
    ]
    if stable_statuses:
        result["mpc_terminal_phase_p47_count"] = sum(
            mpc_status_is_intentional_disengagement(status)
            for status in stable_statuses
        )
        result["mpc_non_solved_status_count"] = sum(
            not mpc_status_is_solver_success(status)
            and not mpc_status_is_intentional_disengagement(status)
            for status in stable_statuses
        )

    stable_active_constraints = [
        sample.values[0]
        for sample in mpc_active_constraint_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_active_constraints:
        result.update(
            {
                "mpc_active_constraints_mean": sum(stable_active_constraints)
                / len(stable_active_constraints),
                "mpc_active_constraints_max": max(stable_active_constraints),
            }
        )

    stable_controls = [
        sample
        for sample in mpc_first_control_samples
        if sample.time_s >= stable_start_s
    ]
    if stable_controls:
        control_norms = [math.hypot(*sample.values) for sample in stable_controls]
        control_increments = [
            math.hypot(
                current.values[0] - previous.values[0],
                current.values[1] - previous.values[1],
            )
            for previous, current in zip(stable_controls, stable_controls[1:])
        ]
        result.update(
            {
                "mpc_first_control_max_mps2": max(control_norms),
                "mpc_first_control_rmse_mps2": rmse(control_norms),
                "mpc_control_increment_rmse_mps2": (
                    rmse(control_increments) if control_increments else 0.0
                ),
                "mpc_control_sample_count": len(stable_controls),
            }
        )

    return result


def print_human_readable(result: dict[str, float | int | str]) -> None:
    print(f"Bag: {result['bag']}")
    print(f"Stable duration: {result['stable_duration_s']:.3f} s")
    print(
        "Horizontal position RMSE: "
        f"{result['horizontal_position_rmse_m']:.4f} m"
    )
    print(
        "Relative velocity RMSE: "
        f"{result['relative_velocity_rmse_mps']:.4f} m/s"
    )
    print(
        "Maximum horizontal error: "
        f"{result['maximum_horizontal_error_m']:.4f} m"
    )
    print(
        "Predicted position RMSE: "
        f"{result['predicted_position_rmse_m']:.4f} m"
    )
    print(
        "Marker losses / GNSS recoveries: "
        f"{result['marker_loss_count']} / {result['gnss_recovery_count']}"
    )
    print(
        "Maximum horizontal speed / acceleration: "
        f"{result['maximum_horizontal_speed_mps']:.4f} m/s / "
        f"{result['maximum_horizontal_acceleration_mps2']:.4f} m/s^2"
    )
    print(
        "Maximum absolute roll / pitch: "
        f"{result['maximum_absolute_roll_deg']:.3f} deg / "
        f"{result['maximum_absolute_pitch_deg']:.3f} deg"
    )
    if "effective_relative_velocity_gain_min" in result:
        print(
            "Effective relative velocity gain min / mean / max: "
            f"{result['effective_relative_velocity_gain_min']:.4f} / "
            f"{result['effective_relative_velocity_gain_mean']:.4f} / "
            f"{result['effective_relative_velocity_gain_max']:.4f}"
        )
    if "estimated_deck_acceleration_max_mps2" in result:
        print(
            "Estimated deck acceleration RMSE / max: "
            f"{result['estimated_deck_acceleration_rmse_mps2']:.4f} m/s^2 / "
            f"{result['estimated_deck_acceleration_max_mps2']:.4f} m/s^2"
        )
    if "mpc_solve_time_mean_ms" in result:
        print(
            "MPC solve time mean / P95 / max: "
            f"{result['mpc_solve_time_mean_ms']:.4f} / "
            f"{result['mpc_solve_time_p95_ms']:.4f} / "
            f"{result['mpc_solve_time_max_ms']:.4f} ms"
        )
        print(
            "MPC iterations mean / P95 / max: "
            f"{result.get('mpc_iteration_mean', 0.0):.2f} / "
            f"{result.get('mpc_iteration_p95', 0.0):.2f} / "
            f"{result.get('mpc_iteration_max', 0.0):.0f}"
        )
        print(
            "MPC fallback / deadline misses: "
            f"{result.get('mpc_fallback_count', 0)} / "
            f"{result.get('mpc_deadline_miss_count', 0)}"
        )
        print(
            "MPC control RMSE / increment RMSE / max: "
            f"{result.get('mpc_first_control_rmse_mps2', 0.0):.4f} / "
            f"{result.get('mpc_control_increment_rmse_mps2', 0.0):.4f} / "
            f"{result.get('mpc_first_control_max_mps2', 0.0):.4f} m/s^2"
        )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Evaluate P4 moving-deck tracking metrics from a ROS 2 bag."
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
    parser.add_argument(
        "--discard-seconds",
        type=float,
        default=5.0,
        help="seconds discarded after first entering TRACK_TARGET (default: 5.0)",
    )
    parser.add_argument(
        "--world-origin-latitude",
        type=float,
        default=47.397971057728974,
    )
    parser.add_argument(
        "--world-origin-longitude",
        type=float,
        default=8.546163739800146,
    )
    parser.add_argument("--world-origin-altitude", type=float, default=0.0)
    parser.add_argument(
        "--control-period-ms",
        type=float,
        default=50.0,
        help="MPC deadline used for solve-time checks in milliseconds (default: 50)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="print machine-readable JSON instead of the text summary",
    )
    args = parser.parse_args()
    if not math.isfinite(args.discard_seconds) or args.discard_seconds < 0.0:
        parser.error("--discard-seconds must be finite and non-negative")
    if not math.isfinite(args.control_period_ms) or args.control_period_ms <= 0.0:
        parser.error("--control-period-ms must be finite and positive")
    if not finite_values(
        (
            args.world_origin_latitude,
            args.world_origin_longitude,
            args.world_origin_altitude,
        )
    ):
        parser.error("world origin values must be finite")
    if not -90.0 <= args.world_origin_latitude <= 90.0:
        parser.error("world origin latitude must be within [-90, 90]")
    if not -180.0 <= args.world_origin_longitude <= 180.0:
        parser.error("world origin longitude must be within [-180, 180]")
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
