#!/usr/bin/env python3
"""离线评测 ArUco 甲板 6-DoF shadow；Ground Truth 仅在本脚本中读取。"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence

from evaluate_horizontal_tracking import GeodeticPosition, world_enu_to_local_ned


GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"
UAV_GROUND_TRUTH_TOPIC = "/simulation/uav/ground_truth_pose"
STATE_TOPIC = "/landing/deck_motion_shadow/state"
TRAJECTORY_TOPIC = "/landing/deck_motion_shadow/trajectory"
STATUS_TOPIC = "/landing/deck_motion_shadow/status"
TRUSTED_HORIZON_TOPIC = "/landing/deck_motion_shadow/trusted_horizon_s"
LANDING_STATE_TOPIC = "/landing/state"
MARKER_ID_TOPIC = "/landing/active_marker_id"
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"
REQUIRED_TOPICS = {
    GROUND_TRUTH_TOPIC,
    UAV_GROUND_TRUTH_TOPIC,
    STATE_TOPIC,
    TRAJECTORY_TOPIC,
    STATUS_TOPIC,
    TRUSTED_HORIZON_TOPIC,
    LANDING_STATE_TOPIC,
    MARKER_ID_TOPIC,
    LOCAL_POSITION_TOPIC,
    VEHICLE_COMMAND_TOPIC,
}

TRACKING_STATES = {"TRACK_TARGET", "WAIT_LANDING_WINDOW"}
DESCENT_STATES = {
    "DESCEND",
    "TEST_HEIGHT_HOLD",
    "FINAL_DESCENT",
    "TOUCHDOWN_CANDIDATE_HOLD",
    "TOUCHDOWN_HOLD",
    "DONE",
}
NAV_LAND_COMMAND = 21
ARM_DISARM_COMMAND = 400
PREDICTION_HORIZON_S = 0.50
Q_NED_ENU = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
Q_BODY_DECK = (1.0, 0.0, 0.0, 0.0)
WORLD_ORIGIN = GeodeticPosition(47.397971057728974, 8.546163739800146, 0.0)
DEFAULT_UAV_MODEL_NAME = "x500_mono_cam_down_0"


@dataclass(frozen=True)
class RigidState:
    time_s: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


@dataclass(frozen=True)
class ShadowState:
    bag_time_s: float
    state: RigidState


@dataclass(frozen=True)
class PredictionSample:
    origin_time_s: float
    target_time_s: float
    state: RigidState
    linear_acceleration: tuple[float, float, float]
    angular_acceleration: tuple[float, float, float]


def finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def vector_add(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    return tuple(x + y for x, y in zip(a, b))


def vector_scale(vector: Sequence[float], scale: float) -> tuple[float, ...]:
    return tuple(scale * value for value in vector)


def vector_difference(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    return tuple(x - y for x, y in zip(a, b))


def vector_norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def normalize_quaternion(q: Sequence[float]) -> tuple[float, float, float, float]:
    if len(q) != 4 or not finite(q):
        raise ValueError("quaternion must contain four finite values")
    norm = vector_norm(q)
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is too small")
    return tuple(value / norm for value in q)  # type: ignore[return-value]


def normalize_quaternion_or_none(
    q: Sequence[float],
) -> Optional[tuple[float, float, float, float]]:
    try:
        return normalize_quaternion(q)
    except ValueError:
        return None


def quaternion_multiply(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quaternion_rotate(q: Sequence[float], vector: Sequence[float]) -> tuple[float, ...]:
    w, x, y, z = normalize_quaternion(q)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def quaternion_slerp(
    a: Sequence[float], b: Sequence[float], ratio: float
) -> tuple[float, float, float, float]:
    qa = normalize_quaternion(a)
    qb = normalize_quaternion(b)
    dot = sum(x * y for x, y in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-value for value in qb)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return normalize_quaternion(
            tuple((1.0 - ratio) * x + ratio * y for x, y in zip(qa, qb))
        )
    angle = math.acos(dot)
    scale = math.sin(angle)
    return normalize_quaternion(
        tuple(
            (math.sin((1.0 - ratio) * angle) * x + math.sin(ratio * angle) * y)
            / scale
            for x, y in zip(qa, qb)
        )
    )


def interpolate_truth(samples: Sequence[RigidState], time_s: float) -> Optional[RigidState]:
    """只在 Ground Truth 闭区间内插值；末尾没有未来真值时返回 None。"""

    if not samples or time_s < samples[0].time_s or time_s > samples[-1].time_s:
        return None
    times = [sample.time_s for sample in samples]
    upper = bisect.bisect_left(times, time_s)
    if upper < len(samples) and samples[upper].time_s == time_s:
        return samples[upper]
    if upper == 0 or upper == len(samples):
        return None
    lower = samples[upper - 1]
    higher = samples[upper]
    ratio = (time_s - lower.time_s) / (higher.time_s - lower.time_s)

    def lerp(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
        return tuple((1.0 - ratio) * x + ratio * y for x, y in zip(a, b))

    return RigidState(
        time_s,
        lerp(lower.position, higher.position),  # type: ignore[arg-type]
        quaternion_slerp(lower.orientation, higher.orientation, ratio),
        lerp(lower.linear_velocity, higher.linear_velocity),  # type: ignore[arg-type]
        lerp(lower.angular_velocity, higher.angular_velocity),  # type: ignore[arg-type]
    )


def relative_truth(deck: RigidState, uav: RigidState) -> RigidState:
    """构造当前相对位置和甲板自身 twist 的 shadow 真值。"""

    return RigidState(
        deck.time_s,
        vector_difference(deck.position, uav.position),  # type: ignore[arg-type]
        deck.orientation,
        deck.linear_velocity,
        deck.angular_velocity,
    )


def prediction_truth(deck_future: RigidState, uav_at_origin: RigidState) -> RigidState:
    """构造甲板未来状态相对轨迹发布时 UAV 固定原点的真值。"""

    return RigidState(
        deck_future.time_s,
        vector_difference(deck_future.position, uav_at_origin.position),  # type: ignore[arg-type]
        deck_future.orientation,
        deck_future.linear_velocity,
        deck_future.angular_velocity,
    )


def with_finite_difference_velocity(samples: Sequence[RigidState]) -> list[RigidState]:
    """用相邻 Gazebo 真值位姿计算线速度；重复时间戳保留最后一帧。"""

    unique = {sample.time_s: sample for sample in samples}
    ordered = [unique[time_s] for time_s in sorted(unique)]
    if len(ordered) < 2:
        return []
    result: list[RigidState] = []
    for index, sample in enumerate(ordered):
        lower = ordered[max(0, index - 1)]
        upper = ordered[min(len(ordered) - 1, index + 1)]
        dt_s = upper.time_s - lower.time_s
        if dt_s <= 0.0:
            continue
        velocity = vector_scale(
            vector_difference(upper.position, lower.position), 1.0 / dt_s
        )
        result.append(
            RigidState(
                sample.time_s,
                sample.position,
                sample.orientation,
                velocity,  # type: ignore[arg-type]
                sample.angular_velocity,
            )
        )
    return result


def percentile(values: Sequence[float], probability: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (position - lower) * (ordered[upper] - ordered[lower])


def summary(values: Sequence[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "rmse": None, "p95": None, "max": None}
    return {
        "count": len(values),
        "rmse": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def normal_and_yaw(q: Sequence[float]) -> tuple[tuple[float, ...], float]:
    normal = quaternion_rotate(q, (0.0, 0.0, 1.0))
    deck_x = quaternion_rotate(q, (1.0, 0.0, 0.0))
    return normal, math.atan2(deck_x[1], deck_x[0])


def normal_error_deg(a: Sequence[float], b: Sequence[float]) -> float:
    dot = sum(x * y for x, y in zip(a, b))
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def state_errors(estimate: RigidState, truth: RigidState) -> dict[str, float]:
    position_error = vector_difference(estimate.position, truth.position)
    velocity_error = vector_difference(estimate.linear_velocity, truth.linear_velocity)
    angular_error = vector_difference(estimate.angular_velocity, truth.angular_velocity)
    estimate_normal, estimate_yaw = normal_and_yaw(estimate.orientation)
    truth_normal, truth_yaw = normal_and_yaw(truth.orientation)
    return {
        "horizontal_position_m": vector_norm(position_error[:2]),
        "vertical_position_m": abs(position_error[2]),
        "normal_deg": normal_error_deg(estimate_normal, truth_normal),
        "yaw_deg": abs(math.degrees(math.remainder(estimate_yaw - truth_yaw, 2.0 * math.pi))),
        "horizontal_velocity_mps": vector_norm(velocity_error[:2]),
        "vertical_velocity_mps": abs(velocity_error[2]),
        "angular_velocity_degps": math.degrees(vector_norm(angular_error)),
    }


def stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def duration_seconds(duration: Any) -> float:
    return float(duration.sec) + 1.0e-9 * float(duration.nanosec)


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
        from rosbag2_py import ConverterOptions, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError("source ROS Humble and px4_msgs before running evaluator") from error
    return rosbag2_py, deserialize_message, get_message, StorageOptions, ConverterOptions


def latest_value(samples: Sequence[tuple[float, Any]], time_s: float) -> Any:
    if not samples:
        return None
    index = bisect.bisect_right([sample[0] for sample in samples], time_s) - 1
    return samples[index][1] if index >= 0 else None


def derivative_truth(
    samples: Sequence[RigidState], time_s: float
) -> Optional[tuple[tuple[float, ...], tuple[float, ...]]]:
    before = interpolate_truth(samples, time_s - 0.02)
    after = interpolate_truth(samples, time_s + 0.02)
    if before is None or after is None:
        return None
    return (
        vector_scale(vector_difference(after.linear_velocity, before.linear_velocity), 25.0),
        vector_scale(vector_difference(after.angular_velocity, before.angular_velocity), 25.0),
    )


def evaluate(
    bag_path: Path, uav_model_name: str = DEFAULT_UAV_MODEL_NAME
) -> dict[str, Any]:
    rosbag2_py, deserialize_message, get_message, StorageOptions, ConverterOptions = (
        load_ros_modules()
    )
    bag_uri = resolve_bag_uri(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_uri), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = REQUIRED_TOPICS - topic_types.keys()
    if missing:
        raise RuntimeError("bag is missing required topics: " + ", ".join(sorted(missing)))
    message_types = {topic: get_message(topic_types[topic]) for topic in REQUIRED_TOPICS}

    truth_world: list[RigidState] = []
    truth_by_receipt_time_world: list[RigidState] = []
    uav_truth_world: list[RigidState] = []
    shadow_states: list[ShadowState] = []
    predictions: list[PredictionSample] = []
    statuses: list[tuple[float, str]] = []
    trusted_horizons: list[tuple[float, float]] = []
    landing_states: list[tuple[float, str]] = []
    marker_ids: list[tuple[float, int]] = []
    local_positions: list[tuple[float, Any]] = []
    commands: list[tuple[int, float]] = []
    invalid_output_count = 0
    invalid_ground_truth_count = 0

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
            if q_enu_body is None:
                invalid_ground_truth_count += 1
                continue
            q_ned_body = normalize_quaternion(quaternion_multiply(Q_NED_ENU, q_enu_body))
            q_ned_deck = normalize_quaternion(
                quaternion_multiply(q_ned_body, Q_BODY_DECK)
            )
            values = (
                pose.position.x, pose.position.y, pose.position.z,
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            )
            if not finite(values):
                invalid_ground_truth_count += 1
                continue
            converted = RigidState(
                stamp_seconds(message.header.stamp),
                (pose.position.x, pose.position.y, pose.position.z),
                q_ned_deck,
                (twist.linear.y, twist.linear.x, -twist.linear.z),
                quaternion_rotate(
                    q_ned_body,
                    (twist.angular.x, twist.angular.y, twist.angular.z),
                ),
            )
            truth_world.append(converted)
            truth_by_receipt_time_world.append(
                RigidState(
                    bag_time_s,
                    converted.position,
                    converted.orientation,
                    converted.linear_velocity,
                    converted.angular_velocity,
                )
            )
        elif topic == UAV_GROUND_TRUTH_TOPIC:
            if message.child_frame_id != uav_model_name + "/base_link":
                continue
            pose = message.pose.pose
            q_enu_body = normalize_quaternion_or_none(
                (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            )
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
            if q_enu_body is None or not finite(values):
                invalid_ground_truth_count += 1
                continue
            uav_truth_world.append(
                RigidState(
                    stamp_seconds(message.header.stamp),
                    values,
                    normalize_quaternion(quaternion_multiply(Q_NED_ENU, q_enu_body)),
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                )
            )
        elif topic == STATE_TOPIC:
            if (
                message.header.frame_id != "uav_centered_ned"
                or message.child_frame_id != "deck_landing_up"
            ):
                invalid_output_count += 1
                continue
            pose = message.pose.pose
            twist = message.twist.twist
            values = (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z,
                twist.linear.x, twist.linear.y, twist.linear.z,
                twist.angular.x, twist.angular.y, twist.angular.z,
            )
            if not finite(values):
                invalid_output_count += 1
                continue
            orientation = normalize_quaternion_or_none(values[3:7])
            if orientation is None:
                invalid_output_count += 1
                continue
            shadow_states.append(
                ShadowState(
                    bag_time_s,
                    RigidState(
                        stamp_seconds(message.header.stamp),
                        tuple(values[:3]),  # type: ignore[arg-type]
                        orientation,
                        tuple(values[7:10]),  # type: ignore[arg-type]
                        tuple(values[10:13]),  # type: ignore[arg-type]
                    ),
                )
            )
        elif topic == TRAJECTORY_TOPIC:
            base_time_s = stamp_seconds(message.header.stamp)
            if message.header.frame_id != "uav_origin_ned" or message.joint_names != ["deck_landing_up"]:
                invalid_output_count += 1
                continue
            for point in message.points:
                if (
                    len(point.transforms) != 1
                    or len(point.velocities) != 1
                    or len(point.accelerations) != 1
                ):
                    invalid_output_count += 1
                    continue
                relative_time_s = duration_seconds(point.time_from_start)
                if abs(relative_time_s - PREDICTION_HORIZON_S) > 0.026:
                    continue
                transform = point.transforms[0]
                velocity = point.velocities[0]
                acceleration = point.accelerations[0]
                values = (
                    transform.translation.x, transform.translation.y, transform.translation.z,
                    transform.rotation.w, transform.rotation.x, transform.rotation.y,
                    transform.rotation.z,
                    velocity.linear.x, velocity.linear.y, velocity.linear.z,
                    velocity.angular.x, velocity.angular.y, velocity.angular.z,
                    acceleration.linear.x, acceleration.linear.y, acceleration.linear.z,
                    acceleration.angular.x, acceleration.angular.y, acceleration.angular.z,
                )
                if not finite(values):
                    invalid_output_count += 1
                    continue
                target_time_s = base_time_s + relative_time_s
                orientation = normalize_quaternion_or_none(values[3:7])
                if orientation is None:
                    invalid_output_count += 1
                    continue
                predictions.append(
                    PredictionSample(
                        base_time_s,
                        target_time_s,
                        RigidState(
                            target_time_s,
                            tuple(values[:3]),  # type: ignore[arg-type]
                            orientation,
                            tuple(values[7:10]),  # type: ignore[arg-type]
                            tuple(values[10:13]),  # type: ignore[arg-type]
                        ),
                        tuple(values[13:16]),  # type: ignore[arg-type]
                        tuple(values[16:19]),  # type: ignore[arg-type]
                    )
                )
                break
        elif topic == STATUS_TOPIC:
            statuses.append((bag_time_s, message.data))
        elif topic == TRUSTED_HORIZON_TOPIC:
            if math.isfinite(message.data) and 0.0 <= message.data <= 0.5 + 1.0e-9:
                trusted_horizons.append((bag_time_s, float(message.data)))
            else:
                invalid_output_count += 1
        elif topic == LANDING_STATE_TOPIC:
            landing_states.append((bag_time_s, message.data))
        elif topic == MARKER_ID_TOPIC:
            marker_ids.append((bag_time_s, int(message.data)))
        elif topic == LOCAL_POSITION_TOPIC:
            values = (float(message.x), float(message.y), float(message.z))
            if finite(values):
                local_positions.append((bag_time_s, message))
            else:
                invalid_output_count += 1
        elif topic == VEHICLE_COMMAND_TOPIC:
            commands.append((int(message.command), float(message.param1)))

    reference = next(
        (
            message for _, message in local_positions
            if bool(message.xy_global) and bool(message.z_global)
            and finite((message.ref_lat, message.ref_lon, message.ref_alt))
        ),
        None,
    )
    if reference is None:
        raise RuntimeError("no valid PX4 local geodetic reference is available")
    local_origin = GeodeticPosition(
        float(reference.ref_lat), float(reference.ref_lon), float(reference.ref_alt)
    )

    def to_local_ned(sample: RigidState) -> RigidState:
        return RigidState(
            sample.time_s,
            world_enu_to_local_ned(sample.position, WORLD_ORIGIN, local_origin),
            sample.orientation,
            sample.linear_velocity,
            sample.angular_velocity,
        )

    deck_truth = sorted(
        (to_local_ned(sample) for sample in truth_world), key=lambda sample: sample.time_s
    )
    uav_truth = with_finite_difference_velocity(
        [to_local_ned(sample) for sample in uav_truth_world]
    )
    if not uav_truth:
        raise RuntimeError(
            f"no Gazebo pose samples found for UAV model {uav_model_name!r}"
        )
    current_truth = [
        relative_truth(deck, uav)
        for deck in deck_truth
        if (uav := interpolate_truth(uav_truth, deck.time_s)) is not None
    ]
    if not current_truth:
        raise RuntimeError("deck and UAV Ground Truth have no common time interval")
    truth_by_receipt_time = sorted(
        (to_local_ned(sample) for sample in truth_by_receipt_time_world),
        key=lambda sample: sample.time_s,
    )
    shadow_states.sort(key=lambda sample: sample.state.time_s)
    statuses.sort()
    trusted_horizons.sort()
    landing_states.sort()
    marker_ids.sort()

    current_errors: dict[str, list[float]] = {key: [] for key in (
        "horizontal_position_m", "vertical_position_m", "normal_deg", "yaw_deg",
        "horizontal_velocity_mps", "vertical_velocity_mps", "angular_velocity_degps",
    )}
    time_sync_error_count = invalid_ground_truth_count
    for sample in shadow_states:
        aligned = interpolate_truth(current_truth, sample.state.time_s)
        if aligned is None:
            time_sync_error_count += 1
            continue
        for key, value in state_errors(sample.state, aligned).items():
            current_errors[key].append(value)

    prediction_errors = {key: [] for key in current_errors}
    acceleration_errors: list[float] = []
    angular_acceleration_errors_degps2: list[float] = []
    excluded_without_future_truth = 0
    for prediction in predictions:
        deck_aligned = interpolate_truth(deck_truth, prediction.target_time_s)
        uav_at_origin = interpolate_truth(uav_truth, prediction.origin_time_s)
        if deck_aligned is None or uav_at_origin is None:
            if deck_truth and prediction.target_time_s > deck_truth[-1].time_s:
                excluded_without_future_truth += 1
            else:
                time_sync_error_count += 1
            continue
        aligned = prediction_truth(deck_aligned, uav_at_origin)
        for key, value in state_errors(prediction.state, aligned).items():
            prediction_errors[key].append(value)
        derivatives = derivative_truth(deck_truth, prediction.target_time_s)
        if derivatives is not None:
            acceleration_errors.append(
                vector_norm(vector_difference(prediction.linear_acceleration, derivatives[0]))
            )
            angular_acceleration_errors_degps2.append(
                math.degrees(
                    vector_norm(vector_difference(prediction.angular_acceleration, derivatives[1]))
                )
            )

    marker_switch_jumps: list[float] = []
    previous_marker: Optional[int] = None
    previous_normal: Optional[tuple[float, ...]] = None
    for sample in sorted(shadow_states, key=lambda item: item.bag_time_s):
        marker = latest_value(marker_ids, sample.bag_time_s)
        normal, _ = normal_and_yaw(sample.state.orientation)
        if (
            marker is not None and previous_marker is not None and marker != previous_marker
            and previous_normal is not None
        ):
            marker_switch_jumps.append(normal_error_deg(normal, previous_normal))
        previous_marker = marker
        previous_normal = normal

    eligible_horizons = [
        (time_s, horizon) for time_s, horizon in trusted_horizons
        if latest_value(landing_states, time_s) in TRACKING_STATES
    ]
    valid_coverage = (
        sum(
            horizon > 0.0 and ":TRUSTED" in (latest_value(statuses, time_s) or "")
            for time_s, horizon in eligible_horizons
        ) / len(eligible_horizons)
        if eligible_horizons else 0.0
    )

    descent_state_count = sum(state in DESCENT_STATES for _, state in landing_states)
    nav_land_count = sum(command == NAV_LAND_COMMAND for command, _ in commands)
    disarm_count = sum(
        command == ARM_DISARM_COMMAND and param1 < 0.5 for command, param1 in commands
    )
    contact_count = 0
    penetration_count = 0
    relative_heights: list[float] = []
    for bag_time_s, local_position in local_positions:
        if latest_value(landing_states, bag_time_s) not in TRACKING_STATES:
            continue
        aligned = interpolate_truth(truth_by_receipt_time, bag_time_s)
        if aligned is None:
            continue
        normal, _ = normal_and_yaw(aligned.orientation)
        height = sum(
            n * delta
            for n, delta in zip(
                normal,
                vector_difference(
                    (local_position.x, local_position.y, local_position.z), aligned.position
                ),
            )
        )
        relative_heights.append(height)
        clearance = height - 0.227
        contact_count += clearance <= 0.0
        penetration_count += clearance < -0.005

    current_summary = {key: summary(values) for key, values in current_errors.items()}
    prediction_summary = {key: summary(values) for key, values in prediction_errors.items()}
    marker_jump_max = max(marker_switch_jumps, default=0.0)

    def metric_at_most(metric: dict[str, float | int | None], key: str, limit: float) -> bool:
        value = metric[key]
        return value is not None and float(value) <= limit

    hard_gates = {
        "no_descent_state": descent_state_count == 0,
        "no_contact": contact_count == 0,
        "no_penetration": penetration_count == 0,
        "no_nav_land": nav_land_count == 0,
        "no_disarm": disarm_count == 0,
        "no_time_sync_error": time_sync_error_count == 0,
        "no_nonfinite_output": invalid_output_count == 0,
        "valid_coverage_ge_95pct": valid_coverage >= 0.95,
        "current_normal_rmse_le_1deg": metric_at_most(
            current_summary["normal_deg"], "rmse", 1.0
        ),
        "current_normal_p95_le_1p5deg": metric_at_most(
            current_summary["normal_deg"], "p95", 1.5
        ),
        "marker_switch_normal_jump_le_1deg": marker_jump_max <= 1.0,
        "prediction_horizontal_p95_le_0p15m": metric_at_most(
            prediction_summary["horizontal_position_m"], "p95", 0.15
        ),
        "prediction_vertical_p95_le_0p10m": metric_at_most(
            prediction_summary["vertical_position_m"], "p95", 0.10
        ),
        "prediction_normal_p95_le_2deg": metric_at_most(
            prediction_summary["normal_deg"], "p95", 2.0
        ),
        "prediction_yaw_p95_le_3deg": metric_at_most(
            prediction_summary["yaw_deg"], "p95", 3.0
        ),
        "prediction_horizontal_velocity_p95_le_0p15mps": metric_at_most(
            prediction_summary["horizontal_velocity_mps"], "p95", 0.15
        ),
        "prediction_vertical_velocity_p95_le_0p10mps": metric_at_most(
            prediction_summary["vertical_velocity_mps"], "p95", 0.10
        ),
        "prediction_angular_velocity_p95_le_2degps": metric_at_most(
            prediction_summary["angular_velocity_degps"], "p95", 2.0
        ),
    }
    return {
        "bag": str(bag_uri),
        "state_contract": (
            "state: uav_centered_ned current deck-uav pose with deck NED twist; "
            "trajectory: uav_origin_ned frozen at header time"
        ),
        "uav_ground_truth_model": uav_model_name,
        "passed": all(hard_gates.values()),
        "hard_gates": hard_gates,
        "valid_coverage": valid_coverage,
        "current_errors": current_summary,
        "prediction_0p5s_errors": prediction_summary,
        "reported_without_hard_gate": {
            "linear_acceleration_vector_error_mps2": summary(acceleration_errors),
            "angular_acceleration_vector_error_degps2": summary(
                angular_acceleration_errors_degps2
            ),
        },
        "marker_switch_normal_jump_deg": summary(marker_switch_jumps),
        "relative_height_m": summary(relative_heights),
        "counts": {
            "descent_state": descent_state_count,
            "contact": contact_count,
            "penetration": penetration_count,
            "nav_land": nav_land_count,
            "disarm": disarm_count,
            "time_sync_error": time_sync_error_count,
            "nonfinite_output": invalid_output_count,
            "invalid_ground_truth": invalid_ground_truth_count,
            "prediction_excluded_without_future_truth": excluded_without_future_truth,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--uav-model-name", default=DEFAULT_UAV_MODEL_NAME)
    args = parser.parse_args()
    result = evaluate(args.bag, args.uav_model_name)
    output = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
