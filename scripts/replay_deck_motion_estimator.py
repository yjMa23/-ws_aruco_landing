#!/usr/bin/env python3
"""使用 production C++ math 对固定 Bag 做 DeckMotionEstimator 离线因果 replay。"""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import subprocess
from typing import Any, Optional, Sequence

import yaml

from analyze_future_twist_causality import (
    POINT_TIME_TOLERANCE_S,
    STATE_TRAJECTORY_PAIR_TOLERANCE_S,
    nearest_state_sample_time,
)
from evaluate_deck_motion_shadow import duration_seconds, load_ros_modules, stamp_seconds

ARUCO_POSE_TOPIC = "/aruco/pose"
ARUCO_ID_TOPIC = "/aruco/id"
ACTIVE_MARKER_TOPIC = "/landing/active_marker_id"
LANDING_STATE_TOPIC = "/landing/state"
STATE_TOPIC = "/landing/deck_motion_shadow/state"
TRAJECTORY_TOPIC = "/landing/deck_motion_shadow/trajectory"
VEHICLE_ODOMETRY_TOPIC = "/fmu/out/vehicle_odometry"

VISUAL_STATES = {
    "ACQUIRE_ARUCO",
    "VISUAL_HANDOVER",
    "TRACK_TARGET",
    "WAIT_LANDING_WINDOW",
    "DESCEND",
    "TEST_HEIGHT_HOLD",
    "FINAL_DESCENT",
    "TOUCHDOWN_CANDIDATE_HOLD",
    "TOUCHDOWN_HOLD",
    "RECOVER_CLIMB",
}

SAMPLE_TIME_MAX_ERROR_S = 1.0e-6
VECTOR_P95_ERROR = 1.0e-6
VECTOR_MAX_ERROR = 1.0e-5
MATCH_STAMP_TOLERANCE_NS = 2

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class ArucoRecord:
    bag_time_s: float
    sample_stamp_ns: int
    position: Vector3
    orientation_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True)
class OdomRecord:
    bag_time_s: float
    sample_timestamp_us: int
    sync_timestamp_us: int
    position: Vector3
    orientation_wxyz: tuple[float, float, float, float]
    velocity: Vector3
    velocity_valid: bool


@dataclass(frozen=True)
class TrajectoryOrigin:
    index: int
    bag_time_s: float
    publish_time_s: float
    sample_time_s: float
    linear_velocity: Vector3
    linear_acceleration: Vector3
    angular_velocity: Vector3
    angular_acceleration: Vector3


@dataclass(frozen=True)
class ReplayPrediction:
    index: int
    valid: bool
    sample_time_s: Optional[float]
    linear_velocity: Optional[Vector3]
    linear_acceleration: Optional[Vector3]
    angular_velocity: Optional[Vector3]
    angular_acceleration: Optional[Vector3]


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


def vector_error(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def accepted_sample_schedule(state_stamps_ns: Sequence[int]) -> list[int]:
    """state header 只在 estimator 接受新 sample 后变化；保留首次及每次变化。"""

    accepted: list[int] = []
    previous: Optional[int] = None
    for value in state_stamps_ns:
        if value <= 0:
            continue
        if previous is None or value != previous:
            accepted.append(value)
            previous = value
    return accepted


def match_accepted_aruco(
    accepted_stamps_ns: Sequence[int], aruco_records: Sequence[ArucoRecord]
) -> list[ArucoRecord]:
    """按图像 header stamp 匹配 accepted schedule，不使用 shadow state 数值作为输入。"""

    by_stamp = sorted(aruco_records, key=lambda record: (record.sample_stamp_ns, record.bag_time_s))
    stamps = [record.sample_stamp_ns for record in by_stamp]
    matched: list[ArucoRecord] = []
    for accepted in accepted_stamps_ns:
        index = bisect.bisect_left(stamps, accepted)
        candidates: list[ArucoRecord] = []
        for candidate_index in (index - 1, index, index + 1):
            if 0 <= candidate_index < len(by_stamp):
                candidate = by_stamp[candidate_index]
                if abs(candidate.sample_stamp_ns - accepted) <= MATCH_STAMP_TOLERANCE_NS:
                    candidates.append(candidate)
        if not candidates:
            raise RuntimeError(f"no /aruco/pose sample matches accepted stamp {accepted}")
        matched.append(min(candidates, key=lambda record: record.bag_time_s))
    return matched


def bag_to_ros_time(bag_time_s: float, anchors: Sequence[tuple[float, float]]) -> float:
    """用 trajectory publish stamp 的稳健全局偏移恢复 ROS simulation receipt time。"""

    if not anchors or not math.isfinite(bag_time_s):
        raise ValueError("bag-to-ROS mapping requires finite time and anchors")
    # rosbag receipt 使用 wall time，而 node callback 的 get_clock() 使用 simulation time。
    # 两者在固定 episode 内同速推进；trajectory header 提供同一控制 tick 的 simulation time。
    # 使用全局中位数避免把 rosbag 跨 topic 的 1~2 ms recorder latency 抖动误注入 PX4 clock filter。
    offset = statistics.median(bag - ros for bag, ros in anchors)
    return bag_time_s - offset


def covariance_diagonal_std(variance: Vector3) -> Vector3:
    if any(not math.isfinite(value) or value < -1.0e-12 for value in variance):
        raise ValueError("covariance diagonal must be finite and non-negative")
    return tuple(math.sqrt(max(0.0, value)) for value in variance)  # type: ignore[return-value]


def load_episode_inputs(bag_path: Path) -> dict[str, Any]:
    rosbag2_py, deserialize_message, get_message, StorageOptions, ConverterOptions = load_ros_modules()
    reader = rosbag2_py.SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = {
        ARUCO_POSE_TOPIC,
        ARUCO_ID_TOPIC,
        ACTIVE_MARKER_TOPIC,
        LANDING_STATE_TOPIC,
        STATE_TOPIC,
        TRAJECTORY_TOPIC,
        VEHICLE_ODOMETRY_TOPIC,
    }
    missing = required - topic_types.keys()
    if missing:
        raise RuntimeError("bag is missing required replay topics: " + ", ".join(sorted(missing)))
    message_types = {topic: get_message(topic_types[topic]) for topic in required}

    aruco: list[ArucoRecord] = []
    odometry: list[OdomRecord] = []
    marker_events: list[tuple[float, int]] = []
    active_marker_count = 0
    landing_state_events: list[tuple[float, str]] = []
    state_stamps: list[tuple[float, int]] = []
    raw_trajectories: list[tuple[float, float, Any]] = []

    odom_type = message_types[VEHICLE_ODOMETRY_TOPIC]
    pose_frame_ned = int(getattr(odom_type, "POSE_FRAME_NED"))
    velocity_frame_ned = int(getattr(odom_type, "VELOCITY_FRAME_NED"))

    while reader.has_next():
        topic, serialized, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        bag_time_s = bag_timestamp_ns * 1.0e-9
        if topic == ARUCO_POSE_TOPIC:
            pose = message.pose
            values = (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z,
            )
            if all(math.isfinite(value) for value in values):
                aruco.append(
                    ArucoRecord(
                        bag_time_s,
                        stamp_ns(message.header.stamp),
                        (pose.position.x, pose.position.y, pose.position.z),
                        (
                            pose.orientation.w, pose.orientation.x,
                            pose.orientation.y, pose.orientation.z,
                        ),
                    )
                )
        elif topic == ARUCO_ID_TOPIC:
            marker_events.append((bag_time_s, int(message.data)))
        elif topic == ACTIVE_MARKER_TOPIC:
            # 该 topic 只用于确认 Bag 中的 controller marker snapshot 存在；
            # estimator update 的 marker_id 实际来自 /aruco/id。
            active_marker_count += 1
        elif topic == LANDING_STATE_TOPIC:
            landing_state_events.append((bag_time_s, str(message.data)))
        elif topic == STATE_TOPIC:
            state_stamps.append((bag_time_s, stamp_ns(message.header.stamp)))
        elif topic == TRAJECTORY_TOPIC:
            origin = None
            for point in message.points:
                if abs(duration_seconds(point.time_from_start)) <= POINT_TIME_TOLERANCE_S:
                    origin = point
                    break
            if origin is not None and len(origin.velocities) == 1 and len(origin.accelerations) == 1:
                raw_trajectories.append((bag_time_s, stamp_seconds(message.header.stamp), origin))
        elif topic == VEHICLE_ODOMETRY_TOPIC:
            pose_values = tuple(float(value) for value in message.position)
            q = tuple(float(value) for value in message.q)
            if (
                int(message.pose_frame) != pose_frame_ned
                or not all(math.isfinite(value) for value in pose_values)
                or not all(math.isfinite(value) for value in q)
            ):
                continue
            sample_timestamp_us = int(message.timestamp_sample or message.timestamp)
            sync_timestamp_us = int(message.timestamp or sample_timestamp_us)
            velocity = tuple(float(value) for value in message.velocity)
            velocity_valid = (
                int(message.velocity_frame) == velocity_frame_ned
                and all(math.isfinite(value) for value in velocity)
            )
            odometry.append(
                OdomRecord(
                    bag_time_s,
                    sample_timestamp_us,
                    sync_timestamp_us,
                    pose_values,  # type: ignore[arg-type]
                    q,  # type: ignore[arg-type]
                    velocity if velocity_valid else (0.0, 0.0, 0.0),  # type: ignore[arg-type]
                    velocity_valid,
                )
            )

    state_stamps.sort(key=lambda item: item[0])
    aruco.sort(key=lambda item: item.bag_time_s)
    odometry.sort(key=lambda item: item.bag_time_s)
    marker_events.sort()
    landing_state_events.sort()
    raw_trajectories.sort(key=lambda item: item[0])

    state_pairs_s = [(bag_time, sample_ns * 1.0e-9) for bag_time, sample_ns in state_stamps]
    trajectories: list[TrajectoryOrigin] = []
    for index, (bag_time_s, publish_time_s, origin) in enumerate(raw_trajectories):
        sample_time_s = nearest_state_sample_time(
            state_pairs_s, bag_time_s, STATE_TRAJECTORY_PAIR_TOLERANCE_S
        )
        if sample_time_s is None:
            raise RuntimeError(f"trajectory {index} has no paired shadow state")
        velocity = origin.velocities[0]
        acceleration = origin.accelerations[0]
        values = (
            velocity.linear.x, velocity.linear.y, velocity.linear.z,
            acceleration.linear.x, acceleration.linear.y, acceleration.linear.z,
            velocity.angular.x, velocity.angular.y, velocity.angular.z,
            acceleration.angular.x, acceleration.angular.y, acceleration.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f"trajectory {index} origin contains nonfinite values")
        trajectories.append(
            TrajectoryOrigin(
                index,
                bag_time_s,
                publish_time_s,
                sample_time_s,
                (velocity.linear.x, velocity.linear.y, velocity.linear.z),
                (acceleration.linear.x, acceleration.linear.y, acceleration.linear.z),
                (velocity.angular.x, velocity.angular.y, velocity.angular.z),
                (acceleration.angular.x, acceleration.angular.y, acceleration.angular.z),
            )
        )

    accepted_stamps = accepted_sample_schedule([sample for _, sample in state_stamps])
    accepted_aruco = match_accepted_aruco(accepted_stamps, aruco)
    return {
        "aruco": aruco,
        "accepted_aruco": accepted_aruco,
        "accepted_stamps_ns": accepted_stamps,
        "odometry": odometry,
        "marker_events": marker_events,
        "landing_state_events": landing_state_events,
        "trajectories": trajectories,
        "anchors": [(record.bag_time_s, record.publish_time_s) for record in trajectories],
        "topic_counts": {
            ARUCO_POSE_TOPIC: len(aruco),
            ARUCO_ID_TOPIC: len(marker_events),
            ACTIVE_MARKER_TOPIC: active_marker_count,
            LANDING_STATE_TOPIC: len(landing_state_events),
            STATE_TOPIC: len(state_stamps),
            TRAJECTORY_TOPIC: len(trajectories),
            VEHICLE_ODOMETRY_TOPIC: len(odometry),
        },
    }


def latest_marker_id(marker_events: Sequence[tuple[float, int]], bag_time_s: float) -> int:
    if not marker_events:
        return -1
    bag_times = [item[0] for item in marker_events]
    index = bisect.bisect_right(bag_times, bag_time_s) - 1
    return marker_events[index][1] if index >= 0 else -1


def latest_landing_state(
    state_events: Sequence[tuple[float, str]], bag_time_s: float
) -> Optional[str]:
    if not state_events:
        return None
    bag_times = [item[0] for item in state_events]
    index = bisect.bisect_right(bag_times, bag_time_s) - 1
    return state_events[index][1] if index >= 0 else None


def clamp_aruco_event_time(
    record_bag_time_s: float,
    first_index: int,
    trajectories: Sequence[TrajectoryOrigin],
) -> float:
    """将 ArUco callback 排在第一个已反映该 sample 的 production control tick 之前。"""

    if first_index < 0 or first_index >= len(trajectories):
        raise ValueError("first_index is outside trajectory range")
    upper = trajectories[first_index].bag_time_s - 1.0e-4
    lower = (
        trajectories[first_index - 1].bag_time_s + 1.0e-4
        if first_index > 0 else -math.inf
    )
    return min(max(record_bag_time_s, lower), upper)


def config_line(controller_config: Path) -> str:
    params = yaml.safe_load(controller_config.read_text(encoding="utf-8"))[
        "px4_aruco_landing_node"
    ]["ros__parameters"]
    t = params["camera_extrinsic.translation_frd_m"]
    q = params["camera_extrinsic.rotation_wxyz"]
    names = (
        "deck_motion_shadow.linear_jerk_std_mps3",
        "deck_motion_shadow.angular_jerk_std_radps3",
        "deck_motion_shadow.measurement_horizontal_std_m",
        "deck_motion_shadow.measurement_vertical_std_m",
        "deck_motion_shadow.measurement_orientation_std_rad",
        "deck_motion_shadow.initial_position_std_m",
        "deck_motion_shadow.initial_velocity_std_mps",
        "deck_motion_shadow.initial_acceleration_std_mps2",
        "deck_motion_shadow.initial_orientation_std_rad",
        "deck_motion_shadow.initial_angular_velocity_std_radps",
        "deck_motion_shadow.initial_angular_acceleration_std_radps2",
        "deck_motion_shadow.minimum_sample_dt_s",
        "deck_motion_shadow.maximum_sample_dt_s",
        "deck_motion_shadow.reinitialize_gap_s",
        "deck_motion_shadow.position_innovation_gate_mahalanobis",
        "deck_motion_shadow.orientation_innovation_gate_mahalanobis",
        "deck_motion_shadow.minimum_upward_normal_component",
        "deck_motion_shadow.prediction_sample_period_s",
        "deck_motion_shadow.trusted_prediction_horizon_s",
        "deck_motion_shadow.maximum_prediction_horizon_s",
        "deck_motion_shadow.kinematic_fit_window_s",
    )
    values = [
        params["vehicle_pose_history.history_duration_s"],
        params["vehicle_pose_history.max_endpoint_hold_s"],
        params["vehicle_pose_history.clock_offset_filter_gain"],
        params["vehicle_pose_history.max_clock_offset_jump_s"],
        *t,
        *q,
        *(params[name] for name in names),
    ]
    return "CONFIG " + " ".join(format(float(value), ".17g") for value in values)


def build_replay_input(episode_dir: Path, inputs: dict[str, Any]) -> str:
    lines = [config_line(episode_dir / "controller_config.yaml")]
    marker_events = inputs["marker_events"]
    landing_state_events = inputs["landing_state_events"]
    events: list[tuple[float, int, str]] = []

    for odom in inputs["odometry"]:
        receipt_ros_time_s = bag_to_ros_time(odom.bag_time_s, inputs["anchors"])
        values = (
            receipt_ros_time_s,
            odom.sample_timestamp_us,
            odom.sync_timestamp_us,
            *odom.position,
            *odom.orientation_wxyz,
            *odom.velocity,
            int(odom.velocity_valid),
        )
        events.append(
            (odom.bag_time_s, 0, "ODOM " + " ".join(str(value) for value in values))
        )

    trajectories = inputs["trajectories"]
    if not trajectories:
        raise RuntimeError("episode has no production trajectory origins")
    final_sample_ns = int(round(trajectories[-1].sample_time_s * 1.0e9))
    for record in inputs["aruco"]:
        if record.sample_stamp_ns > final_sample_ns + MATCH_STAMP_TOLERANCE_NS:
            continue
        if latest_landing_state(landing_state_events, record.bag_time_s) not in VISUAL_STATES:
            continue
        marker_id = latest_marker_id(marker_events, record.bag_time_s)
        values = (
            record.sample_stamp_ns * 1.0e-9,
            marker_id,
            *record.position,
            *record.orientation_wxyz,
        )
        # 每个 state header 只暴露 control tick 时“最后一个” accepted sample，30 Hz ArUco
        # 可能在两个 20 Hz tick 之间被 estimator 连续接受多次。因此 replay 必须保留这些中间
        # causal sensor updates；state 仅用于保证任何未来 sample 不会被错误地提前到旧 control tick 前。
        first_index = next(
            (
                index for index, trajectory in enumerate(trajectories)
                if int(round(trajectory.sample_time_s * 1.0e9))
                >= record.sample_stamp_ns - MATCH_STAMP_TOLERANCE_NS
            ),
            None,
        )
        if first_index is None:
            continue
        sort_time = clamp_aruco_event_time(record.bag_time_s, first_index, trajectories)
        events.append(
            (sort_time, 1, "ARUCO " + " ".join(str(value) for value in values))
        )

    for trajectory in trajectories:
        events.append(
            (
                trajectory.bag_time_s,
                2,
                f"PREDICT {trajectory.index} {trajectory.publish_time_s:.17g}",
            )
        )

    events.sort(key=lambda item: (item[0], item[1]))
    lines.extend(event[2] for event in events)
    return "\n".join(lines) + "\n"


def parse_replay_output(text: str) -> tuple[dict[int, ReplayPrediction], list[dict[str, Any]]]:
    predictions: dict[int, ReplayPrediction] = {}
    updates: list[dict[str, Any]] = []
    for line in text.splitlines():
        fields = line.split("\t")
        if not fields:
            continue
        if fields[0] == "U":
            updates.append({"sample_time_s": float(fields[1]), "status": fields[2]})
            continue
        if fields[0] != "P":
            continue
        index = int(fields[1])
        valid = fields[2] == "1"
        if not valid:
            predictions[index] = ReplayPrediction(
                index, False, None, None, None, None, None
            )
            continue
        values = [float(value) for value in fields[3:]]
        if len(values) != 13:
            raise RuntimeError(f"unexpected replay prediction field count: {len(values)}")
        predictions[index] = ReplayPrediction(
            index=index,
            valid=True,
            sample_time_s=values[0],
            linear_velocity=tuple(values[1:4]),  # type: ignore[arg-type]
            linear_acceleration=tuple(values[4:7]),  # type: ignore[arg-type]
            angular_velocity=tuple(values[7:10]),  # type: ignore[arg-type]
            angular_acceleration=tuple(values[10:13]),  # type: ignore[arg-type]
        )
    return predictions, updates


def error_distribution(values: Sequence[float]) -> dict[str, float | int | None]:
    clean = [value for value in values if math.isfinite(value)]
    return {
        "count": len(clean),
        "p95": percentile(clean, 0.95),
        "max": max(clean) if clean else None,
    }


def compare_replay(
    trajectories: Sequence[TrajectoryOrigin], predictions: dict[int, ReplayPrediction]
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    sample_errors: list[float] = []
    linear_velocity_errors: list[float] = []
    linear_acceleration_errors: list[float] = []
    angular_velocity_errors: list[float] = []
    angular_acceleration_errors: list[float] = []
    frames: list[dict[str, Any]] = []

    for production in trajectories:
        replay = predictions.get(production.index)
        if replay is None or not replay.valid:
            continue
        assert replay.sample_time_s is not None
        assert replay.linear_velocity is not None
        assert replay.linear_acceleration is not None
        assert replay.angular_velocity is not None
        assert replay.angular_acceleration is not None
        sample_error = abs(replay.sample_time_s - production.sample_time_s)
        lv = vector_error(replay.linear_velocity, production.linear_velocity)
        la = vector_error(replay.linear_acceleration, production.linear_acceleration)
        av = vector_error(replay.angular_velocity, production.angular_velocity)
        aa = vector_error(replay.angular_acceleration, production.angular_acceleration)
        sample_errors.append(sample_error)
        linear_velocity_errors.append(lv)
        linear_acceleration_errors.append(la)
        angular_velocity_errors.append(av)
        angular_acceleration_errors.append(aa)
        frames.append(
            {
                "index": production.index,
                "publish_time_s": production.publish_time_s,
                "sample_time_s": replay.sample_time_s,
                "linear_velocity_ned_mps": replay.linear_velocity,
                "linear_acceleration_ned_mps2": replay.linear_acceleration,
                "angular_velocity_ned_radps": replay.angular_velocity,
                "angular_acceleration_ned_radps2": replay.angular_acceleration,
            }
        )

    coverage = len(frames) / len(trajectories) if trajectories else 0.0
    metrics = {
        "paired_origin_count": len(frames),
        "production_origin_count": len(trajectories),
        "paired_origin_coverage": coverage,
        "sample_time_abs_error_s": error_distribution(sample_errors),
        "linear_velocity_vector_error_mps": error_distribution(linear_velocity_errors),
        "linear_acceleration_vector_error_mps2": error_distribution(linear_acceleration_errors),
        "angular_velocity_vector_error_radps": error_distribution(angular_velocity_errors),
        "angular_acceleration_vector_error_radps2": error_distribution(angular_acceleration_errors),
    }
    def value_or_inf(summary: dict[str, float | int | None], key: str) -> float:
        value = summary[key]
        return float(value) if value is not None else math.inf

    metrics["strict_equivalence_pass"] = bool(
        coverage >= 0.99
        and value_or_inf(metrics["sample_time_abs_error_s"], "max") <= SAMPLE_TIME_MAX_ERROR_S
        and value_or_inf(metrics["linear_velocity_vector_error_mps"], "p95") <= VECTOR_P95_ERROR
        and value_or_inf(metrics["linear_velocity_vector_error_mps"], "max") <= VECTOR_MAX_ERROR
        and value_or_inf(metrics["linear_acceleration_vector_error_mps2"], "p95") <= VECTOR_P95_ERROR
        and value_or_inf(metrics["linear_acceleration_vector_error_mps2"], "max") <= VECTOR_MAX_ERROR
        and value_or_inf(metrics["angular_velocity_vector_error_radps"], "p95") <= VECTOR_P95_ERROR
        and value_or_inf(metrics["angular_velocity_vector_error_radps"], "max") <= VECTOR_MAX_ERROR
        and value_or_inf(metrics["angular_acceleration_vector_error_radps2"], "p95") <= VECTOR_P95_ERROR
        and value_or_inf(metrics["angular_acceleration_vector_error_radps2"], "max") <= VECTOR_MAX_ERROR
    )
    return metrics, frames


def run_episode(episode_dir: Path, executable: Path) -> dict[str, Any]:
    inputs = load_episode_inputs(episode_dir / "bag")
    replay_input = build_replay_input(episode_dir, inputs)
    completed = subprocess.run(
        [str(executable)],
        input=replay_input,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"replay executable failed for {episode_dir.name}: {completed.stderr.strip()}"
        )
    predictions, updates = parse_replay_output(completed.stdout)
    metrics, frames = compare_replay(inputs["trajectories"], predictions)
    return {
        "episode": episode_dir.name,
        "topic_counts": inputs["topic_counts"],
        "accepted_sample_count": len(inputs["accepted_stamps_ns"]),
        "matched_accepted_aruco_count": len(inputs["accepted_aruco"]),
        "update_status_counts": {
            status: sum(1 for item in updates if item["status"] == status)
            for status in sorted({item["status"] for item in updates})
        },
        "bag_to_ros_offset_s": {
            "median": statistics.median(
                bag - ros for bag, ros in inputs["anchors"]
            ),
            "min": min(bag - ros for bag, ros in inputs["anchors"]),
            "max": max(bag - ros for bag, ros in inputs["anchors"]),
        },
        "equivalence": metrics,
        "frames": frames,
    }


def aggregate_equivalence(episodes: Sequence[dict[str, Any]]) -> dict[str, Any]:
    total = sum(item["equivalence"]["production_origin_count"] for item in episodes)
    paired = sum(item["equivalence"]["paired_origin_count"] for item in episodes)
    keys = (
        "sample_time_abs_error_s",
        "linear_velocity_vector_error_mps",
        "linear_acceleration_vector_error_mps2",
        "angular_velocity_vector_error_radps",
        "angular_acceleration_vector_error_radps2",
    )
    # Episode-level max/P95 are retained; global conservative summary uses the worst episode values.
    aggregate: dict[str, Any] = {
        "paired_origin_count": paired,
        "production_origin_count": total,
        "paired_origin_coverage": paired / total if total else 0.0,
    }
    for key in keys:
        p95_values = [item["equivalence"][key]["p95"] for item in episodes]
        max_values = [item["equivalence"][key]["max"] for item in episodes]
        aggregate[key] = {
            "worst_episode_p95": max(value for value in p95_values if value is not None),
            "max": max(value for value in max_values if value is not None),
        }
    aggregate["strict_equivalence_pass"] = all(
        item["equivalence"]["strict_equivalence_pass"] for item in episodes
    )
    return aggregate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--replay-executable",
        type=Path,
        default=Path(
            "install/aruco_precision_landing_cpp/lib/aruco_precision_landing_cpp/"
            "deck_motion_estimator_replay"
        ),
    )
    args = parser.parse_args()
    if not args.replay_executable.is_file():
        raise SystemExit(f"replay executable not found: {args.replay_executable}")

    episode_dirs = sorted(
        path for path in args.matrix_dir.iterdir()
        if path.is_dir() and (path / "bag" / "metadata.yaml").is_file()
    )
    if len(episode_dirs) != 12:
        raise SystemExit(f"expected fixed 12-Bag matrix, found {len(episode_dirs)} episodes")

    episodes = [run_episode(episode_dir, args.replay_executable) for episode_dir in episode_dirs]
    result = {
        "matrix_dir": str(args.matrix_dir),
        "causal_topics": [
            ARUCO_POSE_TOPIC,
            ARUCO_ID_TOPIC,
            ACTIVE_MARKER_TOPIC + " (presence/count validation only)",
            LANDING_STATE_TOPIC + " (causal visual-state eligibility only)",
            STATE_TOPIC + " (header stamp only for accepted-time/control-tick reconstruction)",
            TRAJECTORY_TOPIC + " (publish time and production origin only for equivalence)",
            VEHICLE_ODOMETRY_TOPIC,
        ],
        "production_math": {
            "deck_motion_estimator": True,
            "vehicle_pose_history": True,
            "coordinate_transform": True,
            "production_estimator_modified": False,
        },
        "equivalence_thresholds": {
            "paired_origin_coverage_min": 0.99,
            "sample_time_max_abs_error_s": SAMPLE_TIME_MAX_ERROR_S,
            "vector_p95_error": VECTOR_P95_ERROR,
            "vector_max_error": VECTOR_MAX_ERROR,
        },
        "episodes": episodes,
        "aggregate_equivalence": aggregate_equivalence(episodes),
    }
    output = args.output or args.matrix_dir / "future_twist_estimator_replay.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(result["aggregate_equivalence"], indent=2, allow_nan=False))
    print(f"wrote {output}")
    return 0 if result["aggregate_equivalence"]["strict_equivalence_pass"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
