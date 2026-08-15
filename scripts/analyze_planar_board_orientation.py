#!/usr/bin/env python3
"""仅使用既有 Bag 诊断 Marine Planar Board 的甲板法向误差来源。"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Sequence

from evaluate_deck_motion_shadow import (
    ARUCO_BOARD_MARKER_COUNT_TOPIC,
    ARUCO_BOARD_REPROJECTION_RMSE_TOPIC,
    ARUCO_POSE_SOURCE_TOPIC,
    ARUCO_POSE_TOPIC,
    ARUCO_VISIBLE_TOPIC,
    GROUND_TRUTH_TOPIC,
    LANDING_STATE_TOPIC,
    Q_BODY_DECK,
    Q_NED_ENU,
    STATE_TOPIC,
    TRACKING_STATES,
    UAV_GROUND_TRUTH_TOPIC,
    RigidState,
    interpolate_truth,
    load_ros_modules,
    normal_and_yaw,
    normal_error_deg,
    normalize_quaternion,
    normalize_quaternion_or_none,
    pair_planar_diagnostics,
    percentile,
    quaternion_multiply,
    quaternion_rotate,
    resolve_bag_uri,
    stamp_seconds,
    vector_norm,
)

MARKER_POSE_NED_TOPIC = "/landing/marker_pose_ned"
VEHICLE_ODOMETRY_TOPIC = "/fmu/out/vehicle_odometry"
DEFAULT_UAV_MODEL_NAME = "x500_mono_cam_down_0"
Q_BODY_FRD_CAMERA_OPTICAL = (
    math.sqrt(0.5),
    0.0,
    0.0,
    math.sqrt(0.5),
)
# Gazebo x500 model pose uses FLU body axes while PX4 VehicleOdometry uses FRD.
Q_GAZEBO_UAV_FLU_TO_FRD = (0.0, 1.0, 0.0, 0.0)
PAIR_TOLERANCE_S = 0.05
PX4_GT_RECEIPT_PAIR_TOLERANCE_S = 0.03
PX4_POSE_FRAME_NED = 1
PRE_ACQUIRE_STATES = {
    "INIT",
    "WAIT_FOR_PX4",
    "OFFBOARD_PRE_STREAM",
    "ARM_AND_TAKEOFF",
    "WAIT_DECK_GNSS",
    "RENDEZVOUS_GNSS",
}
TILT_BINS = (
    (0.0, 0.5, "0-0.5 deg"),
    (0.5, 1.0, "0.5-1 deg"),
    (1.0, 2.0, "1-2 deg"),
    (2.0, 3.0, "2-3 deg"),
    (3.0, math.inf, ">3 deg"),
)


@dataclass(frozen=True)
class TimedPose:
    bag_time_s: float
    sample_time_s: float
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class TimedOrientation:
    bag_time_s: float
    sample_time_s: float
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class Px4Orientation:
    bag_time_s: float
    timestamp_sample_s: float
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class GeometryFrame:
    episode: str
    scenario: str
    true_tilt_deg: float
    estimated_tilt_deg: float
    raw_gt_attitude_normal_error_deg: float
    marker_pose_ned_normal_error_deg: float
    camera_to_board_distance_m: float
    board_center_x_normalized: float
    board_center_y_normalized: float
    board_center_radius_normalized: float
    marker_count: int
    reprojection_rmse_px: float


def latest_value(samples: Sequence[tuple[float, Any]], time_s: float) -> Any:
    if not samples:
        return None
    times = [sample[0] for sample in samples]
    index = bisect.bisect_right(times, time_s) - 1
    return samples[index][1] if index >= 0 else None


def nearest_sample(
    samples: Sequence[Any],
    times: Sequence[float],
    time_s: float,
    tolerance_s: float,
) -> Optional[Any]:
    if not samples:
        return None
    insertion = bisect.bisect_left(times, time_s)
    candidates = [
        index
        for index in (insertion - 1, insertion)
        if 0 <= index < len(samples)
    ]
    if not candidates:
        return None
    index = min(candidates, key=lambda candidate: abs(times[candidate] - time_s))
    return samples[index] if abs(times[index] - time_s) <= tolerance_s else None


def quaternion_angle_error_deg(a: Sequence[float], b: Sequence[float]) -> float:
    qa = normalize_quaternion(a)
    qb = normalize_quaternion(b)
    dot = abs(sum(x * y for x, y in zip(qa, qb)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def body_normal_error_deg(a: Sequence[float], b: Sequence[float]) -> float:
    return normal_error_deg(
        quaternion_rotate(a, (0.0, 0.0, 1.0)),
        quaternion_rotate(b, (0.0, 0.0, 1.0)),
    )


def deck_tilt_deg(orientation_ned: Sequence[float]) -> float:
    normal, _ = normal_and_yaw(orientation_ned)
    return normal_error_deg(normal, (0.0, 0.0, -1.0))


def distribution_summary(values: Sequence[float]) -> dict[str, float | int | None]:
    finite_values = [float(value) for value in values if math.isfinite(value)]
    if not finite_values:
        return {
            "count": 0,
            "median": None,
            "rmse": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": len(finite_values),
        "median": percentile(finite_values, 0.50),
        "rmse": math.sqrt(
            sum(value * value for value in finite_values) / len(finite_values)
        ),
        "p95": percentile(finite_values, 0.95),
        "max": max(finite_values),
    }


def tilt_bin_label(tilt_deg: float) -> Optional[str]:
    if not math.isfinite(tilt_deg) or tilt_deg < 0.0:
        return None
    for lower, upper, label in TILT_BINS:
        if lower <= tilt_deg < upper:
            return label
    return None


def pearson_correlation(x: Sequence[float], y: Sequence[float]) -> Optional[float]:
    pairs = [
        (float(a), float(b))
        for a, b in zip(x, y)
        if math.isfinite(a) and math.isfinite(b)
    ]
    if len(pairs) < 2:
        return None
    xs = [pair[0] for pair in pairs]
    ys = [pair[1] for pair in pairs]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    x_ss = sum((value - x_mean) ** 2 for value in xs)
    y_ss = sum((value - y_mean) ** 2 for value in ys)
    if x_ss <= 1.0e-18 or y_ss <= 1.0e-18:
        return None
    covariance = sum(
        (a - x_mean) * (b - y_mean) for a, b in pairs
    )
    return covariance / math.sqrt(x_ss * y_ss)


def reconstruct_deck_orientation_with_uav_gt(
    uav_orientation_ned_frd: Sequence[float],
    camera_deck_orientation: Sequence[float],
    body_camera_orientation: Sequence[float] = Q_BODY_FRD_CAMERA_OPTICAL,
) -> tuple[float, float, float, float]:
    return normalize_quaternion(
        quaternion_multiply(
            quaternion_multiply(uav_orientation_ned_frd, body_camera_orientation),
            camera_deck_orientation,
        )
    )


def _rigid_orientation_state(
    sample_time_s: float,
    orientation: Sequence[float],
) -> RigidState:
    return RigidState(
        sample_time_s,
        (0.0, 0.0, 0.0),
        normalize_quaternion(orientation),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
    )


def load_episode(
    bag_path: Path,
    uav_model_name: str = DEFAULT_UAV_MODEL_NAME,
) -> dict[str, Any]:
    rosbag2_py, deserialize_message, get_message, StorageOptions, ConverterOptions = (
        load_ros_modules()
    )
    topics = {
        GROUND_TRUTH_TOPIC,
        UAV_GROUND_TRUTH_TOPIC,
        STATE_TOPIC,
        LANDING_STATE_TOPIC,
        ARUCO_POSE_TOPIC,
        ARUCO_VISIBLE_TOPIC,
        ARUCO_POSE_SOURCE_TOPIC,
        ARUCO_BOARD_MARKER_COUNT_TOPIC,
        ARUCO_BOARD_REPROJECTION_RMSE_TOPIC,
        MARKER_POSE_NED_TOPIC,
        VEHICLE_ODOMETRY_TOPIC,
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        StorageOptions(uri=str(resolve_bag_uri(bag_path)), storage_id="sqlite3"),
        ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = topics - topic_types.keys()
    if missing:
        raise RuntimeError("bag is missing required topics: " + ", ".join(sorted(missing)))
    message_types = {topic: get_message(topic_types[topic]) for topic in topics}

    deck_truth: list[RigidState] = []
    uav_truth: list[RigidState] = []
    uav_truth_receipt: list[TimedOrientation] = []
    raw_poses: list[TimedPose] = []
    marker_poses_ned: list[TimedPose] = []
    shadow_states: list[TimedPose] = []
    px4_orientations: list[Px4Orientation] = []
    landing_states: list[tuple[float, str]] = []
    pose_sources: list[tuple[float, str]] = []
    marker_counts: list[tuple[float, int]] = []
    reprojection_rmses: list[tuple[float, float]] = []
    visibility: list[tuple[float, bool]] = []

    while reader.has_next():
        topic, serialized, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        bag_time_s = bag_timestamp_ns * 1.0e-9
        if topic == GROUND_TRUTH_TOPIC:
            pose = message.pose.pose
            q_enu_deck_body = normalize_quaternion_or_none(
                (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            )
            if q_enu_deck_body is None:
                continue
            q_ned_deck = normalize_quaternion(
                quaternion_multiply(
                    quaternion_multiply(Q_NED_ENU, q_enu_deck_body),
                    Q_BODY_DECK,
                )
            )
            deck_truth.append(
                _rigid_orientation_state(stamp_seconds(message.header.stamp), q_ned_deck)
            )
        elif topic == UAV_GROUND_TRUTH_TOPIC:
            if message.child_frame_id != uav_model_name + "/base_link":
                continue
            pose = message.pose.pose
            q_enu_flu = normalize_quaternion_or_none(
                (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            )
            if q_enu_flu is None:
                continue
            # UAV GT orientation is only used by this offline diagnosis. Convert Gazebo
            # world ENU / model FLU into PX4 local NED / body FRD before comparison.
            q_ned_frd = normalize_quaternion(
                quaternion_multiply(
                    quaternion_multiply(Q_NED_ENU, q_enu_flu),
                    Q_GAZEBO_UAV_FLU_TO_FRD,
                )
            )
            sample_time_s = stamp_seconds(message.header.stamp)
            uav_truth.append(_rigid_orientation_state(sample_time_s, q_ned_frd))
            uav_truth_receipt.append(
                TimedOrientation(bag_time_s, sample_time_s, q_ned_frd)
            )
        elif topic == ARUCO_POSE_TOPIC:
            pose = message.pose
            q = normalize_quaternion_or_none(
                (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            )
            values = (pose.position.x, pose.position.y, pose.position.z)
            if q is None or not all(math.isfinite(value) for value in values):
                continue
            raw_poses.append(
                TimedPose(
                    bag_time_s,
                    stamp_seconds(message.header.stamp),
                    tuple(float(value) for value in values),
                    q,
                )
            )
        elif topic == MARKER_POSE_NED_TOPIC:
            pose = message.pose
            q = normalize_quaternion_or_none(
                (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            )
            values = (pose.position.x, pose.position.y, pose.position.z)
            if q is None or not all(math.isfinite(value) for value in values):
                continue
            marker_poses_ned.append(
                TimedPose(
                    bag_time_s,
                    stamp_seconds(message.header.stamp),
                    tuple(float(value) for value in values),
                    q,
                )
            )
        elif topic == STATE_TOPIC:
            pose = message.pose.pose
            q = normalize_quaternion_or_none(
                (
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                )
            )
            values = (pose.position.x, pose.position.y, pose.position.z)
            if q is None or not all(math.isfinite(value) for value in values):
                continue
            shadow_states.append(
                TimedPose(
                    bag_time_s,
                    stamp_seconds(message.header.stamp),
                    tuple(float(value) for value in values),
                    q,
                )
            )
        elif topic == VEHICLE_ODOMETRY_TOPIC:
            q = normalize_quaternion_or_none(tuple(float(value) for value in message.q))
            timestamp_us = int(message.timestamp_sample or message.timestamp)
            pose_frame = int(message.pose_frame)
            if q is None or timestamp_us <= 0 or pose_frame != PX4_POSE_FRAME_NED:
                continue
            px4_orientations.append(
                Px4Orientation(bag_time_s, timestamp_us * 1.0e-6, q)
            )
        elif topic == LANDING_STATE_TOPIC:
            landing_states.append((bag_time_s, str(message.data)))
        elif topic == ARUCO_POSE_SOURCE_TOPIC:
            pose_sources.append((bag_time_s, str(message.data)))
        elif topic == ARUCO_BOARD_MARKER_COUNT_TOPIC:
            marker_counts.append((bag_time_s, int(message.data)))
        elif topic == ARUCO_BOARD_REPROJECTION_RMSE_TOPIC:
            reprojection_rmses.append((bag_time_s, float(message.data)))
        elif topic == ARUCO_VISIBLE_TOPIC:
            visibility.append((bag_time_s, bool(message.data)))

    deck_truth.sort(key=lambda sample: sample.time_s)
    uav_truth.sort(key=lambda sample: sample.time_s)
    uav_truth_receipt.sort(key=lambda sample: sample.bag_time_s)
    raw_poses.sort(key=lambda sample: sample.bag_time_s)
    marker_poses_ned.sort(key=lambda sample: sample.bag_time_s)
    shadow_states.sort(key=lambda sample: sample.bag_time_s)
    px4_orientations.sort(key=lambda sample: sample.bag_time_s)
    landing_states.sort()
    pose_sources.sort()
    marker_counts.sort()
    reprojection_rmses.sort()
    visibility.sort()
    diagnostic_frames, unpaired_diagnostics = pair_planar_diagnostics(
        pose_sources, marker_counts, reprojection_rmses, visibility
    )

    return {
        "deck_truth": deck_truth,
        "uav_truth": uav_truth,
        "uav_truth_receipt": uav_truth_receipt,
        "raw_poses": raw_poses,
        "marker_poses_ned": marker_poses_ned,
        "shadow_states": shadow_states,
        "px4_orientations": px4_orientations,
        "landing_states": landing_states,
        "diagnostic_frames": diagnostic_frames,
        "unpaired_diagnostics": unpaired_diagnostics,
    }


def estimate_px4_to_ros_offset_s(
    px4_samples: Sequence[Px4Orientation],
    gt_receipt_samples: Sequence[TimedOrientation],
) -> tuple[float, dict[str, Any]]:
    gt_times = [sample.bag_time_s for sample in gt_receipt_samples]
    offsets: list[float] = []
    receipt_gaps_ms: list[float] = []
    for sample in px4_samples:
        gt = nearest_sample(
            gt_receipt_samples,
            gt_times,
            sample.bag_time_s,
            PX4_GT_RECEIPT_PAIR_TOLERANCE_S,
        )
        if gt is None:
            continue
        offsets.append(gt.sample_time_s - sample.timestamp_sample_s)
        receipt_gaps_ms.append(abs(gt.bag_time_s - sample.bag_time_s) * 1000.0)
    if not offsets:
        raise RuntimeError("cannot align PX4 odometry with UAV Ground Truth by receipt time")
    offset_s = percentile(offsets, 0.50)
    residuals_ms = [abs(value - offset_s) * 1000.0 for value in offsets]
    return offset_s, {
        "pair_count": len(offsets),
        "offset_s": offset_s,
        "offset_residual_ms": distribution_summary(residuals_ms),
        "receipt_pair_gap_ms": distribution_summary(receipt_gaps_ms),
        "method": (
            "median(Gazebo UAV GT header time - PX4 timestamp_sample), after nearest "
            "bag-receipt pairing; GT is offline-only and never enters online estimation"
        ),
    }


def analyze_episode(
    bag_path: Path,
    episode_name: Optional[str] = None,
    uav_model_name: str = DEFAULT_UAV_MODEL_NAME,
) -> tuple[dict[str, Any], list[GeometryFrame]]:
    data = load_episode(bag_path, uav_model_name)
    deck_truth: list[RigidState] = data["deck_truth"]
    uav_truth: list[RigidState] = data["uav_truth"]
    raw_poses: list[TimedPose] = data["raw_poses"]
    marker_poses_ned: list[TimedPose] = data["marker_poses_ned"]
    shadow_states: list[TimedPose] = data["shadow_states"]
    landing_states: list[tuple[float, str]] = data["landing_states"]
    diagnostic_frames = data["diagnostic_frames"]
    px4_samples: list[Px4Orientation] = data["px4_orientations"]
    uav_truth_receipt: list[TimedOrientation] = data["uav_truth_receipt"]

    if not deck_truth or not uav_truth or not raw_poses or not marker_poses_ned:
        raise RuntimeError("episode does not contain enough deck/UAV/raw/marker pose samples")

    episode = episode_name or bag_path.parent.parent.name
    scenario = episode.rsplit("_s", 1)[0]
    raw_times = [sample.bag_time_s for sample in raw_poses]
    diagnostic_times = [sample.bag_time_s for sample in diagnostic_frames]

    raw_gt_errors: list[float] = []
    marker_errors: list[float] = []
    geometry_frames: list[GeometryFrame] = []
    accepted_raw_count = 0
    unmatched_marker_to_raw = 0
    unmatched_raw_to_diagnostics = 0

    # marker_pose_ned 只在视觉候选通过在线安全检查后发布，因此以它为共同样本集合；
    # 用最近 raw /aruco/pose 的 image header 恢复真正的视觉采样时刻。
    for marker_pose in marker_poses_ned:
        raw = nearest_sample(
            raw_poses, raw_times, marker_pose.bag_time_s, PAIR_TOLERANCE_S
        )
        if raw is None:
            unmatched_marker_to_raw += 1
            continue
        diagnostic = nearest_sample(
            diagnostic_frames,
            diagnostic_times,
            raw.bag_time_s,
            PAIR_TOLERANCE_S,
        )
        if diagnostic is None:
            unmatched_raw_to_diagnostics += 1
            continue
        if diagnostic.pose_source != "PLANAR_BOARD_MULTI":
            continue
        uav = interpolate_truth(uav_truth, raw.sample_time_s)
        deck = interpolate_truth(deck_truth, raw.sample_time_s)
        if uav is None or deck is None:
            continue
        reconstructed = reconstruct_deck_orientation_with_uav_gt(
            uav.orientation, raw.orientation
        )
        raw_normal, _ = normal_and_yaw(reconstructed)
        marker_normal, _ = normal_and_yaw(marker_pose.orientation)
        truth_normal, _ = normal_and_yaw(deck.orientation)
        raw_error = normal_error_deg(raw_normal, truth_normal)
        marker_error = normal_error_deg(marker_normal, truth_normal)
        raw_gt_errors.append(raw_error)
        marker_errors.append(marker_error)
        accepted_raw_count += 1

        x, y, z = raw.position
        if z <= 1.0e-9:
            continue
        geometry_frames.append(
            GeometryFrame(
                episode=episode,
                scenario=scenario,
                true_tilt_deg=deck_tilt_deg(deck.orientation),
                estimated_tilt_deg=deck_tilt_deg(reconstructed),
                raw_gt_attitude_normal_error_deg=raw_error,
                marker_pose_ned_normal_error_deg=marker_error,
                camera_to_board_distance_m=vector_norm(raw.position),
                board_center_x_normalized=x / z,
                board_center_y_normalized=y / z,
                board_center_radius_normalized=math.hypot(x / z, y / z),
                marker_count=int(diagnostic.marker_count),
                reprojection_rmse_px=float(diagnostic.reprojection_rmse_px),
            )
        )

    shadow_errors: list[float] = []
    shadow_tracking_errors: list[float] = []
    for sample in shadow_states:
        deck = interpolate_truth(deck_truth, sample.sample_time_s)
        if deck is None:
            continue
        estimate_normal, _ = normal_and_yaw(sample.orientation)
        truth_normal, _ = normal_and_yaw(deck.orientation)
        error = normal_error_deg(estimate_normal, truth_normal)
        shadow_errors.append(error)
        if latest_value(landing_states, sample.bag_time_s) in TRACKING_STATES:
            shadow_tracking_errors.append(error)

    px4_offset_s, px4_alignment = estimate_px4_to_ros_offset_s(
        px4_samples, uav_truth_receipt
    )
    px4_mapped = [
        TimedOrientation(
            sample.bag_time_s,
            sample.timestamp_sample_s + px4_offset_s,
            sample.orientation,
        )
        for sample in px4_samples
    ]
    px4_mapped.sort(key=lambda sample: sample.sample_time_s)
    px4_sample_times = [sample.sample_time_s for sample in px4_mapped]
    uav_truth_sample_times = [sample.time_s for sample in uav_truth]

    def attitude_stage_summary(states: set[str]) -> dict[str, Any]:
        full_errors: list[float] = []
        normal_errors: list[float] = []
        for sample in px4_mapped:
            if latest_value(landing_states, sample.bag_time_s) not in states:
                continue
            gt = interpolate_truth(uav_truth, sample.sample_time_s)
            if gt is None:
                continue
            full_errors.append(quaternion_angle_error_deg(sample.orientation, gt.orientation))
            normal_errors.append(body_normal_error_deg(sample.orientation, gt.orientation))
        return {
            "px4_body_normal_error_deg": distribution_summary(normal_errors),
            "px4_full_attitude_error_deg": distribution_summary(full_errors),
        }

    tracking_image_full_errors: list[float] = []
    tracking_image_normal_errors: list[float] = []
    tracking_image_px4_gaps_ms: list[float] = []
    tracking_image_gt_gaps_ms: list[float] = []
    for raw in raw_poses:
        if latest_value(landing_states, raw.bag_time_s) not in TRACKING_STATES:
            continue
        px4 = nearest_sample(
            px4_mapped,
            px4_sample_times,
            raw.sample_time_s,
            PAIR_TOLERANCE_S,
        )
        if px4 is None:
            continue
        gt = interpolate_truth(uav_truth, raw.sample_time_s)
        if gt is None:
            continue
        tracking_image_full_errors.append(
            quaternion_angle_error_deg(px4.orientation, gt.orientation)
        )
        tracking_image_normal_errors.append(
            body_normal_error_deg(px4.orientation, gt.orientation)
        )
        tracking_image_px4_gaps_ms.append(
            abs(px4.sample_time_s - raw.sample_time_s) * 1000.0
        )
        insertion = bisect.bisect_left(uav_truth_sample_times, raw.sample_time_s)
        gt_candidates = [
            index
            for index in (insertion - 1, insertion)
            if 0 <= index < len(uav_truth_sample_times)
        ]
        if gt_candidates:
            nearest_gt_index = min(
                gt_candidates,
                key=lambda index: abs(uav_truth_sample_times[index] - raw.sample_time_s),
            )
            tracking_image_gt_gaps_ms.append(
                abs(uav_truth_sample_times[nearest_gt_index] - raw.sample_time_s) * 1000.0
            )

    tilt_bins: dict[str, Any] = {}
    for _, _, label in TILT_BINS:
        frames = [
            frame
            for frame in geometry_frames
            if tilt_bin_label(frame.true_tilt_deg) == label
        ]
        tilt_bins[label] = {
            "count": len(frames),
            "raw_gt_attitude_normal_error_deg": distribution_summary(
                [frame.raw_gt_attitude_normal_error_deg for frame in frames]
            ),
            "marker_pose_ned_normal_error_deg": distribution_summary(
                [frame.marker_pose_ned_normal_error_deg for frame in frames]
            ),
        }

    raw_geometry_errors = [
        frame.raw_gt_attitude_normal_error_deg for frame in geometry_frames
    ]
    result = {
        "episode": episode,
        "bag": str(resolve_bag_uri(bag_path)),
        "sample_counts": {
            "raw_aruco_pose": len(raw_poses),
            "accepted_marker_pose_ned": len(marker_poses_ned),
            "shadow_state": len(shadow_states),
            "accepted_planar_multi_common": accepted_raw_count,
            "geometry_frames": len(geometry_frames),
            "unmatched_marker_to_raw": unmatched_marker_to_raw,
            "unmatched_raw_to_diagnostics": unmatched_raw_to_diagnostics,
            "unpaired_planar_diagnostics": data["unpaired_diagnostics"],
        },
        "normal_error_decomposition": {
            "raw_gt_attitude_normal_rmse_deg": distribution_summary(raw_gt_errors)["rmse"],
            "raw_gt_attitude_normal_p95_deg": distribution_summary(raw_gt_errors)["p95"],
            "marker_pose_ned_normal_rmse_deg": distribution_summary(marker_errors)["rmse"],
            "marker_pose_ned_normal_p95_deg": distribution_summary(marker_errors)["p95"],
            "shadow_normal_rmse_deg": distribution_summary(shadow_errors)["rmse"],
            "shadow_normal_p95_deg": distribution_summary(shadow_errors)["p95"],
            "shadow_tracking_normal_rmse_deg": distribution_summary(shadow_tracking_errors)["rmse"],
            "shadow_tracking_normal_p95_deg": distribution_summary(shadow_tracking_errors)["p95"],
            "raw_gt_attitude_distribution_deg": distribution_summary(raw_gt_errors),
            "marker_pose_ned_distribution_deg": distribution_summary(marker_errors),
            "shadow_distribution_deg": distribution_summary(shadow_errors),
        },
        "planar_geometry": {
            "true_deck_tilt_deg": distribution_summary(
                [frame.true_tilt_deg for frame in geometry_frames]
            ),
            "estimated_deck_tilt_deg": distribution_summary(
                [frame.estimated_tilt_deg for frame in geometry_frames]
            ),
            "camera_to_board_distance_m": distribution_summary(
                [frame.camera_to_board_distance_m for frame in geometry_frames]
            ),
            "board_center_x_normalized": distribution_summary(
                [frame.board_center_x_normalized for frame in geometry_frames]
            ),
            "board_center_y_normalized": distribution_summary(
                [frame.board_center_y_normalized for frame in geometry_frames]
            ),
            "board_center_radius_normalized": distribution_summary(
                [frame.board_center_radius_normalized for frame in geometry_frames]
            ),
            "marker_count": distribution_summary(
                [float(frame.marker_count) for frame in geometry_frames]
            ),
            "reprojection_rmse_px": distribution_summary(
                [frame.reprojection_rmse_px for frame in geometry_frames]
            ),
            "normal_error_correlations": {
                "true_deck_tilt_deg": pearson_correlation(
                    [frame.true_tilt_deg for frame in geometry_frames], raw_geometry_errors
                ),
                "camera_to_board_distance_m": pearson_correlation(
                    [frame.camera_to_board_distance_m for frame in geometry_frames],
                    raw_geometry_errors,
                ),
                "board_center_radius_normalized": pearson_correlation(
                    [frame.board_center_radius_normalized for frame in geometry_frames],
                    raw_geometry_errors,
                ),
                "marker_count": pearson_correlation(
                    [float(frame.marker_count) for frame in geometry_frames],
                    raw_geometry_errors,
                ),
                "reprojection_rmse_px": pearson_correlation(
                    [frame.reprojection_rmse_px for frame in geometry_frames],
                    raw_geometry_errors,
                ),
            },
            "tilt_bins": tilt_bins,
            "image_center_note": (
                "Bag 未记录 Board corners/corner centroid；因此这里用 raw T_camera_deck 的 "
                "x/z、y/z 表示 pose-implied normalized image center。该量可排除明显离轴，"
                "但不能回溯真实检测角点质心。"
            ),
        },
        "px4_attitude_timing": {
            "pre_acquire": attitude_stage_summary(PRE_ACQUIRE_STATES),
            "tracking": attitude_stage_summary(set(TRACKING_STATES)),
            "tracking_image_time": {
                "px4_body_normal_error_deg": distribution_summary(
                    tracking_image_normal_errors
                ),
                "px4_full_attitude_error_deg": distribution_summary(
                    tracking_image_full_errors
                ),
                "nearest_px4_sample_gap_ms": distribution_summary(
                    tracking_image_px4_gaps_ms
                ),
                "nearest_gt_sample_gap_ms": distribution_summary(
                    tracking_image_gt_gaps_ms
                ),
            },
            "px4_to_ros_alignment": px4_alignment,
        },
    }
    return result, geometry_frames


def aggregate_results(
    episode_results: Sequence[dict[str, Any]],
    geometry_frames: Sequence[GeometryFrame],
) -> dict[str, Any]:
    def decomposition_values(key: str) -> list[float]:
        values: list[float] = []
        for result in episode_results:
            value = result["normal_error_decomposition"].get(key)
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                values.append(float(value))
        return values

    tilt_bins: dict[str, Any] = {}
    for _, _, label in TILT_BINS:
        frames = [
            frame for frame in geometry_frames if tilt_bin_label(frame.true_tilt_deg) == label
        ]
        tilt_bins[label] = {
            "count": len(frames),
            "raw_gt_attitude_normal_error_deg": distribution_summary(
                [frame.raw_gt_attitude_normal_error_deg for frame in frames]
            ),
            "marker_pose_ned_normal_error_deg": distribution_summary(
                [frame.marker_pose_ned_normal_error_deg for frame in frames]
            ),
        }

    raw_errors = [frame.raw_gt_attitude_normal_error_deg for frame in geometry_frames]
    scenario_groups: dict[str, list[dict[str, Any]]] = {}
    for result in episode_results:
        scenario = str(result["episode"]).rsplit("_s", 1)[0]
        scenario_groups.setdefault(scenario, []).append(result)

    scenario_summary: dict[str, Any] = {}
    scenario_tilt_bins: dict[str, Any] = {}
    scenario_geometry: dict[str, Any] = {}
    for scenario, results in sorted(scenario_groups.items()):
        scenario_summary[scenario] = {
            key: distribution_summary(
                [
                    float(result["normal_error_decomposition"][key])
                    for result in results
                    if isinstance(result["normal_error_decomposition"].get(key), (int, float))
                ]
            )
            for key in (
                "raw_gt_attitude_normal_rmse_deg",
                "raw_gt_attitude_normal_p95_deg",
                "marker_pose_ned_normal_rmse_deg",
                "marker_pose_ned_normal_p95_deg",
                "shadow_normal_rmse_deg",
                "shadow_normal_p95_deg",
            )
        }
        frames = [frame for frame in geometry_frames if frame.scenario == scenario]
        scenario_tilt_bins[scenario] = {}
        for lower, upper, label in TILT_BINS:
            binned = [
                frame.raw_gt_attitude_normal_error_deg
                for frame in frames
                if lower <= frame.true_tilt_deg < upper
            ]
            scenario_tilt_bins[scenario][label] = distribution_summary(binned)
        scenario_geometry[scenario] = {
            "camera_to_board_distance_m": distribution_summary(
                [frame.camera_to_board_distance_m for frame in frames]
            ),
            "board_center_radius_normalized": distribution_summary(
                [frame.board_center_radius_normalized for frame in frames]
            ),
            "marker_count": distribution_summary(
                [float(frame.marker_count) for frame in frames]
            ),
            "reprojection_rmse_px": distribution_summary(
                [frame.reprojection_rmse_px for frame in frames]
            ),
        }

    return {
        "episode_count": len(episode_results),
        "scenario_summary_of_episode_metrics": scenario_summary,
        "scenario_geometry": scenario_geometry,
        "scenario_tilt_bins_raw_gt_attitude": scenario_tilt_bins,
        "rollpitch_only_tilt_bins_raw_gt_attitude": scenario_tilt_bins.get(
            "rollpitch", {}
        ),
        "tilt_bins": tilt_bins,
        "geometry_frame_correlations": {
            "true_deck_tilt_deg": pearson_correlation(
                [frame.true_tilt_deg for frame in geometry_frames], raw_errors
            ),
            "camera_to_board_distance_m": pearson_correlation(
                [frame.camera_to_board_distance_m for frame in geometry_frames], raw_errors
            ),
            "board_center_radius_normalized": pearson_correlation(
                [frame.board_center_radius_normalized for frame in geometry_frames], raw_errors
            ),
            "marker_count": pearson_correlation(
                [float(frame.marker_count) for frame in geometry_frames], raw_errors
            ),
            "reprojection_rmse_px": pearson_correlation(
                [frame.reprojection_rmse_px for frame in geometry_frames], raw_errors
            ),
        },
        "episode_metric_ranges": {
            key: distribution_summary(decomposition_values(key))
            for key in (
                "raw_gt_attitude_normal_rmse_deg",
                "raw_gt_attitude_normal_p95_deg",
                "marker_pose_ned_normal_rmse_deg",
                "marker_pose_ned_normal_p95_deg",
                "shadow_normal_rmse_deg",
                "shadow_normal_p95_deg",
            )
        },
    }


def discover_matrix_bags(matrix_dir: Path) -> list[tuple[str, Path]]:
    episodes: list[tuple[str, Path]] = []
    for episode_dir in sorted(path for path in matrix_dir.iterdir() if path.is_dir()):
        bag_dir = episode_dir / "bag"
        if (bag_dir / "metadata.yaml").is_file():
            episodes.append((episode_dir.name, bag_dir))
    return episodes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bag", type=Path, help="单轮 rosbag 目录或 db3 文件")
    source.add_argument("--matrix-dir", type=Path, help="包含正式 episode 子目录的矩阵目录")
    parser.add_argument("--uav-model-name", default=DEFAULT_UAV_MODEL_NAME)
    parser.add_argument("--output", type=Path, help="可选：同时写入严格 JSON 诊断结果")
    parser.add_argument("--compact", action="store_true", help="紧凑 JSON 输出")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.bag is not None:
        result, geometry = analyze_episode(args.bag, uav_model_name=args.uav_model_name)
        output = {
            "episodes": [result],
            "aggregate": aggregate_results([result], geometry),
        }
    else:
        episodes = discover_matrix_bags(args.matrix_dir)
        if not episodes:
            raise RuntimeError(f"no episode bags found below {args.matrix_dir}")
        results: list[dict[str, Any]] = []
        all_geometry: list[GeometryFrame] = []
        for episode_name, bag in episodes:
            result, geometry = analyze_episode(
                bag, episode_name=episode_name, uav_model_name=args.uav_model_name
            )
            results.append(result)
            all_geometry.extend(geometry)
        output = {
            "data_source": str(args.matrix_dir),
            "episodes": results,
            "aggregate": aggregate_results(results, all_geometry),
            "safety_boundary": (
                "Ground Truth is consumed only after online pose/state messages already exist; "
                "the analyzer does not publish detector/estimator/controller inputs."
            ),
        }
    encoded = json.dumps(
        output, ensure_ascii=False, allow_nan=False, indent=None if args.compact else 2
    )
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
