#!/usr/bin/env python3
"""离线评测 deck-geometry shadow 倾斜甲板 shadow 几何诊断。

本脚本只读取 rosbag。Ground Truth 仅用于离线法向误差对照，不会进入控制器。
"""

from __future__ import annotations

import argparse
import bisect
import collections
import contextlib
import io
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

from evaluate_horizontal_tracking import GeodeticPosition, world_enu_to_local_ned
from evaluate_final_descent_touchdown import (
    ARM_DISARM_COMMAND,
    NAV_LAND_COMMAND,
    load_ros_modules,
    resolve_bag_uri,
    state_sequence,
)
from evaluate_heave_touchdown import (
    contact_transition_counts,
    transition_entry_count,
)

STATUS_TOPIC = "/landing/deck_plane/status"
NORMAL_TOPIC = "/landing/deck_plane/upward_normal_ned"
BODY_CLEARANCE_TOPIC = "/landing/deck_plane/body_clearance"
SKID_CLEARANCES_TOPIC = "/landing/deck_plane/skid_clearances"
MIN_CLEARANCE_TOPIC = "/landing/deck_plane/minimum_skid_clearance"
MAX_CLEARANCE_TOPIC = "/landing/deck_plane/maximum_skid_clearance"
SPREAD_TOPIC = "/landing/deck_plane/clearance_spread"
FIRST_CONTACT_TOPIC = "/landing/deck_plane/first_contact_point_index"
NORMAL_VELOCITY_TOPIC = "/landing/deck_plane/normal_relative_velocity"
SKID_VELOCITIES_TOPIC = "/landing/deck_plane/skid_normal_relative_velocities"
TANGENTIAL_POSITION_TOPIC = "/landing/deck_plane/tangential_position_error"
TANGENTIAL_VELOCITY_TOPIC = "/landing/deck_plane/tangential_relative_velocity"
ACTIVE_MARKER_TOPIC = "/landing/active_marker_id"
MARKER_POSE_NED_TOPIC = "/landing/marker_pose_ned"
RELATIVE_HEIGHT_TOPIC = "/landing/relative_height"
RAW_RELATIVE_HEIGHT_TOPIC = "/landing/raw_relative_height"
HEIGHT_REFERENCE_TOPIC = "/landing/relative_height_reference"
ARUCO_VISIBLE_TOPIC = "/aruco/visible"
ESTIMATED_ATTITUDE_TOPIC = "/landing/estimated_deck_attitude"
NORMAL_RATE_TOPIC = "/landing/deck_plane/normal_rate_degps"
MARKER_SWITCH_JUMP_TOPIC = "/landing/deck_plane/marker_switch_normal_jump_deg"
MARKER_NORMALS_TOPIC = "/landing/deck_plane/marker_normals_by_id"
MARKER_NORMAL_VALID_MASK_TOPIC = "/landing/deck_plane/marker_normal_valid_mask"
GROUND_TRUTH_TOPIC = "/simulation/deck/ground_truth"
STATE_TOPIC = "/landing/state"
TARGET_TOPIC = "/landing/target_pose"
TRAJECTORY_TOPIC = "/fmu/in/trajectory_setpoint"
VEHICLE_COMMAND_TOPIC = "/fmu/in/vehicle_command"
LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position_v1"
VEHICLE_ODOMETRY_TOPIC = "/fmu/out/vehicle_odometry"
TOUCHDOWN_STATUS_TOPIC = "/landing/touchdown_status"
TOUCHDOWN_EVIDENCE_TOPIC = "/landing/touchdown_evidence"
TOUCHDOWN_DURATION_TOPIC = "/landing/touchdown_candidate_duration"
TOUCHDOWN_CONFIRMED_TOPIC = "/landing/touchdown_confirmed"
HOLD_REFERENCE_TOPIC = "/landing/touchdown_hold_relative_height_reference"
HOLD_TARGET_TOPIC = "/landing/touchdown_hold_vertical_target"
HOLD_MODE_TOPIC = "/landing/touchdown_hold_mode"
HOLD_REASON_TOPIC = "/landing/touchdown_hold_reason"
LAND_DETECTED_TOPIC = "/fmu/out/vehicle_land_detected"
TERMINAL_ENABLED_TOPIC = "/landing/terminal_stabilization/enabled"
TERMINAL_MODE_TOPIC = "/landing/terminal_stabilization/mode"
TERMINAL_REASON_TOPIC = "/landing/terminal_stabilization/reason"
TERMINAL_DESIRED_NORMAL_TOPIC = "/landing/terminal_stabilization/desired_normal"
TERMINAL_DESIRED_ATTITUDE_TOPIC = "/landing/terminal_stabilization/desired_roll_pitch"
TERMINAL_ACTUAL_ATTITUDE_TOPIC = "/landing/terminal_stabilization/actual_roll_pitch"
TERMINAL_ATTITUDE_ERROR_TOPIC = "/landing/terminal_stabilization/attitude_error"
TERMINAL_ACCELERATION_BIAS_TOPIC = "/landing/terminal_stabilization/acceleration_bias_ned"
TERMINAL_COMBINED_ACCELERATION_TOPIC = (
    "/landing/terminal_stabilization/combined_acceleration_ff_ned"
)
TERMINAL_CONTACT_ANCHOR_TOPIC = "/landing/terminal_stabilization/contact_anchor"
TERMINAL_COMPLIANT_TARGET_TOPIC = "/landing/terminal_stabilization/compliant_target"
TERMINAL_DIVERGENCE_TOPIC = "/landing/terminal_stabilization/divergence_status"

# Scenario names and YAML RPY follow Gazebo world ENU. The public geometry
# diagnostics use local NED. Therefore Gazebo roll maps to NED pitch, while
# Gazebo pitch maps to NED roll (same sign for the fixed single-axis cases).
SCENARIO_ATTITUDES_DEG = {
    "static": (0.0, 0.0),
    "constant02": (0.0, 0.0),
    "tilt_roll_pos_2deg": (0.0, 2.0),
    "tilt_roll_neg_2deg": (0.0, -2.0),
    "tilt_pitch_pos_2deg": (2.0, 0.0),
    "tilt_pitch_neg_2deg": (-2.0, 0.0),
}
FORBIDDEN_SAFE_ALTITUDE_STATES = {
    "DESCEND",
    "TEST_HEIGHT_HOLD",
    "FINAL_DESCENT",
    "TOUCHDOWN_CANDIDATE_HOLD",
    "TOUCHDOWN_HOLD",
}
FIXED_TILT_SAFE_DESCENT_FORBIDDEN_STATES = {
    "FINAL_DESCENT",
    "TOUCHDOWN_CANDIDATE_HOLD",
    "TOUCHDOWN_HOLD",
    "RECOVER_CLIMB",
    "RECOVER_TO_GNSS",
    "ABORT",
    "DONE",
}
FIXED_TILT_SAFE_DESCENT_MIN_TEST_HEIGHT_HOLD_S = 8.0
FIXED_TILT_SAFE_DESCENT_MIN_GROUND_TRUTH_SKID_CLEARANCE_M = 0.09

REQUIRED_SHADOW_TOPICS = {
    STATUS_TOPIC,
    NORMAL_TOPIC,
    BODY_CLEARANCE_TOPIC,
    SKID_CLEARANCES_TOPIC,
    MIN_CLEARANCE_TOPIC,
    MAX_CLEARANCE_TOPIC,
    SPREAD_TOPIC,
    FIRST_CONTACT_TOPIC,
    NORMAL_VELOCITY_TOPIC,
    TANGENTIAL_POSITION_TOPIC,
    TANGENTIAL_VELOCITY_TOPIC,
    ACTIVE_MARKER_TOPIC,
    ESTIMATED_ATTITUDE_TOPIC,
    NORMAL_RATE_TOPIC,
    MARKER_SWITCH_JUMP_TOPIC,
    MARKER_NORMALS_TOPIC,
    MARKER_NORMAL_VALID_MASK_TOPIC,
}
FIXED_TILT_SAFE_ALTITUDE_REQUIRED_TOPICS = REQUIRED_SHADOW_TOPICS | {
    GROUND_TRUTH_TOPIC,
    STATE_TOPIC,
    TARGET_TOPIC,
    TRAJECTORY_TOPIC,
    VEHICLE_COMMAND_TOPIC,
    LOCAL_POSITION_TOPIC,
    VEHICLE_ODOMETRY_TOPIC,
}
FIXED_TILT_SAFE_DESCENT_REQUIRED_TOPICS = FIXED_TILT_SAFE_ALTITUDE_REQUIRED_TOPICS | {
    RELATIVE_HEIGHT_TOPIC,
    HEIGHT_REFERENCE_TOPIC,
    ARUCO_VISIBLE_TOPIC,
}
TILTED_DECK_TOUCHDOWN_REQUIRED_TOPICS = FIXED_TILT_SAFE_DESCENT_REQUIRED_TOPICS | {
    TOUCHDOWN_STATUS_TOPIC,
    TOUCHDOWN_EVIDENCE_TOPIC,
    TOUCHDOWN_DURATION_TOPIC,
    TOUCHDOWN_CONFIRMED_TOPIC,
    HOLD_REFERENCE_TOPIC,
    HOLD_TARGET_TOPIC,
    HOLD_MODE_TOPIC,
    HOLD_REASON_TOPIC,
    LAND_DETECTED_TOPIC,
}
TERMINAL_STABILIZATION_REQUIRED_TOPICS = {
    TERMINAL_ENABLED_TOPIC,
    TERMINAL_MODE_TOPIC,
    TERMINAL_REASON_TOPIC,
    TERMINAL_DESIRED_NORMAL_TOPIC,
    TERMINAL_DESIRED_ATTITUDE_TOPIC,
    TERMINAL_ACTUAL_ATTITUDE_TOPIC,
    TERMINAL_ATTITUDE_ERROR_TOPIC,
    TERMINAL_ACCELERATION_BIAS_TOPIC,
    TERMINAL_COMBINED_ACCELERATION_TOPIC,
    TERMINAL_CONTACT_ANCHOR_TOPIC,
    TERMINAL_COMPLIANT_TARGET_TOPIC,
    TERMINAL_DIVERGENCE_TOPIC,
}


@dataclass(frozen=True)
class TimedSample:
    """带 rosbag 时间戳的有限数值向量。"""

    time_s: float
    values: tuple[float, ...]


@dataclass(frozen=True)
class MarkerHistorySample:
    """Historical raw Marker normal with synchronized ID, truth, and height."""

    time_s: float
    marker_id: int
    raw_normal_ned: tuple[float, float, float]
    truth_normal_ned: tuple[float, float, float]
    relative_height_m: float


def finite_values(values: Iterable[float]) -> bool:
    """仅当全部数值有限时返回真。"""

    return all(math.isfinite(float(value)) for value in values)


def normalize_vector(vector: Sequence[float]) -> tuple[float, float, float]:
    """归一化三维向量；非法或近零输入抛出 ValueError。"""

    if len(vector) != 3 or not finite_values(vector):
        raise ValueError("normal must contain three finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in vector))
    if norm <= 1.0e-12:
        raise ValueError("normal norm is too small")
    return tuple(float(value) / norm for value in vector)  # type: ignore[return-value]


def expected_attitude_deg(scenario: str) -> tuple[float, float]:
    """返回 fixed-tilt safe altitude 场景冻结的 `(roll, pitch)` 真值，单位 degree。"""

    if scenario not in SCENARIO_ATTITUDES_DEG:
        raise ValueError(f"unsupported fixed-tilt safe altitude scenario: {scenario}")
    return SCENARIO_ATTITUDES_DEG[scenario]


def calibration_axis_from_truth(truth_roll_pitch_deg: Sequence[float]) -> str:
    """Choose the signed calibration axis from Ground Truth, never the scenario name."""

    if len(truth_roll_pitch_deg) < 2 or not finite_values(truth_roll_pitch_deg[:2]):
        raise ValueError("truth roll/pitch must contain two finite values")
    roll_deg = float(truth_roll_pitch_deg[0])
    pitch_deg = float(truth_roll_pitch_deg[1])
    if max(abs(roll_deg), abs(pitch_deg)) < 0.5:
        return "tilt"
    return "roll" if abs(roll_deg) >= abs(pitch_deg) else "pitch"


def expected_upward_normal_ned(scenario: str) -> tuple[float, float, float]:
    """按 tilted-deck 坐标约定返回场景理论向上法向。"""

    roll_deg, pitch_deg = expected_attitude_deg(scenario)
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    return normalize_vector(
        (
            -math.sin(pitch) * math.cos(roll),
            math.sin(roll),
            -math.cos(pitch) * math.cos(roll),
        )
    )


def normal_to_attitude_deg(normal: Sequence[float]) -> tuple[float, float, float]:
    """把 local NED 向上法向转换为 yaw 无关 roll/pitch/tilt，单位 degree。"""

    nx, ny, nz = normalize_vector(normal)
    roll = math.asin(max(-1.0, min(1.0, ny)))
    pitch = math.atan2(-nx, -nz)
    tilt = math.acos(max(-1.0, min(1.0, -nz)))
    return math.degrees(roll), math.degrees(pitch), math.degrees(tilt)


def normal_angle_error_deg(
    estimated_normal: Sequence[float], truth_normal: Sequence[float]
) -> float:
    """计算两个无方向歧义、指向甲板上方法向之间的夹角，单位 degree。"""

    estimated = normalize_vector(estimated_normal)
    truth = normalize_vector(truth_normal)
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(estimated, truth))))
    return math.degrees(math.acos(dot))


def contact_point_index(clearances_m: Sequence[float]) -> int:
    """返回四端点最小间隙的确定性首索引。"""

    if len(clearances_m) != 4 or not finite_values(clearances_m):
        raise ValueError("four finite skid clearances are required")
    return min(range(4), key=lambda index: float(clearances_m[index]))


def clearance_spread(clearances_m: Sequence[float]) -> float:
    """返回四端点最大与最小间隙之差。"""

    if len(clearances_m) != 4 or not finite_values(clearances_m):
        raise ValueError("four finite skid clearances are required")
    return max(float(value) for value in clearances_m) - min(
        float(value) for value in clearances_m
    )


def fixed_tilt_safe_descent_clearance_threshold_definition() -> dict[str, float]:
    """返回 fixed-tilt safe descent 在真实实验前冻结的最小滑橇间隙推导。"""

    tilt_rad = math.radians(2.0)
    body_gap_m = 0.50 * math.cos(tilt_rad)
    vertical_leg_projection_m = 0.227 * math.cos(tilt_rad)
    roll_profile_min_m = body_gap_m - vertical_leg_projection_m - 0.125 * math.sin(tilt_rad)
    pitch_profile_min_m = body_gap_m - vertical_leg_projection_m - 0.132 * math.sin(tilt_rad)
    theoretical_min_m = min(roll_profile_min_m, pitch_profile_min_m)
    fixed_tilt_safe_altitude_worst_error_m = 0.16652911274062454
    frozen_threshold_m = FIXED_TILT_SAFE_DESCENT_MIN_GROUND_TRUTH_SKID_CLEARANCE_M
    return {
        "target_body_height_m": 0.50,
        "contact_point_down_offset_m": 0.227,
        "tilt_deg": 2.0,
        "roll_profile_theoretical_minimum_skid_clearance_m": roll_profile_min_m,
        "pitch_profile_theoretical_minimum_skid_clearance_m": pitch_profile_min_m,
        "worst_theoretical_minimum_skid_clearance_m": theoretical_min_m,
        "fixed_tilt_safe_altitude_worst_absolute_skid_geometry_error_m": fixed_tilt_safe_altitude_worst_error_m,
        "frozen_minimum_ground_truth_skid_clearance_m": frozen_threshold_m,
        "reserved_total_error_margin_m": theoretical_min_m - frozen_threshold_m,
        "additional_unallocated_margin_m": (
            theoretical_min_m - frozen_threshold_m - fixed_tilt_safe_altitude_worst_error_m
        ),
    }


def fixed_tilt_safe_descent_gate(metrics: dict[str, Any]) -> dict[str, Any]:
    """按冻结阈值判定 fixed-tilt safe descent 安全下降，不把观察量冒充 PASS 门。"""

    thresholds = {
        "test_height_hold_duration_min_s": FIXED_TILT_SAFE_DESCENT_MIN_TEST_HEIGHT_HOLD_S,
        "minimum_ground_truth_skid_clearance_m": FIXED_TILT_SAFE_DESCENT_MIN_GROUND_TRUTH_SKID_CLEARANCE_M,
        "horizontal_error_rmse_max_m": 0.08,
        "horizontal_error_max_m": 0.15,
        "normal_rmse_max_deg": 1.0,
        "normal_p95_max_deg": 1.5,
        "sign_accuracy_min": 0.95,
        "marker_switch_jump_max_deg": 1.0,
    }
    states = list(metrics.get("state_sequence", []))
    failed_checks: list[str] = []
    if "DESCEND" not in states:
        failed_checks.append("missing_descend")
    if "TEST_HEIGHT_HOLD" not in states:
        failed_checks.append("missing_test_height_hold")
    for state in states:
        if state in FIXED_TILT_SAFE_DESCENT_FORBIDDEN_STATES:
            if state == "FINAL_DESCENT":
                check = "final_descent"
            elif state.startswith("TOUCHDOWN"):
                check = "touchdown"
            else:
                check = state.lower()
            if check not in failed_checks:
                failed_checks.append(check)
    numeric_checks = (
        ("test_height_hold_duration", "test_height_hold_duration_s", thresholds["test_height_hold_duration_min_s"], lambda value, limit: value >= limit),
        ("clearance", "ground_truth_minimum_skid_clearance_m", thresholds["minimum_ground_truth_skid_clearance_m"], lambda value, limit: value >= limit),
        ("horizontal_rmse", "horizontal_error_rmse_m", thresholds["horizontal_error_rmse_max_m"], lambda value, limit: value <= limit),
        ("horizontal_max", "horizontal_error_max_m", thresholds["horizontal_error_max_m"], lambda value, limit: value <= limit),
        ("normal_rmse", "normal_rmse_deg", thresholds["normal_rmse_max_deg"], lambda value, limit: value <= limit),
        ("normal_p95", "normal_p95_deg", thresholds["normal_p95_max_deg"], lambda value, limit: value <= limit),
        ("sign_accuracy", "sign_accuracy", thresholds["sign_accuracy_min"], lambda value, limit: value >= limit),
        ("marker_jump", "marker_switch_jump_max_deg", thresholds["marker_switch_jump_max_deg"], lambda value, limit: value <= limit),
    )
    for check, key, limit, predicate in numeric_checks:
        value = metrics.get(key)
        if value is None or not math.isfinite(float(value)) or not predicate(float(value), float(limit)):
            failed_checks.append(check)
    zero_count_checks = (
        ("contact", "ground_truth_contact_count"),
        ("penetration", "ground_truth_penetration_count"),
        ("time_sync", "time_sync_failure_count"),
        ("nan_inf", "nan_inf_count"),
        ("nav_land", "nav_land_command_count"),
        ("disarm", "disarm_command_count"),
    )
    for check, key in zero_count_checks:
        if int(metrics.get(key, 0)) != 0:
            failed_checks.append(check)
    return {
        "passed": not failed_checks,
        "failed_checks": failed_checks,
        "thresholds": thresholds,
        "clearance_threshold_definition": fixed_tilt_safe_descent_clearance_threshold_definition(),
    }


def tilted_deck_touchdown_gate(metrics: dict[str, Any]) -> dict[str, Any]:
    """按 fixed-tilt touchdown 冻结门判定固定正 2° 甲板真实触地。

    `0.05 m/s` 触地法向速度仅作为目标值记录，`0.12 m/s` 才是硬安全门。
    姿态发散使用确定性窗口差：接触后末端 2 s 的最大绝对 roll/pitch，减去
    接触后起始 2 s 的最大绝对 roll/pitch；该增量不得超过 2°。
    """

    thresholds = {
        "candidate_duration_min_s": 0.50,
        "hold_duration_min_s": 10.0,
        "horizontal_error_rmse_max_m": 0.08,
        "horizontal_error_max_m": 0.15,
        "relative_horizontal_velocity_rmse_max_mps": 0.10,
        "tangential_velocity_rmse_max_mps": 0.10,
        "touchdown_h_min_min_m": -0.05,
        "touchdown_h_min_max_m": 0.03,
        "touchdown_h_max_max_m": 0.05,
        "touchdown_clearance_spread_max_m": 0.03,
        "touchdown_normal_speed_max_mps": 0.12,
        "touchdown_normal_speed_target_mps": 0.05,
        "post_touchdown_tangential_slip_max_m": 0.10,
        "hold_tangential_velocity_p95_max_mps": 0.05,
        "post_touchdown_roll_max_abs_deg": 10.0,
        "post_touchdown_pitch_max_abs_deg": 10.0,
        "attitude_divergence_delta_max_deg": 2.0,
    }
    states = list(metrics.get("state_sequence", []))
    failed_checks: list[str] = []
    required_states = (
        ("missing_final_descent", "FINAL_DESCENT"),
        ("missing_candidate", "TOUCHDOWN_CANDIDATE_HOLD"),
        ("missing_touchdown_hold", "TOUCHDOWN_HOLD"),
    )
    for check, state in required_states:
        if state not in states:
            failed_checks.append(check)

    numeric_checks = (
        ("candidate_duration", "candidate_duration_s", thresholds["candidate_duration_min_s"], lambda value, limit: value >= limit),
        ("hold_duration", "hold_duration_s", thresholds["hold_duration_min_s"], lambda value, limit: value >= limit),
        ("horizontal_rmse", "horizontal_error_rmse_m", thresholds["horizontal_error_rmse_max_m"], lambda value, limit: value <= limit),
        ("horizontal_max", "horizontal_error_max_m", thresholds["horizontal_error_max_m"], lambda value, limit: value <= limit),
        ("relative_horizontal_velocity_rmse", "relative_horizontal_velocity_rmse_mps", thresholds["relative_horizontal_velocity_rmse_max_mps"], lambda value, limit: value <= limit),
        ("tangential_velocity_rmse", "tangential_velocity_rmse_mps", thresholds["tangential_velocity_rmse_max_mps"], lambda value, limit: value <= limit),
        ("touchdown_h_max", "touchdown_h_max_m", thresholds["touchdown_h_max_max_m"], lambda value, limit: value <= limit),
        ("touchdown_clearance_spread", "touchdown_clearance_spread_m", thresholds["touchdown_clearance_spread_max_m"], lambda value, limit: value <= limit),
        ("touchdown_normal_speed", "touchdown_normal_relative_velocity_mps", thresholds["touchdown_normal_speed_max_mps"], lambda value, limit: abs(value) <= limit),
        ("post_touchdown_slip", "post_touchdown_tangential_slip_m", thresholds["post_touchdown_tangential_slip_max_m"], lambda value, limit: value <= limit),
        ("hold_tangential_velocity_p95", "hold_tangential_velocity_p95_mps", thresholds["hold_tangential_velocity_p95_max_mps"], lambda value, limit: value <= limit),
        ("post_touchdown_roll", "post_touchdown_roll_max_abs_deg", thresholds["post_touchdown_roll_max_abs_deg"], lambda value, limit: value <= limit),
        ("post_touchdown_pitch", "post_touchdown_pitch_max_abs_deg", thresholds["post_touchdown_pitch_max_abs_deg"], lambda value, limit: value <= limit),
        ("attitude_divergence", "attitude_divergence_delta_deg", thresholds["attitude_divergence_delta_max_deg"], lambda value, limit: value <= limit),
    )
    nonfinite_required = False
    for check, key, limit, predicate in numeric_checks:
        value = metrics.get(key)
        if value is None or not math.isfinite(float(value)):
            nonfinite_required = True
            if check not in failed_checks:
                failed_checks.append(check)
            continue
        if not predicate(float(value), float(limit)):
            failed_checks.append(check)

    h_min = metrics.get("touchdown_h_min_m")
    if h_min is None or not math.isfinite(float(h_min)):
        nonfinite_required = True
        failed_checks.append("touchdown_h_min")
    elif not (
        thresholds["touchdown_h_min_min_m"]
        <= float(h_min)
        <= thresholds["touchdown_h_min_max_m"]
    ):
        failed_checks.append("touchdown_h_min")

    zero_count_checks = (
        ("detach", "detach_count"),
        ("secondary_contact", "secondary_contact_count"),
        ("recovery", "recovery_count"),
        ("time_sync", "time_sync_failure_count"),
        ("nan_inf", "nan_inf_count"),
        ("nav_land", "nav_land_command_count"),
        ("disarm", "disarm_command_count"),
    )
    for check, key in zero_count_checks:
        try:
            value = int(metrics.get(key, 0))
        except (TypeError, ValueError, OverflowError):
            value = 1
            nonfinite_required = True
        if value != 0 and check not in failed_checks:
            failed_checks.append(check)
    if nonfinite_required and "nan_inf" not in failed_checks:
        failed_checks.append("nan_inf")

    normal_speed = metrics.get("touchdown_normal_relative_velocity_mps")
    target_achieved = bool(
        normal_speed is not None
        and math.isfinite(float(normal_speed))
        and abs(float(normal_speed)) <= thresholds["touchdown_normal_speed_target_mps"]
    )
    return {
        "passed": not failed_checks,
        "failed_checks": failed_checks,
        "thresholds": thresholds,
        "target_touchdown_speed_achieved": target_achieved,
        "target_touchdown_speed_status": (
            "TARGET_ACHIEVED" if target_achieved else "TARGET_NOT_ACHIEVED_WITHIN_HARD_SAFETY_GATE"
        ),
    }


def interpolate_sample(
    samples: Sequence[TimedSample], time_s: float, max_gap_s: float
) -> Optional[tuple[float, ...]]:
    """在线性插值两侧均不超过 max_gap_s 时返回向量，否则返回 None。"""

    if not math.isfinite(time_s) or not math.isfinite(max_gap_s) or max_gap_s < 0.0:
        raise ValueError("time and max_gap_s must be finite; max_gap_s must be non-negative")
    if not samples:
        return None
    times = [sample.time_s for sample in samples]
    index = bisect.bisect_left(times, time_s)
    if index < len(samples) and abs(samples[index].time_s - time_s) <= 1.0e-12:
        return samples[index].values
    if index == 0:
        sample = samples[0]
        return sample.values if sample.time_s - time_s <= max_gap_s else None
    if index == len(samples):
        sample = samples[-1]
        return sample.values if time_s - sample.time_s <= max_gap_s else None

    lower = samples[index - 1]
    upper = samples[index]
    if time_s - lower.time_s > max_gap_s or upper.time_s - time_s > max_gap_s:
        return None
    if len(lower.values) != len(upper.values) or upper.time_s <= lower.time_s:
        return None
    alpha = (time_s - lower.time_s) / (upper.time_s - lower.time_s)
    values = tuple(
        lower_value + alpha * (upper_value - lower_value)
        for lower_value, upper_value in zip(lower.values, upper.values)
    )
    return values if finite_values(values) else None


def nonfinite_times_at_or_after(
    times_s: Sequence[float], start_time_s: Optional[float]
) -> list[float]:
    """Return non-finite sample timestamps that are safety-relevant after tracking starts."""

    if start_time_s is None:
        return [float(time_s) for time_s in times_s]
    return [float(time_s) for time_s in times_s if time_s >= start_time_s]


def timed_samples_between(
    samples: Sequence[TimedSample],
    start_s: Optional[float],
    end_s: Optional[float],
) -> list[TimedSample]:
    """返回闭区间内样本；用于把自动停机尾帧与冻结验收窗口隔离。"""

    if start_s is None or end_s is None or end_s < start_s:
        return []
    return [sample for sample in samples if start_s <= sample.time_s <= end_s]


def nearest_sample(
    samples: Sequence[TimedSample], time_s: float, max_gap_s: float
) -> Optional[tuple[float, ...]]:
    """返回时间上最近且不超过 max_gap_s 的样本，避免顺序发布值被跨周期插值。"""

    if not math.isfinite(time_s) or not math.isfinite(max_gap_s) or max_gap_s < 0.0:
        raise ValueError("time and max_gap_s must be finite; max_gap_s must be non-negative")
    if not samples:
        return None
    times = [sample.time_s for sample in samples]
    index = bisect.bisect_left(times, time_s)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    closest = min(candidates, key=lambda sample: abs(sample.time_s - time_s))
    return closest.values if abs(closest.time_s - time_s) <= max_gap_s else None


def topic_has_messages(
    topic: str,
    numeric: dict[str, list[TimedSample]],
    strings: dict[str, list[tuple[float, str]]],
    loaded: dict[str, Any],
) -> bool:
    """按话题的实际存储容器判断 Bag 中是否存在消息。"""

    special_containers = {
        LOCAL_POSITION_TOPIC: "local_positions",
        VEHICLE_ODOMETRY_TOPIC: "vehicle_odometry",
        VEHICLE_COMMAND_TOPIC: "commands",
        LAND_DETECTED_TOPIC: "land_flags",
    }
    container_name = special_containers.get(topic)
    if container_name is not None:
        return bool(loaded.get(container_name, []))
    return bool(numeric.get(topic, [])) or bool(strings.get(topic, []))


def terminal_stabilization_counterfactual_bias_from_normal(
    upward_normal_ned: Sequence[float],
    gravity_mps2: float = 9.80665,
    maximum_tilt_deg: float = 2.5,
    acceleration_limit_mps2: float = 0.45,
) -> tuple[float, float]:
    """按冻结 B1 公式生成历史重放用水平 acceleration bias。"""

    normal = normalize_vector(upward_normal_ned)
    if normal[2] >= -0.50:
        raise ValueError("upward normal does not satisfy the frozen upward gate")
    raw = (
        -gravity_mps2 * normal[0] / normal[2],
        -gravity_mps2 * normal[1] / normal[2],
    )
    raw_norm = math.hypot(*raw)
    limit = min(
        acceleration_limit_mps2,
        gravity_mps2 * math.tan(math.radians(maximum_tilt_deg)),
    )
    if raw_norm > limit and raw_norm > 0.0:
        scale = limit / raw_norm
        return raw[0] * scale, raw[1] * scale
    return raw


def terminal_stabilization_replay_failure_explanation(
    tilted_deck_touchdown_metrics: Optional[dict[str, Any]],
) -> str:
    """把既有硬门结果归类为通过、滑移或姿态发散证据。"""

    if not tilted_deck_touchdown_metrics:
        return "insufficient_tilted_deck_touchdown_metrics"
    contact = tilted_deck_touchdown_metrics.get("contact_metrics", tilted_deck_touchdown_metrics)
    roll = contact.get("post_touchdown_roll_max_abs_deg")
    pitch = contact.get("post_touchdown_pitch_max_abs_deg")
    recovery = contact.get("recovery_count", 0)
    detach = contact.get("detach_count", 0)
    if (
        (roll is not None and math.isfinite(float(roll)) and float(roll) > 10.0)
        or (pitch is not None and math.isfinite(float(pitch)) and float(pitch) > 10.0)
        or int(recovery or 0) > 0
        or int(detach or 0) > 0
    ):
        return "attitude_divergence_or_detach"
    slip = contact.get("post_touchdown_tangential_slip_m")
    if slip is not None and math.isfinite(float(slip)) and float(slip) > 0.10:
        return "post_contact_slip"
    if tilted_deck_touchdown_metrics.get("tilted_deck_touchdown_passed") is True:
        return "historical_tilted_deck_touchdown_pass"
    return "other_frozen_gate_failure"


def terminal_stabilization_tracking_state_authorized(mode: str, state: Optional[str]) -> bool:
    """限定姿态跟踪门的稳定评测窗口，不改变命令限幅/连续性窗口。"""

    if mode == "active":
        return state in {"TOUCHDOWN_CANDIDATE_HOLD", "TOUCHDOWN_HOLD"}
    if mode == "rehearsal":
        return state == "TEST_HEIGHT_HOLD"
    return False


def terminal_stabilization_gate(metrics: dict[str, Any]) -> dict[str, Any]:
    """应用真实实验前冻结的 terminal contact stabilization 功能自身门。

    shadow 模式只要求诊断激活、命令有限/连续以及无非法值；rehearsal 和
    active 模式额外要求实际姿态跟踪、无 fallback、无保护触发。rehearsal
    还检查无接触水平漂移。
    """

    thresholds = {
        "command_tilt_max_deg": 2.5,
        "command_tilt_slew_max_degps": 4.5,
        "combined_acceleration_max_mps2": 1.50,
        "attitude_tracking_error_p95_max_deg": 1.50,
        "rehearsal_horizontal_drift_max_m": 0.15,
    }
    mode = str(metrics.get("mode", "unknown"))
    failed_checks: list[str] = []
    try:
        activation_count = int(metrics.get("activation_sample_count", 0))
    except (TypeError, ValueError, OverflowError):
        activation_count = 0
    if activation_count <= 0:
        failed_checks.append("activation")

    numeric_checks = (
        (
            "command_tilt",
            "command_tilt_max_deg",
            thresholds["command_tilt_max_deg"],
        ),
        (
            "command_tilt_slew",
            "command_tilt_slew_max_degps",
            thresholds["command_tilt_slew_max_degps"],
        ),
        (
            "combined_acceleration",
            "combined_acceleration_max_mps2",
            thresholds["combined_acceleration_max_mps2"],
        ),
    )
    nonfinite_required = False
    for check, key, limit in numeric_checks:
        value = metrics.get(key)
        if value is None or not math.isfinite(float(value)):
            nonfinite_required = True
            failed_checks.append(check)
        elif float(value) > limit:
            failed_checks.append(check)

    if mode in {"rehearsal", "active"}:
        tracking_error = metrics.get("attitude_tracking_error_p95_deg")
        if tracking_error is None or not math.isfinite(float(tracking_error)):
            nonfinite_required = True
            failed_checks.append("attitude_tracking")
        elif float(tracking_error) > thresholds["attitude_tracking_error_p95_max_deg"]:
            failed_checks.append("attitude_tracking")
        if int(metrics.get("fallback_count_after_activation", 0)) != 0:
            failed_checks.append("fallback")

    if mode == "rehearsal":
        drift = metrics.get("rehearsal_horizontal_drift_max_m")
        if drift is None or not math.isfinite(float(drift)):
            nonfinite_required = True
            failed_checks.append("rehearsal_drift")
        elif float(drift) > thresholds["rehearsal_horizontal_drift_max_m"]:
            failed_checks.append("rehearsal_drift")

    if int(metrics.get("divergence_protection_count", 0)) != 0:
        failed_checks.append("divergence_protection")
    if int(metrics.get("time_sync_failure_count", 0)) != 0:
        failed_checks.append("time_sync")
    try:
        nan_inf_count = int(metrics.get("nan_inf_count", 0))
    except (TypeError, ValueError, OverflowError):
        nan_inf_count = 1
    if nonfinite_required or nan_inf_count != 0:
        failed_checks.append("nan_inf")

    failed_checks = list(dict.fromkeys(failed_checks))
    return {
        "passed": not failed_checks,
        "failed_checks": failed_checks,
        "thresholds": thresholds,
        "mode": mode,
    }


def validate_required_topics(
    available_topics: Iterable[str], required_topics: Iterable[str] = REQUIRED_SHADOW_TOPICS
) -> None:
    """缺失任一 deck-geometry shadow 话题时抛出带话题名的 RuntimeError。"""

    missing = set(required_topics) - set(available_topics)
    if missing:
        raise RuntimeError(
            "bag is missing required deck-geometry shadow topics: " + ", ".join(sorted(missing))
        )


def percentile(values: Sequence[float], quantile: float) -> float:
    """对有限样本使用线性插值计算分位数。"""

    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return math.nan
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be within [0, 1]")
    if len(finite) == 1:
        return finite[0]
    position = quantile * (len(finite) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    alpha = position - lower
    return finite[lower] * (1.0 - alpha) + finite[upper] * alpha


def scalar_summary(values: Sequence[float]) -> dict[str, Optional[float] | int]:
    """生成有限标量样本的 count/min/max/mean/RMSE/P95/max_abs。"""

    finite = [float(value) for value in values if math.isfinite(float(value))]
    if not finite:
        return {
            "count": 0,
            "min": None,
            "max": None,
            "mean": None,
            "rmse": None,
            "p95": None,
            "max_abs": None,
        }
    return {
        "count": len(finite),
        "min": min(finite),
        "max": max(finite),
        "mean": sum(finite) / len(finite),
        "rmse": math.sqrt(sum(value * value for value in finite) / len(finite)),
        "p95": percentile(finite, 0.95),
        "max_abs": max(abs(value) for value in finite),
    }


def marker_normal_statistics(
    normals_by_id: dict[int, Sequence[Sequence[float]]],
    truth_normal: Optional[Sequence[float]] = None,
) -> dict[str, Any]:
    """按 Marker ID 汇总样本数、平均法向、姿态与相对真值误差。"""

    result: dict[str, Any] = {}
    for marker_id in sorted(normals_by_id):
        normalized = [normalize_vector(normal) for normal in normals_by_id[marker_id]]
        if not normalized:
            continue
        mean_vector = tuple(
            sum(normal[axis] for normal in normalized) / len(normalized)
            for axis in range(3)
        )
        mean_normal = normalize_vector(mean_vector)
        angle_errors = (
            [normal_angle_error_deg(normal, truth_normal) for normal in normalized]
            if truth_normal is not None
            else []
        )
        roll_values = [normal_to_attitude_deg(normal)[0] for normal in normalized]
        pitch_values = [normal_to_attitude_deg(normal)[1] for normal in normalized]
        tilt_values = [normal_to_attitude_deg(normal)[2] for normal in normalized]
        result[str(marker_id)] = {
            "sample_count": len(normalized),
            "mean_normal_ned": [float(value) for value in mean_normal],
            "mean_attitude_deg": {
                "roll": sum(roll_values) / len(roll_values),
                "pitch": sum(pitch_values) / len(pitch_values),
                "tilt": sum(tilt_values) / len(tilt_values),
            },
            "roll_deg": scalar_summary(roll_values),
            "pitch_deg": scalar_summary(pitch_values),
            "tilt_deg": scalar_summary(tilt_values),
            "normal_angle_error_deg": scalar_summary(angle_errors),
        }
    return result


def marker_history_replay_statistics(
    samples: Sequence[MarkerHistorySample],
    filter_gain: float = 0.08,
    initial_filtered_normal: Optional[Sequence[float]] = None,
) -> dict[str, Any]:
    """Replay the shadow-only normal filter over an accepted multi-scale Bag."""

    if not math.isfinite(filter_gain) or not 0.0 < filter_gain <= 1.0:
        raise ValueError("marker history filter_gain must be within (0, 1]")
    ordered = sorted(samples, key=lambda sample: sample.time_s)
    per_id: dict[int, dict[str, list[Any]]] = collections.defaultdict(
        lambda: {
            "normals": [],
            "truth_normals": [],
            "roll": [],
            "pitch": [],
            "tilt": [],
            "angle_error": [],
            "height": [],
        }
    )
    filtered_normal: Optional[tuple[float, float, float]] = (
        normalize_vector(initial_filtered_normal)
        if initial_filtered_normal is not None
        else None
    )
    previous_marker_id: Optional[int] = None
    previous_filtered_normal: Optional[tuple[float, float, float]] = None
    switch_jumps: list[float] = []
    switch_sequence: list[str] = []
    switch_events: list[dict[str, Any]] = []

    for sample in ordered:
        if sample.marker_id not in range(4):
            raise ValueError(f"invalid Marker ID in history: {sample.marker_id}")
        if not math.isfinite(sample.time_s) or not math.isfinite(sample.relative_height_m):
            raise ValueError("Marker history timestamps and heights must be finite")
        raw_normal = normalize_vector(sample.raw_normal_ned)
        truth_normal = normalize_vector(sample.truth_normal_ned)
        if filtered_normal is None:
            filtered_normal = raw_normal
        else:
            filtered_normal = normalize_vector(
                tuple(
                    (1.0 - filter_gain) * filtered_normal[axis]
                    + filter_gain * raw_normal[axis]
                    for axis in range(3)
                )
            )
        attitude = normal_to_attitude_deg(filtered_normal)
        marker_data = per_id[sample.marker_id]
        marker_data["normals"].append(filtered_normal)
        marker_data["truth_normals"].append(truth_normal)
        marker_data["roll"].append(attitude[0])
        marker_data["pitch"].append(attitude[1])
        marker_data["tilt"].append(attitude[2])
        marker_data["angle_error"].append(
            normal_angle_error_deg(filtered_normal, truth_normal)
        )
        marker_data["height"].append(float(sample.relative_height_m))

        if (
            previous_marker_id is not None
            and previous_filtered_normal is not None
            and sample.marker_id != previous_marker_id
        ):
            jump_deg = normal_angle_error_deg(previous_filtered_normal, filtered_normal)
            switch_jumps.append(jump_deg)
            transition = f"{previous_marker_id}->{sample.marker_id}"
            switch_sequence.append(transition)
            switch_events.append(
                {
                    "time_s": float(sample.time_s),
                    "transition": transition,
                    "relative_height_m": float(sample.relative_height_m),
                    "normal_jump_deg": jump_deg,
                }
            )
        previous_marker_id = sample.marker_id
        previous_filtered_normal = filtered_normal

    marker_statistics: dict[str, Any] = {}
    for marker_id in sorted(per_id):
        marker_data = per_id[marker_id]
        normals = marker_data["normals"]
        mean_normal = normalize_vector(
            tuple(
                sum(normal[axis] for normal in normals) / len(normals)
                for axis in range(3)
            )
        )
        marker_statistics[str(marker_id)] = {
            "sample_count": len(normals),
            "mean_normal_ned": list(mean_normal),
            "mean_attitude_deg": {
                "roll": sum(marker_data["roll"]) / len(marker_data["roll"]),
                "pitch": sum(marker_data["pitch"]) / len(marker_data["pitch"]),
                "tilt": sum(marker_data["tilt"]) / len(marker_data["tilt"]),
            },
            "roll_deg": scalar_summary(marker_data["roll"]),
            "pitch_deg": scalar_summary(marker_data["pitch"]),
            "tilt_deg": scalar_summary(marker_data["tilt"]),
            "normal_angle_error_deg": scalar_summary(marker_data["angle_error"]),
            "relative_height_m": scalar_summary(marker_data["height"]),
        }

    synthetic_sign_by_id: dict[str, Any] = {}
    all_synthetic_signs_correct = True
    for marker_id, statistics in marker_statistics.items():
        checks: dict[str, Any] = {}
        marker_all_correct = True
        for axis in ("roll", "pitch"):
            fixed_bias_deg = float(statistics["mean_attitude_deg"][axis])
            for expected_deg in (2.0, -2.0):
                projected_deg = expected_deg + fixed_bias_deg
                sign_correct = projected_deg * expected_deg > 0.0
                marker_all_correct = marker_all_correct and sign_correct
                checks[f"{axis}_{'pos' if expected_deg > 0.0 else 'neg'}_2deg"] = {
                    "fixed_bias_deg": fixed_bias_deg,
                    "projected_estimate_deg": projected_deg,
                    "sign_correct": sign_correct,
                }
        all_synthetic_signs_correct = (
            all_synthetic_signs_correct and marker_all_correct
        )
        synthetic_sign_by_id[marker_id] = {
            "checks": checks,
            "all_signs_correct": marker_all_correct,
        }

    observed = sorted(per_id)
    missing = [marker_id for marker_id in range(4) if marker_id not in per_id]
    jump_summary = scalar_summary(switch_jumps)
    failed_checks: list[str] = []
    if missing:
        failed_checks.append("missing_marker_ids")
    if not switch_jumps:
        failed_checks.append("no_marker_switches")
    if jump_summary["max"] is not None and float(jump_summary["max"]) > 1.0:
        failed_checks.append("marker_switch_jump")

    return {
        "filter_gain": filter_gain,
        "sample_count": len(ordered),
        "marker_ids_observed": observed,
        "marker_ids_missing": missing,
        "marker_statistics": marker_statistics,
        "synthetic_two_degree_sign_observation": {
            "status": "observation_only_not_real_tilted_imagery",
            "method": (
                "add each real horizontal Marker fixed roll/pitch bias to theoretical "
                "+/-2 degree local-NED tilt; this is not a substitute for real tilted "
                "imagery for IDs 1-3"
            ),
            "by_marker_id": synthetic_sign_by_id,
            "all_signs_correct": all_synthetic_signs_correct,
        },
        "switch_count": len(switch_jumps),
        "switch_sequence": switch_sequence,
        "switch_events": switch_events,
        "switch_jump_deg": jump_summary,
        "thresholds": {"marker_switch_jump_max_deg": 1.0},
        "failed_checks": failed_checks,
        "passed": not failed_checks,
    }


def calibration_gate(
    signed_axis_errors_deg: Sequence[float],
    normal_angle_errors_deg: Sequence[float],
    sign_correct_samples: Sequence[bool],
    marker_switch_jumps_deg: Sequence[float],
    require_signed_mean: bool = True,
    require_sign_accuracy: bool = True,
) -> dict[str, Any]:
    """按 tilted-deck 计划冻结阈值判断固定 ±2° 法向标定。"""

    thresholds = {
        "mean_signed_error_abs_max_deg": 0.5,
        "rmse_max_deg": 1.0,
        "p95_max_deg": 1.5,
        "sign_accuracy_min": 0.95,
        "marker_switch_jump_max_deg": 1.0,
    }
    signed = [float(value) for value in signed_axis_errors_deg if math.isfinite(float(value))]
    absolute = [
        abs(float(value))
        for value in normal_angle_errors_deg
        if math.isfinite(float(value))
    ]
    sign_accuracy = (
        sum(bool(value) for value in sign_correct_samples) / len(sign_correct_samples)
        if sign_correct_samples
        else None
    )
    jump_values = [
        abs(float(value)) for value in marker_switch_jumps_deg if math.isfinite(float(value))
    ]
    mean_signed_error = sum(signed) / len(signed) if signed else None
    rmse = math.sqrt(sum(value * value for value in absolute) / len(absolute)) if absolute else None
    p95 = percentile(absolute, 0.95) if absolute else None
    maximum_jump = max(jump_values) if jump_values else 0.0

    failed_checks: list[str] = []
    if require_signed_mean and (
        mean_signed_error is None
        or abs(mean_signed_error) > thresholds["mean_signed_error_abs_max_deg"]
    ):
        failed_checks.append("mean_signed_error")
    if rmse is None or rmse > thresholds["rmse_max_deg"]:
        failed_checks.append("rmse")
    if p95 is None or p95 > thresholds["p95_max_deg"]:
        failed_checks.append("p95")
    if require_sign_accuracy and (
        sign_accuracy is None or sign_accuracy < thresholds["sign_accuracy_min"]
    ):
        failed_checks.append("sign_accuracy")
    if maximum_jump > thresholds["marker_switch_jump_max_deg"]:
        failed_checks.append("marker_switch_jump")

    return {
        "thresholds": thresholds,
        "sample_count": len(absolute),
        "mean_signed_error_deg": mean_signed_error,
        "rmse_deg": rmse,
        "p95_deg": p95,
        "rmse_p95_metric": "complete_normal_angle_error_deg",
        "sign_accuracy": sign_accuracy,
        "marker_switch_jump_max_deg": maximum_jump,
        "applicability": {
            "mean_signed_error": "required" if require_signed_mean else "not_applicable",
            "sign_accuracy": "required" if require_sign_accuracy else "not_applicable",
        },
        "failed_checks": failed_checks,
        "passed": not failed_checks,
    }


def _quaternion_rotate_wxyz(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    if len(quaternion) != 4 or not finite_values(quaternion):
        raise ValueError("quaternion must contain four finite wxyz values")
    w, x, y, z = (float(value) for value in quaternion)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is too small")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    vx, vy, vz = (float(value) for value in vector)
    # q * v * q^-1 的展开式。
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def quaternion_to_roll_pitch_deg(
    quaternion_wxyz: Sequence[float],
) -> tuple[float, float]:
    """将 PX4 姿态四元数转换为 roll/pitch，单位 degree。"""

    if len(quaternion_wxyz) != 4 or not finite_values(quaternion_wxyz):
        raise ValueError("quaternion must contain four finite wxyz values")
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is too small")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_value = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(roll), math.degrees(math.asin(pitch_value))


def ground_truth_upward_normal_ned(quaternion_wxyz: Sequence[float]) -> tuple[float, ...]:
    """将 Gazebo 甲板 body +Z 通过 world ENU 姿态转换为 local NED 向上法向。"""

    normal_enu = _quaternion_rotate_wxyz(quaternion_wxyz, (0.0, 0.0, 1.0))
    normal_ned = (normal_enu[1], normal_enu[0], -normal_enu[2])
    return normalize_vector(normal_ned)


def _append_numeric(
    samples: list[TimedSample],
    invalid_counts: collections.Counter[str],
    topic: str,
    time_s: float,
    values: Sequence[float],
    expected_size: int,
) -> None:
    if len(values) != expected_size or not finite_values(values):
        invalid_counts[topic] += 1
        return
    samples.append(TimedSample(time_s, tuple(float(value) for value in values)))


def _load_samples(args: argparse.Namespace) -> dict[str, Any]:
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
    validate_required_topics(topic_types)
    if args.tilted_deck_touchdown:
        validate_required_topics(topic_types, TILTED_DECK_TOUCHDOWN_REQUIRED_TOPICS)
    elif args.fixed_tilt_safe_descent:
        validate_required_topics(topic_types, FIXED_TILT_SAFE_DESCENT_REQUIRED_TOPICS)
    elif args.scenario is not None:
        validate_required_topics(topic_types, FIXED_TILT_SAFE_ALTITUDE_REQUIRED_TOPICS)
    elif args.require_ground_truth and GROUND_TRUTH_TOPIC not in topic_types:
        raise RuntimeError("bag is missing required Ground Truth topic")
    if args.terminal_stabilization:
        validate_required_topics(topic_types, TERMINAL_STABILIZATION_REQUIRED_TOPICS)

    selected_topics = set(REQUIRED_SHADOW_TOPICS) | {
        SKID_VELOCITIES_TOPIC,
        GROUND_TRUTH_TOPIC,
        STATE_TOPIC,
        TARGET_TOPIC,
        TRAJECTORY_TOPIC,
        VEHICLE_COMMAND_TOPIC,
        LOCAL_POSITION_TOPIC,
        VEHICLE_ODOMETRY_TOPIC,
        RELATIVE_HEIGHT_TOPIC,
        RAW_RELATIVE_HEIGHT_TOPIC,
        HEIGHT_REFERENCE_TOPIC,
        ARUCO_VISIBLE_TOPIC,
        TOUCHDOWN_STATUS_TOPIC,
        TOUCHDOWN_EVIDENCE_TOPIC,
        TOUCHDOWN_DURATION_TOPIC,
        TOUCHDOWN_CONFIRMED_TOPIC,
        HOLD_REFERENCE_TOPIC,
        HOLD_TARGET_TOPIC,
        HOLD_MODE_TOPIC,
        HOLD_REASON_TOPIC,
        LAND_DETECTED_TOPIC,
    } | TERMINAL_STABILIZATION_REQUIRED_TOPICS
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in selected_topics
        if topic in topic_types
    }

    numeric: dict[str, list[TimedSample]] = collections.defaultdict(list)
    strings: dict[str, list[tuple[float, str]]] = collections.defaultdict(list)
    invalid_counts: collections.Counter[str] = collections.Counter()
    commands: list[tuple[int, float]] = []
    ground_truth_positions_enu: list[TimedSample] = []
    local_positions: list[tuple[float, Any]] = []
    vehicle_odometry: list[TimedSample] = []
    trajectory_nonfinite_position_times: list[float] = []
    target_nonfinite_position_times: list[float] = []
    land_flags: list[tuple[float, dict[str, bool]]] = []

    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        time_s = timestamp_ns * 1.0e-9
        message = deserialize_message(serialized_data, message_types[topic])
        if topic in {
            STATUS_TOPIC,
            STATE_TOPIC,
            TOUCHDOWN_STATUS_TOPIC,
            HOLD_MODE_TOPIC,
            HOLD_REASON_TOPIC,
            TERMINAL_MODE_TOPIC,
            TERMINAL_REASON_TOPIC,
            TERMINAL_DIVERGENCE_TOPIC,
        }:
            strings[topic].append((time_s, str(message.data)))
        elif topic in {
            BODY_CLEARANCE_TOPIC,
            MIN_CLEARANCE_TOPIC,
            MAX_CLEARANCE_TOPIC,
            SPREAD_TOPIC,
            NORMAL_VELOCITY_TOPIC,
            NORMAL_RATE_TOPIC,
            MARKER_SWITCH_JUMP_TOPIC,
            RELATIVE_HEIGHT_TOPIC,
            RAW_RELATIVE_HEIGHT_TOPIC,
            HEIGHT_REFERENCE_TOPIC,
            TOUCHDOWN_EVIDENCE_TOPIC,
            TOUCHDOWN_DURATION_TOPIC,
            HOLD_REFERENCE_TOPIC,
            HOLD_TARGET_TOPIC,
        }:
            _append_numeric(numeric[topic], invalid_counts, topic, time_s, (message.data,), 1)
        elif topic in {FIRST_CONTACT_TOPIC, ACTIVE_MARKER_TOPIC, MARKER_NORMAL_VALID_MASK_TOPIC}:
            _append_numeric(numeric[topic], invalid_counts, topic, time_s, (message.data,), 1)
        elif topic in {
            ARUCO_VISIBLE_TOPIC,
            TOUCHDOWN_CONFIRMED_TOPIC,
            TERMINAL_ENABLED_TOPIC,
        }:
            _append_numeric(
                numeric[topic], invalid_counts, topic, time_s, (1.0 if bool(message.data) else 0.0,), 1
            )
        elif topic in {
            NORMAL_TOPIC,
            TANGENTIAL_POSITION_TOPIC,
            TANGENTIAL_VELOCITY_TOPIC,
            TERMINAL_DESIRED_NORMAL_TOPIC,
            TERMINAL_DESIRED_ATTITUDE_TOPIC,
            TERMINAL_ACTUAL_ATTITUDE_TOPIC,
            TERMINAL_ATTITUDE_ERROR_TOPIC,
            TERMINAL_ACCELERATION_BIAS_TOPIC,
            TERMINAL_COMBINED_ACCELERATION_TOPIC,
            TERMINAL_CONTACT_ANCHOR_TOPIC,
            TERMINAL_COMPLIANT_TARGET_TOPIC,
        }:
            vector = message.vector
            _append_numeric(
                numeric[topic], invalid_counts, topic, time_s,
                (vector.x, vector.y, vector.z), 3,
            )
        elif topic == ESTIMATED_ATTITUDE_TOPIC:
            vector = message.vector
            _append_numeric(
                numeric[topic], invalid_counts, topic, time_s,
                (vector.x, vector.y, vector.z), 3,
            )
        elif topic in {SKID_CLEARANCES_TOPIC, SKID_VELOCITIES_TOPIC}:
            _append_numeric(
                numeric[topic], invalid_counts, topic, time_s, tuple(message.data), 4
            )
        elif topic == MARKER_NORMALS_TOPIC:
            _append_numeric(
                numeric[topic], invalid_counts, topic, time_s, tuple(message.data), 12
            )
        elif topic == TARGET_TOPIC:
            position = message.pose.position
            position_values = (
                float(position.x),
                float(position.y),
                float(position.z),
            )
            if finite_values(position_values):
                numeric[topic].append(TimedSample(time_s, position_values))
            else:
                # Target pose is intentionally unavailable before PX4/local-state
                # initialization. Any non-finite target after tracking begins is a
                # hard safety failure.
                target_nonfinite_position_times.append(time_s)
        elif topic == TRAJECTORY_TOPIC:
            position_values = tuple(float(value) for value in message.position)
            velocity_values = tuple(float(value) for value in message.velocity)
            if finite_values(position_values):
                numeric[topic].append(TimedSample(time_s, position_values))
            else:
                # PX4 uses an all-NaN setpoint as an intentional disabled-dimension
                # placeholder during controller startup. It is still reported, and
                # becomes a hard failure if it occurs once visual tracking begins.
                trajectory_nonfinite_position_times.append(time_s)
            if finite_values(velocity_values):
                numeric[TRAJECTORY_TOPIC + ":velocity"].append(
                    TimedSample(time_s, velocity_values)
                )
            acceleration_values = tuple(float(value) for value in message.acceleration)
            finite_horizontal_acceleration = finite_values(acceleration_values[:2])
            if finite_horizontal_acceleration:
                numeric[TRAJECTORY_TOPIC + ":acceleration_xy"].append(
                    TimedSample(time_s, acceleration_values[:2])
                )
        elif topic == VEHICLE_COMMAND_TOPIC:
            command = int(message.command)
            param1 = float(message.param1)
            if not math.isfinite(param1):
                invalid_counts[topic] += 1
            else:
                commands.append((command, param1))
        elif topic == LOCAL_POSITION_TOPIC:
            local_positions.append((time_s, message))
        elif topic == VEHICLE_ODOMETRY_TOPIC:
            _append_numeric(
                vehicle_odometry,
                invalid_counts,
                topic,
                time_s,
                tuple(message.position) + tuple(message.q),
                7,
            )
            _append_numeric(
                numeric[VEHICLE_ODOMETRY_TOPIC + ":angular_velocity"],
                invalid_counts,
                VEHICLE_ODOMETRY_TOPIC + ":angular_velocity",
                time_s,
                tuple(message.angular_velocity),
                3,
            )
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
                            "vertical_movement",
                            "horizontal_movement",
                            "rotational_movement",
                        )
                    },
                )
            )
        elif topic == GROUND_TRUTH_TOPIC:
            orientation = message.pose.pose.orientation
            position = message.pose.pose.position
            position_values = (position.x, position.y, position.z)
            try:
                normal_ned = ground_truth_upward_normal_ned(
                    (orientation.w, orientation.x, orientation.y, orientation.z)
                )
            except ValueError:
                invalid_counts[topic] += 1
            else:
                linear = message.twist.twist.linear
                angular = message.twist.twist.angular
                linear_values = (linear.x, linear.y, linear.z)
                angular_values = (angular.x, angular.y, angular.z)
                if finite_values(position_values + linear_values + angular_values):
                    numeric[topic].append(TimedSample(time_s, normal_ned))
                    numeric[GROUND_TRUTH_TOPIC + ":linear_velocity_enu"].append(
                        TimedSample(time_s, tuple(float(value) for value in linear_values))
                    )
                    numeric[GROUND_TRUTH_TOPIC + ":angular_velocity_body"].append(
                        TimedSample(time_s, tuple(float(value) for value in angular_values))
                    )
                    ground_truth_positions_enu.append(
                        TimedSample(time_s, tuple(float(value) for value in position_values))
                    )
                else:
                    invalid_counts[topic] += 1

    for topic in numeric:
        numeric[topic].sort(key=lambda sample: sample.time_s)
    vehicle_odometry.sort(key=lambda sample: sample.time_s)
    ground_truth_positions_enu.sort(key=lambda sample: sample.time_s)
    return {
        "bag": str(bag_uri),
        "topic_types": topic_types,
        "numeric": numeric,
        "strings": strings,
        "invalid_counts": invalid_counts,
        "commands": commands,
        "ground_truth_positions_enu": ground_truth_positions_enu,
        "local_positions": local_positions,
        "vehicle_odometry": vehicle_odometry,
        "trajectory_nonfinite_position_times": trajectory_nonfinite_position_times,
        "target_nonfinite_position_times": target_nonfinite_position_times,
        "land_flags": land_flags,
    }


def select_marker_history_height_topic(
    available_topics: Iterable[str],
    sample_counts: Optional[dict[str, int]] = None,
) -> str:
    """Choose a present, non-empty relative-height topic for Marker-history replay."""

    available = set(available_topics)
    candidates = (RELATIVE_HEIGHT_TOPIC, RAW_RELATIVE_HEIGHT_TOPIC)
    if sample_counts is None:
        for topic in candidates:
            if topic in available:
                return topic
    else:
        for topic in candidates:
            if topic in available and sample_counts.get(topic, 0) > 0:
                return topic
    raise RuntimeError(
        "marker history bag is missing a non-empty relative height topic: "
        f"{RELATIVE_HEIGHT_TOPIC} or {RAW_RELATIVE_HEIGHT_TOPIC}"
    )


def analyze_marker_history_bag(
    bag: Path,
    filter_gain: float = 0.08,
    max_sync_gap_s: float = 0.10,
) -> dict[str, Any]:
    """Replay real multi-scale Marker history from an already accepted rosbag."""

    if not math.isfinite(max_sync_gap_s) or max_sync_gap_s < 0.0:
        raise ValueError("marker history max_sync_gap_s must be finite and non-negative")
    (
        rosbag2_py,
        deserialize_message,
        get_message,
        storage_options_type,
        converter_options_type,
    ) = load_ros_modules()
    bag_uri = resolve_bag_uri(bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        storage_options_type(uri=str(bag_uri), storage_id="sqlite3"),
        converter_options_type(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    height_topics = {
        topic
        for topic in (RELATIVE_HEIGHT_TOPIC, RAW_RELATIVE_HEIGHT_TOPIC)
        if topic in topic_types
    }
    select_marker_history_height_topic(height_topics)
    required_topics = {
        MARKER_POSE_NED_TOPIC,
        ACTIVE_MARKER_TOPIC,
        STATE_TOPIC,
        GROUND_TRUTH_TOPIC,
    }
    missing = sorted(required_topics - set(topic_types))
    if missing:
        raise RuntimeError(
            "marker history bag is missing required topics: " + ", ".join(missing)
        )
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in required_topics | height_topics
    }

    marker_quaternions: list[TimedSample] = []
    marker_ids: list[TimedSample] = []
    relative_heights_by_topic: dict[str, list[TimedSample]] = {
        topic: [] for topic in height_topics
    }
    truth_normals: list[TimedSample] = []
    states: list[tuple[float, str]] = []
    invalid_counts: collections.Counter[str] = collections.Counter()

    while reader.has_next():
        topic, serialized_data, timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        time_s = timestamp_ns * 1.0e-9
        message = deserialize_message(serialized_data, message_types[topic])
        if topic == MARKER_POSE_NED_TOPIC:
            orientation = message.pose.orientation
            _append_numeric(
                marker_quaternions,
                invalid_counts,
                topic,
                time_s,
                (orientation.w, orientation.x, orientation.y, orientation.z),
                4,
            )
        elif topic == ACTIVE_MARKER_TOPIC:
            _append_numeric(
                marker_ids, invalid_counts, topic, time_s, (message.data,), 1
            )
        elif topic in height_topics:
            _append_numeric(
                relative_heights_by_topic[topic],
                invalid_counts,
                topic,
                time_s,
                (message.data,),
                1,
            )
        elif topic == STATE_TOPIC:
            states.append((time_s, str(message.data)))
        elif topic == GROUND_TRUTH_TOPIC:
            orientation = message.pose.pose.orientation
            try:
                normal = ground_truth_upward_normal_ned(
                    (orientation.w, orientation.x, orientation.y, orientation.z)
                )
            except ValueError:
                invalid_counts[topic] += 1
            else:
                truth_normals.append(TimedSample(time_s, normal))

    marker_quaternions.sort(key=lambda sample: sample.time_s)
    marker_ids.sort(key=lambda sample: sample.time_s)
    for samples in relative_heights_by_topic.values():
        samples.sort(key=lambda sample: sample.time_s)
    height_topic = select_marker_history_height_topic(
        height_topics,
        {topic: len(samples) for topic, samples in relative_heights_by_topic.items()},
    )
    relative_heights = relative_heights_by_topic[height_topic]
    truth_normals.sort(key=lambda sample: sample.time_s)
    states.sort(key=lambda item: item[0])
    start_time_s = next(
        (time_s for time_s, state in states if state == "ACQUIRE_ARUCO"),
        None,
    )
    if start_time_s is None:
        raise RuntimeError("marker history bag never entered ACQUIRE_ARUCO")

    if not marker_quaternions or not marker_ids or not relative_heights or not truth_normals:
        raise RuntimeError("marker history bag contains an empty required topic")
    analysis_start_time_s = max(
        start_time_s,
        marker_ids[0].time_s,
        relative_heights[0].time_s,
        truth_normals[0].time_s,
    )
    analysis_end_time_s = min(
        marker_quaternions[-1].time_s,
        marker_ids[-1].time_s,
        relative_heights[-1].time_s,
        truth_normals[-1].time_s,
    )
    if analysis_start_time_s > analysis_end_time_s:
        raise RuntimeError("marker history required topics have no common time interval")

    synchronized_samples: list[MarkerHistorySample] = []
    sync_failures = 0
    pre_overlap_sample_count = 0
    post_overlap_sample_count = 0
    warmup_filtered_normal: Optional[tuple[float, float, float]] = None
    for quaternion_sample in marker_quaternions:
        if quaternion_sample.time_s < start_time_s:
            continue
        try:
            raw_normal = normalize_vector(
                _quaternion_rotate_wxyz(quaternion_sample.values, (0.0, 0.0, 1.0))
            )
        except ValueError:
            invalid_counts[MARKER_POSE_NED_TOPIC] += 1
            continue
        if quaternion_sample.time_s < analysis_start_time_s:
            pre_overlap_sample_count += 1
            if warmup_filtered_normal is None:
                warmup_filtered_normal = raw_normal
            else:
                warmup_filtered_normal = normalize_vector(
                    tuple(
                        (1.0 - filter_gain) * warmup_filtered_normal[axis]
                        + filter_gain * raw_normal[axis]
                        for axis in range(3)
                    )
                )
            continue
        if quaternion_sample.time_s > analysis_end_time_s:
            post_overlap_sample_count += 1
            continue
        marker_values = nearest_sample(
            marker_ids, quaternion_sample.time_s, max_sync_gap_s
        )
        height_values = nearest_sample(
            relative_heights, quaternion_sample.time_s, max_sync_gap_s
        )
        truth_normal = interpolate_sample(
            truth_normals, quaternion_sample.time_s, max_sync_gap_s
        )
        if marker_values is None or height_values is None or truth_normal is None:
            sync_failures += 1
            continue
        synchronized_samples.append(
            MarkerHistorySample(
                quaternion_sample.time_s,
                int(round(marker_values[0])),
                raw_normal,
                normalize_vector(truth_normal),
                float(height_values[0]),
            )
        )

    replay = marker_history_replay_statistics(
        synchronized_samples,
        filter_gain=filter_gain,
        initial_filtered_normal=warmup_filtered_normal,
    )
    invalid_count = sum(invalid_counts.values())
    failed_checks = list(replay["failed_checks"])
    if invalid_count:
        failed_checks.append("invalid_numeric_data")
    if sync_failures:
        failed_checks.append("time_sync")
    replay.update(
        {
            "bag": str(bag_uri),
            "start_state": "ACQUIRE_ARUCO",
            "relative_height_topic": height_topic,
            "start_time_s": start_time_s,
            "analysis_start_time_s": analysis_start_time_s,
            "analysis_end_time_s": analysis_end_time_s,
            "pre_overlap_filter_warmup_sample_count": pre_overlap_sample_count,
            "post_overlap_sample_count": post_overlap_sample_count,
            "invalid_numeric_counts": dict(invalid_counts),
            "invalid_numeric_count": invalid_count,
            "time_sync_failure_count": sync_failures,
            "ground_truth_isolation": (
                "Ground Truth is consumed only by this offline historical replay."
            ),
            "evidence_scope": (
                "real accepted horizontal final descent/touchdown Bag; used to quantify "
                "multi-scale fixed bias and actual Marker switches without opening "
                "new tilted descent"
            ),
            "failed_checks": failed_checks,
            "passed": not failed_checks,
        }
    )
    return replay


def _tilted_deck_touchdown_metrics(
    args: argparse.Namespace,
    loaded: dict[str, Any],
    numeric: dict[str, list[TimedSample]],
    strings: dict[str, list[tuple[float, str]]],
    gt_positions_ned: Sequence[TimedSample],
    gt_minimum_clearance_samples: Sequence[TimedSample],
    gt_skid_clearance_samples: Sequence[Sequence[TimedSample]],
    states: Sequence[str],
    state_durations_s: collections.Counter[str],
    bag_end_s: float,
    time_sync_failure_count: int,
    nan_inf_count: int,
    nav_land_command_count: int,
    disarm_command_count: int,
    normal_summary: dict[str, Any],
    calibration: Optional[dict[str, Any]],
    marker_switch_count: int,
    marker_jump_summary: dict[str, Any],
    visual_maximum_not_visible_gap_s: float,
) -> dict[str, Any]:
    """计算 fixed-tilt touchdown 真实触地指标并应用冻结硬门。"""

    state_samples = strings[STATE_TOPIC]
    final_start_s = next(
        (time_s for time_s, state in state_samples if state == "FINAL_DESCENT"), None
    )
    state_candidate_start_s = next(
        (
            time_s
            for time_s, state in state_samples
            if state == "TOUCHDOWN_CANDIDATE_HOLD"
        ),
        None,
    )
    hold_start_s = next(
        (time_s for time_s, state in state_samples if state == "TOUCHDOWN_HOLD"), None
    )
    touchdown_status = strings[TOUCHDOWN_STATUS_TOPIC]
    candidate_start_s = next(
        (time_s for time_s, value in touchdown_status if value == "CANDIDATE"),
        state_candidate_start_s,
    )
    confirmed_start_s = next(
        (time_s for time_s, value in touchdown_status if value == "CONFIRMED"), None
    )
    if confirmed_start_s is None:
        confirmed_start_s = next(
            (
                sample.time_s
                for sample in numeric[TOUCHDOWN_CONFIRMED_TOPIC]
                if sample.values[0] >= 0.5
            ),
            None,
        )

    contact_enter_m = 0.03
    detached_enter_m = 0.05
    true_first_contact_s = next(
        (
            sample.time_s
            for sample in gt_minimum_clearance_samples
            if sample.values[0] <= contact_enter_m
        ),
        None,
    )

    candidate_duration_s = max(
        (sample.values[0] for sample in numeric[TOUCHDOWN_DURATION_TOPIC]),
        default=math.nan,
    )
    hold_duration_s = (
        max(0.0, bag_end_s - hold_start_s) if hold_start_s is not None else 0.0
    )
    hard_gate_validation_end_s = (
        min(bag_end_s, hold_start_s + 10.0)
        if hold_start_s is not None
        else bag_end_s
    )
    status_values = [value for _, value in touchdown_status]
    candidate_entry_count = transition_entry_count(status_values, "CANDIDATE")

    def scalar_at(topic: str, time_s: Optional[float]) -> Optional[float]:
        if time_s is None:
            return None
        value = nearest_sample(numeric[topic], time_s, args.max_sync_gap_s)
        if value is None or not value or not math.isfinite(value[0]):
            return None
        return float(value[0])

    touchdown_h_min_m = scalar_at(MIN_CLEARANCE_TOPIC, confirmed_start_s)
    touchdown_h_max_m = scalar_at(MAX_CLEARANCE_TOPIC, confirmed_start_s)
    touchdown_spread_m = scalar_at(SPREAD_TOPIC, confirmed_start_s)
    touchdown_normal_velocity_mps = scalar_at(NORMAL_VELOCITY_TOPIC, confirmed_start_s)
    touchdown_tangent = (
        nearest_sample(numeric[TANGENTIAL_VELOCITY_TOPIC], confirmed_start_s, args.max_sync_gap_s)
        if confirmed_start_s is not None
        else None
    )
    touchdown_tangential_velocity_mps = (
        math.sqrt(sum(value * value for value in touchdown_tangent))
        if touchdown_tangent is not None
        else None
    )

    gt_first_contact_clearances = [
        scalar_at_topic
        for scalar_at_topic in (
            (
                nearest_sample(samples, true_first_contact_s, args.max_sync_gap_s)[0]
                if true_first_contact_s is not None
                and nearest_sample(samples, true_first_contact_s, args.max_sync_gap_s)
                is not None
                else math.nan
            )
            for samples in gt_skid_clearance_samples
        )
    ]
    gt_first_contact_valid = len(gt_first_contact_clearances) == 4 and finite_values(
        gt_first_contact_clearances
    )
    gt_first_contact_h_min_m = (
        min(gt_first_contact_clearances) if gt_first_contact_valid else None
    )
    gt_first_contact_h_max_m = (
        max(gt_first_contact_clearances) if gt_first_contact_valid else None
    )
    gt_first_contact_spread_m = (
        clearance_spread(gt_first_contact_clearances) if gt_first_contact_valid else None
    )
    ground_truth_first_contact_index = (
        contact_point_index(gt_first_contact_clearances) if gt_first_contact_valid else None
    )
    predicted_first_contact_index_value = scalar_at(FIRST_CONTACT_TOPIC, true_first_contact_s)
    predicted_first_contact_index = (
        int(round(predicted_first_contact_index_value))
        if predicted_first_contact_index_value is not None
        else None
    )

    skid_contact_times: list[tuple[float, int]] = []
    for skid_index, samples in enumerate(gt_skid_clearance_samples):
        contact_time = next(
            (sample.time_s for sample in samples if sample.values[0] <= contact_enter_m),
            None,
        )
        if contact_time is not None:
            skid_contact_times.append((contact_time, skid_index))
    skid_contact_times.sort()
    other_side_contact_delay_s = (
        skid_contact_times[1][0] - skid_contact_times[0][0]
        if len(skid_contact_times) >= 2
        else None
    )

    gt_confirm_clearances: list[float] = []
    if confirmed_start_s is not None:
        for samples in gt_skid_clearance_samples:
            value = nearest_sample(samples, confirmed_start_s, args.max_sync_gap_s)
            if value is None:
                gt_confirm_clearances = []
                break
            gt_confirm_clearances.append(float(value[0]))
    gt_confirm_valid = len(gt_confirm_clearances) == 4 and finite_values(
        gt_confirm_clearances
    )

    horizontal_errors_m: list[float] = []
    relative_horizontal_speeds_mps: list[float] = []
    relative_positions: list[TimedSample] = []
    relative_velocities: list[TimedSample] = []
    gt_velocity_enu = numeric[GROUND_TRUTH_TOPIC + ":linear_velocity_enu"]
    if final_start_s is not None:
        for time_s, message in loaded["local_positions"]:
            if time_s < final_start_s:
                continue
            position_values = (float(message.x), float(message.y), float(message.z))
            if not finite_values(position_values):
                continue
            deck_position = interpolate_sample(
                gt_positions_ned, time_s, args.max_sync_gap_s
            )
            if deck_position is None:
                continue
            relative_xy = (
                position_values[0] - deck_position[0],
                position_values[1] - deck_position[1],
            )
            relative_positions.append(TimedSample(time_s, relative_xy))
            horizontal_errors_m.append(math.hypot(*relative_xy))

            velocity_values = (float(message.vx), float(message.vy))
            deck_velocity_enu = interpolate_sample(
                gt_velocity_enu, time_s, args.max_sync_gap_s
            )
            if finite_values(velocity_values) and deck_velocity_enu is not None:
                deck_velocity_ned_xy = (
                    deck_velocity_enu[1],
                    deck_velocity_enu[0],
                )
                relative_velocity = (
                    velocity_values[0] - deck_velocity_ned_xy[0],
                    velocity_values[1] - deck_velocity_ned_xy[1],
                )
                relative_velocities.append(TimedSample(time_s, relative_velocity))
                relative_horizontal_speeds_mps.append(math.hypot(*relative_velocity))

    tangential_velocity_after_final = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[TANGENTIAL_VELOCITY_TOPIC]
        if final_start_s is not None and sample.time_s >= final_start_s
    ]
    tangential_position_after_final = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[TANGENTIAL_POSITION_TOPIC]
        if final_start_s is not None and sample.time_s >= final_start_s
    ]

    contact_relative_positions = timed_samples_between(
        relative_positions, true_first_contact_s, hard_gate_validation_end_s
    )
    post_touchdown_slip_m = math.nan
    if contact_relative_positions:
        origin = contact_relative_positions[0].values
        post_touchdown_slip_m = max(
            math.hypot(
                sample.values[0] - origin[0], sample.values[1] - origin[1]
            )
            for sample in contact_relative_positions
        )
    hold_velocity_norms = [
        math.hypot(*sample.values)
        for sample in timed_samples_between(
            relative_velocities,
            hold_start_s + 0.50 if hold_start_s is not None else None,
            hard_gate_validation_end_s,
        )
    ]
    hold_tangential_velocity_p95_mps = percentile(hold_velocity_norms, 0.95)

    validation_contact_clearances = [
        sample.values[0]
        for sample in timed_samples_between(
            gt_minimum_clearance_samples,
            true_first_contact_s,
            hard_gate_validation_end_s,
        )
    ]
    detach_count, secondary_contact_count = contact_transition_counts(
        validation_contact_clearances,
        contact_enter_m=contact_enter_m,
        detached_enter_m=detached_enter_m,
    )
    full_contact_clearances = [
        sample.values[0]
        for sample in timed_samples_between(
            gt_minimum_clearance_samples, true_first_contact_s, bag_end_s
        )
    ]
    full_bag_detach_count, full_bag_secondary_contact_count = contact_transition_counts(
        full_contact_clearances,
        contact_enter_m=contact_enter_m,
        detached_enter_m=detached_enter_m,
    )

    attitude_samples: list[tuple[float, float, float]] = []
    if true_first_contact_s is not None:
        for sample in timed_samples_between(
            loaded["vehicle_odometry"],
            true_first_contact_s,
            hard_gate_validation_end_s,
        ):
            try:
                roll_deg, pitch_deg = quaternion_to_roll_pitch_deg(sample.values[3:7])
            except ValueError:
                continue
            attitude_samples.append((sample.time_s, roll_deg, pitch_deg))
    post_roll_max_abs_deg = (
        max(abs(sample[1]) for sample in attitude_samples) if attitude_samples else math.nan
    )
    post_pitch_max_abs_deg = (
        max(abs(sample[2]) for sample in attitude_samples) if attitude_samples else math.nan
    )
    attitude_divergence_delta_deg = math.nan
    if attitude_samples and true_first_contact_s is not None:
        early = [
            max(abs(roll), abs(pitch))
            for time_s, roll, pitch in attitude_samples
            if time_s <= true_first_contact_s + 2.0
        ]
        late_start_s = max(true_first_contact_s, hard_gate_validation_end_s - 2.0)
        late = [
            max(abs(roll), abs(pitch))
            for time_s, roll, pitch in attitude_samples
            if time_s >= late_start_s
        ]
        if early and late:
            attitude_divergence_delta_deg = max(0.0, max(late) - max(early))

    target_normal_speed_mps = math.nan
    if confirmed_start_s is not None:
        reference_window = [
            sample
            for sample in numeric[HEIGHT_REFERENCE_TOPIC]
            if confirmed_start_s - 0.75 <= sample.time_s <= confirmed_start_s
        ]
        rates = []
        for previous, current in zip(reference_window, reference_window[1:]):
            dt_s = current.time_s - previous.time_s
            if dt_s > 0.0:
                rates.append(abs(current.values[0] - previous.values[0]) / dt_s)
        if rates:
            target_normal_speed_mps = percentile(rates, 0.50)

    land_flag_fields = (
        "ground_contact",
        "maybe_landed",
        "landed",
        "vertical_movement",
        "horizontal_movement",
        "rotational_movement",
    )
    first_px4_flag_times_s = {
        field: next(
            (
                time_s
                for time_s, flags in loaded["land_flags"]
                if final_start_s is not None
                and time_s >= final_start_s
                and flags[field]
            ),
            None,
        )
        for field in land_flag_fields
    }
    px4_flag_true_counts = {
        field: sum(
            1
            for time_s, flags in loaded["land_flags"]
            if final_start_s is not None
            and time_s >= final_start_s
            and flags[field]
        )
        for field in land_flag_fields
    }

    recovery_count = sum(
        state in {"RECOVER_TO_GNSS", "RECOVER_CLIMB"} for state in states
    )
    horizontal_summary = scalar_summary(horizontal_errors_m)
    relative_velocity_summary = scalar_summary(relative_horizontal_speeds_mps)
    tangential_velocity_summary = scalar_summary(tangential_velocity_after_final)
    tangential_position_summary = scalar_summary(tangential_position_after_final)

    gate_metrics = {
        "state_sequence": list(states),
        "candidate_duration_s": candidate_duration_s,
        "hold_duration_s": hold_duration_s,
        "horizontal_error_rmse_m": horizontal_summary["rmse"],
        "horizontal_error_max_m": horizontal_summary["max"],
        "relative_horizontal_velocity_rmse_mps": relative_velocity_summary["rmse"],
        "tangential_velocity_rmse_mps": tangential_velocity_summary["rmse"],
        "touchdown_h_min_m": touchdown_h_min_m,
        "touchdown_h_max_m": touchdown_h_max_m,
        "touchdown_clearance_spread_m": touchdown_spread_m,
        "touchdown_normal_relative_velocity_mps": touchdown_normal_velocity_mps,
        "touchdown_tangential_velocity_mps": touchdown_tangential_velocity_mps,
        "post_touchdown_tangential_slip_m": post_touchdown_slip_m,
        "hold_tangential_velocity_p95_mps": hold_tangential_velocity_p95_mps,
        "post_touchdown_roll_max_abs_deg": post_roll_max_abs_deg,
        "post_touchdown_pitch_max_abs_deg": post_pitch_max_abs_deg,
        "attitude_divergence_delta_deg": attitude_divergence_delta_deg,
        "detach_count": detach_count,
        "secondary_contact_count": secondary_contact_count,
        "recovery_count": recovery_count,
        "time_sync_failure_count": time_sync_failure_count,
        "nan_inf_count": nan_inf_count,
        "nav_land_command_count": nav_land_command_count,
        "disarm_command_count": disarm_command_count,
    }
    gate = tilted_deck_touchdown_gate(gate_metrics)
    return {
        "evaluation_start_s": final_start_s,
        "true_first_contact_s": true_first_contact_s,
        "touchdown_candidate_start_s": candidate_start_s,
        "touchdown_confirmed_s": confirmed_start_s,
        "touchdown_hold_start_s": hold_start_s,
        "bag_end_s": bag_end_s,
        "hard_gate_validation_end_s": hard_gate_validation_end_s,
        "hard_gate_window_definition": (
            "true first contact through the first complete 10.0 s of TOUCHDOWN_HOLD; "
            "later samples are automatic-shutdown observation only"
        ),
        "candidate_duration_s": candidate_duration_s,
        "candidate_entry_count": candidate_entry_count,
        "candidate_repeat_count": max(0, candidate_entry_count - 1),
        "hold_duration_s": hold_duration_s,
        "touchdown_h_min_m": touchdown_h_min_m,
        "touchdown_h_max_m": touchdown_h_max_m,
        "touchdown_clearance_spread_m": touchdown_spread_m,
        "ground_truth_first_contact_h_min_m": gt_first_contact_h_min_m,
        "ground_truth_first_contact_h_max_m": gt_first_contact_h_max_m,
        "ground_truth_first_contact_clearance_spread_m": gt_first_contact_spread_m,
        "ground_truth_confirmed_h_min_m": (
            min(gt_confirm_clearances) if gt_confirm_valid else None
        ),
        "ground_truth_confirmed_h_max_m": (
            max(gt_confirm_clearances) if gt_confirm_valid else None
        ),
        "ground_truth_confirmed_clearance_spread_m": (
            clearance_spread(gt_confirm_clearances) if gt_confirm_valid else None
        ),
        "predicted_first_contact_index": predicted_first_contact_index,
        "ground_truth_first_contact_index": ground_truth_first_contact_index,
        "first_contact_index_consistent": (
            predicted_first_contact_index == ground_truth_first_contact_index
            if predicted_first_contact_index is not None
            and ground_truth_first_contact_index is not None
            else None
        ),
        "other_side_contact_delay_s": other_side_contact_delay_s,
        "touchdown_normal_relative_velocity_mps": touchdown_normal_velocity_mps,
        "touchdown_target_normal_relative_speed_mps": (
            target_normal_speed_mps if math.isfinite(target_normal_speed_mps) else None
        ),
        "touchdown_target_normal_speed_goal_mps": 0.05,
        "touchdown_tangential_velocity_mps": touchdown_tangential_velocity_mps,
        "horizontal_error_m": horizontal_summary,
        "relative_horizontal_velocity_mps": relative_velocity_summary,
        "tangential_position_error_m": tangential_position_summary,
        "tangential_relative_velocity_mps": tangential_velocity_summary,
        "post_touchdown_tangential_slip_m": (
            post_touchdown_slip_m if math.isfinite(post_touchdown_slip_m) else None
        ),
        "hold_tangential_velocity_p95_mps": (
            hold_tangential_velocity_p95_mps
            if math.isfinite(hold_tangential_velocity_p95_mps)
            else None
        ),
        "detach_count": detach_count,
        "secondary_contact_count": secondary_contact_count,
        "full_bag_detach_count_observation_only": full_bag_detach_count,
        "full_bag_secondary_contact_count_observation_only": (
            full_bag_secondary_contact_count
        ),
        "post_touchdown_roll_max_abs_deg": (
            post_roll_max_abs_deg if math.isfinite(post_roll_max_abs_deg) else None
        ),
        "post_touchdown_pitch_max_abs_deg": (
            post_pitch_max_abs_deg if math.isfinite(post_pitch_max_abs_deg) else None
        ),
        "attitude_divergence_delta_deg": (
            attitude_divergence_delta_deg
            if math.isfinite(attitude_divergence_delta_deg)
            else None
        ),
        "attitude_divergence_definition": (
            "max(0, max_abs_roll_pitch_in_final_2s - "
            "max_abs_roll_pitch_in_first_2s_after_true_contact) <= 2 deg"
        ),
        "first_px4_land_flag_times_s": first_px4_flag_times_s,
        "px4_land_flag_true_counts": px4_flag_true_counts,
        "normal_rmse_deg": normal_summary["rmse"],
        "normal_p95_deg": normal_summary["p95"],
        "normal_sign_accuracy": (
            calibration.get("sign_accuracy") if calibration is not None else None
        ),
        "marker_switch_count": marker_switch_count,
        "marker_switch_jump_max_deg": marker_jump_summary["max"],
        "visual_maximum_not_visible_gap_s": visual_maximum_not_visible_gap_s,
        "recovery_count": recovery_count,
        "time_sync_failure_count": time_sync_failure_count,
        "nan_inf_count": nan_inf_count,
        "nav_land_command_count": nav_land_command_count,
        "disarm_command_count": disarm_command_count,
        "touchdown_hold_mode_counts": dict(
            collections.Counter(value for _, value in strings[HOLD_MODE_TOPIC])
        ),
        "touchdown_hold_reason_counts": dict(
            collections.Counter(value for _, value in strings[HOLD_REASON_TOPIC])
        ),
        "touchdown_evidence": scalar_summary(
            [sample.values[0] for sample in numeric[TOUCHDOWN_EVIDENCE_TOPIC]]
        ),
        "controller_visible_estimates": {
            "touchdown_h_min_m": touchdown_h_min_m,
            "touchdown_h_max_m": touchdown_h_max_m,
            "touchdown_clearance_spread_m": touchdown_spread_m,
            "touchdown_normal_relative_velocity_mps": touchdown_normal_velocity_mps,
            "touchdown_tangential_velocity_mps": touchdown_tangential_velocity_mps,
            "target_normal_relative_speed_mps": (
                target_normal_speed_mps if math.isfinite(target_normal_speed_mps) else None
            ),
        },
        "evaluator_only_ground_truth": {
            "true_first_contact_s": true_first_contact_s,
            "first_contact_clearances_m": (
                gt_first_contact_clearances if gt_first_contact_valid else None
            ),
            "first_contact_index": ground_truth_first_contact_index,
            "post_touchdown_tangential_slip_m": (
                post_touchdown_slip_m if math.isfinite(post_touchdown_slip_m) else None
            ),
            "detach_count": detach_count,
            "secondary_contact_count": secondary_contact_count,
            "full_bag_detach_count_observation_only": full_bag_detach_count,
            "full_bag_secondary_contact_count_observation_only": (
                full_bag_secondary_contact_count
            ),
        },
        "hard_pass_gate": gate,
        "observation_only_metrics": {
            "other_side_contact_delay_s": other_side_contact_delay_s,
            "candidate_repeat_count": max(0, candidate_entry_count - 1),
            "touchdown_target_normal_relative_speed_mps": (
                target_normal_speed_mps if math.isfinite(target_normal_speed_mps) else None
            ),
            "touchdown_tangential_velocity_mps": touchdown_tangential_velocity_mps,
            "automatic_shutdown_tail": {
                "bag_end_s": bag_end_s,
                "hard_gate_validation_end_s": hard_gate_validation_end_s,
                "detach_count_after_validation_window": max(
                    0, full_bag_detach_count - detach_count
                ),
                "secondary_contact_count_after_validation_window": max(
                    0, full_bag_secondary_contact_count - secondary_contact_count
                ),
            },
            "px4_movement_bits": {
                key: px4_flag_true_counts[key]
                for key in (
                    "vertical_movement",
                    "horizontal_movement",
                    "rotational_movement",
                )
            },
        },
        "tilted_deck_touchdown_passed": gate["passed"],
    }


def _terminal_stabilization_historical_replay_metrics(
    args: argparse.Namespace,
    loaded: dict[str, Any],
    numeric: dict[str, list[TimedSample]],
    strings: dict[str, list[tuple[float, str]]],
    tilted_deck_touchdown_metrics: Optional[dict[str, Any]],
) -> dict[str, Any]:
    """在历史 Bag 上反事实重放冻结 B1 命令，不把结果当成控制 PASS。"""

    state_samples = strings[STATE_TOPIC]
    state_times = [time_s for time_s, _ in state_samples]
    authorized_states = {
        "FINAL_DESCENT",
        "TOUCHDOWN_CANDIDATE_HOLD",
        "TOUCHDOWN_HOLD",
    }
    gravity_mps2 = 9.80665
    activation_duration_s = 0.50
    deactivation_duration_s = 0.30
    acceleration_slew_mps3 = min(
        0.80, gravity_mps2 * math.radians(4.0)
    )
    marker_jump_gate_deg = 1.0

    current_bias = (0.0, 0.0)
    activation_weight = 0.0
    previous_time_s: Optional[float] = None
    replay_samples: list[dict[str, Any]] = []
    desired_tilt_deg: list[float] = []
    desired_roll_deg: list[float] = []
    desired_pitch_deg: list[float] = []
    actual_tilt_deg: list[float] = []
    tracking_error_deg: list[float] = []
    bias_norm_mps2: list[float] = []
    slew_degps: list[float] = []
    fallback_count = 0
    invalid_numeric_count = 0
    time_reset_count = 0
    activation_states: set[str] = set()
    activation_time_s: Optional[float] = None
    previous_desired_rp: Optional[tuple[float, float]] = None
    previous_desired_time_s: Optional[float] = None

    for sample in numeric[NORMAL_TOPIC]:
        state_index = bisect.bisect_right(state_times, sample.time_s) - 1
        state = state_samples[state_index][1] if state_index >= 0 else None
        dt_s = (
            sample.time_s - previous_time_s
            if previous_time_s is not None
            else 0.02
        )
        previous_time_s = sample.time_s
        if not math.isfinite(dt_s) or dt_s <= 0.0 or dt_s > 0.20:
            current_bias = (0.0, 0.0)
            activation_weight = 0.0
            time_reset_count += 1
            continue

        authorized = state in authorized_states
        target_bias = (0.0, 0.0)
        valid = False
        fallback_reason = "inactive_state"
        if authorized:
            jump = nearest_sample(
                numeric[MARKER_SWITCH_JUMP_TOPIC], sample.time_s, args.max_sync_gap_s
            )
            jump_deg = float(jump[0]) if jump is not None else 0.0
            try:
                if jump_deg > marker_jump_gate_deg:
                    raise ValueError("marker_switch_jump")
                target_bias = terminal_stabilization_counterfactual_bias_from_normal(sample.values)
                valid = True
                fallback_reason = "fresh_historical_visual_normal"
            except ValueError as error:
                fallback_count += 1
                fallback_reason = str(error)

        if valid:
            activation_weight = min(
                1.0, activation_weight + dt_s / activation_duration_s
            )
            if activation_time_s is None:
                activation_time_s = sample.time_s
            if state is not None:
                activation_states.add(state)
        else:
            activation_weight = max(
                0.0, activation_weight - dt_s / deactivation_duration_s
            )
        target_bias = (
            target_bias[0] * activation_weight,
            target_bias[1] * activation_weight,
        )
        delta = (
            target_bias[0] - current_bias[0],
            target_bias[1] - current_bias[1],
        )
        delta_norm = math.hypot(*delta)
        maximum_step = acceleration_slew_mps3 * dt_s
        if delta_norm > maximum_step and delta_norm > 0.0:
            scale = maximum_step / delta_norm
            current_bias = (
                current_bias[0] + delta[0] * scale,
                current_bias[1] + delta[1] * scale,
            )
        else:
            current_bias = target_bias
        if not finite_values(current_bias):
            invalid_numeric_count += 1
            current_bias = (0.0, 0.0)
            activation_weight = 0.0
            continue

        desired_normal = normalize_vector(
            (current_bias[0], current_bias[1], -gravity_mps2)
        )
        desired_roll, desired_pitch, desired_tilt = normal_to_attitude_deg(
            desired_normal
        )
        command_active = authorized or activation_weight > 0.0
        if command_active:
            desired_tilt_deg.append(desired_tilt)
            desired_roll_deg.append(desired_roll)
            desired_pitch_deg.append(desired_pitch)
            bias_norm_mps2.append(math.hypot(*current_bias))
            if previous_desired_rp is not None and previous_desired_time_s is not None:
                command_dt = sample.time_s - previous_desired_time_s
                if command_dt > 0.0:
                    slew_degps.append(
                        math.hypot(
                            desired_roll - previous_desired_rp[0],
                            desired_pitch - previous_desired_rp[1],
                        )
                        / command_dt
                    )
            previous_desired_rp = (desired_roll, desired_pitch)
            previous_desired_time_s = sample.time_s
        else:
            previous_desired_rp = None
            previous_desired_time_s = None

        actual_values = interpolate_sample(
            loaded["vehicle_odometry"], sample.time_s, args.max_sync_gap_s
        ) if command_active else None
        actual_roll = None
        actual_pitch = None
        if actual_values is not None:
            try:
                actual_roll, actual_pitch = quaternion_to_roll_pitch_deg(
                    actual_values[3:7]
                )
            except ValueError:
                invalid_numeric_count += 1
            else:
                actual_tilt_deg.append(math.hypot(actual_roll, actual_pitch))
                tracking_error_deg.append(
                    math.hypot(
                        desired_roll - actual_roll,
                        desired_pitch - actual_pitch,
                    )
                )

        replay_samples.append(
            {
                "time_s": sample.time_s,
                "state": state,
                "valid": valid,
                "reason": fallback_reason,
                "activation_weight": activation_weight,
                "desired_roll_deg": desired_roll,
                "desired_pitch_deg": desired_pitch,
                "acceleration_bias_ned_mps2": [
                    current_bias[0],
                    current_bias[1],
                ],
                "actual_roll_deg": actual_roll,
                "actual_pitch_deg": actual_pitch,
            }
        )

    return {
        "scope": "counterfactual_offline_replay_not_control_validation",
        "activation_time_s": activation_time_s,
        "activation_states": sorted(activation_states),
        "total_visual_sample_count": len(replay_samples),
        "active_command_sample_count": len(desired_tilt_deg),
        "desired_roll_deg": scalar_summary(desired_roll_deg),
        "desired_pitch_deg": scalar_summary(desired_pitch_deg),
        "desired_tilt_deg": scalar_summary(desired_tilt_deg),
        "actual_tilt_deg": scalar_summary(actual_tilt_deg),
        "attitude_tracking_error_deg": scalar_summary(tracking_error_deg),
        "command_slew_degps": scalar_summary(slew_degps),
        "acceleration_bias_norm_mps2": scalar_summary(bias_norm_mps2),
        "fallback_count": fallback_count,
        "time_reset_count": time_reset_count,
        "invalid_numeric_count": invalid_numeric_count,
        "failure_explanation": terminal_stabilization_replay_failure_explanation(tilted_deck_touchdown_metrics),
        "ground_truth_control_isolation": (
            "counterfactual command uses recorded visual normal and landing state only; "
            "Ground Truth remains limited to offline fixed-tilt touchdown metric comparison"
        ),
        "control_pass_claimed": False,
        "preview_samples": [
            sample
            for sample in replay_samples
            if sample["state"] in authorized_states
        ][:5],
    }


def _terminal_stabilization_metrics(
    args: argparse.Namespace,
    loaded: dict[str, Any],
    numeric: dict[str, list[TimedSample]],
    strings: dict[str, list[tuple[float, str]]],
    total_invalid_numeric: int,
) -> dict[str, Any]:
    """计算 terminal contact stabilization 诊断、命令连续性、姿态跟踪和顺应指标。"""

    mode_samples = strings[TERMINAL_MODE_TOPIC]
    state_samples = strings[STATE_TOPIC]

    def latest_string(samples: Sequence[tuple[float, str]], time_s: float) -> Optional[str]:
        if not samples:
            return None
        times = [sample_time for sample_time, _ in samples]
        index = bisect.bisect_right(times, time_s) - 1
        return samples[index][1] if index >= 0 else None

    def nearest_age(samples: Sequence[TimedSample], time_s: float) -> Optional[float]:
        if not samples:
            return None
        times = [sample.time_s for sample in samples]
        index = bisect.bisect_left(times, time_s)
        candidates = []
        if index < len(samples):
            candidates.append(abs(samples[index].time_s - time_s))
        if index > 0:
            candidates.append(abs(samples[index - 1].time_s - time_s))
        return min(candidates) if candidates else None

    active_modes = {"ACTIVE", "REHEARSAL"}
    diagnostic_active_times = [
        time_s for time_s, value in mode_samples if value in active_modes
    ]
    applied_times = [
        sample.time_s
        for sample in numeric[TERMINAL_ENABLED_TOPIC]
        if sample.values[0] >= 0.5
    ]
    activation_times = diagnostic_active_times or applied_times
    activation_time_s = min(activation_times) if activation_times else None
    activation_states = sorted(
        {
            state
            for time_s in activation_times
            for state in [latest_string(state_samples, time_s)]
            if state is not None
        }
    )

    def active_at(time_s: float) -> bool:
        return latest_string(mode_samples, time_s) in active_modes

    desired_attitude = [
        sample
        for sample in numeric[TERMINAL_DESIRED_ATTITUDE_TOPIC]
        if active_at(sample.time_s)
    ]
    desired_normal_samples = [
        sample
        for sample in numeric[TERMINAL_DESIRED_NORMAL_TOPIC]
        if active_at(sample.time_s)
    ]
    # 冻结的命令倾角是 body-z/甲板法向相对 NED vertical 的几何夹角。
    # 对同时含有 roll/pitch 的姿态，hypot(roll,pitch) 不是该夹角，并会在
    # 2.5° 限幅处产生约 5e-5° 的假超限。直接由单位法向计算。
    desired_tilt_deg = []
    for sample in desired_normal_samples:
        normal_norm = math.sqrt(sum(value * value for value in sample.values))
        if normal_norm <= 0.0 or not math.isfinite(normal_norm):
            continue
        desired_tilt_deg.append(
            math.degrees(
                math.acos(
                    max(-1.0, min(1.0, -sample.values[2] / normal_norm))
                )
            )
        )
    command_slew_degps: list[float] = []
    for previous, current in zip(desired_attitude, desired_attitude[1:]):
        dt_s = current.time_s - previous.time_s
        if dt_s <= 0.0 or not math.isfinite(dt_s):
            continue
        delta = math.hypot(
            current.values[0] - previous.values[0],
            current.values[1] - previous.values[1],
        )
        command_slew_degps.append(math.degrees(delta) / dt_s)

    actual_attitude_samples = numeric[TERMINAL_ACTUAL_ATTITUDE_TOPIC]
    tracking_errors_deg: list[float] = []
    tracking_sync_failures = 0
    tracking_states: set[str] = set()
    for sample in desired_attitude:
        tracking_state = latest_string(state_samples, sample.time_s)
        if not terminal_stabilization_tracking_state_authorized(args.terminal_stabilization_mode, tracking_state):
            continue
        if tracking_state is not None:
            tracking_states.add(tracking_state)
        actual = nearest_sample(actual_attitude_samples, sample.time_s, args.max_sync_gap_s)
        if actual is None:
            tracking_sync_failures += 1
            continue
        tracking_errors_deg.append(
            math.degrees(
                math.hypot(
                    sample.values[0] - actual[0],
                    sample.values[1] - actual[1],
                )
            )
        )

    actual_tilt_deg = [
        math.degrees(math.hypot(sample.values[0], sample.values[1]))
        for sample in actual_attitude_samples
        if activation_time_s is not None and sample.time_s >= activation_time_s
    ]
    actual_angular_rate_degps = [
        math.degrees(math.sqrt(sum(value * value for value in sample.values)))
        for sample in numeric[VEHICLE_ODOMETRY_TOPIC + ":angular_velocity"]
        if activation_time_s is not None and sample.time_s >= activation_time_s
    ]
    acceleration_bias_norms = [
        math.hypot(sample.values[0], sample.values[1])
        for sample in numeric[TERMINAL_ACCELERATION_BIAS_TOPIC]
        if active_at(sample.time_s)
    ]
    combined_acceleration_norms = [
        math.hypot(sample.values[0], sample.values[1])
        for sample in numeric[TERMINAL_COMBINED_ACCELERATION_TOPIC]
        if active_at(sample.time_s)
    ]

    normal_freshness_ages_s: list[float] = []
    normal_freshness_sync_failures = 0
    for sample in numeric[TERMINAL_DESIRED_NORMAL_TOPIC]:
        if not active_at(sample.time_s):
            continue
        age = nearest_age(numeric[NORMAL_TOPIC], sample.time_s)
        if age is None:
            normal_freshness_sync_failures += 1
        else:
            normal_freshness_ages_s.append(age)

    fallback_count = 0
    previous_fallback = False
    fallback_count_after_activation = 0
    for time_s, value in mode_samples:
        current_fallback = value == "FALLBACK"
        if current_fallback and not previous_fallback:
            fallback_count += 1
            if activation_time_s is not None and time_s >= activation_time_s:
                fallback_count_after_activation += 1
        previous_fallback = current_fallback

    divergence_protection_count = 0
    previous_divergence = False
    for _, value in strings[TERMINAL_DIVERGENCE_TOPIC]:
        current_divergence = value.startswith("sustained_")
        if current_divergence and not previous_divergence:
            divergence_protection_count += 1
        previous_divergence = current_divergence

    def movement_metrics(samples: Sequence[TimedSample]) -> dict[str, Any]:
        filtered = [
            sample
            for sample in samples
            if activation_time_s is not None and sample.time_s >= activation_time_s
        ]
        steps = [
            math.hypot(
                current.values[0] - previous.values[0],
                current.values[1] - previous.values[1],
            )
            for previous, current in zip(filtered, filtered[1:])
        ]
        return {
            "sample_count": len(filtered),
            "total_m": sum(steps),
            "maximum_step_m": max(steps) if steps else 0.0,
            "net_m": (
                math.hypot(
                    filtered[-1].values[0] - filtered[0].values[0],
                    filtered[-1].values[1] - filtered[0].values[1],
                )
                if len(filtered) >= 2
                else 0.0
            ),
        }

    active_odometry = [
        sample
        for sample in loaded["vehicle_odometry"]
        if activation_time_s is not None and sample.time_s >= activation_time_s
    ]
    rehearsal_drift_max_m = None
    if active_odometry:
        start_position = active_odometry[0].values
        rehearsal_drift_max_m = max(
            math.hypot(
                sample.values[0] - start_position[0],
                sample.values[1] - start_position[1],
            )
            for sample in active_odometry
        )

    desired_summary = scalar_summary(desired_tilt_deg)
    tracking_summary = scalar_summary(tracking_errors_deg)
    slew_summary = scalar_summary(command_slew_degps)
    combined_summary = scalar_summary(combined_acceleration_norms)
    terminal_stabilization_gate_metrics = {
        "mode": args.terminal_stabilization_mode,
        "activation_sample_count": len(activation_times),
        "command_tilt_max_deg": desired_summary["max"],
        "command_tilt_slew_max_degps": slew_summary["max"],
        "combined_acceleration_max_mps2": combined_summary["max"],
        "attitude_tracking_error_p95_deg": tracking_summary["p95"],
        "rehearsal_horizontal_drift_max_m": rehearsal_drift_max_m,
        "fallback_count_after_activation": fallback_count_after_activation,
        "divergence_protection_count": divergence_protection_count,
        "time_sync_failure_count": (
            tracking_sync_failures + normal_freshness_sync_failures
        ),
        "nan_inf_count": total_invalid_numeric,
    }
    gate = terminal_stabilization_gate(terminal_stabilization_gate_metrics)
    return {
        "mode": args.terminal_stabilization_mode,
        "activation_time_s": activation_time_s,
        "activation_sample_count": len(activation_times),
        "applied_sample_count": len(applied_times),
        "activation_states": activation_states,
        "desired_tilt_deg": desired_summary,
        "actual_tilt_deg": scalar_summary(actual_tilt_deg),
        "attitude_tracking_error_deg": tracking_summary,
        "attitude_tracking_states": sorted(tracking_states),
        "command_slew_degps": slew_summary,
        "actual_angular_rate_degps": scalar_summary(actual_angular_rate_degps),
        "acceleration_bias_norm_mps2": scalar_summary(acceleration_bias_norms),
        "combined_acceleration_norm_mps2": combined_summary,
        "normal_freshness_age_s": scalar_summary(normal_freshness_ages_s),
        "fallback_count": fallback_count,
        "fallback_count_after_activation": fallback_count_after_activation,
        "contact_anchor_movement": movement_metrics(
            numeric[TERMINAL_CONTACT_ANCHOR_TOPIC]
        ),
        "compliant_target_movement": movement_metrics(
            numeric[TERMINAL_COMPLIANT_TARGET_TOPIC]
        ),
        "divergence_protection_count": divergence_protection_count,
        "rehearsal_horizontal_drift_max_m": rehearsal_drift_max_m,
        "tracking_sync_failure_count": tracking_sync_failures,
        "normal_freshness_sync_failure_count": normal_freshness_sync_failures,
        "hard_pass_gate": gate,
        "terminal_stabilization_passed": gate["passed"],
        "ground_truth_control_isolation": (
            "all terminal contact stabilization control diagnostics are generated online from visual/PX4 data; "
            "Ground Truth remains evaluator-only"
        ),
    }


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    loaded = _load_samples(args)
    numeric: dict[str, list[TimedSample]] = loaded["numeric"]
    strings: dict[str, list[tuple[float, str]]] = loaded["strings"]
    invalid_counts: collections.Counter[str] = loaded["invalid_counts"]

    aligned_count = 0
    time_sync_failures = 0
    consistency_failures = 0
    index_mismatches = 0
    aligned_spreads: list[float] = []
    aligned_minimums: list[float] = []
    aligned_maximums: list[float] = []
    first_contact_counts: collections.Counter[int] = collections.Counter()

    for body_sample in numeric[BODY_CLEARANCE_TOPIC]:
        time_s = body_sample.time_s
        skid = nearest_sample(numeric[SKID_CLEARANCES_TOPIC], time_s, args.max_sync_gap_s)
        minimum = nearest_sample(numeric[MIN_CLEARANCE_TOPIC], time_s, args.max_sync_gap_s)
        maximum = nearest_sample(numeric[MAX_CLEARANCE_TOPIC], time_s, args.max_sync_gap_s)
        spread = nearest_sample(numeric[SPREAD_TOPIC], time_s, args.max_sync_gap_s)
        index = nearest_sample(numeric[FIRST_CONTACT_TOPIC], time_s, args.max_sync_gap_s)
        normal = nearest_sample(numeric[NORMAL_TOPIC], time_s, args.max_sync_gap_s)
        tangent = nearest_sample(
            numeric[TANGENTIAL_POSITION_TOPIC], time_s, args.max_sync_gap_s
        )
        if any(value is None for value in (skid, minimum, maximum, spread, index, normal, tangent)):
            time_sync_failures += 1
            continue
        assert skid is not None and minimum is not None and maximum is not None
        assert spread is not None and index is not None
        expected_minimum = min(skid)
        expected_maximum = max(skid)
        expected_spread = clearance_spread(skid)
        expected_index = contact_point_index(skid)
        if (
            abs(minimum[0] - expected_minimum) > args.consistency_tolerance_m
            or abs(maximum[0] - expected_maximum) > args.consistency_tolerance_m
            or abs(spread[0] - expected_spread) > args.consistency_tolerance_m
        ):
            consistency_failures += 1
        reported_index = int(round(index[0]))
        if reported_index != expected_index:
            index_mismatches += 1
        aligned_count += 1
        aligned_minimums.append(minimum[0])
        aligned_maximums.append(maximum[0])
        aligned_spreads.append(spread[0])
        first_contact_counts[reported_index] += 1

    skid_summaries = [
        scalar_summary([sample.values[index] for sample in numeric[SKID_CLEARANCES_TOPIC]])
        for index in range(4)
    ]
    tangential_position_norms = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[TANGENTIAL_POSITION_TOPIC]
    ]
    tangential_velocity_norms = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[TANGENTIAL_VELOCITY_TOPIC]
    ]

    normal_errors_deg: list[float] = []
    estimated_roll_deg: list[float] = []
    estimated_pitch_deg: list[float] = []
    estimated_tilt_deg: list[float] = []
    truth_roll_deg: list[float] = []
    truth_pitch_deg: list[float] = []
    truth_tilt_deg: list[float] = []
    signed_axis_errors_deg: list[float] = []
    sign_correct_samples: list[bool] = []
    normal_truth_sync_failures = 0
    scenario_attitude = expected_attitude_deg(args.scenario) if args.scenario else None

    for sample in numeric[NORMAL_TOPIC]:
        estimated_attitude = normal_to_attitude_deg(sample.values)
        estimated_roll_deg.append(estimated_attitude[0])
        estimated_pitch_deg.append(estimated_attitude[1])
        estimated_tilt_deg.append(estimated_attitude[2])
        truth = interpolate_sample(numeric[GROUND_TRUTH_TOPIC], sample.time_s, args.max_sync_gap_s)
        if truth is None:
            if args.require_ground_truth or args.scenario is not None:
                normal_truth_sync_failures += 1
            continue
        truth_attitude = normal_to_attitude_deg(truth)
        truth_roll_deg.append(truth_attitude[0])
        truth_pitch_deg.append(truth_attitude[1])
        truth_tilt_deg.append(truth_attitude[2])
        normal_errors_deg.append(normal_angle_error_deg(sample.values, truth))

        if scenario_attitude is None:
            continue
        calibration_axis = calibration_axis_from_truth(truth_attitude[:2])
        if calibration_axis == "roll":
            estimated_axis = estimated_attitude[0]
            truth_axis = truth_attitude[0]
            sign_correct_samples.append(estimated_axis * truth_axis > 0.0)
        elif calibration_axis == "pitch":
            estimated_axis = estimated_attitude[1]
            truth_axis = truth_attitude[1]
            sign_correct_samples.append(estimated_axis * truth_axis > 0.0)
        else:
            estimated_axis = estimated_attitude[2]
            truth_axis = truth_attitude[2]
            sign_correct_samples.append(True)
        error = estimated_axis - truth_axis
        signed_axis_errors_deg.append(error)

    time_sync_failures += normal_truth_sync_failures

    def mean_normal(samples: Sequence[TimedSample]) -> Optional[tuple[float, float, float]]:
        if not samples:
            return None
        averaged = tuple(
            sum(sample.values[axis] for sample in samples) / len(samples)
            for axis in range(3)
        )
        return normalize_vector(averaged)

    estimated_mean_normal = mean_normal(numeric[NORMAL_TOPIC])
    truth_mean_normal = mean_normal(numeric[GROUND_TRUTH_TOPIC])

    active_normals_by_id: dict[int, list[tuple[float, ...]]] = collections.defaultdict(list)
    active_marker_ids: list[int] = []
    for normal_sample in numeric[NORMAL_TOPIC]:
        marker = nearest_sample(
            numeric[ACTIVE_MARKER_TOPIC], normal_sample.time_s, args.max_sync_gap_s
        )
        if marker is None:
            time_sync_failures += 1
            continue
        marker_id = int(round(marker[0]))
        if marker_id not in range(4):
            continue
        active_marker_ids.append(marker_id)
        active_normals_by_id[marker_id].append(normal_sample.values)

    all_normals_by_id: dict[int, list[tuple[float, ...]]] = collections.defaultdict(list)
    marker_mask_sync_failures = 0
    for sample in numeric[MARKER_NORMALS_TOPIC]:
        mask = nearest_sample(
            numeric[MARKER_NORMAL_VALID_MASK_TOPIC], sample.time_s, args.max_sync_gap_s
        )
        if mask is None:
            marker_mask_sync_failures += 1
            continue
        valid_mask = int(round(mask[0]))
        for marker_id in range(4):
            if valid_mask & (1 << marker_id):
                normal = sample.values[3 * marker_id : 3 * marker_id + 3]
                try:
                    all_normals_by_id[marker_id].append(normalize_vector(normal))
                except ValueError:
                    invalid_counts[MARKER_NORMALS_TOPIC] += 1
    time_sync_failures += marker_mask_sync_failures

    mean_truth_roll_pitch = (
        (
            sum(truth_roll_deg) / len(truth_roll_deg),
            sum(truth_pitch_deg) / len(truth_pitch_deg),
        )
        if truth_roll_deg and truth_pitch_deg
        else None
    )
    calibration_axis = (
        calibration_axis_from_truth(mean_truth_roll_pitch)
        if mean_truth_roll_pitch is not None
        else None
    )

    marker_switch_count = sum(
        current != previous
        for previous, current in zip(active_marker_ids, active_marker_ids[1:])
    )
    marker_switch_jumps = [
        sample.values[0] for sample in numeric[MARKER_SWITCH_JUMP_TOPIC]
    ]
    calibration = (
        calibration_gate(
            signed_axis_errors_deg,
            normal_errors_deg,
            sign_correct_samples,
            marker_switch_jumps,
            require_signed_mean=calibration_axis in {"roll", "pitch"},
            require_sign_accuracy=calibration_axis in {"roll", "pitch"},
        )
        if args.scenario is not None
        else None
    )

    gt_positions_ned: list[TimedSample] = []
    valid_reference = next(
        (
            message
            for _, message in loaded["local_positions"]
            if bool(message.xy_global)
            and bool(message.z_global)
            and finite_values((message.ref_lat, message.ref_lon, message.ref_alt))
        ),
        None,
    )
    if valid_reference is not None:
        local_origin = GeodeticPosition(
            float(valid_reference.ref_lat),
            float(valid_reference.ref_lon),
            float(valid_reference.ref_alt),
        )
        world_origin = GeodeticPosition(
            args.world_origin_latitude,
            args.world_origin_longitude,
            args.world_origin_altitude,
        )
        gt_positions_ned = [
            TimedSample(
                sample.time_s,
                tuple(world_enu_to_local_ned(sample.values, world_origin, local_origin)),
            )
            for sample in loaded["ground_truth_positions_enu"]
        ]

    gt_geometry_aligned_count = 0
    gt_geometry_sync_failures = 0
    gt_body_clearances: list[float] = []
    gt_skid_clearances: list[list[float]] = [[], [], [], []]
    gt_minimum_clearance_samples: list[TimedSample] = []
    gt_skid_clearance_samples: list[list[TimedSample]] = [[], [], [], []]
    gt_spreads: list[float] = []
    gt_first_contact_counts: collections.Counter[int] = collections.Counter()
    body_clearance_errors: list[float] = []
    skid_clearance_errors: list[float] = []
    spread_errors: list[float] = []
    contact_points_body_frd = (
        (-0.125, -0.132, 0.227),
        (0.125, -0.132, 0.227),
        (-0.125, 0.132, 0.227),
        (0.125, 0.132, 0.227),
    )
    if gt_positions_ned:
        for body_sample in numeric[BODY_CLEARANCE_TOPIC]:
            time_s = body_sample.time_s
            truth_normal = interpolate_sample(
                numeric[GROUND_TRUTH_TOPIC], time_s, args.max_sync_gap_s
            )
            deck_position = interpolate_sample(gt_positions_ned, time_s, args.max_sync_gap_s)
            odometry = interpolate_sample(
                loaded["vehicle_odometry"], time_s, args.max_sync_gap_s
            )
            reported_skids = nearest_sample(
                numeric[SKID_CLEARANCES_TOPIC], time_s, args.max_sync_gap_s
            )
            reported_spread = nearest_sample(
                numeric[SPREAD_TOPIC], time_s, args.max_sync_gap_s
            )
            if any(
                value is None
                for value in (
                    truth_normal,
                    deck_position,
                    odometry,
                    reported_skids,
                    reported_spread,
                )
            ):
                gt_geometry_sync_failures += 1
                continue
            assert truth_normal is not None and deck_position is not None
            assert odometry is not None and reported_skids is not None
            assert reported_spread is not None
            uav_position = odometry[:3]
            quaternion = odometry[3:7]
            body_gap = sum(
                truth_normal[axis] * (uav_position[axis] - deck_position[axis])
                for axis in range(3)
            )
            skids = []
            for point in contact_points_body_frd:
                arm_ned = _quaternion_rotate_wxyz(quaternion, point)
                contact_position = tuple(
                    uav_position[axis] + arm_ned[axis] for axis in range(3)
                )
                skids.append(
                    sum(
                        truth_normal[axis]
                        * (contact_position[axis] - deck_position[axis])
                        for axis in range(3)
                    )
                )
            truth_spread = clearance_spread(skids)
            gt_geometry_aligned_count += 1
            gt_body_clearances.append(body_gap)
            gt_minimum_clearance_samples.append(TimedSample(time_s, (min(skids),)))
            for index, value in enumerate(skids):
                gt_skid_clearances[index].append(value)
                gt_skid_clearance_samples[index].append(TimedSample(time_s, (value,)))
                skid_clearance_errors.append(reported_skids[index] - value)
            gt_spreads.append(truth_spread)
            gt_first_contact_counts[contact_point_index(skids)] += 1
            body_clearance_errors.append(body_sample.values[0] - body_gap)
            spread_errors.append(reported_spread[0] - truth_spread)
    elif args.scenario is not None:
        gt_geometry_sync_failures = len(numeric[BODY_CLEARANCE_TOPIC])
    time_sync_failures += gt_geometry_sync_failures

    states = state_sequence(strings[STATE_TOPIC])
    state_counts = dict(collections.Counter(state for _, state in strings[STATE_TOPIC]))
    all_times = [
        sample.time_s
        for samples in numeric.values()
        for sample in samples
    ]
    all_times.extend(sample.time_s for sample in loaded["vehicle_odometry"])
    all_times.extend(time_s for samples in strings.values() for time_s, _ in samples)
    bag_end_s = max(all_times) if all_times else 0.0
    state_durations_s: collections.Counter[str] = collections.Counter()
    state_samples = strings[STATE_TOPIC]
    for index, (time_s, state) in enumerate(state_samples):
        next_time_s = state_samples[index + 1][0] if index + 1 < len(state_samples) else bag_end_s
        state_durations_s[state] += max(0.0, next_time_s - time_s)

    forbidden_states = (
        []
        if args.fixed_tilt_safe_descent or args.tilted_deck_touchdown
        else [state for state in states if state in FORBIDDEN_SAFE_ALTITUDE_STATES]
    )
    fixed_tilt_safe_descent_forbidden_states = [state for state in states if state in FIXED_TILT_SAFE_DESCENT_FORBIDDEN_STATES]
    tracking_start_s = next(
        (
            time_s
            for time_s, state in state_samples
            if state in {"TRACK_TARGET", "WAIT_LANDING_WINDOW"}
        ),
        None,
    )
    descent_start_s = next(
        (time_s for time_s, state in state_samples if state == "DESCEND"),
        None,
    )
    evaluation_start_s = (
        next((time_s for time_s, state in state_samples if state == "FINAL_DESCENT"), None)
        if args.tilted_deck_touchdown
        else (descent_start_s if args.fixed_tilt_safe_descent else tracking_start_s)
    )
    trajectory_nonfinite_after_tracking = nonfinite_times_at_or_after(
        loaded["trajectory_nonfinite_position_times"], tracking_start_s
    )
    target_nonfinite_after_tracking = nonfinite_times_at_or_after(
        loaded["target_nonfinite_position_times"], tracking_start_s
    )
    target_z_after_tracking = [
        sample.values[2]
        for sample in numeric[TARGET_TOPIC]
        if tracking_start_s is not None and sample.time_s >= tracking_start_s
    ]
    trajectory_z_after_tracking = [
        sample.values[2]
        for sample in numeric[TRAJECTORY_TOPIC]
        if tracking_start_s is not None and sample.time_s >= tracking_start_s
    ]
    target_z_span = (
        max(target_z_after_tracking) - min(target_z_after_tracking)
        if target_z_after_tracking
        else None
    )
    trajectory_z_span = (
        max(trajectory_z_after_tracking) - min(trajectory_z_after_tracking)
        if trajectory_z_after_tracking
        else None
    )
    nav_land_commands = sum(
        command == NAV_LAND_COMMAND for command, _ in loaded["commands"]
    )
    disarm_commands = sum(
        command == ARM_DISARM_COMMAND and param1 < 0.5
        for command, param1 in loaded["commands"]
    )

    fixed_tilt_safe_descent_time_sync_failures = 0
    horizontal_errors_m: list[float] = []
    if args.fixed_tilt_safe_descent and descent_start_s is not None:
        for odometry_sample in loaded["vehicle_odometry"]:
            if odometry_sample.time_s < descent_start_s:
                continue
            target = nearest_sample(
                numeric[TARGET_TOPIC], odometry_sample.time_s, args.max_sync_gap_s
            )
            if target is None:
                fixed_tilt_safe_descent_time_sync_failures += 1
                continue
            dx = odometry_sample.values[0] - target[0]
            dy = odometry_sample.values[1] - target[1]
            horizontal_errors_m.append(math.hypot(dx, dy))
    time_sync_failures += fixed_tilt_safe_descent_time_sync_failures

    def values_after(topic: str, start_s: Optional[float], component: int = 0) -> list[float]:
        if start_s is None:
            return []
        return [
            sample.values[component]
            for sample in numeric[topic]
            if sample.time_s >= start_s
        ]

    fixed_tilt_safe_descent_relative_heights = values_after(RELATIVE_HEIGHT_TOPIC, descent_start_s)
    fixed_tilt_safe_descent_height_references = values_after(HEIGHT_REFERENCE_TOPIC, descent_start_s)
    fixed_tilt_safe_descent_tangential_position_norms = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[TANGENTIAL_POSITION_TOPIC]
        if descent_start_s is not None and sample.time_s >= descent_start_s
    ]
    fixed_tilt_safe_descent_tangential_velocity_norms = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[TANGENTIAL_VELOCITY_TOPIC]
        if descent_start_s is not None and sample.time_s >= descent_start_s
    ]
    fixed_tilt_safe_descent_gt_minimum_clearances = [
        sample.values[0]
        for sample in gt_minimum_clearance_samples
        if descent_start_s is not None and sample.time_s >= descent_start_s
    ]
    fixed_tilt_safe_descent_gt_skid_clearances = [
        [
            sample.values[0]
            for sample in samples
            if descent_start_s is not None and sample.time_s >= descent_start_s
        ]
        for samples in gt_skid_clearance_samples
    ]
    ground_truth_contact_count = sum(value <= 0.005 for value in fixed_tilt_safe_descent_gt_minimum_clearances)
    ground_truth_penetration_count = sum(value < 0.0 for value in fixed_tilt_safe_descent_gt_minimum_clearances)

    visible_samples = [
        sample
        for sample in numeric[ARUCO_VISIBLE_TOPIC]
        if descent_start_s is not None and sample.time_s >= descent_start_s
    ]
    visible_count = sum(sample.values[0] >= 0.5 for sample in visible_samples)
    maximum_not_visible_gap_s = 0.0
    gap_start_s: Optional[float] = None
    for sample in visible_samples:
        if sample.values[0] < 0.5 and gap_start_s is None:
            gap_start_s = sample.time_s
        elif sample.values[0] >= 0.5 and gap_start_s is not None:
            maximum_not_visible_gap_s = max(
                maximum_not_visible_gap_s, sample.time_s - gap_start_s
            )
            gap_start_s = None
    if gap_start_s is not None and visible_samples:
        maximum_not_visible_gap_s = max(
            maximum_not_visible_gap_s, visible_samples[-1].time_s - gap_start_s
        )

    status_values = [value for _, value in strings[STATUS_TOPIC]]
    status_counts = dict(collections.Counter(status_values))
    invalid_status_count = sum(value.startswith("INVALID") for value in status_values)
    ok_status_count = sum(value.startswith("OK_SHADOW") for value in status_values)
    total_invalid_numeric = sum(invalid_counts.values())
    required_message_topics = {
        NORMAL_TOPIC,
        BODY_CLEARANCE_TOPIC,
        SKID_CLEARANCES_TOPIC,
        MIN_CLEARANCE_TOPIC,
        MAX_CLEARANCE_TOPIC,
        SPREAD_TOPIC,
        FIRST_CONTACT_TOPIC,
        TANGENTIAL_POSITION_TOPIC,
    }
    if args.scenario is not None:
        required_message_topics |= {
            GROUND_TRUTH_TOPIC,
            ACTIVE_MARKER_TOPIC,
            ESTIMATED_ATTITUDE_TOPIC,
            MARKER_NORMALS_TOPIC,
            MARKER_NORMAL_VALID_MASK_TOPIC,
            STATE_TOPIC,
            TARGET_TOPIC,
            TRAJECTORY_TOPIC,
        }
    if args.fixed_tilt_safe_descent or args.tilted_deck_touchdown:
        required_message_topics |= {
            RELATIVE_HEIGHT_TOPIC,
            HEIGHT_REFERENCE_TOPIC,
            ARUCO_VISIBLE_TOPIC,
        }
    if args.tilted_deck_touchdown:
        required_message_topics |= TILTED_DECK_TOUCHDOWN_REQUIRED_TOPICS
    if args.terminal_stabilization:
        required_message_topics |= {
            TERMINAL_ENABLED_TOPIC,
            TERMINAL_MODE_TOPIC,
            TERMINAL_REASON_TOPIC,
            TERMINAL_DESIRED_NORMAL_TOPIC,
            TERMINAL_DESIRED_ATTITUDE_TOPIC,
            TERMINAL_ACTUAL_ATTITUDE_TOPIC,
            TERMINAL_ATTITUDE_ERROR_TOPIC,
            TERMINAL_ACCELERATION_BIAS_TOPIC,
            TERMINAL_COMBINED_ACCELERATION_TOPIC,
            TERMINAL_DIVERGENCE_TOPIC,
        }
        if args.terminal_stabilization_mode == "active":
            required_message_topics |= {
                TERMINAL_CONTACT_ANCHOR_TOPIC,
                TERMINAL_COMPLIANT_TARGET_TOPIC,
            }
    topics_without_messages = sorted(
        topic
        for topic in required_message_topics
        if not topic_has_messages(topic, numeric, strings, loaded)
    )
    data_valid = (
        aligned_count > 0
        and not topics_without_messages
        and total_invalid_numeric == 0
        and not trajectory_nonfinite_after_tracking
        and not target_nonfinite_after_tracking
        and time_sync_failures == 0
        and consistency_failures == 0
        and index_mismatches == 0
        and ok_status_count > 0
    )
    safety_passed = (
        not forbidden_states
        and nav_land_commands == 0
        and disarm_commands == 0
        and tracking_start_s is not None
    )
    fixed_tilt_safe_altitude_passed = bool(
        args.scenario is not None
        and not args.fixed_tilt_safe_descent
        and not args.tilted_deck_touchdown
        and data_valid
        and safety_passed
        and calibration is not None
        and calibration["passed"]
        and bool(normal_errors_deg)
        and gt_geometry_aligned_count > 0
    )

    horizontal_summary = scalar_summary(horizontal_errors_m)
    normal_summary = scalar_summary(normal_errors_deg)
    marker_jump_summary = scalar_summary(marker_switch_jumps)
    fixed_tilt_safe_descent_gate_metrics = {
        "state_sequence": states,
        "test_height_hold_duration_s": state_durations_s.get("TEST_HEIGHT_HOLD", 0.0),
        "ground_truth_minimum_skid_clearance_m": (
            min(fixed_tilt_safe_descent_gt_minimum_clearances) if fixed_tilt_safe_descent_gt_minimum_clearances else None
        ),
        "ground_truth_contact_count": ground_truth_contact_count,
        "ground_truth_penetration_count": ground_truth_penetration_count,
        "horizontal_error_rmse_m": horizontal_summary["rmse"],
        "horizontal_error_max_m": horizontal_summary["max"],
        "normal_rmse_deg": normal_summary["rmse"],
        "normal_p95_deg": normal_summary["p95"],
        "sign_accuracy": (
            calibration.get("sign_accuracy") if calibration is not None else None
        ),
        "marker_switch_jump_max_deg": marker_jump_summary["max"],
        "time_sync_failure_count": time_sync_failures,
        "nan_inf_count": total_invalid_numeric,
        "nav_land_command_count": nav_land_commands,
        "disarm_command_count": disarm_commands,
    }
    fixed_tilt_safe_descent_gate = (
        fixed_tilt_safe_descent_gate(fixed_tilt_safe_descent_gate_metrics)
        if args.fixed_tilt_safe_descent
        else None
    )
    fixed_tilt_safe_descent_passed = bool(
        args.fixed_tilt_safe_descent
        and data_valid
        and fixed_tilt_safe_descent_gate is not None
        and fixed_tilt_safe_descent_gate["passed"]
        and bool(fixed_tilt_safe_descent_gt_minimum_clearances)
        and bool(horizontal_errors_m)
    )
    tilted_deck_touchdown_metrics = (
        _tilted_deck_touchdown_metrics(
            args,
            loaded,
            numeric,
            strings,
            gt_positions_ned,
            gt_minimum_clearance_samples,
            gt_skid_clearance_samples,
            states,
            state_durations_s,
            bag_end_s,
            time_sync_failures,
            total_invalid_numeric,
            nav_land_commands,
            disarm_commands,
            normal_summary,
            calibration,
            marker_switch_count,
            marker_jump_summary,
            maximum_not_visible_gap_s,
        )
        if args.tilted_deck_touchdown
        else None
    )
    tilted_deck_touchdown_passed = bool(
        args.tilted_deck_touchdown
        and data_valid
        and tilted_deck_touchdown_metrics is not None
        and tilted_deck_touchdown_metrics["tilted_deck_touchdown_passed"]
    )
    terminal_stabilization_replay_metrics = (
        _terminal_stabilization_historical_replay_metrics(
            args,
            loaded,
            numeric,
            strings,
            tilted_deck_touchdown_metrics,
        )
        if args.terminal_stabilization_replay
        else None
    )
    terminal_stabilization_metrics = (
        _terminal_stabilization_metrics(
            args,
            loaded,
            numeric,
            strings,
            total_invalid_numeric,
        )
        if args.terminal_stabilization
        else None
    )
    terminal_stabilization_passed = bool(
        args.terminal_stabilization
        and data_valid
        and terminal_stabilization_metrics is not None
        and terminal_stabilization_metrics["terminal_stabilization_passed"]
    )

    gt_angular_speed = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[GROUND_TRUTH_TOPIC + ":angular_velocity_body"]
    ]
    gt_linear_speed = [
        math.sqrt(sum(value * value for value in sample.values))
        for sample in numeric[GROUND_TRUTH_TOPIC + ":linear_velocity_enu"]
    ]
    estimated_attitude_topic_deg = {
        "roll": scalar_summary(
            [math.degrees(sample.values[0]) for sample in numeric[ESTIMATED_ATTITUDE_TOPIC]]
        ),
        "pitch": scalar_summary(
            [math.degrees(sample.values[1]) for sample in numeric[ESTIMATED_ATTITUDE_TOPIC]]
        ),
        "tilt": scalar_summary(
            [math.degrees(sample.values[2]) for sample in numeric[ESTIMATED_ATTITUDE_TOPIC]]
        ),
    }
    workspace_dir = Path(__file__).resolve().parents[1]
    git_commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=workspace_dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    ).stdout.strip()
    git_dirty = bool(
        subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=workspace_dir,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
        ).stdout.strip()
    )

    return {
        "bag": loaded["bag"],
        "scenario": args.scenario,
        "seed": args.seed,
        "git_commit": git_commit,
        "git_dirty": git_dirty,
        "state_sequence": states,
        "state_sample_counts": state_counts,
        "state_durations_s": dict(state_durations_s),
        "entered_descend": "DESCEND" in states,
        "entered_test_height_hold": "TEST_HEIGHT_HOLD" in states,
        "test_height_hold_duration_s": state_durations_s.get("TEST_HEIGHT_HOLD", 0.0),
        "forbidden_safe_altitude_states": forbidden_states,
        "forbidden_fixed_tilt_safe_descent_states": fixed_tilt_safe_descent_forbidden_states,
        "required_topics_present": not topics_without_messages,
        "topics_without_messages": topics_without_messages,
        "message_counts": {
            topic: len(samples) for topic, samples in sorted(numeric.items())
        },
        "status_counts": status_counts,
        "ok_shadow_status_count": ok_status_count,
        "invalid_status_count": invalid_status_count,
        "invalid_numeric_counts": dict(invalid_counts),
        "nan_inf_count": total_invalid_numeric,
        "trajectory_nonfinite_position_count": len(
            loaded["trajectory_nonfinite_position_times"]
        ),
        "trajectory_nonfinite_after_tracking_count": len(
            trajectory_nonfinite_after_tracking
        ),
        "trajectory_startup_nan_semantics": (
            "all-NaN PX4 trajectory placeholders before TRACK_TARGET are reported "
            "but are not geometry/control-output failures"
        ),
        "target_nonfinite_position_count": len(
            loaded["target_nonfinite_position_times"]
        ),
        "target_nonfinite_after_tracking_count": len(
            target_nonfinite_after_tracking
        ),
        "target_startup_nan_semantics": (
            "non-finite target poses before TRACK_TARGET indicate unavailable startup "
            "state; any occurrence at or after TRACK_TARGET is a hard failure"
        ),
        "aligned_sample_count": aligned_count,
        "time_sync_failure_count": time_sync_failures,
        "fixed_tilt_safe_descent_horizontal_sync_failure_count": fixed_tilt_safe_descent_time_sync_failures,
        "normal_truth_sync_failure_count": normal_truth_sync_failures,
        "marker_mask_sync_failure_count": marker_mask_sync_failures,
        "ground_truth_geometry_sync_failure_count": gt_geometry_sync_failures,
        "geometry_consistency_failure_count": consistency_failures,
        "first_contact_index_mismatch_count": index_mismatches,
        "estimated_mean_normal_ned": (
            list(estimated_mean_normal) if estimated_mean_normal is not None else None
        ),
        "ground_truth_mean_normal_ned": (
            list(truth_mean_normal) if truth_mean_normal is not None else None
        ),
        "normal_angle_error_deg": scalar_summary(normal_errors_deg),
        "estimated_attitude_from_normal_deg": {
            "roll": scalar_summary(estimated_roll_deg),
            "pitch": scalar_summary(estimated_pitch_deg),
            "tilt": scalar_summary(estimated_tilt_deg),
        },
        "estimated_attitude_topic_deg": estimated_attitude_topic_deg,
        "ground_truth_attitude_deg": {
            "roll": scalar_summary(truth_roll_deg),
            "pitch": scalar_summary(truth_pitch_deg),
            "tilt": scalar_summary(truth_tilt_deg),
        },
        "expected_attitude_deg": (
            {"roll": scenario_attitude[0], "pitch": scenario_attitude[1]}
            if scenario_attitude is not None
            else None
        ),
        "calibration_axis_from_ground_truth": calibration_axis,
        "calibration_gate": calibration,
        "zero_vs_two_degree_observation": {
            "status": "observation_only_no_cross_bag_threshold",
            "mean_tilt_deg": scalar_summary(estimated_tilt_deg)["mean"],
            "expected_tilt_deg": (
                math.hypot(*scenario_attitude) if scenario_attitude is not None else None
            ),
        },
        "active_marker_sample_counts": {
            str(marker_id): len(active_normals_by_id.get(marker_id, []))
            for marker_id in range(4)
        },
        "marker_normal_statistics": marker_normal_statistics(
            all_normals_by_id,
            truth_mean_normal,
        ),
        "active_marker_normal_statistics": marker_normal_statistics(
            active_normals_by_id,
            truth_mean_normal,
        ),
        "marker_switch_count": marker_switch_count,
        "normal_rate_degps": scalar_summary(
            [sample.values[0] for sample in numeric[NORMAL_RATE_TOPIC]]
        ),
        "marker_switch_normal_jump_deg": scalar_summary(marker_switch_jumps),
        "body_clearance_m": scalar_summary(
            [sample.values[0] for sample in numeric[BODY_CLEARANCE_TOPIC]]
        ),
        "skid_clearances_m": skid_summaries,
        "minimum_skid_clearance_m": scalar_summary(aligned_minimums),
        "maximum_skid_clearance_m": scalar_summary(aligned_maximums),
        "clearance_spread_m": scalar_summary(aligned_spreads),
        "first_contact_point_counts": {
            str(index): count for index, count in sorted(first_contact_counts.items())
        },
        "ground_truth_geometry": {
            "aligned_sample_count": gt_geometry_aligned_count,
            "body_clearance_m": scalar_summary(gt_body_clearances),
            "skid_clearances_m": [scalar_summary(values) for values in gt_skid_clearances],
            "clearance_spread_m": scalar_summary(gt_spreads),
            "first_contact_point_counts": {
                str(index): count
                for index, count in sorted(gt_first_contact_counts.items())
            },
            "body_clearance_error_m": scalar_summary(body_clearance_errors),
            "skid_clearance_error_m": scalar_summary(skid_clearance_errors),
            "clearance_spread_error_m": scalar_summary(spread_errors),
            "threshold_status": (
                "fixed_tilt_safe_descent_frozen_ground_truth_minimum_clearance_gate"
                if args.fixed_tilt_safe_descent
                else "observation_only_plan_has_no_frozen_GT_clearance_threshold"
            ),
        },
        "tilted_deck_touchdown_metrics": tilted_deck_touchdown_metrics,
        "terminal_stabilization_historical_replay_metrics": terminal_stabilization_replay_metrics,
        "terminal_stabilization_metrics": terminal_stabilization_metrics,
        "fixed_tilt_safe_descent_metrics": {
            "evaluation_start_s": evaluation_start_s,
            "relative_height_z_m": scalar_summary(fixed_tilt_safe_descent_relative_heights),
            "height_reference_m": scalar_summary(fixed_tilt_safe_descent_height_references),
            "horizontal_error_m": horizontal_summary,
            "tangential_position_error_norm_m": scalar_summary(
                fixed_tilt_safe_descent_tangential_position_norms
            ),
            "tangential_relative_velocity_norm_mps": scalar_summary(
                fixed_tilt_safe_descent_tangential_velocity_norms
            ),
            "ground_truth_minimum_skid_clearance_m": scalar_summary(
                fixed_tilt_safe_descent_gt_minimum_clearances
            ),
            "ground_truth_skid_clearances_m": [
                scalar_summary(values) for values in fixed_tilt_safe_descent_gt_skid_clearances
            ],
            "ground_truth_contact_count": ground_truth_contact_count,
            "ground_truth_penetration_count": ground_truth_penetration_count,
            "first_contact_index": (
                contact_point_index(
                    [min(values) for values in fixed_tilt_safe_descent_gt_skid_clearances]
                )
                if all(fixed_tilt_safe_descent_gt_skid_clearances)
                else None
            ),
            "visual_freshness": {
                "status": "observation_only_no_frozen_threshold",
                "sample_count": len(visible_samples),
                "visible_count": visible_count,
                "visible_fraction": (
                    visible_count / len(visible_samples) if visible_samples else None
                ),
                "maximum_not_visible_gap_s": maximum_not_visible_gap_s,
            },
            "normal_rmse_deg": normal_summary["rmse"],
            "normal_p95_deg": normal_summary["p95"],
            "signed_axis_mean_error_deg": (
                calibration.get("mean_signed_error_deg")
                if calibration is not None
                else None
            ),
            "sign_accuracy": (
                calibration.get("sign_accuracy") if calibration is not None else None
            ),
            "marker_switch_count": marker_switch_count,
            "marker_switch_jump_max_deg": marker_jump_summary["max"],
            "recover_climb_count": state_counts.get("RECOVER_CLIMB", 0),
            "gnss_recovery_count": state_counts.get("RECOVER_TO_GNSS", 0),
            "final_descent_count": state_counts.get("FINAL_DESCENT", 0),
            "touchdown_candidate_count": state_counts.get(
                "TOUCHDOWN_CANDIDATE_HOLD", 0
            ),
            "touchdown_hold_count": state_counts.get("TOUCHDOWN_HOLD", 0),
            "nav_land_command_count": nav_land_commands,
            "disarm_command_count": disarm_commands,
            "nan_inf_count": total_invalid_numeric,
            "time_sync_failure_count": time_sync_failures,
            "gate": fixed_tilt_safe_descent_gate,
            "metric_semantics": {
                "pass_gate": [
                    "state path",
                    "hold duration",
                    "ground-truth non-contact clearance",
                    "horizontal error",
                    "complete-normal calibration",
                    "Marker switch jump",
                    "time sync",
                    "finite numeric output",
                    "NAV_LAND/Disarm",
                ],
                "observation_only": [
                    "height distributions",
                    "tangential metrics",
                    "relative horizontal velocity",
                    "visual freshness",
                    "per-skid distributions",
                ],
                "ground_truth_offline_only": [
                    "minimum skid clearance",
                    "per-skid clearance",
                    "contact and penetration counts",
                ],
            },
        },
        "normal_relative_velocity_mps": scalar_summary(
            [sample.values[0] for sample in numeric[NORMAL_VELOCITY_TOPIC]]
        ),
        "tangential_position_error_norm_m": scalar_summary(tangential_position_norms),
        "tangential_relative_velocity_norm_mps": scalar_summary(
            tangential_velocity_norms
        ),
        "ground_truth_linear_speed_mps": scalar_summary(gt_linear_speed),
        "ground_truth_angular_speed_radps": scalar_summary(gt_angular_speed),
        "target_z_after_tracking_m": scalar_summary(target_z_after_tracking),
        "target_z_span_after_tracking_m": target_z_span,
        "trajectory_z_after_tracking_m": scalar_summary(trajectory_z_after_tracking),
        "trajectory_z_span_after_tracking_m": trajectory_z_span,
        "nav_land_command_count": nav_land_commands,
        "disarm_command_count": disarm_commands,
        "ground_truth_used": bool(normal_errors_deg),
        "ground_truth_isolation": (
            "Ground Truth is read only by this offline evaluator; production controller and "
            "detector subscriptions are forbidden and covered by static tests."
        ),
        "shadow_control_isolation": {
            "production_output_semantics_changed": False,
            "evidence": (
                "shadow update executes after trajectory publication and is absent from the "
                "state machine; covered by static tests"
            ),
        },
        "evaluation_mode": (
            "terminal_stabilization_historical_replay_on_tilted_deck_touchdown"
            if args.terminal_stabilization_replay
            else (
            f"terminal_stabilization_{args.terminal_stabilization_mode}_on_" + (
                "tilted_deck_touchdown"
                if args.tilted_deck_touchdown
                else (
                    "fixed_tilt_safe_descent"
                    if args.fixed_tilt_safe_descent
                    else "fixed_tilt_safe_altitude_strict"
                )
            )
            if args.terminal_stabilization
            else (
            "tilted_deck_touchdown"
            if args.tilted_deck_touchdown
            else (
                "fixed_tilt_safe_descent"
                if args.fixed_tilt_safe_descent
                else ("fixed_tilt_safe_altitude_strict" if args.scenario is not None else "deck_geometry_shadow_regression")
            )
            )
            )
        ),
        "deck_geometry_shadow_data_valid": data_valid,
        "safe_altitude_safety_passed": safety_passed,
        "fixed_tilt_safe_altitude_validation_passed": fixed_tilt_safe_altitude_passed,
        "fixed_tilt_safe_descent_passed": fixed_tilt_safe_descent_passed,
        "tilted_deck_touchdown_passed": tilted_deck_touchdown_passed,
        "terminal_stabilization_passed": terminal_stabilization_passed,
        "positive_touchdown_passed": (
            tilted_deck_touchdown_passed and (terminal_stabilization_passed if args.terminal_stabilization else True)
            if args.tilted_deck_touchdown
            else False
        ),
        "final_result": (
            "PASS"
            if (
                (
                    tilted_deck_touchdown_passed
                    if args.tilted_deck_touchdown
                    else (
                        fixed_tilt_safe_descent_passed
                        if args.fixed_tilt_safe_descent
                        else (fixed_tilt_safe_altitude_passed if args.scenario is not None else data_valid)
                    )
                )
                and (terminal_stabilization_passed if args.terminal_stabilization else True)
            )
            else "FAIL"
        ),
    }


def compare_evaluation_results(results: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """Summarize cross-bag 0°/±2° separation and global fixed-tilt safe altitude safety evidence."""

    if not results:
        raise ValueError("cross-bag comparison requires at least one evaluation result")

    grouped: dict[str, list[dict[str, Any]]] = collections.defaultdict(list)
    for result in results:
        scenario = result.get("scenario")
        if scenario not in SCENARIO_ATTITUDES_DEG:
            raise ValueError(f"invalid or missing scenario in comparison result: {scenario}")
        grouped[str(scenario)].append(result)

    static_results = grouped.get("static", [])
    static_axis_means: dict[str, list[float]] = {"roll": [], "pitch": []}
    for result in static_results:
        attitude = result.get("estimated_attitude_from_normal_deg", {})
        for axis in ("roll", "pitch"):
            value = attitude.get(axis, {}).get("mean")
            if value is not None and math.isfinite(float(value)):
                static_axis_means[axis].append(float(value))

    scenario_summaries: dict[str, Any] = {}
    marker_sample_totals = {marker_id: 0 for marker_id in range(4)}
    total_marker_switches = 0
    total_nav_land_commands = 0
    total_disarm_commands = 0
    failed_run_count = 0

    for scenario, scenario_results in sorted(grouped.items()):
        axis = scenario_results[0].get("calibration_axis_from_ground_truth")
        axis_values: list[float] = []
        tilt_values: list[float] = []
        rmse_values: list[float] = []
        p95_values: list[float] = []
        sign_values: list[float] = []
        seeds: list[int] = []
        for result in scenario_results:
            if result.get("final_result") != "PASS":
                failed_run_count += 1
            seed = result.get("seed")
            if seed is not None:
                seeds.append(int(seed))
            attitude = result.get("estimated_attitude_from_normal_deg", {})
            axis_key = axis if axis in {"roll", "pitch"} else "tilt"
            axis_mean = attitude.get(axis_key, {}).get("mean")
            tilt_mean = attitude.get("tilt", {}).get("mean")
            if axis_mean is not None and math.isfinite(float(axis_mean)):
                axis_values.append(float(axis_mean))
            if tilt_mean is not None and math.isfinite(float(tilt_mean)):
                tilt_values.append(float(tilt_mean))
            gate = result.get("calibration_gate") or {}
            for source, destination in (
                (gate.get("rmse_deg"), rmse_values),
                (gate.get("p95_deg"), p95_values),
                (gate.get("sign_accuracy"), sign_values),
            ):
                if source is not None and math.isfinite(float(source)):
                    destination.append(float(source))
            counts = result.get("active_marker_sample_counts", {})
            for marker_id in range(4):
                marker_sample_totals[marker_id] += int(counts.get(str(marker_id), 0))
            total_marker_switches += int(result.get("marker_switch_count", 0))
            total_nav_land_commands += int(result.get("nav_land_command_count", 0))
            total_disarm_commands += int(result.get("disarm_command_count", 0))

        separation = None
        if scenario != "static" and axis in {"roll", "pitch"}:
            baseline_values = static_axis_means[axis]
            separations = [
                abs(value - baseline)
                for value in axis_values
                for baseline in baseline_values
            ]
            if separations:
                separation = min(separations)

        scenario_summaries[scenario] = {
            "run_count": len(scenario_results),
            "seeds": sorted(seeds),
            "calibration_axis": axis,
            "axis_mean_deg": scalar_summary(axis_values),
            "tilt_mean_deg": scalar_summary(tilt_values),
            "rmse_deg": scalar_summary(rmse_values),
            "p95_deg": scalar_summary(p95_values),
            "sign_accuracy": scalar_summary(sign_values),
            "minimum_axis_mean_separation_from_static_deg": separation,
            "all_runs_passed": all(
                result.get("final_result") == "PASS" for result in scenario_results
            ),
        }

    observed_marker_ids = [
        marker_id for marker_id, count in marker_sample_totals.items() if count > 0
    ]
    missing_marker_ids = [
        marker_id for marker_id, count in marker_sample_totals.items() if count == 0
    ]
    return {
        "run_count": len(results),
        "failed_run_count": failed_run_count,
        "scenarios": scenario_summaries,
        "marker_sample_totals": {
            str(marker_id): count for marker_id, count in marker_sample_totals.items()
        },
        "marker_ids_observed": observed_marker_ids,
        "marker_ids_missing": missing_marker_ids,
        "total_marker_switches": total_marker_switches,
        "total_nav_land_commands": total_nav_land_commands,
        "total_disarm_commands": total_disarm_commands,
        "zero_vs_two_degree_threshold_status": (
            "observation_only_plan_has_no_frozen_cross_bag_separation_threshold"
        ),
    }


def print_human_readable(result: dict[str, Any]) -> None:
    print(f"Bag: {result['bag']}")
    print(f"Scenario / seed: {result['scenario']} / {result['seed']}")
    print("State sequence: " + " -> ".join(result["state_sequence"]))
    print(
        "Shadow status OK / invalid: "
        f"{result['ok_shadow_status_count']} / {result['invalid_status_count']}"
    )
    print(
        "Aligned / sync-failed / consistency-failed / index-mismatch: "
        f"{result['aligned_sample_count']} / {result['time_sync_failure_count']} / "
        f"{result['geometry_consistency_failure_count']} / "
        f"{result['first_contact_index_mismatch_count']}"
    )
    print(f"Body clearance: {result['body_clearance_m']}")
    print(f"Skid clearances: {result['skid_clearances_m']}")
    print(f"Minimum clearance: {result['minimum_skid_clearance_m']}")
    print(f"Maximum clearance: {result['maximum_skid_clearance_m']}")
    print(f"Clearance spread: {result['clearance_spread_m']}")
    print(f"First contact counts: {result['first_contact_point_counts']}")
    print(f"Normal relative velocity: {result['normal_relative_velocity_mps']}")
    print(f"Tangential position norm: {result['tangential_position_error_norm_m']}")
    print(
        "Tangential relative velocity norm: "
        f"{result['tangential_relative_velocity_norm_mps']}"
    )
    print(f"Estimated mean normal: {result['estimated_mean_normal_ned']}")
    print(f"Ground Truth mean normal: {result['ground_truth_mean_normal_ned']}")
    print(f"Normal angle error: {result['normal_angle_error_deg']}")
    print(f"Estimated attitude: {result['estimated_attitude_from_normal_deg']}")
    print(f"Ground Truth attitude: {result['ground_truth_attitude_deg']}")
    print(f"Marker sample counts: {result['active_marker_sample_counts']}")
    print(f"Marker normal statistics: {result['marker_normal_statistics']}")
    print(
        "Marker switches / jump: "
        f"{result['marker_switch_count']} / {result['marker_switch_normal_jump_deg']}"
    )
    print(f"Ground Truth geometry: {result['ground_truth_geometry']}")
    print(
        "Target / trajectory z span after tracking: "
        f"{result['target_z_span_after_tracking_m']} / "
        f"{result['trajectory_z_span_after_tracking_m']}"
    )
    print(
        "NAV_LAND / Disarm: "
        f"{result['nav_land_command_count']} / {result['disarm_command_count']}"
    )
    print(f"Ground Truth isolation: {result['ground_truth_isolation']}")
    print(f"Calibration gate: {result['calibration_gate']}")
    if result.get("terminal_stabilization_historical_replay_metrics") is not None:
        print(
            "terminal contact stabilization historical counterfactual replay: "
            f"{result['terminal_stabilization_historical_replay_metrics']}"
        )
    if result.get("terminal_stabilization_metrics") is not None:
        print(
            "terminal contact stabilization metrics: "
            f"{result['terminal_stabilization_metrics']}"
        )
        print(
            "terminal contact stabilization: "
            + (
                "PASS"
                if result["terminal_stabilization_passed"]
                else "FAIL"
            )
        )
    if "tilted_deck_touchdown" in str(result.get("evaluation_mode")):
        print(f"fixed-tilt touchdown metrics: {result['tilted_deck_touchdown_metrics']}")
        print(
            "fixed-tilt touchdown: "
            + ("PASS" if result["tilted_deck_touchdown_passed"] else "FAIL")
        )
    elif "fixed_tilt_safe_descent" in str(result.get("evaluation_mode")):
        print(f"fixed-tilt safe descent metrics: {result['fixed_tilt_safe_descent_metrics']}")
        print(
            "fixed-tilt safe descent safe descent: "
            + ("PASS" if result["fixed_tilt_safe_descent_passed"] else "FAIL")
        )
    print(
        "deck-geometry shadow geometry data: "
        + ("PASS" if result["deck_geometry_shadow_data_valid"] else "FAIL")
    )
    if "cross_bag_comparison" in result:
        print(f"Cross-bag comparison: {result['cross_bag_comparison']}")
    print(f"Final result: {result['final_result']}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate deck-geometry shadow, fixed-tilt safe descent, touchdown, "
            "and terminal contact stabilization diagnostics from a rosbag."
        )
    )
    parser.add_argument("bag", type=Path, help="rosbag directory or .db3 file")
    parser.add_argument(
        "--scenario",
        choices=sorted(SCENARIO_ATTITUDES_DEG),
        help="recorded tilted-deck scenario; required for strict fixed-tilt safe altitude/fixed-tilt safe descent/fixed-tilt touchdown evaluation",
    )
    parser.add_argument(
        "--fixed-tilt-safe-descent",
        action="store_true",
        help=(
            "apply the frozen fixed-tilt safe descent DESCEND -> TEST_HEIGHT_HOLD, non-contact, "
            "tracking, normal-calibration, and command safety gates"
        ),
    )
    parser.add_argument(
        "--tilted-deck-touchdown",
        action="store_true",
        help=(
            "apply the frozen fixed-tilt touchdown FINAL_DESCENT -> TOUCHDOWN_CANDIDATE_HOLD -> "
            "TOUCHDOWN_HOLD contact, tracking, slip, attitude, and command gates"
        ),
    )
    parser.add_argument(
        "--terminal-stabilization-replay",
        action="store_true",
        help=(
            "counterfactually replay frozen terminal contact stabilization B1 command on a historical fixed-tilt touchdown bag; "
            "never claims control PASS"
        ),
    )
    parser.add_argument(
        "--terminal-stabilization",
        action="store_true",
        help="apply the frozen terminal stabilization command, tracking, fallback, and safety gates",
    )
    parser.add_argument(
        "--terminal-stabilization-mode",
        choices=("shadow", "rehearsal", "active"),
        default="shadow",
        help="terminal contact stabilization experiment mode represented by the bag",
    )
    parser.add_argument("--seed", type=int, help="record deterministic experiment seed")
    parser.add_argument("--json", action="store_true", help="print JSON output")
    parser.add_argument(
        "--output-json", type=Path, help="also write the complete JSON result"
    )
    parser.add_argument(
        "--output-text", type=Path, help="also write the human-readable evaluation"
    )
    parser.add_argument(
        "--comparison-json",
        type=Path,
        nargs="+",
        help="attach a cross-bag comparison built from evaluator JSON files",
    )
    parser.add_argument(
        "--marker-history-bag",
        type=Path,
        help=(
            "replay an already accepted real multi-scale Bag to quantify per-ID "
            "normal bias and actual Marker switch jumps"
        ),
    )
    parser.add_argument(
        "--marker-history-filter-gain",
        type=float,
        default=0.08,
        help="shadow-only normal filter gain used for historical replay",
    )
    parser.add_argument("--max-sync-gap-s", type=float, default=0.10)
    parser.add_argument("--consistency-tolerance-m", type=float, default=1.0e-6)
    parser.add_argument(
        "--world-origin-latitude", type=float, default=47.397971057728974
    )
    parser.add_argument(
        "--world-origin-longitude", type=float, default=8.546163739800146
    )
    parser.add_argument("--world-origin-altitude", type=float, default=0.0)
    parser.add_argument(
        "--require-ground-truth",
        action="store_true",
        help="require evaluator-only /simulation/deck/ground_truth",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        if args.max_sync_gap_s < 0.0 or not math.isfinite(args.max_sync_gap_s):
            raise ValueError("--max-sync-gap-s must be finite and non-negative")
        if args.consistency_tolerance_m < 0.0 or not math.isfinite(
            args.consistency_tolerance_m
        ):
            raise ValueError("--consistency-tolerance-m must be finite and non-negative")
        if args.seed is not None and args.seed < 0:
            raise ValueError("--seed must be non-negative")
        if not finite_values(
            (
                args.world_origin_latitude,
                args.world_origin_longitude,
                args.world_origin_altitude,
            )
        ):
            raise ValueError("world origin must contain finite values")
        if args.fixed_tilt_safe_descent and args.tilted_deck_touchdown:
            raise ValueError("--fixed-tilt-safe-descent and --tilted-deck-touchdown are mutually exclusive")
        if args.terminal_stabilization_replay and args.terminal_stabilization:
            raise ValueError("--terminal-stabilization-replay and --terminal-stabilization are mutually exclusive")
        if args.terminal_stabilization_replay:
            if not args.tilted_deck_touchdown:
                raise ValueError("--terminal-stabilization-replay requires --tilted-deck-touchdown")
            if args.scenario not in {
                "tilt_roll_pos_2deg",
                "tilt_pitch_pos_2deg",
            }:
                raise ValueError(
                    "--terminal-stabilization-replay is restricted to positive fixed +2 degree tilt scenarios"
                )
        if args.terminal_stabilization:
            if args.scenario not in {
                "tilt_roll_pos_2deg",
                "tilt_pitch_pos_2deg",
            }:
                raise ValueError(
                    "--terminal-stabilization is restricted to positive fixed +2 degree tilt scenarios"
                )
            if args.terminal_stabilization_mode == "active" and not args.tilted_deck_touchdown:
                raise ValueError("terminal contact stabilization active mode requires --tilted-deck-touchdown")
            if args.terminal_stabilization_mode == "rehearsal" and not args.fixed_tilt_safe_descent:
                raise ValueError("terminal contact stabilization rehearsal mode requires --fixed-tilt-safe-descent")
            if args.terminal_stabilization_mode == "shadow" and args.tilted_deck_touchdown:
                raise ValueError("terminal contact stabilization shadow mode cannot be combined with touchdown")
        if args.tilted_deck_touchdown:
            if args.scenario not in {
                "static",
                "constant02",
                "tilt_roll_pos_2deg",
                "tilt_pitch_pos_2deg",
            }:
                raise ValueError(
                    "--tilted-deck-touchdown requires static, constant02, "
                    "tilt_roll_pos_2deg, or tilt_pitch_pos_2deg"
                )
            args.require_ground_truth = True
        elif args.fixed_tilt_safe_descent:
            if args.scenario not in {
                "static",
                "constant02",
                "tilt_roll_pos_2deg",
                "tilt_pitch_pos_2deg",
            }:
                raise ValueError(
                    "--fixed-tilt-safe-descent requires static, constant02, "
                    "tilt_roll_pos_2deg, or tilt_pitch_pos_2deg"
                )
            args.require_ground_truth = True
        elif args.scenario is not None:
            args.require_ground_truth = True
        result = evaluate(args)
        if args.comparison_json:
            comparison_results = []
            for comparison_path in args.comparison_json:
                comparison_results.append(
                    json.loads(comparison_path.read_text(encoding="utf-8"))
                )
            result["cross_bag_comparison"] = compare_evaluation_results(
                comparison_results
            )
        if args.marker_history_bag is not None:
            marker_history = analyze_marker_history_bag(
                args.marker_history_bag,
                filter_gain=args.marker_history_filter_gain,
                max_sync_gap_s=args.max_sync_gap_s,
            )
            result["marker_history_replay"] = marker_history
            if not marker_history["passed"]:
                result["fixed_tilt_safe_altitude_validation_passed"] = False
                result["final_result"] = "FAIL"
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    if args.output_text:
        output_buffer = io.StringIO()
        with contextlib.redirect_stdout(output_buffer):
            print_human_readable(result)
        args.output_text.parent.mkdir(parents=True, exist_ok=True)
        args.output_text.write_text(output_buffer.getvalue(), encoding="utf-8")
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print_human_readable(result)
    passed = (
        result["tilted_deck_touchdown_passed"]
        if args.tilted_deck_touchdown
        else (
            result["fixed_tilt_safe_descent_passed"]
            if args.fixed_tilt_safe_descent
            else (
                result["fixed_tilt_safe_altitude_validation_passed"]
                if args.scenario is not None
                else result["deck_geometry_shadow_data_valid"]
            )
        )
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
