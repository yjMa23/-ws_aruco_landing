#!/usr/bin/env python3
"""不启动仿真的 6-DoF deck-motion shadow 接口、评测与安全隔离测试。"""

from __future__ import annotations

import math
import json
import subprocess
import sys
import unittest
from pathlib import Path

import yaml

WORKSPACE_DIR = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE_DIR / "scripts"))

from evaluate_deck_motion_shadow import (  # noqa: E402
    Q_BODY_DECK,
    Q_NED_ENU,
    RigidState,
    derivative_truth,
    interpolate_truth,
    normal_and_yaw,
    normalize_quaternion,
    prediction_truth,
    quaternion_multiply,
    relative_truth,
    state_errors,
    summary,
)
from evaluate_horizontal_tracking import (  # noqa: E402
    GeodeticPosition,
    local_enu_to_wgs84,
    world_enu_to_local_ned,
)


class DeckMotionShadowTests(unittest.TestCase):
    def test_ground_truth_frame_contract_preserves_gazebo_deck_axes(self) -> None:
        deck_to_ned = normalize_quaternion(
            quaternion_multiply(Q_NED_ENU, Q_BODY_DECK)
        )
        normal, yaw = normal_and_yaw(deck_to_ned)
        self.assertAlmostEqual(normal[0], 0.0)
        self.assertAlmostEqual(normal[1], 0.0)
        self.assertAlmostEqual(normal[2], -1.0)
        self.assertAlmostEqual(yaw, math.pi / 2.0)

    def test_ground_truth_position_uses_px4_local_origin(self) -> None:
        world_origin = GeodeticPosition(47.397971057728974, 8.546163739800146, 0.0)
        px4_origin = local_enu_to_wgs84((-4.0, 0.0, 0.2), world_origin)
        position_ned = world_enu_to_local_ned((0.0, 0.0, 2.0), world_origin, px4_origin)
        self.assertAlmostEqual(position_ned[0], 0.0, places=5)
        self.assertAlmostEqual(position_ned[1], 4.0, places=5)
        self.assertAlmostEqual(position_ned[2], -1.8, places=5)

    def test_truth_interpolation_excludes_missing_future_target(self) -> None:
        samples = [
            RigidState(t, (t, 0.0, 2.0), (0.0, 1.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            for t in (1.0, 2.0)
        ]
        midpoint = interpolate_truth(samples, 1.5)
        self.assertIsNotNone(midpoint)
        self.assertAlmostEqual(midpoint.position[0], 1.5)
        self.assertIsNone(interpolate_truth(samples, 2.01))

    def test_relative_truth_subtracts_only_uav_translation(self) -> None:
        deck = RigidState(
            1.0, (4.0, -2.0, 3.0), (1.0, 0.0, 0.0, 0.0),
            (1.2, 0.4, -0.1), (0.1, 0.2, 0.3),
        )
        uav = RigidState(
            1.0, (1.0, -3.0, -2.0), (1.0, 0.0, 0.0, 0.0),
            (0.2, -0.1, 0.3), (0.0, 0.0, 0.0),
        )
        relative = relative_truth(deck, uav)
        self.assertEqual(relative.position, (3.0, 1.0, 5.0))
        self.assertEqual(relative.linear_velocity, deck.linear_velocity)
        self.assertEqual(relative.orientation, deck.orientation)
        self.assertEqual(relative.angular_velocity, deck.angular_velocity)

        future = RigidState(
            1.5, (5.0, -1.0, 2.5), deck.orientation,
            (1.4, 0.2, -0.2), deck.angular_velocity,
        )
        predicted = prediction_truth(future, uav)
        self.assertEqual(predicted.position, (4.0, 2.0, 4.5))
        self.assertEqual(predicted.linear_velocity, future.linear_velocity)

    def test_derivative_truth_keeps_linear_and_angular_channels_separate(self) -> None:
        samples = [
            RigidState(
                time_s,
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0),
                (2.0 * time_s, 0.0, 0.0),
                (0.0, 3.0 * time_s, 0.0),
            )
            for time_s in (0.98, 1.02)
        ]
        derivatives = derivative_truth(samples, 1.0)
        self.assertIsNotNone(derivatives)
        self.assertAlmostEqual(derivatives[0][0], 2.0)
        self.assertAlmostEqual(derivatives[1][1], 3.0)

    def test_empty_metric_summary_is_strict_json(self) -> None:
        encoded = json.dumps(summary([]), allow_nan=False)
        self.assertEqual(json.loads(encoded), {"count": 0, "rmse": None, "p95": None, "max": None})

    def test_yaw_error_wraps_across_pi(self) -> None:
        def yaw_orientation(degrees: float) -> tuple[float, ...]:
            angle = math.radians(degrees)
            yaw = (math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0))
            return normalize_quaternion(quaternion_multiply(yaw, (0.0, 1.0, 0.0, 0.0)))

        estimate = RigidState(0.0, (0.0, 0.0, 0.0), yaw_orientation(179.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        truth = RigidState(0.0, (0.0, 0.0, 0.0), yaw_orientation(-179.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(state_errors(estimate, truth)["yaw_deg"], 2.0)

    def test_yaml_freezes_shadow_horizons_and_standard_topics(self) -> None:
        config = yaml.safe_load(
            (WORKSPACE_DIR / "src" / "aruco_precision_landing_cpp" / "config" / "px4_aruco_landing.yaml").read_text(encoding="utf-8")
        )["px4_aruco_landing_node"]["ros__parameters"]
        self.assertTrue(config["deck_motion_shadow.enabled"])
        self.assertEqual(config["deck_motion_shadow.prediction_sample_period_s"], 0.05)
        self.assertEqual(config["deck_motion_shadow.trusted_prediction_horizon_s"], 0.50)
        self.assertEqual(config["deck_motion_shadow.maximum_prediction_horizon_s"], 1.00)
        self.assertEqual(config["deck_motion_shadow.kinematic_fit_window_s"], 0.30)
        source = (WORKSPACE_DIR / "src" / "aruco_precision_landing_cpp" / "src" / "px4_aruco_landing_node.cpp").read_text(encoding="utf-8")
        for suffix in ("state", "trajectory", "status", "trusted_horizon_s"):
            self.assertIn(f'/landing/deck_motion_shadow/{suffix}', source)

    def test_shadow_uses_uav_centered_ned_without_absolute_position(self) -> None:
        source = (WORKSPACE_DIR / "src" / "aruco_precision_landing_cpp" / "src" / "px4_aruco_landing_node.cpp").read_text(encoding="utf-8")
        callback_start = source.index("void Px4ArucoLandingNode::aruco_pose_callback")
        callback_end = source.index("void Px4ArucoLandingNode::aruco_visible_callback", callback_start)
        callback = source[callback_start:callback_end]
        self.assertIn("relative_deck_pose_ned", callback)
        self.assertIn("deck_motion_estimator_->update(", callback)
        self.assertIn("relative_deck_pose_ned, uav_velocity_ned_mps", callback)
        self.assertNotIn(
            "deck_motion_estimator_->update(marker_pose_ned",
            callback,
        )
        publish_start = source.index("void Px4ArucoLandingNode::publish_deck_motion_shadow")
        publish_end = source.index("void Px4ArucoLandingNode::publish_deck_plane_geometry", publish_start)
        publish_source = source[publish_start:publish_end]
        self.assertIn('state_msg.header.frame_id = "uav_centered_ned"', publish_source)
        self.assertIn('trajectory_msg.header.frame_id = "uav_origin_ned"', publish_source)

    def test_shadow_is_published_after_setpoint_and_absent_from_control_functions(self) -> None:
        source = (WORKSPACE_DIR / "src" / "aruco_precision_landing_cpp" / "src" / "px4_aruco_landing_node.cpp").read_text(encoding="utf-8")
        timer_start = source.index("void Px4ArucoLandingNode::control_timer_callback")
        timer_end = source.index("void Px4ArucoLandingNode::run_state_machine", timer_start)
        timer = source[timer_start:timer_end]
        self.assertLess(timer.index("publish_trajectory_setpoint();"), timer.index("publish_deck_motion_shadow(now);"))
        state_machine_end = source.index("void Px4ArucoLandingNode::transition_to", timer_end)
        self.assertNotIn("deck_motion", source[timer_end:state_machine_end])
        window_start = source.index("void Px4ArucoLandingNode::update_landing_window")
        window_end = source.index("std::optional<RelativeDescentOutput>", window_start)
        self.assertNotIn("deck_motion", source[window_start:window_end])

    def test_rigid_body_motion_is_safe_altitude_only(self) -> None:
        script = WORKSPACE_DIR / "scripts" / "start_sitl.sh"
        base = [
            str(script), "--scenario", "rigid_body_motion", "--dry-run",
            "--rendezvous-altitude", "7.0",
        ]
        accepted = subprocess.run(base, text=True, capture_output=True, check=False)
        self.assertEqual(accepted.returncode, 0, accepted.stderr)
        self.assertIn("rendezvous_altitude_m=7.0", accepted.stdout)
        diagnostic = subprocess.run(
            [str(script), "--scenario", "rigid_body_motion", "--dry-run",
             "--rendezvous-altitude", "5.0"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(diagnostic.returncode, 0, diagnostic.stderr)
        self.assertIn("rendezvous_altitude_m=5.0", diagnostic.stdout)
        for unsafe in (
            ["--enable-relative-descent"],
            ["--enable-relative-descent", "--enable-final-descent"],
            ["--enable-terminal-contact-stabilization"],
        ):
            rejected = subprocess.run([*base, *unsafe], text=True, capture_output=True, check=False)
            self.assertNotEqual(rejected.returncode, 0)

    def test_formal_runner_freezes_twelve_seeded_episodes(self) -> None:
        runner = WORKSPACE_DIR / "scripts" / "run_deck_motion_shadow_experiments.py"
        completed = subprocess.run(
            [str(runner), "--output", "/tmp/deck-motion-shadow-dry", "--dry-run"],
            cwd=WORKSPACE_DIR,
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        episodes = json.loads(completed.stdout)["episodes"]
        self.assertEqual(len(episodes), 12)
        self.assertTrue(
            all(
                ["--rendezvous-altitude", "7.0"]
                == episode["command"][
                    episode["command"].index("--rendezvous-altitude"):
                    episode["command"].index("--rendezvous-altitude") + 2
                ]
                for episode in episodes
            )
        )
        self.assertEqual(
            {(episode["scenario"], episode["seed"]) for episode in episodes},
            {
                (scenario, seed)
                for scenario in ("static", "rollpitch", "combined", "rigid_body_motion")
                for seed in (1, 2, 3)
            },
        )

    def test_ground_truth_topic_exists_only_in_simulation_or_offline_evaluators(self) -> None:
        forbidden = "/simulation/deck/" + "ground_truth"
        forbidden_uav = "/simulation/uav/" + "ground_truth_pose"
        production = list((WORKSPACE_DIR / "src" / "aruco_precision_landing_cpp" / "src").glob("*.cpp"))
        production += list((WORKSPACE_DIR / "src" / "aruco_detector" / "src").glob("*.cpp"))
        for path in production:
            source = path.read_text(encoding="utf-8")
            self.assertNotIn(forbidden, source)
            self.assertNotIn(forbidden_uav, source)

        simulator_source = (
            WORKSPACE_DIR / "src" / "moving_deck_sim" / "src" /
            "moving_deck_controller.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("/simulation/uav/ground_truth_pose", simulator_source)
        self.assertIn("pose.name() == uav_model_name_", simulator_source)


if __name__ == "__main__":
    unittest.main()
