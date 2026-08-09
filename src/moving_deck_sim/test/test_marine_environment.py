#!/usr/bin/env python3

import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE_DIR = Path(__file__).resolve().parents[3]
START_SCRIPT = WORKSPACE_DIR / "scripts" / "start_sitl.sh"


class MarineEnvironmentTest(unittest.TestCase):
    def run_dry(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(START_SCRIPT), *args, "--dry-run"],
            cwd=WORKSPACE_DIR,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_legacy_is_default_environment(self) -> None:
        completed = self.run_dry("--scenario", "static")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("environment=legacy", completed.stdout)

    def test_legacy_defaults_keep_historical_world_model_and_spawn(self) -> None:
        launch = (
            WORKSPACE_DIR / "src" / "moving_deck_sim" / "launch" /
            "moving_deck_sim.launch.py"
        ).read_text(encoding="utf-8")
        script = START_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('default_value="legacy"', launch)
        self.assertIn('"worlds", "aruco_moving_deck.sdf"', launch)
        self.assertIn('"model_name": "moving_deck"', launch)
        self.assertIn('px4_spawn_pose="-4,0,0.2"', script)
        self.assertIn('environment="legacy"', script)

    def test_environment_argument_rejects_unknown_value(self) -> None:
        completed = self.run_dry("--environment", "unknown")
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("invalid environment", completed.stderr)

    def test_marine_static_is_safe_altitude_only(self) -> None:
        completed = self.run_dry("--environment", "marine", "--scenario", "static")
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("environment=marine", completed.stdout)
        self.assertIn("relative_descent_enabled=false", completed.stdout)
        self.assertIn("final_descent_enabled=false", completed.stdout)

    def test_marine_rejects_all_descent_and_contact_modes(self) -> None:
        unsafe_options = (
            ("--enable-relative-descent",),
            ("--enable-relative-descent", "--enable-final-descent"),
            ("--terminal-contact-stabilization-shadow",),
            ("--terminal-contact-stabilization-rehearsal",),
            ("--enable-terminal-contact-stabilization",),
        )
        for options in unsafe_options:
            with self.subTest(options=options):
                completed = self.run_dry(
                    "--environment", "marine", "--scenario", "static", *options
                )
                self.assertNotEqual(completed.returncode, 0)
                self.assertIn(
                    "marine environment currently supports safe-altitude validation only",
                    completed.stderr,
                )

    def test_marine_world_and_vessel_keep_expected_geometry(self) -> None:
        world = (
            WORKSPACE_DIR / "src" / "moving_deck_sim" / "worlds" /
            "aruco_marine_vessel.sdf"
        ).read_text(encoding="utf-8")
        vessel = (
            WORKSPACE_DIR / "src" / "moving_deck_sim" / "models" /
            "landing_vessel" / "model.sdf"
        ).read_text(encoding="utf-8")
        self.assertIn('<world name="aruco">', world)
        self.assertIn("model://landing_vessel", world)
        self.assertIn('name="uav_launch_platform"', world)
        self.assertIn('name="ocean_visual"', world)
        self.assertIn('<link name="vessel_body">', vessel)
        self.assertIn('name="landing_deck"', vessel)
        self.assertIn('<pose>0 0 2 0 0 0</pose>', vessel)
        self.assertIn("/simulation/deck/ground_truth_raw", vessel)

    def test_marine_marker_geometry_matches_legacy_deck(self) -> None:
        model_dir = WORKSPACE_DIR / "src" / "moving_deck_sim" / "models"
        legacy_root = ET.parse(model_dir / "moving_deck" / "model.sdf").getroot()
        marine_root = ET.parse(model_dir / "landing_vessel" / "model.sdf").getroot()
        marker_names = (
            "aruco_marker_far_visual",
            "aruco_marker_far_secondary_visual",
            "aruco_marker_far_secondary_roll_positive_visual",
            "aruco_marker_far_secondary_roll_negative_visual",
            "aruco_marker_near_visual",
            "aruco_marker_ultra_near_visual",
            "aruco_marker_contact_visual",
        )

        legacy_link = legacy_root.find("./model/link[@name='moving_deck']")
        marine_link = marine_root.find("./model/link[@name='vessel_body']")
        self.assertIsNotNone(legacy_link)
        self.assertIsNotNone(marine_link)
        for name in marker_names:
            with self.subTest(marker=name):
                legacy = legacy_link.find(f"./visual[@name='{name}']")
                marine = marine_link.find(f"./visual[@name='{name}']")
                self.assertIsNotNone(legacy)
                self.assertIsNotNone(marine)
                self.assertEqual(legacy.findtext("pose").strip(), marine.findtext("pose").strip())
                self.assertEqual(
                    legacy.findtext("./geometry/plane/size").strip(),
                    marine.findtext("./geometry/plane/size").strip(),
                )
                self.assertEqual(
                    legacy.findtext("./material/pbr/metal/albedo_map").strip(),
                    marine.findtext("./material/pbr/metal/albedo_map").strip(),
                )
                self.assertEqual(marine.find("pose").attrib.get("relative_to"), "landing_deck")


if __name__ == "__main__":
    unittest.main()
