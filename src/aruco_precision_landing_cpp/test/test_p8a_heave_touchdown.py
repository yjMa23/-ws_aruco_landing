#!/usr/bin/env python3
"""不启动 PX4/Gazebo 的 P8A 升沉触地脚本与评测纯函数测试。"""

from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

import yaml

WORKSPACE_DIR = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = WORKSPACE_DIR / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from evaluate_p8a_heave_touchdown import (  # noqa: E402
    contact_transition_counts,
    percentile,
    transition_entry_count,
)


class P8AHeaveTouchdownTests(unittest.TestCase):
    def test_heave_profiles_match_roadmap(self) -> None:
        expected = {
            "heave_h1.yaml": (0.10, 10.0),
            "heave_h2.yaml": (0.20, 8.0),
            "heave_h3.yaml": (0.30, 8.0),
        }
        config_dir = WORKSPACE_DIR / "src" / "moving_deck_sim" / "config"
        for filename, (amplitude, period) in expected.items():
            data = yaml.safe_load((config_dir / filename).read_text(encoding="utf-8"))
            parameters = data["moving_deck_controller"]["ros__parameters"]
            self.assertEqual(parameters["scenario"], "S3_HEAVE")
            self.assertAlmostEqual(parameters["amplitude_z_m"], amplitude)
            self.assertAlmostEqual(parameters["period_z_s"], period)
            self.assertEqual(parameters["amplitude_rpy_deg"], [0.0, 0.0, 0.0])

    def test_start_script_allows_all_heave_profiles_but_blocks_rollpitch(self) -> None:
        script = WORKSPACE_DIR / "scripts" / "start_sitl.sh"
        common = [
            str(script),
            "--enable-relative-descent",
            "--enable-final-descent",
        ]
        for scenario in ("heave_h1", "heave_h2", "heave_h3"):
            allowed = subprocess.run(
                [*common, "--scenario", scenario, "--descent-test-height", "0.51"],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertNotEqual(allowed.returncode, 0)
            self.assertIn("P6B final descent requires", allowed.stderr)
            self.assertNotIn("currently supports", allowed.stderr)

        rollpitch = subprocess.run(
            [*common, "--scenario", "rollpitch"],
            text=True,
            capture_output=True,
            check=False,
        )
        self.assertNotEqual(rollpitch.returncode, 0)
        self.assertIn("P8A heave profiles only", rollpitch.stderr)

    def test_percentile_uses_linear_interpolation(self) -> None:
        self.assertAlmostEqual(percentile([1.0, 2.0, 3.0, 4.0], 0.95), 3.85)
        self.assertEqual(percentile([2.0], 0.95), 2.0)
        self.assertTrue(percentile([], 0.95) != percentile([], 0.95))

    def test_contact_transitions_use_hysteresis(self) -> None:
        clearances = [0.01, 0.02, 0.06, 0.07, 0.04, 0.02, 0.01]
        detachments, secondary_contacts = contact_transition_counts(
            clearances,
            contact_enter_m=0.03,
            detached_enter_m=0.05,
        )
        self.assertEqual(detachments, 1)
        self.assertEqual(secondary_contacts, 1)

    def test_contact_transitions_do_not_count_noise_inside_hysteresis(self) -> None:
        clearances = [0.02, 0.035, 0.04, 0.025, 0.03]
        self.assertEqual(
            contact_transition_counts(
                clearances,
                contact_enter_m=0.03,
                detached_enter_m=0.05,
            ),
            (0, 0),
        )

    def test_transition_entry_count(self) -> None:
        values = ["AIRBORNE", "CANDIDATE", "CANDIDATE", "AIRBORNE", "CANDIDATE"]
        self.assertEqual(transition_entry_count(values, "CANDIDATE"), 2)


if __name__ == "__main__":
    unittest.main()
