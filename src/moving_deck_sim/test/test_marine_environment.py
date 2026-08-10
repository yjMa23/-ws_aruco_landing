#!/usr/bin/env python3

import os
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


WORKSPACE_DIR = Path(__file__).resolve().parents[3]
START_SCRIPT = WORKSPACE_DIR / "scripts" / "start_sitl.sh"
MODELS_DIR = WORKSPACE_DIR / "src" / "moving_deck_sim" / "models"
MARINE_WORLD = (
    WORKSPACE_DIR / "src" / "moving_deck_sim" / "worlds" / "aruco_marine_vessel.sdf"
)
WAMV_MODEL = MODELS_DIR / "vrx_wamv_landing" / "model.sdf"
OCEAN_MODEL = MODELS_DIR / "vrx_ocean_visual" / "model.sdf"
VRX_COMMIT = "7609d1bd90ce7edb29d040a082f949e8b089c864"


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
        self.assertIn("terminal_contact_stabilization_enabled=false", completed.stdout)

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

    def test_vrx_upstream_metadata_and_import_contract_exist(self) -> None:
        cmake = (WORKSPACE_DIR / "src" / "moving_deck_sim" / "CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        third_party = (
            MODELS_DIR / "vrx_wamv_landing" / "THIRD_PARTY.md"
        ).read_text(encoding="utf-8")
        license_file = MODELS_DIR / "vrx_wamv_landing" / "LICENSE-VRX.txt"
        self.assertIn(VRX_COMMIT, cmake)
        self.assertIn(VRX_COMMIT, third_party)
        self.assertIn("Apache License 2.0", third_party)
        self.assertTrue(license_file.is_file())
        self.assertIn("Apache License", license_file.read_text(encoding="utf-8"))

    def test_vrx_imported_assets_exist_in_build_stage(self) -> None:
        default_stage = WORKSPACE_DIR / "build" / "moving_deck_sim" / "vrx_assets"
        stage = Path(os.environ.get("VRX_ASSET_STAGE_DIR", str(default_stage)))
        required = (
            "vrx_wamv_landing/meshes/WAM-V-Base.dae",
            "vrx_wamv_landing/meshes/WAM-V_Albedo.png",
            "vrx_wamv_landing/meshes/WAM-V_Normal.png",
            "vrx_wamv_landing/meshes/WAM-V_Roughness.png",
            "vrx_wamv_landing/meshes/WAM-V_Metalness.png",
            "vrx_ocean_visual/materials/textures/wave_normals.dds",
        )
        for relative_path in required:
            with self.subTest(asset=relative_path):
                asset = stage / relative_path
                self.assertTrue(asset.is_file(), f"missing imported VRX asset: {asset}")
                self.assertGreater(asset.stat().st_size, 0)

    def test_marine_world_uses_wamv_ocean_and_independent_launch_platform(self) -> None:
        world = MARINE_WORLD.read_text(encoding="utf-8")
        self.assertIn('<world name="aruco">', world)
        self.assertIn("model://vrx_wamv_landing", world)
        self.assertIn("model://vrx_ocean_visual", world)
        self.assertIn('name="uav_launch_platform"', world)
        self.assertIn("<pose>-12 0 0.30 0 0 0</pose>", world)
        self.assertIn("<world_frame_orientation>ENU</world_frame_orientation>", world)
        self.assertIn("<max_step_size>0.004</max_step_size>", world)
        self.assertIn("<sky/>", world)

    def test_marine_wamv_model_has_expected_reference_deck_and_pbr_visual(self) -> None:
        root = ET.parse(WAMV_MODEL).getroot()
        model = root.find("./model[@name='vrx_wamv_landing']")
        self.assertIsNotNone(model)
        vessel = model.find("./link[@name='vessel_body']")
        self.assertIsNotNone(vessel)
        self.assertEqual(
            vessel.findtext("./visual[@name='wamv_base_visual']/geometry/mesh/uri"),
            "model://vrx_wamv_landing/meshes/WAM-V-Base.dae",
        )
        self.assertEqual(
            vessel.findtext(
                "./visual[@name='wamv_base_visual']/material/pbr/metal/albedo_map"
            ),
            "model://vrx_wamv_landing/meshes/WAM-V_Albedo.png",
        )
        deck_frame = model.find("./frame[@name='landing_deck']")
        self.assertIsNotNone(deck_frame)
        self.assertEqual(deck_frame.attrib.get("attached_to"), "vessel_body")
        self.assertEqual(deck_frame.findtext("pose").strip(), "0 0 1.8 0 0 0")
        self.assertEqual(
            vessel.findtext("./collision[@name='landing_deck_collision']/geometry/box/size"),
            "2.4 2.4 0.1",
        )
        self.assertEqual(
            model.findtext("./plugin[@name='gz::sim::systems::OdometryPublisher']/odom_topic"),
            "/simulation/deck/ground_truth_raw",
        )

    def test_marine_ocean_is_visual_only_with_vrx_normal_texture(self) -> None:
        root = ET.parse(OCEAN_MODEL).getroot()
        model = root.find("./model[@name='vrx_ocean_visual']")
        self.assertIsNotNone(model)
        link = model.find("./link[@name='surface']")
        self.assertIsNotNone(link)
        self.assertIsNone(link.find("collision"))
        self.assertEqual(
            link.findtext("./visual/material/pbr/metal/normal_map"),
            "model://vrx_ocean_visual/materials/textures/wave_normals.dds",
        )
        self.assertIsNone(model.find("plugin"))

    def test_marine_runtime_sdf_has_no_fuel_or_user_cache_dependency(self) -> None:
        runtime_files = (MARINE_WORLD, WAMV_MODEL, OCEAN_MODEL)
        forbidden = (
            "fuel.gazebosim.org",
            "fuel://",
            "/home/",
            "~/.gz/fuel",
            "~/.ignition/fuel",
        )
        for path in runtime_files:
            text = path.read_text(encoding="utf-8")
            for token in forbidden:
                with self.subTest(path=path.name, token=token):
                    self.assertNotIn(token, text)

    def test_marine_launch_uses_wamv_transform_and_spawn(self) -> None:
        launch = (
            WORKSPACE_DIR / "src" / "moving_deck_sim" / "launch" /
            "moving_deck_sim.launch.py"
        ).read_text(encoding="utf-8")
        script = START_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('MARINE_NEUTRAL_DECK_HEIGHT_M = 2.0', launch)
        self.assertIn('MARINE_DECK_OFFSET_BODY = [0.0, 0.0, 1.8]', launch)
        self.assertIn('"model_name": "vrx_wamv_landing"', launch)
        self.assertIn('px4_spawn_pose="-12,0,0.6"', script)

    def test_marine_marker_geometry_matches_legacy_deck(self) -> None:
        legacy_root = ET.parse(MODELS_DIR / "moving_deck" / "model.sdf").getroot()
        marine_root = ET.parse(WAMV_MODEL).getroot()
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

    def test_production_landing_code_does_not_subscribe_ground_truth(self) -> None:
        production_roots = (
            WORKSPACE_DIR / "src" / "aruco_detector",
            WORKSPACE_DIR / "src" / "aruco_precision_landing_cpp",
        )
        token = "/simulation/deck/ground_truth"
        for root in production_roots:
            for suffix in ("*.cpp", "*.hpp", "*.py"):
                for path in root.rglob(suffix):
                    with self.subTest(path=path):
                        self.assertNotIn(token, path.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
