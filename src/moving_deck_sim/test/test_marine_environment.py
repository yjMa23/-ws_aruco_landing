#!/usr/bin/env python3

import ast
import os
import re
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
ARUCO_CONFIG_DIR = WORKSPACE_DIR / "src" / "aruco_detector" / "config"
LEGACY_ARUCO_CONFIG = ARUCO_CONFIG_DIR / "aruco_detector.yaml"
MARINE_ARUCO_CONFIG = ARUCO_CONFIG_DIR / "aruco_detector_marine.yaml"
VRX_COMMIT = "7609d1bd90ce7edb29d040a082f949e8b089c864"


def list_parameter(path: Path, name: str) -> list[float | int]:
    text = path.read_text(encoding="utf-8")
    match = re.search(
        rf"^\s*{re.escape(name)}:\s*(\[[^\n]*\])\s*$", text, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"parameter not found: {name} in {path}")
    value = ast.literal_eval(match.group(1))
    if not isinstance(value, list):
        raise AssertionError(f"parameter is not a list: {name}")
    return value


def scalar_parameter(path: Path, name: str) -> str:
    text = path.read_text(encoding="utf-8")
    match = re.search(
        rf"^\s*{re.escape(name)}:\s*([^#\n]+?)\s*$", text, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"parameter not found: {name} in {path}")
    return match.group(1).strip()


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
        self.assertIn("aruco_detector_config=aruco_detector.yaml", completed.stdout)

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
        self.assertIn("aruco_detector_config=aruco_detector_marine.yaml", completed.stdout)
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
            "vrx_ocean_visual/materials/textures/skybox_lowres.dds",
            "vrx_ocean_visual/meshes/waterlow.dae",
            "vrx_ocean_visual/materials/programs/GerstnerWaves_vs_330.glsl",
            "vrx_ocean_visual/materials/programs/GerstnerWaves_fs_330.glsl",
            "vrx_wave_visual/WaveVisual.cc",
            "vrx_wave_visual/WaveVisual.hh",
            "vrx_wave_visual/Wavefield.cc",
            "vrx_wave_visual/Wavefield.hh",
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

    def test_marine_ocean_is_visual_only_dynamic_vrx_wavevisual(self) -> None:
        root = ET.parse(OCEAN_MODEL).getroot()
        model = root.find("./model[@name='vrx_ocean_visual']")
        self.assertIsNotNone(model)
        link = model.find("./link[@name='surface']")
        self.assertIsNotNone(link)
        self.assertIsNone(link.find("collision"))
        visual = link.find("./visual[@name='ocean_surface_visual']")
        self.assertIsNotNone(visual)
        self.assertEqual(
            visual.findtext("./geometry/mesh/uri"),
            "model://vrx_ocean_visual/meshes/waterlow.dae",
        )
        plugin = visual.find("./plugin[@name='vrx::WaveVisual']")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.attrib.get("filename"), "libWaveVisual.so")
        self.assertEqual(
            plugin.findtext("./shader/vertex"),
            "materials/programs/GerstnerWaves_vs_330.glsl",
        )
        self.assertEqual(
            plugin.findtext("./shader/fragment"),
            "materials/programs/GerstnerWaves_fs_330.glsl",
        )
        self.assertEqual(plugin.findtext("./wavefield/wave/model"), "CWR")
        self.assertEqual(plugin.findtext("./wavefield/wave/number"), "3")
        self.assertEqual(plugin.findtext("./wavefield/wave/amplitude"), "0.06")
        self.assertEqual(plugin.findtext("./wavefield/wave/period"), "4.0")
        self.assertEqual(plugin.findtext("./wavefield/wave/steepness"), "0.02")

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
        self.assertIn('GZ_SIM_SYSTEM_PLUGIN_PATH', launch)
        self.assertIn('os.path.join(package_prefix, "lib")', launch)
        self.assertIn('px4_spawn_pose="-12,0,0.6"', script)

    def test_legacy_marker_geometry_and_detector_calibration_remain_frozen(self) -> None:
        legacy_root = ET.parse(MODELS_DIR / "moving_deck" / "model.sdf").getroot()
        legacy_link = legacy_root.find("./model/link[@name='moving_deck']")
        self.assertIsNotNone(legacy_link)
        expected_visuals = {
            "aruco_marker_far_visual": (
                "0.45 0 0.001 0 0 0",
                "0.5 0.5",
                "model://arucotag/arucotag.png",
            ),
            "aruco_marker_far_secondary_visual": (
                "-0.75 0 0.2851650429449553 0 0.7853981633974483 0",
                "0.75 0.75",
                "model://moving_deck/aruco_id4.pgm",
            ),
            "aruco_marker_far_secondary_roll_positive_visual": (
                "0 0.75 0.2851650429449553 0.7853981633974483 0 0",
                "0.75 0.75",
                "model://moving_deck/aruco_id5.pgm",
            ),
            "aruco_marker_far_secondary_roll_negative_visual": (
                "0 -0.75 0.2851650429449553 -0.7853981633974483 0 0",
                "0.75 0.75",
                "model://moving_deck/aruco_id6.pgm",
            ),
            "aruco_marker_near_visual": (
                "-0.18 0 0.002 0 0 0",
                "0.20 0.20",
                "model://moving_deck/aruco_id1.pgm",
            ),
            "aruco_marker_ultra_near_visual": (
                "0.05 0 0.003 0 0 0",
                "0.04 0.04",
                "model://moving_deck/aruco_id2.pgm",
            ),
            "aruco_marker_contact_visual": (
                "0 0 0.004 0 0 0",
                "0.02 0.02",
                "model://moving_deck/aruco_id3.pgm",
            ),
        }
        for name, expected in expected_visuals.items():
            with self.subTest(marker=name):
                visual = legacy_link.find(f"./visual[@name='{name}']")
                self.assertIsNotNone(visual)
                self.assertEqual(visual.findtext("pose").strip(), expected[0])
                self.assertEqual(visual.findtext("./geometry/plane/size").strip(), expected[1])
                self.assertEqual(
                    visual.findtext("./material/pbr/metal/albedo_map").strip(), expected[2]
                )

        self.assertEqual(list_parameter(LEGACY_ARUCO_CONFIG, "marker_ids"), [0, 1, 2, 3])
        self.assertEqual(
            list_parameter(LEGACY_ARUCO_CONFIG, "marker_lengths_m"),
            [0.50, 0.20, 0.04, 0.02],
        )
        self.assertEqual(
            list_parameter(LEGACY_ARUCO_CONFIG, "far_board.marker_ids"), [0, 4, 5, 6]
        )
        self.assertEqual(
            list_parameter(LEGACY_ARUCO_CONFIG, "far_board.marker_lengths_m"),
            [0.50, 0.75, 0.75, 0.75],
        )
        self.assertEqual(scalar_parameter(LEGACY_ARUCO_CONFIG, "far_board.pose_model"), "noncoplanar")
        self.assertEqual(
            list_parameter(LEGACY_ARUCO_CONFIG, "far_board.marker_poses_deck_xyz_rpy"),
            [
                0.45, 0.0, 0.0, 0.0, 0.0, 0.0,
                -0.75, 0.0, 0.2851650429449553, 0.0, 0.7853981633974483, 0.0,
                0.0, 0.75, 0.2851650429449553, 0.7853981633974483, 0.0, 0.0,
                0.0, -0.75, 0.2851650429449553, -0.7853981633974483, 0.0, 0.0,
            ],
        )

    def test_marine_planar_board_matches_detector_calibration(self) -> None:
        marine_root = ET.parse(WAMV_MODEL).getroot()
        marine_link = marine_root.find("./model/link[@name='vessel_body']")
        self.assertIsNotNone(marine_link)

        ids = list_parameter(MARINE_ARUCO_CONFIG, "far_board.marker_ids")
        lengths = list_parameter(MARINE_ARUCO_CONFIG, "far_board.marker_lengths_m")
        poses = list_parameter(MARINE_ARUCO_CONFIG, "far_board.marker_poses_deck_xyz_rpy")
        self.assertEqual(ids, [4, 5, 6, 7])
        self.assertEqual(lengths, [0.50, 0.50, 0.50, 0.50])
        self.assertEqual(scalar_parameter(MARINE_ARUCO_CONFIG, "far_board.pose_model"), "planar")
        self.assertEqual(len(poses), 24)

        expected_centers = {
            4: (0.78, 0.78, 0.002),
            5: (0.78, -0.78, 0.002),
            6: (-0.78, 0.78, 0.002),
            7: (-0.78, -0.78, 0.002),
        }
        visual_names = {
            4: "aruco_marker_planar_id4_visual",
            5: "aruco_marker_planar_id5_visual",
            6: "aruco_marker_planar_id6_visual",
            7: "aruco_marker_planar_id7_visual",
        }
        for index, marker_id in enumerate(ids):
            with self.subTest(marker_id=marker_id):
                config_pose = tuple(float(value) for value in poses[index * 6:(index + 1) * 6])
                self.assertEqual(config_pose[:3], expected_centers[marker_id])
                self.assertEqual(config_pose[3:], (0.0, 0.0, 0.0))

                visual = marine_link.find(f"./visual[@name='{visual_names[marker_id]}']")
                self.assertIsNotNone(visual)
                pose_element = visual.find("pose")
                self.assertIsNotNone(pose_element)
                self.assertEqual(pose_element.attrib.get("relative_to"), "landing_deck")
                sdf_pose = tuple(float(value) for value in pose_element.text.split())
                self.assertEqual(sdf_pose, config_pose)
                self.assertEqual(
                    visual.findtext("./geometry/plane/normal").strip(), "0 0 1"
                )
                self.assertEqual(
                    tuple(float(value) for value in visual.findtext("./geometry/plane/size").split()),
                    (lengths[index], lengths[index]),
                )
                self.assertLessEqual(abs(sdf_pose[0]) + lengths[index] * 0.5, 1.20)
                self.assertLessEqual(abs(sdf_pose[1]) + lengths[index] * 0.5, 1.20)
                self.assertLessEqual(abs(sdf_pose[2]), 0.004)
                self.assertEqual(
                    visual.findtext("./material/pbr/metal/albedo_map").strip(),
                    f"model://moving_deck/aruco_id{marker_id}.pgm",
                )

        id7_texture = MODELS_DIR / "moving_deck" / "aruco_id7.pgm"
        self.assertTrue(id7_texture.is_file())
        self.assertGreater(id7_texture.stat().st_size, 0)

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
