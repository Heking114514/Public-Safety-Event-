#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
READER = WORKSPACE / "scripts" / "navigation_config.py"
SHELL_HELPER = WORKSPACE / "scripts" / "navigation_config.sh"
STARTUP = WORKSPACE / "config" / "navigation_startup.yaml"
RECORDING = WORKSPACE / "config" / "navigation_recording.yaml"


def read_records(startup=STARTUP, recording=RECORDING):
    result = subprocess.run(
        [str(READER), "--startup", str(startup), "--recording", str(recording)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=5,
    )
    records = [line.split("\t", 1) for line in result.stdout.splitlines()]
    return result, records


class NavigationConfigTest(unittest.TestCase):
    def test_defaults_preserve_modes_and_cover_diagnostic_topics(self):
        result, records = read_records()
        self.assertEqual(0, result.returncode, result.stderr)
        settings = {key: value for key, value in records if key != "topic"}
        topics = [value for key, value in records if key == "topic"]

        self.assertNotIn("autostart", settings)
        self.assertNotIn("route_mode", settings)
        self.assertEqual("3", settings["retain_bags"])
        self.assertEqual("scripts/waypoints.csv", settings["default_autostart_route"])
        self.assertEqual(len(topics), len(set(topics)))
        required = {
            "/camera/camera/infra1/camera_info",
            "/camera/camera/infra2/camera_info",
            "/camera/camera/imu",
            "/odometry/visual_raw",
            "/wheel/odom",
            "/imu/control",
            "/odometry/fused",
            "/cmd_vel_nav",
            "/cup_car_serial/control_telemetry",
            "/arena_path_planner/map",
            "/diagnostics",
            "/parameter_events",
            "/rosout",
            "/tf",
            "/tf_static",
        }
        self.assertTrue(required.issubset(topics))
        bulk_sensor_topics = [
            topic for topic in topics
            if "/image" in topic.lower()
            or "pointcloud" in topic.lower()
            or topic.lower().endswith("/points")
        ]
        self.assertEqual([], bulk_sensor_topics)
        self.assertNotIn("/fusion/input/imu", topics)
        self.assertNotIn("/path", topics)

    def test_unknown_or_duplicate_configuration_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            startup_data = yaml.safe_load(STARTUP.read_text(encoding="utf-8"))
            startup_data["navigation_startup"]["autostart"] = True
            bad_startup = root / "startup.yaml"
            bad_startup.write_text(yaml.safe_dump(startup_data), encoding="utf-8")
            result, _ = read_records(startup=bad_startup)
            self.assertEqual(2, result.returncode)
            self.assertIn("unknown startup keys: autostart", result.stderr)

            recording_data = yaml.safe_load(RECORDING.read_text(encoding="utf-8"))
            recording_data["navigation_recording"]["topics"].append(
                recording_data["navigation_recording"]["topics"][0]
            )
            bad_recording = root / "recording.yaml"
            bad_recording.write_text(yaml.safe_dump(recording_data), encoding="utf-8")
            result, _ = read_records(recording=bad_recording)
            self.assertEqual(2, result.returncode)
            self.assertIn("recording topics must be unique", result.stderr)

    def test_shell_loader_does_not_execute_configuration_text(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            marker = root / "executed"
            startup_data = yaml.safe_load(STARTUP.read_text(encoding="utf-8"))
            startup_data["navigation_startup"]["serial_device"] = (
                f"$(touch {marker})"
            )
            startup = root / "startup.yaml"
            startup.write_text(yaml.safe_dump(startup_data), encoding="utf-8")
            script = f"""
set -euo pipefail
WORKSPACE_ROOT={str(WORKSPACE)!r}
CONFIG_HELPER={str(READER)!r}
STARTUP_CONFIG={str(startup)!r}
RECORDING_CONFIG={str(RECORDING)!r}
fail() {{ printf '%s\n' "$*" >&2; exit 1; }}
source {str(SHELL_HELPER)!r}
load_navigation_config
printf '%s\n' "$SERIAL_DEVICE"
"""
            result = subprocess.run(
                ["bash", "-c", script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5,
                env=os.environ.copy(),
            )
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertEqual(f"$(touch {marker})\n", result.stdout)
            self.assertFalse(marker.exists())


if __name__ == "__main__":
    unittest.main()
