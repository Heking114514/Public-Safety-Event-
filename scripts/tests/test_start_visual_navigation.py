#!/usr/bin/env python3

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
START_SCRIPT = WORKSPACE / "scripts" / "start_visual_navigation.sh"


class StartVisualNavigationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.commands = self.root / "commands"
        self.commands.mkdir()
        self.events = self.root / "events.log"
        self.runs = self.root / "navigation_runs"
        self.latest_run = self.root / "latest_navigation_run"
        self.latest_bag = self.root / "latest_navigation_bag"
        self.build_manifest = self.root / "runtime-build.json"
        self._write_mock("pgrep", "printf 'pgrep %s\\n' \"$*\" >>\"$MOCK_EVENTS\"\nexit 1\n")
        self._write_mock("cmake", "printf 'cmake %s\\n' \"$*\" >>\"$MOCK_EVENTS\"\nexit \"${MOCK_CMAKE_EXIT:-0}\"\n")
        self._write_mock("colcon", "printf 'colcon %s\\n' \"$*\" >>\"$MOCK_EVENTS\"\nexit 0\n")
        self._write_mock("ros2", self._ros2_mock())

    def tearDown(self):
        self.temporary.cleanup()

    def _write_mock(self, name, body):
        path = self.commands / name
        path.write_text("#!/usr/bin/env bash\nset -u\n" + body, encoding="utf-8")
        path.chmod(0o755)

    def _ros2_mock(self):
        return r'''
printf 'ros2 %s\n' "$*" >>"$MOCK_EVENTS"
if [[ "$1" == "daemon" ]]; then
  exit 0
fi
if [[ "$1" == "node" && "$2" == "list" ]]; then
  printf '%s\n' \
    /fused_odometry_gate \
    /fused_ekf \
    /map_odom_correction \
    /waypoint_navigator \
    /orbslam3_stereo_inertial \
    /camera/camera
  exit 0
fi
if [[ "$1" == "param" && "$2" == "dump" ]]; then
  node="${!#}"
  printf '%s:\n  ros__parameters:\n    test_value: 1\n' "$node"
  exit 0
fi
if [[ "$1" == "service" || "$1" == "topic" ]]; then
  exit 0
fi
if [[ "$1" == "launch" ]]; then
  trap 'exit 0' INT TERM
  while true; do sleep 0.1; done
fi
if [[ "$1" == "bag" && "$2" == "record" ]]; then
  shift 2
  output=""
  while (($# > 0)); do
    if [[ "$1" == "--output" ]]; then
      output="$2"
      shift 2
    else
      shift
    fi
  done
  [[ -n "$output" && ! -e "$output" ]] || exit 91
  mkdir "$output"
  printf 'bag data' >"$output/mock.db3"
  finalize_bag() {
    printf 'rosbag2_bagfile_information: {}\n' >"$output/metadata.yaml"
  }
  trap 'finalize_bag; exit 0' INT TERM
  if [[ -n "${MOCK_BAG_EXIT:-}" ]]; then
    sleep 0.5
    finalize_bag
    exit "$MOCK_BAG_EXIT"
  fi
  while true; do sleep 0.1; done
fi
exit 2
'''

    def environment(self, **overrides):
        environment = os.environ.copy()
        environment.update({
            "MOCK_EVENTS": str(self.events),
            "NAVIGATION_ROS2_COMMAND": str(self.commands / "ros2"),
            "NAVIGATION_PGREP_COMMAND": str(self.commands / "pgrep"),
            "NAVIGATION_CMAKE_COMMAND": str(self.commands / "cmake"),
            "NAVIGATION_COLCON_COMMAND": str(self.commands / "colcon"),
            "NAVIGATION_BUILD_MANIFEST": str(self.build_manifest),
            "NAVIGATION_RUNS_ROOT": str(self.runs),
            "NAVIGATION_LATEST_RUN": str(self.latest_run),
            "NAVIGATION_LATEST_BAG": str(self.latest_bag),
        })
        environment.update(overrides)
        return environment

    @staticmethod
    def arguments(no_build=False):
        arguments = [
            str(START_SCRIPT),
            "--no-serial",
            "--no-imu",
            "--skip-camera-check",
            "--camera-serial",
            "038122250473",
        ]
        if no_build:
            arguments.append("--no-build")
        return arguments

    def wait_for(self, predicate, timeout=10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                result = predicate()
                if result:
                    return result
            except (FileNotFoundError, json.JSONDecodeError):
                pass
            time.sleep(0.05)
        self.fail("timed out waiting for mocked navigation state")

    def latest_manifest(self):
        return json.loads((self.latest_run.resolve() / "run_manifest.json").read_text())

    def test_build_failure_has_no_runtime_side_effects(self):
        self.build_manifest.write_bytes(b"previous successful manifest")
        result = subprocess.run(
            self.arguments(),
            env=self.environment(MOCK_CMAKE_EXIT="19"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=20,
        )
        self.assertEqual(19, result.returncode)
        self.assertEqual(b"previous successful manifest", self.build_manifest.read_bytes())
        events = self.events.read_text()
        self.assertIn("cmake ", events)
        self.assertNotIn("pgrep ", events)
        self.assertNotIn("ros2 ", events)
        self.assertFalse(self.runs.exists())

    def test_exit_status_history_parameters_and_term_are_preserved(self):
        self.latest_bag.mkdir()
        legacy_bytes = b"existing valid bag"
        (self.latest_bag / "legacy.db3").write_bytes(legacy_bytes)

        failed = subprocess.run(
            self.arguments(),
            env=self.environment(MOCK_BAG_EXIT="37"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=30,
        )
        self.assertEqual(37, failed.returncode, failed.stderr)
        first_failed_run = self.latest_run.resolve()
        failed_manifest = json.loads((first_failed_run / "run_manifest.json").read_text())
        self.assertEqual(37, failed_manifest["exit_code"])
        self.assertEqual("failed", failed_manifest["status"])
        self.assertEqual(
            self.arguments()[1:], failed_manifest["raw_argv"]
        )
        self.assertEqual(
            "038122250473", failed_manifest["effective_settings"]["camera_serial"]
        )
        self.assertEqual("success", failed_manifest["parameters"]["/fused_ekf"]["status"])
        fused_snapshot = Path(failed_manifest["parameters"]["/fused_ekf"]["path"])
        self.assertIn("/fused_ekf", yaml.safe_load(fused_snapshot.read_text()))
        self.assertEqual(legacy_bytes, (self.latest_bag.resolve() / "legacy.db3").read_bytes())

        first_events = self.events.read_text().splitlines()
        first_last_build = max(
            index for index, event in enumerate(first_events)
            if event.startswith("cmake ") or event.startswith("colcon ")
        )
        first_runtime = min(
            index for index, event in enumerate(first_events)
            if event.startswith("pgrep ") or event.startswith("ros2 ")
        )
        self.assertLess(first_last_build, first_runtime)

        second_event_start = len(first_events)
        second_failed = subprocess.run(
            self.arguments(),
            env=self.environment(MOCK_BAG_EXIT="37"),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=30,
        )
        self.assertEqual(37, second_failed.returncode, second_failed.stderr)
        failed_run = self.latest_run.resolve()
        self.assertNotEqual(first_failed_run, failed_run)
        self.assertTrue(first_failed_run.exists())
        self.assertEqual(legacy_bytes, (self.latest_bag.resolve() / "legacy.db3").read_bytes())

        events = self.events.read_text().splitlines()
        orb_builds = [
            event for event in events
            if event.startswith("cmake ") and "--target ORB_SLAM3" in event
        ]
        self.assertEqual(2, len(orb_builds))
        second_events = events[second_event_start:]
        second_last_build = max(
            index for index, event in enumerate(second_events)
            if event.startswith("cmake ") or event.startswith("colcon ")
        )
        second_runtime = min(
            index for index, event in enumerate(second_events)
            if event.startswith("pgrep ") or event.startswith("ros2 ")
        )
        self.assertLess(second_last_build, second_runtime)

        serial_arguments = [
            argument for argument in self.arguments(no_build=True)
            if argument != "--no-serial"
        ]
        event_count = len(events)
        serial_mismatch = subprocess.run(
            serial_arguments,
            env=self.environment(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=20,
        )
        self.assertNotEqual(0, serial_mismatch.returncode)
        self.assertIn("lacks required artifact 'wheel_odometry'", serial_mismatch.stderr)
        self.assertEqual(event_count, len(self.events.read_text().splitlines()))

        no_build_event_start = len(self.events.read_text().splitlines())

        process = subprocess.Popen(
            self.arguments(no_build=True),
            env=self.environment(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            interrupted_run = self.wait_for(
                lambda: self.latest_run.resolve()
                if self.latest_run.is_symlink() and self.latest_run.resolve() != failed_run
                else None
            )
            self.wait_for(
                lambda: len(self.latest_manifest().get("parameters", {})) >= 8
            )
            process.terminate()
            stdout, stderr = process.communicate(timeout=20)
        except Exception:
            process.kill()
            process.wait(timeout=5)
            raise
        self.assertEqual(143, process.returncode, stderr)
        interrupted_manifest = json.loads(
            (interrupted_run / "run_manifest.json").read_text()
        )
        self.assertEqual(143, interrupted_manifest["exit_code"])
        self.assertEqual("interrupted", interrupted_manifest["status"])
        self.assertTrue(interrupted_manifest["bag"]["metadata_present"])
        self.assertEqual(interrupted_run / "bag", self.latest_bag.resolve())
        self.assertTrue(failed_run.exists())

        int_process = subprocess.Popen(
            self.arguments(no_build=True),
            env=self.environment(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            int_run = self.wait_for(
                lambda: self.latest_run.resolve()
                if self.latest_run.is_symlink() and self.latest_run.resolve() != interrupted_run
                else None
            )
            self.wait_for(
                lambda: len(self.latest_manifest().get("parameters", {})) >= 8
            )
            int_process.send_signal(signal.SIGINT)
            _, int_stderr = int_process.communicate(timeout=20)
        except Exception:
            int_process.kill()
            int_process.wait(timeout=5)
            raise
        self.assertEqual(130, int_process.returncode, int_stderr)
        int_manifest = json.loads((int_run / "run_manifest.json").read_text())
        self.assertEqual(130, int_manifest["exit_code"])
        self.assertEqual("interrupted", int_manifest["status"])
        no_build_events = self.events.read_text().splitlines()[no_build_event_start:]
        self.assertFalse(any(event.startswith("cmake ") for event in no_build_events))
        self.assertFalse(any(event.startswith("colcon ") for event in no_build_events))


if __name__ == "__main__":
    unittest.main()
