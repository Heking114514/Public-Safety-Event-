"""ROS-free regression tests for the odometry test supervisor."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import MagicMock, Mock, patch

SPEC = importlib.util.spec_from_file_location(
    "odometry_test_under_test", Path(__file__).resolve().parents[1] / "odometry_test.py"
)
odometry = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = odometry
SPEC.loader.exec_module(odometry)


class FakeChild:
    def __init__(self):
        self.timeouts = []

    def stop(self, timeout):
        self.timeouts.append(timeout)


class OdometryTests(unittest.TestCase):
    def test_argument_defaults_and_negative_duration(self):
        args = odometry.parse_arguments([])
        self.assertEqual((args.speed, args.distance), (0.06, 3.0))
        self.assertAlmostEqual(args.duration, 60.0)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            odometry.parse_arguments(["--duration", "-1"])
        self.assertEqual(error.exception.code, 2)

    def test_health_errors(self):
        sample = odometry.Sample
        samples = {key: sample(True, 10, 20) for key in ("connected", "healthy", "fused", "local")}
        samples["fusion"] = sample("FULL", 10)
        self.assertEqual(odometry.health_errors(samples, 10, 20, 1), [])
        samples["fusion"] = sample("DEGRADED_DEAD_RECKONING", 10)
        self.assertNotIn("visual", samples)
        self.assertEqual(odometry.health_errors(samples, 10, 20, 1), [])
        failures = [("fusion", sample("FAULT", 10)), ("connected", sample(False, 10)),
                    ("healthy", sample(False, 10)), ("local", sample(True, 8, 20)),
                    ("fused", sample(True, 10, 18))]
        for key, value in failures:
            with self.subTest(key=key, value=value):
                self.assertTrue(odometry.health_errors({**samples, key: value}, 10, 20, 1))

    def test_production_camera_launch_without_navigator(self):
        commands = odometry.launch_commands(odometry.parse_arguments([]), Path("unused-bag"))
        self.assertEqual(set(commands), {"recorder", "serial", "localization"})
        localization = commands["localization"]
        self.assertEqual(localization[1:4], ["launch", "fused_odometry", "odometry_bringup.launch.py"])
        self.assertIn("infra_profile:=848x480x15", localization)
        self.assertIn("serial_no:=_038122250473", localization)
        self.assertFalse(any("navigator" in arg for command in commands.values() for arg in command))

    def test_finish_closes_all_children_and_lock_after_errors(self):
        for failure in ("event", "publish_velocity", "close"):
            with self.subTest(failure=failure):
                runner = odometry.TestRunner(odometry.parse_arguments([]))
                runner.state, runner.session = {}, MagicMock(spec=Path)
                (runner.session / "bag" / "metadata.yaml").is_file.return_value = True
                runner.monitor, runner.lock = Mock(), Mock()
                runner.event = Mock()
                target = runner.event if failure == "event" else getattr(runner.monitor, failure)
                target.side_effect = RuntimeError(failure + " failed")
                runner.children = {name: FakeChild() for name in ("localization", "serial", "recorder")}
                with patch.object(runner, "set_phase", side_effect=lambda phase: setattr(runner, "phase", phase)), \
                     patch.object(odometry.time, "monotonic", side_effect=[0.0, 0.1, 1.0]), \
                     patch.object(odometry, "log"):
                    self.assertEqual(runner.finish(0), 1)
                for name, child in runner.children.items():
                    self.assertEqual(child.timeouts, [30 if name == "recorder" else 8])
                runner.monitor.publish_velocity.assert_called_once_with(0.0)
                runner.monitor.close.assert_called_once_with()
                runner.lock.close.assert_called_once_with()
                self.assertEqual(runner.phase, "FAILED")
                self.assertTrue(any(failure + " failed" in error for error in runner.state["cleanup_errors"]))

    def test_child_sigint_saves_metadata_and_stops_process_group(self):
        script = """
import pathlib, signal, subprocess, sys, time
folder = pathlib.Path(sys.argv[1])
def stop(*args):
    (folder / 'metadata.yaml').write_text('version: 5\\n')
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
worker = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])
(folder / 'ready').write_text(str(worker.pid))
while True:
    time.sleep(1)
"""
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            child = odometry.Child("simulator", [sys.executable, "-c", script, directory], session)
            try:
                deadline = time.monotonic() + 5
                while not (session / "ready").is_file() and time.monotonic() < deadline:
                    self.assertIsNone(child.process.poll())
                    time.sleep(0.02)
                self.assertTrue((session / "ready").is_file(), "simulator did not become ready")
                worker_pid = int((session / "ready").read_text())
                self.assertIn(worker_pid, odometry.group_members(child.process.pid))
                child.stop(3)
                self.assertEqual(child.process.returncode, 0)
                self.assertEqual((session / "metadata.yaml").read_text(), "version: 5\n")
                self.assertEqual(odometry.group_members(child.process.pid), [])
                self.assertTrue(child.stream.closed)
            finally:
                if odometry.group_members(child.process.pid):
                    child.stop(1)
                if not child.stream.closed:
                    child.stream.close()


if __name__ == "__main__":
    unittest.main()
