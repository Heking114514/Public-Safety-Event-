#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import tempfile
import unittest


WORKSPACE = Path(__file__).resolve().parents[2]
SCRIPT = WORKSPACE / "scripts" / "record_camera_samples.sh"


class RecordCameraSamplesTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.commands = self.root / "commands"
        self.commands.mkdir()
        self.output = self.root / "camera_samples"
        self.events = self.root / "events.log"
        self.setup = self.root / "setup.bash"
        self.setup.write_text("true\n", encoding="utf-8")
        self.daemon = self.root / "mock_daemon.py"
        self.daemon.write_text(
            "#!/usr/bin/env python3\n"
            "import signal\n"
            "import sys\n"
            "import time\n"
            "from pathlib import Path\n"
            "output = Path(sys.argv[2]) if len(sys.argv) > 2 else None\n"
            "def stop(_signum, _frame):\n"
            "    if output is not None:\n"
            "        (output / 'metadata.yaml').write_text(\n"
            "            'rosbag2_bagfile_information: {}\\n'\n"
            "        )\n"
            "    raise SystemExit(0)\n"
            "signal.signal(signal.SIGINT, stop)\n"
            "signal.signal(signal.SIGTERM, stop)\n"
            "while True:\n"
            "    time.sleep(0.1)\n",
            encoding="utf-8",
        )
        self.daemon.chmod(0o755)
        self._write_mock("pgrep", "exit 1\n")
        self._write_mock(
            "rs-enumerate-devices",
            "printf 'Device Name: Intel RealSense D455\\n'\n"
            "printf '    Serial Number: 038122250473\\n'\n",
        )
        self._write_mock("ros2", self._ros2_mock())

    def tearDown(self):
        self.temporary.cleanup()

    def _write_mock(self, name, body):
        path = self.commands / name
        path.write_text(
            "#!/usr/bin/env bash\nset -u\n" + body,
            encoding="utf-8",
        )
        path.chmod(0o755)

    @staticmethod
    def _ros2_mock():
        return r'''
printf 'ros2 %s\n' "$*" >>"$MOCK_EVENTS"
if [[ "$1" == "topic" && "$2" == "list" ]]; then
  printf '%s\n' \
    /camera/camera/color/image_raw \
    /camera/camera/color/camera_info \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/aligned_depth_to_color/camera_info
  exit 0
fi
if [[ "$1" == "run" ]]; then
  exec "$MOCK_DAEMON" camera
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
  printf 'bag data\n' >"$output/camera.db3.zstd"
  exec "$MOCK_DAEMON" bag "$output"
fi
exit 2
'''

    def environment(self):
        environment = os.environ.copy()
        environment["PATH"] = f"{self.commands}:{environment['PATH']}"
        environment.update({
            "MOCK_EVENTS": str(self.events),
            "MOCK_DAEMON": str(self.daemon),
            "CAMERA_SAMPLE_ROS2_COMMAND": str(self.commands / "ros2"),
            "CAMERA_SAMPLE_PGREP_COMMAND": str(self.commands / "pgrep"),
            "CAMERA_SAMPLE_ROS_SETUP": str(self.setup),
            "CAMERA_SAMPLE_OUTPUT_ROOT": str(self.output),
            "CAMERA_SAMPLE_WARMUP_S": "0",
            "CAMERA_SAMPLE_TOPIC_TIMEOUT_S": "2",
        })
        return environment

    def test_records_one_labeled_sample_without_drive_topics(self):
        result = subprocess.run(
            [str(SCRIPT), "--duration", "1", "--samples", "1"],
            input="left_corner_40cm\n",
            env=self.environment(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
        )

        self.assertEqual(0, result.returncode, result.stderr)
        sessions = list(self.output.iterdir())
        self.assertEqual(1, len(sessions))
        session = sessions[0]
        bag = session / "samples" / "0001" / "bag"
        self.assertTrue((bag / "metadata.yaml").is_file())
        self.assertEqual(
            "left_corner_40cm\n",
            (session / "samples" / "0001" / "note.txt").read_text(),
        )
        index = (session / "samples.tsv").read_text()
        self.assertIn("0001\tleft_corner_40cm", index)

        events = self.events.read_text()
        self.assertIn("ros2 run realsense2_camera realsense2_camera_node", events)
        self.assertIn("/camera/camera/color/image_raw", events)
        self.assertIn("/camera/camera/aligned_depth_to_color/image_raw", events)
        self.assertNotIn("cmd_vel", events)
        self.assertNotIn("cup_car_serial", events)

    def test_no_depth_omits_depth_topics_from_bag(self):
        result = subprocess.run(
            [
                str(SCRIPT),
                "--duration",
                "1",
                "--samples",
                "1",
                "--no-depth",
                "--no-camera",
            ],
            input="rgb_only\n",
            env=self.environment(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10,
        )

        self.assertEqual(0, result.returncode, result.stderr)
        events = self.events.read_text()
        bag_event = next(
            line for line in events.splitlines()
            if line.startswith("ros2 bag record ")
        )
        self.assertIn("/camera/camera/color/image_raw", bag_event)
        self.assertNotIn("aligned_depth_to_color", bag_event)
        self.assertNotIn("ros2 run realsense2_camera", events)

    def test_rejects_zero_duration(self):
        result = subprocess.run(
            [str(SCRIPT), "--duration", "0"],
            env=self.environment(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5,
        )

        self.assertNotEqual(0, result.returncode)
        self.assertIn("--duration must be a positive integer", result.stderr)


if __name__ == "__main__":
    unittest.main()
