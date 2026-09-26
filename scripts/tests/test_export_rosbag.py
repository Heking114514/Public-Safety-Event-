#!/usr/bin/env python3

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
SCRIPT = SCRIPTS / "export_rosbag.py"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

try:
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from rclpy.serialization import serialize_message
    import rosbag2_py
    from sensor_msgs.msg import Image
    from std_msgs.msg import Int32, String

    HAVE_ROS = shutil.which("zstd") is not None
except ImportError:
    HAVE_ROS = False


IMAGE_TOPIC = "/camera/camera/color/image_raw"
ROUTE_TOPIC = "/waypoint_navigation/route_input"
WAYPOINT_TOPIC = "/waypoint_navigation/current_waypoint"
STATUS_TOPIC = "/waypoint_navigation/status"


def write_camera_bag(bag_dir, frames, width=64, height=48, routes=True):
    """Write a zstd-compressed bag holding `frames` rgb8 camera images.

    With `routes`, a short route plus waypoint/status traffic is recorded too,
    so scripts/analyze_navigation_bag.py has segments to report.
    """
    staging = bag_dir.parent / "staging"
    storage = rosbag2_py.StorageOptions(uri=str(staging), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage, converter)
    topics = {IMAGE_TOPIC: "sensor_msgs/msg/Image"}
    if routes:
        topics.update({
            ROUTE_TOPIC: "nav_msgs/msg/Path",
            WAYPOINT_TOPIC: "std_msgs/msg/Int32",
            STATUS_TOPIC: "std_msgs/msg/String",
        })
    for name, topic_type in topics.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=name, type=topic_type, serialization_format="cdr"
            )
        )

    for index in range(frames):
        image = Image()
        image.height = height
        image.width = width
        image.encoding = "rgb8"
        image.is_bigendian = 0
        image.step = width * 3
        image.data = np.full((height, width, 3), index * 7, dtype=np.uint8).tobytes()
        writer.write(
            IMAGE_TOPIC, serialize_message(image), 1_000_000_000 + index * 33_333_333
        )

    if routes:
        route = RosPath()
        for index in range(3):
            pose = PoseStamped()
            pose.pose.position.x = 0.5 * index
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            route.poses.append(pose)
        writer.write(ROUTE_TOPIC, serialize_message(route), 1_000_000_000)

        for index in range(3):
            stamp = 1_000_000_000 + (index + 1) * 200_000_000
            waypoint = Int32()
            waypoint.data = index
            writer.write(WAYPOINT_TOPIC, serialize_message(waypoint), stamp)
            status = String()
            status.data = "FOLLOWING" if index else "BRAKING_AT_WAYPOINT"
            writer.write(STATUS_TOPIC, serialize_message(status), stamp)
    del writer

    # rosbag2 writes a complete metadata.yaml beside the staged .db3 files.
    # Reuse it with the compressed file names so the fixture matches a real
    # zstd FILE-compressed recording.
    metadata = yaml.safe_load((staging / "metadata.yaml").read_text())
    info = metadata["rosbag2_bagfile_information"]
    db3_files = sorted(staging.glob("*.db3"))
    info["compression_format"] = "zstd"
    info["compression_mode"] = "FILE"
    info["relative_file_paths"] = [f"{path.name}.zstd" for path in db3_files]

    bag_dir.mkdir(parents=True, exist_ok=True)
    for source in db3_files:
        subprocess.run(
            ["zstd", "-q", "-f", str(source), "-o", str(bag_dir / f"{source.name}.zstd")],
            check=True,
        )
    shutil.rmtree(staging)
    (bag_dir / "metadata.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8"
    )


@unittest.skipUnless(HAVE_ROS, "rosbag2_py, sensor_msgs and zstd are required")
class ExportRosbagTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.bag = self.root / "run" / "bag"
        self.out = self.root / "out"
        write_camera_bag(self.bag, frames=12)

    def tearDown(self):
        self.temporary.cleanup()

    def export(self, *argv):
        return subprocess.run(
            [sys.executable, str(SCRIPT), str(self.bag), "--out", str(self.out), *argv],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=120,
        )

    def sources(self):
        return {
            path.name: path.read_bytes() for path in self.bag.glob("*.db3.zstd")
        }

    def test_export_keeps_compressed_sources_and_writes_manifest(self):
        before = self.sources()
        self.assertNotEqual({}, before)

        result = self.export()
        self.assertEqual(0, result.returncode, result.stdout)

        # The recording is the only copy of the run: export must never remove it.
        self.assertEqual(before, self.sources())

        decompressed = sorted((self.out / "decompressed").glob("*.db3"))
        self.assertEqual(1, len(decompressed))
        self.assertGreater(decompressed[0].stat().st_size, 0)

        manifest = json.loads((self.out / "export_manifest.json").read_text())
        self.assertEqual(str(self.bag), manifest["bag_dir"])
        self.assertEqual(12, manifest["topics"][IMAGE_TOPIC]["messages"])
        self.assertEqual(1, len(manifest["decompressed_files"]))
        self.assertEqual(12, manifest["video"]["frames_written"])
        self.assertEqual(0, manifest["video"]["frames_skipped"])
        self.assertTrue(Path(manifest["video"]["path"]).is_file())

    def test_decompressed_cache_opens_directly_as_a_bag(self):
        result = self.export("--no-video")
        self.assertEqual(0, result.returncode, result.stdout)

        cache = self.out / "decompressed"
        self.assertTrue((cache / "metadata.yaml").is_file())

        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(cache), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        seen = {item.name: 0 for item in reader.get_all_topics_and_types()}
        messages = 0
        while reader.has_next():
            topic, _data, _stamp = reader.read_next()
            seen[topic] += 1
            messages += 1

        self.assertEqual(12, seen[IMAGE_TOPIC])
        # 12 images plus the route, three waypoints and three status messages.
        self.assertEqual(19, messages)

    def test_no_video_skips_rendering_but_still_writes_manifest(self):
        result = self.export("--no-video")
        self.assertEqual(0, result.returncode, result.stdout)

        manifest = json.loads((self.out / "export_manifest.json").read_text())
        self.assertIsNone(manifest["video"])
        self.assertFalse(list(self.out.glob("*.mp4")))

    def test_analysis_runs_against_the_decompressed_cache(self):
        result = self.export("--no-video")
        self.assertEqual(0, result.returncode, result.stdout)

        manifest = json.loads((self.out / "export_manifest.json").read_text())
        self.assertEqual("ok", manifest["analysis"]["status"])
        analysis_dir = self.out / "analysis"
        self.assertTrue((analysis_dir / "summary.json").is_file())
        self.assertTrue((analysis_dir / "segments.csv").is_file())

    def test_no_analysis_skips_the_route_analysis(self):
        result = self.export("--no-analysis")
        self.assertEqual(0, result.returncode, result.stdout)

        manifest = json.loads((self.out / "export_manifest.json").read_text())
        self.assertIsNone(manifest["analysis"])
        self.assertFalse((self.out / "analysis").exists())

    def test_route_less_bag_marks_the_analysis_skipped(self):
        shutil.rmtree(self.bag)
        write_camera_bag(self.bag, frames=4, routes=False)

        result = self.export("--no-video")
        self.assertEqual(0, result.returncode, result.stdout)

        manifest = json.loads((self.out / "export_manifest.json").read_text())
        self.assertEqual("skipped", manifest["analysis"]["status"])

    def test_stride_and_max_frames_bound_the_written_video(self):
        result = self.export("--stride", "2", "--max-frames", "3")
        self.assertEqual(0, result.returncode, result.stdout)

        manifest = json.loads((self.out / "export_manifest.json").read_text())
        video = manifest["video"]
        self.assertEqual(3, video["frames_written"])
        # images_seen counts images examined before the early stop, not the bag
        # total (which is reported under topics).
        self.assertEqual(5, video["images_seen"])
        self.assertEqual(12, manifest["topics"][IMAGE_TOPIC]["messages"])

    def test_rendered_video_decodes_to_the_written_frame_count(self):
        import cv2

        result = self.export("--max-frames", "5")
        self.assertEqual(0, result.returncode, result.stdout)

        video_path = json.loads(
            (self.out / "export_manifest.json").read_text()
        )["video"]["path"]
        capture = cv2.VideoCapture(video_path)
        decoded = 0
        while capture.read()[0]:
            decoded += 1
        capture.release()
        self.assertEqual(5, decoded)

    def test_absent_camera_topic_skips_video_but_keeps_the_manifest(self):
        result = self.export("--image-topic", "/camera/absent/image_raw")
        self.assertEqual(0, result.returncode, result.stdout)

        manifest = json.loads((self.out / "export_manifest.json").read_text())
        self.assertIsNone(manifest["video"])
        self.assertIn("/camera/absent/image_raw", manifest["video_note"])
        self.assertFalse(list(self.out.glob("*.mp4")))


class VolumeSelectionTest(unittest.TestCase):
    """ensure_recording_volume must not mount a volume it does not need."""

    def setUp(self):
        import export_rosbag as export

        self.export = export
        self.temporary = tempfile.TemporaryDirectory()
        root = Path(self.temporary.name)
        self.mounts = (("E", root / "E"), ("Data", root / "Data"))
        self.data_root = root / "Data" / "rosbag_recording"

    def tearDown(self):
        self.temporary.cleanup()

    def select(self, mounted, mount_volume):
        with mock.patch.object(self.export, "VOLUMES", self.mounts), mock.patch.object(
            self.export, "is_mounted", side_effect=mounted
        ), mock.patch.object(
            self.export, "mount_volume", side_effect=mount_volume
        ):
            return self.export.ensure_recording_volume()

    def test_ready_volume_is_used_without_mounting_the_absent_one(self):
        self.data_root.mkdir(parents=True)
        data_mount = self.mounts[1][1]
        attempts = []

        selected = self.select(
            lambda mountpoint: Path(mountpoint) == data_mount,
            lambda label, mountpoint: attempts.append(label) or False,
        )

        self.assertEqual([], attempts)
        self.assertEqual(self.data_root, selected)

    def test_absent_volume_is_mounted_when_nothing_is_ready(self):
        attempts = []

        def mount_volume(label, mountpoint):
            attempts.append(label)
            if label != "Data":
                return False
            self.data_root.mkdir(parents=True)
            return True

        selected = self.select(lambda mountpoint: False, mount_volume)

        self.assertEqual(["E", "Data"], attempts)
        self.assertEqual(self.data_root, selected)

    def test_no_volume_yields_none(self):
        self.assertIsNone(self.select(lambda mountpoint: False, lambda *_: False))


if __name__ == "__main__":
    unittest.main()
