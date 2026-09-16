#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from virtual_odometry import (  # noqa: E402
    PlanarPose,
    arena_pose_to_map_pose,
    load_start_pose,
    normalize_angle,
    yaw_quaternion,
)


class VirtualOdometryTest(unittest.TestCase):
    def test_start_pose_maps_to_zero_pose(self):
        start = PlanarPose(1.6, 4.3, -0.5 * math.pi)

        pose = arena_pose_to_map_pose(start, start)

        self.assertAlmostEqual(0.0, pose.x)
        self.assertAlmostEqual(0.0, pose.y)
        self.assertAlmostEqual(0.0, pose.yaw)

    def test_arena_south_is_map_forward(self):
        start = PlanarPose(1.6, 4.3, -0.5 * math.pi)
        arena_pose = PlanarPose(1.6, 4.1, -0.5 * math.pi)

        pose = arena_pose_to_map_pose(arena_pose, start)

        self.assertAlmostEqual(0.2, pose.x)
        self.assertAlmostEqual(0.0, pose.y)
        self.assertAlmostEqual(0.0, pose.yaw)

    def test_arena_east_is_map_left_after_launch_heading_transform(self):
        start = PlanarPose(1.6, 4.3, -0.5 * math.pi)
        arena_pose = PlanarPose(2.1, 4.3, 0.0)

        pose = arena_pose_to_map_pose(arena_pose, start)

        self.assertAlmostEqual(0.0, pose.x)
        self.assertAlmostEqual(0.5, pose.y)
        self.assertAlmostEqual(0.5 * math.pi, pose.yaw)

    def test_loads_start_pose_from_arena_config(self):
        with tempfile.TemporaryDirectory() as root:
            config_path = Path(root) / "arena.yaml"
            config_path.write_text(
                "start:\n"
                "  position_m: [1.25, 3.5]\n"
                "  heading_deg: -90.0\n",
                encoding="utf-8",
            )

            pose = load_start_pose(config_path)

        self.assertAlmostEqual(1.25, pose.x)
        self.assertAlmostEqual(3.5, pose.y)
        self.assertAlmostEqual(-0.5 * math.pi, pose.yaw)

    def test_yaw_quaternion_is_normalized(self):
        qx, qy, qz, qw = yaw_quaternion(normalize_angle(3.0 * math.pi))

        self.assertEqual(0.0, qx)
        self.assertEqual(0.0, qy)
        self.assertAlmostEqual(1.0, qx * qx + qy * qy + qz * qz + qw * qw)


if __name__ == "__main__":
    unittest.main()
