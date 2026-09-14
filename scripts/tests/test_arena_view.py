#!/usr/bin/env python3

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from arena_view import (  # noqa: E402
    CAR,
    MAP_AXIS,
    PATH_POINT,
    ArenaView,
    map_axes_in_arena,
)


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


class Canvas:
    def __init__(self):
        self.calls = []

    def winfo_width(self):
        return 1000

    def winfo_height(self):
        return 700

    def delete(self, *args, **kwargs):
        self.calls.append(("delete", args, kwargs))

    def __getattr__(self, name):
        if name.startswith("create_"):
            def record(*args, **kwargs):
                self.calls.append((name, args, kwargs))
            return record
        raise AttributeError(name)


class ArenaViewTest(unittest.TestCase):
    def setUp(self):
        controller = SimpleNamespace(
            arena={"free_regions": [[0, 0, 2.4, 1.8]], "obstacles": []},
            config={
                "start": {"position_m": [1.2, 1.7], "heading_deg": -90.0},
                "tunnels": {},
            },
            tasks=[("1", [0.6, 0.6])],
            mode_var=Value("vehicle"),
            dynamic_obstacles=[(1.0, 0.4, 1.2, 0.6)],
            layer_routes={1: [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0)]},
            layer_display_segments={},
            active_route=[],
            active_route_layer=1,
            mission_running=False,
            visible_layer=1,
            odom_trace=[(0.0, 0.0), (0.2, 0.1)],
            vehicle_x=0.2,
            vehicle_y=0.1,
            vehicle_yaw=0.3,
            vehicle_length_m=0.217,
            vehicle_width_m=0.210,
            safety_margin_m=0.015,
        )
        self.view = ArenaView.__new__(ArenaView)
        self.view.controller = controller
        self.view.width_m = 2.4
        self.view.height_m = 1.8
        self.view.canvas = Canvas()

    def test_coordinate_transform_round_trip(self):
        canvas = self.view.to_canvas(1.2, 0.9)
        world = self.view.to_world(*canvas)
        self.assertAlmostEqual(1.2, world[0])
        self.assertAlmostEqual(0.9, world[1])

    def test_ros_map_axes_follow_start_pose_transform(self):
        origin, map_x_end, map_y_end = map_axes_in_arena(
            {"position_m": [1.6, 4.3], "heading_deg": -90.0},
            axis_length_m=0.5,
        )

        self.assertEqual((1.6, 4.3), origin)
        self.assertAlmostEqual(1.6, map_x_end[0])
        self.assertAlmostEqual(3.8, map_x_end[1])
        self.assertAlmostEqual(2.1, map_y_end[0])
        self.assertAlmostEqual(4.3, map_y_end[1])

    def test_draw_renders_map_route_trace_and_vehicle(self):
        self.view.draw()
        operations = [call[0] for call in self.view.canvas.calls]
        self.assertIn("create_rectangle", operations)
        self.assertIn("create_line", operations)
        self.assertIn("create_polygon", operations)
        self.assertIn("create_oval", operations)
        self.assertIn("create_text", operations)
        axis_lines = [
            call
            for call in self.view.canvas.calls
            if call[0] == "create_line" and call[2].get("fill") == MAP_AXIS
        ]
        self.assertEqual(2, len(axis_lines))

    def test_every_output_path_point_is_a_small_red_dot(self):
        route = [
            (0.0, 0.0, 0.0),
            (0.4, 0.0, 0.0),
            (0.8, 0.0, 0.0),
            (1.0, 0.2, math.pi / 2.0),
        ]
        self.view.controller.layer_routes = {1: route}

        self.view.draw()

        points = [
            call
            for call in self.view.canvas.calls
            if call[0] == "create_oval" and call[2].get("fill") == PATH_POINT
        ]
        self.assertEqual(len(route), len(points))
        self.assertTrue(all(call[1][2] - call[1][0] == 3.5 for call in points))

    def test_completed_preview_stages_remain_in_their_own_colours(self):
        self.view.controller.layer_routes = {
            1: [(0.0, 0.0, 0.0), (0.8, 0.0, 0.0)],
            2: [(0.8, 0.0, 0.0), (0.8, 0.8, math.pi / 2.0)],
            3: [(0.8, 0.8, 0.0), (1.6, 0.8, 0.0)],
        }

        self.view.draw()

        line_colours = {
            call[2].get("fill")
            for call in self.view.canvas.calls
            if call[0] == "create_line"
        }
        self.assertTrue({"#2563eb", "#16a34a", "#9333ea"}.issubset(line_colours))

    def test_vehicle_body_has_a_single_pointed_nose(self):
        self.view.draw()
        bodies = [
            call
            for call in self.view.canvas.calls
            if call[0] == "create_polygon" and call[2].get("fill") == CAR
        ]

        self.assertEqual(1, len(bodies))
        self.assertEqual(10, len(bodies[0][1]))

    def test_running_vehicle_draws_acknowledged_active_route(self):
        self.view.controller.mission_running = True
        self.view.controller.active_route = [
            (0.2, 0.1, 0.0),
            (0.2, 1.4, math.pi / 2.0),
        ]
        self.view.draw()
        active_lines = [
            call
            for call in self.view.canvas.calls
            if call[0] == "create_line" and call[2].get("fill") == "#ea580c"
        ]
        self.assertTrue(active_lines)

    def test_running_vehicle_never_falls_back_to_stale_preview(self):
        self.view.controller.mission_running = True
        self.view.controller.active_route = []
        self.view.draw()
        preview_lines = [
            call
            for call in self.view.canvas.calls
            if call[0] == "create_line" and call[2].get("fill") == "#2563eb"
        ]
        self.assertEqual([], preview_lines)


if __name__ == "__main__":
    unittest.main()
