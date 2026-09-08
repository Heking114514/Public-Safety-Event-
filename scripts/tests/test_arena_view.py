#!/usr/bin/env python3

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))

from arena_view import ArenaView  # noqa: E402


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
            config={"tunnels": {}},
            tasks=[("1", [0.6, 0.6])],
            mode_var=Value("vehicle"),
            dynamic_obstacles=[(1.0, 0.4, 1.2, 0.6)],
            layer_routes={1: [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0)]},
            layer_display_segments={},
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

    def test_draw_renders_map_route_trace_and_vehicle(self):
        self.view.draw()
        operations = [call[0] for call in self.view.canvas.calls]
        self.assertIn("create_rectangle", operations)
        self.assertIn("create_line", operations)
        self.assertIn("create_polygon", operations)
        self.assertIn("create_oval", operations)
        self.assertIn("create_text", operations)


if __name__ == "__main__":
    unittest.main()
