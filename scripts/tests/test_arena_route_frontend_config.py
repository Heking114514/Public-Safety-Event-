#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest


WORKSPACE = Path(__file__).resolve().parents[2]
FRONTEND = WORKSPACE / "scripts" / "arena_route_frontend.py"


def odometry_topic_choices() -> set[str]:
    tree = ast.parse(FRONTEND.read_text(encoding="utf-8"))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == "ODOMETRY_TOPIC_CHOICES"
            for target in node.targets
        ):
            continue
        return set(ast.literal_eval(node.value))
    raise AssertionError("ODOMETRY_TOPIC_CHOICES not found")


class ArenaRouteFrontendConfigTest(unittest.TestCase):
    def test_frontend_accepts_navigation_odometry_sources(self):
        self.assertEqual(
            {
                "/odometry/local_map",
                "/odometry/fused",
                "/odometry/landmark_corrected",
            },
            odometry_topic_choices(),
        )


if __name__ == "__main__":
    unittest.main()
