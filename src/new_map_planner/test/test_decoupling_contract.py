"""Static contract checks for the planner/controller boundary."""

import os
from pathlib import Path


def _planner_root() -> Path:
    source_dir = Path(__file__).resolve().parents[1]
    return Path(os.environ.get("NEW_MAP_PLANNER_SOURCE_DIR", source_dir))


def test_planner_has_no_final_navigation_dependency():
    root = _planner_root()
    checked_files = (
        root / "CMakeLists.txt",
        root / "package.xml",
        root / "include/new_map_planner/maneuver.hpp",
        root / "src/maneuver.cpp",
    )
    for path in checked_files:
        text = path.read_text(encoding="utf-8")
        assert "final_navigation" not in text, path


def test_planner_does_not_publish_motion_commands():
    root = _planner_root()
    source_files = list((root / "src").glob("*.cpp"))
    source_text = "\n".join(path.read_text(encoding="utf-8") for path in source_files)
    assert "geometry_msgs::msg::Twist" not in source_text
    assert "/cmd_vel_nav" not in source_text

