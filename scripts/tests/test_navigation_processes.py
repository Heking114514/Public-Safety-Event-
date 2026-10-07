#!/usr/bin/env python3

from pathlib import Path
import subprocess
import tempfile
import unittest


WORKSPACE = Path(__file__).resolve().parents[2]
HELPER = WORKSPACE / "scripts" / "navigation_processes.sh"


class NavigationProcessTest(unittest.TestCase):
    def classify_self_with(self, function, *arguments):
        script = f"""
set -euo pipefail
WORKSPACE_ROOT={str(WORKSPACE)!r}
source {str(HELPER)!r}
{function} $$
"""
        return subprocess.run(
            ["bash", "-c", script, "navigation-process-test", *map(str, arguments)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=5,
        )

    def classify_self(self, *arguments):
        return self.classify_self_with("is_user_gui_pid", *arguments)

    def test_workspace_gui_scripts_are_preserved(self):
        gui_paths = [
            WORKSPACE / "scripts" / "arena_route_frontend.py",
            WORKSPACE / "scripts" / "arena_map_editor.py",
            WORKSPACE / "scripts" / "start_arena_planner.sh",
            WORKSPACE / "scripts" / "start_new_map_planner.sh",
            WORKSPACE / "src" / "new_map_planner" / "scripts" / "new_map_planner_ui",
            WORKSPACE / "route_editor.py",
            WORKSPACE / "src" / "visual_navigation" / "scripts" / "route_editor.py",
            WORKSPACE
            / "install"
            / "visual_navigation"
            / "lib"
            / "visual_navigation"
            / "route_editor.py",
        ]
        for path in gui_paths:
            with self.subTest(path=path):
                result = self.classify_self(path)
                self.assertEqual(0, result.returncode, result.stderr)

    def test_route_editor_ros_parents_are_preserved(self):
        for arguments in (
            ("ros2", "launch", "visual_navigation", "route_editor.launch.py"),
            ("ros2", "run", "visual_navigation", "route_editor.py"),
            ("ros2", "launch", "new_map_planner", "new_map_planner.launch.py"),
            (
                "ros2",
                "launch",
                "navigation_mission_manager",
                "mission_navigation.launch.py",
            ),
            ("ros2", "run", "new_map_planner", "new_map_planner_ui"),
        ):
            with self.subTest(arguments=arguments):
                result = self.classify_self(*arguments)
                self.assertEqual(0, result.returncode, result.stderr)

    def test_mission_manager_node_is_preserved(self):
        executable = (
            WORKSPACE
            / "install"
            / "navigation_mission_manager"
            / "lib"
            / "navigation_mission_manager"
            / "mission_manager_node"
        )
        result = self.classify_self_with("is_preserved_ros_pid", executable)

        self.assertEqual(0, result.returncode, result.stderr)

    def test_same_basename_outside_the_workspace_is_not_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            unrelated = Path(temporary) / "route_editor.py"
            unrelated.write_text("# unrelated\n", encoding="utf-8")
            result = self.classify_self(unrelated)
        self.assertNotEqual(0, result.returncode)

    def test_old_arena_planner_is_gone_before_restart_returns(self):
        planner = subprocess.Popen(
            [
                "bash",
                "-c",
                "trap 'exit 0' TERM; while true; do sleep 0.05; done",
                "/opt/ros/humble/bin/ros2",
                "launch",
                "arena_path_planner",
                "arena_path_planner.launch.py",
            ]
        )
        script = f"""
set -euo pipefail
WORKSPACE_ROOT={str(WORKSPACE)!r}
log() {{ :; }}
source {str(HELPER)!r}
find_ros_processes() {{ printf '%s\\n' {planner.pid}; }}
stop_arena_planner_processes
"""
        try:
            result = subprocess.run(
                ["bash", "-c", script],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=8,
            )
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertIsNotNone(planner.poll())
        finally:
            if planner.poll() is None:
                planner.kill()
            planner.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
