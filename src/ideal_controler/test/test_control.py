import math
from pathlib import Path

from ideal_controler.control import arc_command, fillet_path, normalize


def test_arc_command_turns_toward_left_target():
    linear, angular = arc_command(0.0, 0.0, 0.0, 1.0, 1.0, 0.08, 2.0, 0.65)
    assert linear == 0.08
    assert angular > 0.0


def test_arc_command_clamps_angular_speed():
    _, angular = arc_command(0.0, 0.0, 0.0, 0.01, 0.10, 0.08, 2.0, 0.65)
    assert math.isclose(angular, 0.65)


def test_normalize_wraps_angle():
    assert math.isclose(normalize(3.0 * math.pi), math.pi, abs_tol=1.0e-9)


def test_fillet_path_replaces_right_angle_with_circle_samples():
    path = fillet_path([(0.0, 0.0), (1.0, 0.0), (1.0, -1.0)], 0.1)
    assert len(path) > 5
    assert path[1] == (0.9, 0.0)
    assert math.isclose(path[-2][0], 1.0, abs_tol=1.0e-9)
    assert math.isclose(path[-2][1], -0.1, abs_tol=1.0e-9)


def test_ideal_obstacle_launch_has_an_isolated_loopback_stack():
    launch = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "ideal_obstacle_test.launch.py"
    ).read_text()

    assert 'vehicle_loopback.launch.py' in launch
    assert 'planner_control_loopback.launch.py' not in launch
    assert 'use_final_navigation' not in launch
    assert 'use_debug_controller' not in launch
    assert 'executable="ideal_controler"' in launch
    assert '"enforce_static_collision": "false"' in launch
