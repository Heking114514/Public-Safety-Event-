import math
from types import SimpleNamespace

from vehicle_loopback_sim.route_control import Pose2D, Waypoint, parse_path, project


def test_parse_path_reads_turn_junction_marker():
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=1.0e-3),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    path = SimpleNamespace(poses=[SimpleNamespace(pose=pose)])

    route = parse_path(path)

    assert len(route) == 1
    assert route[0].x == 1.0
    assert route[0].y == 2.0
    assert route[0].turn_junction


def test_project_reports_remaining_and_left_cross_track():
    start = Waypoint(0.0, 0.0, 0.0, False)
    target = Waypoint(1.0, 0.0, 0.0, False)
    pose = Pose2D(0.25, 0.10, 0.0)

    projection = project(start, target, pose)

    assert projection.valid
    assert math.isclose(projection.remaining, 0.75)
    assert math.isclose(projection.cross_track, 0.10)
    assert math.isclose(projection.heading, 0.0)
