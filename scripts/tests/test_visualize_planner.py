import json
import subprocess
import sys
from pathlib import Path

import yaml

SCRIPTS = Path(__file__).resolve().parents[1]
WORKSPACE = SCRIPTS.parent
CONFIG = WORKSPACE / "src" / "new_map_planner" / "config" / "arena.yaml"
sys.path.insert(0, str(SCRIPTS))

from visualize_planner import build_trace, dijkstra, load_config  # noqa: E402


def test_mission_trace_contains_route_metadata_and_resets_count():
    _, nodes, lanes, free, static, length, width, margin = load_config(CONFIG)
    route, _ = dijkstra(nodes, lanes, free, static, length, width, margin, "launch", "x1_y4",
                        blocked_lane_ids={"ROAD_TOP_LEFT"})
    trace = build_trace(CONFIG, route, lanes, {"ROAD_TOP_LEFT"}, 9, 0,
                        "launch", "x1_y4", nodes)
    assert trace["route_lanes"]
    assert all(item["lane_id"] != "ROAD_TOP_LEFT" for item in trace["route_lanes"])
    assert trace["obstacle_revision"] == 9
    assert trace["blocked_lane_ids"] == ["ROAD_TOP_LEFT"]
    assert trace["nodes"][-1]["supernode_id"] == "s1"
    assert trace["ordinary_count"] == 0
    assert all(not set(c["recovery_edges"]) & {"ROAD_TOP_LEFT"}
               for c in trace["recovery_candidates"])


def test_cli_writes_json_trace_without_ros():
    output = Path("/tmp/planner_visualizer_test_trace.json")
    result = subprocess.run([
        sys.executable, str(SCRIPTS / "visualize_planner.py"),
        "--config", str(CONFIG), "--mode", "mission", "--trace", str(output),
        "--blocked-lane", "ROAD_TOP_LEFT", "--obstacle-revision", "3",
        "--save", "/tmp/planner_visualizer_test.png",
    ], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["obstacle_revision"] == 3
    assert "route_lanes" in payload
