"""独立仿真的前进约束、降级优先级和返航路线回归。"""

from collections import Counter
import math
from pathlib import Path
import sys
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
from simulate_route import Arena, CORNER_OBSTACLE_RFID, solve


class RouteSimulationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = yaml.safe_load((PACKAGE / "config/arena.yaml").read_text(encoding="utf-8"))

    def audit_forward(self, arena, result):
        self.assertIn(result["status"], {"SUCCESS", "PARTIAL_SUCCESS"})
        self.assertTrue(result["steps"])
        node, heading, ordinary = arena.home, arena.start_heading, 0
        tasks, tunnels, counts = set(), set(), Counter()
        for step in result["steps"]:
            self.assertEqual(step["from"], node)
            self.assertEqual(step["motion"], "forward")
            self.assertNotIn("reverse", step["motion"])
            lid, target = step["lane_id"], step["to"]
            self.assertNotIn(lid, arena.blocked)
            lane = arena.lanes[lid]
            self.assertEqual({node, target}, {lane["from"], lane["to"]})
            outgoing = step["heading_index"]
            dx = arena.nodes[target][0] - arena.nodes[node][0]
            dy = arena.nodes[target][1] - arena.nodes[node][1]
            hx, hy = [(1, 0), (0, 1), (-1, 0), (0, -1)][outgoing]
            self.assertAlmostEqual(dx * hy - dy * hx, 0.0)
            self.assertGreater(dx * hx + dy * hy, 0.0)
            self.assertNotEqual((outgoing - heading) % 4, 2)
            task = arena.rfid.get(target)
            ordinary = 0 if task else ordinary + 1
            if ordinary > 2:
                self.assertTrue(step["rfid_corridor"], "第三个普通节点只能来自完整 RFID 共线例外")
            self.assertEqual(step["ordinary_after"], ordinary)
            self.assertEqual(step["rfid_stop_s"], int(bool(task)))
            self.assertEqual(step["announce"], bool(task and task not in tasks))
            if task:
                tasks.add(task)
            if lane.get("tunnel"):
                tunnels.add(lid)
            counts[lid] += 1
            self.assertEqual(step["pass_number"], counts[lid])
            node, heading = target, outgoing
        self.assertEqual(node, arena.home)
        self.assertEqual(counts, result["lane_counts"])
        self.assertEqual(result["returned_home"], True)
        self.assertEqual(result["reverse_steps"] if "reverse_steps" in result else 0, 0)
        self.assertEqual(result["reverse_distance_m"] if "reverse_distance_m" in result else 0, 0)
        self.assertEqual(result["rfid_completed"], len(tasks.intersection(set(arena.required_rfid.values()))))
        self.assertEqual(result["tunnels_completed"], len(tunnels))

        tagged = [step for step in result["steps"] if step["rfid_corridor"]]
        groups = []
        for step in tagged:
            if not groups or groups[-1][-1]["rfid_corridor"] != step["rfid_corridor"]:
                groups.append([])
            groups[-1].append(step)
        for group in groups:
            self.assertIn(group[0]["from"], arena.rfid)
            self.assertIn(group[-1]["to"], arena.rfid)
            self.assertEqual({step["heading_index"] for step in group}, {group[0]["heading_index"]})
            self.assertEqual({step["motion"] for step in group}, {"forward"})

    def test_no_obstacles_completes_all_targets(self):
        arena = Arena(self.data)
        result = solve(arena, timeout=0.3, max_states=80000)
        self.audit_forward(arena, result)
        self.assertEqual(result["status"], "SUCCESS")
        self.assertEqual(result["rfid_completed"], 12)
        self.assertEqual(result["tunnels_completed"], 4)

    def test_screenshot_edges_keep_rfid_and_drop_tunnel(self):
        arena = Arena(self.data, ["7,8", "6,10", "5,6"])
        result = solve(arena, timeout=0.5, max_states=80000)
        self.audit_forward(arena, result)
        self.assertEqual(result["status"], "PARTIAL_SUCCESS")
        self.assertEqual(result["rfid_completed"], 12)
        self.assertEqual(result["tunnels_completed"], 3)
        self.assertNotIn("TUNNEL_2", {step["tunnel"] for step in result["steps"] if step["tunnel"]})
        self.assertTrue(any(item["kind"] == "TUNNEL" and item["target"] == "TUNNEL_2"
                            for item in result["deferred_targets"]))

    def test_corner_edges_skip_only_their_rfid(self):
        for pair, node in CORNER_OBSTACLE_RFID.items():
            pair_text = ",".join(str(value) for value in sorted(pair))
            reverse_text = ",".join(str(value) for value in sorted(pair, reverse=True))
            first, second = Arena(self.data, [pair_text]), Arena(self.data, [reverse_text])
            self.assertEqual(first.blocked, second.blocked)
            self.assertNotIn(node, first.required_rfid)
            task = next(task for task, target in self.data["tasks"].items() if target == node)
            self.assertTrue(any(item["kind"] == "RFID" and item["target"] == task
                                for item in first.skipped_targets))

    def test_corner_skip_does_not_delete_adjacent_nodes(self):
        arena = Arena(self.data, ["1,2"])
        self.assertIn("x0_y3", arena.nodes)
        self.assertIn("ROAD_X0_Y2", {lid for lid in arena.lanes if lid not in arena.blocked})
        self.assertIn("ROAD_Y3_LEFT", arena.blocked)

    def test_collinear_rfid_corridor_is_explicit(self):
        arena = Arena(self.data)
        options = list(arena.transitions("x1_y4", 3, 0))
        macro = next(path for state, path, _ in options if len(path) > 1 and path[0][2])
        self.assertGreaterEqual(len(macro), 4)
        self.assertEqual(macro[-1][0][0], "x1_y0")
        self.assertTrue(all(edge[2] == macro[0][2] for edge in macro))
        self.assertEqual(macro[0][0][2], 1)
        self.assertEqual(macro[2][0][2], 3)
        self.assertEqual(macro[-1][0][2], 0)
        broken = Arena(self.data, ["10,14"])
        self.assertNotIn(("x1_y4", 3), broken.corridors)

    def test_budget_keeps_a_legal_return_route(self):
        arena = Arena(self.data)
        result = solve(arena, timeout=0.01, max_states=200000)
        self.audit_forward(arena, result)
        self.assertEqual(result["optimization_stop_reason"], "timeout")
        self.assertEqual(result["returned_home"], True)

    def test_reversed_pairs_are_deduplicated(self):
        first = Arena(self.data, ["1,2"])
        second = Arena(self.data, ["2,1", "(1,2)"])
        self.assertEqual(first.blocked, second.blocked)
        self.assertEqual(solve(first, timeout=0.2, max_states=60000)["status"],
                         solve(second, timeout=0.2, max_states=60000)["status"])

    def test_invalid_edges(self):
        for pair in ("2,3", "1,3", "0,1", "1,1", "(1,2"):
            with self.subTest(pair=pair), self.assertRaises(ValueError):
                Arena(self.data, [pair])

    def test_result_has_no_turnaround_or_reverse_actions(self):
        result = solve(Arena(self.data, ["7,8", "6,10", "5,6"]), timeout=0.3, max_states=60000)
        self.assertTrue(all(step["motion"] == "forward" for step in result["steps"]))
        self.assertTrue(all("倒车" not in step["action"] for step in result["steps"]))
        self.assertTrue(all((step["heading_index"] - before["heading_index"]) % 4 != 2
                            for before, step in zip([{"heading_index": 3}] + result["steps"], result["steps"])))


if __name__ == "__main__":
    unittest.main()
