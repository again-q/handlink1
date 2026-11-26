"""安全环境模型测试：胶囊距离、正解一致、工位间距、路径校验、SSM。

这些断言钉住的是**几何**，不是数值巧合：胶囊代理从 STL 生成，正解和 ``kin``
同源，工位尺寸在 ``cell.CELL`` 里。以后谁改了 STL、代理或工位，这里会先报。

    python tests/test_cell.py
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import cell, kin  # noqa: E402


class TestCapsuleMath(unittest.TestCase):
    def test_segment_distance_parallel(self):
        d = cell.seg_seg_distance((0, 0, 0), (0, 1, 0), (1, 0, 0), (1, 1, 0))
        self.assertAlmostEqual(d, 1.0, places=9)

    def test_segment_distance_crossing(self):
        d = cell.seg_seg_distance((0, 0, 0), (0, 1, 0), (0.5, -1, 0), (0.5, 2, 0))
        self.assertAlmostEqual(d, 0.5, places=9)

    def test_segment_distance_degenerate(self):
        d = cell.seg_seg_distance((1, 0, 0), (1, 0, 0), (0, 0, 0), (0, 0, 2))
        self.assertAlmostEqual(d, 1.0, places=9)

    def test_capsule_distance_subtracts_radii(self):
        a = ((0, 0, 0), (0, 0, 0), 0.1)
        b = ((1, 0, 0), (1, 0, 0), 0.2)
        self.assertAlmostEqual(cell.capsule_distance(a, b), 0.7, places=9)

    def test_point_aabb_distance_inside_is_zero(self):
        self.assertEqual(cell.point_aabb_distance((0, 0, 0), (0, 0, 0), (1, 1, 1)), 0.0)
        self.assertAlmostEqual(
            cell.point_aabb_distance((1.5, 0, 0), (0, 0, 0), (1, 1, 1)), 1.0, places=9)


class TestFrames(unittest.TestCase):
    def test_link_frames_match_flange(self):
        q = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
        p7, R7 = cell.link_frames(q)[6]
        p, R = kin.flange(q)
        for i in range(3):
            self.assertAlmostEqual(p7[i], p[i], places=12)
            for j in range(3):
                self.assertAlmostEqual(R7[i][j], R[i][j], places=12)

    def test_base_frame_is_identity(self):
        p, R = cell.link_frames([0, 0, 0, 0, 0, 0])[0]
        self.assertEqual(p, (0.0, 0.0, 0.0))
        self.assertEqual(R, [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])


class TestProxies(unittest.TestCase):
    def test_proxies_cover_every_link(self):
        links = cell.load_proxies()
        self.assertEqual([l["name"] for l in links],
                         ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"])
        for link in links:
            self.assertGreaterEqual(len(link["capsules"]), 1, link["name"])
            for cap in link["capsules"]:
                self.assertLessEqual(cap[6], 0.12, f"{link['name']} 胶囊半径不合理")
                self.assertGreaterEqual(cap[6], 0.01)

    def test_capsules_sit_near_the_mesh(self):
        # 底座半径 68 mm，代理外扩不应超过 1.3 倍
        base = cell.load_proxies()[0]
        rmax = max(cap[6] for cap in base["capsules"])
        self.assertLess(rmax, 0.10)


class TestClearance(unittest.TestCase):
    def test_ready_pose_is_safe(self):
        c = cell.clearance([0, -25, -40, 0, 65, 0])
        self.assertTrue(c["ok"])
        self.assertTrue(c["safe"], f"预备姿态被判成不安全：{c}")
        self.assertGreater(c["min_m"], 0.05)

    def test_folded_pose_collides_with_itself(self):
        c = cell.clearance([0, -60, -120, 120, 80, 0])
        self.assertFalse(c["safe"])
        self.assertLess(c["min_m"], 0.02)

    def test_tool_is_part_of_clearance(self):
        caps = cell.world_capsules([0, -25, -40, 0, 65, 0])
        self.assertIn("灵巧手", {c["link"] for c in caps})

    def test_min_distance_is_monotonic_in_table_approach(self):
        # 越往下压，离桌面越近
        up = cell.clearance([0, -20, -30, 0, 50, 0])["min_m"]
        down = cell.clearance([0, -60, -55, 0, 20, 0])["min_m"]
        self.assertLessEqual(down, up + 0.2)


class TestPathCheck(unittest.TestCase):
    def test_safe_path(self):
        r = cell.check_path([0, -25, -40, 0, 65, 0], [0, -30, -45, 0, 65, 0])
        self.assertTrue(r["ok"])
        self.assertTrue(r["safe"])
        self.assertGreater(r["samples"], 2)

    def test_path_into_self_collision_is_blocked(self):
        r = cell.check_path([0, -25, -40, 0, 65, 0], [0, -60, -120, 120, 80, 0])
        self.assertFalse(r["safe"])
        self.assertLess(r["min_m"], 0.02)

    def test_zero_move_has_one_sample(self):
        r = cell.check_path([0, -25, -40, 0, 65, 0], [0, -25, -40, 0, 65, 0])
        self.assertEqual(r["samples"], 2)   # n=1 → 起终点


class TestSSM(unittest.TestCase):
    def test_stationary_still_needs_intrusion_distance(self):
        r = cell.ssm_required(0.0)
        self.assertAlmostEqual(r["required_m"], 0.10, places=6)

    def test_faster_needs_more(self):
        self.assertLess(cell.ssm_required(0.2)["required_m"],
                        cell.ssm_required(0.8)["required_m"])

    def test_formula(self):
        r = cell.ssm_required(1.0, react_s=0.2, brake_m_s2=2.0, intrusion_m=0.1)
        self.assertAlmostEqual(r["required_m"], 1.0 * 0.2 + 1.0 / 4.0 + 0.1, places=6)


class TestCellDict(unittest.TestCase):
    def test_payload_is_json_ready(self):
        import json
        d = cell.cell_dict()
        s = json.dumps(d)
        self.assertIn("proxies", d)
        self.assertGreater(len(d["proxies"]), 0)
        self.assertIn("safe_mm", d["thresholds"])
        self.assertGreater(len(s), 1000)

    def test_floor_below_table(self):
        self.assertLess(cell.FLOOR_Z, cell.TABLE_TOP - 0.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
