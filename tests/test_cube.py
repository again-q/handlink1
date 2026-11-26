"""走步工具的单元测试：不接魔方、不接硬件。"""
from __future__ import annotations

import unittest

from handlink import cube


class TestParse(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(cube.parse_moves("R U R' U2"),
                         [("R", 1), ("U", 1), ("R", 3), ("U", 2)])

    def test_curly_prime(self):
        self.assertEqual(cube.parse_moves("U’"), [("U", 3)])
        self.assertEqual(cube.parse_moves("F′"), [("F", 3)])

    def test_bad(self):
        with self.assertRaises(ValueError):
            cube.parse_moves("X")
        with self.assertRaises(ValueError):
            cube.parse_moves("R3")
        with self.assertRaises(ValueError):
            cube.parse_moves("R4")


class TestConvert(unittest.TestCase):
    def test_format(self):
        self.assertEqual(cube.format_moves([("R", 1), ("U", 2), ("F", 3)]),
                         "R U2 F3")

    def test_invert(self):
        moves = cube.parse_moves("R U' F2")
        self.assertEqual(cube.format_moves(cube.invert(moves)), "F2 U R3")
        self.assertEqual(cube.invert(cube.invert(moves)), moves)

    def test_identity(self):
        self.assertEqual(cube.format_moves(cube.group_by_face(cube.parse_moves("R R'"))), "")


class TestGroup(unittest.TestCase):
    def test_cancel_adjacent(self):
        self.assertEqual(cube.group_by_face(cube.parse_moves("R R' U U U")),
                         [("U", 3)])

    def test_merge_and_split(self):
        self.assertEqual(cube.group_by_face(cube.parse_moves("R U U2 R")),
                         [("R", 1), ("U", 3), ("R", 1)])

    def test_segments(self):
        self.assertEqual(cube.segments(cube.parse_moves("R U R'")),
                         [("R", 1), ("U", 1), ("R", 3)])

    def test_quarter_turns(self):
        self.assertEqual(cube.quarter_turns(cube.parse_moves("R U2 F'")), 6)


class TestPlan(unittest.TestCase):
    def test_regrasps_default(self):
        # 可以把第一段先摆到朝上，所以是段数 - 1
        self.assertEqual(cube.regrasps(cube.parse_moves("R U R'")), 2)

    def test_regrasps_first_face(self):
        moves = cube.parse_moves("R U R'")
        self.assertEqual(cube.regrasps(moves, first_face="R"), 2)
        self.assertEqual(cube.regrasps(moves, first_face="U"), 3)

    def test_estimate(self):
        est = cube.estimate_seconds(cube.parse_moves("R U R'"),
                                    turn_s=3.0, regrasp_s=20.0)
        self.assertEqual(est["quarter_turns"], 5)
        self.assertEqual(est["segments"], 3)
        self.assertEqual(est["regrasps"], 2)
        self.assertEqual(est["total_s"], 55.0)


if __name__ == "__main__":
    unittest.main()
