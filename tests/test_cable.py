"""手腕电缆防缠测试：等效角、反向走线、累计扭转、上限拦截。

    python tests/test_cable.py
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import cable, kin  # noqa: E402


class TestWrap(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(cable.wrap180(10), 10)
        self.assertAlmostEqual(cable.wrap180(-10), -10)
        self.assertAlmostEqual(cable.wrap180(190), -170)
        self.assertAlmostEqual(cable.wrap180(-190), 170)
        self.assertAlmostEqual(cable.wrap180(360), 0)


class TestTrack(unittest.TestCase):
    def test_mark_resets(self):
        g = cable.TwistGuard(joints=(3, 5), limit_deg=360)
        g.mark([0] * 6)
        g.observe([10, 0, 0, 45, 0, 0])
        self.assertAlmostEqual(g.twist[3], 45)
        g.mark([10, 0, 0, 45, 0, 0])
        self.assertAlmostEqual(g.twist[3], 0)

    def test_wrap_across_180(self):
        g = cable.TwistGuard(joints=(3,), limit_deg=360)
        g.mark([0] * 6)
        g.observe([0, 0, 0, 170, 0, 0])     # +170
        g.observe([0, 0, 0, -170, 0, 0])    # 跨过 ±180：实际 +20（到 190）
        self.assertAlmostEqual(g.twist[3], 190)

    def test_ignores_unprotected_joints(self):
        g = cable.TwistGuard(joints=(3,), limit_deg=360)
        g.mark([0] * 6)
        g.observe([50, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(g.twist[0], 0)


class TestChoose(unittest.TestCase):
    def test_short_way_when_unwound(self):
        g = cable.TwistGuard(joints=(3,), limit_deg=360)
        g.mark([0] * 6)
        out, notes, blocked = g.choose([0, 0, 0, 74.5, 0, 0], [0, 0, 0, 0, 0, 0])
        self.assertFalse(blocked)
        self.assertAlmostEqual(out[3], 0)
        self.assertEqual(notes, [])

    def test_short_way_when_target_near_minus_360(self):
        # 当前 -350°，目标 0°：走 -10° 到 -360 比走 +350° 到 0 更近
        g = cable.TwistGuard(joints=(3,), limit_deg=360)
        g.mark([0, 0, 0, -350, 0, 0])
        out, notes, blocked = g.choose([0, 0, 0, -350, 0, 0], [0, 0, 0, 0, 0, 0])
        self.assertFalse(blocked)
        self.assertAlmostEqual(out[3], -360)
        self.assertTrue(notes)

    def test_unwinds_when_wound_clockwise(self):
        # 累计 +350°，目标还是 0°：应当选 -360°，反方向把线松开
        g = cable.TwistGuard(joints=(3,), limit_deg=360)
        g.mark([0] * 6)
        g.twist[3] = 350.0
        out, notes, blocked = g.choose([0] * 6, [0] * 6)
        self.assertFalse(blocked)
        self.assertAlmostEqual(out[3], -360)
        self.assertIn("反方向", notes[0])

    def test_blocked_by_limit(self):
        g = cable.TwistGuard(joints=(3,), limit_deg=10)
        g.mark([0] * 6)
        out, notes, blocked = g.choose([0] * 6, [0, 0, 0, 90, 0, 0])
        self.assertTrue(blocked)
        self.assertTrue(notes)

    def test_no_gratuitous_spin_when_already_there(self):
        # 已经在中途目标上、扭转也不算大时，不该为了回缠凭空转一圈
        g = cable.TwistGuard(joints=(3,), limit_deg=360)
        g.mark([0] * 6)
        g.twist[3] = 200.0
        out, notes, blocked = g.choose([0, 0, 0, 200, 0, 0], [0, 0, 0, 200, 0, 0])
        self.assertFalse(blocked)
        self.assertAlmostEqual(out[3], 200)
        self.assertEqual(notes, [])

    def test_respects_joint_limits(self):
        # 目标 350°，等效角只有 350 和 -10；-10 若超限则必须留在 350
        g = cable.TwistGuard(joints=(3,), limit_deg=360)
        g.mark([0] * 6)
        out, _, blocked = g.choose([0, 0, 0, 340, 0, 0], [0, 0, 0, 350, 0, 0])
        self.assertFalse(blocked)
        self.assertTrue(-360 <= out[3] <= 360)
        self.assertIn(out[3], (350.0, -10.0))

    def test_entry_state_shape(self):
        g = cable.TwistGuard(joints=(3, 5), limit_deg=270)
        g.mark([0] * 6)
        s = g.state()
        self.assertEqual(s["protected_joints"], [4, 6])
        self.assertIn("J4", s["twist_deg"])
        self.assertEqual(s["limit_deg"], 270.0)
        self.assertEqual(len(s["reference"]), 6)


class TestDefaults(unittest.TestCase):
    def test_default_protects_wrist_roll_and_flange(self):
        g = cable.TwistGuard()
        self.assertEqual(g.joints, (3, 5))
        self.assertEqual(g.limit, 360.0)

    def test_env_override(self):
        import os
        old = os.environ.get("HANDLINK_CABLE_JOINTS")
        os.environ["HANDLINK_CABLE_JOINTS"] = "6"
        try:
            self.assertEqual(cable.joints_from_env(), (5,))
        finally:
            if old is None:
                os.environ.pop("HANDLINK_CABLE_JOINTS", None)
            else:
                os.environ["HANDLINK_CABLE_JOINTS"] = old


if __name__ == "__main__":
    unittest.main(verbosity=2)
