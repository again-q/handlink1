"""抓取规划测试：反解一致性、抓取点几何、路径检查阈值、手的包络。

断言的几何关系（不写死具体关节角），这样以后换臂、换手、换工位，这里会先报。

    python tests/test_pickbox.py
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import cell, kin, pickbox  # noqa: E402


class TestKinematicsIK(unittest.TestCase):
    def test_ik_roundtrip_measured_pose(self):
        q = [112.0, 47.0, 66.5, 112.0, 107.5, -17.5]
        pose = kin.fk(q)
        solved = kin.ik(pose, seed=q)
        self.assertIsNotNone(solved, "实测姿态反解不出来")
        back = kin.fk(solved)
        self.assertLess(max(abs(back[i] - pose[i]) for i in range(3)), 1.0)
        for i in range(3, 6):
            d = (back[i] - pose[i] + 180.0) % 360.0 - 180.0
            self.assertLess(abs(d), 0.5)

    def test_ik_roundtrip_table_pose(self):
        # 桌面正上方、工具朝下的一个姿态（由控制器解出来的一组关节角）
        q = [211.5, 8.8, 97.5, 163.6, 90.0, 121.4]
        pose = kin.fk(q)
        solved = kin.ik(pose, seed=q)
        self.assertIsNotNone(solved)
        back = kin.fk(solved)
        self.assertLess(max(abs(back[i] - pose[i]) for i in range(3)), 1.0)


class TestHandModel(unittest.TestCase):
    def test_fingertips_are_thin(self):
        caps = cell.CELL["tool"]["capsules"]
        self.assertGreaterEqual(len(caps), 6, "手模型至少要有手掌 + 四指 + 拇指")
        thin = [c for c in caps if c[2] <= 0.012]
        self.assertGreaterEqual(len(thin), 4, "指尖段至少要 4 根细胶囊")
        # 模型的最低点必须和 pickbox 用的常数一致。2026-11-18 就是这一条没守住：
        # 模型最低点 0.194、实际 0.247，模型报「离桌面还剩 55 mm」时指尖其实
        # 已经贴上了，臂顶着桌子走了 30 s。两边只要差一点，安全距离就系统性骗人。
        lowest = max(max(c[0][2], c[1][2]) + c[2] for c in caps)
        self.assertAlmostEqual(lowest, pickbox.TOOL_TIP_M, places=3)

    def test_grasp_pose_clears_table(self):
        # 指尖停在 60 mm 盒子高度一半（30 mm），手不能“插进”桌面。
        flange_z = 0.030 + pickbox.TOOL_TIP_M
        tip_z = flange_z - pickbox.TOOL_TIP_M
        self.assertAlmostEqual(tip_z, 0.030, places=6)
        # 工具 +Z 朝下时，世界 z = 法兰 z −（工具 z + 半径）
        deepest = max(max(c[0][2], c[1][2]) + c[2]
                      for c in cell.CELL["tool"]["capsules"])
        self.assertGreater(flange_z - deepest, 0.010,
                           "手模型的最低点离桌面不到 10 mm")


class TestPlanGeometry(unittest.TestCase):
    def setUp(self):
        self.t = pickbox.GraspTarget(name="盒子", x=0.15, y=0.25, w=0.08, d=0.08,
                                     h=0.06, yaw=0.0)
        self.plan = pickbox.build_plan(self.t, [-98.0, 241.0, 516.0, 118.0, -61.0, -113.0])

    def test_heights_are_ordered(self):
        self.assertGreater(self.plan.transit[2], self.plan.pre[2])
        self.assertGreater(self.plan.pre[2], self.plan.grasp[2])
        self.assertGreater(self.plan.lift[2], self.plan.grasp[2])

    def test_tip_height(self):
        tip = self.plan.grasp[2] / 1000.0 - pickbox.TOOL_TIP_M
        self.assertAlmostEqual(tip, 0.030, places=6)      # 60 mm 盒子的一半

    def test_position_and_yaw(self):
        self.assertAlmostEqual(self.plan.grasp[0], 150.0, places=6)
        self.assertAlmostEqual(self.plan.grasp[1], 250.0, places=6)
        self.assertEqual(self.plan.grasp[3:], [180.0, 0.0, 0.0])

    def test_hand_yaw_rotates(self):
        plan = pickbox.build_plan(self.t, [0, 0, 0, 0, 0, 0], hand_yaw=30.0)
        self.assertAlmostEqual(plan.grasp[5], 30.0, places=6)

    def test_rejects_big_target(self):
        big = pickbox.GraspTarget("大盒", 0.1, 0.1, 0.2, 0.2, 0.06)
        with self.assertRaises(pickbox.PickPlanError):
            pickbox.build_plan(big, [0] * 6)

    def test_rejects_unreachable(self):
        far = pickbox.GraspTarget("远", 0.9, 0.0, 0.08, 0.08, 0.06)
        with self.assertRaises(pickbox.PickPlanError):
            pickbox.build_plan(far, [0] * 6)


class _StubArm:
    """假臂：固定关节角，反解直接返回预设值，用来单独测路径检查。"""

    def __init__(self, joints, ik_result):
        self._joints = list(joints)
        self._ik = list(ik_result)

    def joints(self):
        return list(self._joints)

    def ik(self, pose, near=None):
        return list(self._ik)


class TestPathCheck(unittest.TestCase):
    def setUp(self):
        self.plan = pickbox.build_plan(
            pickbox.GraspTarget("盒子", 0.15, 0.25, 0.08, 0.08, 0.06),
            [-98.0, 241.0, 516.0, 118.0, -61.0, -113.0])

    def test_threshold_is_passed_through(self):
        q = [211.5, 8.8, 97.5, 163.6, 90.0, 121.4]
        arm = _StubArm(q, q)
        report = pickbox.check_plan(self.plan, arm, safe_mm=10.0, step_mm=50.0)
        self.assertEqual(report["safe_mm"], 10.0)
        self.assertTrue(all(r["safe"] for r in report["rows"]))
        self.assertGreater(report["min_mm"], 9.0)

    def test_blocked_path_raises(self):
        # 让反解返回一个“手贴着桌面”的关节角，10 mm 阈值下必须拒绝
        q_touch = [211.5, 30.8, 117.6, 121.5, 90.0, 121.4]
        arm = _StubArm(q_touch, q_touch)
        with self.assertRaises(pickbox.PickPlanError):
            pickbox.check_plan(self.plan, arm, safe_mm=60.0, step_mm=50.0)


class TestClearanceThreshold(unittest.TestCase):
    def test_safe_mm_override(self):
        # 间距落在 10～60 mm 之间：60 mm 判不安全、10 mm 判安全，才验得出来。
        # 姿态随工具长度改过一次（工具从 0.194 修正到 0.247），这里按新长度取。
        joints = [211.5, 20.8, 109.6, 121.5, 90.0, 121.4]
        c60 = cell.clearance(joints, safe_mm=60.0)
        c10 = cell.clearance(joints, safe_mm=10.0)
        self.assertGreaterEqual(c60["min_m"], 0.0)
        self.assertEqual(c60["min_m"], c10["min_m"])
        self.assertLessEqual(c10["min_m"], 0.06)
        self.assertFalse(c60["safe"])
        self.assertTrue(c10["safe"])

    def test_check_path_accepts_safe_mm(self):
        a = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        b = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        r = cell.check_path(a, b, safe_mm=1.0)
        self.assertTrue(r["ok"])
        self.assertIn("samples", r)


if __name__ == "__main__":
    unittest.main(verbosity=2)
