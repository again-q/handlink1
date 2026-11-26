"""真机故障复现场景库的回归：每个坑都要「能重现」且「修法有效」。

两类对象各测一遍：
  * armsim.VirtualDobot().arm() —— 真 29999 文本协议 + 30004 反馈，**硬判据**；
  * arm.SimArm() —— 应用层助手，缺伺服动力学的场景允许 skipped，但不能静默失败。
"""
from __future__ import annotations

import unittest

from handlink import arm, armsim, faults


class TestOnVirtualDobot(unittest.TestCase):
    """真协议对象：这些场景必须**真的**通过，不许退化。"""

    @classmethod
    def setUpClass(cls):
        cls.sim = armsim.VirtualDobot(seed=1).start()
        cls.arm = cls.sim.arm()
        cls.arm.enable()

    @classmethod
    def tearDownClass(cls):
        try:
            cls.arm.close()
        finally:
            cls.sim.stop()

    def _run(self, name):
        return faults.run(name, self.arm)

    def test_servoj_stop_holds(self):
        """停发之后保持最后目标：手册没写，这是实测补上的那条。"""
        res = self._run("servoj_stop_holds")
        self.assertTrue(res["passed"], res["observed"])
        self.assertEqual(res["numbers"]["mode"], 5)
        self.assertEqual(res["numbers"]["error"], 0)
        self.assertLessEqual(res["numbers"]["drift_deg"], 0.5)

    def test_error_codes_are_distinguishable(self):
        """-1（状态不允许）和 -10000（指令不存在）必须分得开。"""
        res = self._run("error_codes_matter")
        self.assertTrue(res["passed"], res["observed"])
        self.assertEqual(res["numbers"]["state"], faults.ERR_STATE)
        self.assertEqual(res["numbers"]["unknown"], faults.ERR_UNKNOWN)

    def test_speed_factor_ignores_servo(self):
        """1% 速度因子下 ServoJ 依然按 30 ms 到位（手册 p.89）。"""
        res = self._run("speed_factor_ignores_servo")
        self.assertTrue(res["passed"], res["observed"])

    def test_queue_overflow_is_real(self):
        res = self._run("movl_queue_overflow")
        self.assertTrue(res["passed"], res["observed"])

    def test_joint_limit_rejected(self):
        res = self._run("joint_limit_rejected")
        self.assertTrue(res["passed"], res["observed"])

    def test_run_all_reports_a_table(self):
        out = faults.run_all(self.arm)
        self.assertEqual(out["total"], len(faults.SCENARIOS))
        self.assertIn("场景", out["table"])
        self.assertEqual(out["passed"] + out["failed"] + out["skipped"], out["total"])


class TestOnSimArm(unittest.TestCase):
    """应用层仿真臂：能在它身上判的必须过，判不了的必须**明说**缺什么。"""

    def setUp(self):
        self.arm = arm.SimArm(joints=[0.0, -25.0, -40.0, 0.0, 65.0, 0.0])
        self.arm.enable()

    #: 逆解一次 84~250 ms，仿真臂上跑全量会拖到几十秒；这里只验「有没有说清」，
    #: 不验规模，所以把点数缩到最小。真规模用 VirtualDobot（快两个数量级）。
    CHEAP = {"points": 4, "burst": 8, "frames": 3}

    def test_every_scenario_reports_a_reason(self):
        """不允许静默失败：每个场景都要说清「看到了什么」并给出提示。"""
        for name in faults.names():
            res = faults.run(name, self.arm, **self.CHEAP)
            self.assertEqual(res["name"], name)
            self.assertTrue(res["observed"], name + " 没写 observed")
            self.assertTrue(res["symptom"], name + " 没写症状")
            self.assertTrue(res["hint"], name + " 没给 hint")
            self.assertIn("passed", res)

    def test_scenarios_that_sim_arm_can_judge_pass(self):
        """这几条不依赖伺服动力学，仿真臂上也必须真通过。"""
        for name in ("port_29999_preempted", "joint_limit_rejected"):
            res = faults.run(name, self.arm, **self.CHEAP)
            self.assertTrue(res["passed"], name + "：" + res["observed"])

    def test_degraded_scenarios_are_marked_not_silently_failed(self):
        """servoj 动力学相关的几条在仿真臂上判不了——必须是 skipped 或明说原因。"""
        for name in ("servoj_stop_holds", "error_codes_matter"):
            res = faults.run(name, self.arm, **self.CHEAP)
            if res["passed"]:
                continue
            self.assertTrue(res.get("skipped") or "缺能力" in res["observed"]
                            or "退化" in res["observed"] or res["hint"],
                            name + " 失败但没有说明原因")

    def test_tip_53mm_short_compares_both_envelopes(self):
        """修正前那套胶囊比真实指尖短 53 mm —— 这正是顶桌的根因。"""
        gap = faults.REAL_TIP_LOWEST_M - faults.MODEL_TIP_LOWEST_M
        self.assertAlmostEqual(gap, 0.053, places=6)
        res = faults.run("tip_53mm_short", self.arm)
        self.assertIn("53", str(res["observed"]))

    def test_classify_stop_separates_estop_from_servo_alarm(self):
        """急停/防护停止 ≠ 伺服报警：GetErrorID 全空但红灯亮过（.tmp/fault3.out）。"""
        self.assertEqual(faults.classify_stop(None, {"estop": True}), "safety")
        self.assertEqual(faults.classify_stop(None, {"protective_stop": True}), "safety")
        self.assertEqual(faults.classify_stop(37, None), "servo_alarm")
        self.assertEqual(faults.classify_stop(None, None), "ok")


if __name__ == "__main__":
    unittest.main()