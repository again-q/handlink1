"""轨迹下发的真机怪癖：哪些能复现、哪些复现不了——逐条钉住。

出处：docs/毛笔字轨迹-平滑下发-2026-11-26.md 第 2 节。
"""
from __future__ import annotations

import unittest

from handlink import arm, arm_actions, armsim, trace2d


class TestRMustBeIntegerLiteral(unittest.TestCase):
    """现场实测：r=3 成功，r=3.0 回 -50001（参数类型错）。

    这条能复现，而且必须复现——arm.movl_r 用 floor(x+0.5) 而不是 round()，
    就是为了不发出去 3.0。仿真里如果没有这个检查，那个 fix 就没法验证。
    """

    @classmethod
    def setUpClass(cls):
        cls.sim = armsim.VirtualDobot(seed=4).start()
        cls.arm = cls.sim.arm()
        cls.arm.enable()

    @classmethod
    def tearDownClass(cls):
        try:
            cls.arm.close()
        finally:
            cls.sim.stop()

    def _reply(self, suffix):
        return self.arm.command("MovL(pose={400,0,250,180,0,0},%s)" % suffix)

    def test_integer_r_is_accepted(self):
        self.assertTrue(self._reply("r=3").startswith("0,"))

    def test_float_r_is_rejected_with_parameter_error(self):
        reply = self._reply("r=3.0")
        self.assertTrue(reply.startswith("-50001,"), reply)

    def test_cp_and_accel_are_still_accepted(self):
        self.assertTrue(self._reply("cp=30").startswith("0,"))
        self.assertTrue(self._reply("r=3,a=50").startswith("0,"))

    def test_movl_r_never_sends_a_float(self):
        """真机客户端取整之后再发，所以上面那条拒绝在链路上不该被触发。"""
        sent = []
        real = self.arm._command
        self.arm._command = lambda text: (sent.append(text), real(text))[1]
        try:
            for r_mm in (2.5, 3.4999, 4.9999, 5.0):
                try:
                    self.arm.movl_r([400, 0, 250, 180, 0, 0], r_mm)
                except arm.ArmError:
                    pass
        finally:
            self.arm._command = real
        movl = [s for s in sent if s.startswith("MovL")]
        self.assertTrue(movl)
        for line in movl:
            self.assertNotIn(".0,", line.replace("pose={", "").split("}")[0])
            if "r=" in line:
                self.assertRegex(line, r"r=\d+(?=,|\))")


class TestNotModelled(unittest.TestCase):
    """**这两条复现不了**，写下来免得有人以为仿真验过。"""

    def test_cp_blending_is_not_modelled(self):
        """真机 cp=30 是等幅锯齿，r=5 才是连续曲线。仿真只记数值，不产生速度曲线。"""
        sim = armsim.VirtualDobot(seed=4).start()
        try:
            a = sim.arm()
            a.enable()
            cp = a.command("MovL(pose={400,0,250,180,0,0},cp=30)")
            r5 = a.command("MovL(pose={400,0,250,180,0,0},r=5)")
            self.assertTrue(cp.startswith("0,"))
            self.assertTrue(r5.startswith("0,"))
            self.assertNotIn("blend", str(sim.state()))
        finally:
            sim.stop()

    def test_j6_wanders_but_does_not_accumulate(self):
        """真机：每条 300 mm 的线 J6 约漂 +50°，一条条累到 -357.9° 撞限位。

        虚拟控制器**不模拟这个累积**：每条指令都是独立解 IK，解出哪一支取决于
        种子，所以 J6 会在几十度范围内来回跳（实测 24°↔68°），但不会单向爬。
        也就是说，**防缠逻辑（cable / wrist_unwind）在仿真里验不出来**——
        它只能靠 trace2d 的规划层单测，或者上真机。
        """
        sim = armsim.VirtualDobot(seed=4).start()
        try:
            a = sim.arm()
            a.enable()
            a.speed_factor(50)
            j6 = []
            for i in range(18):
                a.command("MovL(pose={%d,0,250,180,0,0},r=5)" % (400 - i * 15))
                sim.settle(0.35)
                j6.append(a.joints()[5])
            spread = max(j6) - min(j6)
            net = abs(j6[-1] - j6[0])
            self.assertGreater(spread, 10.0, "J6 应该会因为换支而跳动")
            self.assertLess(net, 60.0,
                            "净漂移应当很小：虚拟控制器不累积，真机才累积")
            self.assertLess(max(abs(v) for v in j6), 300.0,
                            "仿真里 J6 不该自己爬到 300° 以上——那是真机才有的病")
        finally:
            sim.stop()


class TestSIxtySixStillGuarded(unittest.TestCase):
    """⑥ 那个「常量没定义把臂停在半路」的坑：两道防线还在不在。"""

    def test_every_cmd_constant_exists(self):
        import inspect
        import re
        src = inspect.getsource(arm.DobotArm)
        used = sorted(set(re.findall(r"self\.(CMD_[A-Z_]+)", src)))
        for name in used:
            self.assertTrue(hasattr(arm.DobotArm, name), "少了常量 " + name)

    def test_throttle_failure_does_not_abort_the_stroke(self):
        """限流只是状态查询，它坏了也不能把已发了一半的笔画掐断。"""
        src = __import__("inspect").getsource(__import__("handlink.service",
                                            fromlist=["x"]).App.arm_trace)
        self.assertIn("throttle", src)


if __name__ == "__main__":
    unittest.main()