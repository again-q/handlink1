"""服务跑在虚拟 Nova 5 上时，三维视图这条链路必须和真机一致。

重点不是「接口能返回 200」，而是：使能真的改了 EnableStatus、
运动真的会经过 RobotMode=7、报警真的会让使能被拒——这些判据全在服务代码里，
仿真如果绕过 socket，它们就永远测不到。
"""
from __future__ import annotations

import time
import unittest

from handlink import arm as arm_mod
from handlink import armsim, bus, service


class TestServiceOnVirtualDobot(unittest.TestCase):
    def setUp(self):
        self.app = service.App(bus.open_sim())
        self.assertTrue(self.app.attach_sim_arm(seed=5))
        self.sim: armsim.VirtualDobot = self.app.arm_sim

    def tearDown(self):
        self.app.close_arm()

    def _feed(self, key):
        return int(self.app.arm_feed.snapshot()[key])

    def test_enable_then_disable_flips_the_feed(self):
        out = self.app.arm_enable(True)
        self.assertTrue(out["ok"], out)
        self.assertEqual(self._feed("EnableStatus"), 1)
        out = self.app.arm_enable(False)
        self.assertTrue(out["ok"], out)
        self.assertEqual(self._feed("EnableStatus"), 0)

    def test_refuses_to_enable_while_alarming(self):
        """真机：有报警时使能会被拒，必须先 ClearError。"""
        self.sim.inject("alarm", code=37)
        time.sleep(0.2)
        out = self.app.arm_enable(True)
        self.assertFalse(out["ok"], out)
        self.assertIn("报警", str(out.get("error", "")) + str(out))

    def test_moving_state_is_visible_to_the_service(self):
        """三维视图靠 30004 的 RobotMode 判断「在动」；仿真必须真的出现 7。"""
        robot = self.app._robot()
        try:
            robot.enable()
            robot.speed_factor(20)
            target = [0.0, -10.0, -30.0, 0.0, 50.0, 0.0]
            robot.move_joints(target, wait=False)
            seen = set()
            deadline = time.time() + 4.0
            while time.time() < deadline:
                seen.add(self._feed("RobotMode"))
                if 7 in seen and self._feed("RobotMode") == 5:
                    break
                time.sleep(0.01)
        finally:
            robot.close()
        self.assertIn(7, seen, "30004 里从没出现过「运行中」，三维视图会以为臂没动")

    def test_estop_latches_and_blocks_motion(self):
        self.sim.inject("estop")
        time.sleep(0.2)
        robot = self.app._robot()
        try:
            with self.assertRaises(arm_mod.ArmError):
                robot.enable()
        finally:
            robot.close()


if __name__ == "__main__":
    unittest.main()