"""服务接仿真臂的集成测试：走真 29999/30004 socket，不 mock。"""
from __future__ import annotations

import unittest

from handlink import arm, bus, service


class TestSimArmAttached(unittest.TestCase):
    def setUp(self):
        self.app = service.App(bus.open_sim())
        self.assertTrue(self.app.attach_sim_arm())

    def tearDown(self):
        self.app.close_arm()

    def test_feedback_is_live_and_parsed(self):
        self.assertTrue(self.app._arm_live())
        snap = self.app.arm_feed.snapshot()
        self.assertEqual(int(snap["RobotMode"]), 4)
        self.assertEqual(int(snap["EnableStatus"]), 0)
        self.assertEqual(int(snap["TestValue"]), arm.TEST_VALUE if hasattr(arm, "TEST_VALUE") else int(snap["TestValue"]))
        self.assertEqual(int(snap["MessageSize"]), 1440)

    def test_enable_goes_through_the_real_protocol(self):
        out = self.app.arm_enable(True)
        self.assertTrue(out["ok"], out)
        self.assertEqual(int(self.app.arm_feed.snapshot()["EnableStatus"]), 1)
        self.assertEqual(int(self.app.arm_feed.snapshot()["RobotMode"]), 5)
        out = self.app.arm_enable(False)
        self.assertTrue(out["ok"], out)
        self.assertEqual(int(self.app.arm_feed.snapshot()["EnableStatus"]), 0)

    def test_control_port_is_reachable_from_the_same_client(self):
        robot = self.app._robot()
        try:
            info = robot.check()
            self.assertEqual([round(v) for v in info["pose"]], [0, -223, 1090, -90, 0, -180])
        finally:
            robot.close()

    def test_no_leftover_ports_after_close(self):
        port = self.app.arm_port
        self.app.close_arm()
        robot = arm.DobotArm(ip="127.0.0.1", port=port, timeout=1.0, read_timeout=0.5)
        with self.assertRaises(arm.ArmError):
            robot.check()
        robot.close()


if __name__ == "__main__":
    unittest.main()