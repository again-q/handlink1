"""网页「使能 / 下使能」的离线测试：假 30004 帧 + 假 dashboard，不需要真机。"""
from __future__ import annotations

import json
import threading
import time
import unittest
import urllib.error
import urllib.request
from unittest import mock

from handlink import arm as arm_mod
from handlink import bus, service


def frame(mode=4, enabled=False, error=0, running=0, speed=20.0, safety=0x3F):
    """一帧 30004 的子集，字段够 describe() 用就行。"""
    return {
        "RobotMode": mode,
        "EnableStatus": 1 if enabled else 0,
        "ErrorStatus": error,
        "RunningStatus": running,
        "BrakeStatus": 0,
        "CollisionState": 0,
        "ToolVectorActual": [400.0, 0.0, 300.0, 180.0, 0.0, 0.0],
        "QActual": [0.0] * 6,
        "IActual": [0.0] * 6,
        "SpeedScaling": speed,
        "VRobot": 48.0,
        "RunTime": 1000,
        "SafetyState": safety,
    }


class FakeFeed:
    """只实现 App 用到的方法。"""

    def __init__(self, mode=4, enabled=False, error=0, running=0, safety=0x3F):
        self.mode = mode
        self.enabled = enabled
        self.error = error
        self.running = running
        self.speed = 20.0
        self.safety = safety
        self.closed = False

    def age(self):
        return 0.01

    def snapshot(self):
        return frame(self.mode, self.enabled, self.error,
                     self.running, self.speed, self.safety)

    def close(self):
        self.closed = True


class FakeRobot:
    instances = []

    def __init__(self, ip="192.168.5.1", feed=None):
        self.ip = ip
        self.feed = feed
        self.commands = []
        FakeRobot.instances.append(self)

    def connect(self):
        self.commands.append("connect")

    def close(self):
        self.commands.append("close")

    def take_control(self):
        self.commands.append("RequestControl")

    def pose(self):
        self.commands.append("GetPose")
        return [400.0, 0.0, 300.0, 180.0, 0.0, 0.0]

    def speed_factor(self, pct):
        self.commands.append("SpeedFactor(%.0f)" % pct)
        self.feed.speed = float(pct)
        return float(pct)

    def enable(self):
        self.commands.append("EnableRobot")
        self.feed.enabled = True
        self.feed.mode = 5

    def disable(self):
        self.commands.append("DisableRobot")
        self.feed.enabled = False
        self.feed.mode = 4


class TestArmEnable(unittest.TestCase):
    def setUp(self):
        FakeRobot.instances = []
        self.hand = bus.open_sim()
        self.app = service.App(self.hand)
        self.feed = FakeFeed()
        self.app.arm_feed = self.feed
        self.app.arm_ip = "127.0.0.1"
        outer = self

        def factory(ip="192.168.5.1", port=None, **kwargs):
            return FakeRobot(ip=ip, feed=outer.feed)

        self.patcher = mock.patch.object(service.arm_mod, "DobotArm", factory)
        self.patcher.start()

    def tearDown(self):
        self.patcher.stop()
        self.hand.close()

    def _robot(self):
        self.assertEqual(len(FakeRobot.instances), 1)
        return FakeRobot.instances[0]

    def test_offline_sends_nothing(self):
        self.app.arm_feed = None
        result = self.app.arm_enable(True)
        self.assertFalse(result["ok"])
        self.assertFalse(result["online"])
        self.assertEqual(FakeRobot.instances, [])

    def test_enable_from_disabled_takes_control_then_enables(self):
        result = self.app.arm_enable(True, speed=15)
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["enabled"])
        robot = self._robot()
        self.assertEqual(robot.commands,
                         ["connect", "RequestControl", "GetPose",
                          "SpeedFactor(15)", "EnableRobot", "close"])
        self.assertEqual(result["speed_scaling"], 15.0)

    def test_enable_clamps_speed(self):
        result = self.app.arm_enable(True, speed=999)
        self.assertTrue(result["ok"], result)
        self.assertIn("SpeedFactor(100)", self._robot().commands)

    def test_enable_already_enabled_is_noop(self):
        self.feed.enabled = True
        self.feed.mode = 5
        result = self.app.arm_enable(True)
        self.assertTrue(result["ok"])
        self.assertTrue(result["already"])
        self.assertEqual(FakeRobot.instances, [])

    def test_enable_refuses_error(self):
        self.feed.error = 7
        result = self.app.arm_enable(True)
        self.assertFalse(result["ok"])
        self.assertIn("报警", result["error"])
        self.assertEqual(FakeRobot.instances, [])

    def test_enable_refuses_running(self):
        self.feed.running = 1
        result = self.app.arm_enable(True)
        self.assertFalse(result["ok"])
        self.assertIn("运动", result["error"])

    def test_enable_refuses_estop(self):
        self.feed.safety = 0
        result = self.app.arm_enable(True)
        self.assertFalse(result["ok"])
        self.assertIn("急停", result["error"])

    def test_disable(self):
        self.feed.enabled = True
        self.feed.mode = 5
        result = self.app.arm_enable(False)
        self.assertTrue(result["ok"], result)
        self.assertFalse(result["enabled"])
        self.assertNotIn("RequestControl", self._robot().commands)
        self.assertIn("DisableRobot", self._robot().commands)

    def test_disable_already_disabled_is_noop(self):
        result = self.app.arm_enable(False)
        self.assertTrue(result["ok"])
        self.assertTrue(result["already"])
        self.assertEqual(FakeRobot.instances, [])


class TestArmEnableHTTP(unittest.TestCase):
    """路由本身：离线时明确回 503，而不是假装成功。"""

    def setUp(self):
        self.hand = bus.open_sim()
        self.app = service.App(self.hand)
        self.httpd = service.make_server(self.app, "127.0.0.1", 0)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.hand.close()

    def test_offline_returns_503(self):
        req = urllib.request.Request(
            "http://127.0.0.1:%d/arm/enable" % self.port,
            data=json.dumps({"on": True}).encode(), method="POST",
            headers={"Content-Type": "application/json"},
        )
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(ctx.exception.code, 503)
        body = json.loads(ctx.exception.read().decode("utf-8"))
        self.assertFalse(body["ok"])
        self.assertFalse(body["online"])

    def test_bad_speed_returns_400(self):
        req = urllib.request.Request(
            "http://127.0.0.1:%d/arm/enable" % self.port,
            data=json.dumps({"on": True, "speed": "fast"}).encode(), method="POST",
            headers={"Content-Type": "application/json"},
        )
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(req, timeout=5)
        self.assertEqual(ctx.exception.code, 400)


if __name__ == "__main__":
    unittest.main()
