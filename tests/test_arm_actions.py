"""组动作（``handlink/arm_actions.py``）的离线测试：不接真机、不接相机。"""
from __future__ import annotations

import json
import threading
import unittest
import urllib.error
import urllib.request
from unittest import mock

from handlink import arm as arm_mod
from handlink import arm_actions, bus, kin, service

READY = [0.0, -25.0, -40.0, 0.0, 65.0, 0.0]


class TestWave(unittest.TestCase):
    def test_structure(self):
        moves = arm_actions.build("wave", READY, cycles=3)
        self.assertEqual(len(moves), 1 + 2 * 3 + 1)
        self.assertEqual(moves[0].label, "抬起（手举到胸前上方）")
        self.assertEqual(moves[-1].label, "放回原位")
        # 第一帧抬起、最后一帧回到起点
        self.assertNotEqual(moves[0].joints, READY)
        self.assertEqual(moves[-1].joints, READY)

    def test_swing_is_symmetric(self):
        moves = arm_actions.build("wave", READY, cycles=2, swing_deg=12, wrist_deg=16)
        lefts = [m for m in moves if m.label.endswith("左")]
        rights = [m for m in moves if m.label.endswith("右")]
        self.assertEqual(len(lefts), 2)
        self.assertEqual(len(rights), 2)
        for left, right in zip(lefts, rights):
            self.assertAlmostEqual(left.joints[0] - READY[0], 12.0, places=6)
            self.assertAlmostEqual(right.joints[0] - READY[0], -12.0, places=6)
            # 手腕反相摆，幅度一致
            self.assertAlmostEqual((left.joints[4] - READY[4])
                                   + (right.joints[4] - READY[4]), -32.0, places=6)

    def test_lift_raises_hand(self):
        """抬起那一步的末端应该比起点高 —— 挥手要看得见。"""
        moves = arm_actions.build("wave", READY)
        before = kin.fk(READY)[2]
        after = kin.fk(moves[0].joints)[2]
        self.assertGreater(after, before + 50.0)

    def test_params_are_clamped(self):
        moves = arm_actions.build("wave", READY, cycles=99, swing_deg=999,
                                  wrist_deg=-5, lift_ms=1, beat_ms=99999)
        self.assertEqual(len(moves), 1 + 2 * 6 + 1)          # cycles 夹到 6
        self.assertAlmostEqual(moves[1].joints[0] - READY[0], 25.0, places=6)  # 摆幅夹到 25
        self.assertLessEqual(moves[1].hold_ms, 1500)         # 停顿夹到 1500 ms

    def test_unknown_action_and_param(self):
        with self.assertRaises(ValueError):
            arm_actions.build("飞起来", READY)
        with self.assertRaises(ValueError) as ctx:
            arm_actions.build("wave", READY, cycles=2, spins=3)
        self.assertIn("参数不对", str(ctx.exception))
        with self.assertRaises(ValueError):
            arm_actions.build("wave", [0, 0, 0])

    def test_never_touches_cable_joints(self):
        """组动作不该给 J4/J6 增量，否则手腕走线会越拧越多。"""
        moves = arm_actions.build("wave", READY, cycles=3)
        for move in moves:
            self.assertAlmostEqual(move.joints[3], READY[3], places=6)
            self.assertAlmostEqual(move.joints[5], READY[5], places=6)


class TestValidate(unittest.TestCase):
    def test_clean_plan_has_no_errors(self):
        issues = arm_actions.validate(arm_actions.build("wave", READY))
        self.assertEqual([i for i in issues if i["level"] == "error"], [])

    def test_detects_joint_limit(self):
        bad = arm_actions.Move("越界", [0, 0, 200, 0, 0, 0], 0)
        issues = arm_actions.validate([bad])
        self.assertTrue(any("越界" in i["message"] for i in issues))

    def test_detects_big_step(self):
        a = arm_actions.Move("一", [0, 0, 0, 0, 0, 0], 0)
        b = arm_actions.Move("二", [0, 0, 120, 0, 0, 0], 0)
        issues = arm_actions.validate([a, b])
        self.assertTrue(any("跨度" in i["message"] for i in issues))

    def test_detects_long_hold(self):
        moves = [arm_actions.Move("停", READY, 4000), arm_actions.Move("停", READY, 4000)]
        issues = arm_actions.validate(moves)
        self.assertTrue(any("停顿" in i["message"] for i in issues))

    def test_warns_when_clamped_by_start_near_limit(self):
        near_j3 = [0.0, -10.0, 155.0, 0.0, 10.0, 0.0]
        issues = arm_actions.validate(arm_actions.build("wave", near_j3))
        self.assertTrue(any(i["level"] == "warning" for i in issues))


class TestEstimateAndPlay(unittest.TestCase):
    def test_estimate_scales_with_cycles(self):
        one = arm_actions.estimate_seconds(arm_actions.build("wave", READY, cycles=1), READY)
        three = arm_actions.estimate_seconds(arm_actions.build("wave", READY, cycles=3), READY)
        self.assertGreater(three, one)
        self.assertGreater(one, 0.0)

    def test_describe_rows(self):
        moves = arm_actions.build("wave", READY, cycles=1)
        rows = arm_actions.describe(moves, READY)
        self.assertEqual(len(rows), len(moves))
        self.assertEqual(rows[0]["step"], 1)
        self.assertEqual(rows[-1]["joints"], [round(v, 1) for v in READY])

    def test_play_on_sim_arm(self):
        robot = arm_mod.SimArm()
        robot.enable()
        seen = []
        result = arm_actions.play(robot, "wave", start=READY, speed=20,
                                  on_step=seen.append)
        self.assertTrue(result["ok"], result)
        self.assertEqual(len(seen), result["total"])
        self.assertEqual(result["steps"], seen)
        self.assertEqual(robot.joints(), READY)          # 做完回原位
        # 仿真臂的记录里每一步都是 MovJ(joint)
        moves = [t for t in robot.trace if t["event"] == "MovJ(joint)"]
        self.assertEqual(len(moves), result["total"])

    def test_play_rejects_bad_action(self):
        robot = arm_mod.SimArm()
        robot.enable()
        with self.assertRaises(ValueError):
            arm_actions.play(robot, "飞起来", start=READY)


class FakeFeed:
    """一帧 30004 的子集，够 describe() 用。"""

    def __init__(self, joints=None, enabled=True, running=0, error=0, mode=5):
        self.joints = list(joints or READY)
        self.enabled = enabled
        self.running = running
        self.error = error
        self.mode = mode

    def age(self):
        return 0.01

    def snapshot(self):
        return {
            "RobotMode": self.mode,
            "EnableStatus": 1 if self.enabled else 0,
            "ErrorStatus": self.error,
            "RunningStatus": self.running,
            "BrakeStatus": 0,
            "CollisionState": 0,
            "ToolVectorActual": kin.fk(self.joints),
            "QActual": list(self.joints),
            "IActual": [0.0] * 6,
            "SpeedScaling": 20.0,
            "VRobot": 48.0,
            "RunTime": 1000,
            "SafetyState": 0x3F,
        }

    def close(self):
        pass


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

    def speed_factor(self, pct):
        self.commands.append("SpeedFactor(%.0f)" % pct)
        return float(pct)

    def move_joints(self, joints, wait=True, timeout=20.0):
        self.commands.append("MovJ")
        if self.feed is not None:
            self.feed.joints = [float(v) for v in joints]
        return {"ok": True, "joints": list(joints)}

    def stop(self):
        self.commands.append("Stop")

    def pose(self):
        return kin.fk(self.feed.joints) if self.feed else [0.0] * 6


class TestService(unittest.TestCase):
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
        self.sleep = mock.patch.object(arm_actions, "SETTLE_S", 0.0)
        self.sleep.start()

    def tearDown(self):
        self.sleep.stop()
        self.patcher.stop()
        self.hand.close()

    def test_preview_builds_plan_without_moving(self):
        result = self.app.arm_action("wave", apply=False, start=READY, cycles=2)
        self.assertTrue(result["ok"], result)
        self.assertFalse(result["moved"])
        self.assertEqual(result["reason"], "preview（没下发）")
        self.assertEqual(len(result["steps"]), 1 + 2 * 2 + 1)
        self.assertEqual(result["start"], [round(v, 3) for v in READY])
        self.assertEqual(result["steps"][-1]["joints"], [round(v, 3) for v in READY])
        self.assertEqual(FakeRobot.instances, [])
        # 预览和 /arm/joints 一样，把视图挪到最后一帧
        self.assertEqual([round(v, 3) for v in self.app.arm_sim_joints],
                         [round(v, 3) for v in READY])

    def test_offline_refuses_to_move(self):
        self.app.arm_feed = None
        result = self.app.arm_action("wave", apply=True, start=READY, cycles=1)
        self.assertFalse(result["ok"])
        self.assertIn("不在线", result["reason"])
        self.assertEqual(FakeRobot.instances, [])

    def test_disabled_arm_refuses(self):
        self.feed.enabled = False
        result = self.app.arm_action("wave", apply=True, start=READY, cycles=1)
        self.assertFalse(result["ok"])
        self.assertIn("使能", result["reason"])
        self.assertEqual(FakeRobot.instances, [])

    def test_error_arm_refuses(self):
        self.feed.error = 7
        result = self.app.arm_action("wave", apply=True, start=READY, cycles=1)
        self.assertFalse(result["ok"])
        self.assertIn("报警", result["reason"])

    def test_runs_all_steps_and_returns_home(self):
        result = self.app.arm_action("wave", apply=True, start=READY, cycles=2,
                                     speed=15, force=True, lift_ms=100, beat_ms=100)
        self.assertTrue(result["ok"], result)
        self.assertTrue(result["moved"])
        self.assertTrue(result["complete"])
        self.assertEqual(len(result["steps_done"]), len(result["steps"]))
        self.assertEqual(result["speed_factor"], 15.0)
        self.assertIn("min_clearance_mm", result)
        robot = FakeRobot.instances[0]
        self.assertEqual(robot.commands[0], "connect")
        self.assertEqual(robot.commands[1], "SpeedFactor(15)")
        self.assertEqual(robot.commands.count("MovJ"), len(result["steps"]))
        self.assertEqual(robot.commands[-1], "close")
        # 做完回到起点
        self.assertEqual([round(v, 3) for v in self.feed.joints],
                         [round(v, 3) for v in READY])

    def test_gate_failure_stops_with_stop(self):
        """第二段被闸门拦下：不发后面的 MovJ，并且 Stop() 停住。"""
        calls = {"n": 0}
        real_gate = self.app._arm_gate

        def flaky(current, target, force=False):
            calls["n"] += 1
            if calls["n"] == 2:
                return None, {"error": "测试：第二段被拦"}
            return real_gate(current, target, force=force)

        with mock.patch.object(self.app, "_arm_gate", flaky):
            result = self.app.arm_action("wave", apply=True, start=READY, cycles=2,
                                         force=True, lift_ms=100, beat_ms=100)
        self.assertFalse(result["ok"])
        self.assertIn("第二段", result["error"])
        self.assertEqual(len(result["steps_done"]), 1)
        self.assertFalse(result["complete"])
        robot = FakeRobot.instances[0]
        self.assertEqual(robot.commands.count("MovJ"), 1)
        self.assertIn("Stop", robot.commands)

    def test_force_bypasses_red_line_only(self):
        """force=True 只跳过碰撞红线；未使能/报警这类前置条件照样拦。"""
        unsafe = {"ok": True, "safe": False, "min_m": 0.005,
                  "pair": ["Link3", "桌面"], "samples": 20}
        with mock.patch.object(service.cell_mod, "check_path", return_value=unsafe):
            refused = self.app.arm_move([0, -30, -45, 0, 70, 0], apply=True)
            forced = self.app.arm_move([0, -30, -45, 0, 70, 0], apply=True, force=True)
        self.assertFalse(refused["ok"])
        self.assertIn("红线", refused["reason"])
        self.assertTrue(forced["ok"], forced)
        self.assertTrue(forced["moved"])
        self.assertEqual(forced["clearance"]["min_m"], 0.005)
        self.assertEqual(len(FakeRobot.instances), 1)

    def test_bad_name(self):
        result = self.app.arm_action("飞起来", apply=False, start=READY)
        self.assertFalse(result["ok"])
        self.assertIn("未知组动作", result["error"])


class TestRoutes(unittest.TestCase):
    def setUp(self):
        self.hand = bus.open_sim()
        self.app = service.App(self.hand)
        self.httpd = service.make_server(self.app, "127.0.0.1", 0)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.hand.close()

    def _post(self, path, body):
        req = urllib.request.Request(
            "http://127.0.0.1:%d%s" % (self.port, path),
            data=json.dumps(body).encode(), method="POST",
            headers={"Content-Type": "application/json"})
        return json.loads(urllib.request.urlopen(req, timeout=10).read().decode("utf-8"))

    def test_action_list(self):
        data = json.loads(urllib.request.urlopen(
            "http://127.0.0.1:%d/arm/actions" % self.port, timeout=10).read().decode("utf-8"))
        self.assertTrue(data["ok"])
        names = [a["name"] for a in data["actions"]]
        self.assertIn("wave", names)
        wave = next(a for a in data["actions"] if a["name"] == "wave")
        self.assertEqual(wave["label"], "挥手")
        self.assertEqual(wave["defaults"]["cycles"], 3)

    def test_action_preview_route(self):
        data = self._post("/arm/action", {"name": "wave", "apply": False,
                                          "start": READY, "cycles": 2})
        self.assertTrue(data["ok"], data)
        self.assertFalse(data["moved"])
        self.assertEqual(len(data["steps"]), 1 + 2 * 2 + 1)
        self.assertEqual(data["label"], "挥手")

    def test_action_route_rejects_missing_name(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self._post("/arm/action", {"apply": False})
        self.assertEqual(ctx.exception.code, 400)

    def test_action_route_rejects_bad_start(self):
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            self._post("/arm/action", {"name": "wave", "start": [0, 0]})
        self.assertEqual(ctx.exception.code, 400)


if __name__ == "__main__":
    unittest.main()
