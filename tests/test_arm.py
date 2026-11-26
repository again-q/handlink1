"""臂驱动与抓取流程的离线测试：假 Dobot 服务器 + 仿真臂，不需要真机。"""
from __future__ import annotations

import socket
import threading
import time
import unittest

from handlink import arm, bus, pick, poses, spec


class FakeDobot(threading.Thread):
    """极简 dashboard 服务：一行命令一行回复，格式与 Dobot 29999 一致。"""

    def __init__(self) -> None:
        super().__init__(daemon=True)
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(1)
        self.port = self._srv.getsockname()[1]
        self.commands = []
        self.pose = [100.0, 0.0, 200.0, 180.0, 0.0, 0.0]
        self.mode = 5
        self.text_reply = None
        # 算法队列：每条 MovJ/MovL 分一个 ResultID，GetCurrentCommandID 回报「执行到哪」
        self.next_id = 1
        self.current_id = 0
        self.lag = 0                      # >0 表示还积压着这么多条没执行
        self._stop = threading.Event()

    def run(self) -> None:
        conn, _ = self._srv.accept()
        f = conn.makefile("rwb")
        try:
            while not self._stop.is_set():
                line = f.readline()
                if not line:
                    break
                cmd = line.decode("ascii", "ignore").strip()
                self.commands.append(cmd)
                f.write((self.reply(cmd) + "\n").encode())
                f.flush()
        finally:
            try:
                conn.close()
            finally:
                self._srv.close()

    def reply(self, cmd: str) -> str:
        if self.text_reply is not None:
            return self.text_reply
        if cmd.startswith("GetPose"):
            body = ",".join(str(int(v)) if float(v).is_integer() else str(v) for v in self.pose)
            return f"0,{{{body}}},GetPose();"
        if cmd.startswith("GetAngle"):
            return "0,{0,0,0,0,0,0},GetAngle();"
        if cmd.startswith("RobotMode"):
            return f"0,{{{self.mode}}},RobotMode();"
        if cmd.startswith("GetCurrentCommandID"):
            return f"0,{{{max(0, self.current_id - self.lag)}}},GetCurrentCommandID();"
        if cmd.startswith(("MovJ", "MovL", "StartPath", "Arc", "Circle")):
            rid = self.next_id
            self.next_id += 1
            self.current_id = rid           # 假装立刻执行完
            return f"0,{{{rid}}},{cmd};"
        if cmd.startswith("Unknown"):
            return "-1,{},Unknown();"
        return f"0,{{}},{cmd};"

    def stop(self) -> None:
        self._stop.set()
        try:
            self._srv.close()
        except OSError:
            pass


class FakeHand(bus.HandBus):
    """记录下发姿态，并把状态直接当作已到位（测流程逻辑，不测伺服）。"""

    name = "fake"

    def __init__(self) -> None:
        self.applied = []
        self._state = {m.motor_id: 0.0 for m in spec.MOTORS}

    def apply(self, pose, reason="") -> dict:
        self.applied.append((reason, dict(pose)))
        for mid, joint in pose.items():
            self._state[mid] = float(joint.angle)
        return {"ok": True, "applied": sorted(pose)}

    def state(self):
        return {
            mid: bus.MotorState(motor_id=mid, angle=a, speed=0.0, current=0.5,
                                target=a, error=None)
            for mid, a in self._state.items()
        }


class TestProtocol(unittest.TestCase):
    def test_fmt_and_parse(self):
        self.assertEqual(arm.fmt_pose([400, 0, 300, 180, 0, 0]), "{400,0,300,180,0,0}")
        self.assertEqual(arm.parse_pose("0,{1,2,3,4,5,6},GetPose();"), [1, 2, 3, 4, 5, 6])
        with self.assertRaises(ValueError):
            arm.fmt_pose([1, 2, 3])
        with self.assertRaises(arm.ArmError):
            arm.parse_pose("0,{},GetPose();")

    def test_fmt_joints(self):
        """关节空间：V4.6 的 MovJ 第一个参数可以是 joint={...}，不需要逆解。"""
        self.assertEqual(arm.fmt_joints([1, -2.5, 3, 4, 5, 6]), "{1,-2.5,3,4,5,6}")
        with self.assertRaises(ValueError):
            arm.fmt_joints([1, 2, 3, 4, 5])


class TestSimArm(unittest.TestCase):
    def test_needs_enable(self):
        a = arm.SimArm()
        with self.assertRaises(arm.ArmError):
            a.move_j([0, 0, 0, 0, 0, 0])
        a.enable()
        a.speed_factor(20)
        self.assertAlmostEqual(a.speed_factor(20), 20.0)
        a.move_l([400, 0, 300, 180, 0, 0])
        self.assertEqual(a.pose(), [400.0, 0.0, 300.0, 180.0, 0.0, 0.0])

    def test_eta_scales_with_speed(self):
        a = arm.SimArm(pose=(0, 0, 100, 180, 0, 0))
        a.enable()
        a.speed_factor(100)
        slow = a.estimate_seconds((0, 0, 0, 180, 0, 0))
        a.speed_factor(50)
        fast = a.estimate_seconds((0, 0, 0, 180, 0, 0))
        self.assertAlmostEqual(slow, 1.0)
        self.assertAlmostEqual(fast, 2.0)

    def test_move_joints(self):
        a = arm.SimArm()
        with self.assertRaises(arm.ArmError):
            a.move_joints([1, 2, 3, 4, 5, 6])
        a.enable()
        result = a.move_joints([328.953, -1.759, -119.878, -63.137, -68.699, -49.355])
        self.assertEqual(result["cmd"], "MovJ(joint)")
        self.assertEqual(a.joints(), [328.953, -1.759, -119.878, -63.137, -68.699, -49.355])
        self.assertEqual(a.trace[-1]["event"], "MovJ(joint)")


class TestDobotArm(unittest.TestCase):
    def setUp(self):
        self.server = FakeDobot()
        self.server.start()
        self.arm = arm.DobotArm(ip="127.0.0.1", port=self.server.port, read_timeout=1.0)
        for _ in range(50):
            if self.server.is_alive():
                break
            time.sleep(0.01)

    def tearDown(self):
        self.arm.close()
        self.server.stop()
        time.sleep(0.02)

    def test_check_reads_only(self):
        info = self.arm.check()
        self.assertEqual(info["pose"], [100, 0, 200, 180, 0, 0])
        self.assertTrue(info["enabled"])
        self.assertTrue(all(c.startswith(("RobotMode", "GetPose", "GetAngle"))
                            for c in self.server.commands))

    def test_move_formats_pose(self):
        result = self.arm.move_j([400, 0, 300, 180, 0, 0], wait=False)
        self.assertTrue(result["ok"])
        self.assertIn("MovJ(pose={400,0,300,180,0,0})", self.server.commands)

    def test_move_l_and_speed(self):
        self.arm.speed_factor(15)
        self.arm.move_l([1, 2, 3, 4, 5, 6], wait=False)
        self.assertIn("SpeedFactor(15)", self.server.commands)
        # 单发一条 MovL 默认 cp=0：这条指令自己走完要停稳，不会被下一条拖着穿过终点。
        self.assertIn("MovL(pose={1,2,3,4,5,6},cp=0)", self.server.commands)

    def test_move_joints_formats_joint(self):
        """真机就是这么动的：MovJ(joint={...})，不是 pose。"""
        self.arm.move_joints([328.953, -1.759, -119.878, -63.137, -68.699, -49.355],
                             wait=False)
        self.assertIn("MovJ(joint={328.953,-1.759,-119.878,-63.137,-68.699,-49.355})",
                      self.server.commands)

    def test_error_code_raises(self):
        with self.assertRaises(arm.ArmError):
            self.arm._command("Unknown()")

    def test_tcp_mode_message_is_explained(self):
        self.server.text_reply = "Control Mode Is Not Tcp"
        with self.assertRaises(arm.ArmError) as ctx:
            self.arm.pose()
        self.assertIn("TCP", str(ctx.exception))

    def test_drag_commands_are_plain_dashboard_text(self):
        """拖动示教就是 StartDrag()/StopDrag() 两条原样指令，没有自定义协议。"""
        self.arm.start_drag()
        self.arm.stop_drag()
        self.assertIn("StartDrag()", self.server.commands)
        self.assertIn("StopDrag()", self.server.commands)


class TestCpDispatch(unittest.TestCase):
    """带 cp 的逐点下发 + 队列 ID 节流（2026-11-25 真机就栽在这一组上）。"""

    def setUp(self):
        self.server = FakeDobot()
        self.server.start()
        self.arm = arm.DobotArm(ip="127.0.0.1", port=self.server.port, read_timeout=1.0)
        for _ in range(50):
            if self.server.is_alive():
                break
            time.sleep(0.01)

    def tearDown(self):
        self.arm.close()
        self.server.stop()
        time.sleep(0.02)

    def test_movl_cp_is_on_the_wire(self):
        rid = self.arm.movl_cp([1, 2, 3, 4, 5, 6], 30)
        self.assertGreater(rid, 0)
        self.assertIn("MovL(pose={1,2,3,4,5,6},cp=30)", self.server.commands)

    def test_current_cmd_id_parses_the_result_id(self):
        rid = self.arm.movl_cp([1, 2, 3, 4, 5, 6], 30)
        self.assertEqual(self.arm.current_cmd_id(), rid)
        self.assertTrue(self.arm.wait_cmd_id(rid, timeout=2.0))

    def test_wait_cmd_id_gives_up_when_queue_is_stuck(self):
        rid = self.arm.movl_cp([1, 2, 3, 4, 5, 6], 30)
        self.server.lag = 100              # 一直没执行到
        self.assertFalse(self.arm.wait_cmd_id(rid, timeout=0.2))

    def test_movl_r_is_on_the_wire(self):
        """r 是绝对毫米，慢速下比 cp 百分比更靠得住；a 是加速度比例。

        控制器只认整数 r：``r=3`` 成功，``r=3.0`` 回 -50001（2026-11-25 实测）。
        """
        rid = self.arm.movl_r([1, 2, 3, 4, 5, 6], 3.0, accel=50)
        self.assertGreater(rid, 0)
        self.assertIn("MovL(pose={1,2,3,4,5,6},r=3,a=50)", self.server.commands)
        self.arm.movl_r([1, 2, 3, 4, 5, 6], 2.5)
        self.assertIn("MovL(pose={1,2,3,4,5,6},r=3)", self.server.commands)   # 2.5 取整

    def test_every_cmd_constant_really_exists(self):
        """真机事故：``current_cmd_id()`` 用了没定义的 ``CMD_ARRIVE_INDEX``，
        轨迹下发到第 40 点才抛 AttributeError，臂当场被 Stop() 掐停。"""
        import inspect
        import re
        src = inspect.getsource(arm.DobotArm)
        used = sorted(set(re.findall(r"self\.(CMD_[A-Z_]+)", src)))
        self.assertIn("CMD_ARRIVE_INDEX", used)
        for name in used:
            self.assertTrue(hasattr(arm.DobotArm, name), "少了常量 " + name)


class TestWaitUntilAngleWrap(unittest.TestCase):
    """控制器在 −180 / +180 之间来回给值（取决于 IK 选了哪一支），同一个朝向不能算没到位。

    现场症状：同一段 63 mm 的垂直下降每次都卡满 40 s 并返回 ``arrived=False``，
    而实测位姿早就到目标了 —— 就是 180 与 −180 直接相减得 360 造成的。
    """

    def test_arrives_when_controller_flips_the_sign(self):
        sim = arm.SimArm()
        sim.enable()
        sim.move_j([100, 0, 200, -180, 0, 0])
        self.assertEqual(sim.pose()[3], -180)
        self.assertTrue(sim.wait_until([100, 0, 200, 180, 0, 0], timeout=0.5))

    def test_not_fooled_by_a_real_angle_error(self):
        sim = arm.SimArm()
        sim.enable()
        sim.move_j([100, 0, 200, -180, 0, 0])
        self.assertFalse(sim.wait_until([100, 0, 200, -170, 0, 0], timeout=0.3))

    def test_ang_diff_wraps_across_zero(self):
        self.assertAlmostEqual(arm.DobotArm._ang_diff(-179, 179), 2.0)
        self.assertAlmostEqual(arm.DobotArm._ang_diff(180, -180), 0.0)
        self.assertAlmostEqual(arm.DobotArm._ang_diff(10, 4), 6.0)


class TestPick(unittest.TestCase):
    def setUp(self):
        self._grace = pick.CLENCH_GRACE
        pick.CLENCH_GRACE = 0.1

    def tearDown(self):
        pick.CLENCH_GRACE = self._grace

    def test_full_sequence(self):
        a = arm.SimArm()
        h = FakeHand()
        logs = []
        result = pick.run(a, h, pick.PickPlan(), log=logs.append)
        self.assertTrue(result["ok"])
        self.assertEqual([s["step"] for s in result["steps"]],
                         ["手张开", "臂回安全位", "移动到预抓取点", "下探到抓取点",
                          "手闭合", "抬升", "手张开（放下）", "臂回安全位"])
        events = [e["event"] for e in a.trace]
        self.assertEqual(events.count("MovL"), 3)   # 预抓取、下探、抬升
        self.assertEqual(events.count("MovJ"), 2)   # 前回位、后回位
        reasons = [r for r, _ in h.applied]
        # 合手分两段（2026-11-19 实测：五指同合会互顶，先让拇指 1 到下位）
        self.assertIn("close:thumb_low", reasons)
        self.assertIn("close:fist", reasons)

    def test_aborts_when_hand_stays_open(self):
        class StuckHand(FakeHand):
            def apply(self, pose, reason=""):
                self.applied.append((reason, dict(pose)))
                return {"ok": True}

            def state(self):
                return {mid: bus.MotorState(motor_id=mid, angle=0.0, speed=0.0,
                                            current=0.5, target=0.0, error=None)
                        for mid in spec.MOTOR_IDS}

        a = arm.SimArm()
        with self.assertRaises(pick.PickError) as ctx:
            pick.run(a, StuckHand(), pick.PickPlan(), log=lambda *_: None)
        self.assertIn("没有合到", str(ctx.exception))
        events = [e["event"] for e in a.trace]
        self.assertEqual(events.count("MovL"), 2)  # 预抓取 + 下探，抬升没发生

    def test_preview_estimates(self):
        info = pick.preview(pick.PickPlan())
        self.assertEqual(len(info["segments"]), 4)
        self.assertGreater(info["eta_s"], 0)

    def test_clenched_uses_ratio(self):
        h = FakeHand()
        h.apply(poses.pose_of("fist"))
        self.assertTrue(pick.clenched(h, poses.POSES["fist"]))
        h._state = {m: 0.0 for m in spec.MOTOR_IDS}
        self.assertFalse(pick.clenched(h, poses.POSES["fist"]))


if __name__ == "__main__":
    unittest.main()
