"""虚拟 Nova 5 控制器的测试：29999 文本口 + 30004 反馈口，全程不需要真机。

和 test_arm.py 里的 FakeDobot 不同，这里的 VirtualDobot 是**行为级**的：
算法队列会真的排队、ServoJ 有 30 ms 控制周期和 t 低通、模式机会从 4 走到 5 再走到 7、
29999 只认一个客户端、30004 每 8 ms 推一帧 1440 字节。

数字对齐 docs/遥操作跟随-调研与方案.md 第 1 节的真机实测（GetPose 中位 3.9 ms、
ServoJ ACK 中位 6.2 ms、停发 10 s 不跑位、SpeedFactor 对 ServoJ 无效）。

    python -m unittest tests.test_armsim -v
"""
from __future__ import annotations

import statistics
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import arm_feed, armsim  # noqa: E402
from handlink import arm as arm_mod  # noqa: E402


#: 仿真起点的位姿。故意用「关节角都是 0」的那一点：它自己的逆解是唯一的，
#: 而 (400,0,300,180,0,0) 这种笛卡尔点在这个工作空间里根本不可达（本机 b 的
#: 数值反解会落进奇异支管，误差几百毫米），拿它当目标会测出一堆假失败。
P0 = [0.0, -223.328, 1090.0, -90.0, 0.0, -180.0]
#: 关节空间的一步（J1 转 20°），给 move_joints 用
J1_JOINTS = [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#: 同一个动作的笛卡尔形式（J1 转 20° 后的位姿），给 movl_cp / pose 用
P1 = [20.0] + P0[1:]


def wait_for(predicate, timeout: float = 2.0, interval: float = 0.01) -> bool:
    """轮询到 predicate 为真；超时返回 False（不用 sleep 大段）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return False


class _SimCase(unittest.TestCase):
    """每个用例一台新的虚拟控制器；端口都是系统分配的。"""

    seed = 0
    sim_kwargs: dict = {}

    def setUp(self) -> None:
        kwargs = dict(self.sim_kwargs)
        kwargs.setdefault("seed", self.seed)
        self.sim = armsim.VirtualDobot(**kwargs).start()
        self.addCleanup(self.sim.stop)

    def open_arm(self):
        a = self.sim.arm()
        self.addCleanup(a.close)
        return a


class TestControlPort(_SimCase):
    """29999：DobotArm 原样连上，使能 + 算法队列。"""

    def test_check_and_enable_walk_the_mode_machine(self):
        """上电初态 4（下使能）→ EnableRobot → 5（使能空闲）。"""
        a = self.open_arm()
        info = a.check()
        self.assertEqual(info["mode_raw"], "0,{4},RobotMode();")
        self.assertFalse(info["enabled"])
        self.assertEqual(self.sim.state()["mode"], armsim.MODE_DISABLED)
        self.assertEqual(len(info["joints"]), 6)

        a.enable()
        self.assertTrue(a.enabled())
        self.assertEqual(self.sim.state()["mode"], armsim.MODE_IDLE)
        self.assertTrue(self.sim.state()["enabled"])

        a.disable()
        self.assertFalse(a.enabled())
        self.assertEqual(self.sim.state()["mode"], armsim.MODE_DISABLED)

    def test_moves_return_result_ids_and_the_queue_advances(self):
        """MovJ/MovL 立刻回 ResultID，GetCurrentCommandID 一条条追上去。"""
        a = self.open_arm()
        a.enable()
        first = a.movl_cp(P0, 0)
        self.assertGreater(first, 0)
        self.assertEqual(self.sim.state()["current_id"], first,
                         "控制器一收到就开始执行，当前 ID 立刻跳过去")
        second = a.movl_cp([10.0] + P0[1:], 0)
        self.assertEqual(second, first + 1, "ResultID 是自增的")
        self.assertGreaterEqual(self.sim.state()["queue_len"], 1)

        self.assertTrue(wait_for(lambda: a.current_cmd_id() >= second, timeout=3.0),
                        "队列要走完（每条最长 1 s 仿真时间）")

    def test_move_j_and_move_l_go_through_the_public_api(self):
        """两个高层方法都能用，且真的走到位（用当前位姿当目标，闭环可验）。"""
        a = self.open_arm()
        a.enable()
        here = a.pose()
        self.assertTrue(a.move_j(here, wait=False)["ok"])
        self.assertTrue(a.move_l(here, wait=False)["ok"])
        self.assertTrue(wait_for(lambda: self.sim.state()["queue_len"] == 0, timeout=4.0))
        self.sim.settle(0.1)
        pose = a.pose()
        self.assertTrue(all(abs(x - y) < 1.0 for x, y in zip(pose, here)),
                        "到位：%r vs %r" % (pose, here))

    def test_motion_is_refused_while_disabled(self):
        """未使能不许动——真机回 -1（状态不允许）。"""
        a = self.open_arm()
        with self.assertRaises(arm_mod.ArmError) as ctx:
            a._command("MovJ(joint={10,0,0,0,0,0})")
        self.assertTrue(str(ctx.exception).startswith("-1"), str(ctx.exception))
        self.assertEqual(self.sim.state()["queue_len"], 0)

    def test_joint_limit_is_rejected(self):
        """J2 限位 ±180，超了不许排进队列。"""
        a = self.open_arm()
        a.enable()
        with self.assertRaises(arm_mod.ArmError) as ctx:
            a._command("MovJ(joint={0,200,0,0,0,0})")
        self.assertTrue(str(ctx.exception).startswith("%d" % armsim.ERR_LIMIT),
                        str(ctx.exception))

    def test_stop_halts_the_queue(self):
        a = self.open_arm()
        a.enable()
        a.movl_cp([10.0] + P0[1:], 0)
        a.stop()
        self.assertEqual(self.sim.state()["queue_len"], 0)
        self.assertEqual(self.sim.state()["mode"], armsim.MODE_IDLE)


class TestLatency(_SimCase):
    """实测延迟分布：GetPose 中位 3.9 ms、ServoJ ACK 中位 6.2 ms。"""

    def _round_trips(self, fn, n=24):
        samples = []
        for _ in range(n):
            started = time.perf_counter()
            fn()
            samples.append((time.perf_counter() - started) * 1000.0)
        return samples

    def test_getpose_matches_the_measured_median(self):
        a = self.open_arm()
        median = statistics.median(self._round_trips(a.pose))
        # 实测 3.9 ms；本机回环 + 客户端开销会让它略高一点
        self.assertGreater(median, 2.5, "延迟不能是 0，那就不像真机了")
        self.assertLess(median, 8.0, "中位远高于实测的 3.9 ms：%0.2f" % median)

    def test_servoj_ack_is_slower_than_a_read(self):
        """实测 ServoJ ACK 中位 6.2 ms > GetPose 3.9 ms。"""
        a = self.open_arm()
        a.enable()
        joints = a.joints()
        servo = statistics.median(self._round_trips(lambda: a.servo_j(joints, t=0.03)))
        read = statistics.median(self._round_trips(a.pose))
        self.assertGreater(servo, read)

    def test_latency_scale_slows_everything_down(self):
        """latency_scale 是给超时逻辑用的放大旋钮。"""
        slow = armsim.VirtualDobot(seed=3, latency_scale=8.0).start()
        self.addCleanup(slow.stop)
        a = slow.arm()
        self.addCleanup(a.close)
        median = statistics.median(self._round_trips(a.pose, n=12))
        self.assertGreater(median, 15.0, "8 倍之后应该明显变慢：%0.1f ms" % median)


class TestFeedback(_SimCase):
    """30004：Feed 能原样解析出正确字段。"""

    def _feed(self):
        feed = arm_feed.Feed("127.0.0.1", self.sim.feedback_port).open()
        self.addCleanup(feed.close)
        return feed

    def test_frame_is_1440_bytes_of_known_fields(self):
        feed = self._feed()
        frame = feed.wait_frame(timeout=3.0)
        self.assertEqual(frame["MessageSize"], arm_feed.FRAME_SIZE)
        self.assertEqual(frame["TestValue"], arm_feed.TEST_VALUE)
        self.assertEqual(frame["RobotMode"], armsim.MODE_DISABLED)
        self.assertEqual(frame["EnableStatus"], 0)
        self.assertEqual(frame["BrakeStatus"], 0)      # 实测使能/下使能都是 0
        self.assertEqual(frame["ErrorStatus"], 0)
        self.assertEqual(frame["CollisionState"], 0)
        self.assertEqual(len(frame["QActual"]), 6)
        self.assertEqual(len(frame["ToolVectorActual"]), 6)

    def test_enable_and_motion_show_up_in_the_feed(self):
        feed = self._feed()
        a = self.open_arm()
        a.enable()
        self.assertTrue(wait_for(
            lambda: int(feed.snapshot()["EnableStatus"]) == 1, timeout=2.0))

        # 速度因子 20% 时 J1 转 60° 要走满 1 s（仿真里单条最长 1 s），
        # RobotMode=7 的窗口足够宽——不然 20° 的短动作 0.2 s 就完了，
        # 8 ms 一帧的反馈很容易正好错过（这条以前偶发失败）。
        a.speed_factor(20)
        a.move_joints([60.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=False)
        self.assertTrue(wait_for(
            lambda: int(feed.snapshot()["CurrentCommandId"]) >= 1, timeout=2.0))
        self.assertTrue(wait_for(lambda: int(feed.snapshot()["RobotMode"]) == 7,
                                 timeout=2.0), "运动时 RobotMode 应该是 7")
        self.assertTrue(wait_for(
            lambda: int(feed.snapshot()["RobotMode"]) == armsim.MODE_IDLE, timeout=4.0),
            "走完应该回到 5（使能空闲）")

    def test_pose_matches_forward_kinematics(self):
        """反馈里的 ToolVectorActual 和 GetPose() 是同一套正解。"""
        from handlink import kin
        feed = self._feed()
        frame = feed.wait_frame(timeout=3.0)
        expected = kin.fk(frame["QActual"])
        for got, want in zip(frame["ToolVectorActual"], expected):
            self.assertAlmostEqual(got, want, places=3)

    def test_frames_arrive_at_the_configured_rate(self):
        feed = self._feed()
        feed.wait_frame(timeout=3.0)
        started, base = time.time(), feed.frames
        time.sleep(0.5)
        rate = (feed.frames - base) / max(time.time() - started, 1e-6)
        self.assertGreater(rate, 60.0, "帧率太低：%0.1f" % rate)
        self.assertEqual(feed.bad_frames, 0)

    def test_qactual_lags_qtarget_instead_of_jumping(self):
        """实测 QActual 相对 QTarget 有 1~2 帧滞后，不是瞬间相等。

        走一个 60° 的关节 MovJ（100 °/s，0.6 s），边动边采最大值：如果 QActual
        是瞬间赋值的，这个差值永远为 0。
        """
        feed = self._feed()
        a = self.open_arm()
        a.enable()
        a.move_joints([60.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=False)

        peak = 0.0
        deadline = time.time() + 2.0
        while time.time() < deadline and peak < 1.0:
            frame = feed.snapshot()
            peak = max(peak, max(abs(x - y)
                                 for x, y in zip(frame["QActual"], frame["QTarget"])))
            time.sleep(0.005)
        self.assertGreater(peak, 1.0, "运动中 QTarget 应该跑到前面去（峰值 %0.2f°）" % peak)

        def settled():
            frame = feed.snapshot()
            return max(abs(x - y) for x, y in
                       zip(frame["QActual"], frame["QTarget"])) < 0.05
        self.assertTrue(wait_for(settled, timeout=3.0),
                        "停下来之后 QActual 应该追平 QTarget")


class TestServoJ(_SimCase):
    """ServoJ：30 ms 节拍、t 低通、停发保持、SpeedFactor 无效。"""

    def test_hold_at_current_joints_is_zero_drift(self):
        """实测：目标=当前关节角，连续 60 帧关节漂移 0.000°。"""
        a = self.open_arm()
        a.enable()
        start = a.joints()
        for _ in range(20):
            a.servo_j(start, t=0.03)
        self.sim.settle(0.2)
        drift = max(abs(b - c) for b, c in zip(start, a.joints()))
        self.assertLess(drift, 1e-3, "原地保持不该有漂移：%r" % drift)

    def test_stopping_the_stream_holds_without_alarm(self):
        """实测：停发之后保持最后目标，RobotMode=5、ErrorStatus=0，不跑位。"""
        a = self.open_arm()
        a.enable()
        start = a.joints()
        target = list(start)
        target[0] += 4.0
        a.servo_j(target, t=0.03)
        self.sim.settle(0.5)
        arrived = a.joints()
        self.assertGreater(arrived[0] - start[0], 3.0, "伺服应该真的走过去")

        self.sim.settle(1.0)                       # 停发
        self.assertEqual(self.sim.state()["mode"], armsim.MODE_IDLE)
        self.assertEqual(self.sim.state()["error"], 0)
        drift = max(abs(b - c) for b, c in zip(arrived, a.joints()))
        self.assertLess(drift, 1e-3, "停发之后不该跑位：%r" % drift)

    def test_t_is_a_first_order_lag(self):
        """t 越大越慢（控制器朝「t 秒前的目标」走）。"""
        a = self.open_arm()
        a.enable()
        start = a.joints()
        fast = list(start)
        fast[0] += 6.0
        a.servo_j(fast, t=0.03)
        self.sim.settle(0.09)
        moved_fast = a.joints()[0] - start[0]

        self.sim.settle(1.0)                       # 停稳
        base = a.joints()
        slow = list(base)
        slow[0] += 6.0
        a.servo_j(slow, t=0.3)
        self.sim.settle(0.09)
        moved_slow = a.joints()[0] - base[0]
        self.assertGreater(moved_fast, moved_slow * 1.5,
                           "t=0.03 应该比 t=0.3 快得多：%0.2f vs %0.2f"
                           % (moved_fast, moved_slow))

    def test_servo_rejects_wrong_argument_count(self):
        a = self.open_arm()
        a.enable()
        with self.assertRaises(arm_mod.ArmError):
            a._command("ServoJ(1,2,3,t=0.03)")

    def test_result_ids_increase_per_frame(self):
        a = self.open_arm()
        a.enable()
        joints = a.joints()
        first = a.servo_j(joints, t=0.03)
        second = a.servo_j(joints, t=0.03)
        self.assertGreater(second, first)
        self.assertEqual(self.sim.state()["servo_frames"], 2)


class TestSpeedFactor(_SimCase):
    """手册 p.89：SpeedFactor 不影响 ServoJ/ServoP，只影响 MovJ/MovL。"""

    def test_speed_factor_does_not_slow_servoj(self):
        a = self.open_arm()
        a.enable()
        start = a.joints()

        a.speed_factor(100)
        fast = list(start)
        fast[0] += 4.0
        a.servo_j(fast, t=0.03)
        self.sim.settle(0.12)
        moved_100 = a.joints()[0] - start[0]

        self.sim.settle(0.8)
        base = a.joints()
        a.speed_factor(5)                          # 慢到 5%
        slow = list(base)
        slow[0] += 4.0
        a.servo_j(slow, t=0.03)
        self.sim.settle(0.12)
        moved_5 = a.joints()[0] - base[0]

        self.assertGreater(moved_100, 2.0)
        self.assertAlmostEqual(moved_100, moved_5, delta=0.4,
                               msg="SpeedFactor 不该影响 ServoJ：%0.2f vs %0.2f"
                                   % (moved_100, moved_5))

    def test_speed_factor_does_slow_movj(self):
        """反过来钉住：MovJ 必须受 SpeedFactor 影响，否则上面那条测试没意义。"""
        a = self.open_arm()
        a.enable()
        start = a.joints()

        a.speed_factor(100)
        fast = list(start)
        fast[0] += 10.0
        a.move_joints(fast, wait=False)
        self.sim.settle(0.15)
        moved_100 = a.joints()[0] - start[0]
        self.assertTrue(wait_for(lambda: self.sim.state()["queue_len"] == 0, timeout=3.0))

        self.sim.settle(0.5)
        base = a.joints()
        a.speed_factor(5)
        slow = list(base)
        slow[0] += 10.0
        a.move_joints(slow, wait=False)
        self.sim.settle(0.15)
        moved_5 = a.joints()[0] - base[0]
        self.assertGreater(moved_100, moved_5 * 2.0,
                           "100%% 应该明显比 5%% 快：%0.2f vs %0.2f"
                           % (moved_100, moved_5))

    def test_ack_behaviour_is_unchanged(self):
        """改速度后 ServoJ 的 ACK 码和延迟量级都不变（实测 60/60 全接受）。"""
        a = self.open_arm()
        a.enable()
        joints = a.joints()

        def sample():
            lat = []
            for _ in range(10):
                started = time.perf_counter()
                a.servo_j(joints, t=0.03)          # 报错就说明 ACK 行为变了
                lat.append((time.perf_counter() - started) * 1000.0)
            return statistics.median(lat)

        a.speed_factor(100)
        before = sample()
        a.speed_factor(3)
        after = sample()
        self.assertLess(abs(after - before), 5.0,
                        "SpeedFactor 不该改变 ServoJ 的 ACK：%0.2f vs %0.2f"
                        % (before, after))


class TestReturnCodes(_SimCase):
    """两个返回码要能分开：-1 是状态不允许，-10000 是指令不存在。"""

    def test_servoj_while_disabled_returns_minus_one(self):
        a = self.open_arm()
        with self.assertRaises(arm_mod.ArmError) as ctx:
            a._command("ServoJ(0,0,0,0,0,0,t=0.03)")
        self.assertTrue(str(ctx.exception).startswith("-1,"), str(ctx.exception))

    def test_unknown_command_returns_minus_10000(self):
        a = self.open_arm()
        with self.assertRaises(arm_mod.ArmError) as ctx:
            a._command("NoSuchCommand(1,2,3)")
        self.assertTrue(str(ctx.exception).startswith("-10000,"), str(ctx.exception))

    def test_sync_does_not_exist_in_v46(self):
        """手册 V4.6.0 全文没有 Sync()，真机会回 -10000。"""
        a = self.open_arm()
        with self.assertRaises(arm_mod.ArmError) as ctx:
            a._command("Sync()")
        self.assertIn("-10000", str(ctx.exception))

    def test_getqueuedcmdcurrentindex_is_missing(self):
        """arm.queue_count() 依赖这一点：本机固件没有这条指令。"""
        a = self.open_arm()
        self.assertEqual(a.queue_count(), -1)


class TestSingleClient(_SimCase):
    """29999 同一时刻只允许一个客户端：后连上的把先连上的踢掉。"""

    def test_second_client_kicks_the_first(self):
        import socket
        first = socket.create_connection(("127.0.0.1", self.sim.port), timeout=2.0)
        self.addCleanup(first.close)
        first.settimeout(1.0)
        first.sendall(b"RobotMode()\n")
        self.assertIn(b"RobotMode", first.recv(4096))

        second = socket.create_connection(("127.0.0.1", self.sim.port), timeout=2.0)
        self.addCleanup(second.close)
        second.settimeout(1.0)
        second.sendall(b"RobotMode()\n")
        self.assertIn(b"RobotMode", second.recv(4096))

        # 先连的那个应该已经拿到 EOF / 连接重置（等价 Windows 10053）
        kicked = False
        try:
            first.sendall(b"RobotMode()\n")
            kicked = first.recv(4096) == b""
        except OSError:
            kicked = True
        self.assertTrue(kicked, "第一个客户端没有被踢掉")
        self.assertGreaterEqual(self.sim.state()["kicked"], 1)

    def test_dobotarm_reconnects_after_being_kicked(self):
        """DobotArm 自带一次重连：被顶掉之后再发指令还能自己站起来。"""
        a = self.open_arm()
        a.pose()
        other = self.open_arm()
        other.pose()                               # 顶掉 a
        self.assertEqual(a.pose(), a.pose(), "重连之后照常工作")

    def test_disconnect_injection_drops_everyone(self):
        a = self.open_arm()
        a.pose()
        self.sim.inject("disconnect")
        self.assertEqual(self.sim.state()["clients"], 0)


class TestQueueBacklog(_SimCase):
    """积压超过阈值时新指令被拒（真机固件回 -50001 一类）。"""

    def test_backlog_makes_the_queue_reject(self):
        a = self.open_arm()
        a.enable()
        self.assertGreater(a.movl_cp([10.0] + P0[1:], 0), 0)

        injected = self.sim.inject("queue_backlog", n=40, capacity=32)
        self.assertGreaterEqual(injected["queue_len"], 32)
        with self.assertRaises(arm_mod.ArmError) as ctx:
            a.movl_cp([20.0] + P0[1:], 0)
        self.assertTrue(str(ctx.exception).startswith("%d" % armsim.ERR_QUEUE),
                        str(ctx.exception))

    def test_reset_clears_the_backlog(self):
        a = self.open_arm()
        a.enable()
        self.sim.inject("queue_backlog", n=40, capacity=8)
        with self.assertRaises(arm_mod.ArmError):
            a.movl_cp([10.0] + P0[1:], 0)
        self.sim.inject("reset")
        self.assertGreater(a.movl_cp([10.0] + P0[1:], 0), 0)


class TestFaults(_SimCase):
    """故障注入：报警 / 丢帧 / 碰撞 / 急停，都要能从 30004 看见。"""

    def _feed(self):
        feed = arm_feed.Feed("127.0.0.1", self.sim.feedback_port).open()
        self.addCleanup(feed.close)
        feed.wait_frame(timeout=3.0)
        return feed

    def test_alarm_sets_mode_nine_and_error_status(self):
        a = self.open_arm()
        feed = self._feed()
        a.enable()
        self.sim.inject("alarm", code=7)
        described = wait_for(lambda: feed.snapshot()["RobotMode"] == armsim.MODE_ALARM,
                             timeout=2.0)
        self.assertTrue(described, "报警后 RobotMode 应该是 9")
        self.assertEqual(int(feed.snapshot()["ErrorStatus"]), 7)
        self.assertEqual(int(feed.snapshot()["EnableStatus"]), 1,
                         "报警不清使能状态（真机也是抱着闸）")

        with self.assertRaises(arm_mod.ArmError):
            a.movl_cp([10.0] + P0[1:], 0)
        a.clear_error()
        self.assertTrue(wait_for(
            lambda: int(feed.snapshot()["ErrorStatus"]) == 0, timeout=2.0))
        self.assertGreater(a.movl_cp([10.0] + P0[1:], 0), 0)

    def test_estop_clears_the_safety_bit(self):
        feed = self._feed()
        self.sim.inject("estop", on=True)
        self.assertTrue(wait_for(
            lambda: int(feed.snapshot()["SafetyState"]) & 0x01 == 0, timeout=2.0))
        self.assertTrue(arm_feed.describe(feed.snapshot())["safety"]["estop"])
        self.assertEqual(int(feed.snapshot()["RobotMode"]), armsim.MODE_ALARM)

    def test_collision_stops_the_motion(self):
        a = self.open_arm()
        feed = self._feed()
        a.enable()
        start = a.joints()
        target = list(start)
        target[0] += 20.0
        a.servo_j(target, t=0.03)
        self.sim.settle(0.1)

        self.sim.inject("collision")
        self.assertTrue(wait_for(
            lambda: int(feed.snapshot()["CollisionState"]) == 1, timeout=2.0))
        frozen = a.joints()
        self.sim.settle(0.4)
        self.assertLess(max(abs(x - y) for x, y in zip(frozen, a.joints())), 1e-3,
                        "碰撞之后不该继续跟目标")
        with self.assertRaises(arm_mod.ArmError):
            a.servo_j(target, t=0.03)

    def test_dropped_frames_are_counted_and_felt(self):
        feed = self._feed()
        report = self.sim.inject("drop_frames", frames=40)
        self.assertEqual(report["frames"], 40)
        self.assertTrue(wait_for(
            lambda: self.sim.state()["dropped_frames"] >= 20, timeout=3.0))
        self.assertGreater(feed.frames, 0, "丢帧期间 Feed 仍然拿着上一帧")

    def test_unknown_fault_raises(self):
        with self.assertRaises(ValueError):
            self.sim.inject("self-destruct")

    def test_state_snapshot_has_the_documented_keys(self):
        keys = {"mode", "enabled", "pose", "joints", "error", "queue_len",
                "current_id", "servo_frames"}
        self.assertTrue(keys.issubset(set(self.sim.state())))


class TestServoQueueModes(unittest.TestCase):
    """latest-wins 是默认（真机实测语义），fifo 用来复现橡皮筋。"""

    def test_latest_wins_keeps_only_the_newest_frame(self):
        sim = armsim.VirtualDobot(seed=2).start()
        self.addCleanup(sim.stop)
        a = sim.arm()
        self.addCleanup(a.close)
        a.enable()
        joints = a.joints()
        for i in range(5):
            a.servo_j([v + i * 0.5 for v in joints], t=0.03)
        self.assertEqual(sim.state()["servo_queue_len"], 1)

    def test_fifo_queues_every_frame(self):
        sim = armsim.VirtualDobot(seed=2, servo_queue="fifo").start()
        self.addCleanup(sim.stop)
        a = sim.arm()
        self.addCleanup(a.close)
        a.enable()
        joints = a.joints()
        for i in range(5):
            a.servo_j([v + i * 0.5 for v in joints], t=0.03)
        self.assertEqual(sim.state()["servo_queue_len"], 5)
        sim.settle(0.5)
        self.assertEqual(sim.state()["servo_queue_len"], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
