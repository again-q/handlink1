"""仿真臂「拟真度」回归：这些行为是照着真机实测写的，退化回去就该红。

每条用例对应一条真机事实，出处写在 docstring 里。改 arm.SimArm 之前先跑一遍。
"""
from __future__ import annotations

import json
import time
import unittest

from handlink import arm, kin

READY = [0.0, -25.0, -40.0, 0.0, 65.0, 0.0]


class TestKinematicTruth(unittest.TestCase):
    """关节角是唯一真相：pose() 与 joints() 不能是两份互不相干的数。"""

    def test_pose_is_forward_kinematics_of_joints(self):
        sim = arm.SimArm()
        sim.enable()
        sim.move_joints(READY, wait=True)
        pose = sim.pose()
        expected = kin.fk(READY)
        for i in range(6):
            self.assertAlmostEqual(pose[i], expected[i], places=2)

    def test_joint_move_moves_the_pose(self):
        sim = arm.SimArm()
        sim.enable()
        before = sim.pose()[:3]
        sim.move_joints(READY, wait=True)
        after = sim.pose()[:3]
        self.assertGreater(max(abs(a - b) for a, b in zip(before, after)), 10.0)

    def test_pose_move_moves_the_joints(self):
        sim = arm.SimArm()
        sim.enable()
        sim.move_j([400.0, 0.0, 200.0, 180.0, 0.0, 0.0], wait=True)
        q = sim.joints()
        self.assertGreater(max(abs(v) for v in q), 1.0)
        self.assertAlmostEqual(kin.fk(q)[2], 200.0, delta=2.0)

    def test_constructor_joints_wins_over_pose(self):
        sim = arm.SimArm(joints=READY)
        for a, b in zip(sim.joints(), READY):
            self.assertAlmostEqual(a, b, places=6)
        self.assertAlmostEqual(sim.pose()[2], kin.fk(READY)[2], places=2)


class TestTiming(unittest.TestCase):
    """会花时间：加速度受限的梯形曲线，按 SpeedFactor 缩放（手册：六轴 100 度/秒）。"""

    def test_servo_speed_grows_with_speed_factor(self):
        slow = arm.SimArm(joints=READY)
        fast = arm.SimArm(joints=READY)
        slow.enable(); fast.enable()
        slow.speed_factor(20); fast.speed_factor(100)
        slow.move_joints([0.0, -65.0, -40.0, 0.0, 65.0, 0.0], wait=False)
        fast.move_joints([0.0, -65.0, -40.0, 0.0, 65.0, 0.0], wait=False)
        self.assertGreater(slow._queue[0]["dur"], fast._queue[0]["dur"] * 2)

    def test_duration_is_physically_bounded(self):
        """40 度行程、速度因子 100%：100 度/秒 下至少 0.35 s，不可能瞬移。"""
        sim = arm.SimArm(joints=READY)
        sim.enable()
        sim.speed_factor(100)
        result = sim.move_joints([0.0, -65.0, -40.0, 0.0, 65.0, 0.0], wait=False)
        self.assertGreaterEqual(result["eta_s"], 0.35)
        self.assertLessEqual(result["eta_s"], 2.0)

    def test_wait_does_not_block_on_the_virtual_clock(self):
        """默认虚拟时钟：wait=True 立刻返回，但状态确实走到了目标。"""
        sim = arm.SimArm(joints=READY)
        sim.enable()
        sim.speed_factor(10)
        started = time.perf_counter()
        result = sim.move_joints([0.0, -5.0, -40.0, 0.0, 65.0, 0.0], wait=True)
        elapsed = time.perf_counter() - started
        self.assertTrue(result["arrived"])
        self.assertLess(elapsed, 1.0)
        self.assertAlmostEqual(sim.joints()[1], -5.0, places=3)

    def test_realtime_mode_really_waits(self):
        sim = arm.SimArm(joints=READY, realtime=True)
        sim.enable()
        sim.speed_factor(100)
        started = time.perf_counter()
        sim.move_joints([0.0, -28.0, -40.0, 0.0, 65.0, 0.0], wait=True, timeout=5.0)
        self.assertGreater(time.perf_counter() - started, 0.01)


class TestQueue(unittest.TestCase):
    """算法队列：wait=False 排队执行，GetCurrentCommandID 跟着跳。"""

    def test_batch_queues_and_ids_grow(self):
        sim = arm.SimArm(joints=READY)
        sim.enable()
        ids = [sim.move_joints([0.0, -25.0 - i, -40.0, 0.0, 65.0, 0.0], wait=False)["id"]
               for i in range(5)]
        self.assertEqual(ids, sorted(ids))
        self.assertEqual(len(set(ids)), 5)

    def test_advance_runs_the_queue(self):
        sim = arm.SimArm(joints=READY)
        sim.enable()
        for i in range(3):
            sim.move_joints([0.0, -25.0 - i * 5, -40.0, 0.0, 65.0, 0.0], wait=False)
        sim.advance(60.0)
        self.assertIsNone(sim._active)
        self.assertEqual(len(sim._queue), 0)
        self.assertAlmostEqual(sim.joints()[1], -35.0, places=3)

    def test_queue_overflow_is_rejected(self):
        """真机固件队列积压也会拒；仿真必须能走到同一条代码路径。"""
        sim = arm.SimArm(joints=READY, max_queue=3)
        sim.enable()
        sim.speed_factor(5)
        for _ in range(3):
            sim.move_joints([0.0, -60.0, -40.0, 0.0, 65.0, 0.0], wait=False)
        with self.assertRaises(arm.ArmError) as ctx:
            sim.move_joints([0.0, -60.0, -40.0, 0.0, 65.0, 0.0], wait=False)
        self.assertIn("-50001", str(ctx.exception))

    def test_get_current_command_id_matches_result_ids(self):
        sim = arm.SimArm(joints=READY)
        sim.enable()
        rid = sim.move_joints([0.0, -30.0, -40.0, 0.0, 65.0, 0.0], wait=False)["id"]
        sim.advance(10.0)
        self.assertGreaterEqual(sim.current_cmd_id(), rid)


class TestErrorCodes(unittest.TestCase):
    """错误码照抄真机：这是故障场景唯一的统一判据。"""

    def test_motion_without_enable_is_minus_1(self):
        sim = arm.SimArm()
        self.assertEqual(sim.command("MovJ(joint={0,0,0,0,0,0})"), "-1,{},MovJ(...);")

    def test_servo_without_enable_is_minus_1_not_unknown(self):
        """实测：未使能发 ServoJ 回 -1（状态不允许），不是 -10000（指令不存在）。"""
        reply = arm.SimArm().command("ServoJ(0,0,0,0,0,0,0.03)")
        self.assertTrue(reply.startswith("-1,"), reply)

    def test_unknown_command_is_minus_10000(self):
        sim = arm.SimArm()
        sim.enable()
        self.assertTrue(sim.command("FlyToMoon()").startswith("-10000,"))

    def test_joint_limit_is_minus_2(self):
        sim = arm.SimArm()
        sim.enable()
        with self.assertRaises(arm.ArmError) as ctx:
            sim.move_joints([0.0, 200.0, 0.0, 0.0, 0.0, 0.0])
        self.assertIn("-2", str(ctx.exception))

    def test_robot_mode_reflects_state(self):
        sim = arm.SimArm()
        self.assertIn("{4}", sim.command("RobotMode()"))
        sim.enable()
        self.assertIn("{5}", sim.command("RobotMode()"))
        sim._error = 7
        self.assertIn("{9}", sim.command("RobotMode()"))

    def test_get_pose_and_angle_are_parseable_by_the_real_client(self):
        """真机客户端解析器必须能直接吃仿真回复，否则「同一段脚本两边跑」是假的。"""
        sim = arm.SimArm()
        sim.enable()
        self.assertEqual(len(arm.parse_pose(sim.command("GetPose()"))), 6)
        reply = sim.command("GetAngle()")
        start, end = reply.find("{"), reply.rfind("}")
        self.assertEqual(len([x for x in reply[start + 1:end].split(",") if x.strip()]), 6)


class TestServoSemantics(unittest.TestCase):
    """ServoJ：手册 p.89 起；数字来自 2026-11-15 真机实测。"""

    def test_default_t_is_the_documented_trap(self):
        """手册默认 t=0.1，控制器朝 100 ms 前的目标走，被误判成网络慢。"""
        sim = arm.SimArm()
        sim.enable()
        self.assertAlmostEqual(sim._servo_t, 0.1, places=6)
        sim.servo_j([0.0] * 6)
        self.assertAlmostEqual(sim._servo_t, 0.03, places=6)

    def test_explicit_t_is_preserved_and_clamped(self):
        sim = arm.SimArm()
        sim.enable()
        sim.servo_j([0.0] * 6, t=0.5)
        self.assertAlmostEqual(sim._servo_t, 0.5, places=6)
        sim.servo_j([0.0] * 6, t=0.0001)
        self.assertAlmostEqual(sim._servo_t, 0.004, places=6)

    def test_speed_factor_does_not_touch_servoj(self):
        """手册 p.89：ServoJ/ServoP 不受全局速率影响。这是真机上踩过的坑。"""
        slow = arm.SimArm(joints=READY)
        fast = arm.SimArm(joints=READY)
        slow.enable(); fast.enable()
        slow.speed_factor(1); fast.speed_factor(100)
        for i in range(5):
            slow.servo_j([0.0, -25.0 - i, -40.0, 0.0, 65.0, 0.0])
            fast.servo_j([0.0, -25.0 - i, -40.0, 0.0, 65.0, 0.0])
        self.assertEqual(slow._servo_frames, fast._servo_frames)
        self.assertAlmostEqual(slow._servo_t, fast._servo_t, places=6)

    def test_holding_the_same_target_is_zero_drift(self):
        """实测：目标=当前关节角，连续 60 帧后关节漂移 0.000 度。"""
        sim = arm.SimArm(joints=READY)
        sim.enable()
        for _ in range(60):
            sim.servo_j(sim.joints())
        for a, b in zip(sim.joints(), READY):
            self.assertAlmostEqual(a, b, places=6)

    def test_stopping_servo_holds_last_target_without_alarm(self):
        """实测：停发 10 s 不跑位、不报警（RobotMode=5、ErrorStatus=0）。"""
        sim = arm.SimArm(joints=READY)
        sim.enable()
        sim.servo_j([0.0, -30.0, -35.0, 0.0, 60.0, 0.0])
        sim._servo_target = None
        sim.advance(10.0)
        for a, b in zip(sim.joints(), READY):
            self.assertAlmostEqual(a, b, places=6)
        self.assertIn("{5}", sim.command("RobotMode()"))
        self.assertIn("[]", sim.command("GetErrorID()"))

    def test_fifo_mode_is_available_to_reproduce_rubber_banding(self):
        """手册把 ServoJ 归为队列指令，实测却是 latest-wins；两种都要能选。"""
        fifo = arm.SimArm(joints=READY, servo_queue="fifo")
        fifo.enable()
        for i in range(4):
            fifo.servo_j([0.0, -25.0 - i, -40.0, 0.0, 65.0, 0.0])
        self.assertEqual(len(fifo._servo_pending), 4)
        latest = arm.SimArm(joints=READY)
        latest.enable()
        for i in range(4):
            latest.servo_j([0.0, -25.0 - i, -40.0, 0.0, 65.0, 0.0])
        self.assertEqual(len(latest._servo_pending), 0)


class TestSameTextOnBothArms(unittest.TestCase):
    """同一段 29999 文本必须两边都能跑，这是「仿真即真机」的前提。"""

    SCRIPT = [
        "EnableRobot()",
        "SpeedFactor(20)",
        "MovJ(joint={0,-25,-40,0,65,0})",
        "GetCurrentCommandID()",
        "GetPose()",
        "GetAngle()",
        "GetErrorID()",
        "DisableRobot()",
    ]

    def test_every_command_returns_a_dobot_shaped_reply(self):
        sim = arm.SimArm()
        for line in self.SCRIPT:
            reply = sim.command(line)
            self.assertIsInstance(reply, str)
            self.assertTrue(reply.rstrip().endswith(";"), reply)
            self.assertIn("{", reply)

    def test_trace_is_json_serialisable(self):
        sim = arm.SimArm(joints=READY)
        sim.enable()
        sim.move_joints([0.0, -30.0, -40.0, 0.0, 65.0, 0.0], wait=True)
        json.dumps(sim.trace)


if __name__ == "__main__":
    unittest.main()
