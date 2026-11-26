"""handsim 的验收测试：把 docs/现状.md 第 2 节的实测数字逐条钉住。

全部用 step() 推进虚拟时间，不 sleep；整包 < 20 s。
    python -m unittest tests.test_handsim -v
"""
from __future__ import annotations

import unittest

from handlink import bus, spec
from handlink.handsim import (
    BIT_HW_OC,
    BIT_I_LIMIT,
    BIT_POS_LIMIT,
    BIT_RUNNING,
    BIT_SW_OC,
    BIT_UNDERVOLT,
    HandSim,
)

DT = 0.002
INDEX = 103
SETTLE = 0.05          # 等于最大下发延迟（15 ms）再留一点余量


def run_for(sim: HandSim, seconds: float, dt: float = DT) -> None:
    """用固定步长推进虚拟时间。"""
    for _ in range(int(round(seconds / dt))):
        sim.step(dt)


class HandSimTest(unittest.TestCase):
    def make(self, **kwargs) -> HandSim:
        sim = HandSim(**kwargs)
        sim.open()
        self.addCleanup(sim.close)
        sim.attach()
        self.addCleanup(sim.detach)
        return sim

    def rest_angle(self, sim: HandSim, mid: int) -> float:
        """回零后的落点。注意：每次重新校准都会把手指拉回零位（真机回零语义），
        所以只能取「刚上电、还没有新指令」的那一次。"""
        return sim.state()[mid].angle

    # ---------------------------------------------------------- 1. 速度/行程
    def test_90deg_trip_takes_1_6s(self):
        """[实测] speed=200 → 约 55°/s；90° 行程 1.6 s。"""
        sim = self.make(seed=0)
        zero = self.rest_angle(sim, INDEX)
        target = zero + 90.0

        sim.apply({INDEX: spec.Joint(target, 200.0, 1.5)})
        # 不能瞬移：50 ms 才走了一点点
        run_for(sim, 0.05)
        early = sim.state()[INDEX].angle - zero
        self.assertLess(early, 6.0, f"50 ms 就走了 {early:.1f}°，太快，不像真机")

        # 巡航段速度应接近 55 °/s
        run_for(sim, 0.45)                      # t ≈ 0.5 s，已在匀速段
        p0 = sim.state()[INDEX].angle
        run_for(sim, 0.4)
        cruise = (sim.state()[INDEX].angle - p0) / 0.4
        self.assertAlmostEqual(cruise, 55.0, delta=3.0)   # [实测] 55–57 °/s

        # 继续走到 +90°，记录到位时刻
        reached = None
        for _ in range(int(round(2.0 / DT))):
            sim.step(DT)
            if sim.state()[INDEX].angle >= target - 0.5:
                reached = sim.t
                break
        self.assertIsNotNone(reached, "90° 行程 2 s 内没走完")
        self.assertAlmostEqual(reached, 1.6, delta=0.25,
                               msg=f"90° 实际 {reached:.2f}s，实测 1.6s")

        # [实测] 2 s 内稳定：到 2.2 s 已贴住终点（落点含 0.5–1.5° 回零偏差）
        run_for(sim, max(0.0, 2.2 - sim.t))
        settled = sim.state()[INDEX].angle
        self.assertGreaterEqual(settled, target - 0.2)
        self.assertLessEqual(settled, target + 1.5)
        self.assertEqual(sim.state()[INDEX].speed, 0.0, "到位后不应还在爬行")

    # ---------------------------------------------------------- 2. 电流不足
    def test_06a_stalls_near_118deg(self):
        """[实测] 0.6 A 多指同动触发 I-Lim，卡在 118°。"""
        sim = self.make(seed=1)
        sim.apply({m: spec.Joint(180.0, 200.0, 0.6) for m in spec.MOTOR_IDS})
        run_for(sim, 3.0)

        st = sim.state()[INDEX]
        self.assertAlmostEqual(st.angle, 118.0, delta=4.0,
                               msg=f"0.6 A 停在 {st.angle:.1f}°，实测 118°")
        self.assertIn("I-LIM", st.error or "")
        self.assertAlmostEqual(st.current, 0.6, delta=0.02)

        hb = sim.heartbeat()["motors"][INDEX]
        self.assertTrue(hb["running"] & BIT_I_LIMIT, "心跳应点亮 0x10 电流限幅")
        self.assertIn("i-limit", hb["running_bits"])

        # 卡住之后继续推进也不动（不是慢，是真的走不动）
        before = sim.state()[INDEX].angle
        run_for(sim, 1.0)
        self.assertLess(abs(sim.state()[INDEX].angle - before), 0.5)

    def test_10a_finishes_the_trip(self):
        """[实测] ≥1.0 A 能走完。"""
        sim = self.make(seed=1)
        sim.apply({m: spec.Joint(180.0, 200.0, 1.0) for m in spec.MOTOR_IDS})
        run_for(sim, 3.5)
        st = sim.state()[INDEX]
        self.assertGreater(st.angle, 175.0, f"1.0 A 只走到 {st.angle:.1f}°")
        self.assertIsNone(st.error)
        self.assertFalse(sim.heartbeat()["motors"][INDEX]["running"] & BIT_I_LIMIT)

    # ---------------------------------------------------------- 3. 电流上限
    def test_current_limits_are_thumb_08_others_35(self):
        """[手册] 拇指 0.8 A、其余 3.5 A；超了钳位并点亮状态位。"""
        sim = self.make(seed=2)
        res = sim.apply({
            102: spec.Joint(30.0, 100.0, 2.0),      # 拇指，越 0.8
            INDEX: spec.Joint(60.0, 200.0, 6.0),    # 四指，越 3.5
        })
        clamped = {(c["motor_id"], c["field"]) for c in res["clamped"]}
        self.assertIn((102, "torque"), clamped)
        self.assertIn((INDEX, "torque"), clamped)
        for item in res["clamped"]:
            self.assertEqual(item["limit"], spec.MOTOR_BY_ID[item["motor_id"]].current_max)

        run_for(sim, 0.2)
        hb = sim.heartbeat()["motors"]
        self.assertTrue(hb[102]["running"] & BIT_SW_OC)
        self.assertTrue(hb[INDEX]["running"] & BIT_SW_OC)
        self.assertEqual(hb[102]["current_limit"], 0.8)
        self.assertEqual(hb[INDEX]["current_limit"], 3.5)
        # 实际电流不会超过硬件上限
        self.assertLessEqual(sim.state()[102].current, 0.8 + 1e-9)
        self.assertLessEqual(sim.state()[INDEX].current, 3.5 + 1e-9)

    def test_position_limit_bit(self):
        """[手册] 0x20 位置限幅：下发超出 pos_max 时点亮。"""
        sim = self.make(seed=2)
        res = sim.apply({107: spec.Joint(90.0, 100.0, 0.5)})   # 拇指2 上限 45°
        self.assertTrue(any(c["field"] == "angle" for c in res["clamped"]))
        run_for(sim, 0.05)
        self.assertTrue(sim.heartbeat()["motors"][107]["running"] & BIT_POS_LIMIT)

    # ---------------------------------------------------------- 4. 心跳
    def test_heartbeat_3000_frames_per_second(self):
        """[实测] 每指约 500 Hz，六指合计约 3000 帧/秒。"""
        sim = self.make(seed=4, hz=500.0)
        run_for(sim, 1.0)
        self.assertAlmostEqual(sim.frames(), 3000, delta=6)
        hb = sim.heartbeat()
        self.assertAlmostEqual(hb["frames_per_s"], 3000.0, delta=1.0)
        for mid in spec.MOTOR_IDS:
            self.assertAlmostEqual(hb["motors"][mid]["count"], 500, delta=3)
            self.assertEqual(hb["motors"][mid]["calib"], 0x08)
            self.assertTrue(hb["motors"][mid]["calibrated"])
        run_for(sim, 1.0)                      # 再过 1 s 再涨 3000
        self.assertAlmostEqual(sim.frames(), 6000, delta=6)

    def test_running_bit_only_while_moving(self):
        """[手册] 0x01 运行位。"""
        sim = self.make(seed=4)
        self.assertFalse(sim.heartbeat()["motors"][INDEX]["running"] & BIT_RUNNING)
        sim.apply({INDEX: spec.Joint(90.0, 200.0, 1.5)})
        run_for(sim, 0.3)
        self.assertTrue(sim.heartbeat()["motors"][INDEX]["running"] & BIT_RUNNING)

    # ---------------------------------------------------------- 5. 重复性
    def test_repeatability_05_to_15_deg(self):
        """[实测] 回零后落在 0.5–1.5°（随机但按 seed 可复现）。"""
        landings = []
        for seed in range(8):
            sim = HandSim(seed=seed)
            sim.open()
            try:
                for mid in spec.MOTOR_IDS:
                    value = sim.state()[mid].angle
                    self.assertGreaterEqual(value, 0.5)
                    self.assertLessEqual(value, 1.5)
                    landings.append(value)
            finally:
                sim.close()
        self.assertEqual(len(landings), 8 * len(spec.MOTOR_IDS))
        self.assertGreater(len(set(round(v, 4) for v in landings)), 6, "落点应随机散布")

        # 同一个 seed 必须逐位可复现
        a, b = HandSim(seed=5), HandSim(seed=5)
        a.open()
        b.open()
        try:
            self.assertEqual({m: a.state()[m].angle for m in spec.MOTOR_IDS},
                             {m: b.state()[m].angle for m in spec.MOTOR_IDS})
        finally:
            a.close()
            b.close()

    # ---------------------------------------------------------- 6. 断电变软
    def test_power_off_sags_and_needs_recalibration(self):
        """[实测] 断电手指变软、位置塌陷；再上电要重新校准。"""
        sim = self.make(seed=6)
        sim.apply({INDEX: spec.Joint(50.0, 200.0, 2.0)})
        run_for(sim, 2.5)
        high = sim.state()[INDEX].angle
        self.assertGreater(high, 45.0)

        sim.inject("power_off")
        run_for(sim, 0.5)
        half = sim.state()[INDEX].angle
        self.assertLess(half, high)              # 开始塌
        self.assertGreater(half, 5.0)            # 但没瞬移
        run_for(sim, 4.0)
        low = sim.state()[INDEX].angle
        self.assertLess(low, 3.0, f"断电 4.5 s 还停在 {low:.1f}°")
        self.assertEqual(sim.state()[INDEX].current, 0.0)
        self.assertFalse(sim.heartbeat()["calibrated"])
        self.assertIn("POWER_OFF", sim.state()[INDEX].error or "")

        # 重新上电：先未校准，等一会儿才校准成功（calib & 0x08）
        sim.inject("power_off", on=False)
        self.assertFalse(sim.heartbeat()["calibrated"])
        self.assertEqual(sim.heartbeat()["motors"][INDEX]["calib"], 0x00)
        run_for(sim, 0.7)
        self.assertTrue(sim.heartbeat()["calibrated"])
        self.assertEqual(sim.heartbeat()["motors"][INDEX]["calib"], 0x08)
        # 校准即回零：落点回到 0.5–1.5°
        self.assertLessEqual(sim.state()[INDEX].angle, 1.5)

    # ---------------------------------------------------------- 7. 适配器独占
    def test_adapter_is_exclusive(self):
        """[实测] 同一时刻只有一个进程能占用适配器。"""
        a = HandSim(seed=0)
        a.open()
        b = HandSim(seed=0)
        b.open()
        try:
            a.attach()
            with self.assertRaises(RuntimeError):
                a.attach()                 # 重复 attach
            with self.assertRaises(RuntimeError):
                b.attach()                 # 别人占着
            a.detach()
            b.attach()                     # 释放后可再占用
            b.detach()
            a.attach()
            a.close()                      # close 也要释放
            b.attach()
            b.detach()
        finally:
            a.close()
            b.close()

    # ---------------------------------------------------------- 8. 丢帧/延迟
    def test_can_loss_is_observable_and_blocks_motion(self):
        """[实测] CAN 会丢帧；丢了就是丢了（电机保持旧目标）。"""
        sim = self.make(seed=8)
        zero = self.rest_angle(sim, INDEX)

        sim.inject("can_loss", rate=1.0)
        dropped = sim.apply({INDEX: spec.Joint(zero + 90.0, 200.0, 1.5)})
        self.assertTrue(dropped["dropped"])
        self.assertEqual(dropped["applied"], [])
        self.assertEqual(dropped["lost_frames"], 1)
        run_for(sim, 1.0)
        self.assertLess(abs(sim.state()[INDEX].angle - zero), 0.05,
                        "丢帧后电机不应收到新目标")

        # 恢复后同一帧能过去
        sim.inject("can_loss", on=False)
        ok = sim.apply({INDEX: spec.Joint(zero + 90.0, 200.0, 1.5)})
        self.assertFalse(ok["dropped"])
        self.assertEqual(ok["applied"], [INDEX])
        run_for(sim, 1.0)
        self.assertGreater(sim.state()[INDEX].angle - zero, 40.0)

        # 半丢：长期看丢帧比例接近注入值
        sim.inject("can_loss", rate=0.5)
        dropped_n = 0
        for _ in range(200):
            if sim.apply({INDEX: spec.Joint(zero, 60.0, 0.5)})["dropped"]:
                dropped_n += 1
            run_for(sim, SETTLE, dt=0.02)
        self.assertGreater(dropped_n, 70, f"丢帧率明显偏低：{dropped_n}/200")
        self.assertLess(dropped_n, 130, f"丢帧率明显偏高：{dropped_n}/200")

    def test_latency_jitter_1_to_15ms(self):
        """[实测] 单帧 Transmit 1–15 ms。"""
        sim = self.make(seed=9)
        delays = [sim.apply({INDEX: spec.Joint(10.0, 60.0, 0.5)})["latency_ms"]
                  for _ in range(200)]
        self.assertGreaterEqual(min(delays), 1.0)
        self.assertLessEqual(max(delays), 15.0)
        self.assertGreater(len(set(delays)), 50, "延迟应有抖动，不是常数")

    # ---------------------------------------------------------- 9. 其他故障
    def test_undervoltage_bit_and_slowdown(self):
        """[手册] 0x08 电压异常。"""
        sim = self.make(seed=10)
        sim.inject("undervoltage", seconds=0.5)
        run_for(sim, 0.1)
        self.assertTrue(sim.heartbeat()["motors"][INDEX]["running"] & BIT_UNDERVOLT)
        run_for(sim, 0.6)
        self.assertFalse(sim.heartbeat()["motors"][INDEX]["running"] & BIT_UNDERVOLT)

    def test_stall_sets_overcurrent_bits(self):
        """[手册] 0x02 硬件过流 + 0x10 电流限幅。"""
        sim = self.make(seed=10)
        sim.inject("stall", motor=INDEX)
        sim.apply({INDEX: spec.Joint(25.0, 200.0, 2.0)})
        run_for(sim, 1.0)
        bits = sim.heartbeat()["motors"][INDEX]["running"]
        self.assertTrue(bits & BIT_HW_OC)
        self.assertTrue(bits & BIT_I_LIMIT)
        self.assertLess(sim.state()[INDEX].angle, 5.0, "堵转时不应继续走")
        sim.inject("stall", motor=INDEX, on=False)
        run_for(sim, 2.0)
        self.assertGreater(sim.state()[INDEX].angle, 20.0)

    def test_uncalibrated_fault(self):
        """[手册] calib & 0x08 = 校准成功。"""
        sim = self.make(seed=10)
        sim.inject("uncalibrated")
        self.assertFalse(sim.heartbeat()["calibrated"])
        self.assertEqual(sim.heartbeat()["motors"][INDEX]["calib"], 0x00)
        sim.inject("uncalibrated", on=False)
        self.assertEqual(sim.heartbeat()["motors"][INDEX]["calib"], 0x08)

    def test_unknown_fault_rejected(self):
        sim = self.make(seed=10)
        with self.assertRaises(ValueError):
            sim.inject("meltdown")

    # ---------------------------------------------------------- 10. 接口一致性
    def test_is_a_handbus(self):
        sim = self.make(seed=11)
        self.assertIsInstance(sim, bus.HandBus)
        self.assertEqual(sim.name, "handsim")
        snap = sim.snapshot()
        self.assertEqual(snap["bus"], "handsim")
        self.assertEqual(set(snap["motors"]), {str(m) for m in spec.MOTOR_IDS})
        self.assertEqual(set(sim.pose()), set(spec.MOTOR_IDS))

    def test_deterministic_for_same_seed(self):
        """同样 seed + 同样 step 序列 → 逐位相同。"""
        def trace(seed: int):
            sim = HandSim(seed=seed)
            sim.open()
            try:
                sim.apply({m: spec.Joint(150.0, 200.0, 1.2) for m in spec.MOTOR_IDS})
                run_for(sim, 1.5)
                return {m: round(sim.state()[m].angle, 6) for m in spec.MOTOR_IDS}
            finally:
                sim.close()
        self.assertEqual(trace(12), trace(12))
        self.assertNotEqual(trace(12), trace(13))

    def test_unknown_motor_rejected(self):
        sim = self.make(seed=11)
        with self.assertRaises(ValueError):
            sim.apply({999: spec.Joint(10.0, 60.0, 0.5)})


if __name__ == "__main__":
    unittest.main(verbosity=2)
