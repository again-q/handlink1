"""数据手套解析/换算测试（不需要手套，也不需要真 socket）。

    python tests/test_glove.py
"""
from __future__ import annotations

import json
import socket
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import bus, glove, service  # noqa: E402

BONES, PER = glove.BONES, glove.PER


class Clock:
    """可控时钟：换算里的零点窗口和速率上限都按它走。"""

    def __init__(self, t: float = 100.0) -> None:
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def tick(self, dt: float = 0.05) -> float:
        self.t += float(dt)
        return self.t


def make_values(flex_deg: float = 30.0, side: str = "right",
                wrist_deg: float = 0.0, roll_deg: float = 0.0) -> list:
    """32 关节 × 6 数。

    本手 1~15 节（手指）的横向欧拉角 = flex_deg，第 0 节（手腕）= wrist_deg；
    roll_deg 模拟"整条手臂滚转"——按实测的绝对姿态语义，所有节的同一个欧拉角
    一起平移，手指自己并没有动。
    """
    base = 0 if side == "right" else 16
    values = []
    for joint in range(BONES):
        active = base <= joint < base + 16
        ey = 0.0
        if active:
            ey = (wrist_deg if joint == base else flex_deg) + roll_deg
        values += [0.0, 0.0, 0.0, 0.0, ey, 0.0]
    return values


def packets(fn: int, values: list, parts: int = 2) -> list:
    """把一帧切成 parts 个 UDP 文本包（和 MotionVenus 的 subpackage 一致）。"""
    chunk = (len(values) + parts - 1) // parts
    out = []
    for i in range(parts):
        nums = values[i * chunk:(i + 1) * chunk]
        out.append(f"fn {fn} subpackage {i + 1}/{parts}," + ",".join(f"{v:g}" for v in nums) + ",")
    return out


def free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class TestAssembler(unittest.TestCase):
    def test_two_subpackages_make_one_frame(self):
        a = glove.Assembler()
        values = make_values(30)
        p1, p2 = packets(7, values)
        self.assertIsNone(a.feed(p1))
        frame = a.feed(p2)
        self.assertIsNotNone(frame)
        self.assertEqual(len(frame), BONES * PER)
        self.assertAlmostEqual(frame[4], 0.0)              # 第 0 节 = 手腕，不弯
        self.assertAlmostEqual(frame[PER + 4], 30.0)        # 第 1 节 = 手指
        self.assertAlmostEqual(frame[5 * PER + 5], 0.0)

    def test_partial_packet_does_not_count(self):
        a = glove.Assembler()
        values = make_values(30)
        p1, _p2 = packets(9, values)
        self.assertIsNone(a.feed(p1))
        # 新的一帧（fn 10）直接找上门：旧的半帧不能拼进来
        frame = a.feed(packets(10, values)[1])
        self.assertIsNone(frame)          # 只有 2/2，凑不成整帧

    def test_newer_fn_drops_stale_half_frame(self):
        a = glove.Assembler()
        values = make_values(30)
        for text in packets(1, values):
            a.feed(text)
        for text in packets(2, values):
            frame = a.feed(text)
        self.assertIsNotNone(frame)
        self.assertEqual(list(a.pending), [])

    def test_garbage_ignored(self):
        a = glove.Assembler()
        self.assertIsNone(a.feed("hello, world"))
        self.assertIsNone(a.feed("fn 3 subpackage 1/2,"))     # 数字不够

    def test_three_way_split(self):
        a = glove.Assembler()
        values = make_values(45)
        for text in packets(4, values, parts=3):
            frame = a.feed(text)
        self.assertIsNotNone(frame)
        self.assertEqual(len(frame), BONES * PER)


def groups_of(values: list) -> list:
    return [values[i * PER:(i + 1) * PER] for i in range(BONES)]


def frame_of(joints: dict) -> list:
    """按 {关节号: (ex, ey, ez)} 造一帧，其余全是 0。"""
    values = [0.0] * (BONES * PER)
    for joint, angles in joints.items():
        for i, deg in enumerate(angles):
            values[joint * PER + 3 + i] = float(deg)
    return values


class TestAngles(unittest.TestCase):
    """abs = 1.1.x 的老算法（欧拉角直接求和）；rel = 现在默认的"减掉手腕"。"""

    def test_abs_chain_sum(self):
        groups = [[0, 0, 0, 0, 30, 10] for _ in range(BONES)]
        raw = glove.chain_angles(groups, "right", frame="abs")
        self.assertAlmostEqual(raw[103], 90)     # 食指 3 节 × 各 30
        self.assertAlmostEqual(raw[107], 60)     # 拇指2 2 节
        self.assertAlmostEqual(raw[106], 90)

    def test_abs_frame_reproduces_the_rotation_bug(self):
        """老算法：手指一动不动，手臂滚 60°，读数涨了 3 节 × 60°。"""
        flat = glove.chain_angles(groups_of(make_values(20)), "right", frame="abs")
        rolled = glove.chain_angles(groups_of(make_values(20, roll_deg=60)),
                                    "right", frame="abs")
        self.assertAlmostEqual(rolled[103] - flat[103], 180)
        self.assertAlmostEqual(rolled[102] - flat[102], 60)      # 拇指根只剩 1 节

    def test_rel_cancels_arm_roll(self):
        """默认 rel：手臂滚多少度，六指读数都不许变（拇指也不许）。"""
        flat = glove.chain_angles(groups_of(make_values(20)), "right")
        for roll in (30, 90, 180, 300):
            rolled = glove.chain_angles(groups_of(make_values(20, roll_deg=roll)), "right")
            for motor in glove.FINGERS:
                self.assertAlmostEqual(rolled[motor], flat[motor], places=6)

    def test_rel_keeps_finger_flexion(self):
        raw = glove.chain_angles(groups_of(make_values(30)), "right")
        self.assertAlmostEqual(raw[102], 30)     # 拇指1：只剩 CMC 一节
        self.assertAlmostEqual(raw[103], 90)
        self.assertAlmostEqual(raw[107], 60)     # 拇指2：两节

    def test_rel_ext_also_cancels_roll(self):
        flat = glove.chain_angles(groups_of(make_values(20)), "right", frame="rel-ext")
        rolled = glove.chain_angles(groups_of(make_values(20, roll_deg=120)),
                                    "right", frame="rel-ext")
        for motor in glove.FINGERS:
            self.assertAlmostEqual(rolled[motor], flat[motor], places=6)

    def test_left_hand_reads_upper_half(self):
        values = make_values(40, side="left")
        raw = glove.chain_angles(groups_of(values), "left")
        self.assertAlmostEqual(raw[103], 120)
        self.assertAlmostEqual(glove.chain_angles(groups_of(values), "right")[103], 0)

    def test_bad_frame_rejected(self):
        with self.assertRaises(ValueError):
            glove.chain_angles(groups_of(make_values(0)), "right", frame="palm")

    def test_short_frame_does_not_raise(self):
        self.assertAlmostEqual(glove.chain_angles([], "right")[102], 0.0)
        self.assertAlmostEqual(glove.chain_angles([[]], "right")[103], 0.0)

    def test_flex_uses_lateral_euler(self):
        self.assertAlmostEqual(glove.flex([0, 0, 0, 90, 0, 12]), 12)
        self.assertAlmostEqual(glove.flex([0, 0, 0, 90, -25, 3]), 25)
        self.assertAlmostEqual(glove.flex([]), 0.0)


class ReceiverCase(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.clock = Clock()

        def apply(motors, speed, torque):
            self.calls.append((dict(motors), speed, torque))

        self.r = glove.GloveReceiver(apply=apply, clock=self.clock)

    def feed(self, flex_deg, dt=0.05, side="right", fn=None):
        self.clock.tick(dt)
        fn = fn if fn is not None else 1
        out = None
        for text in packets(fn, make_values(flex_deg, side)):
            self.clock.tick(0.001)
            out = self.r.feed_text(text, now=self.clock.t)
        return out


class TestConversion(ReceiverCase):
    def test_first_frame_is_zero(self):
        out = self.feed(40)
        self.assertIsNotNone(out)
        for value in out["motors"].values():
            self.assertAlmostEqual(value, 0.0, places=1)
        self.assertEqual(len(self.calls), 1)

    def test_baseline_subtracts_and_full_scale_maps_to_max(self):
        self.feed(30)                    # 摊平基线 = 90（食指三节 × 30）
        out = self.feed(180)             # 握满 540 - 90 = 450 → 450/150 = 3 倍行程
        self.assertAlmostEqual(out["raw"][103], 540)
        self.assertAlmostEqual(out["motors"][103], 180.0)   # 被夹到上限
        self.assertAlmostEqual(out["motors"][107], 45.0)    # 拇指2 上限 45
        self.assertGreater(out["motors"][104], 0)

    def test_gain_scales(self):
        self.feed(30)
        self.r.set_gain(0.5)
        out = self.feed(90)              # (270-90)*180/150*0.5 = 108
        self.assertAlmostEqual(out["motors"][103], 108.0, places=1)

    def test_never_negative(self):
        self.feed(120)                   # 基线很高
        out = self.feed(10)              # 手反而更平
        for value in out["motors"].values():
            self.assertGreaterEqual(value, 0.0)

    def test_rate_limit_caps_posts(self):
        self.r.rate = 10.0               # 100 ms 一次
        for _ in range(5):
            self.feed(60, dt=0.02)
        self.assertLessEqual(len(self.calls), 2)

    def test_code_mode_mutes_but_keeps_frames(self):
        self.feed(30)
        self.r.set_mode("code")
        before = len(self.calls)
        out = self.feed(180)
        self.assertEqual(len(self.calls), before)
        self.assertEqual(out["post"], None)
        self.assertEqual(self.r.frames, 2)
        self.r.set_mode("glove")
        self.feed(180, dt=0.2)
        self.assertEqual(len(self.calls), before + 1)

    def test_zero_resets_baseline(self):
        self.feed(30)
        self.feed(180)
        self.r.zero()
        out = self.feed(180)
        self.assertAlmostEqual(out["motors"][103], 0.0, places=1)

    def test_apply_error_does_not_kill_receiver(self):
        def boom(motors, speed, torque):
            raise RuntimeError("CAN 断了")
        self.r.apply_fn = boom
        self.feed(30)
        self.feed(180)
        self.assertEqual(self.r.errors, 2)
        self.assertIn("CAN 断了", self.r.last_error)
        self.assertEqual(self.r.frames, 2)

    def test_rejected_apply_is_visible(self):
        self.r.apply_fn = lambda m, s, t: {"ok": False, "error": "安全检查未通过"}
        out = self.feed(30)
        self.assertEqual(self.r.errors, 1)
        self.assertEqual(self.r.applied, 0)
        self.assertIn("安全检查", self.r.last_error)
        self.assertFalse(out["post"]["ok"])

    def test_fps_ignores_millisecond_bursts(self):
        for _ in range(3):
            self.feed(30, dt=0.001)
        self.assertEqual(self.r.fps, 0.0)


class TestThumbSplit(unittest.TestCase):
    """拇指根（102）只跟 CMC，指尖（107）只跟 MCP/IP——别再重叠。

    老映射 102=(1,2,3)、107=(2,3)：CMC 一动，102 走了 107 不动，看着就是"整根往外展"。
    """

    def test_thumb_base_moves_only_102(self):
        raw = glove.chain_angles(groups_of(frame_of({1: (0, 30, 0)})), "right")
        self.assertGreater(raw[102], 29)
        self.assertAlmostEqual(raw[107], 0.0, places=6)

    def test_thumb_tip_moves_only_107(self):
        raw = glove.chain_angles(groups_of(frame_of({2: (0, 20, 0), 3: (0, 20, 0)})), "right")
        self.assertAlmostEqual(raw[102], 0.0, places=6)
        self.assertGreater(raw[107], 39)


class TestAxes(ReceiverCase):
    """教一次屈曲轴：之后外展、绕骨头自转都不再被当成"弯手指"。"""

    def setUp(self):
        super().setUp()
        # 主机的临时目录被沙箱挡着，写到工作区的 .tmp 里
        self.tmp = ROOT / ".tmp" / f"axes-{id(self)}.json"
        self.tmp.unlink(missing_ok=True)
        self.r.axes_path = self.tmp

    def tearDown(self):
        self.tmp.unlink(missing_ok=True)

    def feed_j(self, joints, dt=0.05):
        self.clock.tick(dt)
        return self.r.process_frame(frame_of(joints), now=self.clock.t)

    def teach_flexion(self):
        """只做屈曲（绕 Y），不带外展（绕 X）。"""
        self.r.teach_start()
        for deg in range(5, 46, 5):
            self.feed_j({1: (0, deg, 0), 4: (0, deg, 0), 5: (0, deg, 0), 6: (0, deg, 0)})
        return self.r.teach_stop()

    def test_teach_learns_axis_and_saves(self):
        out = self.teach_flexion()
        self.assertTrue(out["ok"])
        self.assertTrue(self.tmp.exists())
        axis = self.r.axes[1]
        self.assertAlmostEqual(abs(axis[1]), 1.0, places=3)
        self.assertAlmostEqual(self.r.state()["axes"], len(self.r.axes))

    def test_abduction_is_not_flexion(self):
        self.teach_flexion()
        out = self.feed_j({1: (35, 0, 0)})        # 纯外展：绕 X
        self.assertAlmostEqual(out["raw"][102], 0.0, places=1)

    def test_flexion_is_signed(self):
        self.teach_flexion()
        self.assertGreater(self.feed_j({1: (0, 40, 0)})["raw"][102], 30)
        self.assertLess(self.feed_j({1: (0, -40, 0)})["raw"][102], -30)

    def test_axes_reload_from_disk(self):
        self.teach_flexion()
        other = glove.GloveReceiver(clock=self.clock)
        other.axes_path = self.tmp
        self.assertIn(1, other._load_axes())

    def test_too_short_teach_is_rejected(self):
        self.r.teach_start()
        out = self.r.teach_stop()
        self.assertFalse(out["ok"])
        self.assertEqual(self.r.axes, {})


class TestBaseline(ReceiverCase):
    """实时任务里最危险的不是抖，是"零点被一次下冲带走"。"""

    def feed_span(self, flex_deg, dt=0.05):
        self.clock.tick(dt)
        return self.r.process_frame(make_values(flex_deg), now=self.clock.t)

    def test_single_dip_does_not_rezero(self):
        for _ in range(4):
            self.feed_span(30)
        self.feed_span(20)                        # 单帧下冲 10°
        out = self.feed_span(30)                  # 马上回到摊平
        for value in out["motors"].values():
            self.assertAlmostEqual(value, 0.0, places=1)   # 老算法这里会跳出 36°

    def test_sustained_dip_is_accepted(self):
        for _ in range(4):
            self.feed_span(30)
        for _ in range(8):                        # 连续 0.4 s 真的更平了
            self.feed_span(20)
        out = self.feed_span(30)
        self.assertGreater(out["motors"][103], 20)   # 40° 跨度 × 180/150

    def test_floor_rises_slowly_when_held(self):
        for _ in range(4):
            self.feed_span(30)
        for _ in range(200):                      # 贴着基线 10 s
            self.feed_span(30, dt=0.05)
        floor = self.r._baseline(103)
        self.assertLess(floor - 90.0, 15.0)       # 只允许慢慢追，不会窜到 90 以上很多

    def test_spike_is_counted(self):
        self.feed_span(20)
        self.feed_span(20)
        self.clock.tick(0.05)
        self.r.process_frame(frame_of({4: (0, 60, 0)}), now=self.clock.t)   # 一帧尖峰
        self.feed_span(20)                                                # 立刻弹回
        self.assertEqual(self.r.spikes, 1)
        self.assertIn("spikes", self.r.state())

    def test_fast_closing_is_not_a_spike(self):
        for deg in (0, 40, 80, 120):                                      # 单向快速握拳
            self.clock.tick(0.05)
            self.r.process_frame(frame_of({4: (0, deg, 0)}), now=self.clock.t)
        self.assertEqual(self.r.spikes, 0)


class TestArmRoll(ReceiverCase):
    """现场那个毛病：手摆着不动、胳膊转一圈，拇指（和整只手）自己动。"""

    def feed_frame(self, **kwargs):
        self.clock.tick(0.05)
        return self.r.process_frame(make_values(**kwargs), now=self.clock.t)

    def test_thumb_holds_still_while_arm_rolls(self):
        self.feed_frame(flex_deg=20)                     # 摊平基线
        for roll in (45, 120, 200, 300, 360):
            out = self.feed_frame(flex_deg=20, roll_deg=roll)
            for motor, value in out["motors"].items():
                self.assertAlmostEqual(value, 0.0, places=1)

    def test_abs_frame_still_shows_the_bug(self):
        """切回老算法必须还能复现，否则说明这个测试根本没在测那件事。"""
        self.r.set_frame("abs")
        self.feed_frame(flex_deg=20)
        out = self.feed_frame(flex_deg=20, roll_deg=120)
        self.assertGreater(out["motors"][102], 50)
        self.assertGreater(out["motors"][103], 50)

    def test_set_frame_resets_baseline(self):
        self.feed_frame(flex_deg=20)
        self.r.set_frame("abs")
        self.assertFalse(self.r.baseline_ready())
        self.assertEqual(self.r.state()["frame"], "abs")

    def test_frame_survives_in_state(self):
        self.r.set_frame("rel-ext")
        self.assertEqual(self.r.state()["frame"], "rel-ext")


class TestDeadband(ReceiverCase):
    def test_deadband_trims_thumb_wobble(self):
        self.r.set_deadband({102: 2})
        self.clock.tick(0.05)
        self.r.process_frame(make_values(20), now=self.clock.t)   # 基线：102=20、103=60
        self.clock.tick(0.05)
        out = self.r.process_frame(make_values(25), now=self.clock.t)
        # 102 只剩 CMC 一节：25 - 20 = 5，抹掉 2 死区剩 3 → 3 × 100/150 = 2.0
        self.assertAlmostEqual(out["motors"][102], 2.0, places=1)
        # 没设死区的指头不受影响：15 × 180/150 = 18
        self.assertAlmostEqual(out["motors"][103], 18.0, places=1)

    def test_zero_deadband_is_dropped(self):
        self.r.set_deadband({102: 0, 107: 5})
        self.assertEqual(self.r.deadband, {107: 5})


class TestValidation(unittest.TestCase):
    def setUp(self):
        self.r = glove.GloveReceiver()

    def test_bad_side(self):
        with self.assertRaises(ValueError):
            self.r.start(side="middle")

    def test_bad_port(self):
        with self.assertRaises(ValueError):
            self.r.start(udp=70000)

    def test_bad_gain(self):
        with self.assertRaises(ValueError):
            self.r.start(gain=9)

    def test_bad_mode(self):
        with self.assertRaises(ValueError):
            self.r.set_mode("teleop")

    def test_bad_rate(self):
        with self.assertRaises(ValueError):
            self.r.start(rate=0)

    def test_bad_frame(self):
        with self.assertRaises(ValueError):
            self.r.start(frame="palm")
        with self.assertRaises(ValueError):
            self.r.set_frame("palm")

    def test_bad_deadband(self):
        with self.assertRaises(ValueError):
            self.r.start(deadband={102: 90})
        with self.assertRaises(ValueError):
            self.r.start(deadband={201: 5})

    def test_default_frame_is_rel(self):
        self.assertEqual(self.r.state()["frame"], "rel")

    def test_toggle(self):
        self.assertEqual(self.r.toggle_mode()["mode"], "code")
        self.assertEqual(self.r.toggle_mode()["mode"], "glove")

    def test_stop_without_start(self):
        out = self.r.stop()
        self.assertFalse(out["running"])


class TestSocket(unittest.TestCase):
    def test_bind_failure_reports_hint(self):
        r = glove.GloveReceiver()
        out = r.start(udp=free_port(), bind="192.0.2.1")
        self.assertFalse(out["ok"])
        self.assertIn("绑定失败", out["error"])
        self.assertTrue(out["hint"])

    def test_successful_start_clears_start_error(self):
        r = glove.GloveReceiver()
        r.start_error = "上次绑失败"
        out = r.start(udp=free_port())
        try:
            self.assertTrue(out["ok"])
            self.assertEqual(r.state()["start_error"], "")
        finally:
            r.stop()

    def test_port_taken_by_another_receiver(self):
        """同一端口只能有一个监听者：老桥接还占着 5001 时必须明确报错，不能静默抢包。"""
        first = glove.GloveReceiver()
        port = free_port()
        self.assertTrue(first.start(udp=port)["ok"])
        try:
            second = glove.GloveReceiver()
            out = second.start(udp=port)
            self.assertFalse(out["ok"])
            self.assertIn("绑定失败", out["error"])
            state = second.state()
            self.assertIn("绑定失败", state["start_error"])
            self.assertIn("绑定失败", state["hint"])
        finally:
            first.stop()

    def test_end_to_end_udp(self):
        applied = []
        r = glove.GloveReceiver(apply=lambda m, s, t: applied.append(dict(m)))
        port = free_port()
        out = r.start(udp=port, side="right", rate=100.0)
        self.assertTrue(out["ok"], out)
        self.assertTrue(out["running"])
        try:
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            values = make_values(120)
            for _ in range(20):
                for text in packets(5, values):
                    tx.sendto(text.encode(), ("127.0.0.1", port))
                time.sleep(0.02)
            deadline = time.time() + 3.0
            while time.time() < deadline and r.frames < 2:
                time.sleep(0.05)
            tx.close()
            self.assertGreaterEqual(r.frames, 2)
            state = r.state()
            self.assertTrue(state["live"])
            self.assertEqual(state["mode"], "glove")
            self.assertTrue(applied)
        finally:
            r.stop()
        self.assertFalse(r.state()["running"])
        self.assertFalse(r.state()["live"])


class TestStateShape(unittest.TestCase):
    def test_state_keys(self):
        r = glove.GloveReceiver()
        state = r.state()
        for key in ("running", "mode", "udp", "side", "gain", "frames", "live",
                    "frame_age_s", "baseline_ready", "motors", "raw", "hint"):
            self.assertIn(key, state)
        self.assertFalse(state["running"])
        self.assertIsNone(state["frame_age_s"])
        self.assertIn("启动监听", state["hint"])

    def test_summary_is_small(self):
        r = glove.GloveReceiver()
        summary = r.summary()
        self.assertTrue(set(summary) <= {"running", "mode", "live", "fps", "frames",
                                          "udp", "side", "gain", "last_error"})
        self.assertIn("frames", summary)

    def test_finger_table(self):
        rows = glove.finger_table()
        self.assertEqual([row["motor_id"] for row in rows],
                         [102, 107, 103, 104, 105, 106])
        self.assertEqual(rows[1]["max"], 45.0)
        self.assertEqual(rows[2]["chain"], [4, 5, 6])


class TestServiceEndpoints(unittest.TestCase):
    """服务层接口：页面/脚本看到的就是这些。"""

    @classmethod
    def setUpClass(cls):
        cls.app = service.App(bus.open_sim(), root=ROOT)
        cls.srv = service.make_server(cls.app, "127.0.0.1", 0)
        cls.base = f"http://127.0.0.1:{cls.srv.server_address[1]}"
        threading.Thread(target=cls.srv.serve_forever, name="test-http", daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.app.glove.stop()
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        self.app.glove.stop()
        self.app.glove.set_mode("glove")
        self.app.glove.frames = 0                # 计数和帧历史别跨用例
        self.app.glove._motor_hist.clear()
        self.app.glove.set_frame("rel")          # 别把上一个用例的换算方式带过来
        self.app.glove.set_deadband({})
        axes = ROOT / ".tmp" / "svc-axes.json"   # 别把测试学到的轴写进真配置
        axes.unlink(missing_ok=True)
        self.app.glove.axes_path = axes

    def get(self, path):
        with urllib.request.urlopen(self.base + path, timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body=None):
        req = urllib.request.Request(self.base + path, data=json.dumps(body or {}).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read())

    def test_state_shape(self):
        j = self.get("/glove/state")
        self.assertTrue(j["ok"])
        self.assertFalse(j["running"])
        self.assertEqual(j["udp"], self.app.glove.udp)
        self.assertEqual(j["side"], "right")

    def test_start_feed_and_stop(self):
        port = free_port()
        code, started = self.post("/glove/start", {"udp": port, "rate": 100})
        self.assertEqual(code, 200)
        self.assertTrue(started["running"])
        self.assertEqual(started["udp"], port)

        calmer = make_values(30)
        self.app.glove.process_frame(calmer)
        grip = make_values(180)
        self.app.glove.process_frame(grip)

        j = self.get("/glove/state")
        self.assertEqual(j["frames"], 2)
        self.assertEqual(j["motors"]["103"], 180.0)
        self.assertGreater(j["applied"], 0)

        code, stopped = self.post("/glove/stop")
        self.assertEqual(code, 200)
        self.assertFalse(stopped["running"])

    def test_mode_and_gain_endpoints(self):
        code, j = self.post("/glove/mode", {"mode": "code"})
        self.assertEqual((code, j["mode"]), (200, "code"))
        code, j = self.post("/glove/gain", {"gain": 1.3})
        self.assertEqual((code, j["gain"]), (200, 1.3))
        code, j = self.post("/glove/mode", {"mode": "nope"})
        self.assertEqual(code, 400)
        code, j = self.post("/glove/gain", {"gain": 9})
        self.assertEqual(code, 400)
        code, j = self.post("/glove/toggle")
        self.assertEqual((code, j["mode"]), (200, "glove"))

    def test_start_rejects_bad_params(self):
        code, j = self.post("/glove/start", {"udp": 70000})
        self.assertEqual(code, 400)
        self.assertIn("参数不对", j["error"])

    def test_frame_and_deadband_endpoints(self):
        code, j = self.post("/glove/frame", {"frame": "abs"})
        self.assertEqual((code, j["frame"]), (200, "abs"))
        code, _j = self.post("/glove/frame", {"frame": "palm"})
        self.assertEqual(code, 400)

        code, j = self.post("/glove/deadband", {"deadband": {"102": 6}})
        self.assertEqual((code, j["deadband"]), (200, {"102": 6.0}))
        code, _j = self.post("/glove/deadband", {"deadband": {"102": 99}})
        self.assertEqual(code, 400)

        j = self.get("/glove/state")
        self.assertEqual(j["frame"], "abs")
        self.assertEqual(j["deadband"], {"102": 6.0})

    def test_glove_save_point_and_nudge(self):
        points = ROOT / ".tmp" / "svc-points"
        points.mkdir(parents=True, exist_ok=True)
        for old_file in points.glob("zz-*.txt"):
            old_file.unlink()
        old = self.app.dirs.points
        self.app.dirs.points = points
        try:
            code, _j = self.post("/glove/save", {"name": "zz-pen"})
            self.assertEqual(code, 409)                  # 没在跟手
            self.post("/glove/start", {"udp": free_port(), "rate": 100, "dry_run": True})
            for _ in range(4):
                self.app.glove.process_frame(make_values(20))
            for _ in range(6):
                self.app.glove.process_frame(make_values(40))
            code, j = self.post("/glove/save", {"name": "zz-pen"})
            self.assertEqual(code, 200, j)
            self.assertTrue(Path(j["path"]).is_file())
            self.assertEqual(j["saturated"], [])
            self.assertEqual(j["motors"]["103"], 72.0)   # 三节 120 - 基线 60 → 60 × 180/150
            code, _j = self.post("/glove/save", {"name": "zz-pen"})
            self.assertEqual(code, 409)                  # 同名不覆盖
            code, j = self.post("/glove/save", {"name": "zz-pen", "overwrite": True})
            self.assertEqual(code, 200, j)

            code, j = self.post("/point/nudge", {"name": "zz-pen", "delta": {"103": -6},
                                                 "apply": False})
            self.assertEqual(code, 200, j)
            self.assertAlmostEqual(j["motors"]["103"], 66.0)     # 72 − 6
            code, _j = self.post("/point/nudge", {"name": "zz-nope", "delta": {"103": 1}})
            self.assertEqual(code, 404)
            code, _j = self.post("/point/nudge", {"name": "zz-pen", "delta": {"999": 1}})
            self.assertEqual(code, 400)
            self.post("/glove/stop")
        finally:
            self.app.dirs.points = old

    def test_teach_endpoints(self):
        code, j = self.post("/glove/teach", {"action": "start"})
        self.assertEqual((code, j["teaching"]), (200, True))
        code, j = self.post("/glove/teach", {"action": "stop"})
        self.assertEqual(code, 200)
        self.assertIn("axes", j)
        code, j = self.post("/glove/teach", {"action": "status"})
        self.assertEqual(code, 200)
        self.assertIn("axes", j)
        code, _j = self.post("/glove/teach", {"action": "nope"})
        self.assertEqual(code, 400)

    def test_health_has_glove(self):
        self.assertIn("glove", self.get("/health"))
        self.assertEqual(len(self.get("/glove/limits")["fingers"]), 6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
