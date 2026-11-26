"""30004 实时反馈解析的单元测试：字节位置、小端、对齐。不需要真机。

字段偏移来自《TCP/IP-API-V4.6.0》第 123~126 页的表，这里逐条钉住，
以后换固件/换型号时如果厂商改了布局，这几个测试会先炸。

    python tests/test_arm_feed.py
"""
from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import arm_feed  # noqa: E402


def build_frame(**over) -> bytes:
    """按手册造一包 1440 字节，默认值都合法。"""
    buf = bytearray(arm_feed.FRAME_SIZE)
    values = {
        "MessageSize": arm_feed.FRAME_SIZE,
        "RobotMode": 5,
        "TimeStamp": 1_800_000_000_000,
        "RunTime": 12345,
        "TestValue": arm_feed.TEST_VALUE,
        "SpeedScaling": 20.0,
        "VRobot": 48.2,
        "IRobot": 3.3,
        "QActual": [1.5, -2.5, 3.5, -4.5, 5.5, -6.5],
        "ToolVectorActual": [200.0, -100.0, 350.0, 180.0, 0.0, -45.0],
        "IActual": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "BrakeStatus": 0b111111,
        "EnableStatus": 1,
        "ErrorStatus": 0,
        "SafetyState": 0xFF,
        "SixForceOnline": 1,
        "Load": 0.51,
        "CurrentCommandId": 7,
        "ActualQuaternion": [1.0, 0.0, 0.0, 0.0],
    }
    values.update(over)
    for name, value in values.items():
        off, fmt, count = arm_feed._FIELDS[name]
        seq = value if isinstance(value, (list, tuple)) else [value]
        if fmt == "c":
            raw = (seq[0].encode() if isinstance(seq[0], str) else seq[0])[:count]
            buf[off:off + count] = raw.ljust(count, b"\x00")
        else:
            struct.pack_into("<" + fmt * count, buf, off, *seq)
    return bytes(buf)


class TestLayout(unittest.TestCase):
    def test_frame_size_and_magic(self):
        frame = arm_feed.parse_frame(build_frame())
        self.assertEqual(frame["MessageSize"], 1440)
        self.assertEqual(frame["TestValue"], 0x0123456789ABCDEF)
        self.assertEqual(len(build_frame()), arm_feed.FRAME_SIZE)

    def test_joint_and_pose_offsets(self):
        """QActual 在 432，ToolVectorActual 在 624 —— 手册表里的字节位置。"""
        raw = bytearray(arm_feed.FRAME_SIZE)
        struct.pack_into("<6d", raw, 432, 1, 2, 3, 4, 5, 6)
        struct.pack_into("<6d", raw, 624, 10, 20, 30, 40, 50, 60)
        struct.pack_into("<Q", raw, 48, arm_feed.TEST_VALUE)
        frame = arm_feed.parse_frame(bytes(raw))
        self.assertEqual(frame["QActual"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(frame["ToolVectorActual"], [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])

    def test_little_endian(self):
        """同样的数值按大端写进 64，读出来必须是错的 —— 钉住字节序。"""
        raw = bytearray(arm_feed.FRAME_SIZE)
        struct.pack_into(">d", raw, 64, 20.0)  # 故意大端
        struct.pack_into("<Q", raw, 48, arm_feed.TEST_VALUE)
        self.assertNotEqual(arm_feed.parse_frame(bytes(raw))["SpeedScaling"], 20.0)
        raw2 = bytearray(arm_feed.FRAME_SIZE)
        struct.pack_into("<d", raw2, 64, 20.0)
        struct.pack_into("<Q", raw2, 48, arm_feed.TEST_VALUE)
        self.assertEqual(arm_feed.parse_frame(bytes(raw2))["SpeedScaling"], 20.0)

    def test_short_packet_rejected(self):
        with self.assertRaises(arm_feed.FeedbackError):
            arm_feed.parse_frame(b"\x00" * 100)


class TestDescribe(unittest.TestCase):
    def test_summary_says_when_control_is_allowed(self):
        disabled = arm_feed.describe(
            arm_feed.parse_frame(build_frame(RobotMode=4, EnableStatus=0)))
        self.assertEqual(disabled["mode_name"], "下使能")
        self.assertTrue(disabled["can_take_control"])
        self.assertFalse(disabled["enabled"])

        running = arm_feed.describe(
            arm_feed.parse_frame(build_frame(RobotMode=7, EnableStatus=1)))
        self.assertEqual(running["mode_name"], "运行中")
        self.assertFalse(running["can_take_control"],
                         "运行中切 TCP 会被控制器拒绝，脚本要能提前拦住")

    def test_safety_bits_are_active_low(self):
        """SafetyState 的位是低有效：0xFF = 什么都没发生。"""
        quiet = arm_feed.describe(arm_feed.parse_frame(build_frame(SafetyState=0xFF)))
        self.assertFalse(any(quiet["safety"].values()))
        estop = arm_feed.describe(arm_feed.parse_frame(build_frame(SafetyState=0xFE)))
        self.assertTrue(estop["safety"]["estop"])


class TestCarve(unittest.TestCase):
    def test_aligns_when_stream_starts_mid_packet(self):
        good = build_frame()
        stream = b"\x00\x07garbage" + good + good
        frames, rest, skipped = arm_feed.carve(stream)
        self.assertEqual(len(frames), 2)
        self.assertEqual(rest, b"")
        self.assertGreater(skipped, 0, "接在包中间时要丢掉前面的碎片")

    def test_keeps_partial_packet(self):
        good = build_frame()
        frames, rest, _ = arm_feed.carve(good + good[:500])
        self.assertEqual(len(frames), 1)
        self.assertEqual(len(rest), 500)

    def test_rejects_wrong_magic(self):
        bad = bytearray(build_frame())
        struct.pack_into("<Q", bad, 48, 0)
        frames, _, skipped = arm_feed.carve(bytes(bad) * 2)
        self.assertEqual(frames, [])
        self.assertGreater(skipped, 0)


class TestModes(unittest.TestCase):
    def test_only_poweroff_and_disabled_can_take_control(self):
        self.assertEqual(arm_feed.CONTROLLABLE_MODES, {3, 4})
        for mode in arm_feed.ROBOT_MODES:
            described = arm_feed.describe(arm_feed.parse_frame(build_frame(RobotMode=mode)))
            self.assertEqual(described["can_take_control"], mode in {3, 4})


if __name__ == "__main__":
    unittest.main(verbosity=2)
