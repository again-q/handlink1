"""协议层单元测试：帧构造、大小端、解析、钳位。不需要真机。

    python tests/test_screw.py
"""
from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import screw  # noqa: E402


class TestFrames(unittest.TestCase):
    def test_position_frame_layout(self):
        frame = screw.set_position(103, 30.0)
        self.assertEqual(frame[0], 0xA2)
        self.assertEqual(frame[1], 103)
        self.assertEqual(struct.unpack("<f", frame[2:6])[0], 30.0)
        self.assertEqual(len(frame), 6)

    def test_float_is_little_endian(self):
        frame = screw.set_position(104, 1.0)
        # 1.0f 大端是 3F 80 00 00；小端是 00 00 80 3F
        self.assertEqual(frame[2:6], b"\x00\x00\x80\x3f")

    def test_speed_and_current_frames(self):
        self.assertEqual(screw.set_speed(104, 200.0)[:2], bytes([0xE0, 104]))
        self.assertEqual(screw.set_current(104, 1.0)[:2], bytes([0xD0, 104]))
        self.assertEqual(struct.unpack("<f", screw.set_current(104, 1.0)[2:6])[0], 1.0)

    def test_query_and_toggle_frames(self):
        self.assertEqual(screw.get_position(103), bytes([0xA4, 103]))
        self.assertEqual(screw.get_speed(103), bytes([0xA6, 103]))
        self.assertEqual(screw.get_current(103), bytes([0xA8, 103]))
        self.assertEqual(screw.enable(103, True), bytes([0xF2, 103, 1]))
        self.assertEqual(screw.enable(103, False), bytes([0xF2, 103, 0]))
        self.assertEqual(screw.clear_error(103), bytes([0x0E, 103]))


class TestParse(unittest.TestCase):
    def test_response_with_value(self):
        raw = bytes([0xA5, 103, 0x06]) + struct.pack("<f", 29.6)
        frame = screw.parse(raw)
        self.assertIsInstance(frame, screw.Frame)
        self.assertEqual(frame.node, 103)
        self.assertEqual(frame.seq, 0x06)
        self.assertAlmostEqual(frame.value, 29.6, places=4)

    def test_ack_without_value(self):
        frame = screw.parse(bytes([0xF3, 103, 0x0C]))
        self.assertIsInstance(frame, screw.Frame)
        self.assertEqual(frame.node, 103)
        self.assertIsNone(frame.value)

    def test_heartbeat(self):
        hb = screw.parse(bytes([0x05, 0x67, 0x6D, 0x08, 0x01]))
        self.assertIsInstance(hb, screw.Heartbeat)
        self.assertEqual(hb.node, 103)
        self.assertEqual(hb.calib, 0x08)
        self.assertEqual(hb.running, 0x01)
        self.assertIn("running", hb.state)

    def test_heartbeat_error_bits(self):
        hb = screw.parse(bytes([0x05, 0x67, 0x6D, 0x08, 0x11]))
        self.assertIn("I-LIM", hb.state)


class TestClamp(unittest.TestCase):
    def test_angle_clamp(self):
        self.assertEqual(screw.clamp_angle(107, 90), 45.0)
        self.assertEqual(screw.clamp_angle(102, -5), 0.0)
        self.assertEqual(screw.clamp_angle(104, 200), 180.0)

    def test_current_clamp(self):
        self.assertEqual(screw.clamp_current(102, 3.0), 0.8)
        self.assertEqual(screw.clamp_current(104, 10.0), 3.5)
        self.assertEqual(screw.clamp_current(104, -1.0), 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
