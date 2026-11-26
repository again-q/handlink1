"""麦克风自测：没麦克风、没权限、非 Windows 都要全绿。"""
from __future__ import annotations

import unittest
import uuid
import wave
from pathlib import Path

from handlink import audio

# 沙箱里系统 TEMP 不可写，临时文件统一放工作区（跟 test_handlink 一致）
_TMP = Path(__file__).resolve().parent.parent / ".tmp"


def _mkdtemp() -> Path:
    """默认权限的临时目录：TemporaryDirectory 清理时会 chmod，沙箱里会被拒。"""
    _TMP.mkdir(parents=True, exist_ok=True)
    p = _TMP / f"hl-audio-{uuid.uuid4().hex[:10]}"
    p.mkdir(parents=True, exist_ok=True)
    return p


class TestLevels(unittest.TestCase):
    def test_silence(self):
        data = b"\x00\x00" * 800
        rms, peak = audio.block_rms(data)
        self.assertEqual(rms, 0.0)
        self.assertEqual(peak, 0.0)
        self.assertEqual(audio.to_db(0.0), -90.0)
        self.assertEqual(audio.to_meter(-90.0), 0.0)

    def test_full_scale(self):
        data = b"\xff\x7f" * 800                      # 全是 32767
        rms, peak = audio.block_rms(data)
        self.assertAlmostEqual(rms, 1.0, places=3)
        self.assertAlmostEqual(peak, 1.0, places=3)
        self.assertAlmostEqual(audio.to_db(1.0), 0.0, places=3)
        self.assertEqual(audio.to_meter(0.0), 1.0)

    def test_half_scale_is_minus_six_db(self):
        rms = 0.5
        self.assertAlmostEqual(audio.to_db(rms), -6.0, places=1)

    def test_partial_sample_is_ignored(self):
        rms, peak = audio.block_rms(b"\x00\x40\xff")   # 末尾一个孤立字节
        self.assertGreater(rms, 0.0)
        self.assertGreater(peak, 0.0)

    def test_meter_clamped(self):
        self.assertEqual(audio.to_meter(-200.0), 0.0)
        self.assertEqual(audio.to_meter(20.0), 1.0)


class TestRecorderOffline(unittest.TestCase):
    def test_idle_state(self):
        rec = audio.MicRecorder()
        state = rec.state()
        self.assertTrue(state["ok"])
        self.assertFalse(state["running"])
        self.assertEqual(state["seconds"], 0.0)
        self.assertEqual(state["level"], 0.0)

    def test_stop_when_idle(self):
        rec = audio.MicRecorder()
        result = rec.stop()
        self.assertFalse(result["ok"])
        self.assertIn("没有", result["error"])

    def test_bad_rate(self):
        rec = audio.MicRecorder()
        result = rec.start(rate=100)
        self.assertFalse(result["ok"])
        self.assertIn("rate", result["error"])

    def test_bad_channels(self):
        rec = audio.MicRecorder()
        result = rec.start(channels=7)
        self.assertFalse(result["ok"])
        self.assertIn("channels", result["error"])

    @unittest.skipUnless(audio.IS_WINDOWS, "winmm 只在 Windows 上有")
    def test_start_without_device(self):
        """设备号大到不存在：要返回错误，不能把异常漏出去。"""
        rec = audio.MicRecorder()
        out = _mkdtemp() / "x.wav"
        with self.assertRaises(RuntimeError) as ctx:
            rec.start(index=999, out=out)
        self.assertIn("录音设备", str(ctx.exception))
        self.assertFalse(rec.running)


class TestDevices(unittest.TestCase):
    def test_list_inputs_shape(self):
        devices = audio.list_inputs()
        self.assertIsInstance(devices, list)
        for dev in devices:
            self.assertIsInstance(dev.index, int)
            self.assertTrue(dev.name)
            self.assertIn("index", dev.as_dict())
            self.assertIn("name", dev.as_dict())


class TestClips(unittest.TestCase):
    def test_empty_folder(self):
        folder = _mkdtemp()
        self.assertEqual(audio.clips(folder), [])
        self.assertEqual(audio.clips(folder / "不存在"), [])

    def test_reads_duration(self):
        folder = _mkdtemp()
        path = folder / "clip.wav"
        with wave.open(str(path), "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(b"\x00\x00" * 8000)   # 0.5 s
        items = audio.clips(folder)
        self.assertEqual(len(items), 1)
        self.assertEqual(items[0]["name"], "clip.wav")
        self.assertAlmostEqual(items[0]["seconds"], 0.5, places=2)
        self.assertEqual(items[0]["rate"], 16000)


if __name__ == "__main__":
    unittest.main()
