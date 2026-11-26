"""摄像头 / 识别自测：没有摄像头、没有 opencv 也要全绿。"""
from __future__ import annotations

import unittest
import uuid
from pathlib import Path

from handlink import camera, vision

ROOT = Path(__file__).resolve().parent.parent


class TestConfig(unittest.TestCase):
    def test_defaults(self):
        cfg = camera.CameraConfig()
        self.assertEqual(cfg.index, 0)
        self.assertEqual(cfg.backend, "any")
        self.assertEqual(cfg.rotate, 0)

    def test_normalise(self):
        cfg = camera.CameraConfig(index=-5, backend="uvc", rotate=450, width=-1, height=0)
        self.assertEqual(cfg.index, 0)
        self.assertEqual(cfg.backend, "any")
        self.assertEqual(cfg.rotate, 90)
        self.assertEqual((cfg.width, cfg.height), (0, 0))

    def test_roundtrip(self):
        cfg = camera.CameraConfig(index=2, backend="msmf", width=1280, height=800,
                                  rotate=270, mirror=True)
        again = camera.CameraConfig.from_dict(cfg.as_dict())
        self.assertEqual(again.as_dict(), cfg.as_dict())


class TestCameraOffline(unittest.TestCase):
    def test_state_when_closed(self):
        cam = camera.Camera(camera.CameraConfig(index=1, backend="msmf"))
        state = cam.state()
        self.assertTrue(state["ok"])
        self.assertFalse(state["running"])
        self.assertEqual(state["frames"], 0)
        self.assertEqual(state["config"]["index"], 1)
        self.assertEqual(state["recognizer"]["name"], "none")
        cam.close()               # 关一个没开的摄像头不能炸

    def test_open_without_opencv(self):
        cam = camera.Camera()
        had = camera.HAS_CV2
        camera.HAS_CV2 = False
        try:
            with self.assertRaises(RuntimeError) as ctx:
                cam.open()
            self.assertIn("opencv", str(ctx.exception))
        finally:
            camera.HAS_CV2 = had
            cam.close()

    def test_wait_frame_without_stream(self):
        cam = camera.Camera()
        seq, jpg = cam.wait_frame(-1, timeout=0.05)
        self.assertEqual(jpg, None)
        self.assertEqual(seq, 0)
        self.assertIsNone(cam.snapshot_png())

    def test_recogniser_requires_camera(self):
        cam = camera.Camera()
        result = cam.enable_recognition(True)
        # 没有 mediapipe 时明确说不可用，而不是抛异常
        if result.get("ok"):
            self.assertTrue(result["recognizer"]["available"])
        else:
            self.assertIn("error", result)
        cam.enable_recognition(False)
        self.assertIsNone(cam.recognizer)


class TestDevices(unittest.TestCase):
    def test_list_devices_shape(self):
        devices = camera.list_devices()
        self.assertIsInstance(devices, list)
        for dev in devices:
            self.assertIn("name", dev)
            self.assertIn("present", dev)
            self.assertIsInstance(dev["present"], bool)

    def test_probe_without_opencv_returns_empty(self):
        had = camera.HAS_CV2
        camera.HAS_CV2 = False
        try:
            self.assertEqual(camera.probe_devices(force=True), [])
        finally:
            camera.HAS_CV2 = had


class TestVision(unittest.TestCase):
    def test_null_recogniser(self):
        rec = vision.NullRecognizer("缺依赖")
        info = rec.info()
        self.assertFalse(info["available"])
        self.assertEqual(rec.detect(object()), {})
        frame = object()
        self.assertIs(rec.draw(frame, {}), frame)

    @staticmethod
    def _hand(theta_deg: float):
        """造 21 个关键点：每根手指（和拇指）在中间关节处的夹角都是 theta。"""
        import math

        dx, dy = math.sin(math.radians(theta_deg)), math.cos(math.radians(theta_deg))
        lm = [(0.5, 0.5)] * 21
        for a, b, c in list(vision.FINGERS.values()) + [vision.THUMB]:
            lm[b] = (0.5, 0.5)
            lm[a] = (0.5, 0.6)                    # b 正下方 0.1
            lm[c] = (0.5 + 0.1 * dx, 0.5 + 0.1 * dy)
        return lm

    def test_flex_straight_fingers(self):
        flex = vision.flex_from_landmarks(self._hand(180.0))
        for mid in (103, 104, 105, 106, 102, 107):
            self.assertGreaterEqual(flex[mid], 0.0)
            self.assertLessEqual(flex[mid], 1.0)
            self.assertLess(flex[mid], 0.05)

    def test_flex_bent_fingers(self):
        flex = vision.flex_from_landmarks(self._hand(30.0))
        for mid in (103, 104, 105, 106, 102, 107):
            self.assertGreater(flex[mid], 0.85)

    def test_angle_at(self):
        self.assertAlmostEqual(vision.angle_at((0, 1), (0, 0), (0, -1)), 180.0, places=3)
        self.assertAlmostEqual(vision.angle_at((1, 0), (0, 0), (0, 1)), 90.0, places=3)
        self.assertEqual(vision.angle_at((0, 0), (0, 0), (1, 1)), 180.0)


if __name__ == "__main__":
    unittest.main()
