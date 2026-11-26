"""立方体位置检测的离线测试：只认「盒子在哪」，不读 9 格。

    python tests/test_boxcv.py

全部合成图：蓝桌面 + 白纸 + 一块绿色立方体。钉住的是
「蓝色桌面不算物体」「底边接触点 → 底面中心」和「未标定不硬算」。
"""
from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import boxcv, scene  # noqa: E402
from handlink.boxcv import cv2, np  # noqa: E402


def table_frame(cube: bool = True, color=(60, 180, 60)) -> "np.ndarray":
    """640×480 蓝桌面 + 白纸 + 可选绿色立方体。BGR。"""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:, :] = (200, 120, 60)                         # 蓝色桌面
    cv2.rectangle(img, (120, 80), (520, 400), (245, 245, 245), -1)   # 白纸
    if cube:
        x0, y0, x1, y1 = 240, 140, 340, 250
        cv2.rectangle(img, (x0, y0), (x1, y1), color, -1)
        for i in range(1, 3):
            x = x0 + (x1 - x0) * i // 3
            y = y0 + (y1 - y0) * i // 3
            cv2.line(img, (x, y0), (x, y1), (20, 20, 20), 2)
            cv2.line(img, (x0, y), (x1, y), (20, 20, 20), 2)
    return img


class TestFindBox(unittest.TestCase):
    def test_blank_table_has_no_box(self):
        det = boxcv.find_box(table_frame(cube=False))
        self.assertFalse(det["found"])

    def test_green_cube_is_found(self):
        det = boxcv.find_box(table_frame(cube=True))
        self.assertTrue(det["found"])
        x, y, w, h = det["bbox"]
        self.assertLessEqual(abs(x - 240), 2)
        self.assertLessEqual(abs(y - 140), 2)
        self.assertLessEqual(abs(w - 100), 3)
        self.assertLessEqual(abs(h - 110), 3)
        self.assertGreater(det["fill"], 0.9)
        # 底边接触点在物体下沿，u 在物体中线附近。
        self.assertLessEqual(abs(det["contact"][1] - 250), 2)
        self.assertLessEqual(abs(det["contact"][0] - 290), 4)

    def test_blue_cube_is_not_a_box(self):
        det = boxcv.find_box(table_frame(cube=True, color=(200, 80, 80)))
        self.assertFalse(det["found"])

    def test_roi_limits_search(self):
        frame = table_frame(cube=True)
        self.assertFalse(boxcv.find_box(frame, roi=(0, 0, 160, 120))["found"])
        self.assertTrue(boxcv.find_box(frame, roi=(200, 100, 200, 200))["found"])

    def test_annotate_keeps_shape(self):
        frame = table_frame(cube=True)
        det = boxcv.find_box(frame)
        out = boxcv.annotate(frame, det)
        self.assertEqual(out.shape, frame.shape)


class TestLocate(unittest.TestCase):
    def setUp(self):
        # 不用 TemporaryDirectory：Windows 沙箱里它 cleanup 时会被 ACL 拒绝。
        self.tmpdir = Path(tempfile.mkdtemp(prefix="test-boxcv-", dir=ROOT / ".tmp"))
        self.sc = scene.Scene(path=self.tmpdir / "scene.json")
        # 线性虚拟相机：x = u/1000 - 0.32，y = -v/1000 + 0.24（米）
        self.sc.H = [[0.001, 0.0, -0.32], [0.0, -0.001, 0.24], [0.0, 0.0, 1.0]]
        self.sc.Hinv = scene.invert_h(self.sc.H)
        self.sc.calib = {"updated": "test"}
        self.det = {"found": True, "bbox": [240, 140, 100, 110],
                    "center_px": [290.0, 195.0], "contact": [290.0, 250.0]}

    def tearDown(self):
        pass

    def test_contact_maps_to_table(self):
        loc = boxcv.locate(self.sc, self.det, size_m=0.057)
        self.assertTrue(loc["ok"])
        # 接触点 (290,250) → (-0.030, -0.010) m。
        self.assertAlmostEqual(loc["contact_mm"][0], -30.0, places=1)
        self.assertAlmostEqual(loc["contact_mm"][1], -10.0, places=1)

    def test_center_moves_half_side_away_from_camera(self):
        loc = boxcv.locate(self.sc, self.det, size_m=0.057)
        self.assertTrue(loc["ok"])
        # 画面「上」是 +y，所以底面中心在接触点上方 28.5 mm。
        self.assertAlmostEqual(loc["center_mm"][0], -30.0, places=1)
        self.assertAlmostEqual(loc["center_mm"][1], 18.5, places=1)

    def test_toward_down_reverses(self):
        loc = boxcv.locate(self.sc, self.det, size_m=0.057, toward="down")
        self.assertTrue(loc["ok"])
        self.assertAlmostEqual(loc["center_mm"][1], -38.5, places=1)

    def test_yaw_is_folded_to_cube_symmetry(self):
        loc = boxcv.locate(self.sc, self.det, size_m=0.057)
        self.assertGreaterEqual(loc["yaw"], -45.0)
        self.assertLess(loc["yaw"], 45.0)

    def test_uncalibrated_scene_refuses(self):
        sc = scene.Scene(path=self.tmpdir / "empty.json")
        loc = boxcv.locate(sc, self.det)
        self.assertFalse(loc["ok"])
        self.assertIn("标定", loc["error"])


if __name__ == "__main__":
    unittest.main()
