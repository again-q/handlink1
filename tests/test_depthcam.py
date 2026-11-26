"""深度测量测试：合成一张「桌面 + 盒子」的深度图，量出来的长宽高要和真值对得上。

    python tests/test_depthcam.py
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import depthcam  # noqa: E402


def synthetic_scene(box_w=0.08, box_d=0.06, box_h=0.06, plane_z=1.0,
                    width=848, height=480, fx=600.0, fy=600.0):
    """桌面在 z=plane_z，盒子的顶面在 z=plane_z−box_h，正对相机。"""
    depth = np.full((height, width), int(plane_z * 1000), dtype=np.uint16)
    cx, cy = width / 2.0, height / 2.0
    top_z = plane_z - box_h
    half_w_px = int(round(box_w / 2.0 * fx / top_z))
    half_d_px = int(round(box_d / 2.0 * fy / top_z))
    depth[int(cy) - half_d_px:int(cy) + half_d_px,
          int(cx) - half_w_px:int(cx) + half_w_px] = int(top_z * 1000)
    intr = {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "width": width, "height": height}
    return depth, intr


class TestDepthPoints(unittest.TestCase):
    def test_center_pixel_is_on_axis(self):
        depth, intr = synthetic_scene()
        pts = depthcam.depth_points(depth, intr, step=4)
        # 桌面的点占绝大多数，z 的中位数应该就是桌面距离
        self.assertAlmostEqual(float(np.median(pts[:, 2])), 1.0, places=2)

    def test_scale_grows_with_distance(self):
        depth, intr = synthetic_scene()
        pts = depthcam.depth_points(depth, intr, step=4)
        self.assertGreater(pts[:, 2].min(), 0.0)


class TestPlaneFit(unittest.TestCase):
    def test_fit_flat_plane(self):
        depth, intr = synthetic_scene(box_h=0.0)
        pts = depthcam.depth_points(depth, intr, step=4)
        plane = depthcam.fit_plane(pts)
        self.assertIsNotNone(plane)
        self.assertGreater(plane["ratio"], 0.9)
        self.assertAlmostEqual(abs(plane["d"]), 1.0, places=2)


class TestMeasure(unittest.TestCase):
    def test_box_size(self):
        depth, intr = synthetic_scene(box_w=0.08, box_d=0.06, box_h=0.06)
        res = depthcam.measure_objects(depth, intr)
        self.assertTrue(res["ok"])
        self.assertEqual(len(res["objects"]), 1, res)
        o = res["objects"][0]
        L, D, H = o["size"]
        # 平面内的两条边哪条算「长」取决于基向量方向，排序后比
        a, b = sorted((L, D))
        self.assertAlmostEqual(a, 0.06, delta=0.005)
        self.assertAlmostEqual(b, 0.08, delta=0.005)
        self.assertAlmostEqual(H, 0.06, delta=0.005)
        self.assertEqual(o["label"], "盒子")

    def test_empty_table(self):
        depth, intr = synthetic_scene(box_h=0.0)
        res = depthcam.measure_objects(depth, intr)
        self.assertTrue(res["ok"])
        self.assertEqual(res["objects"], [])

    def test_too_few_points(self):
        depth = np.zeros((480, 848), dtype=np.uint16)
        intr = {"fx": 600, "fy": 600, "cx": 424, "cy": 240}
        res = depthcam.measure_objects(depth, intr)
        self.assertFalse(res["ok"])


class TestHint(unittest.TestCase):
    def test_hint_when_empty(self):
        cam = depthcam.DepthCamera()
        depth = np.zeros((480, 848), dtype=np.uint16)
        self.assertIn("有效深度", cam.hint(depth))


if __name__ == "__main__":
    unittest.main(verbosity=2)
