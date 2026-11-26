"""相机实时环境接入的几何测试：单应、反投影、TTL、AABB、cell 注入。

全部用合成数据：一组「世界桌角 → 画面梯形」的对应点就是一台斜视相机，
不需要真相机。这里钉住的是**坐标约定**（图像 y 向下、世界 y 向上）和
「视觉丢失按 TTL 保守保留」这两条，改实现时先看它们炸不炸。

    python tests/test_scene.py
"""
from __future__ import annotations

import json
import math
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import cell, scene  # noqa: E402

# 一台「斜着俯视桌面」的虚拟相机：世界桌角 → 画面里的梯形（左上/右上/右下/左下）。
WORLD_CORNERS = [[-0.70, 0.45], [0.70, 0.45], [0.70, -0.45], [-0.70, -0.45]]
PIXEL_CORNERS = [[118.0, 74.0], [531.0, 112.0], [606.0, 437.0], [52.0, 392.0]]


class TestHomography(unittest.TestCase):
    def setUp(self):
        self.H = scene.solve_homography(PIXEL_CORNERS, WORLD_CORNERS)

    def test_four_points_reproject_exactly(self):
        rp = scene.reprojection(self.H, PIXEL_CORNERS, WORLD_CORNERS)
        self.assertLess(rp["rms_mm"], 1e-6)
        self.assertLess(rp["max_mm"], 1e-6)

    def test_known_pixel_maps_to_expected_world(self):
        # 梯形两条对角线的交点应当落在世界原点附近（透视下不是正中间，但同阶）。
        x, y = scene.apply_h(self.H, 320.0, 240.0)
        self.assertLess(abs(x), 0.10)
        self.assertLess(abs(y), 0.10)

    def test_inverse_roundtrip(self):
        Hinv = scene.invert_h(self.H)
        for x, y in ((0.0, 0.0), (0.35, -0.2), (-0.6, 0.4)):
            u, v = scene.apply_h(Hinv, x, y)
            bx, by = scene.apply_h(self.H, u, v)
            self.assertAlmostEqual(bx, x, places=9)
            self.assertAlmostEqual(by, y, places=9)

    def test_more_points_least_squares(self):
        # 8 个点、加 0.8 px 噪声：残差应当明显小于「点错一格」的量级。
        extra_pix, extra_world = [], []
        for i, (p, w) in enumerate(zip(PIXEL_CORNERS, WORLD_CORNERS)):
            extra_pix += [p, [p[0] + 0.8, p[1] - 0.7]]
            extra_world += [w, w]
        H = scene.solve_homography(extra_pix, extra_world)
        rp = scene.reprojection(H, extra_pix, extra_world)
        self.assertLess(rp["rms_mm"], 5.0)

    def test_rejects_too_few_points(self):
        with self.assertRaises(ValueError):
            scene.solve_homography(PIXEL_CORNERS[:3], WORLD_CORNERS[:3])

    def test_rejects_collinear_points(self):
        pix = [[0, 0], [100, 0], [200, 0], [300, 0]]
        wld = [[0, 0], [0.1, 0], [0.2, 0], [0.3, 0]]
        with self.assertRaises(ValueError):
            scene.solve_homography(pix, wld)

    def test_preset_world_rotations_stay_on_table(self):
        for k in range(4):
            pts = scene.preset_world(rotate=k * 90)
            self.assertEqual(len(pts), 4)
            for x, y in pts:
                self.assertLessEqual(abs(x), scene.TABLE_SIZE[0] / 2 + 1e-9)
                self.assertLessEqual(abs(y), scene.TABLE_SIZE[1] / 2 + 1e-9)

    def test_mirror_flips_x(self):
        a = scene.preset_world(0, mirror=False)
        b = scene.preset_world(0, mirror=True)
        for (x1, y1), (x2, y2) in zip(a, b):
            self.assertAlmostEqual(x1, -x2)
            self.assertAlmostEqual(y1, y2)


class TestScene(unittest.TestCase):
    def setUp(self):
        self.scene = scene.Scene(path=Path(ROOT) / ".tmp" / "test-scene.json")
        self.scene.calibrate(PIXEL_CORNERS, WORLD_CORNERS)
        self.scene.clear_objects()

    def tearDown(self):
        try:
            self.scene.path.unlink()
        except OSError:
            pass

    def test_calibration_state(self):
        s = self.scene.state()
        self.assertTrue(s["calibrated"])
        self.assertEqual(len(s["homography"]), 3)
        self.assertLess(s["calibration"]["rms_mm"], 1e-3)
        self.assertGreater(s["calibration"]["mm_per_px"], 1.0)
        json.dumps(s)                      # 必须能直接进 HTTP JSON

    def test_unproject_project_roundtrip(self):
        p = self.scene.unproject(320.0, 240.0)
        self.assertIsNotNone(p)
        u, v = self.scene.project(*p)
        self.assertAlmostEqual(u, 320.0, places=6)
        self.assertAlmostEqual(v, 240.0, places=6)

    def test_uncallibrated_rejects_pixels(self):
        fresh = scene.Scene(path=Path(ROOT) / ".tmp" / "test-scene-empty.json")
        self.assertIsNone(fresh.unproject(320.0, 240.0))
        with self.assertRaises(ValueError):
            fresh.place(u=320.0, v=240.0)

    def test_static_cell_has_no_crate(self):
        """「料盒」已经删掉：真实工位上没有这个东西。

        它原来摆在 (0.46,-0.26,0.045)，是凭空加进模型的；现场核对时那个位置
        什么都没有，可它把一条本来可行的抓取路径判成了 −11 mm 碰撞。凭空的
        障碍物比没有障碍物更危险——它会让人相信模型，然后去改本来正确的动作。
        """
        names = [o["name"] for o in cell.CELL["obstacles"]]
        self.assertNotIn("料盒", names)
        self.assertIn("相机立柱", names)

    def test_cube_lands_where_the_world_says(self):
        target = (0.30, -0.20)
        u, v = self.scene.project(*target)
        reading = {"found": True, "center": [u, v], "theta": 0.0, "cell": 60.0,
                   "angle": 0.0, "reliable": True, "score": 1.0, "ms": 10.0,
                   "cells": ["U"] * 9}
        out = self.scene.update_cube(reading)
        self.assertTrue(out["ok"])
        obj = out["object"]
        self.assertAlmostEqual(obj["center"][0], target[0], places=6)
        self.assertAlmostEqual(obj["center"][1], target[1], places=6)
        self.assertAlmostEqual(obj["size"][0], scene.CUBE_M)
        self.assertEqual(obj["source"], "vision")

    def test_cube_enters_cell_but_not_global_cell(self):
        u, v = self.scene.project(0.30, -0.20)
        self.scene.update_cube({"found": True, "center": [u, v], "theta": 0.0,
                                "cell": 60.0, "ms": 5.0})
        merged = self.scene.cell()
        names = [o["name"] for o in merged["obstacles"]]
        self.assertIn("相机·魔方", names)
        self.assertEqual(len(cell.CELL["obstacles"]), 2)   # 全局工位没被改脏
        cube_ob = next(o for o in merged["obstacles"] if o["name"] == "相机·魔方")
        self.assertEqual(cube_ob["type"], "box")
        # 安全模型用「带 yaw 的盒子 → AABB」：0° 时就是 57 mm，45° 时最坏 √2 倍。
        self.assertGreaterEqual(cube_ob["size"][0], scene.CUBE_M - 1e-9)
        self.assertLessEqual(cube_ob["size"][0], scene.CUBE_M * math.sqrt(2) + 1e-9)
        obj = self.scene._objects["相机·魔方"]
        self.assertAlmostEqual(cube_ob["size"][0], obj.aabb_size()[0], places=4)

    def test_update_box_lands_where_world_says(self):
        """只认位置的立方体也走同一张物体表，source=vision、进碰撞模型。"""
        out = self.scene.update_box((0.32, -0.18), size=(0.06, 0.06, 0.06), yaw=12.0,
                                    meta={"origin": "box"})
        self.assertTrue(out["ok"])
        obj = out["object"]
        self.assertAlmostEqual(obj["center"][0], 0.32, places=6)
        self.assertAlmostEqual(obj["center"][1], -0.18, places=6)
        self.assertEqual(obj["source"], "vision")
        self.assertAlmostEqual(obj["yaw"], 12.0, places=6)
        names = [o["name"] for o in self.scene.cell()["obstacles"]]
        self.assertIn("相机·魔方", names)

    def test_box_and_cube_share_one_object(self):
        """面读数成功/失败来回切，表里只能有一个「相机·魔方」。"""
        self.scene.update_box((0.30, -0.20))
        u, v = self.scene.project(0.30, -0.20)
        self.scene.update_cube({"found": True, "center": [u, v], "theta": 0.0,
                                "cell": 60.0, "ms": 5.0})
        cubes = [o for o in self.scene.objects() if o["name"] == "相机·魔方"]
        self.assertEqual(len(cubes), 1)

    def test_vision_loss_keeps_last_object_until_ttl(self):
        u, v = self.scene.project(0.0, 0.0)
        self.scene.update_cube({"found": True, "center": [u, v], "theta": 0.0,
                                "cell": 60.0, "ms": 5.0})
        self.scene.update_cube({"found": False, "ms": 5.0})
        self.assertEqual(len(self.scene.objects()), 1)      # 丢一帧不删
        obj = self.scene._objects["相机·魔方"]
        obj.t -= scene.CUBE_TTL + 0.01                      # 假装过了 TTL
        self.assertEqual(len(self.scene.objects()), 0)
        self.assertEqual(len(self.scene.cell()["obstacles"]), 2)

    def test_manual_object_persists_and_clears(self):
        # 名字别和 cell.py 里已有的静态障碍撞：静态工位是另一套东西。
        self.scene.place("相机料盒", u=320.0, v=240.0, size=[0.2, 0.15, 0.08])
        self.assertEqual(len(self.scene.objects()), 1)
        self.assertEqual(len([o for o in self.scene.cell()["obstacles"]
                              if o["name"] == "相机料盒"]), 1)
        self.assertEqual(self.scene.clear_objects("manual"), 1)
        self.assertEqual(len(self.scene.objects()), 0)

    def test_yaw_is_wrapped_into_aabb(self):
        ob = scene.SceneObject(name="x", kind="box", center=(0.0, 0.0),
                               size=(0.10, 0.02, 0.05), yaw=45.0)
        w, d, h = ob.aabb_size()
        self.assertAlmostEqual(w, 0.12 * math.sqrt(0.5), places=6)
        self.assertAlmostEqual(d, 0.12 * math.sqrt(0.5), places=6)
        self.assertAlmostEqual(h, 0.05, places=6)

    def test_cylinder_obstacle_shape(self):
        self.scene.place("杯子", u=320.0, v=240.0, size=[0.04, 0.12], kind="cylinder")
        ob = next(o for o in self.scene.cell()["obstacles"] if o["name"] == "杯子")
        self.assertEqual(ob["type"], "cylinder")
        self.assertEqual(ob["base"][2], 0.0)
        self.assertEqual(ob["height"], 0.12)

    def test_recalibration_drops_stale_objects(self):
        self.scene.place("临时", u=320.0, v=240.0)
        self.scene.calibrate(PIXEL_CORNERS, WORLD_CORNERS)
        self.assertEqual(len(self.scene.objects()), 0)

    def test_summary_reports_staleness(self):
        u, v = self.scene.project(0.0, 0.0)
        self.scene.update_cube({"found": True, "center": [u, v], "theta": 0.0,
                                "cell": 60.0, "ms": 5.0})
        # 失效线（2 s）比物体 TTL（3 s）早：物体还在的时候就要先报失效。
        self.scene.last_vision_t -= scene.VISION_STALE_S - 0.5
        self.assertFalse(self.scene.summary()["vision_stale"])
        self.scene.last_vision_t -= 1.0
        s = self.scene.summary()
        self.assertTrue(s["vision_stale"])
        self.assertGreater(s["vision_age"], scene.VISION_STALE_S)
        self.assertEqual(len(s["objects"]), 1)       # TTL 还没到，物体仍在


if __name__ == "__main__":
    unittest.main(verbosity=2)
