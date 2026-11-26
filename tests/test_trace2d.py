"""二维手绘轨迹（trace2d）：坐标映射、重采样、反解、配置可调。

这些测试全部是**纯计算**，不连臂、不连总线——画布点到关节流这条链路
本身就只是数学，所以可以脱离硬件跑。
"""
from __future__ import annotations

import math
import unittest

from handlink import kin, trace2d


class TestCanvasMapping(unittest.TestCase):
    def test_corners(self):
        x0, x1 = trace2d.DEFAULT.region_x
        y0, y1 = trace2d.DEFAULT.region_y
        self.assertAlmostEqual(trace2d.canvas_to_paper(0, 0)[0], x0)
        self.assertAlmostEqual(trace2d.canvas_to_paper(1, 1)[0], x1)
        # 默认顶边是近端（y1）：实测三维相机把 y1 投影在屏幕上方，y0 在下方，
        # 这样画布和 /arm3d 的视角才是同一个方向。
        self.assertAlmostEqual(trace2d.canvas_to_paper(0, 0)[1], y1)
        self.assertAlmostEqual(trace2d.canvas_to_paper(1, 1)[1], y0)

    def test_roundtrip(self):
        for u, v in ((0, 0), (1, 1), (0.37, 0.62), (0.5, 0.5)):
            x, y = trace2d.canvas_to_paper(u, v)
            u2, v2 = trace2d.paper_to_canvas(x, y)
            self.assertAlmostEqual(u, u2, places=9)
            self.assertAlmostEqual(v, v2, places=9)

    def test_clamps_outside(self):
        """画布外的输入要夹到边界，而不是算出区域外的坐标。"""
        x, y = trace2d.canvas_to_paper(-0.5, 1.7)
        self.assertAlmostEqual(x, trace2d.DEFAULT.region_x[0])
        self.assertAlmostEqual(y, trace2d.DEFAULT.region_y[0])

    def test_region_is_finite_and_ordered(self):
        cfg = trace2d.DEFAULT
        self.assertLess(cfg.region_x[0], cfg.region_x[1])
        self.assertLess(cfg.region_y[0], cfg.region_y[1])


class TestResample(unittest.TestCase):
    def test_even_spacing(self):
        pts = [(0.0, 0.0), (100.0, 0.0)]
        out = trace2d.resample(pts, 5.0)
        self.assertEqual(len(out), 21)
        for i in range(len(out) - 1):
            self.assertAlmostEqual(math.dist(out[i], out[i + 1]), 5.0, places=9)

    def test_spacing_never_exceeds_step(self):
        pts = [(0.0, 0.0), (37.0, 11.0), (12.0, 90.0), (60.0, 60.0)]
        out = trace2d.resample(pts, 5.0)
        for i in range(len(out) - 1):
            self.assertLessEqual(math.dist(out[i], out[i + 1]), 5.0 + 1e-9)

    def test_degenerate(self):
        self.assertEqual(trace2d.resample([], 5.0), [])
        self.assertEqual(len(trace2d.resample([(1.0, 2.0)], 5.0)), 1)
        # 重复点不应该产生零长度的段
        out = trace2d.resample([(0.0, 0.0), (0.0, 0.0), (10.0, 0.0)], 5.0)
        self.assertEqual(len(out), 3)

    def test_zero_step_is_noop(self):
        pts = [(0.0, 0.0), (10.0, 0.0)]
        self.assertEqual(trace2d.resample(pts, 0.0), pts)

    def test_dense_input_still_yields_full_step_segments(self):
        """手绘的密集点必须被拉开到完整步长，而不是原样留下。

        2026-11-26 现场：画一个圆「直接卡住」。鼠标点本身就 3~4 mm 一个（比步长密），
        老版 resample 只保证「不超过 step」，于是输出全是 3~4 mm 的小段；圆角半径
        要求 ``r ≤ 段长/2``，r=5 要 10 mm，放不下 → 控制器回 -1 → 每个点退回 cp=0
        停车。150 个点就是 150 次急停。
        """
        dense = [(i * 3.0, 0.0) for i in range(101)]        # 300 mm，3 mm 一个点
        out = trace2d.resample(dense, 15.0)
        self.assertLessEqual(len(out), 22)
        for i in range(len(out) - 1):
            seg = math.dist(out[i], out[i + 1])
            # 不能留下放不下圆角的小段（要求 >= 10 mm 才容得下 r=5）
            self.assertGreater(seg, 10.0, "第 %d 段只有 %.1f mm，圆角放不下" % (i, seg))
            self.assertLessEqual(seg, 15.0 + 1e-9)

    def test_single_stroke_input_keeps_uniform_spacing(self):
        """非等距输入的每段长度都应当接近步长，不能忽长忽短。

        注意下限放到 0.7×step 而不是 0.9×：采样点是沿**弧长**等距的，到了拐角处
        弦长必然短于弧长（这是几何事实，不是 bug）。真正要杜绝的是「3 mm 的小段」，
        那才是圆角放不下、逐点停车的元凶。
        """
        pts = [(0.0, 0.0), (2.0, 0.0), (5.5, 1.0), (40.0, 3.0), (41.0, 30.0)]
        out = trace2d.resample(pts, 10.0)
        segs = [math.dist(out[i], out[i + 1]) for i in range(len(out) - 1)]
        self.assertLessEqual(max(segs), 10.0 + 1e-9)
        self.assertGreater(min(segs), 0.7 * 10.0)

    def test_corner_is_snapped_not_rounded_off(self):
        """手绘直角必须保住，不能被等距采样斜切成一段弧。

        拐角被削掉多少，写出来的「口」「国」这类字就会圆掉多少。吸附只挪采样点，
        段长仍要落在合理范围内。
        """
        pts = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0)]
        out = trace2d.resample(pts, 15.0, corner_deg=50.0)
        self.assertIn((100.0, 0.0), out, "直角点被采样削掉了")
        segs = [math.dist(out[i], out[i + 1]) for i in range(len(out) - 1)]
        self.assertGreater(min(segs), 0.55 * 15.0)
        self.assertLessEqual(max(segs), 1.45 * 15.0)

    def test_corner_snapping_can_be_disabled(self):
        """关掉吸附后，拐角不会被特意保住；开着就必须保住。

        用一个**拐角不落在均匀采样点上**的折线（总长 200 mm、步长 15 mm 时
        间距 14.29 mm，100 mm 恰好是第 7 个点，太巧了）。
        """
        pts = [(0.0, 0.0), (93.0, 0.0), (93.0, 107.0)]
        off = trace2d.resample(pts, 15.0, corner_deg=0.0)
        on = trace2d.resample(pts, 15.0, corner_deg=50.0)
        self.assertNotIn((93.0, 0.0), off, "关掉吸附时不该恰好落上拐角")
        self.assertIn((93.0, 0.0), on, "打开吸附时必须保住直角")

    def test_gentle_curves_are_not_treated_as_corners(self):
        """圆弧上的点方向变化小，不该被当成拐角吸附（那会把圆画成多边形）。"""
        arc = [(50.0 + 50.0 * math.cos(a * math.pi / 180.0),
                50.0 + 50.0 * math.sin(a * math.pi / 180.0))
               for a in range(0, 181, 3)]
        out = trace2d.resample(arc, 5.0, corner_deg=50.0)
        for i in range(1, len(out) - 1):
            turn = trace2d._turn_deg(out[i - 1], out[i], out[i + 1])
            self.assertLess(turn, 50.0)


class TestTipGeometry(unittest.TestCase):
    def test_tip_roundtrip(self):
        """指尖点 → 法兰 → 反解 → 正解 → 指尖，应当回到原处。"""
        cfg = trace2d.DEFAULT
        for x, y in ((0.0, -330.0), (-200.0, -400.0), (300.0, -250.0)):
            pose = trace2d.tip_to_flange(x, y, cfg.paper_z, cfg)
            self.assertAlmostEqual(pose[0], x - cfg.tip_offset[0], places=9)
            self.assertAlmostEqual(pose[1], y - cfg.tip_offset[1], places=9)
            self.assertAlmostEqual(pose[2], cfg.paper_z - cfg.tip_offset[2], places=9)
            self.assertEqual(pose[3:], [cfg.tool_roll, cfg.tool_pitch, cfg.tool_yaw])

    def test_tool_points_down(self):
        """rx=180, ry=0 时工具 Z 轴必须是 (0,0,-1)，否则“指尖朝下”不成立。"""
        cfg = trace2d.DEFAULT
        R = kin.rpy_to_matrix(math.radians(cfg.tool_roll),
                              math.radians(cfg.tool_pitch),
                              math.radians(cfg.tool_yaw))
        self.assertAlmostEqual(R[2][2], -1.0, places=6)

    def test_tip_of_joints_matches_target(self):
        """反解出的关节角，正解回算的指尖要落在目标点上。"""
        cfg = trace2d.DEFAULT
        target = (0.0, -330.0)
        pose = trace2d.tip_to_flange(target[0], target[1], cfg.paper_z, cfg)
        q = kin._ik_once(pose, cfg.seed_joints, iters=200)
        self.assertIsNotNone(q)
        tip = trace2d.tip_of_joints(q, cfg)
        self.assertLess(math.dist(tip[:2], target), 1.0)
        self.assertLess(abs(tip[2] - cfg.paper_z), 1.0)


class TestTraceToJoints(unittest.TestCase):
    def test_line(self):
        plan = trace2d.trace_to_joints([[0.2, 0.5], [0.8, 0.5]], trace2d.DEFAULT)
        self.assertTrue(plan["ok"])
        rep = plan["report"]
        self.assertEqual(rep["failed"], 0)
        self.assertEqual(rep["n_points"], rep["n_joints"])
        self.assertGreater(rep["length_mm"], 400)
        self.assertLess(rep["max_joint_step_deg"], 12.0)
        self.assertEqual(len(plan["steps"]), rep["n_joints"])
        for step in plan["steps"]:
            self.assertEqual(len(step["joints"]), 6)

    def test_closed_loop_returns_home(self):
        """首尾相连时，最后一个点应该回到第一个点。"""
        plan = trace2d.trace_to_joints(
            [[0.3, 0.3], [0.7, 0.3], [0.7, 0.7], [0.3, 0.7]], closed=True)
        self.assertTrue(plan["ok"])
        first, last = plan["paper_points"][0], plan["paper_points"][-1]
        self.assertAlmostEqual(first[0], last[0], places=6)
        self.assertAlmostEqual(first[1], last[1], places=6)

    def test_tip_path_follows_paper_points(self):
        plan = trace2d.trace_to_joints([[0.1, 0.2], [0.5, 0.5], [0.9, 0.8]])
        self.assertTrue(plan["ok"])
        worst = max(math.dist(tip[:2], p)
                    for tip, p in zip(plan["tip_path"], plan["paper_points"]))
        self.assertLess(worst, 1.0)

    def test_too_few_points(self):
        self.assertFalse(trace2d.trace_to_joints([[0.5, 0.5]])["ok"])
        self.assertFalse(trace2d.trace_to_joints([])["ok"])

    def test_joint_limits_respected(self):
        plan = trace2d.trace_to_joints([[0.0, 0.0], [1.0, 1.0]], trace2d.DEFAULT)
        self.assertTrue(plan["ok"])
        for step in plan["steps"]:
            self.assertEqual(kin.within_limits(step["joints"]), [])

    def test_hand_drawn_circle_yields_blendable_segments(self):
        """现场复现：手绘一个圆要能一直平滑走，不能逐点停车。

        这条把「输入端是密集手绘点」这个真实条件钉住——之前的单测都用 3 个稀疏点，
        所以 resample 那个 bug 一直没被抓住。
        """
        import dataclasses
        raw = [[0.5 + 0.16 * math.cos(2 * math.pi * i / 150),
                0.5 + 0.16 * math.sin(2 * math.pi * i / 150)] for i in range(150)]
        plan = trace2d.trace_to_joints(raw, trace2d.DEFAULT, closed=True)
        self.assertTrue(plan["ok"])
        rep = plan["report"]
        self.assertEqual(rep["failed"], 0)
        # 150 个输入点必须被压到「按步长算出来的数量」附近，不能原样留 150 个
        self.assertLess(rep["n_points"], 60, "密集输入没有被重采样：%d 点" % rep["n_points"])
        pp = plan["paper_points"]
        min_seg = min(math.dist(pp[i], pp[i + 1]) for i in range(len(pp) - 1))
        need = 2.0 * plan["config"]["blend_r_mm"]      # r <= 段长/2
        self.assertGreater(min_seg, need,
                           "最短段 %.1f mm 容不下 r=%.1fmm 的圆角"
                           % (min_seg, plan["config"]["blend_r_mm"]))


class TestConfig(unittest.TestCase):
    def test_config_is_overridable(self):
        """接真机时要能只改配置，不动算法。"""
        import dataclasses
        cfg = dataclasses.replace(trace2d.DEFAULT, paper_z=120.0, max_step_mm=2.0,
                                  tip_offset=(0.0, 0.0, -200.0))
        plan = trace2d.trace_to_joints([[0.3, 0.5], [0.7, 0.5]], cfg)
        self.assertTrue(plan["ok"])
        self.assertEqual(plan["config"]["paper_z"], 120.0)
        self.assertEqual(plan["config"]["tip_offset"], [0.0, 0.0, -200.0])
        # 步长变小 -> 点数变多
        self.assertGreater(plan["report"]["n_points"], 100)

    def test_region_change_moves_points(self):
        import dataclasses
        cfg = dataclasses.replace(trace2d.DEFAULT, region_x=(-100.0, 100.0),
                                  region_y=(-400.0, -200.0))
        plan = trace2d.trace_to_joints([[0.0, 1.0]], cfg)
        self.assertFalse(plan["ok"])          # 只有一个点
        plan = trace2d.trace_to_joints([[0.0, 0.0], [1.0, 1.0]], cfg)
        self.assertTrue(plan["ok"])
        # 画布左上角 = (x0, y1)，默认顶边是近端
        self.assertAlmostEqual(plan["paper_points"][0][0], -100.0, places=3)
        self.assertAlmostEqual(plan["paper_points"][0][1], -200.0, places=3)

    def test_env_config_file(self):
        import json
        import os
        import uuid
        from pathlib import Path
        root = Path(__file__).resolve().parent.parent / ".tmp"
        root.mkdir(parents=True, exist_ok=True)
        path = root / ("trace-cfg-%s.json" % uuid.uuid4().hex[:8])
        path.write_text(json.dumps({"paper_z": 88.0, "tool_yaw": 30.0}),
                        encoding="utf-8")
        try:
            cfg = trace2d.config_from_env(str(path))
            self.assertEqual(cfg.paper_z, 88.0)
            self.assertEqual(cfg.tool_yaw, 30.0)
            # 没写的字段保持默认
            self.assertEqual(cfg.max_step_mm, trace2d.DEFAULT.max_step_mm)
        finally:
            path.unlink(missing_ok=True)

    def test_bad_config_falls_back(self):
        import uuid
        from pathlib import Path
        root = Path(__file__).resolve().parent.parent / ".tmp"
        root.mkdir(parents=True, exist_ok=True)
        path = root / ("trace-bad-%s.json" % uuid.uuid4().hex[:8])
        path.write_text("{ not json", encoding="utf-8")
        try:
            self.assertEqual(trace2d.config_from_env(str(path)).paper_z,
                             trace2d.DEFAULT.paper_z)
        finally:
            path.unlink(missing_ok=True)


class TestMirror(unittest.TestCase):
    """镜像只改「画布 → 纸面」这一步：轨迹整体翻转，长度和关节特性不变。"""

    def test_mirror_x_flips_corners(self):
        import dataclasses
        cfg = dataclasses.replace(trace2d.DEFAULT, mirror_x=True)
        x0, x1 = cfg.region_x
        self.assertAlmostEqual(trace2d.canvas_to_paper(0, 0, cfg)[0], x1)
        self.assertAlmostEqual(trace2d.canvas_to_paper(1, 0, cfg)[0], x0)

    def test_mirror_y_flips_corners(self):
        import dataclasses
        base = trace2d.DEFAULT
        cfg = dataclasses.replace(base, mirror_y=True)
        y0, y1 = cfg.region_y
        # 默认顶边=近端；镜像后顶边=远端
        self.assertAlmostEqual(trace2d.canvas_to_paper(0, 0, base)[1], y1)
        self.assertAlmostEqual(trace2d.canvas_to_paper(0, 0, cfg)[1], y0)
        self.assertAlmostEqual(trace2d.canvas_to_paper(0, 1, cfg)[1], y1)

    def test_mirror_roundtrip(self):
        import dataclasses
        for mx in (False, True):
            for my in (False, True):
                cfg = dataclasses.replace(trace2d.DEFAULT, mirror_x=mx, mirror_y=my)
                for u, v in ((0, 0), (1, 1), (0.37, 0.62)):
                    p = trace2d.canvas_to_paper(u, v, cfg)
                    u2, v2 = trace2d.paper_to_canvas(p[0], p[1], cfg)
                    self.assertAlmostEqual(u, u2, places=9)
                    self.assertAlmostEqual(v, v2, places=9)

    def test_mirror_keeps_length(self):
        import dataclasses
        base = trace2d.trace_to_joints([[0.1, 0.2], [0.9, 0.8]], trace2d.DEFAULT)
        flip = trace2d.trace_to_joints(
            [[0.1, 0.2], [0.9, 0.8]],
            dataclasses.replace(trace2d.DEFAULT, mirror_x=True))
        self.assertTrue(base["ok"] and flip["ok"])
        self.assertAlmostEqual(base["report"]["length_mm"],
                               flip["report"]["length_mm"], places=1)
        # 起点 x 应该翻到另一端
        self.assertAlmostEqual(base["paper_points"][0][0] + flip["paper_points"][0][0],
                               sum(trace2d.DEFAULT.region_x), places=1)

    def test_mirror_reported_in_config(self):
        import dataclasses
        plan = trace2d.trace_to_joints(
            [[0.2, 0.2], [0.8, 0.8]],
            dataclasses.replace(trace2d.DEFAULT, mirror_x=True))
        self.assertEqual(plan["config"]["mirror"], {"x": True, "y": False})


if __name__ == "__main__":
    unittest.main()
