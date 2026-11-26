"""下发顺序：归位 → 起点 → 逐点 MovL（不连真机，用假臂记录调用）。"""
from __future__ import annotations

import unittest
from unittest import mock

from handlink import arm as arm_mod
from handlink import bus, cell, kin, service, trace2d


class FakeRobot:
    """只记录调用顺序，不动。"""

    calls = []
    cps = []
    radii = []

    def __init__(self, *a, **kw):
        pass

    def connect(self):
        FakeRobot.calls.append(("connect",))

    def close(self):
        FakeRobot.calls.append(("close",))

    def speed_factor(self, pct):
        FakeRobot.calls.append(("speed", pct))

    def move_joints(self, q, wait=True, timeout=20.0):
        FakeRobot.calls.append(("move_joints", [round(float(v), 3) for v in q]))

    def move_l(self, pose, wait=True, timeout=30.0):
        FakeRobot.calls.append(("move_l", [round(float(v), 3) for v in pose]))

    def movl_cp(self, pose, cp):
        FakeRobot.calls.append(("move_l", [round(float(v), 3) for v in pose]))
        FakeRobot.cps.append(int(cp))
        return len(FakeRobot.cps)

    def current_cmd_id(self):
        return len(FakeRobot.cps) + len(FakeRobot.radii)   # 假装机器全走完了

    def movl_r(self, pose, r_mm, accel=None):
        FakeRobot.calls.append(("move_l", [round(float(v), 3) for v in pose]))
        FakeRobot.radii.append((float(r_mm), accel))
        return len(FakeRobot.cps) + len(FakeRobot.radii)

    def wait_cmd_id(self, cmd_id, timeout=30.0):
        return True

    def wait_pose_close(self, target, tol=2.0, timeout=30.0, settle=3):
        FakeRobot.calls.append(("wait_pose_close",))

    def stop(self):
        FakeRobot.calls.append(("stop",))


class FakeFeed:
    """假装 30004 有数据。"""

    def snapshot(self):
        return {"QActual": [0.0] * 6, "ToolVectorActual": [0.0] * 6}

    def age(self):
        return 0.0


class TestTraceHoming(unittest.TestCase):
    def setUp(self):
        FakeRobot.calls = []
        FakeRobot.cps = []
        FakeRobot.radii = []
        self.app = service.App(bus.SimBus())
        self.app.arm_feed = FakeFeed()
        self.cfg = trace2d.DEFAULT
        self.pts = [[0.2, 0.5], [0.8, 0.5]]

    def _patches(self):
        """让 App 以为真机在线、已使能、闸门全过。"""
        return [
            mock.patch.object(self.app, "_arm_live", return_value=True),
            mock.patch.object(self.app, "_arm_precheck", return_value=None),
            mock.patch.object(self.app, "_arm_gate",
                              side_effect=lambda cur, tgt, force=False, unwind=False: (list(tgt), {})),
            mock.patch.object(service.arm_feed, "describe",
                              return_value={"joints": [0.0] * 6,
                                            "pose": [300.0, -300.0, 300.0, 180.0, 0.0, 0.0]}),
            mock.patch.object(arm_mod, "DobotArm", FakeRobot),
        ]

    def test_approaches_start_without_going_home(self):
        """默认不再绕固定归位位姿：抬到安全高度 → 移到起点上方 → 落到起点。

        归位那一步原本要多走一整个大动作（从哪来都要先去 home），既慢又多一次碰撞
        机会；竖直抬升 + 竖直下落本身就足够安全。
        """
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=self.cfg, apply=True)
        finally:
            for p in patches:
                p.stop()

        self.assertTrue(out["ok"], out)
        self.assertFalse(out.get("homed"))
        kinds = [c[0] for c in FakeRobot.calls]
        self.assertNotIn("move_joints", kinds)
        labels = [s["label"] for s in out["approach"]]
        self.assertEqual(labels, ["抬到安全高度", "移到起点上方"])
        start_pose = out["paper_points"][0]
        del start_pose                      # 只需要用到配置里的纸面高度
        flange_z = self.cfg.paper_z - self.cfg.tip_offset[2]
        safe_z = flange_z + self.cfg.approach_lift_mm
        # 第一段：x/y 不动，只抬 z；第二段：抬到起点正上方
        first = out["approach"][0]["pose"]
        self.assertAlmostEqual(first[2], safe_z, places=1)
        second = out["approach"][1]["pose"]
        self.assertAlmostEqual(second[2], safe_z, places=1)
        # 起点那一条是一个落在纸面高度上的 move_l
        moves = [c[1] for c in FakeRobot.calls if c[0] == "move_l"]
        self.assertAlmostEqual(moves[0][2], safe_z, places=1)
        self.assertAlmostEqual(moves[1][2], safe_z, places=1)
        self.assertAlmostEqual(moves[2][2], flange_z, places=1)
        self.assertEqual(kinds[-1], "close")

    def test_points_use_absolute_blend_radius(self):
        """显式设了 blend_r_mm 就走 r（绝对毫米）而不是 cp 百分比。"""
        import dataclasses
        cfg = dataclasses.replace(self.cfg, blend_r_mm=3.0)
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["dispatch"]["blend"], "r=%.1fmm" % cfg.blend_r_mm)
        self.assertGreater(len(FakeRobot.radii), 2)
        self.assertTrue(all(r == cfg.blend_r_mm for r, _a in FakeRobot.radii))
        # 收笔点仍然 cp=0，带 r 会冲过终点
        self.assertEqual(FakeRobot.cps, [0])

    def test_default_blend_is_r(self):
        """默认走 r：cp 在这台固件上实测不混合（速度曲线是锯齿）。"""
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=self.cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["dispatch"]["blend"], "r=%.1fmm" % self.cfg.blend_r_mm)
        self.assertGreater(len(FakeRobot.radii), 2)
        self.assertEqual(FakeRobot.cps, [0])

    def test_cp_still_selectable(self):
        import dataclasses
        cfg = dataclasses.replace(self.cfg, blend_r_mm=0.0, smooth_cp=30)
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["dispatch"]["blend"], "cp=30")
        self.assertEqual(FakeRobot.radii, [])
        self.assertTrue(all(c == 30 for c in FakeRobot.cps[:-1]))
        self.assertEqual(FakeRobot.cps[-1], 0)

    def test_home_still_available_when_explicitly_enabled(self):
        import dataclasses
        cfg = dataclasses.replace(self.cfg, home_enabled=True)
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertTrue(out.get("homed"))
        self.assertEqual(out["approach"], [])
        kinds = [c[0] for c in FakeRobot.calls]
        first_mj = kinds.index("move_joints")
        self.assertEqual(FakeRobot.calls[first_mj][1],
                         [round(v, 3) for v in cfg.home_joints])
        rest = kinds[first_mj + 1:]
        self.assertEqual(rest.count("move_l"), out["steps_done"])
        self.assertEqual(set(rest), {"move_l", "close"})

    def test_preview_never_touches_robot(self):
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=self.cfg, apply=False)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"])
        self.assertEqual(FakeRobot.calls, [])

    def test_hand_homes_with_arm(self):
        """手要摆到设定姿态，而且是在抬臂之后、落到起点之前。"""
        seen = []
        orig = self.app.hand_home

        def spy(pose_name=None):
            seen.append((pose_name, len(FakeRobot.calls)))
            return orig(pose_name)

        patches = self._patches() + [mock.patch.object(self.app, "hand_home", side_effect=spy)]
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=self.cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0][0], self.cfg.hand_home_pose)
        self.assertEqual(out["hand_home"]["pose"], self.cfg.hand_home_pose)
        # 手归位发生在接近段之后：调用序列里已经有 move_l（抬升/平移）
        kinds_before = [c[0] for c in FakeRobot.calls[:seen[0][1]]]
        self.assertGreaterEqual(kinds_before.count("move_l"), 1)
        self.assertNotIn("move_joints", kinds_before)

    def test_hand_home_can_be_disabled(self):
        import dataclasses
        cfg = dataclasses.replace(self.cfg, hand_home_enabled=False)
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertNotIn("hand_home", out)

    def test_blend_radius_goes_out_as_r_not_cp(self):
        """慢速书写用绝对圆角半径 r（毫米）：cp 是百分比，低速下几乎磨不出角。"""
        import dataclasses
        cfg = dataclasses.replace(self.cfg, blend_r_mm=3.0, accel_pct=50)
        patches = self._patches()
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["dispatch"]["blend"], "r=3.0mm")
        self.assertEqual(out["dispatch"]["accel"], 50)
        self.assertGreater(len(FakeRobot.radii), 1)
        self.assertTrue(all(r == 3.0 and a == 50 for r, a in FakeRobot.radii),
                        FakeRobot.radii)
        # 收笔那一条仍然必须是 cp=0，不能带 r 冲出去
        self.assertEqual(FakeRobot.cps, [0])

    def test_wrist_is_unwound_before_writing(self):
        """J6 顶到限位时先把腕子绕回等价角度（2026-11-25 真机 -357.9° 报警）。"""
        feed = {"joints": [0.0, 0.0, 0.0, 0.0, 0.0, -357.891],
                "pose": [300.0, -300.0, 300.0, 180.0, 0.0, 0.0]}
        patches = self._patches()[:3] + [
            mock.patch.object(service.arm_feed, "describe", return_value=feed),
            mock.patch.object(arm_mod, "DobotArm", FakeRobot),
        ]
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=self.cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertEqual(out["wrist_unwind"]["before"], -357.891)
        self.assertAlmostEqual(out["wrist_unwind"]["after"], 2.109, places=2)
        kinds = [c[0] for c in FakeRobot.calls]
        self.assertEqual(kinds[:3], ["connect", "speed", "move_joints"])
        self.assertAlmostEqual(FakeRobot.calls[2][1][5], 2.109, places=2)

    def test_throttle_failure_does_not_cut_the_stroke(self):
        """限流只是一条状态查询。它坏了也不能把已经发了一半的笔画掐断：

        2026-11-25 真机现场，``current_cmd_id()`` 因为常量没定义抛 AttributeError，
        第 40 点之后整条轨迹变成「未执行」，臂被 Stop() 硬停在半路。
        """

        class BlindRobot(FakeRobot):
            def current_cmd_id(self):
                raise AttributeError("'DobotArm' object has no attribute 'CMD_ARRIVE_INDEX'")

        # 窗口调小，保证这条短轨迹真的走到「限流检查」那一步
        import dataclasses
        cfg = dataclasses.replace(self.cfg, batch_size=5)
        patches = self._patches()[:-1] + [mock.patch.object(arm_mod, "DobotArm", BlindRobot)]
        for p in patches:
            p.start()
        try:
            out = self.app.arm_trace(self.pts, cfg=cfg, apply=True)
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertTrue(out["moved"])
        self.assertEqual(out["dispatch"]["throttle"], "off(AttributeError)")
        # 所有点都发出去了，一条没少（steps_done 含起点那一条 move_l）
        self.assertEqual(len(FakeRobot.cps) + len(FakeRobot.radii),
                         out["steps_done"] - 1)


class TestArmHome(unittest.TestCase):
    def setUp(self):
        FakeRobot.calls = []
        FakeRobot.cps = []
        FakeRobot.radii = []
        self.app = service.App(bus.SimBus())
        self.app.arm_feed = FakeFeed()

    def test_preview_moves_nothing(self):
        out = self.app.arm_home(apply=False, pose_name="fist")
        self.assertTrue(out["ok"])
        self.assertFalse(out["moved"])
        self.assertEqual(FakeRobot.calls, [])
        self.assertEqual(out["hand"]["pose"], "fist")
        self.assertEqual(out["arm_joints"], [round(v, 3) for v in trace2d.DEFAULT.home_joints])

    def test_bad_pose_rejected(self):
        out = self.app.arm_home(apply=False, pose_name="no-such-pose")
        self.assertFalse(out["ok"])
        self.assertIn("no-such-pose", out["error"])

    def test_apply_moves_arm_then_reports(self):
        patches = [
            mock.patch.object(self.app, "_arm_live", return_value=True),
            mock.patch.object(self.app, "_arm_precheck", return_value=None),
            mock.patch.object(self.app, "_arm_gate",
                              side_effect=lambda cur, tgt, force=False, unwind=False: (list(tgt), {})),
            mock.patch.object(service.arm_feed, "describe",
                              return_value={"joints": [0.0] * 6}),
            mock.patch.object(arm_mod, "DobotArm", FakeRobot),
        ]
        for p in patches:
            p.start()
        try:
            out = self.app.arm_home(apply=True, pose_name="open")
        finally:
            for p in patches:
                p.stop()
        self.assertTrue(out["ok"], out)
        self.assertTrue(out["moved"])
        kinds = [c[0] for c in FakeRobot.calls]
        self.assertIn("move_joints", kinds)
        self.assertEqual(out["hand"]["pose"], "open")



class TestHomeStages(unittest.TestCase):
    """归位路径必须真的不撞：现在的实现以前会"直接关节插值"，
    臂趴在纸面上时那一下子会扫过桌面。"""

    def setUp(self):
        self.app = service.App(bus.SimBus())
        self.cfg = trace2d.DEFAULT

    def _entry_pose(self, x, y, z=None):
        pose = trace2d.tip_to_flange(x, y, z if z is not None else self.cfg.paper_z,
                                     self.cfg)
        q = kin._ik_once(pose, self.cfg.seed_joints, iters=200)
        self.assertIsNotNone(q, "反解失败 (%s, %s)" % (x, y))
        return q

    def test_normal_trace_start_uses_direct(self):
        q = self._entry_pose(0.0, -330.0)
        plan = self.app.home_stages(q, self.cfg)
        self.assertEqual(plan["plan"], "direct")
        self.assertEqual(len(plan["stages"]), 1)
        # 每一段都必须过得了碰撞红线
        path = cell.check_path(q, plan["stages"][-1])
        self.assertTrue(path["safe"],
                        "归位路径最小间距 %.0f mm" % (path["min_m"] * 1000))

    def test_every_returned_stage_is_safe(self):
        """不管走哪条分支，返回的每一段都要安全。"""
        for x, y in ((0.0, -330.0), (-250.0, -400.0), (350.0, -250.0)):
            q = self._entry_pose(x, y)
            plan = self.app.home_stages(q, self.cfg)
            if plan.get("stages") is None:
                continue
            prev = q
            for stage in plan["stages"]:
                path = cell.check_path(prev, stage)
                self.assertTrue(path["safe"],
                                "(%s,%s) 的归位段最小间距 %.0f mm"
                                % (x, y, path["min_m"] * 1000))
                prev = stage
            # 最后一段必须就是配置里的归位姿态
            self.assertEqual([round(v, 3) for v in plan["stages"][-1]],
                             [round(v, 3) for v in self.cfg.home_joints])

    def test_already_colliding_is_reported_not_attempted(self):
        """起点已经贴在桌面上时，不能硬走。"""
        q = self._entry_pose(0.0, -480.0, 20.0)
        plan = self.app.home_stages(q, self.cfg)
        self.assertIsNone(plan.get("stages"))
        self.assertEqual(plan["plan"], "blocked_now")
        self.assertIn("mm", plan["error"])

    def test_clearance_units_are_millimetres(self):
        """min_m 是米、SAFE_MM 是毫米，比错过一次会把正常姿态全判死。"""
        q = self._entry_pose(0.0, -330.0)
        now = cell.clearance(q)
        self.assertGreater(now["min_m"] * 1000, cell.SAFE_MM)
        plan = self.app.home_stages(q, self.cfg)
        self.assertEqual(plan["plan"], "direct")



if __name__ == "__main__":
    unittest.main()
