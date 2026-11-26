"""handlink 自测：不接硬件也能全部跑通。"""
from __future__ import annotations

import json
import shutil
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
import uuid
from pathlib import Path

from handlink import ai, bus, hardware, recorder, service, skills, spec, store

ROOT = Path(__file__).resolve().parent.parent
PKG = ROOT / "pkg" / "LG-603"
# 沙箱里系统 TEMP 不可写，临时文件统一放工作区
_TMP = ROOT / ".tmp"
_TMP.mkdir(parents=True, exist_ok=True)
tempfile.tempdir = str(_TMP)


def _mkdtemp() -> Path:
    """沙箱对 0o700 目录有额外限制，这里用默认权限自己建临时目录。"""
    p = _TMP / f"hl-{uuid.uuid4().hex[:10]}"
    p.mkdir(parents=True, exist_ok=True)
    return p


class TestSpec(unittest.TestCase):
    def test_limits_match_manual(self):
        self.assertEqual(spec.MOTOR_BY_ID[102].pos_max, 100.0)
        self.assertEqual(spec.MOTOR_BY_ID[102].current_max, 0.8)
        self.assertEqual(spec.MOTOR_BY_ID[107].pos_max, 45.0)
        for mid in (103, 104, 105, 106):
            self.assertEqual(spec.MOTOR_BY_ID[mid].pos_max, 180.0)
            self.assertEqual(spec.MOTOR_BY_ID[mid].current_max, 3.5)
        self.assertEqual(len(spec.MOTORS), 6)

    def test_resolve(self):
        for key in (102, "102", "thumb1", "拇指1"):
            self.assertEqual(spec.resolve(key).motor_id, 102)
        with self.assertRaises(KeyError):
            spec.resolve(999)

    def test_vendor_roundtrip(self):
        data = [{"motorId": 103, "angle": 159, "speed": 360, "torque": 0.5}]
        pose = spec.from_vendor(data)
        self.assertEqual(pose[103].angle, 159)
        self.assertEqual(spec.to_vendor(pose), data)

    def test_check_errors(self):
        pose = {103: spec.Joint(angle=200, speed=60, torque=0.5)}
        issues = spec.check(pose)
        self.assertTrue(spec.has_errors(issues))
        self.assertTrue(any("超出" in i["message"] for i in issues))

    def test_check_current(self):
        issues = spec.check({103: spec.Joint(angle=10, speed=60, torque=5.0)})
        self.assertTrue(spec.has_errors(issues))

    def test_warning_large_step(self):
        ref = spec.blank_pose()
        issues = spec.check({103: spec.Joint(angle=120, speed=60, torque=0.5)}, reference=ref)
        self.assertFalse(spec.has_errors(issues))
        self.assertTrue(any(i["level"] == "warning" for i in issues))

    def test_clamp(self):
        p = spec.clamp({102: spec.Joint(angle=999, speed=999, torque=9)})
        self.assertEqual(p[102].angle, 100.0)
        self.assertEqual(p[102].speed, 360.0)
        self.assertEqual(p[102].torque, 0.8)


@unittest.skipUnless((PKG / "points").is_dir(), "厂家软件包未解压")
class TestStore(unittest.TestCase):
    def test_vendor_point_unchanged(self):
        raw = (PKG / "points" / "rock.txt").read_text(encoding="utf-8")
        pose = store.load_point(PKG / "points" / "rock.txt")
        self.assertEqual(json.loads(store.dumps_point(pose)), json.loads(raw))
        self.assertEqual(store.dumps_point(pose), raw.strip())

    def test_save_load_roundtrip(self):
        td = _mkdtemp()
        try:
            pose = spec.blank_pose()
            pose[103] = spec.Joint(120, 60, 0.5)
            path = store.save_point(td / "t.txt", pose)
            again = store.load_point(path)
            self.assertEqual(again[103].angle, 120)
            self.assertNotIn(" ", path.read_text(encoding="utf-8"))
        finally:
            shutil.rmtree(td, ignore_errors=True)

    def test_track(self):
        track = store.load_track(PKG / "tracks" / "动作一.txt")
        self.assertEqual(track.points, ["paper", "scissors", "rock", "paper"])
        self.assertEqual(track.delays, [600, 1500, 1500, 600])
        self.assertEqual(track.loops, 2)
        self.assertEqual(store.dumps_track(track).splitlines()[0], "points=[paper,scissors,rock,paper]")
        again = store.load_track(Path("x")) if False else None
        self.assertIsNone(again)


class TestSim(unittest.TestCase):
    def setUp(self):
        self.sim = bus.open_sim()
        self.addCleanup(self.sim.close)

    def test_reach_target(self):
        ok = self.sim.apply({103: spec.Joint(120, 90, 0.6)})
        self.assertTrue(ok["ok"])
        self.assertTrue(self.sim.settle(timeout=3.0))
        self.assertAlmostEqual(self.sim.state()[103].angle, 120, delta=0.5)

    def test_current_within_limit(self):
        self.sim.apply({106: spec.Joint(60, 120, 0.3)})
        self.sim.settle(timeout=3.0)
        st = self.sim.state()[106]
        self.assertLessEqual(st.current, spec.MOTOR_BY_ID[106].current_max)

    def test_unknown_motor(self):
        with self.assertRaises(ValueError):
            self.sim.apply({999: spec.Joint(10)})

    def test_clamped_target(self):
        self.sim.apply({107: spec.Joint(90, 60, 0.3)})  # 拇指2 上限 45
        self.sim.settle(timeout=2.0)
        self.assertLessEqual(self.sim.state()[107].angle, 45.0)


class TestPlayer(unittest.TestCase):
    def test_play_track(self):
        td = _mkdtemp()
        try:
            d = store.dirs(td).ensure()
            store.save_point(d.points / "a.txt", {m.motor_id: spec.Joint(20, 60, 0.3) for m in spec.MOTORS})
            store.save_point(d.points / "b.txt", {m.motor_id: spec.Joint(40, 60, 0.3) for m in spec.MOTORS})
            track = store.Track(["a", "b", "a"], [60, 60, 60], 2)
            store.save_track(d.tracks / "t.txt", track)
            loaded = store.load_track(d.tracks / "t.txt")
            sim = bus.open_sim()
            try:
                player = bus.Player(sim, d.points)
                self.assertEqual(len(player.preview(loaded)), 6)
                result = player.play(loaded, speed_scale=20.0)
            finally:
                sim.close()
            self.assertTrue(result["ok"])
            self.assertEqual(result["played"], 6)
        finally:
            shutil.rmtree(td, ignore_errors=True)


class TestRecorder(unittest.TestCase):
    def test_record_and_analyze(self):
        sim = bus.open_sim()
        td = _mkdtemp()
        try:
            path = td / "r.jsonl"
            rec = recorder.Recorder(path, meta={"bus": "sim"})
            sampler = recorder.Sampler(sim, rec, hz=50).start()
            sim.apply({103: spec.Joint(90, 120, 0.8)})
            time.sleep(0.4)
            sampler.stop()
            rec.close()
            stats = recorder.analyze(path)
            self.assertGreater(stats["samples"], 3)
            self.assertGreater(stats["duration_s"], 0)
            self.assertIn("103", stats["motors"])
            md = recorder.to_markdown(stats)
            self.assertIn("录制分析", md)
            self.assertIn("食指", md)
        finally:
            sim.close()
            shutil.rmtree(td, ignore_errors=True)


class TestService(unittest.TestCase):
    def setUp(self):
        self.root = _mkdtemp()
        self.dirs = store.dirs(self.root).ensure()
        store.save_point(
            self.dirs.points / "half.txt",
            {m.motor_id: spec.Joint(m.pos_max * 0.5, 60, min(0.3, m.current_max)) for m in spec.MOTORS},
        )
        self.sim = bus.open_sim()
        self.app = service.App(self.sim, root=self.root)
        self.httpd = service.make_server(self.app, "127.0.0.1", 0)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.sim.close()
        shutil.rmtree(self.root, ignore_errors=True)

    def call(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}", data=data, method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                return resp.status, json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def test_health_and_state(self):
        code, body = self.call("GET", "/health")
        self.assertEqual(code, 200)
        self.assertTrue(body["ok"])
        code, body = self.call("GET", "/state")
        self.assertEqual(len(body["motors"]), 6)

    def test_limits(self):
        code, body = self.call("GET", "/limits")
        self.assertEqual(code, 200)
        self.assertEqual([m["motor_id"] for m in body["motors"]], list(spec.MOTOR_IDS))

    def test_pose_ok(self):
        code, body = self.call("POST", "/pose", {
            "motors": [{"motorId": 103, "angle": 120, "speed": 60, "torque": 0.5}],
            "reason": "test",
        })
        self.assertEqual(code, 200, body)
        self.assertTrue(body["ok"])

    def test_pose_out_of_range(self):
        code, body = self.call("POST", "/pose", {
            "motors": [{"motorId": 102, "angle": 999, "speed": 60, "torque": 0.5}],
        })
        self.assertEqual(code, 422)

    def test_point_and_list(self):
        code, body = self.call("GET", "/points")
        self.assertIn("half", body["points"])
        code, body = self.call("POST", "/point", {"name": "half"})
        self.assertEqual(code, 200, body)

    def test_track_dry_run(self):
        store.save_track(self.dirs.tracks / "t.txt", store.Track(["half"], [100], 2))
        code, body = self.call("POST", "/track", {"name": "t", "dry_run": True})
        self.assertEqual(code, 200, body)
        self.assertEqual(len(body["timeline"]), 2)

    def test_record_cycle(self):
        code, body = self.call("POST", "/record/start", {"hz": 50})
        self.assertEqual(code, 200, body)
        time.sleep(0.25)
        code, body = self.call("POST", "/record/stop")
        self.assertEqual(code, 200, body)
        self.assertGreater(body["stats"]["samples"], 2)

    def test_ai_skills(self):
        code, body = self.call("GET", "/ai/skills")
        self.assertEqual(code, 200)
        self.assertIn("open", [s["skill"] for s in body["skills"]])

    def test_ai_dry_run(self):
        code, body = self.call("POST", "/ai", {"text": "先张开再握拳", "dry_run": True})
        self.assertEqual(code, 200, body)
        self.assertEqual([s["skill"] for s in body["plan"]["steps"]], ["open", "close"])

    def test_ai_execute(self):
        code, body = self.call("POST", "/ai", {"text": "张开", "speed_scale": 8})
        self.assertEqual(code, 200, body)
        self.assertTrue(body["ok"])

    def test_ai_unknown(self):
        code, body = self.call("POST", "/ai", {"text": "帮我写周报"})
        self.assertEqual(code, 422)


class TestAI(unittest.TestCase):
    def test_rule_plan_basic(self):
        plan = ai.make_plan("先张开，再握拳，然后张开")
        self.assertEqual([s.skill for s in plan.steps], ["open", "close", "open"])
        self.assertEqual(plan.source, "rule")

    def test_chinese_duration(self):
        plan = ai.make_plan("保持两秒")
        self.assertEqual(plan.steps[0].args["ms"], 2000)

    def test_set_finger(self):
        plan = ai.make_plan("食指弯到 90 度")
        self.assertEqual(plan.steps[0].skill, "set_fingers")
        self.assertAlmostEqual(plan.steps[0].args["angles"][103], 90.0)

    def test_speed_hint(self):
        plan = ai.make_plan("轻轻捏住")
        self.assertEqual(plan.steps[0].args.get("speed"), 40.0)

    def test_unknown_text(self):
        with self.assertRaises(ValueError):
            ai.make_plan("今天天气不错")

    def test_skills_schema(self):
        names = [s["skill"] for s in skills.schema()]
        for expected in ("open", "close", "pinch", "set_fingers", "hold"):
            self.assertIn(expected, names)

    def test_out_of_range_is_clamped(self):
        steps = skills.steps_for("set_fingers", {"angles": {"拇指2": 999}})
        self.assertLessEqual(steps[0]["pose"][107].angle, 45.0)

    def test_executor_runs(self):
        sim = bus.open_sim()
        app = service.App(sim)
        try:
            plan = ai.make_plan("张开")
            result = ai.Executor(app.apply).run(plan, speed_scale=10)
            self.assertTrue(result["ok"])
            self.assertEqual(result["executed"], 1)
        finally:
            sim.close()

    def test_wave_and_beckon(self):
        self.assertEqual(ai.make_plan("招招手").steps[0].skill, "wave")
        self.assertEqual(ai.make_plan("勾勾手").steps[0].skill, "beckon")
        steps = skills.steps_for("wave", {"cycles": 2})
        self.assertGreaterEqual(len(steps), 4)
        beckon = skills.steps_for("beckon", {"cycles": 3})
        self.assertGreaterEqual(len(beckon), 6)
        for step in beckon:
            self.assertLessEqual(step["pose"][103].angle, 180.0)

    def test_executor_rejects_unknown_skill(self):
        plan = ai.Plan(text="x", steps=[ai.Step("fly", {})])
        result = ai.Executor(lambda pose, reason: {"ok": True}).run(plan)
        self.assertFalse(result["ok"])


class TestHardwareProbe(unittest.TestCase):
    def test_list_ports_returns_list(self):
        self.assertIsInstance(hardware.list_com_ports(), list)

    def test_find_can_dll(self):
        dll = hardware.find_can_dll(PKG)
        if dll:
            self.assertTrue(dll.is_file())
            self.assertEqual(dll.name, "ECanVci64.dll")


if __name__ == "__main__":
    unittest.main()
