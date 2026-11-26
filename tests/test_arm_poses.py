"""保存的姿态：文件读写（arm_poses.py）与 /arm/poses 接口。"""
from __future__ import annotations

import json
import shutil
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
import uuid
from pathlib import Path

from handlink import arm_poses, bus, kin, service

READY = [0.0, -25.0, -40.0, 0.0, 65.0, 0.0]

ROOT = Path(__file__).resolve().parent.parent
# 沙箱里系统 TEMP 不可写，临时文件统一放工作区（和 test_handlink 一样）
_TMP = ROOT / ".tmp"
_TMP.mkdir(parents=True, exist_ok=True)


def _mkdtemp() -> Path:
    p = _TMP / f"arm-poses-{uuid.uuid4().hex[:10]}"
    p.mkdir(parents=True, exist_ok=True)
    return p


class TestPoseStore(unittest.TestCase):
    def setUp(self):
        self.tmp = _mkdtemp()
        self.path = self.tmp / "config" / "arm_poses.json"
        self.store = arm_poses.PoseStore(self.path)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_save_lists_pose_and_joints(self):
        result = self.store.save("桌面抓取", READY, note="料盒正上方")
        self.assertTrue(result["ok"], result)
        self.assertFalse(result["replaced"])
        self.assertEqual(result["pose"]["joints"], READY)
        self.assertEqual(result["pose"]["pose"], [round(v, 3) for v in kin.fk(READY)])
        self.assertEqual(result["pose"]["note"], "料盒正上方")
        self.assertEqual(result["count"], 1)

        listing = self.store.state()
        self.assertEqual(listing["path"], str(self.path))
        self.assertEqual([p["name"] for p in listing["poses"]], ["桌面抓取"])

    def test_persists_across_instances(self):
        self.store.save("位 A", READY)
        again = arm_poses.PoseStore(self.path)
        self.assertEqual([p["name"] for p in again.state()["poses"]], ["位 A"])
        self.assertEqual(again.get("位 A")["joints"], READY)

    def test_auto_name_and_order(self):
        first = self.store.save("", READY)
        second = self.store.save("", READY)
        self.assertEqual(first["pose"]["name"], "姿态 1")
        self.assertEqual(second["pose"]["name"], "姿态 2")
        names = [p["name"] for p in self.store.state()["poses"]]
        self.assertEqual(names, ["姿态 1", "姿态 2"])   # 按示教顺序

    def test_overwrite_keeps_one_record(self):
        first = self.store.save("抓取", READY)
        second = self.store.save("抓取", [0, -30, -45, 0, 70, 0])
        self.assertTrue(second["replaced"])
        self.assertEqual(second["count"], 1)
        self.assertEqual(second["pose"]["created"], first["pose"]["created"])
        self.assertEqual(self.store.get("抓取")["joints"], [0, -30, -45, 0, 70, 0])

    def test_clamped_and_flagged(self):
        result = self.store.save("越界", [0, -999, 999, 0, 0, 0])
        self.assertTrue(result["clamped"])
        self.assertEqual(result["pose"]["joints"][1], kin.JOINT_LIMITS[1][0])
        self.assertEqual(result["pose"]["joints"][2], kin.JOINT_LIMITS[2][1])

    def test_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            self.store.save("x", [0, 0, 0])
        with self.assertRaises(ValueError):
            self.store.save("x", [0, 0, 0, 0, 0, float("nan")])
        with self.assertRaises(ValueError):
            self.store.save("x", ["a", 0, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            self.store.save("带/斜杠", READY)
        with self.assertRaises(ValueError):
            self.store.save("名" * (arm_poses.MAX_NAME + 1), READY)

    def test_delete(self):
        self.store.save("临时", READY)
        result = self.store.delete("临时")
        self.assertTrue(result["ok"])
        self.assertEqual(result["count"], 0)
        self.assertEqual(self.store.state()["poses"], [])
        with self.assertRaises(KeyError):
            self.store.delete("临时")

    def test_corrupt_or_missing_file_is_empty(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text("{ 这不是 JSON", encoding="utf-8")
        self.assertEqual(arm_poses.PoseStore(self.path).state()["count"], 0)
        self.assertEqual(arm_poses.PoseStore(self.tmp / "nope.json").state()["count"], 0)

    def test_legacy_list_and_missing_pose(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps([
            {"name": "老格式", "joints": READY},
            {"name": "坏的", "joints": [1, 2]},
            "根本不是记录",
        ]), encoding="utf-8")
        store = arm_poses.PoseStore(self.path)
        poses = store.state()["poses"]
        self.assertEqual([p["name"] for p in poses], ["老格式"])
        self.assertEqual(poses[0]["pose"], [round(v, 3) for v in kin.fk(READY)])

    def test_public_lists_are_copies(self):
        self.store.save("拷贝", READY)
        rec = self.store.get("拷贝")
        rec["joints"][0] = 123.0
        self.assertEqual(self.store.get("拷贝")["joints"][0], 0.0)


class TestHttp(unittest.TestCase):
    def setUp(self):
        self.tmp = _mkdtemp()
        self.hand = bus.open_sim()
        self.app = service.App(self.hand, root=self.tmp)
        self.path = self.tmp / "config" / "arm_poses.json"
        self.app.arm_poses = arm_poses.PoseStore(self.path)
        self.httpd = service.make_server(self.app, "127.0.0.1", 0)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.hand.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def call(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}", data=data, method=method,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as resp:
                return resp.status, json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def test_save_list_delete(self):
        code, body = self.call("POST", "/arm/poses",
                               {"name": "桌面抓取", "joints": READY, "note": "test"})
        self.assertEqual(code, 200, body)
        self.assertFalse(body["replaced"])
        self.assertEqual(body["pose"]["joints"], READY)

        code, body = self.call("GET", "/arm/poses")
        self.assertEqual(code, 200)
        self.assertEqual([p["name"] for p in body["poses"]], ["桌面抓取"])
        self.assertEqual(body["path"], str(self.path))
        self.assertTrue(self.path.is_file())

        code, body = self.call("POST", "/arm/poses/delete", {"name": "桌面抓取"})
        self.assertEqual(code, 200, body)
        self.assertEqual(body["count"], 0)
        self.assertEqual(self.call("GET", "/arm/poses")[1]["poses"], [])

    def test_empty_name_auto_numbers(self):
        code, body = self.call("POST", "/arm/poses", {"joints": READY})
        self.assertEqual(code, 200, body)
        self.assertEqual(body["pose"]["name"], "姿态 1")
        code, body = self.call("POST", "/arm/poses", {"joints": READY})
        self.assertEqual(body["pose"]["name"], "姿态 2")

    def test_without_joints_uses_current_pose(self):
        self.app.arm_sim_joints = list(READY)
        code, body = self.call("POST", "/arm/poses", {"name": "现在"})
        self.assertEqual(code, 200, body)
        self.assertEqual(body["pose"]["joints"], READY)
        self.assertEqual(body["pose"]["source"], "model")

    def test_bad_joints_rejected(self):
        self.assertEqual(self.call("POST", "/arm/poses", {"joints": [1, 2, 3]})[0], 400)
        self.assertEqual(self.call("POST", "/arm/poses",
                                   {"joints": [0, 0, 0, 0, 0, "x"]})[0], 400)

    def test_delete_missing_is_404(self):
        code, body = self.call("POST", "/arm/poses/delete", {"name": "没有"})
        self.assertEqual(code, 404, body)
        self.assertFalse(body["ok"])

    def test_page_has_save_button(self):
        code, html = self.call_html("/arm3d")
        self.assertEqual(code, 200)
        for text in ('id="btnSavePose"', "保存当前姿态", "/arm/poses",
                     'id="btnForce"', "强制下发"):
            self.assertIn(text, html)

    def call_html(self, path):
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}{path}", timeout=5) as resp:
            return resp.status, resp.read().decode("utf-8")


if __name__ == "__main__":
    unittest.main()
