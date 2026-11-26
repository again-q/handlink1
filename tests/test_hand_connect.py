"""连接机械手：/hand/status、/hand/attach 的离线自测（不碰真硬件）."""
from __future__ import annotations

import json
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from handlink import bus, service, webui

ROOT = Path(__file__).resolve().parent.parent


class TestHandConnect(unittest.TestCase):
    def setUp(self):
        self.sim = bus.open_sim()
        self.app = service.App(self.sim)
        self.httpd = service.make_server(self.app, "127.0.0.1", 0)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def tearDown(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        self.sim.close()

    def call(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}", data=data, method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(req, timeout=15) as resp:
                return resp.status, json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            return exc.code, json.loads(exc.read().decode("utf-8"))

    def test_sim_is_not_a_connected_hand(self):
        code, body = self.call("GET", "/hand/status")
        self.assertEqual(code, 200, body)
        self.assertTrue(body["ok"])
        self.assertEqual(body["bus"], "sim")
        self.assertFalse(body["connected"], "仿真总线不能算连上了真手")
        self.assertIn("仿真", body["detail"])

    def test_attach_sim_is_ok(self):
        code, body = self.call("POST", "/hand/attach", {"bus": "sim"})
        self.assertEqual(code, 200, body)
        self.assertTrue(body["ok"])
        self.assertEqual(body["bus"], "sim")

    def test_attach_missing_dll_reports_and_falls_back(self):
        code, body = self.call("POST", "/hand/attach", {
            "bus": "can",
            "dll": str(ROOT / "no-such-ECanVci64.dll"),
        })
        self.assertEqual(code, 422, body)
        self.assertFalse(body["ok"])
        self.assertEqual(body["bus"], "sim")
        self.assertIn("连不上", body["detail"])
        # 连接失败不能把服务弄死：状态还能读，bus 还在
        code, again = self.call("GET", "/hand/status")
        self.assertEqual(code, 200, again)
        self.assertEqual(again["bus"], "sim")

    def test_health_reports_hand(self):
        code, body = self.call("GET", "/health")
        self.assertEqual(code, 200)
        self.assertIn("hand", body)
        self.assertFalse(body["hand"]["connected"])


class TestDemoPageHasLinkButton(unittest.TestCase):
    def test_button_and_endpoints_in_page(self):
        self.assertIn('id="btnLink"', webui.HTML)
        self.assertIn("连接机械手", webui.HTML)
        self.assertIn("/hand/status", webui.HTML)
        self.assertIn("/hand/attach", webui.HTML)
        self.assertIn('id="linkMsg"', webui.HTML)


if __name__ == "__main__":
    unittest.main()
