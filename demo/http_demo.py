"""HTTP 接口演示：起服务 -> 调接口 -> 打印 -> 关闭。

    python demo/http_demo.py
"""
from __future__ import annotations

import json
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from handlink import bus, recorder, service  # noqa: E402


def call(port: int, method: str, path: str, body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}", data=data, method=method,
        headers={"Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def main() -> int:
    sim = bus.open_sim()
    app = service.App(sim)
    httpd = service.make_server(app, "127.0.0.1", 0)
    port = httpd.server_address[1]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    out = HERE / "http.jsonl"
    print(f"# handlink 已启动: http://127.0.0.1:{port}\n")
    try:
        code, body = call(port, "GET", "/health")
        print(f"GET  /health              -> {code}  bus={body['bus']}  电机={body['motors']}")

        code, body = call(port, "GET", "/limits")
        print(f"GET  /limits              -> {code}  " +
              ", ".join(f"{m['name']}:{m['pos_max']:.0f}°/{m['current_max']}A" for m in body["motors"]))

        code, body = call(port, "POST", "/pose", {
            "motors": [{"motorId": 103, "angle": 120, "speed": 60, "torque": 0.5}],
            "reason": "demo",
        })
        print(f"POST /pose 食指=120°       -> {code}  ok={body['ok']}  其余保持={body['held']}")
        time.sleep(0.4)

        code, body = call(port, "POST", "/pose", {
            "motors": [{"motorId": 102, "angle": 999, "speed": 60, "torque": 0.5}],
        })
        print(f"POST /pose 拇指1=999°(越界) -> {code}  {body['issues'][0]['message']}")

        code, body = call(port, "POST", "/point", {"name": "rock"})
        print(f"POST /point rock          -> {code}  ok={body['ok']}")
        time.sleep(0.6)

        code, body = call(port, "POST", "/track", {"name": "动作一", "dry_run": True})
        print(f"POST /track 动作一(预览)   -> {code}  {len(body['timeline'])} 步 / {body['duration_ms']} ms")

        code, body = call(port, "POST", "/record/start", {"path": str(out), "hz": 20})
        print(f"POST /record/start        -> {code}  {body['path']}")
        call(port, "POST", "/pose", {"motors": [{"motorId": 106, "angle": 150, "speed": 120, "torque": 0.5}]})
        time.sleep(1.0)
        code, body = call(port, "POST", "/record/stop")
        stats = body["stats"]
        print(f"POST /record/stop         -> {code}  采样={stats['samples']} 帧 / {stats['duration_s']} s")
        print()
        print(recorder.to_markdown(stats))
    finally:
        httpd.shutdown()
        httpd.server_close()
        sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
