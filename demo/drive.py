"""通过 HTTP 驱动仿真手：石头 → 剪刀 → 布 → 半握，并录一段动作。

先起服务（另一个窗口）::

    python -m handlink serve --port 8766
    python demo/drive.py
"""
from __future__ import annotations

import json
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8766"
NAMES = {102: "拇指1", 107: "拇指2", 103: "食指", 104: "中指", 105: "无名指", 106: "小指"}
ORDER = [102, 107, 103, 104, 105, 106]


def call(method: str, path: str, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status, json.loads(resp.read().decode())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode())


def width(s: str) -> int:
    return sum(2 if ord(ch) > 127 else 1 for ch in s)


def pad(s: str, n: int) -> str:
    return s + " " * max(0, n - width(s))


def show(title: str) -> None:
    _, snap = call("GET", "/state")
    print(f"  {title}")
    print("    " + pad("手指", 8) + pad("位置", 9) + pad("电流", 8) + "电流上限")
    for mid in ORDER:
        st = snap["motors"][str(mid)]
        print("    " + pad(NAMES[mid], 8) + pad(f"{st['angle']:.1f}°", 9)
              + pad(f"{st['current']:.2f}A", 8) + f"{st['current_limit']}A")
    print()


def main() -> int:
    print(f"# 通过 {BASE} 控制（仿真总线）\n")
    code, _ = call("POST", "/point", {"name": "paper"})
    time.sleep(0.6)
    print("> 张开（paper 点位）")

    for name, label in (("rock", "石头"), ("scissors", "剪刀"), ("paper", "布")):
        code, body = call("POST", "/point", {"name": name})
        time.sleep(0.55)
        show(f"> {label}（POST /point {name} -> {code}）")

    grip = {102: 60, 107: 20, 103: 90, 104: 100, 105: 110, 106: 120}
    code, body = call("POST", "/pose", {
        "motors": {str(k): {"angle": v, "speed": 180, "torque": 0.4} for k, v in grip.items()},
        "reason": "drive:半握",
    })
    time.sleep(1.1)
    show(f"> 半握（POST /pose -> {code}，held={body.get('held')}）")

    print("> 录制一段：食指从 90° 开到 170°")
    rec_path = f"demo/drive-{time.strftime('%H%M%S')}.jsonl"
    call("POST", "/record/start", {"path": rec_path, "hz": 25})
    call("POST", "/pose", {"motors": [{"motorId": 103, "angle": 170, "speed": 120, "torque": 0.5}]})
    time.sleep(1.0)
    code, body = call("POST", "/record/stop")
    stats = body["stats"]
    print(f"  采样 {stats['samples']} 帧 / {stats['duration_s']}s\n")
    print("    " + pad("手指", 8) + pad("行程", 9) + pad("峰值速度", 10) + pad("峰值电流", 10) + "电流占用")
    for mid in ORDER:
        m = stats["motors"].get(str(mid))
        if not m:
            continue
        print("    " + pad(NAMES[mid], 8) + pad(f"{m['rom']}°", 9) + pad(str(m["speed_max"]), 10)
              + pad(f"{m['current_max']}A", 10) + f"{m['current_peak_ratio'] * 100:.0f}%")
    print(f"\n  录制文件: {rec_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


