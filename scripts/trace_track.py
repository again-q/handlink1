"""带着「仪表」下发：把 30004 的速度曲线录下来，用数字判断卡不卡。

为什么需要它：看得见的「一卡一卡」是速度掉到 0 又爬起来。眼睛看容易受主观影响，
但把 TCP 速度按 15 Hz 采下来，用「停顿次数」和「最低速度」就能比较两条轨迹。

用法::

    # 只看数据，不发车
    python scripts/trace_track.py --shape corner --cp 30 --speed 40
    # 真的下发并录制
    python scripts/trace_track.py --shape corner --cp 30 --speed 40 --apply --yes

输出：CSV（每次都写）+ PNG（装了 Pillow 才画）+ 终端摘要。

注意：它**不自己连 30004**，而是问服务要状态——控制柜对 30004 的并发连接很敏感，
再开一条容易把反馈口搞掉。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

BASE = "http://127.0.0.1:8766"

SHAPES = {
    "line": [[0.30, 0.50], [0.50, 0.50], [0.70, 0.50]],
    "corner": [[0.30, 0.60], [0.50, 0.60], [0.50, 0.40], [0.70, 0.40]],
    "circle": [[0.50 + 0.16 * math.cos(a * math.pi / 12),
                0.50 + 0.16 * math.sin(a * math.pi / 12)] for a in range(24)],
}


def http_json(url: str, payload: dict | None = None, timeout: float = 900.0) -> dict:
    if payload is None:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    req = urllib.request.Request(url, data=json.dumps(payload).encode("utf-8"),
                                 headers={"Content-Type": "application/json"},
                                 method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return json.loads(exc.read().decode("utf-8"))


def plot(samples, stops, out_png: Path) -> bool:
    """速度曲线画出来；没装 Pillow 就跳过。"""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return False
    if not samples:
        return False
    w, h = 900, 320
    img = Image.new("RGB", (w, h), (16, 18, 26))
    d = ImageDraw.Draw(img)
    d.rectangle((60, 20, w - 20, h - 50), outline=(60, 64, 80))
    d.text((64, 6), "TCP speed (mm/s) vs time", fill=(210, 214, 230))
    vs = [s[1] for s in samples]
    vmax = max(vs) or 1.0
    tmax = samples[-1][0] or 1.0
    pts = [(60 + (s[0] / tmax) * (w - 80), (h - 50) - (s[1] / vmax) * (h - 90))
           for s in samples]
    if len(pts) > 1:
        d.line(pts, fill=(120, 200, 255), width=2)
    for t0, _v in stops:
        x = 60 + (t0 / tmax) * (w - 80)
        d.line((x, 20, x, h - 50), fill=(240, 120, 120), width=1)
    d.text((64, h - 40), "max %.1f mm/s   stops %d" % (vmax, len(stops)),
           fill=(200, 204, 220))
    out_png.parent.mkdir(parents=True, exist_ok=True)
    img.save(out_png)
    return True


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shape", default="corner", choices=sorted(SHAPES))
    ap.add_argument("--speed", type=float, default=20.0)
    ap.add_argument("--cp", type=int, default=30)
    ap.add_argument("--r", type=float, default=0.0, help=">0 时用 r（毫米）代替 cp")
    ap.add_argument("--accel", type=int, default=0)
    ap.add_argument("--step", type=float, default=10.0)
    ap.add_argument("--hz", type=float, default=15.0, help="采样频率，默认 15 Hz")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--url", default=BASE)
    ap.add_argument("--out", default=".tmp/track")
    args = ap.parse_args()

    cfg = {"max_step_mm": args.step, "smooth_cp": args.cp,
           "blend_r_mm": max(0.0, args.r)}
    if args.accel:
        cfg["accel_pct"] = args.accel
    payload = {"points": SHAPES[args.shape], "apply": bool(args.apply),
               "speed": args.speed, "config": cfg}

    if args.apply and not args.yes:
        print("\n真机会动。确认现场安全。")
        try:
            if input("回车继续，其它任意字符取消：").strip():
                return 1
        except EOFError:
            print("非交互环境请加 --yes。")
            return 1

    result: dict = {}
    error: list[str] = []

    def send() -> None:
        try:
            result.update(http_json(args.url + "/arm/trace", payload))
        except Exception as exc:                     # noqa: BLE001
            error.append(str(exc))

    th = threading.Thread(target=send, daemon=True)
    t0 = time.time()
    th.start()

    samples = []
    last_speed = 0.0
    period = 1.0 / max(1.0, args.hz)
    while th.is_alive():
        try:
            st = http_json(args.url + "/arm/state", timeout=5.0)
            sp = float(st.get("tcp_speed_m_s") or 0.0) * 1000.0
            samples.append((round(time.time() - t0, 3), round(sp, 2)))
            last_speed = sp
        except Exception:                            # noqa: BLE001
            pass
        time.sleep(period)
    th.join(timeout=5.0)
    elapsed = time.time() - t0

    if error:
        print("请求失败：", error[0])
        return 1
    if not result.get("ok"):
        print("未下发：", result.get("reason") or result.get("error"))
        return 1

    # 停顿判定：先动起来（>5 mm/s），随后连续 3 个采样都 < 2 mm/s，算一次停顿
    stops = []
    armed = False
    slow_run = 0
    for t, v in samples:
        if v > 5.0:
            armed = True
            slow_run = 0
        elif armed and v < 2.0:
            slow_run += 1
            if slow_run == 3:
                stops.append((t, v))
        else:
            slow_run = 0

    speeds = [v for _t, v in samples]
    moving = [v for v in speeds if v > 1.0]
    dis = result.get("dispatch") or {}
    rep = result.get("report") or {}

    print("轨迹 %-6s 速度 %g%%  %s  点距 %.1f mm  采样 %d 点"
          % (args.shape, args.speed,
             ("r=%gmm" % args.r) if args.r > 0 else ("cp=%d" % args.cp),
             args.step, len(samples)))
    print("下发 %d 点  实际耗时 %.1f s  规划预计 %.1f s" %
          (dis.get("points", 0), elapsed, rep.get("duration_s", 0)))
    if speeds:
        print("TCP 速度：最大 %.1f  平均(运动中) %.1f  最低(运动中) %.1f mm/s"
              % (max(speeds), sum(moving) / len(moving) if moving else 0.0,
                 min(moving) if moving else 0.0))
    print("停顿（速度掉到 <2 mm/s 连续 3 帧）：%d 次" % len(stops))
    if result.get("timings"):
        print("分段计时：", "  ".join("%s=%.1fs" % (k, v)
                                    for k, v in result["timings"].items()))
    if result.get("blend_warning"):
        print("提示：", result["blend_warning"])
    if result.get("warnings"):
        print("提示：", "；".join(result["warnings"]))

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    csv_path = out.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "tcp_speed_mm_s"])
        w.writerows(samples)
    png = out.with_suffix(".png")
    if plot(samples, stops, png):
        print("写了", csv_path, "和", png)
    else:
        print("写了", csv_path, "（没装 Pillow，没画图）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
