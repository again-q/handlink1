"""不下发直连测试：用一条**固定的**短轨迹，在真机上比 平滑 / 点距 / 速度。

用途：现场判断「又慢又卡、桌子震」到底是哪个参数造成的。每次都发同一条几何轨迹，
只改一个参数，才有可比性——手绘两次画的线不一样，量出来的差别分不清是参数还是笔迹。

它还会**计时**：打印「实际耗时 / 规划预计」，这两个数差得远就说明卡在别的地方。

用法::

    python scripts/trace_send.py                                  # 只看规划
    python scripts/trace_send.py --shape line   --step 10 --speed 20 --apply
    python scripts/trace_send.py --shape corner --r 3 --accel 50 --speed 20 --apply

参数含义（对应 Dobot 手册 MovL 的命名参数）：

* ``--cp``     平滑过渡**比例**（0~100）。百分比，速度越低圆出来的角越小。
* ``--r``      平滑过渡**半径**（毫米）。绝对值，慢速时更靠得住；给了它就优先于 cp。
* ``--accel``  加速度比例 ``a``（1~100）。默认很低时，5 mm 的小段几乎全程在加减速。
* ``--step``   点距（毫米）。越小越密，指令越多，越容易一卡一卡。
* ``--speed``  速度因子 %（控制器 SpeedFactor）。

安全：不带 ``--apply`` 永远不会动臂；带了会先打印参数再问一次。
"""
from __future__ import annotations

import argparse
import json
import math
import time
import urllib.error
import urllib.request

BASE = "http://127.0.0.1:8766"

SHAPES = {
    # 三点直线：没有拐角，平滑参数对它没有影响 —— 它是「机械/伺服基线」
    "line": [[0.30, 0.50], [0.50, 0.50], [0.70, 0.50]],
    # 一个直角拐角：看平滑有没有把角磨圆、有没有在角上停一下
    "corner": [[0.30, 0.60], [0.50, 0.60], [0.50, 0.40], [0.70, 0.40]],
    # 小圆：连续曲率，最容易看出高频抖动
    "circle": [[0.50 + 0.16 * math.cos(a * math.pi / 12),
                0.50 + 0.16 * math.sin(a * math.pi / 12)] for a in range(24)],
}


def post(payload: dict, base: str) -> tuple[dict, float]:
    req = urllib.request.Request(
        base + "/arm/trace",
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    t0 = time.time()
    try:
        with urllib.request.urlopen(req, timeout=900) as resp:
            return json.loads(resp.read().decode("utf-8")), time.time() - t0
    except urllib.error.HTTPError as exc:      # 422 带着失败原因
        return json.loads(exc.read().decode("utf-8")), time.time() - t0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shape", default="line", choices=sorted(SHAPES))
    ap.add_argument("--speed", type=float, default=10.0, help="速度因子 %%，默认 10")
    ap.add_argument("--cp", type=int, default=30, help="平滑比例 0~100，默认 30")
    ap.add_argument("--r", type=float, default=3.0,
                    help="平滑半径 mm（>0 时优先于 cp）；0 表示改用 cp")
    ap.add_argument("--accel", type=int, default=0, help="加速度比例 a（0=不下发）")
    ap.add_argument("--step", type=float, default=5.0, help="点距 mm，默认 5")
    ap.add_argument("--apply", action="store_true", help="真的下发（默认只规划）")
    ap.add_argument("--yes", action="store_true", help="带 --apply 时不再问确认（脚本自动化用）")
    ap.add_argument("--url", default=BASE)
    args = ap.parse_args()

    cfg = {"max_step_mm": args.step, "smooth_cp": args.cp,
           "blend_r_mm": max(0.0, args.r)}       # 0 才是「只用 cp」
    if args.accel:
        cfg["accel_pct"] = args.accel

    blend = ("r=%gmm" % args.r) if args.r > 0 else ("cp=%d" % args.cp)
    print("轨迹 %-6s 速度 %g%%  平滑 %-8s 加速度 %-3s 点距 %.1f mm  下发=%s"
          % (args.shape, args.speed, blend,
             ("%d%%" % args.accel) if args.accel else "默认",
             args.step, "是" if args.apply else "否（只看）"))
    if args.apply and not args.yes:
        print("\n真机会动。确认：工作空间里没人、纸面已空、急停在手边。")
        try:
            if input("回车继续，输入其它任意字符取消：").strip():
                print("已取消。")
                return 1
        except EOFError:                    # 非交互环境没给 --yes 就别动
            print("没有终端确认，已取消（自动化请加 --yes）。")
            return 1

    out, elapsed = post({"points": SHAPES[args.shape], "apply": bool(args.apply),
                         "speed": args.speed, "config": cfg}, args.url)
    if not out.get("ok"):
        print("失败：", out.get("reason") or out.get("error"))
        return 1

    rep = out.get("report") or {}
    dis = out.get("dispatch") or {}
    print("点数 %d  纸面长度 %.1f mm  规划预计 %.1f s"
          % (rep.get("n_points", 0), rep.get("length_mm", 0), rep.get("duration_s", 0)))
    print("反解失败 %d  最大关节跳变 %.2f°  J6 净转 %.1f°"
          % (rep.get("failed", 0), rep.get("max_joint_step_deg", 0),
             rep.get("j6_net_deg", 0)))
    if dis:
        print("下发：%d 点  平滑 %s  加速度 %s  窗口 %s  限流 %s"
              % (dis.get("points", 0), dis.get("blend"), dis.get("accel") or "默认",
                 dis.get("window"), dis.get("throttle")))
        print("实际耗时 %.1f s（含归位 + 起点 + 收笔等待）" % elapsed)
        print("  比规划慢 %.1f 倍" % (elapsed / max(0.1, rep.get("duration_s", 1))))
    else:
        print("（只规划，没下发）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
