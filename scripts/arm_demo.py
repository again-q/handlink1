"""让真机动起来：接管 → 使能 → 走一段看得见的动作 → 回原位。

    python scripts/arm_demo.py            # 只打印要走的动作，不动
    python scripts/arm_demo.py --yes      # 真跑（默认 20% 速度）

动作是「慢速、小幅度、始终在原点附近」：J1 左右各摆 15°、手腕 J5 上下各 25°、
肘 J3 收放 20°，最后回零。全程用 30004 反馈判到位，任何一步超时就 Stop()。
第一次让人看机械臂动，就用这个；比 arm_nudge.py 更有观赏性，但同样不含大行程。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from handlink import arm as arm_mod
from handlink.arm_feed import CONTROLLABLE_MODES, FeedbackError, Feed

TOL_DEG = 0.4


def wait_arrive(feed: Feed, target, timeout: float) -> bool:
    deadline = time.time() + timeout
    hits = 0
    while time.time() < deadline:
        j = [float(v) for v in feed.snapshot()["QActual"]]
        if all(abs(j[i] - target[i]) <= TOL_DEG for i in range(6)):
            hits += 1
            if hits >= 3:
                return True
        else:
            hits = 0
        time.sleep(0.08)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="机械臂动作演示")
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--speed", type=float, default=20.0)
    ap.add_argument("--deg", type=float, default=15.0, help="J1 摆动幅度")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    try:
        feed = Feed(args.ip).open()
    except FeedbackError as exc:
        print(f"臂不在线：{exc}\n"
              "检查三件事：① 连臂的 USB 网卡插回电脑（系统里会出现「以太网 2」，\n"
              "静态 IP 192.168.5.100 会自动回来）；② 控制柜上电、蓝灯常亮；\n"
              "③ 网线插在控制柜 LAN 口。就绪后再跑一次本脚本。", file=sys.stderr)
        return 2

    try:
        feed.wait_frame(timeout=5)
        start = [float(v) for v in feed.snapshot()["QActual"]]
        steps = []
        for name, idx, delta in (("J1 左", 0, +args.deg), ("J1 右", 0, -2 * args.deg),
                                 ("J1 中", 0, +args.deg),
                                 ("手腕上", 4, +25.0), ("手腕下", 4, -50.0), ("手腕中", 4, +25.0),
                                 ("肘部收", 2, +20.0), ("肘部放", 2, -20.0)):
            target = list(start)
            target[idx] += delta
            steps.append({"name": name, "joints": [round(v, 2) for v in target]})

        print(json.dumps({"速度": args.speed, "起点": [round(v, 2) for v in start],
                          "动作": [s["name"] for s in steps]}, ensure_ascii=False, indent=2))
        if not args.yes:
            print("\n只看不动。确认工作空间里没人后加 --yes 真跑。")
            return 0

        desc = feed.summary()
        if desc["mode"] not in CONTROLLABLE_MODES and not desc["enabled"]:
            raise SystemExit(f"当前模式「{desc['mode_name']}」既不能接管也不能动，先看 arm_up.py")

        robot = arm_mod.DobotArm(ip=args.ip)
        robot.connect()
        try:
            if not desc["enabled"]:
                robot.take_control()
                print("已接管 TCP 控制")
                robot.enable()
                time.sleep(1.0)
            robot.speed_factor(args.speed)
            print(f"速度因子 {args.speed:.0f}%")

            for step in steps:
                robot.move_joints(step["joints"], wait=False)
                ok = wait_arrive(feed, step["joints"], timeout=12.0)
                print(f"  {step['name']:8} {'到位' if ok else '超时 → Stop()'}")
                if not ok:
                    robot.stop()
                    break
            robot.move_joints(start, wait=False)
            back = wait_arrive(feed, start, timeout=12.0)
            print(f"  回零位   {'到位' if back else '超时'}")
        finally:
            robot.close()
        snap = feed.summary()
        print(f"\n末态：{snap['mode_name']}，报警 {snap['error']}")
        return 0
    finally:
        feed.close()


if __name__ == "__main__":
    raise SystemExit(main())
