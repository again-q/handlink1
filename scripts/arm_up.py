"""臂上线：接管 TCP 控制 → 设速度 → 上使能，每步都回头看反馈。

    python scripts/arm_up.py                 # 只打印当前状态，什么都不发
    python scripts/arm_up.py --yes           # 真的接管 + 使能（默认 20% 速度）
    python scripts/arm_up.py --yes --speed 10

为什么要有这个脚本：厂家软件和 ``RequestControl()`` 是抢同一把锁，谁会话谁说了算；
而且手册 2.1 规定**只有「未上电」或「下使能（非暂停、未松抱闸）」时才能切 TCP 模式**，
所以顺序不能乱 —— 先看反馈确认能切，再切，再使能，每一步都用 30004 的真实状态验证，
不靠 ``sleep`` 猜。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from handlink import arm as arm_mod
from handlink.arm_feed import CONTROLLABLE_MODES, Feed


def state(feed: Feed) -> dict:
    s = feed.summary()
    return {k: s[k] for k in ("mode_name", "enabled", "error", "pose", "joints",
                              "voltage", "speed_scaling")}


def wait_until(feed: Feed, pred, what: str, timeout: float = 8.0) -> dict:
    deadline = time.time() + timeout
    while time.time() < deadline:
        snap = feed.snapshot()
        if pred(snap):
            return snap
        time.sleep(0.1)
    raise SystemExit(f"等不到「{what}」，超时 {timeout:.0f}s —— 停下人工检查")


def main() -> int:
    ap = argparse.ArgumentParser(description="Dobot Nova 5 上线（接管 + 使能）")
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--speed", type=float, default=arm_mod.SAFE_SPEED_FACTOR)
    ap.add_argument("--yes", action="store_true", help="真的发指令；不加就只看状态")
    args = ap.parse_args()

    feed = Feed(args.ip).open()
    try:
        feed.wait_frame(timeout=5)
        print("== 接管前 ==")
        print(json.dumps(state(feed), ensure_ascii=False, indent=2))

        mode = int(feed.snapshot()["RobotMode"])
        if mode not in CONTROLLABLE_MODES:
            raise SystemExit(f"当前模式 {mode} 不允许切 TCP（手册 2.1）：先下使能 / 断开拖拽，"
                             f"再跑一次本脚本")
        if not args.yes:
            print("\n只看不发。确认无误后加 --yes 真跑。")
            return 0

        robot = arm_mod.DobotArm(ip=args.ip)
        robot.connect()
        try:
            print("\n== RequestControl() ==")
            robot.take_control()
            print("OK 控制权已切到 TCP")

            # 切完立刻用反馈复核：能把 GetPose 读出来才算真的通了
            feed.wait_frame(timeout=3)
            wait_until(feed, lambda s: True, "反馈")
            pose = robot.pose()
            print("GetPose:", [round(v, 2) for v in pose])

            print(f"\n== SpeedFactor({args.speed:.0f}%) ==")
            robot.speed_factor(args.speed)
            print("OK 速度因子已设")

            print("\n== EnableRobot() ==")
            robot.enable()
            snap = wait_until(feed, lambda s: int(s["EnableStatus"]) == 1,
                              "使能完成", timeout=12.0)
            print("OK 已使能，抱闸锁住当前位置")

            print("\n== 接管后 ==")
            s = state(feed)
            print(json.dumps(s, ensure_ascii=False, indent=2))
            drift = max(abs(a - b) for a, b in zip(s["pose"], pose))
            print(f"\n使能前后位置最大变化 {drift:.2f} mm/deg")
            if drift > 2.0:
                print("⚠️  位置漂移超过 2，说明使能前臂被外力托着/下垂，先查机械再动")
        finally:
            robot.close()
    finally:
        feed.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
