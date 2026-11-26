"""收工：停运动 → 下使能（抱闸锁住）→ 关连接。

和 ``arm_up.py`` 成对。下使能不是「断电」——电机不再出力，但抱闸抱住，姿态保持不动，
适合人离开、但控制柜还通着电的时候。真正断电前先跑这个。

    python scripts/arm_down.py            # 只读当前状态
    python scripts/arm_down.py --yes      # 真的停 + 下使能
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from handlink import arm as arm_mod
from handlink.arm_feed import Feed


def main() -> int:
    ap = argparse.ArgumentParser(description="Dobot Nova 5 安全收工")
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    feed = Feed(args.ip).open()
    try:
        feed.wait_frame(timeout=5)
        snap = feed.summary()
        print("== 收回前 ==")
        print(json.dumps({k: snap[k] for k in ("mode_name", "enabled", "brakes", "running",
                                              "error", "pose")}, ensure_ascii=False, indent=2))
        if not args.yes:
            print("\n只看不动。加 --yes 执行 Stop() + DisableRobot()。")
            return 0

        robot = arm_mod.DobotArm(ip=args.ip)
        robot.connect()
        try:
            robot.stop()
            print("Stop() —— 已停止当前运动")
            time.sleep(0.3)
            if snap["enabled"]:
                robot.disable()
                print("DisableRobot() —— 已下使能")
            else:
                print("本来就是下使能，跳过 DisableRobot()")
        finally:
            robot.close()

        deadline = time.time() + 10
        while time.time() < deadline:
            s = feed.snapshot()
            if int(s["EnableStatus"]) == 0:
                break
            time.sleep(0.1)
        after = feed.summary()
        print("\n== 收回后 ==")
        print(json.dumps({k: after[k] for k in ("mode_name", "enabled", "brakes", "error",
                                               "pose", "joints")}, ensure_ascii=False, indent=2))
        if after["brakes"]:
            print("抱闸已抱住，姿态保持不动")
        elif not after["enabled"]:
            print("⚠️  下使能了但抱闸状态是 0：如果臂是靠重力撑住的，别松手")
        return 0
    finally:
        feed.close()


if __name__ == "__main__":
    raise SystemExit(main())
