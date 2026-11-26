"""最小动作测试：只动一个关节 delta 度，再原路回位。

第一次让真机动起来用它 —— 行程小、方向明确、全程用 30004 反馈判到位，
超时立刻 Stop()，不管成没成功都回原位。

    python scripts/arm_nudge.py --joint 1 --delta 3        # 只打印计划，不动
    python scripts/arm_nudge.py --joint 1 --delta 3 --yes  # 真动

J1 是绕底座转，位移最小、不往任何方向伸出；第一次不要用 J2/J3（会甩起来）。
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

ARRIVE_TOL_DEG = 0.3
STABLE_SAMPLES = 3


def wait_arrive(feed: Feed, target: list, timeout: float) -> bool:
    """连续 STABLE_SAMPLES 次采样都在容差内才算到位（避开采样抖动）。"""
    deadline = time.time() + timeout
    hits = 0
    while time.time() < deadline:
        joints = [float(v) for v in feed.snapshot()["QActual"]]
        if all(abs(joints[i] - target[i]) <= ARRIVE_TOL_DEG for i in range(6)):
            hits += 1
            if hits >= STABLE_SAMPLES:
                return True
        else:
            hits = 0
        time.sleep(0.08)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="把某一个关节转 delta 度再回来")
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--joint", type=int, default=1, choices=range(1, 7))
    ap.add_argument("--delta", type=float, default=3.0, help="转动角度，正负决定方向")
    ap.add_argument("--speed", type=float, default=20.0)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--keep", action="store_true", help="动完停在目标点，不回原位")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    if abs(args.delta) > 10:
        raise SystemExit("第一次别超过 10°，先用 --delta 3 确认方向")

    feed = Feed(args.ip).open()
    robot = arm_mod.DobotArm(ip=args.ip)
    try:
        feed.wait_frame(timeout=5)
        snap = feed.summary()
        if not snap["enabled"]:
            raise SystemExit("臂未使能：先跑 scripts/arm_up.py --yes")
        if snap["error"]:
            raise SystemExit(f"有报警（{snap['error']}），先 ClearError 再动")

        origin = [float(v) for v in snap["joints"]]
        target = list(origin)
        target[args.joint - 1] += args.delta
        plan = {
            "joint": f"J{args.joint}",
            "delta_deg": args.delta,
            "from": [round(v, 3) for v in origin],
            "to": [round(v, 3) for v in target],
            "speed_factor": args.speed,
        }
        print(json.dumps(plan, ensure_ascii=False, indent=2))
        if not args.yes:
            print("\n只看不动。确认方向对了再加 --yes。")
            return 0

        robot.connect()
        robot.speed_factor(args.speed)

        def go(point: list, what: str) -> bool:
            t0 = time.time()
            robot.move_joints(point, wait=False)
            ok = wait_arrive(feed, point, args.timeout)
            dt = time.time() - t0
            moved = [round(float(v), 3) for v in feed.snapshot()["QActual"]]
            print(f"{what}: {'到位' if ok else '超时'} 用时 {dt:.2f}s  实际 {moved}")
            if not ok:
                robot.stop()
            return ok

        ok = False
        pose_origin = list(snap["pose"])
        nudge_mm = 0.0
        try:
            ok = go(target, f"J{args.joint} {args.delta:+.1f}°")
            pose_at_target = feed.summary()["pose"]
            nudge_mm = sum((pose_at_target[i] - pose_origin[i]) ** 2
                           for i in range(3)) ** 0.5
        finally:
            if not args.keep:
                go(origin, "回原位")

        back = feed.summary()["pose"]
        drift = sum((back[i] - pose_origin[i]) ** 2 for i in range(3)) ** 0.5
        print(f"\n这一下末端移动了 {nudge_mm:.1f} mm")
        print(f"回到原位后偏差 {drift:.2f} mm（重复定位精度标称 ±0.05 mm）")
        print(f"末态：{feed.summary()['mode_name']}，报警 {feed.summary()['error']}")
        return 0 if ok and drift < 1.0 else 2
    finally:
        robot.close()
        feed.close()


if __name__ == "__main__":
    raise SystemExit(main())
