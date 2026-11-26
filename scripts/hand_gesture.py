"""给真手做手势。角度/速度/电流都是显式表，不猜。

    python scripts/hand_gesture.py show open fist open
    python scripts/hand_gesture.py fist
    python scripts/hand_gesture.py count        # 1-2-3 数数
    python scripts/hand_gesture.py wave         # 四指依次弯下再张开
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from handlink import screw  # noqa: E402
from handlink.canbus import Hand  # noqa: E402

SPEED = 200.0
CURRENT = 1.0  # 0.6A 时多指同时运动会 I-LIM 卡住，1.0A 稳定

# motor -> angle（速度/电流统一用 SPEED / CURRENT）
POSES: dict[str, dict[int, float]] = {
    "open": {102: 0, 107: 0, 103: 0, 104: 0, 105: 0, 106: 0},
    "fist": {102: 50, 107: 40, 103: 140, 104: 140, 105: 140, 106: 140},
    "point": {102: 50, 107: 40, 103: 0, 104: 140, 105: 140, 106: 140},
    "pinch": {102: 45, 107: 30, 103: 55, 104: 10, 105: 10, 106: 10},
    "one": {102: 50, 107: 40, 103: 0, 104: 140, 105: 140, 106: 140},
    "two": {102: 50, 107: 40, 103: 0, 104: 0, 105: 140, 106: 140},
    "three": {102: 50, 107: 40, 103: 0, 104: 0, 105: 0, 106: 140},
}

ORDER = [103, 104, 105, 106, 102, 107]


def apply_pose(h: Hand, pose: dict[int, float], stagger: float = 0.05) -> None:
    for motor in ORDER:
        if motor in pose:
            h.move(motor, pose[motor], speed=SPEED, current=CURRENT)
            time.sleep(stagger)


def wait_all(h: Hand, pose: dict[int, float], timeout: float = 4.0) -> dict[int, float]:
    """等每个关节进入目标 ±3°，返回最终读数。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        for motor in pose:
            h.send(screw.get_position(motor))
        time.sleep(0.15)
        got = {m: h.snap.positions.get(m) for m in pose}
        if all(v is not None and abs(v - pose[m]) <= 3.0 for m, v in got.items()):
            break
    for motor in pose:
        h.send(screw.get_position(motor))
    time.sleep(0.15)
    return {m: h.snap.positions.get(m) for m in pose}


def report(title: str, got: dict[int, float]) -> None:
    body = "  ".join(f"{m}={0.0 if v is None else v:.1f}°" for m, v in got.items())
    print(f"  {title:<6} {body}", flush=True)


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("action", choices=["show", "count", "wave", *POSES])
    ap.add_argument("poses", nargs="*")
    ap.add_argument("--hand", type=lambda s: int(s, 0), default=0)
    ap.add_argument("--timeout", type=float, default=4.0)
    args = ap.parse_args(argv[1:])

    with Hand(hand_id=args.hand or None) as h:
        print(f"hand=0x{h.hand_id:02X}", flush=True)
        if args.action == "show":
            for name in args.poses or ["open", "fist", "open"]:
                apply_pose(h, POSES[name])
                report(name, wait_all(h, POSES[name], args.timeout))
        elif args.action == "count":
            for name in ("one", "two", "three"):
                apply_pose(h, POSES[name])
                report(name, wait_all(h, POSES[name], args.timeout))
                time.sleep(0.4)
            apply_pose(h, POSES["open"])
            report("open", wait_all(h, POSES["open"], args.timeout))
        elif args.action == "wave":
            for _ in range(3):
                for motor in (103, 104, 105, 106):
                    h.move(motor, 120, speed=SPEED, current=CURRENT)
                    time.sleep(0.10)
                time.sleep(0.25)
                for motor in (103, 104, 105, 106):
                    h.move(motor, 0, speed=SPEED, current=CURRENT)
                    time.sleep(0.10)
                time.sleep(0.25)
            print("wave done", flush=True)
        else:
            apply_pose(h, POSES[args.action])
            report(args.action, wait_all(h, POSES[args.action], args.timeout))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
