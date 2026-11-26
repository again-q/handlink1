"""石头剪刀布：人出拳，灵巧手跟着出。社团展台最省事的一个演示。

两种跑法：

    python demo/rps.py                 # 内置仿真，直接玩（键盘输入）
    python demo/rps.py --server        # 连正在跑的 handlink 服务

摄像头识别（MediaPipe）还没接，接口留好了：把 classify() 换成从相机读即可。
"""
from __future__ import annotations

import argparse
import json
import random
import sys
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from handlink import bus, service, spec  # noqa: E402

GESTURES = {
    "rock": ("石头", "rock"),
    "paper": ("布", "paper"),
    "scissors": ("剪刀", "scissors"),
}
BEATS = {"rock": "scissors", "scissors": "paper", "paper": "rock"}  # key 赢 value


class Hand:
    """离线/在线两种实现，对外都是 point(name)。"""

    def __init__(self, server: str | None = None):
        self.server = server
        self.app = None
        if server:
            self.base = server.rstrip("/")
        else:
            self.bus = bus.open_sim()
            self.app = service.App(self.bus)
            self.base = None

    def point(self, name: str) -> None:
        if self.base:
            body = json.dumps({"name": name}).encode()
            req = urllib.request.Request(self.base + "/point", data=body, method="POST",
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=5) as resp:
                resp.read()
        else:
            self.app.apply_point(name)

    def settle(self, seconds: float = 0.6) -> None:
        if self.app:
            self.bus.settle(timeout=seconds)

    def close(self) -> None:
        if self.app:
            self.bus.close()


def classify(text: str):
    """把输入变成手势。摄像头版就把这里换成模型输出。"""
    t = text.strip().lower()
    if t in ("r", "rock", "石头", "1"):
        return "rock"
    if t in ("p", "paper", "布", "2"):
        return "paper"
    if t in ("s", "scissors", "剪刀", "3"):
        return "scissors"
    return None


def judge(mine: str, theirs: str) -> str:
    if mine == theirs:
        return "draw"
    return "win" if BEATS[mine] == theirs else "lose"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", nargs="?", const="http://127.0.0.1:8766", default=None,
                    help="连到已运行的 handlink；不带值默认 127.0.0.1:8766")
    ap.add_argument("--rounds", type=int, default=5)
    ap.add_argument("--auto", action="store_true", help="两边都随机，自测用")
    args = ap.parse_args()

    hand = Hand(args.server)
    scores = {"win": 0, "lose": 0, "draw": 0}
    print("=" * 46)
    print("  灵巧手 · 石头剪刀布      三局两胜制")
    print("  出拳：r 石头 / p 布 / s 剪刀     退出：q")
    print("=" * 46)
    try:
        for i in range(1, args.rounds + 1):
            if args.auto:
                mine = random.choice(list(GESTURES))
            else:
                while True:
                    raw = input(f"\n第 {i} 回合，你出：").strip()
                    if raw.lower() in ("q", "quit", "exit"):
                        print("结束。")
                        return 0
                    mine = classify(raw)
                    if mine:
                        break
                    print("  只认 r / p / s")
            theirs = random.choice(list(GESTURES))

            hand.point("paper")
            time.sleep(0.25)
            hand.point(mine)
            hand.settle(1.0)

            result = judge(mine, theirs)
            scores[result] += 1
            face = {"win": "你赢", "lose": "手赢", "draw": "平"}[result]
            print(f"  你 {GESTURES[mine][0]}  vs  手 {GESTURES[theirs][0]}   →  {face}")
            print(f"  比分   你 {scores['win']} : {scores['lose']} 手   （平 {scores['draw']}）")

            if scores["win"] >= 2 or scores["lose"] >= 2:
                break
        print("\n" + ("你赢了 🎉" if scores["win"] > scores["lose"] else
                      "手赢了" if scores["lose"] > scores["win"] else "打平"))
    finally:
        hand.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
