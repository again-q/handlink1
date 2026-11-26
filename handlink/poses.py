"""预设动作组：角度表全部来自真机实测（不是估的）。

速度 200 ≈ 55°/s，电流 1.0 A —— 低于 0.8 A 时多指同时动会触发 I-LIM 卡在中途。

**合手顺序**：五指同时合会互相顶住——2026-11-19 实测，拇指2（107）卡在 10°
（0.8 A 顶死）、四指卡在 90–110°（各 1.0 A 顶死），握不实。所以合手分两段：
先让拇指1（102）单独走到「下位」:data:`THUMB_LOW`，停稳后再合其余五指。
"""
from __future__ import annotations

import time
from typing import Dict, List

from . import spec

SPEED = 200.0
CURRENT = 1.0

#: 拇指1（102）的「下位」：合手时它先单独走到这里，把拇指让开
THUMB_LOW = 50.0
#: 合手分几段、按什么顺序（姿态名，见 :data:`POSES`）
CLOSE_SEQ: tuple = ("thumb_low", "fist")
#: 每段「到位」的比例：实际角 ≥ 目标 × 这个值就算走到了
ARRIVE_RATIO = 0.85

# motor -> 角度（度）
POSES: Dict[str, Dict[int, float]] = {
    "open": {102: 0, 107: 0, 103: 0, 104: 0, 105: 0, 106: 0},
    # 第一段：只动拇指1（其余电机不在表里 = 保持不动）
    "thumb_low": {102: THUMB_LOW},
    "fist": {102: THUMB_LOW, 107: 40, 103: 140, 104: 140, 105: 140, 106: 140},
    "point": {102: 50, 107: 40, 103: 0, 104: 140, 105: 140, 106: 140},
    "pinch": {102: 45, 107: 30, 103: 55, 104: 10, 105: 10, 106: 10},
    "one": {102: 50, 107: 40, 103: 0, 104: 140, 105: 140, 106: 140},
    "two": {102: 50, 107: 40, 103: 0, 104: 0, 105: 140, 106: 140},
    "three": {102: 50, 107: 40, 103: 0, 104: 0, 105: 0, 106: 140},
}

# 动作 = 一串姿态，中间停顿 hold 秒
ACTIONS: Dict[str, Dict[str, object]] = {
    "open": {"label": "张开", "poses": ["open"], "hold": 0.0},
    "fist": {"label": "握拳", "poses": list(CLOSE_SEQ), "hold": 0.0},
    "point": {"label": "食指指", "poses": ["point"], "hold": 0.0},
    "pinch": {"label": "捏", "poses": ["pinch"], "hold": 0.0},
    "grip": {"label": "握拳→张开", "poses": list(CLOSE_SEQ) + ["open"], "hold": 0.5},
    "count": {"label": "数数 1-2-3", "poses": ["one", "two", "three", "open"], "hold": 0.6},
    "fan": {"label": "开合三次", "poses": ["open"] + list(CLOSE_SEQ) * 2 + ["open"],
            "hold": 0.35},
}


def pose_of(name: str, speed: float = SPEED, current: float = CURRENT) -> spec.Pose:
    """把预设角度表变成 :class:`spec.Pose`（速度/电流统一，按关节上限钳位）。"""
    from . import screw

    angles = POSES[name]
    return {
        mid: spec.Joint(angle, speed, screw.clamp_current(mid, current))
        for mid, angle in angles.items()
    }


def catalogue() -> List[dict]:
    """给网页用的动作清单。"""
    out = []
    for name, meta in ACTIONS.items():
        # 展示用的角度取「最全」的那个姿态：握拳是两段，第一段只有拇指1
        full = max(meta["poses"], key=lambda p: len(POSES[p]))
        out.append({
            "name": name,
            "label": meta["label"],
            "poses": meta["poses"],
            "hold": meta["hold"],
            "angles": POSES[full],
        })
    return out


def wait_pose(hand, pose: spec.Pose, ratio: float = ARRIVE_RATIO,
              timeout: float = 8.0, poll: float = 0.02, stop=None) -> bool:
    """等这一段走到位（实际角 ≥ 目标 × ratio）。

    注意：被东西挡住时**不算失败**——合手本来就该停在物体上，所以只等、不抛。
    """
    deadline = time.time() + max(0.0, timeout)
    while time.time() < deadline:
        if stop is not None and stop.is_set():
            return False
        try:
            state = hand.state()
        except Exception:  # pragma: no cover - 硬件异常不该把流程卡死
            return False
        if all(state[mid].angle >= float(j.angle) * ratio - 0.5
               for mid, j in pose.items() if abs(float(j.angle)) >= 5):
            return True
        time.sleep(poll)
    return False


def close_hand(hand, speed: float = SPEED, current: float = CURRENT,
               ratio: float = ARRIVE_RATIO, timeout: float = 8.0) -> List[str]:
    """合手：先拇指1 单独到下位，停稳后再合其余五指。返回实际走过的段名。

    用于 ``pick`` / ``pickbox`` 这类自己开车时序的流程；网页上的「握拳」按钮走
    :data:`ACTIONS`，顺序相同。
    """
    done: List[str] = []
    for name in CLOSE_SEQ:
        pose = pose_of(name, speed=speed, current=current)
        hand.apply(pose, reason=f"close:{name}")
        done.append(name)
        wait_pose(hand, pose, ratio=ratio, timeout=timeout)
    return done
