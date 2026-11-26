"""手腕电缆防缠：把「绕手腕转」的指令换成不会越缠越紧的等效角。

灵巧手是**有线**的，线从手腕里穿过去。J4（腕部滚转）和 J6（法兰自转）
每转一圈，线就多拧一圈；连续同方向转几次就会绷住甚至扯断。控制器本身
只会照着目标角走最短的那一段（其实连"最短"都不保证，它只认你给的值），
所以防缠得在**下发之前**做。

做法分两层：

1. **等效角**：角度是模 360 的。想在 +250° 停，和想在 -110° 停，物理姿态一样。
   给目标加 360k，挑一个让累计扭转更小的，就能"反方向转回去"。
2. **累计扭转**：光挑最短不够——每次都 +90° 也是最短，四次就 360° 了。
   所以按实测关节角累计（解开 ±180 的跳变），把 |累计| 限制在 ``limit_deg``
   以内；超了就只接受能把扭转**减小**的等效角，减不了就拒绝。

基准点由使用者定：装好线、确认没缠的时候调 ``mark()``（页面上「标记为不缠」），
之后从这个姿态开始累计。服务重启也会重置到这个姿态。
"""
from __future__ import annotations

import os
from typing import Iterable, List, Sequence, Tuple

from . import kin

#: 会拧线的关节（0 基）：J4 腕部滚转、J6 法兰自转
DEFAULT_JOINTS: Tuple[int, ...] = (3, 5)
#: 允许的累计扭转（度）。一圈是 360；线细、走线短的话调小。
DEFAULT_LIMIT = 360.0


def wrap180(delta: float) -> float:
    """把任意角度差折到 (-180, 180]。"""
    return (delta + 180.0) % 360.0 - 180.0


def joints_from_env() -> Tuple[int, ...]:
    """``HANDLINK_CABLE_JOINTS=4,6``（1 基）。默认 J4/J6。"""
    raw = os.environ.get("HANDLINK_CABLE_JOINTS", "").strip()
    if not raw:
        return DEFAULT_JOINTS
    out: List[int] = []
    for part in raw.split(","):
        try:
            idx = int(part.strip()) - 1
        except ValueError:
            continue
        if 0 <= idx < 6 and idx not in out:
            out.append(idx)
    return tuple(out) or DEFAULT_JOINTS


def limit_from_env() -> float:
    try:
        return float(os.environ.get("HANDLINK_CABLE_LIMIT", DEFAULT_LIMIT))
    except ValueError:
        return DEFAULT_LIMIT


class TwistGuard:
    """按实测角累计 J4/J6 的扭转，并给出等效目标角。"""

    def __init__(self, joints: Iterable[int] | None = None,
                 limit_deg: float | None = None) -> None:
        self.joints: Tuple[int, ...] = tuple(joints) if joints else joints_from_env()
        self.limit = float(limit_deg if limit_deg is not None else limit_from_env())
        self.twist: List[float] = [0.0] * 6
        self.last: List[float] | None = None

    # -- 跟踪 -------------------------------------------------------------
    def mark(self, joints: Sequence[float]) -> None:
        """以当前姿态为「没缠」的基准，累计清零。"""
        self.last = [float(v) for v in joints]
        self.twist = [0.0] * 6

    def observe(self, joints: Sequence[float]) -> None:
        """用新一帧实测角更新累计（自动处理 ±180 的跳变）。"""
        if self.last is None:
            self.mark(joints)
            return
        for i in self.joints:
            self.twist[i] += wrap180(float(joints[i]) - self.last[i])
        self.last = [float(v) for v in joints]

    # -- 规划 -------------------------------------------------------------
    def choose(self, current: Sequence[float],
               target: Sequence[float]) -> Tuple[List[float], List[str], bool]:
        """返回 ``(调整后的目标, 说明, 是否被拦)``。

        对每个受保护关节，在 ``target ± 360k`` 里挑一个：
        关节限位内、累计不超限，然后按 ``|累计| + 0.5·|转过的角度|`` 打分。
        这个权重是故意的：正常操作时别为了"回一点缠"就凭空多转大半圈；
        真缠起来（接近上限）时，"少缠"这一项才会压过"少转"。
        """
        out = [float(v) for v in target]
        notes: List[str] = []
        blocked = False
        for i in self.joints:
            base = float(target[i])
            cands: List[Tuple[float, float, float, float]] = []
            for k in (-2, -1, 0, 1, 2):
                cand = base + 360.0 * k
                lo, hi = kin.JOINT_LIMITS[i]
                if cand < lo - 1e-9 or cand > hi + 1e-9:
                    continue
                delta = cand - float(current[i])
                after = self.twist[i] + delta
                if abs(after) > self.limit + 1e-6:
                    continue
                cands.append((abs(after) + 0.5 * abs(delta), abs(delta), cand, after))
            if not cands:
                blocked = True
                notes.append(f"J{i + 1} 到 {base:+.1f}° 会超过扭转上限 ±{self.limit:.0f}°，"
                             f"请先反向回缠")
                continue
            cands.sort()
            _, _, cand, after = cands[0]
            out[i] = cand
            if abs(cand - base) > 1e-6:
                notes.append(f"J{i + 1} {base:+.1f}° → {cand:+.1f}°"
                             f"（走反方向，累计 {after:+.1f}°）")
        return out, notes, blocked

    def state(self) -> dict:
        return {
            "protected_joints": [i + 1 for i in self.joints],
            "twist_deg": {f"J{i + 1}": round(self.twist[i], 1) for i in self.joints},
            "limit_deg": self.limit,
            "reference": [round(v, 3) for v in self.last] if self.last else None,
        }
