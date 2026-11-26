"""抓取流程：臂到位 → 手闭合 → 抬升。

把「臂」和「手」两条链路串成一次可复盘的动作，顺序与互锁写在 :func:`run` 里：

1. 手张开（先保证不会带着闭合的手撞过去）
2. 臂回安全位 → 移到预抓取点上方 → 直线下探到抓取点
3. 手合到握拳（电流上限可调，低于 1.0 A 会卡在半途）
4. **互锁**：手确实合上了才允许抬升
5. 直线抬升 → 手张开 → 回安全位

``SimArm`` + ``SimBus`` 可以离线跑完整流程（``dry_run``），真机只是把两个对象换成
``DobotArm`` 和 ``CanBus``。
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

from . import poses, spec

DEFAULT_HOME: tuple = (400.0, 0.0, 300.0, 180.0, 0.0, 0.0)
DEFAULT_PRE: tuple = (400.0, 0.0, 200.0, 180.0, 0.0, 0.0)
DEFAULT_GRASP: tuple = (400.0, 0.0, 120.0, 180.0, 0.0, 0.0)
DEFAULT_LIFT: tuple = (400.0, 0.0, 250.0, 180.0, 0.0, 0.0)

#: 手「合上了」的判据：实际角 ≥ 目标角 × 这个比例
CLENCH_RATIO = 0.6
#: 判定前的额外等待（s），给手留出闭合时间
CLENCH_GRACE = 2.0


class PickError(RuntimeError):
    """流程中断（未到位、手没合上、臂报错）。"""


@dataclass
class PickPlan:
    """一次抓取的四个点位和参数。坐标单位 mm / 度，与 Dobot 的 GetPose 一致。"""

    home: Sequence[float] = DEFAULT_HOME
    pre: Sequence[float] = DEFAULT_PRE
    grasp: Sequence[float] = DEFAULT_GRASP
    lift: Sequence[float] = DEFAULT_LIFT
    speed: float = 20.0          # 速度因子（%）
    force: float = 1.0           # 闭合电流上限（A）
    hold: float = 0.3            # 合手后等待（s）
    timeout: float = 30.0        # 单段移动超时（s）


def clenched(hand, target: Dict[int, float], ratio: float = CLENCH_RATIO) -> bool:
    """手是否已经合到目标附近。"""
    try:
        state = hand.state()
    except Exception:  # pragma: no cover - 硬件异常不该把流程卡死
        return False
    for mid, angle in target.items():
        if abs(angle) < 5:
            continue
        if state[mid].angle < angle * ratio:
            return False
    return True


def run(arm, hand, plan: Optional[PickPlan] = None,
        log: Callable[[str], None] = print) -> Dict[str, object]:
    """跑一次完整抓取，返回时间线。失败抛 :class:`PickError`。"""
    plan = plan or PickPlan()
    started = time.time()
    steps: List[Dict[str, object]] = []

    def record(name: str, **extra) -> None:
        item = {"step": name, "t_ms": int((time.time() - started) * 1000), **extra}
        steps.append(item)
        log(f"[{item['t_ms']:>6} ms] {name}" + (f"  {extra}" if extra else ""))

    def move(kind: str, target: Sequence[float], label: str) -> None:
        fn = arm.move_j if kind == "MovJ" else arm.move_l
        result = fn(target, wait=True, timeout=plan.timeout)
        arrived = result.get("arrived", True)
        record(label, cmd=kind, to=[round(float(v), 1) for v in target], arrived=arrived)
        if not arrived:
            raise PickError(f"{label} 未到位（超时 {plan.timeout:.0f}s），已停在当前位置")

    arm.enable()
    arm.speed_factor(plan.speed)

    # 1. 手先张开
    hand.apply(poses.pose_of("open"), reason="pick:open")
    record("手张开")

    # 2. 回安全位 → 预抓取 → 下探
    move("MovJ", plan.home, "臂回安全位")
    move("MovL", plan.pre, "移动到预抓取点")
    move("MovL", plan.grasp, "下探到抓取点")

    # 3. 合手：先拇指1（102）单独到下位，停稳后再合其余五指
    poses.close_hand(hand, current=plan.force)
    record("手闭合", force=plan.force)

    # 4. 互锁：没合上就别抬（给手留出闭合时间，再判定）
    deadline = time.time() + max(plan.hold, 0.0) + CLENCH_GRACE
    while time.time() < deadline and not clenched(hand, poses.POSES["fist"]):
        time.sleep(0.05)
    if not clenched(hand, poses.POSES["fist"]):
        raise PickError("手没有合到位（可能电流不足触发 I-LIM），已中止，未抬升")

    # 5. 抬升 → 张开 → 回位
    move("MovL", plan.lift, "抬升")
    hand.apply(poses.pose_of("open"), reason="pick:open")
    record("手张开（放下）")
    move("MovJ", plan.home, "臂回安全位")

    return {
        "ok": True,
        "elapsed_ms": int((time.time() - started) * 1000),
        "plan": {
            "home": list(plan.home), "pre": list(plan.pre),
            "grasp": list(plan.grasp), "lift": list(plan.lift),
            "speed": plan.speed, "force": plan.force,
        },
        "steps": steps,
    }


def preview(plan: Optional[PickPlan] = None) -> Dict[str, object]:
    """不碰硬件，只算一遍要走的距离和大致耗时。"""
    plan = plan or PickPlan()
    import math

    segs = [
        ("MovL 预抓取", plan.home, plan.pre),
        ("MovL 下探", plan.pre, plan.grasp),
        ("MovL 抬升", plan.grasp, plan.lift),
        ("MovJ 回位", plan.lift, plan.home),
    ]
    speed = 100.0 * plan.speed / 100.0  # 100 mm/s @100%
    rows = []
    for name, a, b in segs:
        dist = math.dist(list(a)[:3], list(b)[:3])
        rows.append({"seg": name, "dist_mm": round(dist, 1),
                     "eta_s": round(dist / speed, 2) if speed else 0.0})
    return {"plan": plan.__dict__, "segments": rows,
            "eta_s": round(sum(r["eta_s"] for r in rows), 2)}
