"""机械臂组动作：一串「关节路点 + 停顿」，像舞蹈的分解动作。

``arm.py`` 只回答「怎么让臂走到某个关节角」。真要用起来，人要的是**一串**动作：
「挥手打个招呼」= 抬起来 → 左右摆几次 → 放回原位。把这些写成数据而不是现场拼
``move_joints``，换来四件事：

* **能先看**：整条计划的角度、时长、大概走多远，先预览再决定动不动；
* **相对起点**：路点是相对「现在这个姿态」的增量，臂站在哪都能做，不必先回零；
* **有界**：单步 ≤ 45°、整段 ≤ 8 s、摆放次数 ≤ 6，与学生技能库（``skills.py``）一个思路；
* **不碰走线关节**：只动 J1/J2/J3/J5，J4/J6 是两根会拧线缆的关节（见 ``cable.py``），
  组动作一律不给它们增量 —— 从源头上就不会缠线。

挥手怎么摆
----------
``wave`` 只做三件事：把前臂抬起来（J2 后仰 12°、J3 收 30°、手腕 J5 压 16°），
然后 J1 左右各摆 ``swing_deg``、J5 跟着反相摆 ``wrist_deg``，最后原样放回。
以 ``/arm3d`` 的「预备」姿态 ``[0,-25,-40,0,65,0]`` 为例（``kin.fk`` 算的末端）：

===========  ==================  =========================
路点          关节角               末端 (mm)
===========  ==================  =========================
起点          [0,-25,-40,0,65,0]  611, -172, 720
抬起          [0,-37,-10,0,49,0]  615, -193, 818
左摆          [12,-37,-10,0,65,0]  647, -39, 808
右摆          [-12,-37,-10,0,33,0] 546, -330, 831
===========  ==================  =========================

即手在胸前上方 0.8 m 高度左右摆，幅度约 ±12°，看得很清楚又不会甩到人。

再加一个组动作
--------------
1. 写一个 ``def 名字(start, **参数) -> List[Move]``：``start`` 是起点关节角（度），
   里面用 :func:`offset` / :func:`_move` 生成各段；
2. 登记到 :data:`ACTIONS`（``label`` 给网页按钮用，``defaults`` 是参数默认值，
   ``builder`` 指向第 1 步的函数）。

就这样：``GET /arm/actions`` 会自动列出来，``/arm3d``「组动作」卡片里多一个按钮。

::

    from handlink import arm, arm_actions
    robot = arm.SimArm(); robot.enable()
    arm_actions.play(robot, "wave")          # 仿真里走一遍
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence

from . import kin

#: 真机第一次跑组动作的速度因子（%），与 ``arm.SAFE_SPEED_FACTOR`` 对齐
SAFE_SPEED = 20.0

#: 单步最大关节变化（度）：超过就说明这一步跨太大了，先拆成两段
MAX_STEP_DEG = 45.0

#: 整条组动作的停顿总量上限（毫秒）
MAX_HOLD_MS = 6000

#: 一条组动作最多几段
MAX_MOVES = 24

#: 一步到位后等臂稳住的时间（秒）：30004 的 RunningStatus 比到位判据晚一点
SETTLE_S = 0.15


@dataclass(frozen=True)
class Move:
    """一段动作：走到 :attr:`joints`，停 :attr:`hold_ms` 毫秒。"""

    label: str
    joints: List[float]
    hold_ms: int = 0
    #: 生成时是否被关节限位夹过（夹过说明起点离限位太近，动作会变形）
    clamped: bool = False

    def as_dict(self, digits: int = 3) -> Dict[str, Any]:
        return {"label": self.label,
                "joints": [round(float(v), digits) for v in self.joints],
                "hold_ms": int(self.hold_ms)}


def offset(start: Sequence[float], deltas: Dict[int, float]) -> List[float]:
    """起点 + ``{关节序号(1~6): 增量}``，再按手册限位夹一遍。

    ``deltas`` 的键从 **1** 数（J1=1），和 :data:`kin.JOINT_LIMITS` 一致，也和
    页面上写的 J1..J6 一致，避免内部用 0 起、报错时对不上号。
    """
    q = [float(v) for v in start]
    if len(q) != 6:
        raise ValueError(f"起点需要 6 个关节角，收到 {len(q)} 个")
    for idx, delta in deltas.items():
        i = int(idx) - 1
        if not 0 <= i < 6:
            raise ValueError(f"关节序号越界：{idx}（只能是 1~6）")
        q[i] += float(delta)
    return kin.clamp(q)


def at(label: str, start: Sequence[float], deltas: Dict[int, float],
       hold_ms: int = 0) -> Move:
    """记一段路点：从 ``start`` 叠加 ``deltas``（关节序号 1~6 起），停 ``hold_ms``。

    写新动作时基本只用这一个：``at("低头", base, {5: -22}, 320)``。
    被限位夹过的段会打上 ``clamped``，:func:`validate` 会提醒「动作会变形」。
    """
    raw = [float(v) for v in start]
    for idx, delta in deltas.items():
        i = int(idx) - 1
        if not 0 <= i < 6:
            raise ValueError(f"关节序号越界：{idx}（只能是 1~6）")
        raw[i] += float(delta)
    joints = kin.clamp(raw)
    return Move(label, joints, max(0, int(hold_ms)),
                clamped=any(abs(a - b) > 1e-6 for a, b in zip(joints, raw)))


def _clamp_int(value: Any, lo: int, hi: int, name: str) -> int:
    try:
        v = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 需要整数，收到 {value!r}") from exc
    return max(lo, min(hi, v))


def _clamp_float(value: Any, lo: float, hi: float, name: str) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 需要数字，收到 {value!r}") from exc
    return max(lo, min(hi, v))


# ------------------------------------------------------------------ 挥手

def wave(start: Sequence[float], cycles: Any = 3, swing_deg: Any = 12.0,
         wrist_deg: Any = 16.0, lift_ms: Any = 420, beat_ms: Any = 340) -> List[Move]:
    """挥手：抬起前臂 → J1 左右摆 + 手腕反相摆 → 原样放回。

    :param start: 起点关节角（度），一般是「现在这个姿态」
    :param cycles: 摆几个来回，1~6（默认 3，即左右各 3 次）
    :param swing_deg: J1 左右摆的幅度（度），5~25
    :param wrist_deg: J5 手腕跟着摆的幅度（度），5~30
    :param lift_ms: 抬起后停多久才开始摆（毫秒）
    :param beat_ms: 每一摆停多久（毫秒）
    """
    cycles = _clamp_int(cycles, 1, 6, "cycles")
    swing = _clamp_float(swing_deg, 5.0, 25.0, "swing_deg")
    wrist = _clamp_float(wrist_deg, 5.0, 30.0, "wrist_deg")
    lift_ms = _clamp_int(lift_ms, 100, 1500, "lift_ms")
    beat_ms = _clamp_int(beat_ms, 100, 1500, "beat_ms")

    base = [float(v) for v in start]
    lift = {2: -12.0, 3: 30.0, 5: -16.0}
    moves: List[Move] = [at("抬起（手举到胸前上方）", base, lift, lift_ms)]
    for i in range(cycles):
        moves.append(at(f"第 {i + 1} 摆 · 左", base,
                           {**lift, 1: swing, 5: lift[5] + wrist}, beat_ms))
        moves.append(at(f"第 {i + 1} 摆 · 右", base,
                           {**lift, 1: -swing, 5: lift[5] - wrist}, beat_ms))
    moves.append(at("放回原位", base, {}, 0))
    return moves


def nod(start: Sequence[float], deep_deg: Any = 22.0,
        beat_ms: Any = 320) -> List[Move]:
    """点头示意：手腕先低一下再抬回来，做两遍。"""
    deep = _clamp_float(deep_deg, 5.0, 30.0, "deep_deg")
    beat = _clamp_int(beat_ms, 100, 1500, "beat_ms")
    base = [float(v) for v in start]
    moves: List[Move] = []
    for _ in range(2):
        moves.append(at("低头", base, {5: -deep}, beat))
        moves.append(at("抬头", base, {5: +deep * 0.6}, beat))
    moves.append(at("放回原位", base, {}, 0))
    return moves


#: 组动作登记表：网页按钮、接口清单都从这里来。
ACTIONS: Dict[str, Dict[str, Any]] = {
    "wave": {
        "label": "挥手",
        "description": "抬起前臂，J1 左右摆 + 手腕反相摆 3 次后回原位（示例动作）",
        "builder": wave,
        "defaults": {"cycles": 3, "swing_deg": 12.0, "wrist_deg": 16.0,
                     "lift_ms": 420, "beat_ms": 340},
    },
    "nod": {
        "label": "点头示意",
        "description": "手腕低下再抬起，做两遍后回原位",
        "builder": nod,
        "defaults": {"deep_deg": 22.0, "beat_ms": 320},
    },
}

def build(name: str, start: Sequence[float], **params: Any) -> List[Move]:
    """把一条组动作展开成具体的关节路点。

    ``params`` 里没给的就用 :data:`ACTIONS` 的默认值；给了未知参数会报错
    （比默默忽略更难查）。起点必须 6 个数，否则所有路点都没意义。
    """
    meta = ACTIONS.get(str(name))
    if meta is None:
        raise ValueError(f"未知组动作: {name}（可用: {', '.join(sorted(ACTIONS))}）")
    if len(list(start)) != 6:
        raise ValueError(f"起点需要 6 个关节角，收到 {len(list(start))} 个")
    kwargs = dict(meta["defaults"])
    kwargs.update({k: v for k, v in params.items() if v is not None})
    try:
        moves = meta["builder"]([float(v) for v in start], **kwargs)
    except TypeError as exc:
        known = "、".join(sorted(meta["defaults"]))
        raise ValueError(f"{name} 的参数不对（可用: {known}）；{exc}") from exc
    return moves


def validate(moves: Sequence[Move]) -> List[Dict[str, Any]]:
    """整条组动作的安全检查，返回问题列表（``level`` 是 ``error`` / ``warning``）。"""
    issues: List[Dict[str, Any]] = []
    if not moves:
        return [{"level": "error", "message": "组动作是空的"}]
    if len(moves) > MAX_MOVES:
        issues.append({"level": "error",
                       "message": f"段数过多（{len(moves)} > {MAX_MOVES}）"})
    total_hold = 0
    prev: Optional[List[float]] = None
    for i, move in enumerate(moves, 1):
        if len(move.joints) != 6:
            issues.append({"level": "error",
                           "message": f"第 {i} 段 {move.label} 不是 6 个关节角"})
            continue
        total_hold += int(move.hold_ms)
        bad = kin.within_limits(move.joints)
        if bad:
            issues.append({"level": "error",
                           "message": f"第 {i} 段 {move.label} 关节越界: "
                                      + "、".join(f"J{b}" for b in bad)})
        if move.clamped:
            issues.append({"level": "warning",
                           "message": f"第 {i} 段 {move.label} 有 joints 被限位夹住，"
                                      "起点离限位太近，动作会变形"})
        if prev is not None:
            gap = max(abs(a - b) for a, b in zip(move.joints, prev))
            if gap > MAX_STEP_DEG:
                issues.append({"level": "error",
                               "message": f"第 {i} 段 {move.label} 跨度 {gap:.0f}° "
                                          f"超过单步上限 {MAX_STEP_DEG:.0f}°"})
        prev = list(move.joints)
    if total_hold > MAX_HOLD_MS:
        issues.append({"level": "error",
                       "message": f"停顿总量 {total_hold} ms 超过 {MAX_HOLD_MS} ms"})
    return issues


def estimate_seconds(moves: Sequence[Move], start: Sequence[float],
                     speed: float = SAFE_SPEED) -> float:
    """粗估整条动作要多久（秒）。

    按手册的 100°/s 乘速度因子算关节行程，再叠上停顿和每段的到位稳定时间。
    真机的加速度限制会让它略慢一点，用来判断「这一下大约几秒」够了。
    """
    jerk = kin.JOINT_MAX_SPEED * max(1.0, min(float(speed), 100.0)) / 100.0
    total = 0.0
    prev = [float(v) for v in start]
    for move in moves:
        travel = max(abs(a - b) for a, b in zip(move.joints, prev))
        total += travel / jerk + move.hold_ms / 1000.0 + SETTLE_S + 0.1
        prev = list(move.joints)
    return round(total, 2)


def describe(moves: Sequence[Move], start: Sequence[float],
             speed: float = SAFE_SPEED) -> List[Dict[str, Any]]:
    """给命令行 / 网页看的计划表：每段走到哪、停多久、从上一段走多远。"""
    prev = [float(v) for v in start]
    rows = []
    for i, move in enumerate(moves, 1):
        travel = max(abs(a - b) for a, b in zip(move.joints, prev))
        rows.append({"step": i, "label": move.label,
                     "joints": [round(v, 1) for v in move.joints],
                     "travel_deg": round(travel, 1),
                     "hold_ms": int(move.hold_ms)})
        prev = list(move.joints)
    return rows


def catalogue() -> List[Dict[str, Any]]:
    """组动作清单，给 ``GET /arm/actions`` 和网页按钮用。"""
    out = []
    for name, meta in ACTIONS.items():
        out.append({"name": name, "label": meta["label"],
                    "description": meta["description"],
                    "defaults": dict(meta["defaults"])})
    return out


def play(robot: Any, name: str, start: Optional[Sequence[float]] = None,
         speed: float = SAFE_SPEED, stop_event: Any = None,
         on_step: Optional[Callable[[Dict[str, Any]], None]] = None) -> Dict[str, Any]:
    """在 ``SimArm`` / ``DobotArm`` 上跑一条组动作，返回每段的结果。

    只做「按顺序 move_joints + 停顿」；安全闸门（手腕防缠、碰撞间距）在服务层
    （``service.App.arm_action``）和脚本里各自把关，因为离线/真机要查的东西不一样。
    """
    start = [float(v) for v in (start if start is not None else robot.joints())]
    moves = build(name, start)
    errors = [i["message"] for i in validate(moves) if i["level"] == "error"]
    if errors:
        raise ValueError("；".join(errors))

    robot.speed_factor(speed)
    done: List[Dict[str, Any]] = []
    for i, move in enumerate(moves, 1):
        if stop_event is not None and stop_event.is_set():
            break
        robot.move_joints(move.joints, wait=True, timeout=20.0)
        row = {"step": i, "label": move.label,
               "joints": [round(v, 3) for v in move.joints],
               "hold_ms": int(move.hold_ms)}
        done.append(row)
        if on_step is not None:
            on_step(row)
        time.sleep(SETTLE_S + move.hold_ms / 1000.0)
    return {"ok": len(done) == len(moves), "action": name,
            "steps": done, "total": len(moves),
            "estimate_s": estimate_seconds(moves, start, speed)}
