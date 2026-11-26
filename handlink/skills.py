"""技能库：AI 只允许调用这些原语，不允许直接写电机角度。

设计原则
--------
* **白名单**：模型能说的只有下面这些技能，说别的就拒绝。
* **夹紧**：角度、速度、电流都过一遍限位再下发。
* **有界**：一次最多 12 步、总时长最多 8 秒，避免模型生成一个跑十分钟的剧本。
* **可解释**：每一步都能翻译成中文，出问题知道是哪一步。

给"写 AI 的社员"的接口就是 :func:`schema`（喂给模型）和 :func:`steps_for`
（把模型输出变成动作）。
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from . import spec

MAX_SPEED = 120.0      # AI 模式下速度上限（°）
MAX_TORQUE = 0.5       # AI 模式下电流上限（A）
MAX_STEPS = 12
MAX_TOTAL_MS = 8000
MAX_ANGLE_STEP = 120.0  # 单步最大变化

# 厂家实测点位 + 我们自己补的常用手势（角度，度）
POSE_LIB: Dict[str, Dict[int, float]] = {
    "open":     {102: 0,  107: 0,  103: 0,   104: 0,   105: 0,   106: 0},
    "close":    {102: 40, 107: 25, 103: 159, 104: 146, 105: 154, 106: 175},
    "paper":    {102: 0,  107: 0,  103: 0,   104: 0,   105: 0,   106: 0},
    "rock":     {102: 40, 107: 25, 103: 159, 104: 146, 105: 154, 106: 175},
    "scissors": {102: 68, 107: 27, 103: 0,   104: 0,   105: 138, 106: 146},
    "pinch":    {102: 58, 107: 30, 103: 55,  104: 150, 105: 160, 106: 170},
    "tripod":   {102: 62, 107: 32, 103: 85,  104: 90,  105: 160, 106: 170},
    "point":    {102: 45, 107: 20, 103: 0,   104: 160, 105: 170, 106: 175},
    # 「招手」用的半握：四指微屈、留出活动余量
    "half":     {102: 30, 107: 18, 103: 70,  104: 65,  105: 70,  106: 75},
    # 「勾手」准备位：只留食指伸直，其余收起来
    "beckon_base": {102: 45, 107: 22, 103: 0, 104: 120, 105: 135, 106: 145},
}

DESCRIPTIONS = {
    "open": "张开手（五指伸直）",
    "close": "握拳 / 抓住",
    "paper": "出布（同张开）",
    "rock": "出石头（同握拳）",
    "scissors": "出剪刀（食指中指伸出）",
    "pinch": "两指捏（拇指+食指）",
    "tripod": "三指捏（拇指+食指+中指）",
    "point": "食指指向前方",
    "half": "半握（五指微屈）",
    "beckon_base": "勾手准备位（只留食指伸出）",
    "set_fingers": "指定某几根手指的角度，例如 食指 90 度",
    "hold": "保持当前姿态一段时间",
    "wave": "招手：五指反复张开-半握",
    "beckon": "勾手：食指反复勾动，其余手指收着",
}


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(v)))


def make_pose(angles: Dict[int, float], speed: float = 90.0, torque: float = 0.4) -> spec.Pose:
    """角度 → 受约束的姿态。越界会被夹到限位，不报错（AI 场景要能跑下去）。"""
    speed = _clamp(speed, 1.0, MAX_SPEED)
    torque = _clamp(torque, 0.05, MAX_TORQUE)
    pose: spec.Pose = {}
    for mid, angle in angles.items():
        m = spec.MOTOR_BY_ID[mid]
        pose[mid] = spec.Joint(_clamp(angle, 0.0, m.pos_max), speed, torque)
    return pose


def steps_for(skill: str, args: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
    """把一个技能展开成若干步：``[{"pose": Pose, "delay_ms": int, "note": str}]``。"""
    args = dict(args or {})
    speed = args.get("speed", 90.0)
    torque = args.get("torque", 0.4)
    delay = int(args.get("delay_ms", 500))

    if skill in POSE_LIB:
        pose = make_pose(POSE_LIB[skill], speed, torque)
        return [{"pose": pose, "delay_ms": delay, "note": DESCRIPTIONS.get(skill, skill)}]

    if skill == "set_fingers":
        raw = args.get("angles") or args.get("fingers") or {}
        angles: Dict[int, float] = {}
        for key, value in raw.items():
            angles[spec.resolve(key).motor_id] = float(value)
        if not angles:
            raise ValueError("set_fingers 需要 angles，例如 {\"angles\": {\"食指\": 90}}")
        pose = make_pose(angles, speed, torque)
        note = "、".join(f"{spec.MOTOR_BY_ID[mid].name} {j.angle:.0f}°" for mid, j in pose.items())
        return [{"pose": pose, "delay_ms": delay, "note": f"设定 {note}"}]

    if skill == "hold":
        ms = int(args.get("ms", delay or 1000))
        return [{"pose": {}, "delay_ms": ms, "note": f"保持 {ms} ms"}]

    if skill in ("wave", "beckon"):
        cycles = int(args.get("cycles", 3 if skill == "beckon" else 2))
        period = int(args.get("period_ms", 700))
        cycles = max(1, min(cycles, 5))
        half = max(80, period // 2)
        steps: List[Dict[str, Any]] = []
        if skill == "wave":
            for _ in range(cycles):
                steps.append({"pose": make_pose(POSE_LIB["open"], speed, torque),
                              "delay_ms": half, "note": "张开"})
                steps.append({"pose": make_pose(POSE_LIB["half"], speed, torque),
                              "delay_ms": half, "note": "半握"})
            steps.append({"pose": make_pose(POSE_LIB["open"], speed, torque),
                          "delay_ms": 120, "note": "张开收尾"})
        else:
            base = dict(POSE_LIB["beckon_base"])
            for _ in range(cycles):
                steps.append({"pose": make_pose({**base, 103: 0}, speed, torque),
                              "delay_ms": half, "note": "食指伸直"})
                steps.append({"pose": make_pose({**base, 103: 100}, speed, torque),
                              "delay_ms": half, "note": "食指勾回"})
        return steps

    raise ValueError(f"未知技能: {skill}（可用: {', '.join(sorted(DESCRIPTIONS))}）")


def schema() -> List[Dict[str, Any]]:
    """给模型看的技能清单（也可直接转成 OpenAI tools 定义）。"""
    out = []
    for name, desc in DESCRIPTIONS.items():
        args: Dict[str, Any] = {
            "speed": {"type": "number", "description": f"速度 1~{MAX_SPEED:.0f}，默认 90"},
            "torque": {"type": "number", "description": f"电流上限 A，默认 0.4，最大 {MAX_TORQUE}"},
            "delay_ms": {"type": "integer", "description": "执行后等待毫秒，默认 500"},
        }
        if name == "set_fingers":
            args["angles"] = {
                "type": "object",
                "description": "手指名或电机 ID → 角度（度）。手指名：拇指1/拇指2/食指/中指/无名指/小指",
                "additionalProperties": {"type": "number"},
            }
        if name == "hold":
            args["ms"] = {"type": "integer", "description": "保持毫秒"}
        out.append({"skill": name, "description": desc, "args": args})
    return out


def validate(steps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """整段计划的安全检查，返回问题列表。"""
    issues: List[Dict[str, Any]] = []
    if len(steps) > MAX_STEPS:
        issues.append({"level": "error", "message": f"步骤过多（{len(steps)} > {MAX_STEPS}）"})
    total = sum(int(s.get("delay_ms", 0)) for s in steps)
    if total > MAX_TOTAL_MS:
        issues.append({"level": "error", "message": f"总时长过长（{total} ms > {MAX_TOTAL_MS} ms）"})
        steps[:] = steps[:MAX_STEPS]
    for i, step in enumerate(steps):
        for mid, joint in step["pose"].items():
            m = spec.MOTOR_BY_ID[mid]
            if not (0 <= joint.angle <= m.pos_max):
                issues.append({"level": "error", "message": f"第 {i+1} 步 {m.name} 角度越界 {joint.angle}"})
            if joint.speed > MAX_SPEED:
                issues.append({"level": "error", "message": f"第 {i+1} 步速度超过 AI 上限"})
            if joint.torque > MAX_TORQUE:
                issues.append({"level": "error", "message": f"第 {i+1} 步电流超过 AI 上限"})
    return issues
