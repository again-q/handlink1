"""灵巧手规格、姿态模型与安全检查。

数据来源
--------
1. 《中科灵犀机械手控制系统用户手册》V3.0.1 表 3「手指电机与参数范围」。
2. 软件包 ``points/*.txt`` 的实盘点位格式（motorId / angle / speed / torque）。

本模块不依赖任何硬件：没有机械手也能跑通校验、点位读写和仿真。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Union

# 手册：速度范围统一 0～360
SPEED_MAX = 360.0
# 首次调试 / 改动点位后的建议试动速度（手册 2.2、2.4 第 6 条）
FIRST_RUN_SPEED = 60.0
# 单步超过这个角度视为「大步长」，给出提示
LARGE_STEP_DEG = 45.0


@dataclass(frozen=True)
class Motor:
    """一个手指电机。"""

    key: str          # 程序内别名
    name: str         # 中文名，与手册一致
    motor_id: int     # CAN 电机 ID
    pos_max: float    # 位置上限（度）
    current_max: float  # 电流上限（A），软件里叫「扭矩」
    note: str = ""

    @property
    def speed_max(self) -> float:
        return SPEED_MAX

    def as_dict(self) -> Dict[str, Any]:
        return {
            "key": self.key,
            "name": self.name,
            "motor_id": self.motor_id,
            "pos_min": 0.0,
            "pos_max": self.pos_max,
            "speed_max": self.speed_max,
            "current_max": self.current_max,
        }


# 顺序与厂家点位文件一致：拇指1、拇指2、食指、中指、无名指、小指
MOTORS: tuple = (
    Motor("thumb1", "拇指1", 102, 100.0, 0.8, "0～100°"),
    Motor("thumb2", "拇指2", 107, 45.0, 0.8, "0～45°"),
    Motor("index", "食指", 103, 180.0, 3.5),
    Motor("middle", "中指", 104, 180.0, 3.5),
    Motor("ring", "无名指", 105, 180.0, 3.5),
    Motor("pinky", "小指", 106, 180.0, 3.5),
)
MOTOR_BY_ID: Dict[int, Motor] = {m.motor_id: m for m in MOTORS}
MOTOR_BY_KEY: Dict[str, Motor] = {m.key: m for m in MOTORS}
MOTOR_BY_NAME: Dict[str, Motor] = {m.name: m for m in MOTORS}
MOTOR_IDS: tuple = tuple(m.motor_id for m in MOTORS)


@dataclass
class Joint:
    """单指指令：位置(度) / 速度(0~360) / 扭矩=电流上限(A)。"""

    angle: float = 0.0
    speed: float = FIRST_RUN_SPEED
    torque: float = 0.3

    def as_dict(self) -> Dict[str, float]:
        return {"angle": self.angle, "speed": self.speed, "torque": self.torque}


Pose = Dict[int, Joint]


def resolve(key: Union[int, str, Motor]) -> Motor:
    """把 102 / "102" / "index" / "食指" / Motor 统一解析成 Motor。"""
    if isinstance(key, Motor):
        return key
    if isinstance(key, int):
        if key in MOTOR_BY_ID:
            return MOTOR_BY_ID[key]
        raise KeyError(f"未知电机 ID: {key}（可用: {list(MOTOR_IDS)}）")
    s = str(key).strip()
    if s.isdigit() and int(s) in MOTOR_BY_ID:
        return MOTOR_BY_ID[int(s)]
    for table in (MOTOR_BY_KEY, MOTOR_BY_NAME):
        if s in table:
            return table[s]
    raise KeyError(f"未知电机: {key!r}（可用: {list(MOTOR_IDS)} / {[m.name for m in MOTORS]}）")


def blank_pose(angle: float = 0.0, speed: float = FIRST_RUN_SPEED, torque: float = 0.3) -> Pose:
    """全零姿态（张开手）。"""
    return {m.motor_id: Joint(angle, speed, torque) for m in MOTORS}


def clone(pose: Pose) -> Pose:
    return {mid: Joint(j.angle, j.speed, j.torque) for mid, j in pose.items()}


def complete(pose: Pose, fallback: Optional[Pose] = None) -> Pose:
    """补齐缺失的电机：优先沿用 fallback（当前姿态），否则全零。"""
    base = clone(fallback) if fallback else blank_pose()
    for mid, j in pose.items():
        resolve(mid)  # 校验 ID
        base[mid] = Joint(float(j.angle), float(j.speed), float(j.torque))
    return base


def _num(v: float) -> Union[int, float]:
    """整数值输出 int，与厂家点位文件风格一致。"""
    f = float(v)
    return int(f) if f.is_integer() else round(f, 4)


def to_vendor(pose: Mapping[int, Joint], motor_order: Iterable[int] = MOTOR_IDS) -> List[Dict[str, Any]]:
    """转成厂家点位文件里的 JSON 结构（列表，按固定顺序）。"""
    out: List[Dict[str, Any]] = []
    for mid in motor_order:
        j = pose.get(mid)
        if j is None:
            continue
        out.append(
            {
                "motorId": int(mid),
                "angle": _num(j.angle),
                "speed": _num(j.speed),
                "torque": _num(j.torque),
            }
        )
    return out


def from_vendor(data: Any) -> Pose:
    """解析厂家点位 JSON（也接受 {"motors": [...]} 或 {id: {...}}）。"""
    if isinstance(data, Mapping):
        data = data.get("motors", data)
    pose: Pose = {}
    if isinstance(data, Mapping):
        items = [(k, v) for k, v in data.items()]
        for k, v in items:
            m = resolve(k)
            if isinstance(v, Mapping):
                pose[m.motor_id] = Joint(
                    float(v.get("angle", 0)),
                    float(v.get("speed", FIRST_RUN_SPEED)),
                    float(v.get("torque", 0.3)),
                )
            else:  # {id: angle}
                pose[m.motor_id] = Joint(float(v))
        return pose
    for item in data:
        if isinstance(item, Mapping):
            mid = item.get("motorId", item.get("id", item.get("motor_id")))
            if mid is None:
                raise ValueError(f"点位条目缺少 motorId: {item!r}")
            m = resolve(mid)
            pose[m.motor_id] = Joint(
                float(item.get("angle", 0)),
                float(item.get("speed", FIRST_RUN_SPEED)),
                float(item.get("torque", 0.3)),
            )
        else:
            raise ValueError(f"无法识别的点位条目: {item!r}")
    return pose


def check(pose: Pose, reference: Optional[Pose] = None) -> List[Dict[str, Any]]:
    """安全检查。reference 为「当前实际姿态」，用于判断单步幅度。"""
    issues: List[Dict[str, Any]] = []
    for mid, j in pose.items():
        try:
            m = resolve(mid)
        except KeyError as exc:
            issues.append({"motor_id": mid, "level": "error", "message": str(exc)})
            continue
        if not (0.0 <= float(j.angle) <= m.pos_max):
            issues.append({
                "motor_id": mid, "level": "error",
                "message": f"{m.name} 位置 {j.angle}° 超出 0～{m.pos_max:.0f}°",
            })
        if not (0.0 <= float(j.speed) <= m.speed_max):
            issues.append({
                "motor_id": mid, "level": "error",
                "message": f"{m.name} 速度 {j.speed} 超出 0～{m.speed_max:.0f}",
            })
        if not (0.0 <= float(j.torque) <= m.current_max):
            issues.append({
                "motor_id": mid, "level": "error",
                "message": f"{m.name} 电流上限 {j.torque}A 超出 0～{m.current_max}A",
            })
        if 0.0 < float(j.torque) < 0.05:
            issues.append({
                "motor_id": mid, "level": "warning",
                "message": f"{m.name} 电流上限仅 {j.torque}A，基本不输出力矩",
            })
        if float(j.speed) > 120.0:
            issues.append({
                "motor_id": mid, "level": "warning",
                "message": f"{m.name} 速度 {j.speed} 偏高，首次试动建议 ≤60",
            })
        if reference and mid in reference:
            step = abs(float(j.angle) - float(reference[mid].angle))
            if step > LARGE_STEP_DEG:
                issues.append({
                    "motor_id": mid, "level": "warning",
                    "message": f"{m.name} 单步 {step:.1f}° 偏大，建议先低速分步验证",
                })
    missing = [m.motor_id for m in MOTORS if m.motor_id not in pose]
    if missing:
        issues.append({
            "motor_id": None, "level": "info",
            "message": f"姿态未包含电机 {missing}，下发时这些电机保持原值",
        })
    return issues


def clamp(pose: Pose) -> Pose:
    """把越界值夹到合法范围内（只在明确知情时使用）。"""
    out: Pose = {}
    for mid, j in pose.items():
        m = resolve(mid)
        out[m.motor_id] = Joint(
            min(max(float(j.angle), 0.0), m.pos_max),
            min(max(float(j.speed), 0.0), m.speed_max),
            min(max(float(j.torque), 0.0), m.current_max),
        )
    return out


def has_errors(issues: Iterable[Mapping[str, Any]]) -> bool:
    return any(i.get("level") == "error" for i in issues)


def describe_table() -> List[Dict[str, Any]]:
    return [m.as_dict() for m in MOTORS]


def pose_table(pose: Pose) -> List[Dict[str, Any]]:
    """把姿态排成便于打印的表格。"""
    rows: List[Dict[str, Any]] = []
    for m in MOTORS:
        j = pose.get(m.motor_id)
        rows.append({
            "name": m.name,
            "motor_id": m.motor_id,
            "angle": None if j is None else round(float(j.angle), 2),
            "speed": None if j is None else round(float(j.speed), 1),
            "torque": None if j is None else round(float(j.torque), 3),
            "pos_max": m.pos_max,
            "current_max": m.current_max,
        })
    return rows
