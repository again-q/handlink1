"""协作臂驱动：越疆 Dobot Nova 5（DT-NV050-1）。

臂和手是两条独立链路：

* 手 —— CAN（``handlink.canbus``），1000 kbps，3000 帧/秒心跳；
* 臂 —— 网口 TCP 29999（控制口），另有 30004 反馈口。

这一层只回答一个问题：**怎么让臂动起来、并且知道它到了没有**。
``SimArm`` 让整条流程在没有臂的时候也能跑通（dry-run / 单元测试），
``DobotArm`` 是照 Dobot CR/Nova 的 dashboard 协议写的，**尚未在 Nova 5 真机上验证**。

安全默认值：首次上线 ``SpeedFactor`` 不超过 20%，移动前必须显式 ``enable()``。
"""
from __future__ import annotations

import math
import socket
import threading
import time
from collections import deque
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from . import kin

DEFAULT_IP = "192.168.5.1"
CONTROL_PORT = 29999
FEEDBACK_PORT = 30004

#: 首次调试的最大速度因子（%），改大之前先确认工作空间里没有人
SAFE_SPEED_FACTOR = 20.0

#: 关节/笛卡尔到位判据
POS_TOL_MM = 1.0
ANG_TOL_DEG = 1.0


class ArmError(RuntimeError):
    """臂的通信或指令错误。"""


class _LinkLost(ArmError):
    """29999 连接被对端断开（区别于指令本身报错）。"""


def _num(v: float) -> str:
    """Dobot 协议里的数字：不要科学计数法、不要多余 0。"""
    s = f"{float(v):.3f}".rstrip("0").rstrip(".")
    return s or "0"


def fmt_pose(pose: Sequence[float]) -> str:
    """``{x,y,z,rx,ry,rz}``。"""
    if len(pose) != 6:
        raise ValueError(f"姿态需要 6 个数 (x,y,z,rx,ry,rz)，收到 {len(pose)} 个")
    return "{" + ",".join(_num(v) for v in pose) + "}"


def fmt_joints(joints: Sequence[float]) -> str:
    """``{j1,j2,j3,j4,j5,j6}``，关节空间的 MovJ 用这个。

    手册 V4.6 的 ``MovJ(P,...)`` 里 P 既可以是 ``pose={x,y,z,rx,ry,rz}``，也可以是
    ``joint={j1..j6}``。关节空间不需要逆解，第一次试动作用它最稳。
    """
    if len(joints) != 6:
        raise ValueError(f"关节角需要 6 个数 (j1..j6)，收到 {len(joints)} 个")
    return "{" + ",".join(_num(v) for v in joints) + "}"


def _numbers(text: str) -> List[float]:
    """把 MovL(pose={1,2,3,4,5,6},r=5) 里的数字按出现顺序抠出来。

    只认数字和正负号；GetPose(); 这种没有数字的返回空列表。
    """
    out: List[float] = []
    buf = ""
    for ch in str(text):
        if ch.isdigit() or ch in "+-.":
            buf += ch
        else:
            if buf and buf not in ("+", "-", "."):
                try:
                    out.append(float(buf))
                except ValueError:
                    pass
            buf = ""
    if buf and buf not in ("+", "-", "."):
        try:
            out.append(float(buf))
        except ValueError:
            pass
    return out


def parse_result_id(text: str) -> Optional[int]:
    """从 ``0,{123},MovL(pose={...});`` 里取出算法队列 ID。

    队列指令（MovL/MovJ/Arc/ServoP/StartPath…）下发成功后会**立刻**返回一个 ID，
    指令在后台排队执行；要判断「执行到哪了」就轮询 ``GetCurrentCommandID()`` 和它比。
    """
    start, end = text.find("{"), text.find("}")
    if start < 0 or end <= start:
        return None
    try:
        return int(float(text[start + 1:end].strip()))
    except ValueError:
        return None


def parse_pose(text: str) -> List[float]:
    """从 ``0,{400,0,300,180,0,0},GetPose();`` 里取出 6 个数。"""
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        raise ArmError(f"返回值里没有姿态: {text!r}")
    body = text[start + 1:end]
    values = [float(x) for x in body.replace("，", ",").split(",") if x.strip()]
    if len(values) != 6:
        raise ArmError(f"姿态字段数不对（{len(values)}）: {text!r}")
    return values


class Arm:
    """接口：``SimArm`` 与 ``DobotArm`` 共用。"""

    name = "abstract"

    # -- 生命周期 ---------------------------------------------------------
    def connect(self) -> None:  # pragma: no cover - 接口
        pass

    def close(self) -> None:  # pragma: no cover
        pass

    # -- 状态 -------------------------------------------------------------
    def enabled(self) -> bool:
        raise NotImplementedError

    def pose(self) -> List[float]:
        raise NotImplementedError

    def joints(self) -> List[float]:
        raise NotImplementedError

    def ik(self, pose: Sequence[float],
           near: Optional[Sequence[float]] = None) -> Optional[List[float]]:
        """位姿 → 关节角。默认用本地数值反解；真机子类走控制器自己的 IK。"""
        raise NotImplementedError

    # -- 指令 -------------------------------------------------------------
    def enable(self) -> None:
        raise NotImplementedError

    def disable(self) -> None:
        raise NotImplementedError

    def speed_factor(self, percent: float) -> float:
        raise NotImplementedError

    def move_j(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        raise NotImplementedError

    def move_l(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        raise NotImplementedError

    def move_joints(self, joints: Sequence[float], wait: bool = True,
                    timeout: float = 30.0) -> Dict[str, object]:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    # -- 通用 -------------------------------------------------------------
    @staticmethod
    def _ang_diff(a: float, b: float) -> float:
        """两角之差，归一到 0～180。

        180° 和 −180° 是同一个朝向，但 ``abs(a-b)`` 会算出 360。控制器有时
        返回 −180、有时返回 +180（取决于 IK 选了哪支），直接用减法会让
        ``wait_until`` 永远等不到，白等到 timeout —— 实测白等了 40 s。
        """
        return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)

    def wait_until(self, target: Sequence[float], tol: float = POS_TOL_MM,
                   ang_tol: float = ANG_TOL_DEG, timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            p = self.pose()
            if all(abs(p[i] - target[i]) <= tol for i in range(3)) and \
                    all(self._ang_diff(p[i], target[i]) <= ang_tol for i in range(3, 6)):
                return True
            time.sleep(0.05)
        return False

    def wait_until_joints(self, target: Sequence[float], tol: float = ANG_TOL_DEG,
                          timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            j = self.joints()
            if all(abs(j[i] - target[i]) <= tol for i in range(6)):
                return True
            time.sleep(0.05)
        return False

    def status(self) -> Dict[str, object]:
        return {
            "arm": self.name,
            "enabled": self.enabled(),
            "pose": [round(v, 3) for v in self.pose()],
            "joints": [round(v, 3) for v in self.joints()],
        }


# ================================================================ 仿真臂
#
# 这下面不是「假装动一下」，而是照着真机（Dobot Nova 5）写的最小模型。
# 每一条结论都有出处，改之前先看注释。

#: 29999 一问一答往返延迟（ms，中位/max）= 2026-11-15 真机 30 次 GetPose 实测
SIM_ACK_MS: Tuple[float, float] = (3.9, 5.4)
#: ServoJ 的 ACK 延迟（ms），来自真机 60 帧 @30 ms 节拍实测
SIM_SERVO_ACK_MS: Tuple[float, float] = (6.2, 12.0)
#: 梯形速度曲线的加速段占比（0~0.5）
SIM_ACCEL_FRAC = 0.25
#: 关节加速度上限（度/秒²）：手册没给，按「0.4 s 到满速」反推的工程值
SIM_JOINT_ACCEL = kin.JOINT_MAX_SPEED / 0.4
#: 笛卡尔加速度上限（毫米/秒²），同样是工程值
SIM_LINEAR_ACCEL = 400.0
#: 算法队列最多积压多少条（真机固件同样会拒，只是阈值没公开）
SIM_MAX_QUEUE = 64


def _trapezoid(x: float) -> float:
    """归一的梯形速度曲线积分：输入时间进度 0~1，输出行程进度 0~1。

    匀加速 → 匀速 → 匀减速，两头各占 SIM_ACCEL_FRAC。真机起步和停稳都要
    几百毫秒，用「目标一变就瞬移」的阶跃模型会把加减速全吃掉。
    """
    a = SIM_ACCEL_FRAC
    x = 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)
    span = 1.0 - a
    if span <= 0:
        return x
    if x < a:
        area = x * x / (2.0 * a)
    elif x <= 1.0 - a:
        area = a / 2.0 + (x - a)
    else:
        y = 1.0 - x
        area = span - y * y / (2.0 * a)
    return max(0.0, min(1.0, area / span))


def _duration(delta: float, vmax: float, amax: float) -> float:
    """走 ``delta`` 要多久：梯形曲线，短行程退化成三角（到不了满速）。"""
    delta = abs(float(delta))
    vmax = max(1e-6, float(vmax))
    amax = max(1e-6, float(amax))
    if delta <= 0:
        return 0.0
    if delta >= vmax * vmax / amax:
        return delta / vmax + vmax / amax
    return 2.0 * math.sqrt(delta / amax)


class SimArm(Arm):
    """仿真臂：有运动学、有节拍、有算法队列——像一台真的 Nova 5，只是不带电。

    和「记下目标就瞬间到位」的旧版比，这里补的都是真机上咬过人的地方：

    * **关节角是唯一真相**：``pose()`` 由正解算出。旧版里 ``pose()`` 和
      ``joints()`` 是两份互不相干的数（``move_j`` 只改 pose、``move_joints``
      只改 joints），害得「拿 joints 去查碰撞」的代码在仿真里永远查 0 位姿。
    * **会花时间**：加速度受限的梯形曲线、按 ``SpeedFactor`` 缩放（手册六轴
      100°/s）。默认**虚拟时钟**——``wait=True`` 不真的睡，但 ``trace`` 里记的
      是真实时长，``estimate_s`` 也是真实秒数；``realtime=True`` 才按墙钟走。
    * **算法队列**：``wait=False`` 的指令排队执行，``current_cmd_id()`` 跟着跳；
      积压超过 ``max_queue`` 直接拒（真机固件也会拒，阈值没公开）。
    * **SpeedFactor 只管 MovJ/MovL**（手册 p.89）：ServoJ/ServoP 不受它影响，
      这一点在真机上被误判成「网络慢」。
    * **ServoJ/ServoP**：30 ms 节拍、``t`` 等效一阶低通、默认 latest-wins；
      停发之后保持最后目标、不跑位、不报警（手册没写，实测补上）。
    * **错误码照抄真机**：未使能时运动回 ``-1``（状态不允许）、关节超限 ``-2``、
      未知指令 ``-10000``——故障场景才有统一判据。

    ``command(text)`` 直接吃 29999 的文本协议，所以同一段脚本可以原样跑在
    ``SimArm`` 和 ``DobotArm`` 上。
    """

    name = "sim"

    #: 与真机一致的错误码（29999 回复的第一段）
    ERR_OK = 0
    ERR_STATE = -1           # 状态不允许（未使能 / 报警中）
    ERR_LIMIT = -2           # 关节超限
    ERR_IK = -4              # 位姿无解 / 不可达
    ERR_QUEUE = -50001       # 参数或队列错误
    ERR_UNKNOWN = -10000     # 指令不存在

    def __init__(self, pose: Sequence[float] = (400.0, 0.0, 300.0, 180.0, 0.0, 0.0),
                 max_speed_mm_s: float = 100.0, *,
                 joints: Optional[Sequence[float]] = None,
                 speed_factor: float = SAFE_SPEED_FACTOR,
                 realtime: bool = False,
                 max_queue: int = SIM_MAX_QUEUE,
                 servo_queue: str = "latest") -> None:
        if joints is not None:
            self._joints = [float(v) for v in joints]
            self._pose_exact: Optional[List[float]] = None
        else:
            self._pose = [float(v) for v in pose]
            solved = kin.ik(self._pose)
            self._joints = list(solved) if solved else [0.0] * 6
            #: 构造时给了位姿：控制器就报这个位姿（哪怕联合反解失败）
            self._pose_exact = list(self._pose)
        self._enabled = False
        self._error = 0
        self._factor = max(1.0, min(float(speed_factor), 100.0))
        self.max_speed_mm_s = float(max_speed_mm_s)

        #: 虚拟时钟（秒）：真机的时间感全在它身上。advance() 推，realtime=True 跟墙钟。
        self._clock = 0.0
        self._wall0 = time.perf_counter()
        self._realtime = bool(realtime)

        #: 算法队列：wait=False 的指令排这里，FIFO 执行（真机就是这么排的）
        self._queue: Deque[Dict[str, object]] = deque()
        self._active: Optional[Dict[str, object]] = None
        self._cmd_id = 0
        self.max_queue = max(1, int(max_queue))

        #: ServoJ/ServoP：目标关节 + 时间常数 t（手册默认 0.1，实测建议 0.03）
        self._servo_target: Optional[List[float]] = None
        self._servo_t = 0.1
        self._servo_frames = 0
        self._servo_queue = "fifo" if str(servo_queue).lower() == "fifo" else "latest"
        self._servo_pending: Deque[Tuple[float, List[float]]] = deque()

        #: 逆解缓存：位姿 → 关节角。批量下发时 kin.ik 是主要开销（见 _solve）
        self._ik_cache: Dict[tuple, List[float]] = {}

        self.trace: List[Dict[str, object]] = []
        self.trace.append({"t": 0.0, "event": "connect", "note": "仿真臂（虚拟时钟）"})

    # -- 时钟 -------------------------------------------------------------
    def advance(self, seconds: float) -> None:
        """把虚拟时钟往前推（分段推进，让队列真的往前跑）。"""
        remaining = max(0.0, float(seconds))
        for _ in range(100000):
            if remaining <= 1e-9:
                break
            self._sync()
            if self._active is None and not self._queue:
                self._clock += remaining
                break
            need = 0.002
            if self._active is not None:
                dur = float(self._active.get("dur") or 0.0)
                need = max(0.001, dur - (self._now() - float(self._active["t0"])))
            step = min(remaining, need)
            self._clock += step
            remaining -= step
        self._sync()

    def _now(self) -> float:
        if self._realtime:
            return time.perf_counter() - self._wall0
        return self._clock

    def _tick(self) -> None:
        if self._realtime:
            self._clock = time.perf_counter() - self._wall0

    # 生命周期
    def connect(self) -> None:
        self.trace.append({"t": self._now(), "event": "connect"})

    def close(self) -> None:
        self.trace.append({"t": self._now(), "event": "close"})

    # 状态
    def enabled(self) -> bool:
        return self._enabled

    def pose(self) -> List[float]:
        """当前位姿。位姿指令走完报**下发的那个位姿**，其余时候由正解算。"""
        return list(self._pose_exact if self._pose_exact is not None else kin.fk(self._joints))

    def joints(self) -> List[float]:
        self._sync()
        return list(self._joints)

    def ik(self, pose: Sequence[float],
           near: Optional[Sequence[float]] = None) -> Optional[List[float]]:
        """仿真没有控制器，用本地数值反解（真机走 InverseKin，见 DobotArm.ik）。"""
        return kin.ik(pose, seed=near if near is not None else self._joints)

    # 指令
    def enable(self) -> None:
        self._enabled = True
        self._error = 0
        self.trace.append({"t": self._now(), "event": "enable"})

    def disable(self) -> None:
        self._enabled = False
        self._queue.clear()
        self._active = None
        self.trace.append({"t": self._now(), "event": "disable"})

    def speed_factor(self, percent: float) -> float:
        self._factor = max(1.0, min(float(percent), 100.0))
        return self._factor

    def estimate_seconds(self, target: Sequence[float]) -> float:
        """走这段位姿大约几秒：笛卡尔距离 / 满速，与旧版同一口径。

        刻意不用「关节行程 / 关节速度」：旧版这个数是 1.0 / 2.0 秒这种整齐值，
        下游（pick.preview、网页上的预计耗时）都按这个口径展示。
        """
        dist = math.dist(self.pose()[:3], list(target)[:3])
        speed = self.max_speed_mm_s * self._factor / 100.0
        return dist / speed if speed > 0 else 0.0

    #: 逆解缓存命中率上限：位姿是浮点，按 0.1 mm / 0.01° 量化后再当 key
    IK_CACHE_DECIMALS = 1

    def _solve(self, target: Sequence[float]) -> Optional[List[float]]:
        """带缓存的逆解。

        ``kin.ik`` 是 6 变量阻尼最小二乘，本机实测 **84~160 ms/次**。批量下发
        （一条 300 mm 的笔画约 20~40 个点、故障场景一次推 200 条）如果每次都
        重新解，光逆解就要几十秒——真机是一条指令几毫秒，这个开销纯属仿真
        的失真。这里按量化后的位姿缓存：同一条轨迹反复经过同一批点时直接命中。
        """
        cur = kin.fk(self._joints)
        if all(abs(float(cur[i]) - float(target[i])) <= 0.5 for i in range(3)) and \
                all(self._ang_diff(cur[i], float(target[i])) <= 0.5 for i in range(3, 6)):
            return list(self._joints)       # 已经在目标上：真机也不会重新规划
        key = tuple(round(float(v), self.IK_CACHE_DECIMALS) for v in target)
        cached = self._ik_cache.get(key)
        if cached is not None:
            return list(cached)
        q = kin.ik(list(target), seed=self._joints)
        if q is not None:
            if len(self._ik_cache) >= 4096:
                self._ik_cache.clear()
            self._ik_cache[key] = list(q)
        return q

    def _line_to_pose(self, target: Sequence[float]) -> List[float]:
        """兜底插补：把位姿差折算成关节增量（只走「控制器说无解」的仿真路径）。"""
        cur = kin.fk(self._joints)
        out = list(self._joints)
        for i in range(6):
            delta = float(target[i]) - float(cur[i])
            if i >= 3:
                delta = (delta + 180.0) % 360.0 - 180.0
            out[i] += delta
        return kin.clamp(out)

    # -- 算法队列 ---------------------------------------------------------
    def _sync(self) -> None:
        """把队列推进到当前时刻。所有读状态的入口都先调它。"""
        self._tick()
        for _ in range(1000):
            if self._active is None:
                if not self._queue:
                    return
                self._active = self._queue.popleft()
                self._active["t0"] = self._now()
                cmd_id = self._active.get("id")
                if isinstance(cmd_id, int):
                    self._cmd_id = cmd_id
            act = self._active
            start = act["start"]
            end = act["end"]
            dur = float(act.get("dur") or 0.0)
            p = 1.0 if dur <= 0 else (self._now() - float(act["t0"])) / dur
            f = _trapezoid(p)
            self._joints = [float(start[i]) + (float(end[i]) - float(start[i])) * f
                            for i in range(6)]
            if p < 1.0:
                return
            self._joints = [float(v) for v in end]
            self._finish(act)
            self._active = None

    def _finish(self, act: Dict[str, object]) -> None:
        self._pose_exact = list(act["pose"]) if act.get("pose") is not None else None
        self.trace.append({"t": round(self._now(), 3), "event": str(act["event"]),
                           "to": [round(v, 3) for v in (act["pose"] or self._joints)],
                           "eta_s": round(float(act.get("dur") or 0.0), 2),
                           "id": act.get("id")})

    def _enqueue(self, start: Sequence[float], end: Sequence[float], dur: float,
                 event: str, pose: Optional[Sequence[float]] = None) -> int:
        pending = len(self._queue) + (1 if self._active is not None else 0)
        if pending >= self.max_queue:
            raise ArmError("-50001 算法队列已满（%d 条）：先 wait 或降速，真机同样会拒"
                           % pending)
        self._cmd_id += 1
        self._queue.append({"id": self._cmd_id, "start": list(start), "end": list(end),
                            "dur": float(dur), "event": event,
                            "pose": list(pose) if pose is not None else None})
        return self._cmd_id

    def _drain(self, timeout: float) -> bool:
        """等到队列空。虚拟时钟下直接把时间推过去，不真睡。"""
        deadline = time.perf_counter() + max(0.05, float(timeout))
        for _ in range(100000):
            self._sync()
            if self._active is None and not self._queue:
                return True
            need = 0.002
            if self._active is not None:
                dur = float(self._active.get("dur") or 0.0)
                need = max(0.0005, dur - (self._now() - float(self._active["t0"])))
            if self._realtime:
                time.sleep(min(0.002, max(0.0005, need)))
            else:
                self._clock += need
            if time.perf_counter() > deadline:
                return False
        return False

    def _goto(self, target: Sequence[float], kind: str,
              wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        if len(target) != 6:
            raise ValueError("目标姿态需要 6 个数")
        if not self._enabled or self._error:
            raise ArmError("%d 状态不允许：臂未使能或报警中（先 enable()）" % self.ERR_STATE)
        self._sync()
        q = self._solve(list(target))
        exact = q is not None
        if not exact:
            # 真机控制器也会尽量给你一个解（哪怕残差大）；仿真这边同理：
            # 解不出来就沿直线插补，让「不可达」在关节上是可见的（超限会报 -2）。
            q = self._line_to_pose(target)
        travel = max(abs(a - b) for a, b in zip(q, self._joints))
        vmax = kin.JOINT_MAX_SPEED * self._factor / 100.0
        dur = _duration(travel, vmax, SIM_JOINT_ACCEL)
        cmd_id = self._enqueue(self._joints, q, dur, kind, pose=target)
        result: Dict[str, object] = {"ok": True, "arm": self.name, "cmd": kind,
                                     "to": [round(float(v), 3) for v in target],
                                     "eta_s": round(dur, 2), "id": cmd_id}
        if not exact:
            result["ik"] = False
        if wait:
            result["arrived"] = self._drain(timeout)
        return result

    def move_j(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._goto(pose, "MovJ", wait, timeout)

    def move_l(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._goto(pose, "MovL", wait, timeout)

    def move_joints(self, joints: Sequence[float], wait: bool = True,
                    timeout: float = 30.0) -> Dict[str, object]:
        """关节空间 MovJ：不需要逆解，轨迹只取决于关节插值（和真机一致）。"""
        if len(joints) != 6:
            raise ValueError("关节角需要 6 个数")
        if not self._enabled or self._error:
            raise ArmError("%d 状态不允许：臂未使能或报警中（先 enable()）" % self.ERR_STATE)
        bad = kin.within_limits(joints)
        if bad:
            raise ArmError("%d 关节超限：%s" % (self.ERR_LIMIT, bad))
        self._sync()
        q = [float(v) for v in joints]
        travel = max(abs(a - b) for a, b in zip(q, self._joints))
        vmax = kin.JOINT_MAX_SPEED * self._factor / 100.0
        dur = _duration(travel, vmax, SIM_JOINT_ACCEL)
        cmd_id = self._enqueue(self._joints, q, dur, "MovJ(joint)")
        result: Dict[str, object] = {"ok": True, "arm": self.name, "cmd": "MovJ(joint)",
                                     "joints": [round(v, 3) for v in q],
                                     "eta_s": round(dur, 2), "id": cmd_id}
        if wait:
            result["arrived"] = self._drain(timeout)
        return result

    # -- ServoJ / ServoP（实时流式，真机 30 ms 一帧）-----------------------
    def servo_j(self, joints: Sequence[float], t: Optional[float] = None,
                aheadtime: Optional[float] = None, gain: Optional[float] = None,
                wait: bool = False) -> Dict[str, object]:
        """流式关节指令。真机：30 ms 一帧、ACK 中位 6.2 ms、目标=当前时零漂移。

        ``t`` 是「这一段走多久」，手册默认 0.1——**这就是那个坑**：控制器会朝
        ``t`` 秒前的目标慢慢挪，等效一阶低通，看起来像网络慢。首测用 0.03。
        """
        if len(joints) != 6:
            raise ValueError("关节角需要 6 个数")
        if not self._enabled or self._error:
            raise ArmError("%d 状态不允许" % self.ERR_STATE)
        bad = kin.within_limits(joints)
        if bad:
            raise ArmError("%d 关节超限：%s" % (self.ERR_LIMIT, bad))
        self._servo_t = 0.03 if t is None else max(0.004, min(float(t), 3600.0))
        self._servo_frames += 1
        if self._servo_queue == "fifo":
            self._servo_pending.append((self._servo_t, [float(v) for v in joints]))
        self._servo_target = [float(v) for v in joints]
        self._sync()
        return {"ok": True, "arm": self.name, "cmd": "ServoJ",
                "t": self._servo_t, "frames": self._servo_frames}

    def servo_p(self, pose: Sequence[float], t: Optional[float] = None) -> Dict[str, object]:
        q = kin.ik(list(pose), seed=self._joints)
        if q is None:
            raise ArmError("%d 位姿不可达：%s" % (self.ERR_IK, list(pose)))
        return self.servo_j(q, t=t)

    def stop(self) -> None:
        self._queue.clear()
        self._active = None
        self._servo_target = None
        self._servo_pending.clear()
        self.trace.append({"t": self._now(), "event": "stop"})

    # -- 与真机同名的队列接口 --------------------------------------------
    def current_cmd_id(self) -> int:
        self._sync()
        return self._cmd_id

    def wait_cmd_id(self, cmd_id: int, timeout: float = 30.0) -> bool:
        self._drain(timeout)
        return self._cmd_id >= int(cmd_id)

    def movl_cp(self, pose: Sequence[float], cp: int) -> int:
        """带 cp 的 MovL（真机返回 ResultID）。仿真里 cp 只影响是否停车，忽略。"""
        return int(self.move_l(pose, wait=False).get("id") or 0)

    def movl_r(self, pose: Sequence[float], r_mm: float,
               accel: Optional[int] = None) -> int:
        return int(self.move_l(pose, wait=False).get("id") or 0)

    # -- 29999 文本协议 ---------------------------------------------------
    def command(self, text: str) -> str:
        """吃一条 29999 dashboard 指令，回一条真机格式的回复。

        有了它，同一段脚本/故障场景可以原样跑在 ``SimArm`` 和 ``DobotArm`` 上，
        不用为仿真写第二套接口。不认识的指令照真机回 ``-10000``。
        """
        cmd = str(text).strip()
        name = cmd.split("(", 1)[0].strip()
        try:
            if name == "GetPose":
                body = ",".join(_num(v) for v in self.pose())
                return "0,{%s},GetPose();" % body
            if name == "GetAngle":
                body = ",".join(_num(v) for v in self.joints())
                return "0,{%s},GetAngle();" % body
            if name == "RobotMode":
                mode = 9 if self._error else (5 if self._enabled else 4)
                return "0,{%d},RobotMode();" % mode
            if name == "GetCurrentCommandID":
                return "0,{%d},GetCurrentCommandID();" % self.current_cmd_id()
            if name == "GetErrorID":
                body = "[%d]" % self._error if self._error else "[]"
                return "0,{%s},GetErrorID();" % body
            if name == "EnableRobot":
                self.enable()
                return "0,{},EnableRobot();"
            if name == "DisableRobot":
                self.disable()
                return "0,{},DisableRobot();"
            if name in ("ClearError", "PowerOn", "RequestControl", "StopDrag"):
                if name == "ClearError":
                    self._error = 0
                return "0,{},%s();" % name
            if name == "Stop":
                self.stop()
                return "0,{},Stop();"
            if name == "StartDrag":
                self.trace.append({"t": self._now(), "event": "drag"})
                return "0,{},StartDrag();"
            if name == "SpeedFactor":
                inner = _numbers(cmd)
                if not inner:
                    raise ArmError("-50001 SpeedFactor 需要一个百分比")
                pct = int(self.speed_factor(inner[0]))
                return "0,{%d},SpeedFactor(%d);" % (pct, pct)
            if name == "CP":
                inner = _numbers(cmd)
                pct = int(inner[0]) if inner else 0
                return "0,{%d},CP(%d);" % (pct, pct)
            if name in ("MovJ", "MovL", "Arc", "Circle", "StartPath"):
                return self._dash_move(name, cmd)
            if name in ("ServoJ", "ServoP"):
                if not self._enabled or self._error:
                    return "-1,{%d},%s(...);" % (self._servo_frames, name)
                inner = _numbers(cmd)
                if name == "ServoJ" and len(inner) >= 6:
                    self.servo_j(inner[:6], t=inner[6] if len(inner) > 6 else None)
                return "0,{%d},%s(...);" % (self._servo_frames, name)
        except ArmError as exc:
            code = str(exc).split(" ", 1)[0]
            return "%s,{},%s(...);" % (code, name)
        return "-10000,{},%s();" % name

    def _dash_move(self, name: str, cmd: str) -> str:
        if not self._enabled or self._error:
            return "-1,{},%s(...);" % name
        if "joint=" in cmd:
            inner = _numbers(cmd[cmd.find("joint="):])
            if len(inner) < 6:
                return "-50001,{},%s(...);" % name
            result = self.move_joints(inner[:6], wait=False)
        else:
            inner = _numbers(cmd)
            if len(inner) < 6:
                return "-50001,{},%s(...);" % name
            result = (self.move_j(inner[:6], wait=False) if name == "MovJ"
                      else self.move_l(inner[:6], wait=False))
        return "0,{%d},%s(...);" % (int(result.get("id") or 0), name)

class DobotArm(Arm):
    """真机：TCP 29999 发 dashboard 指令（行文本、一问一答）。

    未在 Nova 5 上验证；第一次接真机先只跑 ``check()``（只读 GetPose/RobotMode），
    确认能读再 enable。指令名集中在 ``CMD_*`` 常量里，固件不一致时改这里。
    """

    name = "dobot"

    CMD_ENABLE = "EnableRobot()"
    CMD_DISABLE = "DisableRobot()"
    CMD_CLEAR = "ClearError()"
    CMD_REQUEST_CONTROL = "RequestControl()"
    CMD_POWER_ON = "PowerOn()"
    CMD_SPEED = "SpeedFactor({:.0f})"
    CMD_MOV_J = "MovJ(pose={})"
    # cp = 平滑过渡比例（0~100，手册「平滑过渡参数」）：>0 时机器人不停车穿过中间点，
    # 否则每条 MovL 都在点上做一次减速-再加速，写出来就是一抖一抖的。最后一点必须 cp=0。
    CMD_MOV_L = "MovL(pose={},cp=0)"
    CMD_MOV_L_CP = "MovL(pose={},cp={:d})"
    CMD_CP = "CP({:d})"
    CMD_MOV_JOINT = "MovJ(joint={})"
    CMD_POSE = "GetPose()"
    CMD_ANGLE = "GetAngle()"
    CMD_MODE = "RobotMode()"
    # 当前执行到算法队列的第几条（手册「GetCurrentCommandID」，返回 0,{ResultID}）。
    # 千万别用 GetQueuedCmdCurrentIndex()：本机固件回 -10000（命令不存在）。
    CMD_ARRIVE_INDEX = "GetCurrentCommandID()"
    CMD_STOP = "Stop()"
    # 拖动示教（背驱）：切进去后电机变柔顺，人可以直接用手把臂掰到任意姿态，
    # 不需要写任何动作指令。退出时保持当前姿态。
    CMD_DRAG_START = "StartDrag()"
    CMD_DRAG_STOP = "StopDrag()"

    def __init__(self, ip: str = DEFAULT_IP, port: int = CONTROL_PORT,
                 timeout: float = 3.0, read_timeout: float = 2.0) -> None:
        self.ip = ip
        self.port = int(port)
        self.timeout = float(timeout)
        self.read_timeout = float(read_timeout)
        self._sock: Optional[socket.socket] = None
        self._lock = threading.RLock()
        self._last_error: Optional[str] = None
        self._factor = SAFE_SPEED_FACTOR

    # -- 连接 -------------------------------------------------------------
    def connect(self) -> None:
        if self._sock is not None:
            return
        try:
            sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        except OSError as exc:
            raise ArmError(f"连不上 {self.ip}:{self.port}（{exc}）—— "
                           f"先确认网线、PC 是 192.168.5.x、臂在 TCP 模式") from exc
        sock.settimeout(self.read_timeout)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._sock = sock

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    # -- 一问一答 ---------------------------------------------------------
    #: 29999 只接受一个客户端：别的进程（示教器/DobotStudio/另一个服务）
    #: 一插进来，我们的连接就会被控制器断开。断开后再发就报 10053。
    #: 所以这里**自动重连一次再重试**，别让一次抢占就废掉整次下发。
    RECONNECT_TRIES = 1

    def command(self, text: str) -> str:
        """发一条 29999 原文，回一条原文。

        和 ``SimArm.command`` 同一个签名——故障场景/上线自检可以拿同一段文本
        先跑仿真再跑真机，不用写两套。真机这边不做任何解释，
        错误码原样返回（``-1`` 状态不允许 / ``-10000`` 指令不存在 / ``-50001`` 参数错），
        要抛异常请用 ``_command``。
        """
        try:
            return self._command(str(text))
        except ArmError as exc:
            return str(exc)

    def _drop(self) -> None:
        """内部用：把坏掉的 socket 丢掉，下次 _command 会重新连。"""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _command(self, text: str) -> str:
        """发一条指令并等回复；连接被对端断开时自动重连一次。"""
        last: Optional[Exception] = None
        for attempt in range(self.RECONNECT_TRIES + 1):
            with self._lock:
                if self._sock is None:
                    try:
                        self.connect()
                    except ArmError as exc:
                        last = exc
                        if attempt < self.RECONNECT_TRIES:
                            time.sleep(0.4)
                            continue
                        raise
                assert self._sock is not None
                try:
                    self._sock.sendall((text + "\n").encode("ascii", "ignore"))
                    return self._check(self._read())
                except OSError as exc:
                    # 10053/10054/WSAECONNRESET：对端把连接掐了，多半是被别的客户端顶掉
                    self._drop()
                    last = ArmError(
                        f"29999 连接被控制器断开（{exc}），重连"
                        f"{self.RECONNECT_TRIES} 次后仍失败 —— "
                        "通常是别的程序（示教器 / DobotStudio / 另一个 handlink）"
                        "占着控制口，先关掉它们")
                except ArmError as exc:
                    # 指令层面报错（-xxxx）不是连接问题，重试没意义；
                    # 只有「连接断了 / 没回复」才值得重连再试
                    if not isinstance(exc, _LinkLost) and "没有回复" not in str(exc):
                        raise
                    self._drop()
                    last = exc
            if attempt < self.RECONNECT_TRIES:
                time.sleep(0.4)
        raise last or ArmError("29999 通信失败")

    def _read(self) -> str:
        assert self._sock is not None
        chunks: List[bytes] = []
        deadline = time.time() + self.read_timeout
        while time.time() < deadline:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                break
            except OSError as exc:
                # 10054/10053 这类是连接层面的事，交给 _command 去重连
                raise _LinkLost(f"接收失败（{exc}）") from exc
            if not data:
                break
            chunks.append(data)
            text = b"".join(chunks).decode("utf-8", "ignore")
            if ";" in text or "{" in text and "}" in text and "\n" in text:
                break
        text = b"".join(chunks).decode("utf-8", "ignore").strip()
        if not text:
            raise ArmError("臂没有回复（超时）—— 检查是否别的程序占着 29999")
        return text

    @staticmethod
    def _check(text: str) -> str:
        low = text.lower()
        if "control mode" in low and "tcp" in low:
            raise ArmError("臂不在 TCP 模式：在示教器 / DobotStudio 里把控制模式切到 TCP")
        if text.startswith("-"):
            raise ArmError(text)
        return text

    def wait_pose_close(self, target: Sequence[float], tol: float = 2.0,
                        timeout: float = 30.0, settle: int = 3) -> bool:
        """等 TCP 真的到 target 附近（连续 settle 帧都在 tol 内）。

        为什么不用 wait_until：那个是给"走完一条指令"用的；
        批量下发 MovL 之后队列里还有几十条指令，得看**实际位置**稳下来。
        """
        deadline = time.time() + timeout
        hits = 0
        while time.time() < deadline:
            p = self.pose()
            near = (all(abs(p[i] - target[i]) <= tol for i in range(3))
                    and all(self._ang_diff(p[i], target[i]) <= max(tol, 3.0)
                            for i in range(3, 6)))
            hits = hits + 1 if near else 0
            if hits >= settle:
                return True
            time.sleep(0.05)
        return False

    def queue_count(self) -> int:
        """控制器里还剩多少条指令（0 表示已经执行完）。

        .. warning::
           **``GetQueuedCmdCurrentIndex()`` 这条命令不存在**：主控固件实测回
           ``-10000,{},GetQueuedCmdCurrentIndex();``（命令不存在），手册 V4.6.0 全文
           也没有它。本方法因此**永远返回 -1**，不能拿它做节流判断。
           节流请看 ``current_cmd_id()`` / ``wait_cmd_id()``：记住下发时 MovL 返回的
           ResultID，再轮询 ``GetCurrentCommandID()`` 比大小（手册「GetCurrentCommandID」）。
        """
        try:
            text = self._command("GetQueuedCmdCurrentIndex()")
        except ArmError:
            return -1
        start, end = text.find("{"), text.find("}")
        if start < 0 or end <= start:
            return -1
        try:
            return int(float(text[start + 1:end]))
        except ValueError:
            return -1

    def _idx(self, text: str, cmd: str) -> str:
        return f"{text} [{cmd}]"

    # -- 状态 -------------------------------------------------------------
    def enabled(self) -> bool:
        try:
            reply = self._command(self.CMD_MODE)
        except ArmError:
            return False
        # RobotMode() -> 0,{5},... 5 = 使能空闲；4 = 未使能
        try:
            start, end = reply.find("{"), reply.find("}", reply.find("{"))
            if start >= 0 and end > start:
                return int(float(reply[start + 1:end])) in (5, 6, 7, 8, 9, 10, 11)
        except (ValueError, TypeError):
            pass
        return False

    def pose(self) -> List[float]:
        return parse_pose(self._check(self._command(self.CMD_POSE)))

    def joints(self) -> List[float]:
        text = self._check(self._command(self.CMD_ANGLE))
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            raise ArmError(f"GetAngle 返回异常: {text!r}")
        return [float(x) for x in text[start + 1:end].split(",") if x.strip()]

    # -- 指令 -------------------------------------------------------------
    def ik(self, pose: Sequence[float],
           near: Optional[Sequence[float]] = None) -> Optional[List[float]]:
        """控制器自己的 ``InverseKin``：真机的解和它自己执行时完全一致。

        带 ``useJointNear=1``，让控制器挑离当前关节最近的那个支管——不然同一个
        位姿有多个解，臂可能突然翻腕。返回 ``None`` 表示控制器说无解。
        """
        p = [float(v) for v in pose]
        near = [float(v) for v in (near if near is not None else self.joints())]
        joints = ",".join(f"{v:.3f}" for v in near)
        text = self._command(
            f"InverseKin({p[0]:.3f},{p[1]:.3f},{p[2]:.3f},"
            f"{p[3]:.3f},{p[4]:.3f},{p[5]:.3f},"
            f"useJointNear=1,jointNear={{{joints}}})")
        start, end = text.find("{"), text.find("}")
        if start < 0 or end <= start:
            return None
        try:
            vals = [float(x) for x in text[start + 1:end].split(",") if x.strip()]
        except ValueError:
            return None
        return vals if len(vals) == 6 else None

    def take_control(self) -> None:
        """请求切到 TCP 模式（不必再开 DobotStudio）。

        协议规定只在「未上电」或「下使能（非暂停、抱闸未松）」时才允许切换。
        """
        self._check(self._command(self.CMD_REQUEST_CONTROL))

    def power_on(self) -> None:
        self._check(self._command(self.CMD_POWER_ON))

    def enable(self) -> None:
        self._check(self._command(self.CMD_ENABLE))

    def disable(self) -> None:
        self._check(self._command(self.CMD_DISABLE))

    def clear_error(self) -> None:
        self._check(self._command(self.CMD_CLEAR))

    def speed_factor(self, percent: float) -> float:
        pct = max(1.0, min(float(percent), 100.0))
        self._check(self._command(self.CMD_SPEED.format(pct)))
        self._factor = pct
        return pct

    def move_j(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._move(self.CMD_MOV_J, "MovJ", pose, wait, timeout)

    def move_l(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._move(self.CMD_MOV_L, "MovL", pose, wait, timeout)

    # 下面几个给 service.arm_trace 的「连续书写」用。默认实现只是把 cp 丢掉、退回
    # 逐点 move_l，仿真臂/测试替身不实现也能跑；真机 DobotArm 重写了它们。
    def movl_cp(self, pose: Sequence[float], cp: int) -> int:
        self.move_l(pose, wait=False)
        return 0

    def movl_r(self, pose: Sequence[float], r_mm: float,
               accel: Optional[int] = None) -> int:
        self.move_l(pose, wait=False)
        return 0

    def current_cmd_id(self) -> int:
        return 0

    def wait_cmd_id(self, cmd_id: int, timeout: float = 30.0) -> bool:
        return True

    # -- 平滑过渡（cp）-----------------------------------------------------
    def set_cp(self, percent: int) -> int:
        """全局平滑过渡比例 ``CP(R)``（手册「CP」），R 取 0~100，未设置时为 0。

        设成 0 就是回到「每点必停」。注意它只影响**没在指令里写 cp 的**那条指令。
        """
        pct = max(0, min(int(round(percent)), 100))
        self._check(self._command(self.CMD_CP.format(pct)))
        return pct

    def movl_cp(self, pose: Sequence[float], cp: int) -> int:
        """发一条带 cp 的 MovL，返回它的算法队列 ResultID（立即返回，不等运动）。"""
        text = self._check(self._command(
            self.CMD_MOV_L_CP.format(fmt_pose(pose), max(0, min(int(cp), 100)))))
        result = parse_result_id(text)
        if result is None:
            raise ArmError(f"MovL 没返回 ResultID，无法判断到位: {text!r}")
        return result

    def movl_r(self, pose: Sequence[float], r_mm: float,
               accel: Optional[int] = None) -> int:
        """发一条带 ``r``（圆角半径，毫米）的 MovL，返回算法队列 ResultID。

        慢速书写时 ``cp``（百分比）圆出来的角很小，几乎等于不停；``r`` 是绝对毫米，
        不随速度缩水，所以低速下磨圆拐角更可靠。``accel`` 给了就一并下发 ``a=``。

        **``r`` 必须是整数**：2026-11-25 实测 ``r=3`` 返回 ``0,{1}`` 成功，
        ``r=3.0`` 返回 ``-50001``（参数类型错误）。所以这里一律取整再发。
        """
        args = ["pose=" + fmt_pose(pose),
                # 四舍五入用 floor(x+0.5)：Python 的 round() 是「银行家舍入」，
                # round(2.5)=2 会让用户以为填 2.5 却拿到 2。
                "r=%d" % max(0, int(math.floor(float(r_mm) + 0.5)))]
        if accel:
            args.append("a=%d" % max(1, min(int(accel), 100)))
        text = self._check(self._command("MovL(" + ",".join(args) + ")"))
        result = parse_result_id(text)
        if result is None:
            raise ArmError(f"MovL 没返回 ResultID，无法判断到位: {text!r}")
        return result

    def current_cmd_id(self) -> int:
        """``GetCurrentCommandID()``：机器人正在执行的算法队列 ID。"""
        result = parse_result_id(self._check(self._command(self.CMD_ARRIVE_INDEX)))
        if result is None:
            raise ArmError("GetCurrentCommandID 返回异常")
        return result

    def wait_cmd_id(self, cmd_id: int, timeout: float = 30.0) -> bool:
        """等 ``GetCurrentCommandID() >= cmd_id``。

        注意：控制器的「当前指令 ID」在**开始执行**那条指令时就跳过去了，所以 ID 到了
        不代表停稳。手册的判据是「ID 相等 **且** RobotMode()==5（使能且空闲）」，这里
        照做：ID 追平后再确认一次空闲。要比 ID 大小，别再拿 GetQueuedCmdCurrentIndex。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.current_cmd_id() >= cmd_id:
                idle = self._command(self.CMD_MODE)
                try:
                    mode = int(float(idle[idle.find("{") + 1:idle.find("}")]))
                except ValueError:
                    mode = -1
                if mode == 5:
                    return True
            time.sleep(0.02)
        return False

    def move_joints(self, joints: Sequence[float], wait: bool = True,
                    timeout: float = 30.0) -> Dict[str, object]:
        """关节空间 MovJ：不需要逆解，轨迹只取决于关节插值。"""
        if len(joints) != 6:
            raise ValueError("关节角需要 6 个数")
        self._check(self._command(self.CMD_MOV_JOINT.format(fmt_joints(joints))))
        result: Dict[str, object] = {"ok": True, "arm": self.name, "cmd": "MovJ(joint)",
                                     "joints": [round(float(v), 3) for v in joints]}
        if wait:
            result["arrived"] = self.wait_until_joints(joints, timeout=timeout)
        return result

    def _move(self, template: str, kind: str, pose: Sequence[float],
              wait: bool, timeout: float) -> Dict[str, object]:
        cmd = template.format(fmt_pose(pose))
        self._check(self._command(cmd))
        result: Dict[str, object] = {"ok": True, "arm": self.name, "cmd": kind, "to": list(pose)}
        if wait:
            result["arrived"] = self.wait_until(pose, timeout=timeout)
        return result

    def stop(self) -> None:
        self._check(self._command(self.CMD_STOP))

    def start_drag(self) -> None:
        """进入拖动示教（背驱）：电机变柔顺，人可以手扶着把臂摆到任意姿态。"""
        self._check(self._command(self.CMD_DRAG_START))

    def stop_drag(self) -> None:
        """退出拖动示教，保持当前姿态，重新由指令控制。"""
        self._check(self._command(self.CMD_DRAG_STOP))

    # -- 只读自检 ---------------------------------------------------------
    def check(self) -> Dict[str, object]:
        """第一次接真机跑这个：只读，不动。"""
        self.connect()
        out: Dict[str, object] = {"ip": self.ip, "port": self.port}
        out["mode_raw"] = self._command(self.CMD_MODE)
        out["pose"] = self.pose()
        out["joints"] = self.joints()
        out["enabled"] = self.enabled()
        return out
