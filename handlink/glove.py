"""MotionVenus 数据手套 → 灵巧手姿态（服务内置版）。

链路：MotionVenus --UDP 127.0.0.1:5001--> 本模块 --> App.apply() --> CAN / 仿真总线

以前是 ``tools/glove_bridge.py``：独立进程、每帧再 POST 一次 ``/pose``。现在搬进
服务本体，少了那一跳 HTTP，手套也就能"长"在插件页面上：

* 插件页面直接调 ``/glove/start``、``/glove/mode``、``/glove/zero``，不用再另起
  一个 8767 控制口；
* 帧解析、关节链求和、零点、增益都在这个模块里，**不需要手套也不需要 socket**
  就能单测（见 ``tests/test_glove.py``）；
* ``python -m handlink.glove`` 仍然保留独立跑法（``tools/glove_bridge.py`` 已经
  改成它的薄壳），服务没起时照样能用。

帧格式（实测）：一帧拆成 ``subpackage 1/2``、``2/2`` 两个 UDP 包，按 ``fn``
合并后是 32 关节 × 6 个数 = ``[x, y, z, ex, ey, ez]``（位置 + 欧拉角 ZXY
relative，单位度）。右手是关节 0~15，左手 16~31。

角度换算沿用实测参数：每根手指取关节链上各关节相对**手腕**的偏转角之和，再减掉
最近 10 秒里该指的**最小值**（摊平基线），最后按"握满约 150°"折算到电机行程。

为什么不能直接用欧拉角（2026-11-20 修）：帧里的 ex/ey/ez 是**整只手在同一参考
系里的绝对姿态**，不是"相对上一节"。手臂一转，16 个关节的姿态一起变，老算法
max(|ey|,|ez|) 就跟着涨——表现出来就是"手没动、胳膊转一圈、大拇指自己动"。
拇指最惨，因为它的骨头几乎垂直于小臂：绕小臂的滚转在四指那儿大多被吃进 roll
（ex，本来就被丢掉），到拇指这儿整个投影到 ey/ez 上；再加上 102 是三节求和、
107 又是 2~3 节，同一份干扰被算了两遍。

所以默认改成 frame="rel"：先用 ZXY 欧拉角拼出每个关节的旋转矩阵，再左乘手腕
（本手第 0 节）矩阵的转置，得到"相对手腕"的旋转，取它的转轴-转角幅值当弯折量。
整只手一起转时 R0ᵀ·Rj 不变，干扰被精确除掉。frame="abs" 是 1.1.x 的老算法，
留着做对照；frame="rel-ext" 换另一种 ZXY 读法（厂家没写清楚是 intrinsic 还是
extrinsic，真机上用 tools/glove_probe.py 比一比哪个更稳）。
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import re
import select
import socket
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple

from . import spec

# ---------------------------------------------------------------- 协议常量

BONES = 32                 # 每帧 32 个关节
PER = 6                    # 每个关节 6 个数
DEFAULT_UDP = 5001
DEFAULT_BIND = "127.0.0.1"
DEFAULT_RATE = 50.0        # 最多下发多少 Hz（手套本身约 20~40 Hz）
DEFAULT_SPEED = 360.0      # 厂家默认 60°/s 太慢，跟手用满速
DEFAULT_TORQUE = 0.6
FULL_SCALE_DEG = 150.0     # 老算法实测：摊平 30~60、握满 150~230（rel 下量级相同）
BASELINE_SECONDS = 10.0    # 自动零点窗口
LIVE_SECONDS = 1.5         # 最近多久收到帧才算「跟随中」
DEADBAND_MAX = 45.0        # 单指死区上限（度）：拇指残留抖动就给它 5~8°
SNAPSHOT_FRAMES = 5       # 存点位时默认取最近几帧的中位数
DIP_SECONDS = 0.25        # 连续低于基线这么久，才承认这是新的"摊平"
FLOOR_EPS = 1.5           # 离基线多少度以内算"贴着"
RISE_AFTER = 6.0          # 贴着基线这么久，才允许慢慢往上追（温度漂移）
RISE_RATE = 2.0           # 往上追的速度上限（度/秒）
SPIKE_DEG = 25.0          # 一帧内跳这么多、下一帧又弹回，才算传感器毛刺
DEFAULT_FRAME = "rel"      # rel = 减掉手腕（整只手的公共旋转）；abs = 1.1.x 老算法
FRAMES: Tuple[str, ...] = ("rel", "rel-ext", "abs")
FRAME_HINT = " / ".join(FRAMES)     # 报错和提示里共用，省得每处再拼一次
AXES_FILE = Path(__file__).resolve().parent / "config" / "glove_axes.json"
AXIS_MIN_DEG = 8.0         # 学轴时，单节至少要转过这么多度才算数

# 每根手指的 (电机 ID, 关节偏移链)；偏移相对本手第 0 个关节（手腕）
#
# 拇指必须拆开（2026-11-20 第二修）：102 是拇指根（CMC），107 是 MCP+IP。
# 老版本 102 写的是 (1,2,3)、107 写的是 (2,3)，两节重叠——CMC 单独一动，102 就往前走、
# 107 不动，看着就是"大拇指整根往外展、指尖不弯"。
FINGERS: Dict[int, Tuple[int, ...]] = {
    102: (1,),           # 拇指1 ← Thumb1（CMC，拇指根）
    107: (2, 3),         # 拇指2 ← Thumb2..3（MCP/IP）
    103: (4, 5, 6),      # 食指
    104: (7, 8, 9),      # 中指
    105: (10, 11, 12),   # 无名指
    106: (13, 14, 15),   # 小指
}

HEAD_RE = re.compile(r"fn\s+(\d+)\s+subpackage\s+(\d+)/(\d+),")

ApplyFn = Callable[[Dict[int, float], float, float], Any]


# ---------------------------------------------------------------- 解析

def parse_fields(text: str) -> List[float]:
    """把 ``...subpackage 1/2,1.2,3,4,`` 后半段的数字全取出来（坏 token 跳过）。"""
    out: List[float] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        try:
            out.append(float(token))
        except ValueError:
            continue
    return out


class Assembler:
    """按 ``fn`` 把 1/2、2/2 两个包拼成一整帧；旧的 fn 直接丢。

    MotionVenus 会持续发新帧，拼不上的半帧留着只会让延迟越攒越大，所以这里
    只保留最近 8 个 fn。
    """

    KEEP = 8

    def __init__(self, bones: int = BONES, per: int = PER) -> None:
        self.bones = bones
        self.per = per
        self.pending: Dict[int, Tuple[int, List[float]]] = {}

    def reset(self) -> None:
        self.pending.clear()

    def feed(self, text: str) -> Optional[List[float]]:
        m = HEAD_RE.search(text)
        if not m:
            return None
        fn, idx, total = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if total < 1 or idx < 1 or idx > total:
            return None
        nums = parse_fields(text[m.end():])

        prev = self.pending.get(fn)
        values = (prev[1] if prev and idx > 1 and prev[0] == idx - 1 else []) + nums
        if idx < total:
            self.pending[fn] = (idx, values)
            for old in sorted(self.pending)[:-self.KEEP]:
                self.pending.pop(old, None)
            return None

        self.pending.pop(fn, None)
        for old in [k for k in self.pending if k < fn]:
            self.pending.pop(old, None)
        if len(values) < self.bones * self.per:
            return None
        return values


def flex(fields: Iterable[float]) -> float:
    """单关节弯折角（度）：欧拉角里最大的两个横向分量（abs 老算法专用）。

    这个量不是旋转不变的——整只手一起转它也会变，所以只留给 frame="abs" 做对照。
    """
    f = list(fields)
    if len(f) < PER:
        return 0.0
    return max(abs(f[4]), abs(f[5]))


# ------------------------------------------------------- 旋转（frame=rel 用）

IDENT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def _rz(t: float):
    c, s = math.cos(t), math.sin(t)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _rx(t: float):
    c, s = math.cos(t), math.sin(t)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _ry(t: float):
    c, s = math.cos(t), math.sin(t)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def rotation(fields: Iterable[float], extrinsic: bool = False):
    """一节的 ZXY 欧拉角（度）→ 3×3 旋转矩阵。

    intrinsic（默认）按 Z→X→Y 依次绕**动轴**转，即 R = Rz·Rx·Ry；extrinsic 就是
    反过来 R = Ry·Rx·Rz。帧里只有一个 "ZXY" 字样，厂家没说是哪种，两种都留着，
    真机上看哪个把手臂转动消得干净（tools/glove_probe.py 会同时算三个方案）。
    """
    f = list(fields)
    if len(f) < PER:
        return IDENT
    ex, ey, ez = (math.radians(f[3]), math.radians(f[4]), math.radians(f[5]))
    if extrinsic:
        return _mat_mul(_mat_mul(_ry(ey), _rx(ex)), _rz(ez))
    return _mat_mul(_mat_mul(_rz(ez), _rx(ex)), _ry(ey))


def twist_deg(rel) -> float:
    """相对旋转 → 转角幅值（度）。恒等矩阵给 0。

    只走 trace：θ = acos((tr-1)/2)。不用 r32-r23 那一套算转轴，因为 θ=180° 时
    那个差向量正好是 0，会把"关节转了半圈"读成"没动"。
    """
    cos = (rel[0][0] + rel[1][1] + rel[2][2] - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, cos))))


def relative_rotation(fields: Iterable[float], wrist: Optional[Iterable[float]] = None,
                      extrinsic: bool = False):
    """这一节相对手腕的旋转矩阵（R0ᵀ·Rj）；wrist 为空就是相对参考系。"""
    rel = rotation(fields, extrinsic)
    if wrist is not None:
        rel = _mat_mul(_transpose(rotation(wrist, extrinsic)), rel)
    return rel


def rotvec(rel) -> Tuple[float, float, float]:
    """旋转矩阵 → 旋转向量（度）。方向和大小都在里面，转角 θ 只是它的长度。

    只取长度（上一版的做法）等于把所有轴都当成"弯":  拇指的 CMC 外展、小臂带动皮肤
    拧骨头，全能凑出一大坨 θ——于是"收拇指"变成了"拇指往外展"。
    """
    d0 = rel[2][1] - rel[1][2]
    d1 = rel[0][2] - rel[2][0]
    d2 = rel[1][0] - rel[0][1]
    n = math.sqrt(d0 * d0 + d1 * d1 + d2 * d2)
    if n < 1e-9:
        return (0.0, 0.0, 0.0)
    cos = (rel[0][0] + rel[1][1] + rel[2][2] - 1.0) / 2.0
    theta = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
    k = theta / n
    return (d0 * k, d1 * k, d2 * k)


def joint_angle(fields: Iterable[float], wrist: Optional[Iterable[float]] = None,
                extrinsic: bool = False) -> float:
    """单关节相对手腕的偏转角大小（度，无符号）。没轴的时候用它兜底。"""
    return twist_deg(relative_rotation(fields, wrist, extrinsic))


def joint_flex(fields: Iterable[float], wrist: Optional[Iterable[float]],
               axis: Tuple[float, float, float], extrinsic: bool = False) -> float:
    """这一节沿屈曲轴的有符号转角（度）：外展、自转都被投影掉，方向对了才是正。"""
    w = rotvec(relative_rotation(fields, wrist, extrinsic))
    return w[0] * axis[0] + w[1] * axis[1] + w[2] * axis[2]


def chain_angles(groups: List[List[float]], side: str = "right",
                 frame: str = DEFAULT_FRAME,
                 axes: Optional[Dict[int, Tuple[float, float, float]]] = None
                 ) -> Dict[int, float]:
    """按关节链求和，得到六根手指的原始弯折量（度，尚未做零点和增益）。

    frame="rel"（默认）：每节先减掉手腕的旋转（R0ᵀ·Rj），整只手的公共旋转——
    包括手臂转一圈——不进入结果；frame="abs"：1.1.x 的老算法，直接取欧拉角。

    axes 给了就用"沿每节屈曲轴投影"（有符号），没给就退回无符号的转角大小。
    """
    if frame not in FRAMES:
        raise ValueError(f"frame 只能是 {FRAME_HINT}，收到 {frame!r}")
    base = 0 if side != "left" else 16
    out: Dict[int, float] = {}
    if frame == "abs":
        for motor, chain in FINGERS.items():
            out[motor] = sum(flex(groups[base + off]) for off in chain
                             if base + off < len(groups))
        return out

    extrinsic = frame == "rel-ext"
    wrist = groups[base] if base < len(groups) else None
    for motor, chain in FINGERS.items():
        total = 0.0
        for off in chain:
            index = base + off
            if index >= len(groups):
                continue
            axis = (axes or {}).get(off)
            if axis is None:
                total += joint_angle(groups[index], wrist, extrinsic)
            else:
                total += joint_flex(groups[index], wrist, axis, extrinsic)
        out[motor] = total
    return out


def _average_axis(samples: List[Tuple[float, float, float]]
                  ) -> Optional[Tuple[float, float, float]]:
    """把一段动作里每节的旋转向量平均成一个单位轴。

    只保留"转得动"的样本（峰值一半以上），逐条归一化再平均——否则慢的那几帧会把
    方向稀释掉。样本太少或根本没转起来就返回 None，那一节继续用无符号兜底。
    """
    if len(samples) < 5:
        return None
    mags = [math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) for w in samples]
    peak = max(mags)
    if peak < AXIS_MIN_DEG:
        return None
    ax = ay = az = 0.0
    for w, m in zip(samples, mags):
        if m < peak * 0.5 or m < 1e-9:
            continue
        ax += w[0] / m
        ay += w[1] / m
        az += w[2] / m
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-6:
        return None
    return (ax / n, ay / n, az / n)


def finger_table() -> List[Dict[str, Any]]:
    """六指 ↔ 手套关节链的对照表（页面显示用）。"""
    return [{"motor_id": m.motor_id, "key": m.key, "name": m.name,
             "max": m.pos_max, "current_max": m.current_max,
             "chain": list(FINGERS[m.motor_id])}
            for m in spec.MOTORS]


# ---------------------------------------------------------------- 接收器

class GloveReceiver:
    """后台收 UDP，把最新一帧换成六指角度，按速率上限交给 ``apply``。

    ``apply`` 的签名是 ``apply(motors: Dict[int, float], speed, torque)``；
    服务里传 ``App.apply_glove``，测试里传一个只记调用的函数。
    """

    def __init__(self, apply: Optional[ApplyFn] = None,
                 limits: Optional[Dict[int, float]] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.apply_fn = apply
        self.limits: Dict[int, float] = dict(
            limits or {m.motor_id: m.pos_max for m in spec.MOTORS})
        self.clock = clock

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._sock: Optional[socket.socket] = None
        self.thread: Optional[threading.Thread] = None
        self.running = False

        # 运行参数
        self.mode = "glove"          # glove = 跟手；code = 代码模式（已隔断）
        self.udp = DEFAULT_UDP
        self.bind = DEFAULT_BIND
        self.side = "right"
        self.gain = 1.0
        self.rate = DEFAULT_RATE
        self.speed = DEFAULT_SPEED
        self.torque = DEFAULT_TORQUE
        self.full_scale = FULL_SCALE_DEG
        self.window_s = BASELINE_SECONDS
        self.frame = DEFAULT_FRAME
        self.deadband: Dict[int, float] = {}
        self.axes: Dict[int, Tuple[float, float, float]] = {}
        self.axes_path: Path = AXES_FILE
        self.teaching = False
        self._teach: Dict[int, List[Tuple[float, float, float]]] = {}
        self.dry_run = False

        # 计数与状态
        self.frames = 0
        self.packets = 0
        self.applied = 0
        self.errors = 0
        self.last_error = ""
        self.start_error = ""
        self.last_frame_at: Optional[float] = None
        self.last_post_at: Optional[float] = None
        self.zeroed_at: Optional[float] = None
        self.last_raw: Dict[int, float] = {}
        self.last_motors: Dict[int, float] = {}
        self.started_at: Optional[float] = None
        self.fps = 0.0

        self.assembler = Assembler()
        self._hist: Dict[int, collections.deque] = {
            m: collections.deque(maxlen=600) for m in FINGERS}
        self._frame_times: collections.deque = collections.deque(maxlen=120)
        self._motor_hist: collections.deque = collections.deque(maxlen=60)
        self._raw_hist: collections.deque = collections.deque(maxlen=3)
        # 基线：floor = 摊平零点。只在"持续低于"时下移，避免单帧下冲把整只手带偏
        self._floor: Dict[int, float] = {}
        self._below_since: Dict[int, Optional[float]] = {}
        self._near_since: Dict[int, Optional[float]] = {}
        self._floor_at: Dict[int, float] = {}
        self.spikes = 0
        self.last_spike_at: Optional[float] = None

    # -- 生命周期 ---------------------------------------------------------
    def start(self, udp: int = DEFAULT_UDP, bind: str = DEFAULT_BIND, side: str = "right",
              gain: float = 1.0, rate: float = DEFAULT_RATE, speed: float = DEFAULT_SPEED,
              torque: float = DEFAULT_TORQUE, full_scale: float = FULL_SCALE_DEG,
              frame: str = DEFAULT_FRAME, deadband: Optional[Dict[Any, float]] = None,
              dry_run: bool = False) -> Dict[str, Any]:
        if side not in ("right", "left"):
            raise ValueError(f"side 只能是 right / left，收到 {side!r}")
        frame = str(frame or DEFAULT_FRAME).strip().lower()
        if frame not in FRAMES:
            raise ValueError(f"frame 只能是 {FRAME_HINT}，收到 {frame!r}")
        udp = int(udp)
        gain, rate = float(gain), float(rate)
        speed, torque, full_scale = float(speed), float(torque), float(full_scale)
        if not 1 <= udp <= 65535:
            raise ValueError(f"UDP 端口 {udp} 不在 1~65535")
        if not 0.1 <= gain <= 3.0:
            raise ValueError(f"增益 {gain} 不在 0.1~3.0")
        if not 1.0 <= rate <= 100.0:
            raise ValueError(f"速率 {rate} Hz 不在 1~100")
        if not 1.0 <= full_scale <= 400.0:
            raise ValueError(f"满量程 {full_scale}° 不在 1~400")
        if not 0.0 <= torque <= 3.5:
            raise ValueError(f"电流上限 {torque}A 不在 0~3.5")
        if not 1.0 <= speed <= spec.SPEED_MAX:
            raise ValueError(f"速度 {speed} 不在 1~{spec.SPEED_MAX:.0f}")
        band = self._clean_deadband(deadband)      # 先校验，别建了 socket 再抛

        with self._lock:
            if self.thread is not None and self.thread.is_alive():
                return {"ok": True, "already": True, **self.state()}
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Windows 上 SO_REUSEADDR 允许多个进程绑同一个 UDP 端口（后绑的把包抢走），
            # 那就永远发现不了「老桥接还占着 5001」。这里优先用独占绑定，冲突就报错。
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                except OSError:
                    pass
            else:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                sock.bind((bind, udp))
            except OSError as exc:
                sock.close()
                error = f"UDP {bind}:{udp} 绑定失败：{exc}"
                self.start_error = error
                self.last_error = error
                return {
                    "ok": False,
                    "error": error,
                    "hint": ("端口被别的程序占着：最可能是还在跑的 tools/glove_bridge.py，"
                             "先关掉它；或者换一个端口，并把 MotionVenus 的转发端口改成同一个。"),
                }
            sock.setblocking(False)

            self.udp, self.bind, self.side = udp, bind, side
            self.start_error = ""
            self.gain, self.rate = gain, rate
            self.speed, self.torque = speed, torque
            self.full_scale = full_scale
            self.frame = frame
            self.axes = self._load_axes()
            self.deadband = band
            self.dry_run = bool(dry_run)
            self._sock = sock
            self._stop.clear()
            self.assembler.reset()
            self.running = True
            self.started_at = self.clock()
            self.thread = threading.Thread(target=self._loop, name="glove-udp", daemon=True)
            self.thread.start()
            return {"ok": True, "already": False, **self.state()}

    def stop(self, timeout: float = 1.5) -> Dict[str, Any]:
        with self._lock:
            self._stop.set()
            sock, self._sock = self._sock, None
            thread, self.thread = self.thread, None
            self.running = False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=timeout)
        return {"ok": True, **self.state()}

    # -- 收包 -------------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                break
            try:
                ready, _, _ = select.select([sock], [], [], 0.05)
            except (OSError, ValueError):
                break
            if not ready:
                continue
            latest: Optional[List[float]] = None
            while True:
                try:
                    data, _ = sock.recvfrom(65535)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    latest = None
                    break
                self.packets += 1
                frame = self.assembler.feed(data.decode("ascii", "ignore"))
                if frame is not None:
                    latest = frame
            if latest is not None:
                try:
                    self.process_frame(latest)
                except Exception as exc:                 # noqa: BLE001
                    self.errors += 1
                    self.last_error = f"{type(exc).__name__}: {exc}"

    # -- 换算 -------------------------------------------------------------
    def feed_text(self, text: str, now: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """喂一包文本（测试用）：凑成整帧就处理，并返回处理结果。"""
        self.packets += 1
        frame = self.assembler.feed(text)
        if frame is None:
            return None
        return self.process_frame(frame, now=now)

    def process_frame(self, values: List[float], now: Optional[float] = None) -> Dict[str, Any]:
        now = self.clock() if now is None else float(now)
        self.frames += 1
        self.last_frame_at = now
        self._frame_times.append(now)
        self._refresh_fps()

        groups = [values[i * PER:(i + 1) * PER] for i in range(BONES)]
        if self.teaching:
            self._sample_teach(groups)
        raw = chain_angles(groups, self.side, self.frame, self.axes)
        for motor, value in raw.items():
            self._push(motor, value, now)

        motors: Dict[int, float] = {}
        for motor, value in raw.items():
            zero = self._baseline(motor)
            span = value - zero
            band = self.deadband.get(motor, 0.0)
            if band:
                span = 0.0 if span <= band else span - band
            scaled = span * (self.limits[motor] / self.full_scale) * self.gain
            motors[motor] = round(max(0.0, min(self.limits[motor], scaled)), 1)

        # 毛刺 = 跳上去又立刻弹回来。正常张开→握拳是单向的，不会被算进来。
        hist = list(self._raw_hist)
        if len(hist) >= 2:
            before, peak = hist[-2], hist[-1]
            if any(abs(peak.get(m, 0.0) - v) > SPIKE_DEG
                   and abs(peak.get(m, 0.0) - before.get(m, 0.0)) > SPIKE_DEG
                   and abs(v - before.get(m, 0.0)) < SPIKE_DEG / 3
                   for m, v in raw.items()):
                self.spikes += 1
                self.last_spike_at = now
        self._raw_hist.append(dict(raw))
        self.last_raw = dict(raw)
        self.last_motors = dict(motors)
        self._motor_hist.append(dict(motors))

        post: Optional[Dict[str, Any]] = None
        if self.mode == "glove" and not self.dry_run and self.apply_fn is not None:
            gap = 1.0 / max(self.rate, 0.1)
            if self.last_post_at is None or now - self.last_post_at >= gap:
                self.last_post_at = now
                try:
                    result = self.apply_fn(dict(motors), self.speed, self.torque)
                    if isinstance(result, dict) and result.get("ok") is False:
                        # 安全闸门拦下来了：记下来给页面看，但别让线程死掉
                        self.errors += 1
                        self.last_error = str(result.get("error") or "下发被安全检查拒绝")
                        post = {"ok": False, "error": self.last_error}
                    else:
                        self.applied += 1
                        post = {"ok": True}
                except Exception as exc:                 # noqa: BLE001
                    self.errors += 1
                    self.last_error = f"{type(exc).__name__}: {exc}"
                    post = {"ok": False, "error": self.last_error}
        return {"raw": self.last_raw, "motors": self.last_motors, "post": post}

    def snapshot(self, frames: int = SNAPSHOT_FRAMES) -> Dict[int, float]:
        """最近几帧里每指角度的中位数——示教存点位用它，比抓单帧稳。

        单帧会带着手套自己的抖动（实测 ±2~3°），抓一帧等于把这次的抖存进点位里；
        中位数还能挡掉偶发的掉帧/跳变。
        """
        n = max(1, int(frames))
        recent = list(self._motor_hist)[-n:]
        out: Dict[int, float] = {}
        for motor in FINGERS:
            vals = sorted(frame[motor] for frame in recent if motor in frame)
            if not vals:
                continue
            mid = len(vals) // 2
            out[motor] = (vals[mid] if len(vals) % 2
                          else (vals[mid - 1] + vals[mid]) / 2.0)
        return out

    def _push(self, motor: int, value: float, now: float) -> None:
        """更新这一指的摊平零点（粘性地板）。

        实时任务里不能再用"最近 10 秒最小值"：任何一次下冲都会立刻成为新零点，整整
        10 秒里整只手都偏高——抓握中表现为"手突然合了一下"，还会来回漂。
        这里的规则是：

        * 往下：要连续低于地板 DIP_SECONDS 才承认（单帧毛刺不算）；
        * 往上：要贴着地板 RISE_AFTER 秒，之后按 RISE_RATE 慢慢追（只管温度漂移，
          不会把抓握中的值一点点吃掉）。
        """
        hist = self._hist[motor]
        hist.append((now, value))
        cutoff = now - self.window_s
        while hist and hist[0][0] < cutoff:
            hist.popleft()

        floor = self._floor.get(motor)
        if floor is None:
            self._floor[motor] = value
            self._below_since[motor] = None
            self._near_since[motor] = None
            self._floor_at[motor] = now
            return

        dt = max(0.0, now - self._floor_at.get(motor, now))
        self._floor_at[motor] = now

        if value < floor - FLOOR_EPS:
            since = self._below_since.get(motor)
            if since is None:
                self._below_since[motor] = now
            elif now - since >= DIP_SECONDS:
                floor = value                       # 持续下探：认了这个新摊平
                self._below_since[motor] = None
                self._near_since[motor] = None
        else:
            self._below_since[motor] = None

        if value <= floor + FLOOR_EPS:
            if self._near_since.get(motor) is None:
                self._near_since[motor] = now
            elif now - float(self._near_since[motor]) >= RISE_AFTER:
                floor += min(RISE_RATE * dt, max(0.0, value - floor))
        else:
            self._near_since[motor] = None

        self._floor[motor] = floor

    def _baseline(self, motor: int) -> float:
        return self._floor.get(motor, 0.0)

    def _refresh_fps(self) -> None:
        times = self._frame_times
        if len(times) < 2:
            self.fps = 0.0
            return
        span = times[-1] - times[0]
        # 一两个包挤在同一毫秒里时算出来会是几百上千 Hz，不好看也不真实：窗口太短就不更新
        if span < 0.2:
            return
        self.fps = min(120.0, (len(times) - 1) / span)

    # -- 控制 -------------------------------------------------------------
    @staticmethod
    def _clean_deadband(deadband: Optional[Dict[Any, float]]) -> Dict[int, float]:
        """把 {电机号: 死区度数} 洗一遍；0 的项不存，省得每帧都查。"""
        out: Dict[int, float] = {}
        for key, value in dict(deadband or {}).items():
            try:
                motor, deg = int(key), float(value)
            except (TypeError, ValueError):
                raise ValueError(f"死区参数不对：{key!r}: {value!r}")
            if motor not in FINGERS:
                raise ValueError(f"死区电机号 {motor} 不在 {sorted(FINGERS)}")
            if not 0.0 <= deg <= DEADBAND_MAX:
                raise ValueError(f"电机 {motor} 的死区 {deg}° 不在 0~{DEADBAND_MAX:.0f}")
            if deg:
                out[motor] = deg
        return out

    def set_frame(self, frame: str) -> Dict[str, Any]:
        """换换算方式。换了基线就得重来，所以顺手把零点窗口清空。"""
        name = str(frame or "").strip().lower()
        if name not in FRAMES:
            raise ValueError(f"frame 只能是 {FRAME_HINT}，收到 {frame!r}")
        with self._lock:
            self.frame = name
            for hist in self._hist.values():
                hist.clear()
            self.zeroed_at = self.clock()
        return {"ok": True, "frame": name,
                "detail": "已切换换算方式并清空基线；先摊平 1~2 秒"}

    # -- 屈曲轴：教一次，之后"外展 / 拧骨头"就不算弯手指了 -----------------
    def teach_start(self) -> Dict[str, Any]:
        with self._lock:
            self._teach = {}
            self.teaching = True
        return {"ok": True, "teaching": True,
                "detail": ("现在做一次『只屈曲』：四指慢慢握到底、拇指往掌心弯（不要往外撇），"
                           "约 2~3 秒后调 stop")}

    def teach_stop(self) -> Dict[str, Any]:
        with self._lock:
            self.teaching = False
            samples, self._teach = self._teach, {}
        axes: Dict[int, Tuple[float, float, float]] = {}
        skipped: List[int] = []
        for off in sorted(samples):
            axis = _average_axis(samples[off])
            if axis is None:
                skipped.append(off)
            else:
                axes[off] = axis
        with self._lock:
            self.axes = axes
        saved = self._save_axes(axes)
        return {"ok": bool(axes),
                "axes": {str(k): [round(c, 4) for c in a] for k, a in axes.items()},
                "skipped": skipped, "saved": saved,
                "detail": (f"学到 {len(axes)} 节的屈曲轴"
                           + (f"；{skipped} 这几节没转起来，仍用无符号兜底" if skipped else ""))}

    def clear_axes(self) -> Dict[str, Any]:
        with self._lock:
            self.axes = {}
        self._save_axes({})
        return {"ok": True, "axes": 0}

    def _sample_teach(self, groups: List[List[float]]) -> None:
        base = 0 if self.side != "left" else 16
        if base >= len(groups):
            return
        extrinsic = self.frame == "rel-ext"
        wrist = groups[base]
        for off in range(1, 16):
            index = base + off
            if index >= len(groups):
                break
            w = rotvec(relative_rotation(groups[index], wrist, extrinsic))
            hist = self._teach.setdefault(off, [])
            hist.append(w)
            if len(hist) > 600:
                hist.pop(0)

    def _load_axes(self) -> Dict[int, Tuple[float, float, float]]:
        try:
            data = json.loads(self.axes_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        out: Dict[int, Tuple[float, float, float]] = {}
        for key, vec in dict(data or {}).items():
            try:
                off, v = int(key), [float(x) for x in vec]
            except (TypeError, ValueError):
                continue
            if len(v) != 3:
                continue
            n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
            if n > 1e-9:
                out[off] = (v[0] / n, v[1] / n, v[2] / n)
        return out

    def _save_axes(self, axes: Dict[int, Tuple[float, float, float]]) -> str:
        try:
            self.axes_path.parent.mkdir(parents=True, exist_ok=True)
            self.axes_path.write_text(
                json.dumps({str(k): [round(c, 5) for c in v] for k, v in axes.items()},
                           indent=1), encoding="utf-8")
            return str(self.axes_path)
        except OSError as exc:
            return f"没存下：{exc}"

    def set_deadband(self, deadband: Optional[Dict[Any, float]]) -> Dict[str, Any]:
        with self._lock:
            self.deadband = self._clean_deadband(deadband)
        return {"ok": True, "deadband": {str(k): v for k, v in self.deadband.items()}}

    def set_mode(self, mode: str) -> Dict[str, Any]:
        name = str(mode or "").strip().lower()
        if name not in ("glove", "code"):
            raise ValueError(f"mode 只能是 glove / code，收到 {mode!r}")
        with self._lock:
            self.mode = name
        return {"ok": True, "mode": name}

    def toggle_mode(self) -> Dict[str, Any]:
        return self.set_mode("code" if self.mode == "glove" else "glove")

    def set_gain(self, gain: float) -> Dict[str, Any]:
        value = float(gain)
        if not 0.1 <= value <= 3.0:
            raise ValueError(f"增益 {value} 不在 0.1~3.0")
        with self._lock:
            self.gain = value
        return {"ok": True, "gain": value}

    def zero(self) -> Dict[str, Any]:
        """把「现在这只手的姿态」当成摊平基线（下一帧重新播种地板）。"""
        with self._lock:
            for hist in self._hist.values():
                hist.clear()
            self._floor.clear()
            self._below_since.clear()
            self._near_since.clear()
            self._floor_at.clear()
            self.zeroed_at = self.clock()
        return {"ok": True, "zeroed_at": self.zeroed_at,
                "detail": "已把当前姿态记成零点；下一帧从这个姿态重新起算"}

    def baseline_ready(self) -> bool:
        return all(len(h) >= 3 for h in self._hist.values())

    # -- 状态 -------------------------------------------------------------
    def state(self) -> Dict[str, Any]:
        now = self.clock()
        age = None if self.last_frame_at is None else max(0.0, now - self.last_frame_at)
        live = bool(self.running) and age is not None and age <= LIVE_SECONDS
        post_age = (None if self.last_post_at is None
                    else round(max(0.0, now - self.last_post_at), 2))
        return {
            "running": self.running,
            "mode": self.mode,
            "udp": self.udp,
            "bind": self.bind,
            "side": self.side,
            "gain": self.gain,
            "rate": self.rate,
            "speed": self.speed,
            "torque": self.torque,
            "full_scale": self.full_scale,
            "frame": self.frame,
            "axes": len(self.axes),
            "teaching": self.teaching,
            "deadband": {str(k): v for k, v in self.deadband.items()},
            "dry_run": self.dry_run,
            "frames": self.frames,
            "packets": self.packets,
            "applied": self.applied,
            "errors": self.errors,
            "fps": round(self.fps, 1),
            "live": live,
            "spikes": self.spikes,
            "last_spike_age_s": (None if self.last_spike_at is None
                                else round(max(0.0, now - self.last_spike_at), 1)),
            "saturated": [str(m) for m, deg in self.last_motors.items()
                          if deg >= self.limits[m] - 0.05],
            "frame_age_s": None if age is None else round(age, 2),
            "last_post_age_s": post_age,
            "baseline_ready": self.baseline_ready(),
            "baseline_window_s": self.window_s,
            "zeroed_at": self.zeroed_at,
            "raw": {str(k): round(v, 1) for k, v in self.last_raw.items()},
            "motors": {str(k): v for k, v in self.last_motors.items()},
            "uptime_s": (None if self.started_at is None
                         else round(max(0.0, now - self.started_at), 1)),
            "last_error": self.last_error,
            "start_error": self.start_error,
            "hint": self._hint(live),
        }

    def summary(self) -> Dict[str, Any]:
        """给 ``/health`` 用的短状态。"""
        s = self.state()
        return {k: s[k] for k in ("running", "mode", "live", "fps", "frames",
                                  "udp", "side", "gain", "last_error")}

    def _hint(self, live: bool) -> str:
        if not self.running:
            if self.start_error:
                return (f"{self.start_error}；先关掉占着 {self.udp} 的程序（多半是 "
                        "tools/glove_bridge.py），或者把两边端口改成同一个")
            return ("先点「启动监听」；MotionVenus 要把数据转发到 127.0.0.1，"
                    f"端口 {self.udp}")
        if self.mode == "code":
            return "代码模式：手套数据只收不发，脚本/滑块可以独占手；点「手套操控」恢复跟手"
        if not live:
            return (f"UDP {self.bind}:{self.udp} 已在监听，但还没收到帧："
                    "看 MotionVenus 的转发目标/端口是否对上，或换手（左/右）")
        if not self.baseline_ready():
            return "刚开始收帧：先把手摊平 1~2 秒，自动零点会追上"
        return ""


# ---------------------------------------------------------------- 独立跑法

def _post_pose(http: str, motors: Dict[int, float], speed: float, torque: float,
               timeout: float = 0.8) -> None:
    body = json.dumps({
        "motors": {str(k): {"angle": v, "speed": speed, "torque": torque}
                   for k, v in motors.items()},
        "reason": "glove",
    }).encode()
    req = urllib.request.Request(http + "/pose", data=body,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        r.read()


def _start_control(receiver: GloveReceiver, port: int) -> ThreadingHTTPServer:
    """8767 控制口（只是为了兼容老脚本 / 老 .bat）。"""

    class Ctrl(BaseHTTPRequestHandler):
        def _json(self, obj: Any, code: int = 200) -> None:
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:                        # noqa: N802
            if self.path.startswith("/toggle"):
                return self._json(receiver.toggle_mode())
            if self.path.startswith("/state"):
                return self._json(receiver.state())
            if self.path.startswith("/mode"):
                return self._json({"ok": True, "mode": receiver.mode})
            return self._json({"ok": False, "error": "unknown path"}, 404)

        def do_POST(self) -> None:                       # noqa: N802
            length = int(self.headers.get("Content-Length") or 0)
            raw = self.rfile.read(length) if length else b"{}"
            try:
                data = json.loads(raw or b"{}")
            except json.JSONDecodeError:
                data = {}
            if self.path.startswith("/toggle"):
                return self._json(receiver.toggle_mode())
            try:
                return self._json(receiver.set_mode(str(data.get("mode", ""))))
            except ValueError as exc:
                return self._json({"ok": False, "error": str(exc)}, 400)

        def log_message(self, *args: Any) -> None:        # 静音
            pass

    httpd = ThreadingHTTPServer(("127.0.0.1", port), Ctrl)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="MotionVenus 手套 → handlink /pose（独立跑法）")
    ap.add_argument("--udp", type=int, default=DEFAULT_UDP)
    ap.add_argument("--bind", default=DEFAULT_BIND)
    ap.add_argument("--http", default="http://127.0.0.1:8766",
                    help="handlink 服务地址；留空字符串则只收不发（--dry-run）")
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--rate", type=float, default=DEFAULT_RATE)
    ap.add_argument("--gain", type=float, default=1.0)
    ap.add_argument("--frame", choices=list(FRAMES), default=DEFAULT_FRAME,
                    help="rel=减掉手腕（默认，手臂怎么转都不串）；abs=老算法做对照")
    ap.add_argument("--deadband", default="", metavar="102=4,107=6",
                    help="给指定电机加死区（度）：拇指残留抖动就 102=4,107=6")
    ap.add_argument("--ctrl-port", type=int, default=8767,
                    help="兼容老 .bat 的 /mode /toggle 控制口，0=不开")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--verbose", action="store_true", help="每 50 帧打一行原始值")
    args = ap.parse_args(argv)

    apply_fn: Optional[ApplyFn] = None
    if args.http and not args.dry_run:
        http_url = args.http

        def apply_fn(motors: Dict[int, float], speed: float, torque: float) -> None:
            _post_pose(http_url, motors, speed, torque)

    band: Dict[int, float] = {}
    for item in str(args.deadband or "").replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        key, _, deg = item.partition("=")
        band[key.strip()] = float(deg or 0)      # type: ignore[index]

    receiver = GloveReceiver(apply=apply_fn)
    result = receiver.start(udp=args.udp, bind=args.bind, side=args.side,
                            gain=args.gain, rate=args.rate, frame=args.frame,
                            deadband=band, dry_run=args.dry_run)
    if not result.get("ok"):
        print(f"[glove] {result.get('error')}", flush=True)
        print(f"[glove] {result.get('hint', '')}", flush=True)
        return 2

    print(f"[glove] 收 UDP {args.bind}:{args.udp} · side={args.side} · "
          f"frame={args.frame} · gain={args.gain} · ≤{args.rate:g} Hz → "
          f"{args.http or '（只收不发）'}", flush=True)
    if args.ctrl_port:
        _start_control(receiver, args.ctrl_port)
        print(f"[glove] 控制口 http://127.0.0.1:{args.ctrl_port}/mode"
              "  (GET 查 / POST 切 / toggle 翻转)", flush=True)
    try:
        last = 0.0
        last_frame = 0
        while True:
            time.sleep(0.5)
            now = receiver.clock()
            if args.verbose and receiver.frames - last_frame >= 50:
                last_frame = receiver.frames
                s = receiver.state()
                print(f"[glove] f{s['frames']} raw="
                      + " ".join(f"{m}:{v:.0f}" for m, v in s["raw"].items())
                      + " -> " + " ".join(f"{m}:{v:.0f}" for m, v in s["motors"].items()),
                      flush=True)
            if receiver.frames and now - last >= 5.0:
                last = now
                s = receiver.state()
                print(f"[glove] f{s['frames']} {s['fps']}Hz "
                      + " ".join(f"{m}:{v:.0f}" for m, v in s["motors"].items()),
                      flush=True)
    except KeyboardInterrupt:
        receiver.stop()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
