"""进程内虚拟 Dobot Nova 5 控制器：29999 文本控制口 + 30004 二进制反馈口。

用法（不需要真机、不开新进程，两个端口都 bind 在 127.0.0.1 的临时端口上）::

    from handlink import armsim
    sim = armsim.VirtualDobot().start()
    arm = sim.arm()                      # handlink.arm.DobotArm(ip, sim.port) 原样连上
    arm.enable()
    arm.move_j([400, 0, 300, 180, 0, 0], wait=False)
    sim.state()["current_id"]

它存在的理由：真机（Nova 5，192.168.5.1）上量到的那些行为——ACK 延迟分布、
算法队列、ServoJ 的 t 低通、停发保持、29999 单客户端抢占、30004 每 8 ms 一帧——
以前只能上真机才能验证，现在可以放进单元测试里。

## 哪些数字来自实测，哪些是固件习惯值

**实测（docs/遥操作跟随-调研与方案.md 第 1 节、docs/现状.md 8.3 节）**

=============================  ==========================================
29999 GetPose() 往返            中位 3.9 ms / p90 5.0 / max 5.4（连续 30 次）
ServoJ ACK 往返                 中位 6.2 ms / p90 7.3 / max 12.0（60 帧 @30 ms）
t 默认值                        0.1 s；手册 p.89-91，实测建议首测用 0.03
SpeedFactor 对 ServoJ           **无效**（手册 p.89「该指令不受全局速率影响」）
未使能时发 ServoJ               -1（状态不允许）
未知指令                        -10000
停发 ServoJ                     保持最后目标、RobotMode=5、ErrorStatus=0，10 s 不跑位
30004 帧率                      125 Hz（8 ms），单帧 1440 字节小端
TestValue                       0x0123456789ABCDEF（第 48 字节）
关节最大速度                    100 °/s（六轴相同）
关节限位                        J1/J4/J5/J6 ±360°，J2 ±180°，J3 ±160°
=============================  ==========================================

**固件习惯值（手册没写，按「像真机」的方式拟的；不喜欢可以改常量）**

* 模式机 4/5/7/9 的迁移顺序（使能→5、有运动→7、报警→9、下使能→4）；
* 队列积压的拒绝码 -50001（movl_r 用 r=3.0 时真机回过 -50001，这里把它同时
  当作「参数/队列不合法」的通用码）；
* 关节超限拒绝码 -40000（固件习惯值，未实测）；
* ServoJ 的响应速度：t 当等效一阶低通时间常数（实测只确认「朝 t 秒前的目标走」，
  没有给传递函数）；
* 30004 里 QActual 相对 QTarget 的 1~2 帧滞后（实测 lag 存在，具体帧数没数）；
* IActual / MActual / MotorTemperatures 是占位量级，只保证「像那么回事」；
* TargetQuaternion / ActualQuaternion 的顺序按 (w,x,y,z) 写，厂家的顺序没文档。

线程模型：一个控制线程（5 ms 一拍）推进队列运动 / 伺服低通 / 模式机；每个 30004
客户端一个推帧线程（feedback_hz）；29999 每个客户端一个处理线程。
所有共享状态由一把 threading.RLock 保护——测试里的 settle() 也从这条路径推进。
"""
from __future__ import annotations

import math
import random
import socket
import struct
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

from . import arm_feed, kin
from .arm import DobotArm, parse_result_id

# ---------------------------------------------------------------- 常量

#: 实测：29999 一问一答往返延迟（毫秒），按 (p50, p90, max) 给
LATENCY_DASHBOARD_MS = (3.9, 5.0, 5.4)
#: 实测：ServoJ/ServoP 的 ACK 往返延迟（毫秒），比普通指令多一步关节限幅
LATENCY_SERVO_MS = (6.2, 7.3, 12.0)

#: 实测（手册 p.89-91）：ServoJ 的 t 默认 0.1 s，官方建议 33 Hz 下发
SERVO_DEFAULT_T = 0.1
SERVO_DEFAULT_AHEADTIME = 50.0
SERVO_DEFAULT_GAIN = 500.0
SERVO_PERIOD = 0.030

#: 实测：关节最大速度 100 °/s、手册附录 A 的关节限位（见 handlink/kin.py）
JOINT_MAX_SPEED = kin.JOINT_MAX_SPEED
JOINT_LIMITS = kin.JOINT_LIMITS

#: 上电初态（docs/现状.md 8.3：mode=4，位姿 (441.3, -199.7, 923.7)）
HOME_JOINTS: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

#: 指令返回码
ERR_STATE = -1          # 状态不允许（未使能 / 报警中）—— 实测
ERR_UNKNOWN = -10000    # 指令不存在 —— 实测
ERR_QUEUE = -50001      # 队列积压 / 参数不合法 —— 固件习惯值
ERR_LIMIT = -40000      # 关节超限 —— 固件习惯值

#: RobotMode（arm_feed.ROBOT_MODES）
MODE_DISABLED = 4
MODE_IDLE = 5
MODE_RUNNING = 7
MODE_ALARM = 9
MODE_DRAG = 6

#: 一次 MovJ/MovL 最长走多久（仿真便利值；真机没有这个上限）
MOVE_MAX_SECONDS = 1.0
#: 一条队列指令最短走多久（避免 0 长度动画把队列瞬间排空）
MOVE_MIN_SECONDS = 0.03

#: Stop() 之后多久才允许队列继续（固件习惯值：让 Stop 真的停得住）
STOP_COOLDOWN = 0.05

#: 控制线程的仿真步长（秒）
TICK_PERIOD = 0.005

#: 延迟的「睡眠余量」：先睡到还差这么多，再自旋到点。
#: Windows 上 time.sleep 的实际粒度约 1 ms，不补这一段的话 GetPose 的中位会
#: 从实测的 3.9 ms 漂到 4.6 ms —— 测延迟的测试就白写了。
SPIN_SLACK = 0.0015


def _sleep_until(target: float) -> None:
    """睡到 target（perf_counter 绝对时刻），最后一段自旋保证亚毫秒精度。"""
    while True:
        now = time.perf_counter()
        left = target - now
        if left <= 0.0:
            return
        if left > SPIN_SLACK:
            time.sleep(left - SPIN_SLACK)


# ---------------------------------------------------------------- 小工具

def _num(v: float, digits: int = 3) -> str:
    """Dobot 协议里的数字：不要科学计数法、不要多余 0。"""
    s = ("%%.%df" % digits) % float(v)
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s or "0"


def _fmt_values(values: Sequence[float], digits: int = 4) -> str:
    return ",".join(_num(v, digits) for v in values)


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def _split_top(text: str, sep: str = ",") -> List[str]:
    """按分隔符切分，但不切花括号/圆括号里的（pose={1,2,3} 要算一个）。"""
    out: List[str] = []
    depth = 0
    cur: List[str] = []
    for ch in text:
        if ch in "{(":
            depth += 1
        elif ch in "})":
            depth = max(0, depth - 1)
        if ch == sep and depth == 0:
            out.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
    if cur:
        out.append("".join(cur))
    return [x.strip() for x in out if x.strip()]


def _is_int_literal(token: Any) -> bool:
    """真机固件的参数类型检查：``r=3`` 可以，``r=3.0`` 回 -50001。

    [实测 2026-11-25] 同一条指令只有这个差别，说明固件按**字面量**判整型，
    不是按值判（3.0 的数值当然是整数）。所以这里也只看字面。
    """
    text = str(token).strip().strip("{}").split(",")[0].strip()
    return bool(text) and text.lstrip("+-").isdigit()


def _parse_values(token) -> List[float]:
    """{1,2,3} 或 1 或 [1 2 3] → 浮点列表。"""
    token = str(token).strip()
    while token and token[0] in "{[(":
        token = token[1:]
    while token and token[-1] in "}])":
        token = token[:-1]
    values: List[float] = []
    for part in token.replace(";", ",").replace(" ", ",").split(","):
        part = part.strip()
        if not part:
            continue
        values.append(float(part))
    return values


def _parse_call(line: str) -> Tuple[str, List[str], Dict[str, str]]:
    """MovL(pose={1,2,3,4,5,6},cp=0) → (MovL, [], {pose: {1,2,3,4,5,6}, cp: 0})。"""
    open_at = line.find("(")
    if open_at < 0 or not line.endswith(")"):
        raise ValueError("不是 NAME(...) 形式")
    name = line[:open_at].strip()
    if not name:
        raise ValueError("没有指令名")
    body = line[open_at + 1:-1]
    positional: List[str] = []
    kwargs: Dict[str, str] = {}
    if body.strip():
        for part in _split_top(body):
            if "=" in part:
                key, _, value = part.partition("=")
                kwargs[key.strip()] = value.strip()
            else:
                positional.append(part)
    return name, positional, kwargs


def _sample_ms(lo: float, p90: float, hi: float, rng: random.Random) -> float:
    """按实测的 (中位, p90, 最大) 抽一个延迟（毫秒）。

    对数正态在 (中位, p90) 上解出 sigma，再截断到 max——真机就是长尾分布，
    用均匀分布会显得「太假」（每次都是 3~5 ms，没有偶发的 12 ms）。
    """
    if p90 <= lo or hi <= lo:
        return max(0.2, min(lo, hi))
    sigma = math.log(p90 / lo) / 1.2815515655446004      # z(0.9)
    value = lo * math.exp(rng.gauss(0.0, sigma))
    return max(0.2, min(value, hi))


def _quat(R: Sequence[Sequence[float]]) -> List[float]:
    """旋转矩阵 → 四元数 (w, x, y, z)（顺序是固件习惯值，厂家没文档）。"""
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x = 0.25 * s, (R[2][1] - R[1][2]) / s
        y, z = (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w, x = (R[2][1] - R[1][2]) / s, 0.25 * s
        y, z = (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w, x = (R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s
        y, z = 0.25 * s, (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w, x = (R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s
        y, z = (R[1][2] + R[2][1]) / s, 0.25 * s
    return [w, x, y, z]


# ---------------------------------------------------------------- 内部数据

class _Motion:
    """队列里的一条动作指令（MovJ / MovL），关节空间线性插值到目标。"""

    __slots__ = ("rid", "name", "start", "target", "t0", "duration")

    def __init__(self, rid: int, name: str, start: Sequence[float],
                 target: Sequence[float], duration: float = 0.0) -> None:
        self.rid = rid
        self.name = name
        self.start = [float(v) for v in start]
        self.target = [float(v) for v in target]
        self.t0 = 0.0
        self.duration = float(duration)


class _ServoFrame:
    """一帧 ServoJ / ServoP 的目标。

    t 是控制器参数（手册默认 0.1 s）：控制器朝「约 t 秒前的目标」走，这里当
    等效一阶低通的时间常数用。aheadtime / gain 只记录，仿真里不起作用
    （实测没有它们的传递函数数据）。
    """

    __slots__ = ("rid", "joints", "t", "aheadtime", "gain", "at")

    def __init__(self, rid: int, joints: Sequence[float], t: float,
                 aheadtime: float, gain: float, at: float) -> None:
        self.rid = rid
        self.joints = [float(v) for v in joints]
        self.t = float(t)
        self.aheadtime = float(aheadtime)
        self.gain = float(gain)
        self.at = at


# ---------------------------------------------------------------- 主体

class VirtualDobot:
    """进程内虚拟 Nova 5 控制器。

    参数
    ----
    seed
        所有随机数（延迟、反馈抖动、丢帧）都从这个种子出，测试可复现。
    latency_scale
        实测延迟的放大倍数。2.0 就是「所有 ACK 慢一倍」，用来测超时逻辑。
    servo_queue
        "latest"（默认，等价真机实测的「每周期取最新一帧」）或 "fifo"
        （严格排队，用来复现「手停了臂还在走」的橡皮筋）。
    jitter_ms
        QActual 的附加随机延迟范围（毫秒）。(0.0, 0.0) 只保留固定滞后，
        (0.5, 1.5) 更像真机的 1~2 帧抖动。
    feedback_hz
        30004 推帧频率，实测 125 Hz（8 ms）。
    """

    def __init__(self, *, seed: int = 0, latency_scale: float = 1.0,
                 servo_queue: str = "latest", jitter_ms: tuple = (0.0, 0.0),
                 feedback_hz: float = 125.0) -> None:
        if servo_queue not in ("latest", "fifo"):
            raise ValueError("servo_queue 只能是 'latest' 或 'fifo'")
        self.seed = int(seed)
        self.latency_scale = float(latency_scale)
        self.servo_queue = servo_queue
        self.jitter_ms = (float(jitter_ms[0]), float(jitter_ms[1]))
        self.feedback_hz = float(feedback_hz)
        self._rng = random.Random(self.seed)

        self._lock = threading.RLock()
        self._stopping = threading.Event()

        # -- 监听 --------------------------------------------------------
        self._srv: Optional[socket.socket] = None
        self._fb_srv: Optional[socket.socket] = None
        self._port = 0
        self._feedback_port = 0
        self._threads: List[threading.Thread] = []
        self._running = False

        # -- 29999 控制口 ------------------------------------------------
        self._client: Optional[socket.socket] = None      # 同一时刻只有一个
        self._client_addr: Optional[tuple] = None
        self._conn_threads: List[threading.Thread] = []
        self._kicked = 0                                   # 被顶掉的客户端数
        self.commands: List[str] = []                      # 收到过的原始命令行

        # -- 运动状态 ----------------------------------------------------
        self._q_target = list(HOME_JOINTS)
        self._q_actual = list(HOME_JOINTS)
        self._history: List[Tuple[float, Tuple[float, ...]]] = []

        # -- 状态机 ------------------------------------------------------
        self._mode = MODE_DISABLED
        self._enabled = False
        self._alarm = False
        self._estop = False
        self._collision = False
        self._stop_until = 0.0
        self._speed_factor = 100.0
        self._control_mode_tcp = True
        self._error_status = 0

        # -- 算法队列 ----------------------------------------------------
        self._next_id = 0
        self._current_id = 0
        self._queue: List[_Motion] = []
        self._motion: Optional[_Motion] = None
        self._queue_capacity = 32
        #: 笛卡尔目标 → 关节角的小缓存。kin.ik 是 150~250 ms 的数值解（文档实测
        #: 243 ms/次），同一批测试反复发同一个点位时不缓存就太亏了。
        self._ik_cache: Dict[Tuple[float, ...], Optional[List[float]]] = {}

        # -- 伺服 --------------------------------------------------------
        self._servo: Optional[_ServoFrame] = None
        self._servo_fifo: List[_ServoFrame] = []
        self._servo_frames = 0
        self._servo_active = False
        self._servo_clock = 0.0        # 攒够 SERVO_PERIOD 才推进一拍控制周期

        # -- 故障注入 ----------------------------------------------------
        self._drop_ratio = 0.0
        self._drop_seeded: Optional[int] = None
        self._dropped = 0

        # -- 统计 --------------------------------------------------------
        self._t0 = time.perf_counter()
        self._last_tick = self._t0
        self._last_frame: Optional[Dict[str, object]] = None
        self._feedback_threads = 0
        self._latencies: List[float] = []
        self.last_latency_ms = 0.0

    # ============================================================ 生命周期

    def start(self) -> "VirtualDobot":
        """启动监听线程。可重复调用；stop() 之后也可以再 start()。"""
        with self._lock:
            if self._running:
                return self
            self._stopping.clear()
            self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._srv.bind(("127.0.0.1", 0))          # 端口由系统分配
            self._srv.listen(8)
            self._port = self._srv.getsockname()[1]

            self._fb_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._fb_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._fb_srv.bind(("127.0.0.1", 0))
            self._fb_srv.listen(8)
            self._feedback_port = self._fb_srv.getsockname()[1]

            self._t0 = time.perf_counter()
            self._last_tick = self._t0
            self._running = True
            self._threads = [
                threading.Thread(target=self._accept_loop,
                                 args=(self._srv, self._on_control),
                                 name="armsim-29999", daemon=True),
                threading.Thread(target=self._accept_loop,
                                 args=(self._fb_srv, self._on_feedback),
                                 name="armsim-30004", daemon=True),
                threading.Thread(target=self._tick_loop, name="armsim-tick", daemon=True),
            ]
            for thread in self._threads:
                thread.start()
            return self

    def stop(self) -> None:
        """停掉所有监听、给所有连着的客户端发 EOF（等价连接被掐）。"""
        with self._lock:
            if not self._running and self._srv is None and self._fb_srv is None:
                return
            self._running = False
            self._stopping.set()
            for sock in (self._srv, self._fb_srv):
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
            self._close_client(notify=True)
            self._srv = None
            self._fb_srv = None
            threads = list(self._threads) + list(self._conn_threads)
            self._threads = []
            self._conn_threads = []
        for thread in threads:
            if thread is not threading.current_thread():
                thread.join(timeout=1.0)

    def __enter__(self) -> "VirtualDobot":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    @property
    def port(self) -> int:
        """29999 文本控制口（系统分配的临时端口）。"""
        return self._port

    @property
    def feedback_port(self) -> int:
        """30004 二进制反馈口（系统分配的临时端口）。"""
        return self._feedback_port

    def arm(self, **kwargs) -> DobotArm:
        """便捷构造一个已连接的 handlink.arm.DobotArm（附加 servo_j/servo_p）。"""
        if not self._port:
            raise RuntimeError("先 start() 再 arm()")
        kwargs.setdefault("ip", "127.0.0.1")
        kwargs.setdefault("port", self._port)
        kwargs.setdefault("read_timeout", 2.0)
        return _VirtualArm(sim=self, **kwargs)

    # ============================================================ 监听

    def _accept_loop(self, srv: socket.socket, handler) -> None:
        while not self._stopping.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                return
            if self._stopping.is_set():
                try:
                    conn.close()
                except OSError:
                    pass
                return
            thread = threading.Thread(target=handler, args=(conn, addr), daemon=True)
            self._conn_threads.append(thread)
            thread.start()

    # ---------------------------------------------------------------- 29999

    def _on_control(self, conn: socket.socket, addr: tuple) -> None:
        """一个 29999 客户端。新连接进来先把老的踢掉——真机就是这么干的。"""
        conn.settimeout(0.2)
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        with self._lock:
            self._close_client(notify=True)     # 顶掉上一个（它会收到 EOF/重置）
            self._client = conn
            self._client_addr = addr
        buf = b""
        try:
            while not self._stopping.is_set():
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    return
                if not chunk:
                    return
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", "ignore").strip()
                    if not line:
                        continue
                    started = time.perf_counter()
                    reply, delay = self._handle_with_latency(line)
                    # 真机是「处理完再回」，所以延迟加在回复之前
                    _sleep_until(started + delay)
                    try:
                        conn.sendall((reply + "\n").encode("utf-8", "ignore"))
                    except OSError:
                        return
        finally:
            with self._lock:
                if self._client is conn:
                    self._client = None
                    self._client_addr = None
            try:
                conn.close()
            except OSError:
                pass

    def _close_client(self, notify: bool = True) -> None:
        """关掉当前 29999 客户端（内部用，调用方持锁）。"""
        conn, self._client = self._client, None
        self._client_addr = None
        if conn is None:
            return
        self._kicked += 1
        try:
            if notify:
                # 半关闭 + 关闭：对面的 send/recv 会立刻拿到 EOF/重置（Windows 10053）
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
            conn.close()
        except OSError:
            pass

    def _handle_with_latency(self, line: str) -> Tuple[str, float]:
        """跑一条指令，返回 (回复文本, 该等多久秒)。"""
        with self._lock:
            now = time.perf_counter()
            self._step(now)
            self.commands.append(line)
            name = line.split("(", 1)[0].strip().lower()
            if name.startswith("servo"):
                median, p90, high = LATENCY_SERVO_MS
            else:
                median, p90, high = LATENCY_DASHBOARD_MS
            delay_ms = _sample_ms(median, p90, high, self._rng) * max(0.0, self.latency_scale)
            self.last_latency_ms = delay_ms
            reply = self._dispatch(line, now)
            self._latencies.append(delay_ms)
            return reply, delay_ms / 1000.0

    # ============================================================ 指令分发

    def _reply_ok(self, echo: str, rid: int = 0) -> str:
        return "0,{%d},%s;" % (rid, echo)

    def _reply_err(self, code: int, echo: str) -> str:
        return "%d,{},%s;" % (code, echo)

    def _dispatch(self, line: str, now: float) -> str:
        try:
            name, args, kwargs = _parse_call(line)
        except ValueError:
            return self._reply_err(ERR_UNKNOWN, line)
        handler = getattr(self, "_cmd_" + name.lower(), None)
        if handler is None:
            # 真机：未知指令回 -10000，一眼就能和「状态不允许(-1)」分开
            return self._reply_err(ERR_UNKNOWN, line)
        try:
            return handler(line, args, kwargs, now)
        except (ValueError, IndexError, TypeError):
            # 参数解析崩了：真机回 -50001（参数类型错误），别变成异常
            return self._reply_err(ERR_QUEUE, line)

    # -- 只读 -------------------------------------------------------------

    def _cmd_getpose(self, echo, args, kwargs, now):
        if not self._control_mode_tcp:
            return "Control Mode Is Not Tcp"
        return "0,{%s},%s;" % (",".join(_num(v, 3) for v in kin.fk(self._q_actual)), echo)

    def _cmd_getangle(self, echo, args, kwargs, now):
        if not self._control_mode_tcp:
            return "Control Mode Is Not Tcp"
        return "0,{%s},%s;" % (",".join(_num(v, 4) for v in self._q_actual), echo)

    def _cmd_robotmode(self, echo, args, kwargs, now):
        return "0,{%d},%s;" % (self._mode, echo)

    def _cmd_getcurrentcommandid(self, echo, args, kwargs, now):
        return "0,{%d},%s;" % (self._current_id, echo)

    def _cmd_geterrorid(self, echo, args, kwargs, now):
        return "0,{%d},%s;" % (self._error_status, echo)

    def _cmd_getqueuedcmdcurrentindex(self, echo, args, kwargs, now):
        # 真机实测：本机固件没有这条指令，回 -10000（handlink/arm.py 里写着）
        return self._reply_err(ERR_UNKNOWN, echo)

    def _cmd_speedfactor(self, echo, args, kwargs, now):
        token = args[0] if args else kwargs.get("r", "50")
        self._speed_factor = max(1.0, min(float(_parse_values(token)[0]), 100.0))
        return self._reply_ok(echo)

    def _cmd_cp(self, echo, args, kwargs, now):
        return self._reply_ok(echo)

    def _cmd_user(self, echo, args, kwargs, now):
        return self._reply_ok(echo)

    def _cmd_tool(self, echo, args, kwargs, now):
        return self._reply_ok(echo)

    def _cmd_setpayload(self, echo, args, kwargs, now):
        return self._reply_ok(echo)

    def _cmd_inversekin(self, echo, args, kwargs, now):
        values = _parse_values(",".join(args[:6])) if args else []
        if len(values) != 6:
            return self._reply_err(ERR_QUEUE, echo)
        near = kwargs.get("jointNear")
        seed = _parse_values(near) if near else list(self._q_actual)
        joints = kin.ik(values, seed if len(seed) == 6 else list(self._q_actual))
        if joints is None:
            return self._reply_err(ERR_QUEUE, echo)   # 控制器说无解
        return "0,{%s},%s;" % (",".join(_num(v, 3) for v in joints), echo)

    # -- 状态机 -----------------------------------------------------------

    def _cmd_requestcontrol(self, echo, args, kwargs, now):
        # 手册 2.1：只有「未上电(3)」或「下使能(4)，非暂停、非松抱闸」才允许切 TCP
        if self._mode not in (3, MODE_DISABLED) or self._alarm:
            return self._reply_err(ERR_STATE, echo)
        self._control_mode_tcp = True
        return self._reply_ok(echo)

    def _cmd_poweron(self, echo, args, kwargs, now):
        if self._enabled or self._alarm:
            return self._reply_err(ERR_STATE, echo)
        self._mode = MODE_DISABLED
        return self._reply_ok(echo)

    def _cmd_enablerobot(self, echo, args, kwargs, now):
        if self._alarm or self._estop:
            return self._reply_err(ERR_STATE, echo)   # 先 ClearError
        self._enabled = True
        self._mode = MODE_IDLE
        self._error_status = 0
        return self._reply_ok(echo)

    def _cmd_disablerobot(self, echo, args, kwargs, now):
        self._enabled = False
        self._motion = None
        self._queue.clear()
        self._clear_servo()
        if not self._alarm:
            self._mode = MODE_DISABLED
        return self._reply_ok(echo)

    def _cmd_clearerror(self, echo, args, kwargs, now):
        self._alarm = False
        self._estop = False
        self._collision = False
        self._error_status = 0
        self._mode = MODE_IDLE if self._enabled else MODE_DISABLED
        return self._reply_ok(echo)

    def _cmd_stop(self, echo, args, kwargs, now):
        self._motion = None
        self._queue.clear()
        self._clear_servo()
        self._stop_until = now + STOP_COOLDOWN
        if not self._alarm:
            self._mode = MODE_IDLE if self._enabled else MODE_DISABLED
        return self._reply_ok(echo)

    def _cmd_startdrag(self, echo, args, kwargs, now):
        if not self._enabled:
            return self._reply_err(ERR_STATE, echo)
        self._mode = MODE_DRAG
        return self._reply_ok(echo)

    def _cmd_stopdrag(self, echo, args, kwargs, now):
        if self._mode == MODE_DRAG:
            self._mode = MODE_IDLE
        return self._reply_ok(echo)

    def _cmd_sync(self, echo, args, kwargs, now):
        # 手册 V4.6.0 全文没有 Sync()（docs/遥操作跟随-调研与方案.md 1.1）
        return self._reply_err(ERR_UNKNOWN, echo)

    # -- 队列运动 ---------------------------------------------------------

    def _can_move(self) -> bool:
        return (self._enabled and not self._alarm and not self._estop
                and not self._collision)

    def _queue_len(self) -> int:
        return len(self._queue) + (1 if self._motion is not None else 0)

    def _new_id(self) -> int:
        self._next_id += 1
        return self._next_id

    def _clear_servo(self) -> None:
        self._servo = None
        self._servo_fifo.clear()
        self._servo_active = False

    def _enqueue(self, echo: str, name: str, joints: Sequence[float]) -> str:
        if not self._can_move():
            return self._reply_err(ERR_STATE, echo)
        if self._queue_len() >= self._queue_capacity:
            # 真机固件在积压超过阈值时直接拒收（-50001 一类）
            return self._reply_err(ERR_QUEUE, echo)
        if kin.within_limits(list(joints)):
            return self._reply_err(ERR_LIMIT, echo)
        rid = self._new_id()
        self._queue.append(_Motion(rid, name, self._q_target, joints))
        # 队列指令一来，伺服流就结束（真机也是：下一条队列指令接管）
        self._clear_servo()
        return self._reply_ok(echo, rid)

    def _cmd_movj(self, echo, args, kwargs, now):
        target = self._target_of(args, kwargs)
        if target is None:
            return self._reply_err(ERR_QUEUE, echo)
        return self._enqueue(echo, "MovJ", target)

    def _cmd_movl(self, echo, args, kwargs, now):
        # [实测 2026-11-25] r 必须是**整数**：r=3 成功，r=3.0 回 -50001（参数类型错）。
        # 这条不模拟的话，`round()` 换成 `floor(x+0.5)` 那个 fix 在仿真里就验证不了。
        raw_r = kwargs.get("r")
        if raw_r is not None and not _is_int_literal(raw_r):
            return self._reply_err(ERR_QUEUE, echo)
        target = self._target_of(args, kwargs)
        if target is None:
            return self._reply_err(ERR_QUEUE, echo)
        return self._enqueue(echo, "MovL", target)

    def _target_of(self, args, kwargs) -> Optional[List[float]]:
        """从 MovJ/MovL 的参数里取出目标关节角：joint= 直接用，pose= 走 IK。"""
        if "joint" in kwargs:
            values = _parse_values(kwargs["joint"])
            return values if len(values) == 6 else None
        token = kwargs.get("pose") or (args[0] if args else None)
        if token is None:
            return None
        pose = _parse_values(token)
        if len(pose) != 6:
            return None
        return self._solve_ik(pose)

    def _solve_ik(self, pose: Sequence[float]) -> Optional[List[float]]:
        """控制器的 InverseKin：数值反解 + 缓存（kin.ik 一次 150~250 ms）。"""
        key = tuple(round(float(v), 3) for v in pose)
        if key in self._ik_cache:
            return self._ik_cache[key]
        if len(self._ik_cache) > 128:
            self._ik_cache.clear()
        joints = kin.ik(list(pose), list(self._q_target))
        self._ik_cache[key] = joints
        return joints

    # -- 伺服 -------------------------------------------------------------

    def _servo_params(self, kwargs):
        t = float(kwargs.get("t", SERVO_DEFAULT_T))
        ahead = float(kwargs.get("aheadtime", SERVO_DEFAULT_AHEADTIME))
        gain = float(kwargs.get("gain", SERVO_DEFAULT_GAIN))
        return max(0.004, min(t, 3600.0)), ahead, gain

    def _push_servo(self, joints: Sequence[float], t: float,
                    aheadtime: float, gain: float, now: float) -> int:
        rid = self._new_id()
        frame = _ServoFrame(rid, joints, t, aheadtime, gain, now)
        if self.servo_queue == "fifo":
            self._servo_fifo.append(frame)
            self._servo = self._servo_fifo[0]
        else:
            # latest-wins：每个控制周期只看最新一帧（实测语义，见 docs 1.2）
            self._servo = frame
            self._servo_fifo = [frame]
        self._servo_active = True
        self._servo_frames += 1
        self._servo_clock = 0.0
        self._motion = None            # 伺服流一旦开始就接管，队列丢弃
        self._queue.clear()
        self._stop_until = 0.0
        return rid

    def _cmd_servoj(self, echo, args, kwargs, now):
        if not self._can_move():
            # 实测：未使能时回 -1（状态不允许），不是 -10000
            return self._reply_err(ERR_STATE, echo)
        values = _parse_values(",".join(args[:6])) if args else []
        if len(values) != 6:
            return self._reply_err(ERR_QUEUE, echo)
        if kin.within_limits(values):
            return self._reply_err(ERR_LIMIT, echo)
        t, ahead, gain = self._servo_params(kwargs)
        return self._reply_ok(echo, self._push_servo(values, t, ahead, gain, now))

    def _cmd_servop(self, echo, args, kwargs, now):
        if not self._can_move():
            return self._reply_err(ERR_STATE, echo)
        values = _parse_values(",".join(args[:6])) if args else []
        if len(values) != 6:
            return self._reply_err(ERR_QUEUE, echo)
        joints = self._solve_ik(values)
        if joints is None:
            return self._reply_err(ERR_QUEUE, echo)
        t, ahead, gain = self._servo_params(kwargs)
        return self._reply_ok(echo, self._push_servo(joints, t, ahead, gain, now))

    # ============================================================ 仿真推进

    def _tick_loop(self) -> None:
        while not self._stopping.is_set():
            time.sleep(TICK_PERIOD)
            with self._lock:
                try:
                    self._step(time.perf_counter())
                except Exception:                     # pragma: no cover - 线程不能死
                    pass

    def _set_joints(self, joints: Sequence[float]) -> None:
        self._q_target = [float(v) for v in joints]

    def _step(self, now: float) -> None:
        """推进一拍：队列 → 伺服 → 实际关节 → 模式 → 历史（调用方持锁）。"""
        dt = max(0.0, now - self._last_tick)
        self._last_tick = now
        halted = (self._alarm or self._estop or self._collision
                  or now < self._stop_until)

        if not halted:
            self._advance_queue(now)
            if self._servo_active:
                # 伺服按 30 ms 控制周期推进（实测节奏），和调用频率无关
                self._servo_clock += dt
                steps = 0
                while self._servo_clock >= SERVO_PERIOD and steps < 50:
                    self._servo_clock -= SERVO_PERIOD
                    self._advance_servo()
                    steps += 1
        self._advance_actual(dt, halted)
        self._update_mode(halted)

        self._history.append((now, tuple(self._q_actual)))
        if len(self._history) > 400:
            del self._history[:200]

    def _advance_queue(self, now: float) -> None:
        motion = self._motion
        if motion is None:
            self._start_next(now)
            return
        if motion.duration <= 0.0 or now >= motion.t0 + motion.duration:
            self._set_joints(motion.target)
            self._current_id = motion.rid
            self._motion = None
            self._start_next(now)
            return
        frac = _clamp01((now - motion.t0) / motion.duration)
        self._set_joints([a + (b - a) * frac
                          for a, b in zip(motion.start, motion.target)])

    def _start_next(self, now: float) -> None:
        if not self._queue:
            return
        motion = self._queue.pop(0)
        delta = max(abs(b - a) for a, b in zip(self._q_target, motion.target))
        # SpeedFactor 只影响 MovJ/MovL/Jog（手册 p.89）；100% 时按 100 °/s 走
        speed = JOINT_MAX_SPEED * max(1.0, min(self._speed_factor, 100.0)) / 100.0
        motion.t0 = now
        motion.start = list(self._q_target)
        motion.duration = min(MOVE_MAX_SECONDS,
                              max(MOVE_MIN_SECONDS, delta / speed if speed > 0 else 0.1))
        self._motion = motion
        self._current_id = motion.rid
        self._servo_active = False

    def _advance_servo(self) -> None:
        frame = self._servo
        if frame is None:
            return
        # t 被当成等效一阶低通的时间常数：控制器朝「t 秒前的目标」走
        alpha = _clamp01(1.0 - math.exp(-SERVO_PERIOD / max(frame.t, 1e-3)))
        alpha = max(0.05, min(alpha, 1.0))
        self._set_joints([a + (b - a) * alpha
                          for a, b in zip(self._q_target, frame.joints)])
        if self.servo_queue == "fifo" and len(self._servo_fifo) > 1:
            self._servo_fifo.pop(0)
            self._servo = self._servo_fifo[0]

    def _advance_actual(self, dt: float, halted: bool) -> None:
        """目标 → 实际：一阶跟随（约 1.5 个反馈周期）+ 关节速度上限。"""
        if halted:
            return
        tau = max(1.5 / max(self.feedback_hz, 1.0), 1e-3)
        step_cap = JOINT_MAX_SPEED * dt
        new: List[float] = []
        for target, actual in zip(self._q_target, self._q_actual):
            step = (target - actual) * min(1.0, dt / tau)
            if abs(step) > step_cap:
                step = math.copysign(step_cap, step)
            new.append(actual + step)
        self._q_actual = new

    def _update_mode(self, halted: bool) -> None:
        if self._alarm or self._estop:
            self._mode = MODE_ALARM
        elif not self._enabled:
            self._mode = MODE_DISABLED
        elif self._mode == MODE_DRAG:
            pass                                   # 拖拽示教，保持
        elif halted:
            self._mode = MODE_IDLE
        elif self._motion is not None or (self._servo_active and
                                          max(abs(b - a) for a, b in
                                              zip(self._q_target, self._q_actual)) > 0.05):
            self._mode = MODE_RUNNING
        else:
            self._mode = MODE_IDLE

    # ============================================================ 30004

    def _on_feedback(self, conn: socket.socket, addr: tuple) -> None:
        conn.settimeout(0.2)
        try:
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        period = 1.0 / max(self.feedback_hz, 1.0)
        next_at = time.perf_counter()
        with self._lock:
            self._feedback_threads += 1
        try:
            while not self._stopping.is_set():
                now = time.perf_counter()
                if now < next_at:
                    time.sleep(min(next_at - now, 0.05))
                    continue
                next_at += period
                if next_at < now - 0.25:
                    next_at = now + period          # 落后太多就重排，别补发
                with self._lock:
                    self._step(now)
                    drop = self._should_drop()
                    frame = None if drop else self._build_frame(now)
                    if drop:
                        self._dropped += 1
                if frame is None:
                    continue
                try:
                    conn.sendall(frame)
                except OSError:
                    return
        finally:
            with self._lock:
                self._feedback_threads -= 1
            try:
                conn.close()
            except OSError:
                pass

    def _should_drop(self) -> bool:
        if self._drop_seeded is not None:
            return self._dropped < self._drop_seeded
        if self._drop_ratio <= 0.0:
            return False
        return self._rng.random() < self._drop_ratio

    def _joints_at(self, t: float) -> List[float]:
        """取 t 时刻的实际关节角（用于 1~2 帧的反馈滞后）。"""
        history = self._history
        if not history:
            return list(self._q_actual)
        if t <= history[0][0]:
            return list(history[0][1])
        for stamp, joints in reversed(history):
            if stamp <= t:
                return list(joints)
        return list(history[0][1])

    def _build_frame(self, now: float) -> bytes:
        """按 arm_feed._FIELDS 的偏移造一包 1440 字节。"""
        buf = bytearray(arm_feed.FRAME_SIZE)

        def put(name: str, value) -> None:
            off, fmt, count = arm_feed._FIELDS[name]
            if fmt == "c":
                raw = (value.encode("ascii", "ignore") if isinstance(value, str)
                       else bytes(value))
                buf[off:off + count] = raw[:count].ljust(count, b"\x00")
                return
            seq = list(value) if isinstance(value, (list, tuple)) else [value]
            struct.pack_into("<" + fmt * count, buf, off, *seq)

        lag = 1.2 / max(self.feedback_hz, 1.0)
        if self.jitter_ms[1] > 0.0:
            lag += self._rng.uniform(*self.jitter_ms) / 1000.0
        q_actual = self._joints_at(now - lag)
        tcp_actual = kin.fk(q_actual)
        tcp_target = kin.fk(self._q_target)

        v_joints = [0.0] * 6
        tcp_speed = [0.0] * 6
        if self._last_frame is not None:
            prev_at = float(self._last_frame["_at"])
            span = max(now - prev_at, 1e-3)
            v_joints = [(b - a) / span
                        for a, b in zip(self._last_frame["QActual"], q_actual)]
            prev_tcp = self._last_frame["ToolVectorActual"]
            tcp_speed = ([(b - a) / span for a, b in zip(prev_tcp[:3], tcp_actual[:3])]
                         + [(b - a) / span for a, b in zip(prev_tcp[3:6], tcp_actual[3:6])])

        safety = 0xFF
        if self._estop:
            safety &= ~0x01                     # SafetyState 的位是低有效
        if self._collision:
            safety &= ~0x02

        pending = self._motion
        q_target = list(pending.target) if pending is not None else list(self._q_target)

        put("MessageSize", arm_feed.FRAME_SIZE)
        put("DigitalInputs", 0)
        put("DigitalOutputs", 0)
        put("RobotMode", self._mode)
        put("TimeStamp", int(time.time() * 1000))
        put("RunTime", int((now - self._t0) * 1000))
        put("TestValue", arm_feed.TEST_VALUE)
        put("SpeedScaling", self._speed_factor)
        put("VRobot", 48.2)                     # 实测母线 48.2 V
        put("IRobot", 3.3)
        put("ProgramState", 0.0)
        put("SafetyIOIn", "OK")
        put("SafetyIOOut", "OK")
        put("QTarget", q_target)
        put("QDTarget", [0.0] * 6)
        put("QDDTarget", [0.0] * 6)
        put("ITarget", [1.0] * 6)
        put("MTarget", [0.0] * 6)
        put("QActual", q_actual)                # 相对 QTarget 滞后 1~2 帧
        put("QDActual", [round(v, 4) for v in v_joints])
        put("IActual", [round(0.8 + 0.3 * math.sin(math.radians(q)), 3) for q in q_actual])
        put("ActualTCPForce", [0.0] * 6)
        put("ToolVectorActual", tcp_actual)
        put("TCPSpeedActual", tcp_speed)
        put("TCPForce", [0.0] * 6)
        put("ToolVectorTarget", tcp_target)
        put("TCPSpeedTarget", [0.0] * 6)
        put("MotorTemperatures", [34.0 + 0.5 * i for i in range(6)])
        put("JointModes", [0.0] * 6)
        put("VActual", v_joints)
        put("HandType", "NONE")
        put("User", 0)
        put("Tool", 0)
        put("RunQueuedCmd", 1 if self._enabled else 0)
        put("PauseCmdFlag", 0)
        put("VelocityRatio", int(self._speed_factor))
        put("AccelerationRatio", int(self._speed_factor))
        put("XYZVelocityRatio", int(self._speed_factor))
        put("RVelocityRatio", int(self._speed_factor))
        put("XYZAccelerationRatio", int(self._speed_factor))
        put("RAccelerationRatio", int(self._speed_factor))
        put("BrakeStatus", 0)                   # 实测：使能/下使能都恒为 0
        put("EnableStatus", 1 if self._enabled else 0)
        put("DragStatus", 1 if self._mode == MODE_DRAG else 0)
        put("RunningStatus", 1 if self._mode == MODE_RUNNING else 0)
        put("ErrorStatus", self._error_status)
        put("JogStatusCR", 0)
        put("CRRobotType", 1)
        put("SixForceOnline", 1)
        put("CollisionState", 1 if self._collision else 0)
        put("VibrationDisZ", 0.0)
        put("CurrentCommandId", self._current_id)
        put("MActual", [0.0] * 6)
        put("Load", 0.51)
        put("CenterX", 0.0)
        put("CenterY", 0.0)
        put("CenterZ", 0.0)
        put("UserFrame", [0.0] * 6)
        put("ToolFrame", [0.0] * 6)
        put("SixForceValue", [0.0] * 6)
        put("TargetQuaternion", _quat(kin.flange(q_target)[1]))
        put("ActualQuaternion", _quat(kin.flange(q_actual)[1]))
        put("AutoManualMode", "MA")
        put("ExportStatus", 0)
        put("SafetyState", safety)
        put("SafeState", 0)

        frame = arm_feed.parse_frame(bytes(buf))
        frame["_at"] = now
        self._last_frame = frame
        return bytes(buf)

    # ============================================================ 对外 API

    def state(self) -> dict:
        """当前状态快照（测试断言用）。"""
        with self._lock:
            now = time.perf_counter()
            self._step(now)
            return {
                "mode": self._mode,
                "enabled": self._enabled,
                "alarm": self._alarm,
                "estop": self._estop,
                "collision": self._collision,
                "pose": [round(v, 4) for v in kin.fk(self._q_actual)],
                "target_pose": [round(v, 4) for v in kin.fk(self._q_target)],
                "joints": [round(v, 5) for v in self._q_actual],
                "target_joints": [round(v, 5) for v in self._q_target],
                "error": self._error_status,
                "queue_len": self._queue_len(),
                "queue_capacity": self._queue_capacity,
                "current_id": self._current_id,
                "servo_frames": self._servo_frames,
                "servo_active": self._servo_active,
                "servo_queue": self.servo_queue,
                "servo_queue_len": len(self._servo_fifo),
                "speed_factor": self._speed_factor,
                "clients": 1 if self._client is not None else 0,
                "kicked": self._kicked,
                "feedback_clients": self._feedback_threads,
                "dropped_frames": self._dropped,
                "last_latency_ms": round(self.last_latency_ms, 3),
                "latency_scale": self.latency_scale,
                "uptime_s": round(now - self._t0, 3),
            }

    def settle(self, seconds: float = 0.2) -> None:
        """按墙钟推进 seconds 秒的仿真时间（没有起线程时也照样能推）。"""
        deadline = time.perf_counter() + max(0.0, float(seconds))
        while True:
            now = time.perf_counter()
            with self._lock:
                self._step(now)
            if now >= deadline:
                return
            time.sleep(min(TICK_PERIOD, max(0.0, deadline - now)))

    def inject(self, fault: str, **kwargs) -> dict:
        """注入故障。未知故障抛 ValueError。

        ================  ==================================================
        alarm             code= 报警码（默认 1）→ ErrorStatus 置位、RobotMode=9、停运动
        estop             on= 按下/松开急停；按下时 SafetyState 的 estop 位清零
        collision         on= 碰撞：CollisionState=1 并立刻停住（不再跟随目标）
        drop_frames       ratio= 丢帧比例（0~1），或 frames= 连续丢 N 帧
        queue_backlog     n= 往队列里塞 n 条占位指令；capacity= 顺带改容量
        disconnect        断开所有 29999 / 30004 连接
        reset             清掉所有注入
        ================  ==================================================
        """
        name = str(fault).strip().lower()
        with self._lock:
            now = time.perf_counter()
            if name == "alarm":
                code = int(kwargs.get("code", 1))
                self._alarm = True
                self._error_status = code
                self._mode = MODE_ALARM
                self._clear_servo()
                self._motion = None
                self._queue.clear()
                return {"fault": name, "error": code, "mode": self._mode}
            if name == "estop":
                on = bool(kwargs.get("on", True))
                self._estop = on
                if on:
                    self._error_status = self._error_status or 1
                return {"fault": name, "on": on, "estop": self._estop}
            if name == "collision":
                on = bool(kwargs.get("on", True))
                self._collision = on
                if on:
                    self._clear_servo()
                    self._motion = None
                    self._queue.clear()
                return {"fault": name, "on": on, "collision": self._collision}
            if name == "drop_frames":
                self._drop_ratio = max(0.0, min(float(kwargs.get("ratio", 0.0)), 1.0))
                frames = kwargs.get("frames")
                self._drop_seeded = None if frames is None else int(frames)
                self._dropped = 0
                return {"fault": name, "ratio": self._drop_ratio,
                        "frames": self._drop_seeded}
            if name == "queue_backlog":
                n = int(kwargs.get("n", 8))
                if "capacity" in kwargs:
                    self._queue_capacity = max(0, int(kwargs["capacity"]))
                for _ in range(n):
                    filler = _Motion(self._new_id(), "Blocked", self._q_target,
                                     list(self._q_target), 3600.0)
                    filler.t0 = now
                    self._queue.append(filler)
                return {"fault": name, "queue_len": self._queue_len(),
                        "capacity": self._queue_capacity}
            if name == "disconnect":
                self._close_client(notify=True)
                self._stopping.set()
                return {"fault": name, "kicked": self._kicked}
            if name == "reset":
                self._drop_ratio = 0.0
                self._drop_seeded = None
                self._dropped = 0
                self._alarm = False
                self._estop = False
                self._collision = False
                self._error_status = 0
                self._queue.clear()
                self._motion = None
                self._queue_capacity = 32
                self._mode = MODE_IDLE if self._enabled else MODE_DISABLED
                return {"fault": name, "ok": True}
        raise ValueError("未知故障 %r（可用：alarm / estop / collision / drop_frames / "
                         "queue_backlog / disconnect / reset）" % (fault,))


# ---------------------------------------------------------------- 客户端糖

class _VirtualArm(DobotArm):
    """DobotArm 的一层薄壳：多了 servo_j / servo_p / mode / queue_len。

    29999 的文本协议一模一样，所以 DobotArm 的每个方法都原样可用；
    这里只是把真机上要手动拼字符串的 ServoJ/ServoP 包了一下（原始指令仍可自己发）。
    """

    def __init__(self, sim: Optional[VirtualDobot] = None, **kwargs) -> None:
        super().__init__(**kwargs)
        self.sim = sim
        self.last_latency_ms = 0.0

    def _command(self, text: str) -> str:
        started = time.perf_counter()
        try:
            return super()._command(text)
        finally:
            self.last_latency_ms = (time.perf_counter() - started) * 1000.0

    def servo_j(self, joints: Sequence[float], t: float = SERVO_DEFAULT_T,
                aheadtime: float = SERVO_DEFAULT_AHEADTIME,
                gain: float = SERVO_DEFAULT_GAIN) -> int:
        """ServoJ：返回 ResultID。t 默认 0.1（手册默认值）。"""
        if len(joints) != 6:
            raise ValueError("关节角需要 6 个数")
        text = self._check(self._command(
            "ServoJ(%s,t=%s,aheadtime=%s,gain=%s)" % (
                _fmt_values(joints), _num(t), _num(aheadtime), _num(gain))))
        rid = parse_result_id(text)
        if rid is None:
            raise RuntimeError("ServoJ 没返回 ResultID: %r" % (text,))
        return rid

    def servo_p(self, pose: Sequence[float], t: float = SERVO_DEFAULT_T,
                aheadtime: float = SERVO_DEFAULT_AHEADTIME,
                gain: float = SERVO_DEFAULT_GAIN) -> int:
        """ServoP：位姿直接下发，控制器内部做逆解。"""
        if len(pose) != 6:
            raise ValueError("姿态需要 6 个数")
        text = self._check(self._command(
            "ServoP(%s,t=%s,aheadtime=%s,gain=%s)" % (
                _fmt_values(pose), _num(t), _num(aheadtime), _num(gain))))
        rid = parse_result_id(text)
        if rid is None:
            raise RuntimeError("ServoP 没返回 ResultID: %r" % (text,))
        return rid

    @property
    def mode(self) -> int:
        reply = self._command(self.CMD_MODE)
        start, end = reply.find("{"), reply.find("}", reply.find("{"))
        try:
            return int(float(reply[start + 1:end]))
        except (ValueError, TypeError):
            return -1

    def queue_len(self) -> int:
        """仿真专用：控制器里还压着几条没执行完。"""
        if self.sim is None:
            return -1
        return int(self.sim.state()["queue_len"])


def _main() -> int:
    """python -m handlink.armsim：起一台虚拟控制器，打印一帧反馈就退出。"""
    import json
    sim = VirtualDobot().start()
    try:
        print(json.dumps(sim.state(), ensure_ascii=False, indent=2))
        feed = arm_feed.Feed("127.0.0.1", sim.feedback_port).open()
        try:
            frame = feed.wait_frame(timeout=2.0)
            print(json.dumps(arm_feed.describe(frame), ensure_ascii=False, indent=2))
        finally:
            feed.close()
    finally:
        sim.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
