"""30004 实时反馈解析 —— 越疆 Dobot TCP/IP API V4.6。

和 ``screw.py`` 里那种「一帧一个值」的 CAN 报文不同，这里每包固定 1440 字节，
**小端**存放 double / uint64，字段位置写死在厂家手册的表里（`docs/` 里有 PDF）。
客户端每 8 ms 收一帧，六轴角度、TCP 位姿、电流、抱闸、使能、报警全在里面。

只读：这个模块永远不往臂上发东西。要发指令走 ``arm.py`` 的 29999 Dashboard。

    from handlink.arm_feed import Feed
    with Feed() as f:
        f.wait_frame(timeout=3)
        print(f.pose(), f.joints(), f.mode_name)
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Dict, List, Optional

FRAME_SIZE = 1440
DEFAULT_IP = "192.168.5.1"
FEEDBACK_PORT = 30004

#: 手册里的内存结构测试值，用来确认字节序没搞反
TEST_VALUE = 0x0123456789ABCDEF

#: 机器人模式（RobotMode 指令说明）
ROBOT_MODES = {
    1: "初始化", 2: "抱闸松开", 3: "未上电", 4: "下使能", 5: "使能空闲",
    6: "拖拽(背驱)", 7: "运行中", 8: "单次运动", 9: "报警", 10: "暂停", 11: "点动",
}
#: 允许 RequestControl() 切 TCP 模式的模式（手册 2.1 的表格）
CONTROLLABLE_MODES = {3, 4}

# 字段表：(偏移, 结构, 数量)。结构 d=double, Q=uint64, I=uint32, H=uint16, B=uint8, c=char
_FIELDS: Dict[str, tuple] = {
    "MessageSize": (0, "H", 1),
    "DigitalInputs": (8, "Q", 1),
    "DigitalOutputs": (16, "Q", 1),
    "RobotMode": (24, "Q", 1),
    "TimeStamp": (32, "Q", 1),
    "RunTime": (40, "Q", 1),
    "TestValue": (48, "Q", 1),
    "SpeedScaling": (64, "d", 1),
    "VRobot": (88, "d", 1),
    "IRobot": (96, "d", 1),
    "ProgramState": (104, "d", 1),
    "SafetyIOIn": (112, "c", 2),
    "SafetyIOOut": (114, "c", 2),
    "QTarget": (192, "d", 6),
    "QDTarget": (240, "d", 6),
    "QDDTarget": (288, "d", 6),
    "ITarget": (336, "d", 6),
    "MTarget": (384, "d", 6),
    "QActual": (432, "d", 6),
    "QDActual": (480, "d", 6),
    "IActual": (528, "d", 6),
    "ActualTCPForce": (576, "d", 6),
    "ToolVectorActual": (624, "d", 6),
    "TCPSpeedActual": (672, "d", 6),
    "TCPForce": (720, "d", 6),
    "ToolVectorTarget": (768, "d", 6),
    "TCPSpeedTarget": (816, "d", 6),
    "MotorTemperatures": (864, "d", 6),
    "JointModes": (912, "d", 6),
    "VActual": (960, "d", 6),
    "HandType": (1008, "c", 4),
    "User": (1012, "B", 1),
    "Tool": (1013, "B", 1),
    "RunQueuedCmd": (1014, "B", 1),
    "PauseCmdFlag": (1015, "B", 1),
    "VelocityRatio": (1016, "B", 1),
    "AccelerationRatio": (1017, "B", 1),
    "XYZVelocityRatio": (1019, "B", 1),
    "RVelocityRatio": (1020, "B", 1),
    "XYZAccelerationRatio": (1021, "B", 1),
    "RAccelerationRatio": (1022, "B", 1),
    "BrakeStatus": (1025, "B", 1),
    "EnableStatus": (1026, "B", 1),
    "DragStatus": (1027, "B", 1),
    "RunningStatus": (1028, "B", 1),
    "ErrorStatus": (1029, "B", 1),
    "JogStatusCR": (1030, "B", 1),
    "CRRobotType": (1031, "B", 1),
    "SixForceOnline": (1037, "B", 1),
    "CollisionState": (1038, "B", 1),
    "VibrationDisZ": (1104, "d", 1),
    "CurrentCommandId": (1112, "Q", 1),
    "MActual": (1120, "d", 6),
    "Load": (1168, "d", 1),
    "CenterX": (1176, "d", 1),
    "CenterY": (1184, "d", 1),
    "CenterZ": (1192, "d", 1),
    "UserFrame": (1200, "d", 6),
    "ToolFrame": (1248, "d", 6),
    "SixForceValue": (1304, "d", 6),
    "TargetQuaternion": (1352, "d", 4),
    "ActualQuaternion": (1384, "d", 4),
    "AutoManualMode": (1416, "c", 2),
    "ExportStatus": (1418, "H", 1),
    "SafetyState": (1420, "B", 1),
    "SafeState": (1421, "B", 1),
}


class FeedbackError(RuntimeError):
    """反馈口的读取/解析错误。"""


def parse_frame(data: bytes) -> Dict[str, object]:
    """把一包 1440 字节解析成 dict。裸字段是原值，单元素字段会摊平。"""
    if len(data) < FRAME_SIZE:
        raise FeedbackError(f"包长度不足：{len(data)} < {FRAME_SIZE}")
    out: Dict[str, object] = {}
    for name, (off, fmt, count) in _FIELDS.items():
        values = struct.unpack_from("<" + fmt * count, data, off)
        if fmt == "c":
            out[name] = b"".join(values).rstrip(b"\x00").decode("ascii", "ignore")
        elif fmt == "d":
            out[name] = [float(v) for v in values] if count > 1 else float(values[0])
        else:
            out[name] = [int(v) for v in values] if count > 1 else int(values[0])
    return out


def describe(frame: Dict[str, object]) -> Dict[str, object]:
    """给人看的摘要：位姿、关节角、模式名、报警/安全状态。"""
    mode = int(frame["RobotMode"])
    safety = int(frame["SafetyState"])
    return {
        "mode": mode,
        "mode_name": ROBOT_MODES.get(mode, f"未知({mode})"),
        "can_take_control": mode in CONTROLLABLE_MODES,
        "enabled": int(frame["EnableStatus"]) == 1,
        "brakes": int(frame["BrakeStatus"]),
        "running": int(frame["RunningStatus"]),
        "error": int(frame["ErrorStatus"]),
        "collision": int(frame["CollisionState"]),
        "pose": [round(v, 3) for v in frame["ToolVectorActual"]],
        "joints": [round(v, 3) for v in frame["QActual"]],
        "current_a": [round(v, 3) for v in frame["IActual"]],
        "speed_scaling": round(float(frame["SpeedScaling"]), 1),
        "voltage": round(float(frame["VRobot"]), 2),
        "run_time_s": round(int(frame["RunTime"]) / 1000.0, 1),
        "safety": {
            "estop": not (safety & 0x01),
            "protective_stop": not (safety & 0x02),
            "reduced_mode": not (safety & 0x04),
            "moving": not (safety & 0x20),
        },
    }


def carve(buf: bytes) -> tuple:
    """从字节流里切出完整的帧，返回 ``(帧列表, 剩下的半包, 丢掉的字节数)``。

    连上 30004 时可能正好接在某一包中间，所以先看头两个字节是不是 1440，
    不是就往后找一个 1440 的起点。切出来的包还要过 TestValue 校验才算数。
    """
    frames = []
    skipped = 0
    while len(buf) >= FRAME_SIZE:
        if struct.unpack_from("<H", buf, 0)[0] != FRAME_SIZE:
            nxt = buf.find(struct.pack("<H", FRAME_SIZE), 1)
            if nxt < 0:
                skipped += len(buf) - 1
                buf = buf[-1:]
                break
            skipped += nxt
            buf = buf[nxt:]
            continue
        packet, buf = buf[:FRAME_SIZE], buf[FRAME_SIZE:]
        frame = parse_frame(packet)
        if frame["TestValue"] != TEST_VALUE:
            skipped += 1
            continue
        frames.append(frame)
    return frames, buf, skipped


class Feed:
    """后台线程收 30004，永远保留最新一帧。只读，不发送。"""

    def __init__(self, ip: str = DEFAULT_IP, port: int = FEEDBACK_PORT,
                 timeout: float = 3.0) -> None:
        self.ip = ip
        self.port = int(port)
        self.timeout = float(timeout)
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame: Optional[Dict[str, object]] = None
        self._frame_at = 0.0
        self._first_at = 0.0
        self.frames = 0
        self.bad_frames = 0
        self.last_error: Optional[str] = None

    # -- 生命周期 ---------------------------------------------------------
    def open(self) -> "Feed":
        if self._sock is not None:
            return self
        try:
            sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        except OSError as exc:
            raise FeedbackError(f"连不上反馈口 {self.ip}:{self.port}（{exc}）") from exc
        sock.settimeout(0.5)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="arm-feed", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        t, self._thread = self._thread, None
        if t is not None:
            t.join(timeout=1.5)
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def __enter__(self) -> "Feed":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- 收包 -------------------------------------------------------------
    def _loop(self) -> None:
        buf = b""
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                chunk = self._sock.recv(8192)
            except socket.timeout:
                continue
            except OSError as exc:
                self.last_error = str(exc)
                return
            if not chunk:
                self.last_error = "对端关闭"
                return
            buf += chunk
            # 对齐：MessageSize 必须是 1440，TestValue 必须等于手册里的魔数
            try:
                frames, buf, skipped = carve(buf)
            except FeedbackError as exc:
                self.bad_frames += 1
                self.last_error = str(exc)
                buf = b""
                continue
            self.bad_frames += skipped
            for frame in frames:
                with self._lock:
                    self._frame = frame
                    self._frame_at = time.time()
                    if not self._first_at:
                        self._first_at = self._frame_at
                    self.frames += 1

    # -- 读取 -------------------------------------------------------------
    def wait_frame(self, timeout: float = 3.0) -> Dict[str, object]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._frame is not None:
                    return dict(self._frame)
            time.sleep(0.02)
        raise FeedbackError(f"{timeout:.0f}s 内没收到有效帧（错帧 {self.bad_frames}）")

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            if self._frame is None:
                raise FeedbackError("还没有帧")
            return dict(self._frame)

    def age(self) -> float:
        with self._lock:
            return time.time() - self._frame_at if self._frame_at else float("inf")

    def rate_hz(self) -> float:
        """从第一帧到现在平均下来多少帧/秒（手册标称 125 Hz）。"""
        with self._lock:
            span = self._frame_at - self._first_at
            return self.frames / span if span > 1e-6 else 0.0

    @property
    def pose(self) -> List[float]:
        return [float(v) for v in self.snapshot()["ToolVectorActual"]]

    @property
    def joints(self) -> List[float]:
        return [float(v) for v in self.snapshot()["QActual"]]

    @property
    def mode(self) -> int:
        return int(self.snapshot()["RobotMode"])

    @property
    def mode_name(self) -> str:
        return ROBOT_MODES.get(self.mode, f"未知({self.mode})")

    def summary(self) -> Dict[str, object]:
        return describe(self.snapshot())


def _main() -> int:
    import json
    import sys
    ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    feed = Feed(ip).open()
    try:
        feed.wait_frame(timeout=5)
        time.sleep(0.3)
        print(json.dumps(feed.summary(), ensure_ascii=False, indent=2))
        print(f"帧率约 {feed.rate_hz():.1f} Hz，错帧 {feed.bad_frames}")
        return 0
    finally:
        feed.close()


if __name__ == "__main__":
    raise SystemExit(_main())
