"""真机总线：带后台收包线程的 CAN 客户端。

为什么必须有收包线程：这台手每秒往总线上灌约 3000 帧心跳（每根手指约 500Hz），
如果只在发命令时才 ``Receive``，驱动内部接收缓冲区很快被填满，
之后 ``Transmit`` 会**阻塞**——表现为脚本卡死。所以收发分离。

    with Hand() as h:
        h.move(104, 30, speed=200, current=0.5)
        print(h.read_position(104))
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from . import screw, spec
from .bus import HandBus, MotorState
from .hardware import ZlgCan

DEFAULT_DLL = (
    Path(__file__).resolve().parent.parent
    / "pkg" / "LG-603" / "src" / "main" / "resources" / "native" / "Windows-x86_64" / "ECanVci64.dll"
)


@dataclass
class Snapshot:
    positions: dict[int, float] = field(default_factory=dict)
    speeds: dict[int, float] = field(default_factory=dict)
    currents: dict[int, float] = field(default_factory=dict)
    heartbeats: dict[int, screw.Heartbeat] = field(default_factory=dict)
    hand_id: int | None = None

    def __str__(self) -> str:
        rows = []
        for motor, label in screw.MOTORS.items():
            p = self.positions.get(motor)
            c = self.currents.get(motor)
            hb = self.heartbeats.get(motor)
            rows.append(
                f"  {motor} {label:<11} "
                f"pos={p if p is None else round(p, 1):>6}  "
                f"I={c if c is None else round(c, 3):>5}  "
                f"{'' if hb is None else hb.state}"
            )
        return "\n".join(rows)


class Hand:
    def __init__(
        self,
        dll: str | Path = DEFAULT_DLL,
        device_type: int = 4,
        device_index: int = 0,
        channel: int = 0,
        baud: int = 1000,
        hand_id: int | None = None,
    ) -> None:
        self.bus = ZlgCan(Path(dll), device_type=device_type, device_index=device_index,
                          channel=channel, baud=baud)
        self.hand_id = hand_id
        self.snap = Snapshot()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._frames = 0

    # ---------------------------------------------------------------- lifecycle
    def open(self) -> "Hand":
        self.bus.open()
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader, name="can-rx", daemon=True)
        self._thread.start()
        if self.hand_id is None:
            self.hand_id = self._detect(1.0)
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.bus.close()

    def __enter__(self) -> "Hand":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # ---------------------------------------------------------------- rx thread
    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                frames = self.bus.read(max_frames=256, wait_ms=10)
            except Exception:  # noqa: BLE001
                time.sleep(0.01)
                continue
            now = time.time()
            for f in frames:
                self._frames += 1
                self._dispatch(f["id"], f["data"], now)

    def _dispatch(self, can_id: int, data: bytes, now: float) -> None:
        parsed = screw.parse(data)
        if parsed is None:
            return
        if can_id < 0x200:  # 手部 ID 是 0x64/0x65 这类小号
            self.snap.hand_id = can_id
        if isinstance(parsed, screw.Heartbeat):
            self.snap.heartbeats[parsed.node] = parsed
            return
        if parsed.value is None:
            return
        func = parsed.func
        if func == 0xA5:
            self.snap.positions[parsed.node] = parsed.value
        elif func == 0xA7:
            self.snap.speeds[parsed.node] = parsed.value
        elif func == 0xA9:
            self.snap.currents[parsed.node] = parsed.value
        self.snap.hand_id = can_id

    def _detect(self, seconds: float) -> int:
        deadline = time.time() + seconds
        while time.time() < deadline:
            if self.snap.hand_id:
                return self.snap.hand_id
            time.sleep(0.02)
        return self._guess()

    def _guess(self) -> int:
        return self.snap.hand_id or screw.HAND_ID

    # ---------------------------------------------------------------- tx
    def send(self, data: bytes) -> bool:
        if self.hand_id is None:
            self.hand_id = self._guess()
        with self._lock:
            return self.bus.send(self.hand_id, data)

    def enable(self, motor: int, on: bool = True) -> None:
        self.send(screw.enable(motor, on))

    def enable_all(self, on: bool = True) -> None:
        """六个电机的驱动器总开关。

        ``on=False`` 就是**泄力**：驱动器关掉、彻底不输出力矩，手指可以用手掰。
        注意这和「把电流上限设成 0」不是一回事——后者驱动器仍然使能，靠内部
        闭环保持位置，手感还是硬的（实测确认过）。
        """
        for mid in spec.MOTOR_IDS:
            self.send(screw.enable(mid, on))

    def set_speed(self, motor: int, speed: float) -> None:
        self.send(screw.set_speed(motor, speed))

    def set_current(self, motor: int, current: float) -> None:
        self.send(screw.set_current(motor, screw.clamp_current(motor, current)))

    def set_position(self, motor: int, angle: float) -> None:
        self.send(screw.set_position(motor, screw.clamp_angle(motor, angle)))

    def move(self, motor: int, angle: float, speed: float = 120, current: float = 0.4) -> None:
        self.enable(motor, True)
        self.set_speed(motor, speed)
        self.set_current(motor, current)
        self.set_position(motor, angle)

    # ---------------------------------------------------------------- query
    def refresh(self, motors=None, timeout: float = 0.5) -> Snapshot:
        motors = list(motors or screw.MOTORS)
        for motor in motors:
            self.send(screw.get_position(motor))
            self.send(screw.get_speed(motor))
            self.send(screw.get_current(motor))
        deadline = time.time() + timeout
        while time.time() < deadline:
            if all(m in self.snap.positions for m in motors):
                if all(m in self.snap.currents for m in motors):
                    break
            time.sleep(0.02)
        return self.snap

    def read_position(self, motor: int, timeout: float = 0.6) -> float | None:
        before = self.snap.positions.get(motor)
        self.send(screw.get_position(motor))
        deadline = time.time() + timeout
        while time.time() < deadline:
            now = self.snap.positions.get(motor)
            if now is not None and now != before:
                return now
            time.sleep(0.01)
        return self.snap.positions.get(motor)

    def wait_settled(self, motor: int, target: float, tolerance: float = 1.0,
                     timeout: float = 5.0, period: float = 0.1) -> float | None:
        deadline = time.time() + timeout
        last = self.snap.positions.get(motor)
        while time.time() < deadline:
            last = self.read_position(motor) or last
            if last is not None and abs(last - target) <= tolerance:
                return last
            time.sleep(period)
        return last

    @property
    def frames_seen(self) -> int:
        return self._frames


# --------------------------------------------------------------------- 适配层
class CanBus(HandBus):
    """把真机 :class:`Hand` 包装成服务层认识的 ``HandBus``。"""

    name = "can"

    def __init__(self, dll: str | Path = DEFAULT_DLL, hand_id: int | None = None,
                 poll_hz: float = 5.0) -> None:
        self.hand = Hand(dll=dll, hand_id=hand_id)
        self.poll_hz = float(poll_hz)
        self._target = {mid: 0.0 for mid in spec.MOTOR_IDS}
        self._lock = threading.RLock()
        self._poll_stop = threading.Event()
        self._poll_thread: threading.Thread | None = None

    # -- 生命周期 ---------------------------------------------------------
    def open(self) -> None:
        self.hand.open()
        self.hand.refresh(timeout=1.0)
        for mid in spec.MOTOR_IDS:
            v = self.hand.snap.positions.get(mid)
            if v is not None:
                self._target[mid] = v
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(target=self._poll, name="hand-poll", daemon=True)
        self._poll_thread.start()

    def close(self) -> None:
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=1.0)
        self.hand.close()

    def _poll(self) -> None:
        period = 1.0 / max(1.0, self.poll_hz)
        while not self._poll_stop.is_set():
            for mid in spec.MOTOR_IDS:
                try:
                    self.hand.send(screw.get_position(mid))
                    self.hand.send(screw.get_current(mid))
                except Exception:  # noqa: BLE001
                    pass
            self._poll_stop.wait(period)

    # -- 指令 -------------------------------------------------------------
    def enable_all(self, on: bool = True) -> None:
        """六个电机的驱动器总开关：``on=False`` 就是**泄力**，手指可用手掰。

        和「把电流上限设成 0」不是一回事：后者驱动器仍使能、靠内部闭环保持
        位置，手感还是硬的（2026-11-19 现场实测确认）。
        """
        with self._lock:
            for mid in spec.MOTOR_IDS:
                self.hand.enable(mid, on)

    def apply(self, pose: spec.Pose, reason: str = "") -> Dict[str, object]:
        unknown = [mid for mid in pose if mid not in spec.MOTOR_BY_ID]
        if unknown:
            raise ValueError(f"未知电机: {unknown}")
        with self._lock:
            for mid, joint in pose.items():
                angle = screw.clamp_angle(mid, joint.angle)
                speed = max(1.0, min(spec.SPEED_MAX, float(joint.speed)))
                current = screw.clamp_current(mid, joint.torque)
                self.hand.move(mid, angle, speed=speed, current=current)
                self._target[mid] = angle
                time.sleep(0.01)
        return {"bus": self.name, "applied": len(pose), "reason": reason, "t": time.time()}

    def stop(self) -> None:
        """真机上的「停止」= 就地定住，不是回零。"""
        with self._lock:
            for mid in spec.MOTOR_IDS:
                now = self.hand.snap.positions.get(mid, self._target.get(mid, 0.0))
                self.hand.move(mid, now, speed=120, current=1.0)
                self._target[mid] = now

    # -- 状态 -------------------------------------------------------------
    def state(self) -> Dict[int, MotorState]:
        snap = self.hand.snap
        out: Dict[int, MotorState] = {}
        for mid in spec.MOTOR_IDS:
            hb = snap.heartbeats.get(mid)
            error = None
            if hb is not None and hb.running & 0x7E:
                error = hb.state
            out[mid] = MotorState(
                motor_id=mid,
                angle=snap.positions.get(mid, 0.0),
                speed=snap.speeds.get(mid, 0.0),
                current=snap.currents.get(mid, 0.0),
                target=self._target.get(mid, 0.0),
                error=error,
            )
        return out

    def snapshot(self) -> Dict[str, object]:
        data = super().snapshot()
        snap = self.hand.snap
        data["hand_id"] = self.hand.hand_id
        data["frames"] = self.hand.frames_seen
        data["calibrated"] = all(
            hb.calib & 0x08 for hb in snap.heartbeats.values()
        ) if snap.heartbeats else False
        return data
