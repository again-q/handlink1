"""总线抽象、仿真总线与动作播放。

三种实现共用一套接口，上层（HTTP 服务、命令行、录制分析）不需要关心底下是
仿真、串口还是 CAN：

* :class:`SimBus`     —— 纯软件一阶位置伺服，可离线跑通全流程
* :class:`handlink.hardware.SerialBus` / ``ZlgCanBus`` —— 真机（实验性）
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional

from . import spec, store


@dataclass
class MotorState:
    motor_id: int
    angle: float
    speed: float
    current: float
    target: float
    error: Optional[str] = None

    def as_dict(self) -> Dict[str, object]:
        m = spec.MOTOR_BY_ID[self.motor_id]
        return {
            "motor_id": self.motor_id,
            "name": m.name,
            "angle": round(self.angle, 3),
            "speed": round(self.speed, 2),
            "current": round(self.current, 3),
            "target": round(self.target, 3),
            "current_limit": m.current_max,
            "error": self.error,
        }


class HandBus:
    """总线接口。"""

    name = "abstract"

    def open(self) -> None:  # pragma: no cover - 接口
        pass

    def close(self) -> None:  # pragma: no cover
        pass

    def apply(self, pose: spec.Pose, reason: str = "") -> Dict[str, object]:
        raise NotImplementedError

    def state(self) -> Dict[int, MotorState]:
        raise NotImplementedError

    def stop(self) -> None:
        self.apply(spec.blank_pose(speed=spec.FIRST_RUN_SPEED), reason="stop")

    # -- 通用工具 ---------------------------------------------------------
    def snapshot(self) -> Dict[str, object]:
        return {
            "bus": self.name,
            "t": time.time(),
            "motors": {str(mid): st.as_dict() for mid, st in self.state().items()},
        }

    def pose(self) -> spec.Pose:
        return {mid: spec.Joint(st.angle, spec.FIRST_RUN_SPEED, spec.MOTOR_BY_ID[mid].current_max * 0.2)
                for mid, st in self.state().items()}


class SimBus(HandBus):
    """仿真手：每个电机按指令速度逼近目标位置，并按负载估算电流。

    模型很朴素（一阶、无耦合、无摩擦），目的是让整条链路能离线跑通并且行为
    可预期，不是力学仿真。
    """

    name = "sim"

    def __init__(self, hz: float = 100.0, load: float = 0.15) -> None:
        self.hz = float(hz)
        self.load = float(load)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pos = {m.motor_id: 0.0 for m in spec.MOTORS}
        self._target = dict(self._pos)
        self._speed = {m.motor_id: spec.FIRST_RUN_SPEED for m in spec.MOTORS}
        self._torque = {m.motor_id: 0.3 for m in spec.MOTORS}
        self._current = {m.motor_id: 0.0 for m in spec.MOTORS}
        self._vel = {m.motor_id: 0.0 for m in spec.MOTORS}
        self._stalled: set = set()
        self._opened_at = 0.0

    # -- 生命周期 ---------------------------------------------------------
    def open(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._opened_at = time.time()
        self._thread = threading.Thread(target=self._run, name="sim-bus", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        self._thread = None

    # -- 指令 -------------------------------------------------------------
    def apply(self, pose: spec.Pose, reason: str = "") -> Dict[str, object]:
        unknown = [mid for mid in pose if mid not in spec.MOTOR_BY_ID]
        if unknown:
            raise ValueError(f"未知电机: {unknown}")
        with self._lock:
            for mid, j in pose.items():
                self._target[mid] = min(max(float(j.angle), 0.0), spec.MOTOR_BY_ID[mid].pos_max)
                self._speed[mid] = min(max(float(j.speed), 0.0), spec.SPEED_MAX)
                self._torque[mid] = min(max(float(j.torque), 0.0), spec.MOTOR_BY_ID[mid].current_max)
        return {"ok": True, "bus": self.name, "applied": sorted(pose), "reason": reason}

    def state(self) -> Dict[int, MotorState]:
        with self._lock:
            return {
                mid: MotorState(
                    motor_id=mid,
                    angle=self._pos[mid],
                    speed=self._vel[mid],
                    current=self._current[mid],
                    target=self._target[mid],
                )
                for mid in self._pos
            }

    # -- 仿真步 -----------------------------------------------------------
    def _run(self) -> None:
        dt = 1.0 / self.hz
        last = time.perf_counter()
        while not self._stop.is_set():
            now = time.perf_counter()
            elapsed = now - last
            if elapsed < dt:
                time.sleep(min(dt - elapsed, 0.005))
                continue
            last = now
            self.step(elapsed)

    def step(self, dt: float) -> None:
        """推进 dt 秒（测试里可以直接调）。"""
        with self._lock:
            for mid, pos in list(self._pos.items()):
                target = self._target[mid]
                speed = self._speed[mid]
                torque = self._torque[mid]
                diff = target - pos
                max_step = speed * dt
                if abs(diff) <= max_step:
                    vel = diff / dt if dt > 0 else 0.0
                    self._pos[mid] = target
                else:
                    vel = math.copysign(max_step / dt, diff) if dt > 0 else 0.0
                    self._pos[mid] = pos + math.copysign(max_step, diff)
                # 电流估算：空载跟随 + 负载 + 堵转
                cur = 0.04 + 0.0008 * abs(vel) + self.load * 0.25
                if abs(target - self._pos[mid]) > 1.0 and abs(vel) < 0.2 * max(speed, 1e-6):
                    cur += 0.35  # 走不动，接近堵转
                if mid in self._stalled:
                    cur = max(cur, torque)
                self._current[mid] = min(cur, torque) if torque > 0 else 0.0
                self._vel[mid] = abs(vel)
                if torque <= 0.05 and abs(diff) > 0.5:
                    self._pos[mid] = pos  # 没有力矩，原地不动
                    self._vel[mid] = 0.0

    # -- 测试辅助 ---------------------------------------------------------
    def settle(self, timeout: float = 5.0) -> bool:
        """等到所有电机到达目标（测试用）。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if all(abs(self._target[m] - self._pos[m]) < 0.05 for m in self._pos):
                    return True
            time.sleep(0.01)
        return False


class Player:
    """按 tracks 文件播放动作（与 LG-603「动作设置」语义一致：先下发点位再等延时）。"""

    def __init__(self, bus: HandBus, points_dir=None) -> None:
        self.bus = bus
        self.points_dir = points_dir or store.dirs().points

    def preview(self, track: store.Track) -> List[Dict[str, object]]:
        steps = store.load_track_steps(track, self.points_dir)
        t = 0
        timeline: List[Dict[str, object]] = []
        for loop in range(track.loops):
            for step in steps:
                timeline.append({
                    "loop": loop + 1,
                    "name": step["name"],
                    "delay_ms": step["delay_ms"],
                    "starts_at_ms": t,
                })
                t += int(step["delay_ms"])
        return timeline

    def play(
        self,
        track: store.Track,
        speed_scale: float = 1.0,
        stop_event: Optional[threading.Event] = None,
        on_step: Optional[Callable[[Dict[str, object]], None]] = None,
    ) -> Dict[str, object]:
        steps = store.load_track_steps(track, self.points_dir)
        played = 0
        started = time.time()
        for loop in range(track.loops):
            for step in steps:
                if stop_event and stop_event.is_set():
                    return {"ok": False, "stopped": True, "played": played,
                            "elapsed_ms": int((time.time() - started) * 1000)}
                self.bus.apply(step["pose"], reason=f"track:{track_name(track)}#{step['name']}")
                played += 1
                if on_step:
                    on_step({"loop": loop + 1, "name": step["name"], "delay_ms": step["delay_ms"]})
                delay = max(0.0, float(step["delay_ms"]) / 1000.0 / max(speed_scale, 1e-6))
                if delay:
                    end = time.time() + delay
                    while time.time() < end:
                        if stop_event and stop_event.is_set():
                            return {"ok": False, "stopped": True, "played": played,
                                    "elapsed_ms": int((time.time() - started) * 1000)}
                        time.sleep(min(0.02, end - time.time()))
        return {"ok": True, "played": played,
                "elapsed_ms": int((time.time() - started) * 1000),
                "duration_ms": track.duration_ms}


def track_name(track: store.Track) -> str:
    return "track"


def open_sim(**kwargs) -> SimBus:
    bus = SimBus(**kwargs)
    bus.open()
    return bus
