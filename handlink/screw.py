"""LG-603 丝杆手 CAN 协议（逆向自厂商 cable-driven.jar + JNA 封装）。

报文结构（经典 CAN，标准帧，ID = 手部 ID，默认 0x64）：

    主机 -> 电机 :  [func][nodeId][payload...]
    电机 -> 主机 :  [func][handId][nodeId][float32 LE @3]   (7 字节)
    心跳         :  [0x05][nodeId][count][calibStatus][runningStatus]  (5 字节)

func 码由 ScrewCommandUtils.buildXxxFrame 反汇编得到；参考值见 :data:`CMD`。
float 一律 little-endian（源码里显式 `order(ByteOrder.LITTLE_ENDIAN)`）。
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

HAND_ID = 0x64  # 左手 CAN ID（厂商日志：创建丝杆手: 左手 (手部ID=64)）

CMD = {
    "heartbeat": 0x05,
    "reset": 0x0A,
    "clear_error": 0x0E,
    "calib_switch": 0xA0,
    "pos_set": 0xA2,
    "pos_get": 0xA4,
    "speed_get": 0xA6,
    "current_get": 0xA8,
    "calib_current": 0xB0,
    "calib_offset": 0xC0,
    "current_set": 0xD0,
    "speed_set": 0xE0,
    "encoder_zero": 0xF0,
    "enable": 0xF2,
}

FUNC_NAME = {v: k for k, v in CMD.items()}

MOTORS = {
    102: "thumb-base",
    103: "index",
    104: "middle",
    105: "ring",
    106: "pinky",
    107: "thumb-tip",
}

# 每个关节的行程（度），来自 MotorConfig / 厂商 limits
LIMITS = {
    102: (0.0, 100.0),
    107: (0.0, 45.0),
    103: (0.0, 180.0),
    104: (0.0, 180.0),
    105: (0.0, 180.0),
    106: (0.0, 180.0),
}

MAX_SPEED = 360.0
MAX_CURRENT = 3.5
THUMB_MAX_CURRENT = 0.8


# ------------------------------------------------------------------ builders
def _f32(value: float) -> bytes:
    return struct.pack("<f", float(value))


def set_position(motor: int, angle: float) -> bytes:
    return bytes([CMD["pos_set"], motor]) + _f32(angle)


def set_speed(motor: int, speed: float) -> bytes:
    return bytes([CMD["speed_set"], motor]) + _f32(speed)


def set_current(motor: int, current: float) -> bytes:
    return bytes([CMD["current_set"], motor]) + _f32(current)


def get_position(motor: int) -> bytes:
    return bytes([CMD["pos_get"], motor])


def get_speed(motor: int) -> bytes:
    return bytes([CMD["speed_get"], motor])


def get_current(motor: int) -> bytes:
    return bytes([CMD["current_get"], motor])


def enable(motor: int, on: bool = True) -> bytes:
    return bytes([CMD["enable"], motor, 1 if on else 0])


def clear_error(motor: int) -> bytes:
    return bytes([CMD["clear_error"], motor])


def reset(motor: int) -> bytes:
    return bytes([CMD["reset"], motor, 1])


def clamp_angle(motor: int, angle: float) -> float:
    lo, hi = LIMITS.get(motor, (0.0, 180.0))
    return max(lo, min(hi, float(angle)))


def clamp_current(motor: int, current: float) -> float:
    cap = THUMB_MAX_CURRENT if motor in (102, 107) else MAX_CURRENT
    return max(0.0, min(cap, float(current)))


# ------------------------------------------------------------------ parsing
@dataclass
class Frame:
    func: int
    node: int
    seq: int | None = None
    value: float | None = None
    hand: int | None = None
    raw: bytes = b""

    def __str__(self) -> str:
        name = FUNC_NAME.get(self.func & 0xFE, FUNC_NAME.get(self.func, f"0x{self.func:02X}"))
        val = "" if self.value is None else f" value={self.value:.3f}"
        return f"{name}/{self.func:02X} node={self.node}{val}"


@dataclass
class Heartbeat:
    node: int
    count: int
    calib: int
    running: int

    @property
    def state(self) -> str:
        bits = []
        if self.running & 0x01:
            bits.append("running")
        if self.running & 0x02:
            bits.append("HW-OC")
        if self.running & 0x04:
            bits.append("SW-OC")
        if self.running & 0x08:
            bits.append("V-ERR")
        if self.running & 0x10:
            bits.append("I-LIM")
        if self.running & 0x20:
            bits.append("POS-LIM")
        if self.running & 0x40:
            bits.append("SPD-LIM")
        return "|".join(bits) or "idle"


def parse(data: bytes) -> Frame | Heartbeat | None:
    """解析电机回传。

    实测：`a4 67` 查询回 `a5 67 06 ff eb ff 3e`
        = [功能码+1][节点][计数][float32 LE @3]
    """
    if not data:
        return None
    func = data[0]
    if func == CMD["heartbeat"] and len(data) == 5:
        return Heartbeat(node=data[1], count=data[2], calib=data[3], running=data[4])
    if len(data) >= 7:
        return Frame(
            func=func,
            node=data[1],
            seq=data[2],
            value=struct.unpack("<f", data[3:7])[0],
            raw=bytes(data),
        )
    if len(data) >= 2:
        return Frame(func=func, node=data[1], raw=bytes(data))
    return None
