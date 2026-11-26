"""直接跟真手通话：读状态 / 小幅动作。绕过厂商 GUI。

用法（仓库根目录）：
    python scripts\\hand_live.py read                      # 只读，安全
    python scripts\\hand_live.py move --motor 103 --angle 10 --speed 30 --current 0.2
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import screw  # noqa: E402
from handlink.hardware import ZlgCan  # noqa: E402

DEFAULT_DLL = ROOT / "pkg" / "LG-603" / "src" / "main" / "resources" / "native" / "Windows-x86_64" / "ECanVci64.dll"


def detect_hand(bus: ZlgCan, seconds: float = 1.0) -> int:
    """听心跳推断 CAN ID（左手 0x65 / 右手 0x64，不同批次会变）。"""
    counts: dict[int, int] = {}
    deadline = time.time() + seconds
    while time.time() < deadline and not counts:
        for f in bus.read(max_frames=64, wait_ms=20):
            if f["data"][:1] == b"\x05":
                counts[f["id"]] = counts.get(f["id"], 0) + 1
    if not counts:
        return screw.HAND_ID
    return max(counts.items(), key=lambda kv: kv[1])[0]


def open_bus(args) -> ZlgCan:
    bus = ZlgCan(
        Path(args.dll),
        device_type=args.device_type,
        device_index=args.device_index,
        channel=args.channel,
        baud=1000,
    )
    if not getattr(args, "wait", 0):
        bus.open()
        return bus

    deadline = time.time() + args.wait
    print(f"等待适配器释放（最多 {args.wait}s）…拔插 USB 后会自动继续", flush=True)
    while True:
        try:
            bus.open()
            print("适配器已就绪", flush=True)
            return bus
        except RuntimeError:
            if time.time() > deadline:
                raise
            time.sleep(1.0)


def drain(bus: ZlgCan, seconds: float) -> list[dict]:
    out: list[dict] = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        out.extend(bus.read(max_frames=64, wait_ms=20))
    return out


def show(frames: list[dict]) -> None:
    for f in frames:
        parsed = screw.parse(f["data"])
        hexs = " ".join(f"{b:02X}" for b in f["data"])
        print(f"  {f['t']:.3f} id=0x{f['id']:03X} [{hexs:<26}]  {parsed}")


def cmd_read(args) -> int:
    bus = open_bus(args)
    if args.hand == 0:
        args.hand = detect_hand(bus)
    print(f"CAN 打开：device_type={args.device_type} ch={args.channel} 1000kbps  hand=0x{args.hand:02X}")
    try:
        print("\n-- 静默监听 0.5s（看心跳）--")
        show(drain(bus, 0.5))

        for motor in sorted(screw.MOTORS):
            for name, builder in (
                ("pos", screw.get_position),
                ("speed", screw.get_speed),
                ("current", screw.get_current),
            ):
                bus.send(args.hand, builder(motor))
                time.sleep(0.03)
            got = drain(bus, 0.35)
            vals = []
            for f in got:
                p = screw.parse(f["data"])
                if isinstance(p, screw.Frame) and p.value is not None and p.node == motor:
                    vals.append(f"{screw.FUNC_NAME.get(p.func, hex(p.func))}={p.value:.2f}")
            label = screw.MOTORS[motor]
            print(f"  {motor} {label:<11} {'  '.join(vals) if vals else '(无应答)'}")
    finally:
        bus.close()
    return 0


def cmd_move(args) -> int:
    motor = args.motor
    angle = screw.clamp_angle(motor, args.angle)
    current = screw.clamp_current(motor, args.current)
    speed = max(1.0, min(screw.MAX_SPEED, args.speed))
    print(f"目标：motor={motor} ({screw.MOTORS.get(motor, '?')}) angle={angle}° speed={speed} current={current}A")
    if angle != args.angle or current != args.current:
        print("  ！已被安全钳位")

    bus = open_bus(args)
    if args.hand == 0:
        args.hand = detect_hand(bus)
    try:
        print("1) 使能      ", bus.send(args.hand, screw.enable(motor, True)))
        time.sleep(0.05)
        print("2) 设速度    ", bus.send(args.hand, screw.set_speed(motor, speed)))
        time.sleep(0.05)
        print("3) 设电流    ", bus.send(args.hand, screw.set_current(motor, current)))
        time.sleep(0.05)
        print("4) 发位置    ", bus.send(args.hand, screw.set_position(motor, angle)))
        time.sleep(0.3)

        print("\n-- 回读位置 --")
        for _ in range(args.watch):
            bus.send(args.hand, screw.get_position(motor))
            time.sleep(0.05)
            show(drain(bus, 0.25))
    finally:
        bus.close()
    return 0


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["read", "move"])
    ap.add_argument("--dll", default=str(DEFAULT_DLL))
    ap.add_argument("--device-type", type=int, default=4)
    ap.add_argument("--device-index", type=int, default=0)
    ap.add_argument("--channel", type=int, default=0)
    ap.add_argument("--hand", type=lambda s: int(s, 0), default=0, help="0 = 自动识别")
    ap.add_argument("--motor", type=int, default=103)
    ap.add_argument("--angle", type=float, default=10.0)
    ap.add_argument("--speed", type=float, default=30.0)
    ap.add_argument("--current", type=float, default=0.2)
    ap.add_argument("--watch", type=int, default=5)
    ap.add_argument("--wait", type=float, default=0.0)
    args = ap.parse_args(argv[1:])
    return cmd_read(args) if args.mode == "read" else cmd_move(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
