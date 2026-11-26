"""真机上量一次：手指别动、只转手臂，看哪套换算不跟着转。

    python tools/glove_probe.py --seconds 20            # 边转边看
    python tools/glove_probe.py --selftest              # 不用手套，验一下这套分析本身

操作：手套戴好，手指张开保持不动，小臂（连着手腕）慢慢转一整圈——正转一圈再反转
一圈最好。工具绑 UDP 5001 收 MotionVenus 转发的帧，按 abs / rel / rel-ext 三种换算
各算一遍六指的原始弯折量，最后打一张"每根手指的波动范围"表。

判读：手指没动，所以波动全是"串进来的"。abs 是 1.1.x 的老算法（拇指那两栏会很大，
就是你看到的现象）；rel 是现在的默认；rel-ext 只是换了一种 ZXY 读法。哪套合计最小
就用哪套：POST /glove/frame {"frame":"rel"}，或 serve --glove-frame rel-ext。
"""
from __future__ import annotations

import argparse
import csv
import select
import socket
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import glove  # noqa: E402

PLANS = ("abs", "rel", "rel-ext")
NAMES = {m.motor_id: f"{m.name}" for m in glove.spec.MOTORS}


def groups_of(values: Sequence[float]) -> List[List[float]]:
    per = glove.PER
    return [list(values[i * per:(i + 1) * per]) for i in range(glove.BONES)]


def span(series: Sequence[float]) -> float:
    return (max(series) - min(series)) if series else 0.0


def report(frames: Sequence[Sequence[float]], side: str = "right") -> Dict[str, Dict[int, float]]:
    """每根手指在每种换算下的波动范围（度）。手指没动，理想值是 0。"""
    out: Dict[str, Dict[int, float]] = {p: {m: 0.0 for m in glove.FINGERS} for p in PLANS}
    if not frames:
        return out
    series = {p: {m: [] for m in glove.FINGERS} for p in PLANS}  # type: ignore[var-annotated]
    for values in frames:
        groups = groups_of(values)
        for plan in PLANS:
            try:
                raw = glove.chain_angles(groups, side, frame=plan)
            except ValueError:
                continue
            for motor, value in raw.items():
                series[plan][motor].append(value)
    for plan in PLANS:
        for motor in glove.FINGERS:
            out[plan][motor] = span(series[plan][motor])
    return out


def table(rows: Dict[str, Dict[int, float]], wrist_span: float) -> str:
    order = [102, 107, 103, 104, 105, 106]
    head = f"{'方案':<9}" + "".join(f"{NAMES.get(m, str(m)):>9}" for m in order) + f"{'合计':>9}"
    lines = [head, "-" * (9 * (len(order) + 2))]
    best = min(PLANS, key=lambda p: sum(rows[p].values()))
    for plan in PLANS:
        total = sum(rows[plan].values())
        mark = "  <- 用这个" if plan == best else ""
        lines.append(f"{plan:<9}" + "".join(f"{rows[plan][m]:>9.1f}" for m in order)
                     + f"{total:>9.1f}" + mark)
    lines.append("")
    lines.append(f"手腕这次总共转了 {wrist_span:.0f}°（手指没动的话，这整段都该被消掉）")
    return "\n".join(lines)


def wrist_span(frames: Sequence[Sequence[float]], side: str = "right") -> float:
    base = 0 if side != "left" else 16
    if not frames:
        return 0.0
    ys = [groups_of(v)[base][4] for v in frames if len(v) >= glove.BONES * glove.PER]
    zs = [groups_of(v)[base][5] for v in frames if len(v) >= glove.BONES * glove.PER]
    return max(span(ys), span(zs))


def selftest_frames() -> List[List[float]]:
    """合成一段"手指固定 20°、手臂滚 0~360°"的帧，用来验分析本身（不需要手套）。"""
    frames = []
    for roll in range(0, 361, 10):
        values = []
        for joint in range(glove.BONES):
            active = 0 <= joint < 16
            ey = 0.0 if not active else (roll if joint == 0 else 20.0 + roll)
            values += [0.0, 0.0, 0.0, 0.0, ey, 0.0]
        frames.append(values)
    return frames


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="量一下手臂转动对六指读数的影响")
    ap.add_argument("--udp", type=int, default=glove.DEFAULT_UDP)
    ap.add_argument("--bind", default=glove.DEFAULT_BIND)
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--csv", help="把原始帧另存一份 CSV（排查用）")
    ap.add_argument("--selftest", action="store_true", help="不接手套，用合成帧验一遍")
    args = ap.parse_args(argv)

    if args.selftest:
        frames = selftest_frames()
        print(table(report(frames, args.side), wrist_span(frames, args.side)))
        return 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        except OSError:
            pass
    try:
        sock.bind((args.bind, args.udp))
    except OSError as exc:
        print(f"UDP {args.bind}:{args.udp} 绑定失败：{exc}")
        print("端口被占着——先停掉 serve --glove 或 tools/glove_bridge.py")
        return 2
    sock.setblocking(False)

    asm = glove.Assembler()
    frames: List[List[float]] = []
    t0 = time.monotonic()
    last = t0
    print(f"收 UDP {args.bind}:{args.udp} · {args.side} 手 · {args.seconds:g}s")
    print("手指张开别动，小臂慢慢转一整圈（再反转回来）……")
    try:
        while time.monotonic() - t0 < args.seconds:
            ready, _, _ = select.select([sock], [], [], 0.1)
            if not ready:
                continue
            while True:
                try:
                    data, _ = sock.recvfrom(65535)
                except (BlockingIOError, InterruptedError):
                    break
                frame = asm.feed(data.decode("ascii", "ignore"))
                if frame is not None:
                    frames.append(frame)
            now = time.monotonic()
            if now - last >= 0.5:
                last = now
                deg = wrist_span(frames[-30:], args.side) if frames else 0.0
                print(f"\r  已收 {len(frames)} 帧 · 最近手腕摆动 {deg:3.0f}° · "
                      f"还剩 {max(0.0, args.seconds - (now - t0)):4.1f}s   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    print()

    if len(frames) < 10:
        print(f"只收到 {len(frames)} 帧，不够判断：看 MotionVenus 是否转发到 "
              f"{args.bind}:{args.udp}、左右手选对没有")
        return 2

    if args.csv:
        path = Path(args.csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["frame"] + [f"j{j}_{name}" for j in range(glove.BONES)
                                         for name in ("x", "y", "z", "ex", "ey", "ez")])
            for i, values in enumerate(frames):
                writer.writerow([i] + list(values))
        print(f"原始帧已存 {path}")

    print()
    print(table(report(frames, args.side), wrist_span(frames, args.side)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())