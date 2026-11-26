"""单手推层实验：夹住中/下层，食指把 U 层推 90°。

先看姿态、不动机器::

    python demo/cube_push.py --bus sim --dry-run

仿真里跑一遍（确认录制/分析链路）::

    python demo/cube_push.py --bus sim --cycles 2

真机（第一次务必低电流低速，手边有人，急停在手边）::

    python demo/cube_push.py --bus can --speed 40 --current 0.6 --index-end 110 --cycles 3

注意：
* 真机第一次先把魔方放到托盘里、手退到安全位，再跑；出现卡滞/异响立刻停。
* 这个脚本只动手指，不动臂；臂的 PUSH_READY 位姿要先用示教/`GetPose()` 摆好。
* 角度是起点，不是标准答案：按你的托盘高度、魔方大小、指垫厚度微调。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from handlink import spec  # noqa: E402
from handlink.bus import SimBus  # noqa: E402
from handlink.recorder import Recorder, Sampler, analyze, to_markdown  # noqa: E402

# 夹持角：食指自由，其余指夹住中/下层。来源是 poses.py 的 point 变体，
# 必须按实物微调（尤其拇指：0.8 A 上限，别夹住要被推的 U 层）。
HOLD = {
    102: (48.0, 0.8),   # 拇指1
    107: (34.0, 0.8),   # 拇指2
    104: (132.0, 1.0),  # 中指
    105: (142.0, 1.0),  # 无名指
    106: (142.0, 0.8),  # 小指
}

INDEX = 103


def build_pose(index_angle: float, speed: float, index_current: float) -> spec.Pose:
    pose: spec.Pose = {}
    for mid, (angle, cur) in HOLD.items():
        m = spec.MOTOR_BY_ID[mid]
        pose[mid] = spec.Joint(angle, speed, min(cur, m.current_max))
    m = spec.MOTOR_BY_ID[INDEX]
    pose[INDEX] = spec.Joint(index_angle, speed, min(index_current, m.current_max))
    return pose


def print_pose(title: str, pose: spec.Pose) -> None:
    print(f"\n{title}")
    print(f"  {'手指':<8}{'ID':>4}{'角度':>8}{'速度':>7}{'电流':>7}")
    for mid in spec.MOTOR_IDS:
        j = pose.get(mid)
        name = spec.MOTOR_BY_ID[mid].name
        if j is None:
            continue
        print(f"  {name:<8}{mid:>4}{j.angle:>7.1f}°{j.speed:>7.0f}{j.torque:>6.2f}A")


def print_state(bus) -> None:
    st = bus.state()
    print(f"  {'手指':<8}{'角度':>8}{'电流':>8}{'上限':>7}")
    for mid in spec.MOTOR_IDS:
        s = st[mid]
        limit = spec.MOTOR_BY_ID[mid].current_max
        print(f"  {spec.MOTOR_BY_ID[mid].name:<8}{s.angle:>7.1f}°{s.current:>7.2f}A{limit:>6.1f}A")


def main() -> int:
    ap = argparse.ArgumentParser(description="食指推 U 层 90° 的实验台")
    ap.add_argument("--bus", choices=("sim", "can"), default="sim", help="sim=仿真，can=真机")
    ap.add_argument("--index-start", type=float, default=0.0, help="食指起始角（度）")
    ap.add_argument("--index-end", type=float, default=120.0, help="食指推到位角（度）")
    ap.add_argument("--speed", type=float, default=60.0, help="速度（0–360，200≈55°/s）")
    ap.add_argument("--current", type=float, default=0.8, help="食指电流上限（A）")
    ap.add_argument("--cycles", type=int, default=3, help="推-退循环次数")
    ap.add_argument("--settle", type=float, default=1.5, help="夹好后的等待（s）")
    ap.add_argument("--push-time", type=float, default=2.0, help="推到位的等待（s）")
    ap.add_argument("--retract-time", type=float, default=1.0, help="退回后的等待（s）")
    ap.add_argument("--out", type=Path, default=None, help="录制 JSONL 路径")
    ap.add_argument("--dry-run", action="store_true", help="只打印姿态，不连接/不动机器")
    args = ap.parse_args()

    ready = build_pose(args.index_start, args.speed, min(args.current, 1.0))
    push = build_pose(args.index_end, args.speed, args.current)

    print_pose("① 夹持 + 食指就位（ready）", ready)
    print_pose("② 食指推到位（push）", push)

    issues = spec.check(push)
    errors = [i for i in issues if i.get("level") == "error"]
    warnings = [i for i in issues if i.get("level") != "error"]
    for item in warnings:
        print(f"  ! {item.get('message')}")
    if errors:
        for item in errors:
            print(f"  ✗ {item.get('message')}")
        print("\n姿态越界，已拒绝下发。")
        return 2

    print("\n时间线: "
          f"ready {args.settle:.1f}s → push {args.push_time:.1f}s → "
          f"retract {args.retract_time:.1f}s × {args.cycles} 次")

    if args.dry_run:
        print("\n--dry-run：没有连接总线、没有动手。")
        return 0

    out = args.out or Path("demo") / f"cube_push-{time.strftime('%H%M%S')}.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)

    if args.bus == "can":
        from handlink.canbus import CanBus  # noqa: WPS433
        bus = CanBus()
    else:
        bus = SimBus()

    rec = Recorder(out, meta={"script": "cube_push", "args": vars(args) | {"out": str(out)}})
    sampler = None
    stopped = False
    try:
        bus.open()
        sampler = Sampler(bus, rec, hz=20).start()

        print("\n[1] 夹持就位（只合夹持指，食指在起始角）")
        rec.cmd(ready, reason="cube_push:ready")
        bus.apply(ready, reason="cube_push:ready")
        time.sleep(args.settle)
        print_state(bus)

        for i in range(1, args.cycles + 1):
            print(f"\n[2.{i}] 食指推：{args.index_start:.0f}° → {args.index_end:.0f}°")
            rec.cmd(push, reason=f"cube_push:push#{i}")
            bus.apply(push, reason=f"cube_push:push#{i}")
            time.sleep(args.push_time)
            print_state(bus)

            print(f"[3.{i}] 食指退回：{args.index_end:.0f}° → {args.index_start:.0f}°")
            rec.cmd(ready, reason=f"cube_push:retract#{i}")
            bus.apply(ready, reason=f"cube_push:retract#{i}")
            time.sleep(args.retract_time)
            print_state(bus)

    except KeyboardInterrupt:
        stopped = True
        print("\n收到 Ctrl+C，停手。")
    finally:
        if sampler is not None:
            sampler.stop()
        try:
            if stopped:
                bus.stop()
        except Exception as exc:  # noqa: BLE001
            print(f"  stop 失败: {exc}")
        rec.close()
        try:
            bus.close()
        except Exception:  # noqa: BLE001
            pass

    stats = analyze(out)
    print("\n" + to_markdown(stats))
    print(f"\n录制文件: {out}")
    print("下一步：用相机/记号量 U 层实际转了多少度，填 docs/魔方单手方案.md §3.4 的表。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
