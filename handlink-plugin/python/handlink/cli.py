"""命令行入口：``python -m handlink <命令>``。"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from . import arm as arm_mod
from . import bus as bus_mod
from . import glove as glove_mod
from . import hardware, recorder, service, spec, store
from . import pick as pick_mod


def _table(rows: List[Dict[str, Any]], columns: List[str]) -> str:
    widths = {c: max([len(str(c))] + [len(str(r.get(c, ""))) for r in rows]) for c in columns}
    line = "  ".join(str(c).ljust(widths[c]) for c in columns)
    out = [line, "-" * len(line)]
    for r in rows:
        out.append("  ".join(str(r.get(c, "")).ljust(widths[c]) for c in columns))
    return "\n".join(out)


def _parse_assignments(items: List[str]) -> spec.Pose:
    """--motor 103=120[,speed=30][,torque=0.5]"""
    pose: spec.Pose = {}
    for item in items:
        mid_s, _, rest = item.partition("=")
        m = spec.resolve(mid_s)
        parts = [rest]
        if "," in rest:
            parts = rest.split(",")
        joint = spec.Joint(angle=float(parts[0]))
        for extra in parts[1:]:
            k, _, v = extra.partition("=")
            if k in ("speed", "速度"):
                joint.speed = float(v)
            elif k in ("torque", "current", "电流"):
                joint.torque = float(v)
            else:
                raise SystemExit(f"无法识别参数 {extra!r}，可用 angle,speed,torque")
        pose[m.motor_id] = joint
    return pose


def cmd_scan(args: argparse.Namespace) -> int:
    info = hardware.scan()
    print(json.dumps(info, ensure_ascii=False, indent=2))
    return 0


def cmd_limits(args: argparse.Namespace) -> int:
    rows = [{
        "手指": m.name, "ID": m.motor_id, "位置": f"0~{m.pos_max:.0f}°",
        "速度": f"0~{m.speed_max:.0f}", "电流": f"{m.current_max}A",
    } for m in spec.MOTORS]
    print(_table(rows, ["手指", "ID", "位置", "速度", "电流"]))
    return 0


def cmd_points(args: argparse.Namespace) -> int:
    d = store.dirs(args.root)
    points = store.list_points(d.points)
    print(f"点位目录: {d.points}")
    for name in points:
        pose = store.load_point(d.points / f"{name}.txt")
        angles = ", ".join(f"{spec.MOTOR_BY_ID[mid].name}:{pose[mid].angle:.0f}°" for mid in spec.MOTOR_IDS if mid in pose)
        print(f"  {name:<12} {angles}")
    if not points:
        print("  （空）")
    return 0


def cmd_pose(args: argparse.Namespace) -> int:
    sim = bus_mod.open_sim()
    app = service.App(sim, root=args.root)
    try:
        if args.point:
            pose = store.load_point(store.resolve_point(args.point, app.dirs.points))
        else:
            pose = _parse_assignments(args.motor)
        pose = spec.complete(pose, fallback=sim.pose())
        result = app.apply(pose, reason="cli:pose")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if not result["ok"]:
            return 2
        if args.wait:
            ok = sim.settle(timeout=args.wait)
            print("已到位" if ok else "超时未到位")
        rows = [{"手指": spec.MOTOR_BY_ID[mid].name, "指令角": f"{pose[mid].angle:.1f}°",
                 "实际角": f"{sim.state()[mid].angle:.1f}°",
                 "电流": f"{sim.state()[mid].current:.2f}A"} for mid in spec.MOTOR_IDS]
        print(_table(rows, ["手指", "指令角", "实际角", "电流"]))
    finally:
        sim.close()
    return 0


def cmd_track(args: argparse.Namespace) -> int:
    d = store.dirs(args.root)
    path = d.tracks / f"{args.name}.txt"
    if not path.is_file():
        print(f"动作不存在: {path}\n可用: {store.list_tracks(d.tracks)}", file=sys.stderr)
        return 2
    track = store.load_track(path)
    player = bus_mod.Player(bus_mod.SimBus(), d.points)
    if args.dry_run:
        print(json.dumps({"track": track.as_dict(), "timeline": player.preview(track),
                          "duration_ms": track.duration_ms}, ensure_ascii=False, indent=2))
        return 0
    sim = bus_mod.open_sim()
    player = bus_mod.Player(sim, d.points)
    try:
        result = player.play(track, speed_scale=args.scale,
                             on_step=lambda s: print(f"  [{s['loop']}] {s['name']} +{s['delay_ms']}ms"))
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        sim.close()
    return 0


def cmd_serve(args: argparse.Namespace) -> int:
    if getattr(args, "bus_name", "sim") == "can":
        from .canbus import CanBus

        hand_bus = CanBus(dll=args.dll or canbus_default_dll(),
                          hand_id=(int(args.hand, 0) if getattr(args, "hand", None) else None))
        hand_bus.open()
    else:
        hand_bus = bus_mod.open_sim()
    app = service.App(hand_bus, root=args.root, points_dir=args.points_dir)
    arm_ip = getattr(args, "arm", None)
    if arm_ip:
        if app.attach_arm(arm_ip):
            print(f"协作臂:   {arm_ip} 已连上（30004 反馈）")
        else:
            print(f"协作臂:   {arm_ip} 连不上 —— {app.arm_error}，三维视图退回仿真")
    print(f"演示页:   http://{args.host}:{args.port}/demo", flush=True)
    print(f"三维臂:   http://{args.host}:{args.port}/arm3d", flush=True)
    print(f"摄像头页: http://{args.host}:{args.port}/camera", flush=True)
    if getattr(args, "glove", False):
        band = {}
        for item in str(getattr(args, "glove_deadband", "") or "").split(","):
            if item.strip():
                key, _, deg = item.partition("=")
                band[key.strip()] = float(deg or 0)
        started = app.glove.start(udp=getattr(args, "glove_udp", glove_mod.DEFAULT_UDP),
                                  side=getattr(args, "glove_side", "right"),
                                  gain=getattr(args, "glove_gain", 1.0),
                                  frame=getattr(args, "glove_frame",
                                                glove_mod.DEFAULT_FRAME),
                                  deadband=band or None)
        if started.get("ok"):
            print(f"数据手套: UDP {started['bind']}:{started['udp']} 监听中"
                  f"（{started['side']}手，frame={started['frame']}，"
                  f"gain={started['gain']}）", flush=True)
        else:
            print(f"数据手套: 启动失败 —— {started.get('error')}"
                  f"（{started.get('hint', '')}）", flush=True)
    try:
        service.serve(app, args.host, args.port)
    finally:
        app.camera.close()
        app.close_arm()
        hand_bus.close()
    return 0


def canbus_default_dll():
    from .canbus import DEFAULT_DLL

    return DEFAULT_DLL


def _parse6(text: str) -> List[float]:
    """``400,0,300,180,0,0`` → [400.0, 0.0, ...]"""
    values = [float(x) for x in text.replace("，", ",").split(",") if x.strip()]
    if len(values) != 6:
        raise SystemExit(f"需要 6 个数 (x,y,z,rx,ry,rz)，收到 {len(values)} 个: {text!r}")
    return values


def _open_hand(args: argparse.Namespace) -> bus_mod.HandBus:
    if getattr(args, "bus_name", "sim") == "can":
        from .canbus import CanBus

        hand = CanBus(dll=args.dll or canbus_default_dll(),
                      hand_id=(int(args.hand, 0) if getattr(args, "hand", None) else None))
        hand.open()
        return hand
    return bus_mod.open_sim()


def cmd_arm(args: argparse.Namespace) -> int:
    if args.ip:
        machine: arm_mod.Arm = arm_mod.DobotArm(ip=args.ip, port=args.port)
    else:
        machine = arm_mod.SimArm()
    machine.connect()
    if not args.ip:
        machine.enable()  # 仿真臂没有真机风险，直接用
    try:
        if args.check:
            payload = machine.check() if hasattr(machine, "check") else machine.status()
            print(json.dumps(payload, ensure_ascii=False, indent=2))
        if args.enable:
            machine.enable()
            print(f"已使能，速度因子 {machine.speed_factor(args.speed):.0f}%")
        if args.stop:
            machine.stop()
            print("已停止（就地定住）")
        if args.pose:
            target = _parse6(args.pose)
            result = machine.move_l(target) if args.linear else machine.move_j(target)
            print(json.dumps(result, ensure_ascii=False, indent=2))
        if not (args.check or args.enable or args.stop or args.pose):
            print(json.dumps(machine.status(), ensure_ascii=False, indent=2))
    except arm_mod.ArmError as exc:
        print(f"臂错误: {exc}", file=sys.stderr)
        return 2
    finally:
        machine.close()
    return 0


def cmd_pick(args: argparse.Namespace) -> int:
    plan = pick_mod.PickPlan(
        home=_parse6(args.home), pre=_parse6(args.pre),
        grasp=_parse6(args.grasp), lift=_parse6(args.lift),
        speed=args.speed, force=args.force, hold=args.hold, timeout=args.timeout,
    )
    if args.preview:
        print(json.dumps(pick_mod.preview(plan), ensure_ascii=False, indent=2))
        return 0

    if args.ip:
        machine: arm_mod.Arm = arm_mod.DobotArm(ip=args.ip, port=args.port)
        machine.connect()
        hand = _open_hand(args)
    else:
        machine = arm_mod.SimArm()
        machine.connect()
        hand = bus_mod.open_sim()
    try:
        result = pick_mod.run(machine, hand, plan)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except (pick_mod.PickError, arm_mod.ArmError) as exc:
        print(f"中止: {exc}", file=sys.stderr)
        return 2
    finally:
        machine.close()
        hand.close()


def cmd_record(args: argparse.Namespace) -> int:
    sim = bus_mod.open_sim()
    app = service.App(sim, root=args.root)
    out = Path(args.out)
    app.start_recording(str(out), hz=args.hz)
    try:
        if args.point:
            app.apply_point(args.point, reason="cli:record")
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            time.sleep(0.05)
    finally:
        result = app.stop_recording()
        sim.close()
    print(recorder.to_markdown(result["stats"]))
    return 0


def cmd_analyze(args: argparse.Namespace) -> int:
    stats = recorder.analyze(Path(args.file))
    if args.markdown:
        print(recorder.to_markdown(stats))
    else:
        print(json.dumps(stats, ensure_ascii=False, indent=2))
    return 0


def cmd_ai(args: argparse.Namespace) -> int:
    text = " ".join(args.text)
    sim = bus_mod.open_sim()
    app = service.App(sim, root=args.root)
    try:
        result = app.ai_run(text, dry_run=args.dry_run, prefer_llm=args.llm,
                            speed_scale=args.scale)
        print("计划:", result.get("explain", ""))
        if result.get("plan", {}).get("source"):
            print("来源:", result["plan"]["source"])
        print(json.dumps({k: v for k, v in result.items() if k not in ("plan", "explain", "log")},
                         ensure_ascii=False, indent=2))
        return 0 if result.get("ok") else 2
    finally:
        sim.close()


def cmd_canmon(args: argparse.Namespace) -> int:
    dll = Path(args.dll) if args.dll else hardware.find_can_dll()
    if not dll or not dll.is_file():
        print("没找到 ECanVci64.dll，用 --dll 指定", file=sys.stderr)
        return 2
    can = hardware.ZlgCan(dll, device_type=args.device_type, channel=args.channel, baud=args.baud)
    can.open()
    print(f"旁听 CAN 通道 {args.channel} @ {args.baud}kbps，Ctrl+C 结束")
    seen: Dict[int, int] = {}
    try:
        deadline = time.time() + args.seconds if args.seconds else None
        while deadline is None or time.time() < deadline:
            for f in can.read(max_frames=64, wait_ms=20):
                seen[f["id"]] = seen.get(f["id"], 0) + 1
                if args.verbose:
                    print(f"{f['id']:>10} {f['data'].hex(' ')}")
    except KeyboardInterrupt:
        pass
    finally:
        can.close()
    print(json.dumps({f"0x{k:X}": v for k, v in sorted(seen.items())}, indent=2))
    return 0


def cmd_mic(args: argparse.Namespace) -> int:
    """不连服务也能验麦：列设备 / 试录一段 / 录到文件。"""
    from . import audio as audio_mod

    if args.list or (not args.test and not args.out):
        devices = audio_mod.list_inputs()
        if not devices:
            print("没有录音设备（或这个系统不是 Windows）。")
            return 1
        for d in devices:
            print(f"  #{d.index}  {d.name}  ({d.channels} ch)")
        print("\n试录: python -m handlink mic --test --seconds 2")
        print("录文件: python -m handlink mic --out recordings/a.wav --seconds 5")
        return 0

    rec = audio_mod.MicRecorder()
    out = Path(args.out) if args.out else None
    try:
        if args.test:
            info = rec.test(index=args.index, seconds=args.seconds, out=out)
            print(json.dumps(info, ensure_ascii=False, indent=2))
            return 0 if info.get("ok") else 1
        started = rec.start(index=args.index, out=out, max_seconds=args.seconds)
        print(f"录音中：{started['path']}（最长 {started['max_seconds']} 秒，Ctrl+C 结束）")
        try:
            while rec.running:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        info = rec.stop()
        print(json.dumps(info, ensure_ascii=False, indent=2))
        return 0 if info.get("ok") else 1
    except RuntimeError as exc:
        print(f"失败：{exc}")
        return 1


def cmd_selftest(args: argparse.Namespace) -> int:
    import unittest

    from pathlib import Path as _P

    here = _P(__file__).resolve().parent.parent
    loader = unittest.TestLoader()
    suite = loader.discover(str(here / "tests"), pattern="test_*.py")
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    return 0 if result.wasSuccessful() else 1


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="handlink", description="灵巧手连接器（离线仿真 + 真机接入）")
    p.add_argument("--root", help="LG-603 软件包根目录（含 points/tracks）")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("scan", help="探测串口 / CAN / 服务 / 软件包")
    s.set_defaults(func=cmd_scan)

    s = sub.add_parser("limits", help="打印电机限位表")
    s.set_defaults(func=cmd_limits)

    s = sub.add_parser("points", help="列出现有点位")
    s.set_defaults(func=cmd_points)

    s = sub.add_parser("pose", help="下发姿态（默认走仿真）")
    s.add_argument("--motor", action="append", default=[], help="103=120,speed=60,torque=0.5")
    s.add_argument("--point", help="直接套用已有点位")
    s.add_argument("--wait", type=float, default=2.0, help="等待到位秒数")
    s.set_defaults(func=cmd_pose)

    s = sub.add_parser("track", help="播放动作")
    s.add_argument("name")
    s.add_argument("--dry-run", action="store_true", help="只打印时间线，不动电机")
    s.add_argument("--scale", type=float, default=1.0, help="速度倍率（越大越快）")
    s.set_defaults(func=cmd_track)

    s = sub.add_parser("serve", help="启动 HTTP 服务")
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=service.DEFAULT_PORT,
                   help=f"HTTP 端口（默认 {service.DEFAULT_PORT}）")
    s.add_argument("--points-dir")
    s.add_argument("--bus", dest="bus_name", choices=["sim", "can"], default="sim",
                   help="sim=仿真（默认），can=真机")
    s.add_argument("--dll", help="ECanVci64.dll 路径（can 模式）")
    s.add_argument("--hand", help="手部 CAN ID，如 0x65；留空自动识别")
    s.add_argument("--arm", metavar="IP",
                   help="协作臂 IP（如 192.168.5.1）；给了就连 30004，没接则三维视图用仿真")
    s.add_argument("--glove", action="store_true",
                   help="同时收 MotionVenus 数据手套（UDP 5001）")
    s.add_argument("--glove-udp", type=int, default=glove_mod.DEFAULT_UDP)
    s.add_argument("--glove-side", choices=["right", "left"], default="right")
    s.add_argument("--glove-gain", type=float, default=1.0)
    s.add_argument("--glove-frame", choices=list(glove_mod.FRAMES),
                   default=glove_mod.DEFAULT_FRAME,
                   help="rel=减掉手腕（默认）；abs=老算法对照；rel-ext=另一种 ZXY 读法")
    s.add_argument("--glove-deadband", default="", metavar="102=4,107=6",
                   help="给指定电机加死区（度）")
    s.set_defaults(func=cmd_serve)

    s = sub.add_parser("record", help="录制一段数据并出分析")
    s.add_argument("--out", required=True)
    s.add_argument("--seconds", type=float, default=3.0)
    s.add_argument("--hz", type=float, default=20.0)
    s.add_argument("--point")
    s.set_defaults(func=cmd_record)

    s = sub.add_parser("analyze", help="分析录制文件")
    s.add_argument("file")
    s.add_argument("--markdown", action="store_true")
    s.set_defaults(func=cmd_analyze)

    s = sub.add_parser("ai", help="自然语言控制：规则解析或大模型规划")
    s.add_argument("text", nargs="+", help="例如：先张开，再握拳，然后张开")
    s.add_argument("--dry-run", action="store_true", help="只出计划，不动电机")
    s.add_argument("--llm", action="store_true", help="用大模型规划（需 HANDLINK_LLM_KEY）")
    s.add_argument("--scale", type=float, default=1.0, help="时间倍率")
    s.set_defaults(func=cmd_ai)

    s = sub.add_parser("canmon", help="CAN 旁听（只读）")
    s.add_argument("--dll")
    s.add_argument("--device-type", type=int, default=21)
    s.add_argument("--channel", type=int, default=1)
    s.add_argument("--baud", type=int, default=1000)
    s.add_argument("--seconds", type=float, default=0)
    s.add_argument("--verbose", action="store_true")
    s.set_defaults(func=cmd_canmon)

    s = sub.add_parser("arm", help="协作臂（Dobot Nova 5）：只读自检 / 使能 / 移动")
    s.add_argument("--ip", help="臂 IP，默认 192.168.5.1；不填则用仿真臂")
    s.add_argument("--port", type=int, default=arm_mod.CONTROL_PORT)
    s.add_argument("--check", action="store_true", help="只读自检：RobotMode + GetPose + GetAngle")
    s.add_argument("--enable", action="store_true")
    s.add_argument("--stop", action="store_true", help="就地定住")
    s.add_argument("--pose", help="目标位姿 x,y,z,rx,ry,rz")
    s.add_argument("--linear", action="store_true", help="用 MovL（默认 MovJ）")
    s.add_argument("--speed", type=float, default=arm_mod.SAFE_SPEED_FACTOR,
                   help=f"速度因子 %%，首次调试别超过 {arm_mod.SAFE_SPEED_FACTOR:.0f}")
    s.set_defaults(func=cmd_arm)

    s = sub.add_parser("pick", help="抓取流程：到位 → 合手 → 抬升（默认仿真）")
    s.add_argument("--ip", help="臂 IP；不填则整条流程跑仿真")
    s.add_argument("--port", type=int, default=arm_mod.CONTROL_PORT)
    s.add_argument("--preview", action="store_true", help="只算距离和耗时，不动")
    s.add_argument("--home", default=",".join(f"{v:g}" for v in pick_mod.DEFAULT_HOME))
    s.add_argument("--pre", default=",".join(f"{v:g}" for v in pick_mod.DEFAULT_PRE))
    s.add_argument("--grasp", default=",".join(f"{v:g}" for v in pick_mod.DEFAULT_GRASP))
    s.add_argument("--lift", default=",".join(f"{v:g}" for v in pick_mod.DEFAULT_LIFT))
    s.add_argument("--speed", type=float, default=arm_mod.SAFE_SPEED_FACTOR)
    s.add_argument("--force", type=float, default=1.0, help="合手电流上限（A），低于 1.0 会卡")
    s.add_argument("--hold", type=float, default=0.3)
    s.add_argument("--timeout", type=float, default=30.0)
    s.add_argument("--bus", dest="bus_name", choices=["sim", "can"], default="sim",
                   help="sim=仿真手（默认），can=真手")
    s.add_argument("--dll")
    s.add_argument("--hand")
    s.set_defaults(func=cmd_pick)

    s = sub.add_parser("mic", help="麦克风：列设备 / 试录 / 录 WAV（不连服务）")
    s.add_argument("--list", action="store_true", help="只列设备（默认）")
    s.add_argument("--test", action="store_true", help="试录一小段，看看响不响")
    s.add_argument("--out", help="录到哪个 WAV（--test 时也会保存）")
    s.add_argument("--seconds", type=float, default=2.0, help="录多久（默认 2 秒）")
    s.add_argument("--index", type=int, default=0, help="设备号（见 --list）")
    s.set_defaults(func=cmd_mic)

    s = sub.add_parser("selftest", help="跑内置测试")
    s.set_defaults(func=cmd_selftest)

    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
