"""定位之后，用一条**检查过**的路径把盒子抓起来。

和 ``pick.py`` 的区别
--------------------
``pick.py`` 是早期跑通「臂 + 手」两条链路的骨架，点位写死、不查碰撞，抓取高度
是 120 mm（法兰到桌面 120 mm 时，灵巧手 185 mm 的指尖已经戳进桌子里了）。
这个模块是给真实工位用的：

1. **目标从场景来**：``scene.objects()`` 里相机/手动放进去的盒子，带位置、长宽高、
   朝向；也可以命令行直接给。
2. **抓取高度按手的尺寸算**：法兰到指尖 185 mm（``cell.CELL["tool"]`` 的包络），
   指尖停在盒子上沿附近，手指从两侧围下去，不是拿法兰去撞桌子。
3. **每段路径都查**：用臂自己的反解（真机走控制器 ``InverseKin``，仿真走
   ``kin.ik``）把每段折成关节角，再 ``cell.check_path()`` 扫一遍最小间距；
   被挡就直接拒绝，不动。
4. **失败就停**：手没合上、路径不安全、反解不出来，都在动作前报错。

安全边界
--------
* 只抓桌面上的、尺寸在手能张开范围里的东西（默认 ≤ 120 mm）；
* 目标本身从障碍物表里拿掉（我们就是要碰它），其余障碍一个都不放过；
* 安全间距沿用 ``cell.SAFE_MM``（40 mm），低于就拒绝；
* 速度固定 20%，不做「快了再说」。

用法
----
    python -m handlink.pickbox --dry-run --x 0.15 --y -0.05 --h 0.06
    python -m handlink.pickbox --dry-run --from-scene
    python -m handlink.pickbox --sim --x 0.15 --y -0.05 --h 0.06
    python -m handlink.pickbox --go --from-scene --arm 192.168.5.1 --yes
"""
from __future__ import annotations

import argparse
import copy
import math
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import arm as arm_mod
from . import cell as cell_mod
from . import kin
from . import poses

#: 法兰到指尖的距离（米）。2026-11-18 实测：臂从 z=286 降 z=220 时在 z≈247
#: 停住顶在桌面上，用户用尺量「约 240」。取实测接触点 247 mm。
#:
#: 以前写 0.185（``cell.CELL["tool"]["capsules"]`` 的模型包络），比真实短 62 mm；
#: 按 0.185 算抓取高度会把工具多往下压 62 mm，直接犁到桌面。
TOOL_TIP_M = 0.247

#: 抓取时指尖停在盒子上沿下方多少（米）。让手指从两侧抱住上沿，不是压顶。
GRASP_INSET_M = 0.01

#: 预抓取点比盒子顶面高多少（米）。
PRE_CLEAR_M = 0.06

#: 抬升高度（米）。
LIFT_M = 0.05

#: 手能张开的最大尺寸（米）。超过就别抓，换更大的手或换目标。
MAX_TARGET_M = 0.12

#: 速度因子（%）。抓取阶段不追求快。
SPEED_PCT = 20.0

#: 仿真/试算的起点：2026-11-15 真机实测的一组位姿（见 README 的调试记录）。
SIM_HOME_POSE = (-98.1, 240.9, 516.5, 117.9, -61.4, -112.7)
SIM_HOME_JOINTS = (-120.5, -25.0, 107.5, 74.5, -35.0, 83.5)


class PickPlanError(RuntimeError):
    """规划阶段就否掉（不可达、被挡、尺寸不合适）。"""


@dataclass
class GraspTarget:
    """要抓的东西：桌面位置 + 长宽高 + 朝向。"""

    name: str
    x: float                  # 米
    y: float                  # 米
    w: float                  # 米（长边方向）
    d: float                  # 米
    h: float                  # 米
    yaw: float = 0.0          # 度，绕 Z
    source: str = "scene"


@dataclass
class PickPlan:
    target: GraspTarget
    home: List[float]          # 当前法兰位姿（起点）
    transit: List[float]       # 过渡点：目标正上方 450 mm，先抬高再平移
    pre: List[float]           # 预抓取（盒子正上方 +6 cm）
    grasp: List[float]         # 抓取（指尖到盒子上沿附近）
    lift: List[float]          # 抬升（预抓取点再高 5 cm）
    speed: float = SPEED_PCT
    hand_yaw: float = 0.0      # 手相对盒子的附加转角（度）


def target_from_objects(objs: Sequence[Dict[str, Any]],
                        name: Optional[str] = None) -> GraspTarget:
    """从物体表里挑一个盒子（服务返回的 /scene/state 或本地 Scene 都行）。"""
    objs = list(objs or [])
    if not objs:
        raise PickPlanError("场景里没有物体：先用相机扫描，或 /scene/place 手动放一个")
    pick = None
    if name:
        for o in objs:
            if name in str(o.get("name")) or str(o.get("name")) in name:
                pick = o
                break
        if pick is None:
            raise PickPlanError(f"场景里没有叫「{name}」的物体：{[o['name'] for o in objs]}")
    else:
        for o in objs:
            if any(k in str(o.get("name")) for k in ("盒", "魔方", "box")):
                pick = o
                break
        if pick is None:                       # 没有明确名字，就拿最大的那个
            pick = max(objs, key=lambda o: float(o["size"][0]) * float(o["size"][1]))
    size = [float(v) for v in pick["size"]]
    if len(size) == 2:
        size = [size[0], size[-1], size[-1]]
    if pick.get("kind") == "cylinder":
        size = [size[0] * 2.0, size[0] * 2.0, size[1]]
    return GraspTarget(name=str(pick.get("name") or "盒子"),
                       x=float(pick["center"][0]), y=float(pick["center"][1]),
                       w=size[0], d=size[1], h=size[2],
                       yaw=float(pick.get("yaw") or 0.0), source=str(pick.get("source")))


def target_from_scene(scene, name: Optional[str] = None) -> GraspTarget:
    """本地 ``Scene`` 实例（测试/脚本用）。"""
    return target_from_objects(scene.objects(), name)


def target_from_service(url: str, name: Optional[str] = None,
                        timeout: float = 4.0) -> GraspTarget:
    """从正在跑的 handlink 服务读 ``/scene/state``，取相机/手动放的物体。

    命令行和网页是两个进程，``Scene`` 里的物体只在服务进程内存里；不查服务的话
    ``--from-scene`` 永远拿到空表。
    """
    import json
    import urllib.request

    base = url.rstrip("/")
    try:
        with urllib.request.urlopen(base + "/scene/state", timeout=timeout) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except Exception as exc:                  # noqa: BLE001
        raise PickPlanError(f"连不上场景服务 {base}（{exc}）；先把 `python -m handlink serve` 起起来")
    if not data.get("calibrated"):
        raise PickPlanError("服务那边桌面还没标定（先 /vision 点桌面四角），位置不可信")
    return target_from_objects(data.get("objects") or [], name)


def check_expected(t: GraspTarget, expect: Sequence[float], tol: float = 0.02) -> None:
    """确认位置：量到的目标必须离「你以为的位置」足够近，否则拒绝抓。"""
    dx = t.x - float(expect[0])
    dy = t.y - float(expect[1])
    d = math.hypot(dx, dy)
    if d > tol:
        raise PickPlanError(
            f"位置对不上：量到 ({t.x * 1000:.0f}, {t.y * 1000:.0f}) mm，"
            f"预期 ({float(expect[0]) * 1000:.0f}, {float(expect[1]) * 1000:.0f}) mm，"
            f"差 {d * 1000:.0f} mm > {tol * 1000:.0f} mm。先确认桌面/相机没动")


def _target_checks(t: GraspTarget) -> None:
    if t.x ** 2 + t.y ** 2 > 0.85 ** 2:
        raise PickPlanError(f"目标 ({t.x:.3f}, {t.y:.3f}) 超出臂展 850 mm")
    if max(t.w, t.d) > MAX_TARGET_M:
        raise PickPlanError(f"目标太大（{max(t.w, t.d) * 1000:.0f} mm > "
                            f"{MAX_TARGET_M * 1000:.0f} mm），手张不了这么开")
    if t.h <= 0.005 or t.h > 0.3:
        raise PickPlanError(f"目标高度 {t.h * 1000:.0f} mm 不在合理范围（5～300 mm）")


def build_plan(t: GraspTarget, current_pose: Sequence[float],
               hand_yaw: float = 0.0, speed: float = SPEED_PCT) -> PickPlan:
    """算四个点。高度按「法兰 → 指尖 185 mm」折算。"""
    _target_checks(t)
    yaw = (float(t.yaw) + float(hand_yaw)) % 360.0
    # 指尖目标高度 → 法兰高度。指尖停在盒子高度的一半：手指从两侧抱上去，
    # 而不是拿指尖压盒顶（压顶会滑）。
    tip_grasp = max(0.02, t.h * 0.5)
    tip_pre = t.h + PRE_CLEAR_M
    tip_lift = tip_pre + LIFT_M
    x, y = t.x * 1000.0, t.y * 1000.0
    pre_z = (tip_pre + TOOL_TIP_M) * 1000.0
    return PickPlan(
        target=t,
        home=[float(v) for v in current_pose],
        # 先抬到高处再平移：直接折过去会扫过料盒/自己的腕部（见 check_plan）。
        transit=[x, y, max(pre_z + 150.0, 450.0), 180.0, 0.0, yaw],
        pre=[x, y, pre_z, 180.0, 0.0, yaw],
        grasp=[x, y, (tip_grasp + TOOL_TIP_M) * 1000.0, 180.0, 0.0, yaw],
        lift=[x, y, (tip_lift + TOOL_TIP_M) * 1000.0, 180.0, 0.0, yaw],
        speed=float(speed), hand_yaw=float(hand_yaw),
    )


def _cell_without(cell: Dict[str, Any], name: str) -> Dict[str, Any]:
    """障碍物表里去掉目标本身（我们就是要碰它）。"""
    out = copy.deepcopy(cell)
    out["obstacles"] = [o for o in out.get("obstacles", [])
                        if str(o.get("name")) != name]
    return out


def _cartesian_samples(a: Sequence[float], b: Sequence[float],
                       step_mm: float = 20.0) -> List[List[float]]:
    """在两段位姿之间按直线采样（MovL 的走法）。"""
    dist = math.dist(list(a)[:3], list(b)[:3])
    n = max(1, min(60, int(math.ceil(dist / max(1.0, step_mm)))))
    out = []
    for i in range(n + 1):
        t = i / n
        out.append([a[k] + (b[k] - a[k]) * t for k in range(6)])
    return out


def check_plan(plan: PickPlan, arm, cell: Optional[Dict[str, Any]] = None,
               step_mm: float = 20.0, safe_mm: float = 10.0) -> Dict[str, Any]:
    """把每段折成关节角，逐段查最小间距。返回报告；不安全抛 ``PickPlanError``。

    ``safe_mm`` 是抓取段用的间距：低速（20%）、路径已知、桌面结构固定，按
    「标定误差 3 + 控制误差 2 + 余量 5」取 10 mm；人的禁入柱不在这儿放宽。
    """
    cell = _cell_without(cell or cell_mod.CELL, plan.target.name)
    rows: List[Dict[str, Any]] = []
    joints = list(arm.joints()) if hasattr(arm, "joints") else [0.0] * 6
    if len(joints) != 6:
        raise PickPlanError("拿不到当前关节角（先使能并读一次状态）")

    segs = [("抬高到过渡点", "MovJ", plan.home, plan.transit),
            ("平移到目标上方", "MovJ", plan.transit, plan.pre),
            ("下探到抓取点", "MovL", plan.pre, plan.grasp),
            ("抬升", "MovL", plan.grasp, plan.lift),
            ("退到预抓取点", "MovJ", plan.lift, plan.pre),
            ("退回过渡点", "MovJ", plan.pre, plan.transit)]
    for label, kind, a, b in segs:
        pts = [b] if kind == "MovJ" else _cartesian_samples(a, b, step_mm)
        prev = joints
        worst = {"min_m": 1e9, "pair": ["", ""], "at": None}
        for pose in pts:
            q = arm.ik(pose, near=prev)
            if q is None:
                raise PickPlanError(f"{label}：这个位姿反解不出来（{pose[:3]}），别动")
            res = cell_mod.check_path(prev, q, cell=cell, safe_mm=safe_mm)
            if not res.get("ok"):
                raise PickPlanError(f"{label}：路径检查失败 {res}")
            if res["min_m"] < worst["min_m"]:
                worst = {"min_m": res["min_m"], "pair": res["pair"],
                         "at": [round(v, 1) for v in pose[:3]]}
            prev = q
        rows.append({"seg": label, "kind": kind, "samples": len(pts),
                     "min_mm": round(worst["min_m"] * 1000, 1),
                     "pair": worst["pair"], "at": worst["at"],
                     "safe": worst["min_m"] >= safe_mm / 1000.0})
        joints = prev
    if not all(r["safe"] for r in rows):
        bad = [r for r in rows if not r["safe"]]
        raise PickPlanError(f"路径不满足 {safe_mm:.0f} mm 安全间距：" + "；".join(
            f"{r['seg']} {r['min_mm']} mm（{r['pair']}）" for r in bad))
    return {"ok": True, "rows": rows, "safe_mm": safe_mm,
            "min_mm": min(r["min_mm"] for r in rows),
            "end_joints": joints}


def run(plan: PickPlan, arm, hand, log=print,
        timeout: float = 30.0) -> Dict[str, Any]:
    """执行：张手 → 预抓取 → 下探 → 合手（互锁）→ 抬升 → 退开 → 张手。"""
    started = time.time()
    steps: List[Dict[str, Any]] = []

    def rec(name: str, **extra) -> None:
        steps.append({"step": name, "t_ms": int((time.time() - started) * 1000), **extra})
        log(f"[{steps[-1]['t_ms']:>6} ms] {name}" + (f"  {extra}" if extra else ""))

    def move(kind: str, target: Sequence[float], label: str) -> None:
        fn = arm.move_j if kind == "MovJ" else arm.move_l
        result = fn(target, wait=True, timeout=timeout)
        rec(label, cmd=kind, to=[round(float(v), 1) for v in target],
            arrived=bool(result.get("arrived", True)))
        if not result.get("arrived", True):
            raise PickPlanError(f"{label} 未到位（超时 {timeout:.0f}s），已停下")

    arm.enable()
    arm.speed_factor(plan.speed)
    hand.apply(poses.pose_of("open"), reason="pickbox:open")
    rec("手张开")

    move("MovJ", plan.transit, "抬高到过渡点")
    move("MovJ", plan.pre, "平移到目标上方")
    move("MovL", plan.grasp, "下探到抓取点")
    poses.close_hand(hand)
    rec("手闭合")
    deadline = time.time() + 2.5
    ok = False
    while time.time() < deadline:
        st = hand.state()
        if all(st[mid].angle >= poses.POSES["fist"][mid] * 0.6
               for mid in poses.POSES["fist"] if abs(poses.POSES["fist"][mid]) >= 5):
            ok = True
            break
        time.sleep(0.05)
    if not ok:
        raise PickPlanError("手没有合到位（可能电流不足触发 I-LIM），已中止，未抬升")
    rec("合手确认")
    move("MovL", plan.lift, "抬升")
    move("MovJ", plan.transit, "退到过渡点")
    rec("完成", seconds=round(time.time() - started, 2))
    return {"ok": True, "steps": steps, "seconds": round(time.time() - started, 2),
            "plan": {"pre": plan.pre, "grasp": plan.grasp, "lift": plan.lift}}


def _fmt_pose(p: Sequence[float]) -> str:
    return "[" + ", ".join(f"{float(v):.1f}" for v in p) + "]"


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="定位后沿安全路径抓起盒子")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--dry-run", action="store_true", help="只算计划和路径，不碰硬件")
    g.add_argument("--sim", action="store_true", help="仿真臂 + 仿真手跑一遍")
    g.add_argument("--go", action="store_true", help="真机执行（要 --yes）")
    ap.add_argument("--from-scene", action="store_true", help="目标从 /scene 的物体表里挑")
    ap.add_argument("--service", default="http://127.0.0.1:8766",
                    help="--from-scene 查询的服务地址")
    ap.add_argument("--expect", nargs=2, type=float, metavar=("X", "Y"),
                    help="确认位置：量到的目标必须在这个点附近（米）")
    ap.add_argument("--tol", type=float, default=0.02,
                    help="--expect 允许的偏差（米，默认 20 mm）")
    ap.add_argument("--name", default=None, help="指定场景里的物体名")
    ap.add_argument("--x", type=float, help="目标 x（米）")
    ap.add_argument("--y", type=float, help="目标 y（米）")
    ap.add_argument("--w", type=float, default=0.08, help="目标长（米）")
    ap.add_argument("--d", type=float, default=0.08, help="目标宽（米）")
    ap.add_argument("--h", type=float, default=0.06, help="目标高（米）")
    ap.add_argument("--yaw", type=float, default=0.0, help="目标朝向（度）")
    ap.add_argument("--hand-yaw", type=float, default=0.0, help="手附加转角（度）")
    ap.add_argument("--clearance-mm", type=float, default=10.0,
                    help="抓取路径的安全间距（毫米，默认 10；人的禁入柱不受影响）")
    ap.add_argument("--arm", default="192.168.5.1", help="真机 IP")
    ap.add_argument("--arm-live", action="store_true",
                    help="dry-run 时连真机只读：用控制器的 IK 和当前关节角检查")
    ap.add_argument("--hand", default="sim", choices=("sim", "can"),
                    help="真机执行时手接不接（can=真手）")
    ap.add_argument("--yes", action="store_true", help="确认真机执行")
    args = ap.parse_args(argv)

    try:
        if args.from_scene:
            target = target_from_service(args.service, args.name)
        elif args.x is not None and args.y is not None:
            target = GraspTarget(name=args.name or "盒子", x=args.x, y=args.y,
                                 w=args.w, d=args.d, h=args.h, yaw=args.yaw,
                                 source="cli")
        else:
            ap.error("要么 --from-scene，要么给 --x/--y")
            return 2
        if args.expect:
            check_expected(target, args.expect, args.tol)
            print(f"位置确认：({target.x * 1000:.0f}, {target.y * 1000:.0f}) mm，"
                  f"在预期 {args.tol * 1000:.0f} mm 内")

        if args.go:
            if not args.yes:
                print("真机执行要加 --yes（先跑 --dry-run 看路径）")
                return 2
            arm = arm_mod.DobotArm(args.arm)
            arm.connect()
            if args.hand == "can":
                from .cli import canbus_default_dll
                from .canbus import CanBus
                hand = CanBus(dll=canbus_default_dll())
            else:
                from .bus import SimBus
                hand = SimBus()
                print("注意：手是仿真的，真机抓取要加 --hand can")
            hand.open()
        else:
            if args.arm_live:
                arm = arm_mod.DobotArm(args.arm)
                arm.connect()
            else:
                arm = arm_mod.SimArm(pose=SIM_HOME_POSE)
                arm._joints = list(SIM_HOME_JOINTS)
            from .bus import SimBus
            hand = SimBus()
            hand.open()

        plan = build_plan(target, arm.pose(), args.hand_yaw)
        print(f"目标：{target.name}  ({target.x * 1000:.0f}, {target.y * 1000:.0f}) mm  "
              f"{target.w * 1000:.0f}×{target.d * 1000:.0f}×{target.h * 1000:.0f} mm  "
              f"yaw {target.yaw:.0f}°")
        print(f"过渡点 {_fmt_pose(plan.transit)}")
        print(f"预抓取 {_fmt_pose(plan.pre)}")
        print(f"抓取   {_fmt_pose(plan.grasp)}")
        print(f"抬升   {_fmt_pose(plan.lift)}")

        # 只读模式（dry-run / --arm-live）绝不动使能：读关节角不需要使能。
        if args.go and not arm.enabled():
            arm.enable()
        report = check_plan(plan, arm, safe_mm=args.clearance_mm)
        print(f"路径检查（阈值 {report['safe_mm']:.0f} mm）："
              f"最小间距 {report['min_mm']:.0f} mm")
        for r in report["rows"]:
            print(f"  {r['seg']:<10} {r['kind']} {r['min_mm']:>6.0f} mm  "
                  f"最近 {r['pair']}  {'OK' if r['safe'] else 'BLOCKED'}")

        if args.dry_run:
            print("dry-run：只做了规划与路径检查，没有动臂。")
            return 0
        if args.sim:
            if not hasattr(arm, "joints"):
                pass
            return 0 if run(plan, arm, hand) else 1
        result = run(plan, arm, hand)
        print(f"完成：{result['seconds']} s")
        return 0
    except (PickPlanError, arm_mod.ArmError) as exc:
        print(f"拒绝执行：{exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
