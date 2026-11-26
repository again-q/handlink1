"""挥手：机械臂组动作的真机演示（默认只看不动）。

    python scripts/arm_wave.py                 # 打印路点 + 逐段安全间距，不动
    python scripts/arm_wave.py --yes           # 真跑（20% 速度，做完回原位）
    python scripts/arm_wave.py --cycles 4 --swing 15 --yes

动作本身在 ``handlink/arm_actions.py``：抬起前臂（J2/J3/J5）→ J1 左右摆、手腕
反相摆 ``cycles`` 次 → 原样放回。**起点就是臂现在这个姿态**，所以不用先回零。

开跑前把每条腿都算一遍：手腕防缠（``cable``）+ 碰撞间距（``cell``），有任意一段
低于红线就整条不跑 —— 组动作是连着的，中间停一下反而更危险。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from handlink import arm as arm_mod
from handlink import arm_actions, cable, cell, kin
from handlink.arm_feed import CONTROLLABLE_MODES, FeedbackError, Feed

TOL_DEG = 0.4


def wait_arrive(feed: Feed, target, timeout: float) -> bool:
    """到位判据要比 arm.wait_until_joints 严一点：连续 3 帧都在容差里才算。"""
    deadline = time.time() + timeout
    hits = 0
    while time.time() < deadline:
        j = [float(v) for v in feed.snapshot()["QActual"]]
        if all(abs(j[i] - target[i]) <= TOL_DEG for i in range(6)):
            hits += 1
            if hits >= 3:
                return True
        else:
            hits = 0
        time.sleep(0.08)
    return False


def print_plan(rows, start, speed: float) -> None:
    print(f"起点 J1..J6 = {[round(v, 1) for v in start]}，速度因子 {speed:.0f}%")
    print(f"{'段':>2} {'动作':<16} {'J1':>7} {'J2':>7} {'J3':>7} {'J4':>7} {'J5':>7} {'J6':>7} "
          f"{'行程':>6} {'停顿':>6} {'间距':>6}")
    for r in rows:
        q = r["joints"]
        mm = r.get("clearance_mm")
        gap = f"{mm:5d}mm" if isinstance(mm, int) else "  —  "
        print(f"{r['step']:>2} {r['label']:<16} "
              + " ".join(f"{v:>7.1f}" for v in q)
              + f" {r['travel_deg']:>5.0f}° {r['hold_ms']:>5d}ms {gap:>7}")


def main() -> int:
    ap = argparse.ArgumentParser(description="机械臂挥手组动作")
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--speed", type=float, default=arm_actions.SAFE_SPEED,
                    help="速度因子 %%，默认 20")
    ap.add_argument("--cycles", type=int, default=3, help="左右摆几次（1~6）")
    ap.add_argument("--swing", type=float, default=12.0, help="J1 摆幅（度）")
    ap.add_argument("--wrist", type=float, default=16.0, help="手腕 J5 摆幅（度）")
    ap.add_argument("--yes", action="store_true", help="真跑；不给就只打印计划")
    ap.add_argument("--force", action="store_true", help="间距不够也跑（现场调试用）")
    args = ap.parse_args()

    try:
        feed = Feed(args.ip).open()
    except FeedbackError as exc:
        print(f"臂不在线：{exc}\n"
              "检查三件事：① 连臂的 USB 网卡插回电脑（系统里会出现「以太网 2」，\n"
              "静态 IP 192.168.5.100 会自动回来）；② 控制柜上电、蓝灯常亮；\n"
              "③ 网线插在控制柜 LAN 口。就绪后再跑一次本脚本。", file=sys.stderr)
        return 2

    try:
        feed.wait_frame(timeout=5)
        start = [float(v) for v in feed.snapshot()["QActual"]]
        try:
            moves = arm_actions.build("wave", start, cycles=args.cycles,
                                      swing_deg=args.swing, wrist_deg=args.wrist)
        except ValueError as exc:
            print(f"参数不对：{exc}", file=sys.stderr)
            return 2

        issues = arm_actions.validate(moves)
        for issue in issues:
            print(f"[{issue['level']}] {issue['message']}", file=sys.stderr)
        if any(i["level"] == "error" for i in issues):
            return 2

        # 整条路径先算一遍：手腕防缠 + 逐段碰撞间距
        rows = arm_actions.describe(moves, start, args.speed)
        guard = cable.TwistGuard()
        guard.observe(start)
        prev = list(start)
        unsafe = []
        for row, move in zip(rows, moves):
            planned, notes, blocked = guard.choose(prev, move.joints)
            path = cell.check_path(prev, planned)
            row["clearance_mm"] = (int(round(path["min_m"] * 1000))
                                   if path.get("ok") and path.get("min_m") is not None else None)
            if blocked or not path.get("ok") or (path.get("min_m", 1.0) * 1000 < cell.SAFE_MM):
                unsafe.append((row["step"], row["label"],
                               "；".join(notes) or
                               (f"间距 {row['clearance_mm']} mm < {cell.SAFE_MM:.0f} mm"
                                if row["clearance_mm"] is not None else "校验失败")))
            prev = planned

        print_plan(rows, start, args.speed)
        seconds = arm_actions.estimate_seconds(moves, start, args.speed)
        print(f"\n共 {len(moves)} 段，预计 {seconds:.1f} s，"
              f"走完回到起点 {[round(v, 1) for v in moves[-1].joints]}")

        if unsafe and not args.force:
            print("\n安全校验没过，整条不跑：", file=sys.stderr)
            for step, label, why in unsafe:
                print(f"  第 {step} 段 {label}：{why}", file=sys.stderr)
            print("先在 /arm3d 里调姿态 / 挪开工位上的东西，确认现场后可加 --force。",
                  file=sys.stderr)
            return 3
        if unsafe:
            print("\n[--force] 以下段低于红线，仍然继续：", file=sys.stderr)
            for step, label, why in unsafe:
                print(f"  第 {step} 段 {label}：{why}", file=sys.stderr)

        if not args.yes:
            print("\n只看不动。确认工作空间里没人、手边没有夹住的线缆后加 --yes 真跑。")
            return 0

        desc = feed.summary()
        if desc["mode"] not in CONTROLLABLE_MODES and not desc["enabled"]:
            raise SystemExit(f"当前模式「{desc['mode_name']}」既不能接管也不能动，先看 arm_up.py")

        robot = arm_mod.DobotArm(ip=args.ip)
        robot.connect()
        try:
            if not desc["enabled"]:
                robot.take_control()
                print("已接管 TCP 控制")
                robot.enable()
                time.sleep(1.0)
            robot.speed_factor(args.speed)
            print(f"速度因子 {args.speed:.0f}%")

            for move in moves:
                robot.move_joints(move.joints, wait=False)
                ok = wait_arrive(feed, move.joints, timeout=15.0)
                print(f"  {move.label:<16} {'到位' if ok else '超时 → Stop()'}")
                if not ok:
                    robot.stop()
                    break
                if move.hold_ms:
                    time.sleep(move.hold_ms / 1000.0)
        finally:
            robot.close()
        snap = feed.summary()
        joints = [float(v) for v in feed.snapshot()["QActual"]]
        drift = max(abs(joints[i] - start[i]) for i in range(6))
        print(f"\n末态：{snap['mode_name']}，报警 {snap['error']}，"
              f"相对起点最大偏差 {drift:.2f}°")
        return 0 if drift < 2.0 else 1
    finally:
        feed.close()


if __name__ == "__main__":
    raise SystemExit(main())
