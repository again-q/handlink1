"""读机械臂的报警码，必要时清报警。

机械臂红灯 = 报错状态（手册：蓝=上电未使能，绿=已使能，黄=录制/复现，红=报错）。

    python scripts/arm_fault.py              # 只读：30004 的状态位 + GetErrorID()
    python scripts/arm_fault.py --clear      # 读完之后 ClearError()，再读一遍

两条通道分工：
* **30004** 是被动反馈，只要网通就能读——`ErrorStatus`、`CollisionState`、
  `SafetyState` 里的急停/防护性停止位都在里面；
* **29999** 的 `GetErrorID()` 才有错误码本身。手册第 131 页写明：报错状态下只允许
  `ClearError / GetErrorID / EmergencyStop / Stop / RobotMode / LogExportUSB`，
  所以别指望这时候还能读位姿。
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from handlink import arm as arm_mod
from handlink.arm_feed import FeedbackError, Feed

GROUP_NAMES = ["控制器/算法", "伺服 J1", "伺服 J2", "伺服 J3", "伺服 J4", "伺服 J5", "伺服 J6"]


def parse_error_ids(text: str):
    """``0,{[[1,2],[],[],...]},GetErrorID();`` → [[1,2], [], ...]"""
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end < 0:
        return None
    groups = re.findall(r"\[([^\]]*)\]", text[start:end])
    if not groups:
        return None
    return [[int(x) for x in re.findall(r"-?\d+", g)] for g in groups]


def main() -> int:
    ap = argparse.ArgumentParser(description="读/清机械臂报警")
    ap.add_argument("--ip", default="192.168.5.1")
    ap.add_argument("--clear", action="store_true", help="读完后尝试 ClearError()")
    args = ap.parse_args()

    out = {"ip": args.ip}

    # -- 反馈口：只要有网就能读，和 TCP 控制模式无关 -----------------------
    try:
        feed = Feed(args.ip).open()
        try:
            feed.wait_frame(timeout=4)
            snap = feed.snapshot()
            summary = feed.summary()
        finally:
            feed.close()
        out["feedback"] = {
            "mode": summary["mode_name"],
            "enabled": summary["enabled"],
            "error_status": int(snap["ErrorStatus"]),
            "collision": int(snap["CollisionState"]),
            "safety": summary["safety"],
            "six_force_online": int(snap["SixForceOnline"]),
            "voltage": round(float(snap["VRobot"]), 2),
        }
    except FeedbackError as exc:
        out["feedback_error"] = str(exc)

    # -- 控制口：错误码 -----------------------------------------------------
    robot = arm_mod.DobotArm(ip=args.ip)
    try:
        raw = robot._command("GetErrorID()")
        ids = parse_error_ids(raw)
        out["get_error_id_raw"] = raw
        if ids:
            out["alarms"] = {GROUP_NAMES[i] if i < len(GROUP_NAMES) else f"组{i}": g
                             for i, g in enumerate(ids) if g}
            if not out["alarms"]:
                out["alarms"] = {}
    except arm_mod.ArmError as exc:
        out["get_error_id_error"] = str(exc)

    print(json.dumps(out, ensure_ascii=False, indent=2))

    if args.clear:
        try:
            robot.clear_error()
            print("\nClearError() 已发出")
            out2 = robot._command("GetErrorID()")
            print("清后：", out2)
        except arm_mod.ArmError as exc:
            print(f"\n清报警失败：{exc}")
            print("多半是报警源还在（急停没复位 / 碰撞没解除 / 关节超限），"
                  "先把物理原因解决再清")
    robot.close()

    # -- 人话提示 -----------------------------------------------------------
    print()
    if "feedback_error" in out:
        print("反馈口都连不上 → 先解决物理链路：USB 网卡、网线、控制柜上电。")
        return 2
    safety = out.get("feedback", {}).get("safety", {})
    if safety.get("estop"):
        print("⚠️  SafetyState 显示急停有效 —— 旋起控制柜/示教器上的急停按钮再清报警。")
    if out.get("feedback", {}).get("collision"):
        print("⚠️  CollisionState 非 0 —— 手臂撞到什么了，先手动让开再清。")
    if out.get("alarms"):
        print("报警码见上面 alarms 字段：第一组是控制器/算法，后六组依次是 J1~J6 伺服。")
        print("具体含义查随机器附的《错误码表》，或把码发我，我对着手册翻。")
    elif "alarms" in out:
        print("GetErrorID() 返回全空 —— 控制器和六个伺服都没有报警码，红灯另有原因"
              "（急停/安全停止/控制柜↔臂通信），看上面 safety 那几项。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
