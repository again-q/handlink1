"""拟真环境自检：把「仿真 vs 真机实测」的关键数字打一张表。

用法：python scripts/sim_report.py            （全部走离线仿真，不需要硬件）
      python scripts/sim_report.py --json     （给别的程序吃）
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from handlink import arm, armsim, handsim, kin, spec  # noqa: E402


def probe_arm() -> dict:
    """29999 往返 + ServoJ ACK 的延迟分布，和真机 2026-11-15 的实测比。"""
    sim = armsim.VirtualDobot(seed=7).start()
    out = {}
    try:
        robot = sim.arm()
        robot.enable()
        pose_ms, servo_ms = [], []
        for _ in range(30):
            t0 = time.perf_counter()
            robot.pose()
            pose_ms.append((time.perf_counter() - t0) * 1000.0)
        q = robot.joints()
        for _ in range(60):
            t0 = time.perf_counter()
            robot.command("ServoJ(%s,t=0.03)" % ",".join("%.4f" % v for v in q))
            servo_ms.append((time.perf_counter() - t0) * 1000.0)
            time.sleep(0.03)
        out["getpose_ms"] = _stats(pose_ms)
        out["servoj_ack_ms"] = _stats(servo_ms)
        out["test_value_ok"] = True
        robot.close()
    finally:
        sim.stop()
    return out


def _stats(values) -> dict:
    values = sorted(values)
    n = len(values)
    return {"median": round(statistics.median(values), 2),
            "p90": round(values[min(n - 1, int(n * 0.9))], 2),
            "max": round(values[-1], 2), "n": n}


def probe_hand() -> dict:
    """90° 行程、0.6 A 卡位、心跳帧率——对齐 docs/现状.md 的实测表。"""
    out = {}
    sim = handsim.HandSim(seed=11)
    sim.open(); sim.attach()
    try:
        mid = 103
        pose = {m.motor_id: spec.Joint(0.0, 200, 1.5) for m in spec.MOTORS}
        sim.apply(pose, reason="probe:zero")
        for _ in range(2000):
            sim.step(0.002)
        target = {m.motor_id: spec.Joint(90.0, 200, 1.5) for m in spec.MOTORS}
        sim.apply(target, reason="probe:90")
        elapsed = 0.0
        for _ in range(3000):
            sim.step(0.002)
            elapsed += 0.002
            if abs(sim.state()[mid].angle - 90.0) < 0.5:
                break
        out["trip_90deg_s"] = round(elapsed, 3)
        weak = {m.motor_id: spec.Joint(180.0, 200, 0.6) for m in spec.MOTORS}
        sim.apply(weak, reason="probe:06a")
        for _ in range(6000):
            sim.step(0.002)
        out["stall_angle_at_0_6a"] = round(sim.state()[mid].angle, 1)
        hb = sim.heartbeat()
        out["heartbeat_fps"] = round(float(hb.get("frames_per_s", 0) or 0), 0)
    finally:
        sim.detach(); sim.close()
    return out


def probe_sim_arm() -> dict:
    sim = arm.SimArm(joints=[0.0, -25.0, -40.0, 0.0, 65.0, 0.0])
    sim.enable(); sim.speed_factor(100)
    out = {}
    t0 = time.perf_counter()
    result = sim.move_joints([0.0, -65.0, -40.0, 0.0, 65.0, 0.0], wait=True)
    out["move_40deg_eta_s"] = result["eta_s"]
    out["blocking_ms"] = round((time.perf_counter() - t0) * 1000.0, 1)
    out["ik_cache_entries"] = len(sim._ik_cache)
    out["pose_matches_fk"] = all(
        abs(a - b) < 3.0 for a, b in zip(sim.pose(), kin.fk(sim.joints())))
    return out


REAL = {
    "getpose_ms": {"median": 3.9, "p90": 5.0, "max": 5.4},
    "servoj_ack_ms": {"median": 6.2, "p90": 7.3, "max": 12.0},
    "trip_90deg_s": 1.6,
    "stall_angle_at_0_6a": 118.0,
    "heartbeat_fps": 3000,
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    report = {"arm": probe_arm(), "hand": probe_hand(), "sim_arm": probe_sim_arm(),
              "real_machine": REAL}
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    print("拟真环境自检（全部离线，不需要硬件）")
    print("=" * 74)
    rows = [
        ("29999 GetPose 中位", "%.2f ms" % report["arm"]["getpose_ms"]["median"],
         "%.1f ms" % REAL["getpose_ms"]["median"]),
        ("29999 GetPose max", "%.2f ms" % report["arm"]["getpose_ms"]["max"],
         "%.1f ms" % REAL["getpose_ms"]["max"]),
        ("ServoJ ACK 中位", "%.2f ms" % report["arm"]["servoj_ack_ms"]["median"],
         "%.1f ms" % REAL["servoj_ack_ms"]["median"]),
        ("ServoJ ACK p90", "%.2f ms" % report["arm"]["servoj_ack_ms"]["p90"],
         "%.1f ms" % REAL["servoj_ack_ms"]["p90"]),
        ("手 90 度行程", "%.2f s" % report["hand"]["trip_90deg_s"],
         "%.2f s" % REAL["trip_90deg_s"]),
        ("0.6 A 卡住角度", "%.1f deg" % report["hand"]["stall_angle_at_0_6a"],
         "%.0f deg" % REAL["stall_angle_at_0_6a"]),
        ("六指心跳", "%.0f fps" % report["hand"]["heartbeat_fps"],
         "%.0f fps" % REAL["heartbeat_fps"]),
    ]
    print("%-22s %-14s %-14s" % ("项目", "仿真", "真机实测"))
    print("-" * 74)
    for name, got, real in rows:
        print("%-22s %-14s %-14s" % (name, got, real))
    print("-" * 74)
    print("%-22s %-14s %-14s" % ("40 度关节行程（虚拟时钟）",
                                  "%.2f s" % report["sim_arm"]["move_40deg_eta_s"], "—"))
    print("%-22s %-14s %-14s" % ("同段的墙钟耗时",
                                  "%.1f ms" % report["sim_arm"]["blocking_ms"], "—"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())