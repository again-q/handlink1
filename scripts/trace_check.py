"""二维轨迹模块自检：不连真机、不连服务，一条命令跑完。

用法::

    python scripts/trace_check.py

它做两件事：

1. 跑 :mod:`tests.test_trace2d` 和 :mod:`tests.test_trace_apply` 的单元测试；
2. 跑一条端到端仿真：画圆 → 反解 → 打印统计；再用假臂验证
   「归位 → 起点 → 逐点」的调用顺序。

任何一项不过，退出码就是 1。
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import arm as arm_mod          # noqa: E402
from handlink import bus, service, trace2d   # noqa: E402

OK = "  [ok]  "
BAD = "  [FAIL]"


def line(text: str = "") -> None:
    print(text)


def _compress(calls) -> str:
    """把连续的同一个动作并成 move_l x 61，不然一行几百个词看不清。"""
    out = []
    for k in calls:
        if out and out[-1][0] == k:
            out[-1][1] += 1
        else:
            out.append([k, 1])
    return " -> ".join(k if n == 1 else "%s x %d" % (k, n) for k, n in out)


def run_tests() -> bool:
    line("1) 单元测试")
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromNames([
        "tests.test_trace2d",
        "tests.test_trace_apply",
    ])
    result = unittest.TextTestRunner(verbosity=0, stream=open("nul", "w")).run(suite)
    total = result.testsRun
    bad = len(result.failures) + len(result.errors)
    if bad:
        line(BAD + " 跑了 %d 个，失败 %d 个" % (total, bad))
        for case, tb in result.failures + result.errors:
            line("        " + str(case))
            line("        " + tb.strip().splitlines()[-1])
        return False
    line(OK + "跑了 %d 个测试，全过" % total)
    return True


def run_simulation() -> bool:
    line()
    line("2) 端到端仿真（画圆 → 反解）")
    app = service.App(bus.SimBus())
    pts = [[0.5 + 0.28 * math.cos(a * math.pi / 90),
            0.5 + 0.30 * math.sin(a * math.pi / 90)] for a in range(0, 360, 6)]
    out = app.arm_trace(pts, apply=False, closed=True)
    if not out.get("ok"):
        line(BAD + "规划失败：" + str(out.get("error")))
        return False
    rep = out["report"]
    line(OK + "点数 %d，长度 %.1f mm，预计 %.1f s"
         % (rep["n_points"], rep["length_mm"], rep["duration_s"]))
    line(OK + "反解失败 %d 个，最大关节跳变 %.2f°，J6 净转 %.1f°"
         % (rep["failed"], rep["max_joint_step_deg"], rep["j6_net_deg"]))
    if rep["failed"]:
        line(BAD + "有反解失败的点")
        return False
    if rep["max_joint_step_deg"] > trace2d.DEFAULT.max_joint_step_deg:
        line(BAD + "关节跳变超过 %.0f°" % trace2d.DEFAULT.max_joint_step_deg)
        return False
    # 轨迹首尾应当闭合
    a, b = out["paper_points"][0], out["paper_points"][-1]
    gap = math.dist(a, b)
    if gap > 1.0:
        line(BAD + "闭合轨迹首尾差了 %.2f mm" % gap)
        return False
    line(OK + "闭合误差 %.3f mm" % gap)
    return True


def run_home_order() -> bool:
    line()
    line("3) 下发顺序（假臂，不连真机）")
    calls = []
    cps = []       # 逐点记录 cp：中间点应当带平滑，收笔点是 0
    radii = []     # 逐点记录 r（毫米）与加速度；默认路径走这个

    class FakeRobot:
        def __init__(self, *a, **kw):
            pass

        def connect(self):
            calls.append("connect")

        def close(self):
            calls.append("close")

        def speed_factor(self, pct):
            calls.append("speed")

        def move_joints(self, q, wait=True, timeout=20.0):
            calls.append("move_joints")

        def move_l(self, pose, wait=True, timeout=30.0):
            calls.append("move_l")

        # 真机才有的「带 cp 的 MovL + 队列 ID 节流」接口，仿真替身照实现一遍
        def movl_cp(self, pose, cp):
            calls.append("movl_cp")
            cps.append(int(cp))
            return len(cps)          # 假 ResultID

        def current_cmd_id(self):
            return len(cps) + len(radii)   # 假装机器全走完了

        def movl_r(self, pose, r_mm, accel=None):
            calls.append("movl_r")
            radii.append((int(round(float(r_mm))), accel))
            return len(cps) + len(radii)

        def wait_cmd_id(self, cmd_id, timeout=30.0):
            return True

        def stop(self):
            calls.append("stop")

    from unittest import mock

    app = service.App(bus.SimBus())
    class FakeFeed:
        def snapshot(self):
            return {"QActual": [0.0] * 6}

        def age(self):
            return 0.0
    app.arm_feed = FakeFeed()
    patches = [
        mock.patch.object(app, "_arm_live", return_value=True),
        mock.patch.object(app, "_arm_precheck", return_value=None),
        mock.patch.object(app, "_arm_gate",
                          side_effect=lambda cur, tgt, force=False: (list(tgt), {})),
        mock.patch.object(service.arm_feed, "describe",
                          # pose 也得给：接近段要用当前法兰位姿算「抬到安全高度」
                          return_value={"joints": [0.0] * 6,
                                        "pose": [300.0, -300.0, 300.0, 180.0, 0.0, 0.0]}),
        mock.patch.object(arm_mod, "DobotArm", FakeRobot),
    ]
    for p in patches:
        p.start()
    try:
        out = app.arm_trace([[0.3, 0.5], [0.7, 0.5]], apply=True)
    finally:
        for p in patches:
            p.stop()
    if not out.get("ok"):
        line(BAD + "下发规划失败：" + str(out.get("reason") or out.get("error")))
        return False
    line(OK + "调用序列：" + _compress(calls))
    # 现在的顺序：connect → speed → （腕部归中）→ 接近段 move_l → move_l（起点位姿）
    #                → movl_r 逐点（默认走 r）→ 最后一个点 cp=0 → close
    if "move_l" not in calls:
        line(BAD + "逐点之前应当先用 move_l 走到轨迹起点位姿")
        return False
    if not (cps or radii):
        line(BAD + "没有任何逐点下发，平滑方案没生效")
        return False
    if "movl_r" in calls:
        # 默认路径：圆角走绝对毫米 r，收笔点仍然 cp=0
        if cps != [0]:
            line(BAD + "走 r 时只有收笔点该用 cp=0，实际 %s" % cps)
            return False
        want = max(1, int(round(trace2d.DEFAULT.blend_r_mm)))
        if any(r != want for r, _a in radii):
            line(BAD + "中间点的 r 应当全是 %d，实际 %s" % (want, [r for r, _a in radii]))
            return False
        line(OK + "起点 → %d 点，中间点 r=%dmm，收笔点 cp=0" % (len(radii), want))
    else:
        # 备用路径：blend_r_mm=0 时退回 cp
        smooth = trace2d.DEFAULT.smooth_cp
        if any(c != smooth for c in cps[:-1]) or cps[-1] != 0:
            line(BAD + "cp 序列不对：%s" % cps)
            return False
        line(OK + "起点 → %d 点，中间点 cp=%d，收笔点 cp=0" % (len(cps), smooth))
    return True


def main() -> int:
    line("=" * 58)
    line("二维手绘轨迹模块自检")
    line("=" * 58)
    results = [run_tests(), run_simulation(), run_home_order()]
    line()
    line("=" * 58)
    if all(results):
        line("全部通过。可以开服务了：")
        line("    python -m handlink serve --bus sim")
        line("    http://127.0.0.1:8766/trace")
        return 0
    line("有项目没通过，见上面的 [FAIL]。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
