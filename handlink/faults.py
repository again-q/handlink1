# -*- coding: utf-8 -*-
"""真机故障复现场景库：把「这个工位真实踩过的坑」变成一键重放的最小场景。

不是通用仿真器。每个场景只回答两件事：

1. **现象能不能重现** —— 没有臂的人也能看到同一个症状（返回 observed）；
2. **修法还有没有效** —— 同一场景里把修法再跑一遍，passed 必须为真。

统一签名::

    def scenario_xxx(arm, hand=None, **kwargs) -> dict

返回 {"name","symptom","expected","observed","passed","hint","trace"}；
trace 是场景自己记的事件流（每一步做了什么、数值多少），出问题先看它。

能跑在三类对象上
----------------
* handlink.arm.SimArm —— 有运动学、虚拟时钟、算法队列的仿真臂；
* handlink.armsim.VirtualDobot 起的虚拟控制器（sim.arm()）—— 真 29999 文本协议；
* tests/ 里的鸭子类型假对象 —— 只要有 enable/move_l/move_joints/pose/joints
  这一套就够；用到别的接口前都会先 hasattr 探一下。

**不改任何已有文件**。某个场景在 SimArm 上缺能力时不补仿真，而是
「退化成 X」并把这件事写进 hint（每个场景 docstring 的「退化」一节）。

场景清单
--------
===========================  ===========================================  =================
名字                         现象                                         出处
===========================  ===========================================  =================
movl_queue_overflow          一次 wait=False 推 200 条 → 队列满、新指令被拒  2026-11-24 真机
batch_vs_cadence             分批等 → 一卡一卡；窗口节流才两头都不占         2026-11-24/25
servoj_default_t_lag         ServoJ 的 t 默认 0.1 太大，被误判成网络慢       遥操作-调研 1.1
servoj_stop_holds            停发 ServoJ：保持、mode=5、不跑位不报警         遥操作-调研 1 节
speed_factor_ignores_servo   1% 速度因子对 ServoJ 无效（手册 p.89）          遥操作-调研 1 节
j6_drift_hits_limit          每条 300 mm 的线漂 ~50°，-357.9° 报警           毛笔字 2④
tip_53mm_short               指尖比模型长 53 mm，模型说 55 mm 时已贴上       cell.py:80-83
port_29999_preempted         29999 被抢占 → 10053，自动重连一次才活          毛笔字 2⑦ / 10053
error_codes_matter           未使能 -1（状态不允许）vs 未知指令 -10000       遥操作-调研 1 节
joint_limit_rejected         关节超限被拒（-2 / -40000）                    kin.JOINT_LIMITS
step_too_big                 组动作单步 > 45° 被拒                          arm_actions.validate
segment_too_long             段数/停顿超上限被拒                            arm_actions.validate
estop_blocks_motion          急停红灯但 GetErrorID 全空：不许动、别误报       .tmp/fault3.out
===========================  ===========================================  =================

用法::

    from handlink import arm, faults
    a = arm.SimArm(); a.enable()
    print(faults.run_all(a))            # 全跑一遍，出汇总表
    print(faults.run("servoj_default_t_lag", a))

    from handlink import armsim         # 真 29999 协议，更硬的复现
    sim = armsim.VirtualDobot().start()
    faults.run_all(sim.arm())

命令行（不需要任何硬件）::

    python -m handlink.faults
"""
from __future__ import annotations

import copy
import math
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from . import arm_actions, cable, cell, kin, trace2d

__all__ = ["SCENARIOS", "run", "run_all", "names", "classify_stop",
           "legacy_cell_model", "REAL_TIP_LOWEST_M", "MODEL_TIP_LOWEST_M"]

#: 实测（cell.py:80-83）：真实指尖最低点在法兰下方 247 mm
REAL_TIP_LOWEST_M = 0.247
#: 修正前那套胶囊的最低点：0.185 + 0.009 = 0.194，比真实短 53 mm
MODEL_TIP_LOWEST_M = 0.194

#: 真机错误码（docs/毛笔字轨迹-平滑下发-2026-11-26.md 第 6 节）
ERR_STATE = -1          # 状态不允许（未使能 / 报警中）
ERR_ALARM = -2          # 报警中
ERR_ESTOP = -3          # 急停
ERR_UNKNOWN = -10000    # 指令不存在
ERR_PARAM = -50001      # 参数类型错（r=3.0 就是踩这个）
#: 「状态不允许 / 参数不合法」这一类：收到了但拒绝执行，和「指令不存在」必须分开
STATE_CODES = (ERR_STATE, ERR_ALARM, ERR_ESTOP, -40000, ERR_PARAM)

#: 真机 29999 被顶掉时 Windows 给的错误码
WIN_CONN_ABORTED = 10053

#: ServoJ 的官方节拍（手册：建议 33 Hz）
SERVO_PERIOD = 0.03


# ================================================================ 小工具

def _err_code(text: Any) -> Optional[int]:
    """从 -1,{},... 或 RAISE -1,... 里取出第一段整数。"""
    s = str(text).strip()
    if s.upper().startswith("RAISE"):
        s = s[5:].strip()
    head = s.split(",", 1)[0].strip()
    try:
        return int(float(head))
    except ValueError:
        return None


def _call(arm: Any, fn: str, *args: Any, **kwargs: Any) -> Dict[str, Any]:
    """调一个可能不存在的接口，把结果/异常统一成 dict（不抛出去）。"""
    method = getattr(arm, fn, None)
    if method is None:
        return {"ok": False, "missing": fn}
    try:
        out = method(*args, **kwargs)
        return {"ok": True, "value": out}
    except Exception as exc:                      # noqa: BLE001 - 场景里要收所有异常
        return {"ok": False, "error": str(exc), "code": _err_code(exc),
                "type": type(exc).__name__}


def _dash(arm: Any, text: str) -> Dict[str, Any]:
    """发一条 29999 原文（只有实现了 command() 的对象才有）。

    有的对象（SimArm / 虚拟控制器）不抛异常，而是把 -1 / -10000 写在回复原文里，
    所以这里把回复的第一段解析成 code 一并带上。
    """
    out = _call(arm, "command", text)
    if out.get("ok"):
        out["code"] = _err_code(out.get("value"))
    return out


def _reply_value(arm: Any, text: str) -> Optional[str]:
    """发一条只读指令，返回回复原文。

    注意负数错误码的回复（``-1,{},...`` / ``-10000,{},...``）**照样返回原文**：
    那不是「这条指令不存在」的异常，而是控制器在告诉你状态；把 -10000 也吞掉
    会让 GetErrorID 之类误判成「读不到」。真正没实现 command() 的对象才返回 None。
    """
    out = _call(arm, "command", text)
    if not out.get("ok") or out.get("missing"):
        return None
    return str(out["value"])


def _cmd_id(arm: Any) -> int:
    """GetCurrentCommandID 的当前值；读不到就返回 0。"""
    out = _call(arm, "current_cmd_id")
    if out.get("ok"):
        try:
            return int(out["value"])
        except (TypeError, ValueError):
            return 0
    return 0


def _error_id(arm: Any) -> Optional[int]:
    """GetErrorID() 里的报警码；0 表示控制器说没报警，None 表示读不到。"""
    text = _reply_value(arm, "GetErrorID()")
    if text is None:
        return None
    if _err_code(text) not in (0, None):
        return None                               # 固件连这条指令都没有，别当成「没报警」
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        return None
    body = text[start + 1:end].strip().strip("[]").strip()
    if not body:
        return 0                                  # 全空 = 控制器说没报警
    try:
        return int(float(body.split(",")[0]))
    except ValueError:
        return None


def _robot_mode(arm: Any) -> Optional[int]:
    """RobotMode()：4 下使能 / 5 使能空闲 / 7 运行 / 9 报警。"""
    text = _reply_value(arm, "RobotMode()")
    if text is not None:
        start, end = text.find("{"), text.rfind("}")
        if start >= 0 and end > start:
            try:
                return int(float(text[start + 1:end].split(",")[0]))
            except ValueError:
                return None
    mode = getattr(arm, "mode", None)
    return int(mode) if isinstance(mode, int) and mode > 0 else None


def _joints(arm: Any) -> List[float]:
    out = _call(arm, "joints")
    value = out.get("value") if out.get("ok") else None
    try:
        return [float(v) for v in value]          # type: ignore[union-attr]
    except (TypeError, ValueError):
        return [0.0] * 6


def _pose(arm: Any) -> List[float]:
    out = _call(arm, "pose")
    value = out.get("value") if out.get("ok") else None
    try:
        return [float(v) for v in value]          # type: ignore[union-attr]
    except (TypeError, ValueError):
        return [0.0] * 6


def _safety(arm: Any) -> Optional[Dict[str, bool]]:
    """急停 / 防护停止。

    DobotArm 本身没有读 SafetyState 的入口（那是 30004 的字段，见 arm_feed）；
    armsim 的 sim.state() 有。读不到返回 None —— 不假装知道。
    """
    sim = getattr(arm, "sim", None)
    state = getattr(sim, "state", None)
    if callable(state):
        try:
            raw = state()
        except Exception:                         # noqa: BLE001
            raw = None
        if isinstance(raw, dict) and "estop" in raw:
            estop = bool(raw.get("estop"))
            return {"estop": estop, "protective_stop": estop}
    feed = getattr(arm, "safety", None)
    if callable(feed):
        try:
            return {str(k): bool(v) for k, v in feed().items()}
        except Exception:                         # noqa: BLE001
            return None
    return None


def classify_stop(error_id: Optional[int],
                  safety: Optional[Dict[str, bool]]) -> str:
    """把「停住了」分成三类，别把安全停止误报成伺服报警。

    "safety"      —— 急停 / 防护停止（安全链断，GetErrorID() 可能是**全空**的，
                     这正是 .tmp/fault3.out 里现场那次的样子）；
    "servo_alarm" —— 控制器报了报警码（伺服/限位/碰撞），要 ClearError；
    "ok"          —— 没有停止迹象。
    """
    if safety and (safety.get("estop") or safety.get("protective_stop")):
        return "safety"
    if error_id:
        return "servo_alarm"
    return "ok"


class _Bench:
    """场景和「时间」打交道的那一层：虚拟时钟 / 仿真控制器 / 真墙钟三选一。"""

    #: 真墙钟下的最大让步（秒）。测试总时长靠它兜底，别在这里 sleep 十几秒。
    WALL_BUDGET = 3.0

    def __init__(self, arm: Any, trace: List[Dict[str, Any]]) -> None:
        self.arm = arm
        self.trace = trace
        self._advance = getattr(arm, "advance", None)
        self._sim = getattr(arm, "sim", None)
        self._settle = getattr(self._sim, "settle", None)
        self._realtime = bool(getattr(arm, "realtime", False))
        if callable(self._settle):
            self.how = "sim.settle"
        elif callable(self._advance) and not self._realtime:
            self.how = "advance"
        else:
            self.how = "wall"
        self.spent = 0.0

    def step(self, seconds: float) -> None:
        """把时间往前推。虚拟时钟不花墙钟；真墙钟有总预算。"""
        seconds = max(0.0, float(seconds))
        if self.how == "advance":
            try:
                self._advance(seconds)
            except Exception:                     # noqa: BLE001
                pass
            self.spent += seconds
            return
        left = max(0.0, min(seconds, self.WALL_BUDGET - self.spent))
        if left <= 0:
            return
        try:
            if self.how == "sim.settle":
                self._settle(left)
            else:
                time.sleep(left)
        except Exception:                         # noqa: BLE001
            pass
        self.spent += left

    def log(self, event: str, **kw: Any) -> Dict[str, Any]:
        row: Dict[str, Any] = {"t": round(self.spent, 3), "event": event}
        row.update(kw)
        self.trace.append(row)
        return row


def _result(name: str, symptom: str, expected: str, observed: str, passed: bool,
            hint: str, trace: Optional[List[Dict[str, Any]]] = None,
            **extra: Any) -> Dict[str, Any]:
    out: Dict[str, Any] = {"name": name, "symptom": symptom, "expected": expected,
                           "observed": observed, "passed": bool(passed),
                           "hint": hint, "trace": list(trace or [])}
    out.update(extra)
    return out


def _ensure_enabled(arm: Any, bench: Optional["_Bench"] = None) -> bool:
    """确保臂是使能的。enable() 在仿真里会占一条队列指令，顺手把队列排干净。"""
    out = _call(arm, "enabled")
    ok = bool(out.get("ok") and out.get("value"))
    if not ok:
        ok = bool(_call(arm, "enable").get("ok"))
    _settle_queue(arm, bench)
    return ok


def _settle_queue(arm: Any, bench: Optional["_Bench"] = None,
                  timeout_s: float = 5.0) -> None:
    """等队列空（enable 之类也会进队列，别把它算成「停发后还在动」）。"""
    ident = _cmd_id(arm)
    wait = getattr(arm, "wait_cmd_id", None)
    if callable(wait):
        try:
            wait(ident, timeout=timeout_s)
            return
        except Exception:                         # noqa: BLE001
            pass
    if bench is not None:
        bench.step(1.0)



# ================================================================ 1. 队列

def _send_one(arm: Any, pose: Sequence[float], cp: int = 30) -> Dict[str, Any]:
    """发一条 MovL（不等运动）。有 movl_cp 就带 cp，没有就退回 move_l。"""
    if hasattr(arm, "movl_cp"):
        out = _call(arm, "movl_cp", pose, cp)
        if not out.get("missing"):
            return out
    return _call(arm, "move_l", pose, wait=False)


def _backlog(arm: Any) -> Optional[int]:
    """控制器里还压着几条（只在能直接问到的时候给，问不到返回 None）。

    SimArm 的 current_cmd_id() 会随队列推进**回退**（它报的是正在执行的那条的
    id），拿「已发 - 当前」算积压会算出负数，所以那种对象上只能用 count 兜。
    """
    queue = getattr(arm, "_queue", None)
    active = getattr(arm, "_active", None)
    if queue is not None and hasattr(queue, "__len__"):
        return int(len(queue)) + (1 if active is not None else 0)
    sim = getattr(arm, "sim", None)
    state = getattr(sim, "state", None)
    if callable(state):
        try:
            raw = state()
        except Exception:                         # noqa: BLE001
            return None
        if isinstance(raw, dict) and "queue_len" in raw:
            return int(raw["queue_len"])
    return None


def _wait_backlog(bench: "_Bench", window: int, timeout_s: float = 20.0,
                  peak: int = 0) -> int:
    """等队列里的在途条数降到 window 以内；返回见过的最大 cmd_id。

    真机 ID 单调，SimArm 的会回退，所以一律取 max，别直接拿 last。
    """
    arm = bench.arm
    spent = 0.0
    while spent < timeout_s:
        peak = max(peak, _cmd_id(arm))
        depth = _backlog(arm)
        if depth is not None and depth <= window:
            return peak
        if depth is None and sent_kind(arm):
            return peak
        bench.step(0.02)
        spent += 0.02
    return peak


def sent_kind(arm: Any) -> bool:
    """没有可数的队列时（纯鸭子对象），不要在节流上死等。"""
    return _backlog(arm) is None


def _paced_movl(bench: "_Bench", pose: Sequence[float], n: int,
                window: int, cp: int = 30) -> Dict[str, int]:
    """窗口节流下发：队列里在途超过 window 条就等一等（只等 ID 追上，不等停稳）。"""
    arm = bench.arm
    peak = _cmd_id(arm)
    sent = rejected = retried = 0
    max_backlog = 0
    for i in range(int(n)):
        out = _send_one(arm, pose, cp)
        if not out.get("ok"):
            # 队列还是满的（-50001）：先等，再重试一次
            retried += 1
            peak = _wait_backlog(bench, int(window), peak=peak)
            out = _send_one(arm, pose, cp)
            if not out.get("ok"):
                rejected += 1
                continue
        sent += 1
        depth = _backlog(arm)
        if depth is not None:
            max_backlog = max(max_backlog, depth)
        if i % max(1, int(window)) == 0:
            peak = _wait_backlog(bench, int(window), peak=peak)
    peak = _wait_backlog(bench, int(window), peak=peak)
    depth = _backlog(arm)
    if depth is not None:
        max_backlog = max(max_backlog, depth)
    return {"sent": sent, "rejected": rejected, "retried": retried,
            "max_backlog": max_backlog}


def scenario_movl_queue_overflow(arm, hand=None, *, points: int = 200,
                                 window: int = 8, **kw) -> dict:
    """逐条 wait=False 的 MovL 把算法队列塞满 → 新指令被拒（真机上就是「卡住」）。

    复现 2026-11-24 真机那次「归位到起点之后一动不动」：一条 200 点的轨迹一次性
    推 200 条 MovL 进控制器，队列有容量上限，超了就拒收 / 停滞。

    修法验证：**边执行边下发**，队列里只留 window 条（用算法队列 ResultID 判断）。
    注意不是「分 20 条一批、等这一批走完」——那个会把「卡死」换成「一卡一卡」
    （见 batch_vs_cadence）。

    出处：.tmp/prev_main.txt 第 4505-4594 行（真机现场）；docs/毛笔字轨迹-平滑下发
    第 2 节；service.arm_trace 的 window/batch_size 注释。

    退化（SimArm）：不模拟「控制器卡住不动」，只模拟「积压超过 max_queue（默认
    64）后新指令回 -50001」。所以这里只断言**被拒**，不断言「静止不动」。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    if not _ensure_enabled(arm, bench):
        return _result("movl_queue_overflow", "队列积压 → 新指令被拒",
                       "积压到上限必须被拒", "臂没使能，场景没法跑", False,
                       "先 arm.enable()", trace)
    base = _pose(arm)
    target = list(base)
    target[0] = base[0] + 20.0                     # 一条 20 mm 的短线

    accepted, rejected, first_error = 0, 0, ""
    for _ in range(int(points)):
        out = _send_one(arm, target)
        if out.get("ok"):
            accepted += 1
        else:
            rejected += 1
            first_error = str(out.get("error", ""))[:120]
            break
    bench.log("burst", sent=accepted + rejected, accepted=accepted,
              rejected=rejected, first_error=first_error)

    _settle_queue(arm, bench)                      # 先把塞满的队列排空再试修法
    paced = _paced_movl(bench, target, int(points), int(window))
    bench.log("paced", window=int(window), sent=paced["sent"],
              rejected=paced["rejected"], max_backlog=paced["max_backlog"])

    passed = (rejected > 0 and str(ERR_PARAM) in first_error
              and paced["sent"] == int(points) and paced["rejected"] == 0
              and paced["max_backlog"] <= 2 * int(window))
    observed = ("一口气推 %d 条：接受 %d 条后被拒（%s）；窗口 %d 节流：%d 条全发完、"
                "被拒 %d 条、队列积压峰值 %d 条"
                % (int(points), accepted, first_error or "没被拒", int(window),
                   paced["sent"], paced["rejected"], paced["max_backlog"]))
    hint = ("别「发 N 条再等这一批走完」——每批末尾都会停车。控的是「队列里在途几条」，"
            "用 GetCurrentCommandID() 做窗口节流（service.arm_trace 的 window）。"
            "SimArm 的 current_cmd_id() 会随队列推进回退，真机 ID 单调，节流里要取 max。")
    return _result("movl_queue_overflow",
                   "一次 wait=False 推 200 条 MovL → 队列塞满，新指令被拒 / 臂卡住",
                   "积压到上限必须被拒（-50001）；窗口节流后 %d 条全发完且积压 ≤ %d"
                   % (int(points), int(window)),
                   observed, passed, hint, trace,
                   numbers={"accepted": accepted, "rejected": rejected,
                            "max_backlog": paced["max_backlog"], "window": int(window)})


# ================================================================ 2. 节拍

def scenario_batch_vs_cadence(arm, hand=None, *, points: int = 24,
                              burst: int = 200, window: int = 8, **kw) -> dict:
    """批大小 vs 节拍的权衡：批太大塞满队列，每批都等又变成「一卡一卡」。

    2026-11-24 的两次反复就卡在这上面：

    * burst=200（等于不分批）→ 队列满；
    * 每点都 wait=True（批=1）→ 每个点都减速到 0，写出来一抖一抖；
    * 窗口节流（按算法队列 ID）→ 队列不积压、中间点不停车。

    「停车次数」在这里等于「有多少条指令是等到停稳才发下一条」；真机上它就是
    速度曲线掉到 0 的次数（scripts/trace_track.py 的判据）。

    出处：.tmp/prev_main.txt 第 4533-4610 行；docs/毛笔字轨迹-平滑下发 第 5 节判据表。

    退化（SimArm）：没有 30004 速度曲线，用 wait=True 的次数当停车次数。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    if not _ensure_enabled(arm, bench):
        return _result("batch_vs_cadence", "批大小 vs 节拍", "场景跑不了", "臂没使能",
                       False, "先 arm.enable()", trace, skipped=True)
    base = _pose(arm)
    pts = int(points)

    def segment(i: int) -> List[float]:
        pose = list(base)
        pose[0] = base[0] + 5.0 * (i + 1)          # 5 mm 一段，模拟重采样后的点距
        return pose

    burst_rejected = 0
    for i in range(int(burst)):
        out = _send_one(arm, segment(i % pts))
        if not out.get("ok"):
            burst_rejected += 1
            break
    bench.log("burst", burst=int(burst), rejected=burst_rejected)
    _settle_queue(arm, bench)                      # 排空再验「每点都等」

    # 每点都等：批=1 的老写法。真墙钟下只跑几段，别把测试拖长。
    waits = pts if bench.how != "wall" else min(pts, 3)
    stalls = 0
    for i in range(waits):
        out = _call(arm, "move_l", segment(i), wait=True, timeout=5.0)
        if out.get("ok"):
            stalls += 1                            # 停下等到了才发下一条
    bench.log("wait_each", points=waits, stalls=stalls)

    paced = _paced_movl(bench, segment(0), pts, int(window))
    bench.log("paced", window=int(window), sent=paced["sent"],
              max_backlog=paced["max_backlog"])

    passed = (burst_rejected > 0 and stalls == waits
              and paced["sent"] == pts and paced["rejected"] == 0
              and paced["max_backlog"] <= 2 * int(window))
    observed = ("burst=200：发到第 %d 条就被拒；每点都等：%d/%d 段各停一次"
                "（= 一卡一卡）；窗口 %d：%d 点无拒绝、积压峰值 %d 条、中途不停车"
                % (pts, stalls, waits, int(window), paced["sent"],
                   paced["max_backlog"]))
    hint = ("批大小不是病根，**等停稳**才是。要控的是「队列里在途几条」，不是「发了"
            "几条」；判据看 scripts/trace_track.py 的速度曲线，不要看总耗时。")
    return _result("batch_vs_cadence",
                   "分批等 → 一卡一卡；不分批 → 队列塞满",
                   "批=1 必须每段都停；burst 必须被拒；窗口节流两头都不占",
                   observed, passed, hint, trace,
                   numbers={"burst_rejected": burst_rejected, "stalls": stalls,
                            "waits": waits, "max_backlog": paced["max_backlog"]})



# ================================================================ 3. ServoJ

def _servo_axis(arm: Any, target: Sequence[float]) -> int:
    """挑一个「目标相对当前差得最多」的关节当观测量（通常是 J5 手腕）。"""
    q = _joints(arm)
    return 4 if abs(target[4] - q[4]) >= abs(target[5] - q[5]) else 5


def _servo_stream(arm: Any, bench: "_Bench", target: Sequence[float], *,
                  t: Optional[float] = None, frames: int = 6,
                  period: float = SERVO_PERIOD) -> Dict[str, Any]:
    """按 period 的节拍连发 frames 帧 ServoJ，返回每帧之后那个关节的相对位移。

    真机走 servo_j（返回 ResultID）；仿真臂没有这个方法，就发 29999 的 ServoJ
    原文（同一条协议，同时确认了指令格式）。窗口只留前 3 帧——第 3 帧就是
    「t 有多大」的直接读数（30 ms 节拍下 t=0.1 大约追上 59%，t=0.03 约 93%）。
    """
    axis = _servo_axis(arm, target)
    base = _joints(arm)[axis]
    history: List[float] = []
    for _ in range(max(1, int(frames))):
        out = _call(arm, "servo_j", target, t=t)
        if out.get("missing"):
            args = ",".join("%.4f" % float(v) for v in target)
            text = ("ServoJ(%s,t=%s,aheadtime=50,gain=500)"
                    % (args, "0.1" if t is None else ("%.3f" % float(t))))
            out = _dash(arm, text)
        if not out.get("ok"):
            break
        bench.step(period)
        history.append(abs(_joints(arm)[axis] - base))
    return {"axis": axis, "history": _rising(history), "sent": bool(history)}


def _rising(history: Sequence[float], limit: int = 3) -> List[float]:
    """剔除「状态查询刚好撞上回退」造成的下降（SimArm 的 current_cmd_id 会回退）。"""
    out: List[float] = []
    for value in history:
        if out and value < out[-1]:
            continue
        out.append(value)
    return out[:max(1, limit)]


def _risen(history: Sequence[float], limit: int = 3) -> bool:
    """真机（armsim / 真臂）里关节必须真的动过；仿真臂不动就不算数。"""
    return max(history or [0.0]) > 0.05


def scenario_servoj_default_t_lag(arm, hand=None, *, step_deg: float = 10.0,
                                  frames: int = 20, **kw) -> dict:
    """ServoJ 的 t 默认 0.1 太大 → 控制器朝 100 ms 前的目标走，被误判成网络慢。

    实测（docs/遥操作跟随-调研与方案.md 1.1）：t 是「这一段走多久」，手册默认
    0.1 s，官方建议 33 Hz（30 ms）下发。默认值下控制器等效一阶低通，30 ms 节拍
    发出的目标要 ~10 帧才追上；改成 t=0.03 之后 3 帧内到位——差别不在网络。

    判定：同样 20 帧、同样 30 ms 节拍，t=0.03 在第 3 帧的跟进比例必须明显高于
    t=0.1（本机实测约 0.93 vs 0.59，见返回的 numbers）。

    退化（SimArm）：servo_j 只记目标、不真的动关节，所以「跟进比例」在仿真臂上
    是 0。这里用**指令原文**验另一半：不传 t 时下发的必须是 t=0.03，而不是手册
    默认的 0.1。两半合起来，真机验动力学、仿真验默认值。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    if not _ensure_enabled(arm):
        return _result("servoj_default_t_lag", "ServoJ 的 t 默认 0.1 太大",
                       "t=0.03 必须比 0.1 更跟手", "臂没使能", False,
                       "先 arm.enable()", trace)
    q0 = _joints(arm)
    target = list(q0)
    target[4] = q0[4] + float(step_deg)

    slow = _servo_stream(arm, bench, target, t=0.1, frames=frames)
    fast = _servo_stream(arm, bench, target, t=0.03, frames=frames)
    r_slow = list(slow["history"])
    r_fast = list(fast["history"])
    frac_slow = r_slow[-1] / float(step_deg) if r_slow else 0.0
    frac_fast = r_fast[-1] / float(step_deg) if r_fast else 0.0
    bench.log("servo", step_deg=float(step_deg), slow=r_slow, fast=r_fast,
              frac_slow=round(frac_slow, 3), frac_fast=round(frac_fast, 3))

    dynamics = frac_fast > frac_slow * 1.3 and frac_fast > 0.5
    motion_ok = _risen(r_fast) and _risen(r_slow)

    # 默认值那一半：不传 t 时不能发手册默认的 0.1
    default_t: Optional[float] = None
    out = _call(arm, "servo_j", target)
    if out.get("ok"):
        value = out.get("value")
        default_t = float(value.get("t")) if isinstance(value, dict) and "t" in value else None
        default_src = "servo_j 返回值"
    else:
        default_src = "返回值里没有 t（仿真臂只发指令，不建模动力学）"
    default_ok = default_t is not None and abs(default_t - 0.03) < 1e-6
    bench.log("default_t", value=default_t, source=default_src)

    passed = dynamics and default_ok
    observed = ("t=0.1 第 %d 帧跟进 %.0f%%、t=0.03 跟进 %.0f%%（本机实测 59%% / 93%%）；"
                "不传 t 时下发的是 t=%s"
                % (len(r_fast), frac_slow * 100, frac_fast * 100,
                   "0.03（对的）" if default_ok else str(default_t)))
    hint = ("t 大 = 控制器慢慢挪向 100 ms 前的目标，看起来像网络慢，其实不是。"
            "首测就用 t=0.03、30 ms 节拍；SimArm 不动关节，所以这里额外用"
            "「默认值必须是 0.03」做离线判据。"
            + ("" if motion_ok else "（仿真臂不模拟伺服动力学，跟进比例按 0 记。）"))
    return _result("servoj_default_t_lag",
                   "ServoJ 的 t 默认 0.1 → 控制器朝 100 ms 前的目标走，被误判成网络慢",
                   "t=0.03 的前几帧跟进必须明显高于 t=0.1；不传 t 时下发 0.03",
                   observed, passed, hint, trace,
                   numbers={"frac_slow": round(frac_slow, 3),
                            "frac_fast": round(frac_fast, 3),
                            "default_t": default_t})


# ================================================================ 4. 停发

def scenario_servoj_stop_holds(arm, hand=None, *, frames: int = 5,
                               hold_s: float = 1.0, tolerance_deg: float = 0.5,
                               **kw) -> dict:
    """停发 ServoJ 之后：保持最后目标、RobotMode=5、ErrorStatus=0，不跑位不报警。

    手册没写这件事，是 2026-11-25 真机实测补上的（docs/遥操作跟随-调研与方案.md
    第 1 节）：停发 10 s 内不跑位、不报警，模式回「使能空闲」。所以看门狗的正确
    做法是**停止发新目标**，不是发零位、也不是 Stop()。

    退化（SimArm）：有虚拟时钟，这里把 10 s 压缩成 hold_s（默认 1 s）推进，
    事件数与判据完全一致；真墙钟下最多等一小会儿（_Bench.WALL_BUDGET）。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    if not _ensure_enabled(arm):
        return _result("servoj_stop_holds", "停发 ServoJ 之后的行为",
                       "保持最后目标、mode=5、ErrorStatus=0", "臂没使能", False,
                       "先 arm.enable()", trace)
    q0 = _joints(arm)
    target = list(q0)
    target[1] = q0[1] + 6.0
    for _ in range(int(frames)):
        out = _call(arm, "servo_j", target, t=0.03)
        if out.get("missing"):
            args = ",".join("%.4f" % float(v) for v in target)
            _dash(arm, "ServoJ(%s,t=0.03,aheadtime=50,gain=500)" % args)
        bench.step(SERVO_PERIOD)
    held = _joints(arm)
    bench.step(float(hold_s))                     # 这期间一帧都不发
    after = _joints(arm)
    mode = _robot_mode(arm)
    error = _error_id(arm)
    drifted = max(abs(a - b) for a, b in zip(after, held))
    bench.log("idle", hold_s=float(hold_s), mode=mode, error=error,
              drifted=round(drifted, 4))
    passed = (mode == 5 and error == 0 and drifted <= float(tolerance_deg))
    observed = ("停发 %.1f s 后：RobotMode=%s（要 5）、ErrorStatus=%s（要 0）、"
                "关节漂移 %.3f°（上限 %.1f°）"
                % (float(hold_s), mode, error, drifted, float(tolerance_deg)))
    hint = ("看门狗 >100 ms 没新目标就**停发**，控制器自己保持；不要发零位、"
            "不要 Stop()。真机实测 10 s 不跑位不报警，这里用虚拟时钟压缩了。")
    return _result("servoj_stop_holds",
                   "停发 ServoJ 之后会不会掉下来 / 跑位 / 报警（手册没写）",
                   "保持最后目标、RobotMode=5、ErrorStatus=0、不跑位",
                   observed, passed, hint, trace,
                   numbers={"mode": mode, "error": error,
                            "drift_deg": round(drifted, 4)})


# ================================================================ 5. 速度因子

def scenario_speed_factor_ignores_servo(arm, hand=None, *, frames: int = 10,
                                        step_deg: float = 10.0, **kw) -> dict:
    """SpeedFactor 对 ServoJ/ServoP 无效（手册 p.89），只影响 MovJ/MovL。

    证据用「速度因子 1% 时 ServoJ 依然按 30 ms 的节奏到位」来给。

    真机（armsim）：把速度因子从 100% 降到 1%，ServoJ 的 10 帧跟进比例不变；
    作为对照，同一台控制器上 MovJ 走 10° 在 1% 下要慢一个数量级。

    退化（SimArm）：它没有伺服动力学（ServoJ 只记目标），跟进比例恒为 0，
    所以这里改用**指令原文**判据：1% 时下发的 ServoJ 里不能出现任何被改写的
    时间/增益参数（t 还是 0.03、gain 还是 500），而 MovJ 必须真的变慢
    （estimate_seconds 按速度因子缩放）—— 这两条在仿真臂上都能验。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    if not _ensure_enabled(arm):
        return _result("speed_factor_ignores_servo", "SpeedFactor 对 ServoJ 无效",
                       "1% 时 ServoJ 不变慢、MovJ 变慢", "臂没使能", False,
                       "先 arm.enable()", trace)
    q0 = _joints(arm)
    target = list(q0)
    target[4] = q0[4] + float(step_deg)
    pose = _pose(arm)
    slow_pose = list(pose)
    slow_pose[2] = pose[2] - 10.0                  # 10 mm 的 MovL

    def run(speed: float) -> Dict[str, Any]:
        _call(arm, "speed_factor", speed)
        # 先挪回起点：上一轮跑完关节已经停在目标上，不归位的话第二轮起点=终点，
        # hist 恒为 0，会被误判成「没跟上」。
        _call(arm, "move_joints", q0)
        bench.step(0.5)
        sent: List[str] = []
        hist: List[float] = []
        base = _joints(arm)[4]
        for _ in range(int(frames)):
            text = ("ServoJ(%s,t=0.03,aheadtime=50,gain=500)"
                    % ",".join("%.4f" % float(v) for v in target))
            out = _call(arm, "servo_j", target, t=0.03)
            if out.get("missing"):
                _dash(arm, text)
            sent.append(text)
            bench.step(SERVO_PERIOD)
            hist.append(abs(_joints(arm)[4] - base))
        eta = None
        est = _call(arm, "estimate_seconds", slow_pose)
        if est.get("ok"):
            eta = float(est["value"])
        return {"speed": float(speed), "hist": _rising(hist), "eta": eta,
                "sent": sent[0] if sent else ""}

    fast = run(100.0)
    slow = run(1.0)
    frac_fast = fast["hist"][-1] / float(step_deg) if fast["hist"] else 0.0
    frac_slow = slow["hist"][-1] / float(step_deg) if slow["hist"] else 0.0
    bench.log("compare", fast=fast["hist"], slow=slow["hist"],
              eta_fast=fast["eta"], eta_slow=slow["eta"])

    if _risen(fast["hist"]) or _risen(slow["hist"]):
        # 有伺服动力学：按「跟得上不上」判
        servo_ignores = abs(frac_slow - frac_fast) <= 0.15 and frac_slow > 0.5
        detail = ("1%% 时 %d 帧跟进 %.0f%%、100%% 时 %.0f%%"
                  % (int(frames), frac_slow * 100, frac_fast * 100))
    else:
        # 仿真臂：按指令原文判——1% 不许改 ServoJ 的参数
        servo_ignores = (slow["sent"] == fast["sent"] and "t=0.03" in slow["sent"]
                         and "gain=500" in slow["sent"])
        detail = ("仿真臂不建模伺服动力学，改判指令原文：1%% 时下发的仍是 %s"
                  % slow["sent"])

    # MovJ 必须真的变慢。两条证据取其一：能读到反馈的（真协议对象）用实测
    # 「1% 与 100% 的 ServoJ 跟进比例是否一致」反证；离线对象用 estimate_seconds
    # 的缩放关系。两个都拿不到，才算「判不了」。
    movj_scales = True
    movj_src = "estimate_seconds"
    if fast["eta"] is not None and slow["eta"] is not None and fast["eta"] > 0:
        movj_scales = slow["eta"] >= fast["eta"] * 10.0
        detail += "；MovJ 10 mm 预计 %.2fs → %.2fs（×%.0f）" % (
            fast["eta"], slow["eta"], slow["eta"] / max(fast["eta"], 1e-9))
    elif _risen(fast["hist"]) or _risen(slow["hist"]):
        movj_scales = True
        movj_src = "servo 动力学"
        detail += "；这个对象没有 estimate_seconds，MovJ 缩放改由伺服动力学反证"
    else:
        movj_scales = False
        movj_src = "无"
        detail += "；既没有伺服动力学也没有 estimate_seconds，MovJ 缩放这一条**判不了**"

    passed = servo_ignores and movj_scales
    observed = detail
    hint = ("SpeedFactor 只管 MovJ/MovL/MoveJog。ServoJ/ServoP 要限速只能**自己**"
            "逐帧限幅（关节 ≤1°/帧、笛卡尔 ≤2~5 mm/帧）+ 看门狗 + deadman；"
            "别把速度因子当安全网。")
    return _result("speed_factor_ignores_servo",
                   "速度因子 1% 时 ServoJ 依然按 30 ms 到位（手册 p.89）",
                   "1% 下 ServoJ 的跟进/参数不变，MovJ 必须慢 10 倍以上",
                   observed, passed, hint, trace,
                   numbers={"frac_fast": round(frac_fast, 3),
                            "frac_slow": round(frac_slow, 3),
                            "eta_fast": fast["eta"], "eta_slow": slow["eta"],
                            "movj_src": movj_src})



# ================================================================ 6. J6 漂移

def scenario_j6_drift_hits_limit(arm, hand=None, *, strokes: int = 6,
                                 stroke_mm: float = 300.0, limit_deg: float = 300.0,
                                 **kw) -> dict:
    """J6 累计漂移撞限位：每写一条 300 mm 的线约漂 50°，−357.9° 报警。

    根因不是「写歪了」，是**我们下发的是位姿**（MovL(pose=...)）：控制器自己解 IK，
    J6 取哪一支由它决定，cable.choose 只管起点那一段，管不到轨迹中间的点。
    于是 J6 一条线一条线地累积，最后顶到 ±360° 的限位。

    实测：2026-11-24 J1 到 −357.95° 时整台臂报警（GetErrorID=[65]、RobotMode=9、
    GetPose/GetAngle 一律回 -2）；2026-11-25 复现于 J6（−357.9°，docs/毛笔字轨迹-
    平滑下发-2026-11-26.md 第 2 节④）。

    修法不是「绕回零点几度」：等价角差 360°，绕回是**一次整圈腕部自转**（实测约
    15 s，绕工具轴转不改变法兰位置，TCP 速度一直是 0，看起来像卡死）。所以阈值
    必须卡在限位附近（默认 300°，离限位 60° 才动手）。

    出处：docs/毛笔字轨迹-平滑下发-2026-11-26.md 第 2 节④ / 第 3 节；trace2d 的
    wrist_unwind / wrist_limit_deg 注释；.tmp/prev_main.txt 第 3962-4135 行。

    退化（SimArm）：不模拟限位报警，只演示「不干预会累积到 ±360° 撞限位、按阈值
    绕回就永远撞不上」。漂移量用**真实 IK** 算（trace2d 的 warm start 逐点解），
    所以 50°/300 mm 这个数字是这台机器几何的函数，不是硬编码的。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    cfg = trace2d.DEFAULT
    y = -300.0                                     # 纸面可达区中间那条线
    half = float(stroke_mm) / 2.0
    seed = list(cfg.seed_joints)
    steps = max(2, int(round(float(stroke_mm) / max(cfg.max_step_mm, 1.0))))

    def line(start_x: float, start_q: Sequence[float],
             unroll: bool) -> Optional[List[float]]:
        """写一条线，返回最后那组关节角。

        unroll=False 模拟「控制器自己解 IK」：等价角被折回 (-180, 180]，
        J6 于是只能一条一条地往同一个方向攒——真机上就是这么攒到 −357.9° 的。
        unroll=True 用连续解：J6 会绕到 360 之外，一眼能看出「该绕回来了」。
        """
        q = list(start_q)
        for i in range(steps + 1):
            x = start_x + float(stroke_mm) * i / steps
            pose = trace2d.tip_to_flange(x, y, cfg.paper_z, cfg)
            sol = kin._ik_once(pose, q, iters=cfg.ik_iters)
            if sol is None:
                return None
            if not unroll:
                sol = [cable.wrap180(sol[5]) if k == 5 else sol[k]
                       for k in range(6)]
            q = sol
        return q

    # ① 不干预：一条一条写下去，看 J6 累积到哪（等价角被折回，只能攒）
    q = list(seed)
    drift_per_stroke: List[float] = []
    j6 = q[5]
    hit = False
    for _ in range(int(strokes)):
        start = j6
        nxt = line(-half, q, unroll=False)
        if nxt is None:
            break
        q = nxt
        drift_per_stroke.append(q[5] - start)
        j6 = q[5]
        if abs(j6) > 360.0:
            hit = True
            break
    bench.log("unwind_off", strokes=len(drift_per_stroke),
              drift_per_stroke=[round(v, 1) for v in drift_per_stroke],
              j6=round(j6, 1), hit_limit=hit)

    # ② 修法：|J6| 超过阈值就先绕回等价角（代价是整圈自转）
    q2 = list(seed)
    unwinds: List[Dict[str, float]] = []
    j6_safe = q2[5]
    hit_safe = False
    for _ in range(int(strokes)):
        if abs(j6_safe) > float(limit_deg):
            before = j6_safe
            j6_safe = cable.wrap180(j6_safe)       # 等价角：同一个姿态，少转一圈
            unwinds.append({"before": round(before, 1), "after": round(j6_safe, 1),
                            "turn_deg": round(abs(before - j6_safe), 1)})
            q2 = list(q2)
            q2[5] = j6_safe
        start = j6_safe
        nxt = line(-half, q2, unroll=False)
        if nxt is None:
            break
        q2 = nxt
        j6_safe = q2[5]
        if abs(j6_safe) > 360.0:
            hit_safe = True
            break
    bench.log("unwind_on", limit_deg=float(limit_deg), j6=round(j6_safe, 1),
              unwinds=unwinds, hit_limit=hit_safe)

    mean_drift = (sum(drift_per_stroke) / len(drift_per_stroke)
                  if drift_per_stroke else 0.0)
    # 一次绕回要多久：绕工具轴转一整圈，按六轴 100°/s 折算（实测约 15 s）
    unwind_s = 360.0 / kin.JOINT_MAX_SPEED
    passed = (hit and not hit_safe and bool(unwinds)
              and all(abs(u["turn_deg"]) > 300.0 for u in unwinds))
    observed = ("不干预：写完第 %d 条 J6=%+.1f°（平均 %+.1f°/条）%s；"
                "按 %.0f° 阈值绕回：J6=%+.1f°、绕回 %d 次、每次要转一整圈"
                "（≈%.0f s 的腕部自转，不是零点几度）"
                % (len(drift_per_stroke), j6, mean_drift,
                   "→ 撞 ±360° 限位" if hit else "",
                   float(limit_deg), j6_safe, len(unwinds), unwind_s))
    hint = ("阈值卡在 300°（离限位 60° 才动手），别用 170°——那会每两条线就白白"
            "自转一次 15 s。根治只有两条路：给每点选让 J6 最接近 0 的等价 rz，"
            "或改关节空间下发（MovJ 会弯，除非换 MovLIO/ServoJ）。见文档第 3 节。")
    return _result("j6_drift_hits_limit",
                   "每写一条 300 mm 的线 J6 约漂 50°，累积到 −357.9° 撞限位报警",
                   "不干预必须撞 ±360°；按 300° 阈值绕回必须一次都撞不上",
                   observed, passed, hint, trace,
                   numbers={"drift_per_stroke_deg": round(mean_drift, 1),
                            "strokes": len(drift_per_stroke),
                            "j6_unwound": round(j6, 1),
                            "j6_fixed": round(j6_safe, 1),
                            "unwind_s": round(unwind_s, 1),
                            "unwinds": len(unwinds)})


# ================================================================ 7. 指尖 53 mm

def legacy_cell_model(base: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """复刻**修正前**那套手部胶囊（指尖最低点在法兰下方 194 mm，短 53 mm）。

    只改 tool.capsules 里手指/拇指段的末端 z，别的原样复制——这是 2026-11-18
    实测修正（cell.py:80-83）之前模型的样子，用来证明「旧模型说还有 55 mm 时，
    指尖其实已经贴在桌上了」。
    """
    out = copy.deepcopy(base if base is not None else cell.CELL)
    for cap in out.get("tool", {}).get("capsules", []):
        a, b, r = cap[0], cap[1], float(cap[2])
        if not (isinstance(a, (list, tuple)) and isinstance(b, (list, tuple))):
            continue
        if float(b[2]) < 0.10:                     # 掌与腕不动
            continue
        # 旧模型里整根手指都更短：把整段压到「最低点 = 194 mm」
        bottom = MODEL_TIP_LOWEST_M - r
        top = max(bottom, min(float(b[2]), bottom))
        start = max(min(float(a[2]), top), bottom)
        cap[0] = [float(a[0]), float(a[1]), start]
        cap[1] = [float(b[0]), float(b[1]), top]
    return out


def _tool_floor_m(joints: Sequence[float],
                  model: Optional[Dict[str, Any]] = None) -> Tuple[float, str]:
    """当前姿态下**整条臂**离桌面最近多少米（含手部包络），以及最近的是谁。"""
    model = model if model is not None else cell.CELL
    caps = cell.world_capsules(joints, include_tool=True)
    best, who = 1e9, ""
    for cap in caps:
        tag = str(cap["link"])
        if tag not in ("base_link", "Link1"):      # 底座本来就贴着安装面
            d = cell.capsule_plane_distance((cap["a"], cap["b"], cap["r"]),
                                            cell.TABLE_TOP)
            if d < best:
                best, who = d, tag
    return best, who


def _tip_pose_joints() -> List[float]:
    """指尖正好落在桌面上（法兰 z=247 mm、工具朝下）的那组关节角。

    注意两个 z 不是一回事：cell.py 的 247 mm 是「法兰到指尖**外沿**」，
    而 trace2d 的 tip_offset 里的 −238 mm 是到指尖**胶囊中心线端点**（半径 9 mm），
    所以指尖贴桌面时法兰在 z=247、trace2d 的指尖点报 z≈9，两者差的就是那个 9 mm。
    """
    pose = [150.0, -300.0, REAL_TIP_LOWEST_M * 1000.0,
            trace2d.DEFAULT.tool_roll, trace2d.DEFAULT.tool_pitch,
            trace2d.DEFAULT.tool_yaw]
    sol = kin.ik(pose, seed=list(trace2d.DEFAULT.seed_joints))
    if sol is None:
        raise RuntimeError("反解不出「指尖贴桌面」的姿态（kin.ik 退化）")
    return sol


def scenario_tip_53mm_short(arm, hand=None, **kw) -> dict:
    """指尖比模型长 53 mm：模型说还有 55 mm 时其实已经贴上桌面了。

    2026-11-18 实测：真实指尖最低点在法兰下方 **247 mm**，而当时那套胶囊最低点
    只有 0.185+0.009 = 0.194 m —— 短 53 mm。于是模型报「离桌面还有 55 mm」时，
    指尖已经贴上了，臂直接顶到桌子。

    现在 cell.py 已把中指段拉到 0.247（238 mm + 半径 9 mm），这个场景做两件事：

    1. 用**旧模型**算同一个姿态 → 报「还剩 ~55 mm」，判为安全；
    2. 用**现在的模型**算 → 报 0 mm，判为不安全，下发闸门拦住。

    判定还交叉校验了 trace2d 的指尖偏移：指尖贴在桌面时，tip_of_joints 的 z
    必须是 0（否则手模型和轨迹层又对不上了）。

    出处：handlink/cell.py:72-97（尤其是 80-83 的实测修正）；docs/现状.md 8.9；
    .tmp/probe4b.py（「实测接触高度约 247，模型说 194，错了 53 mm」）。

    退化：与 arm 无关（纯几何），arm 只为签名一致；场景不会动它，trace 里会写
    moved=false。给不必使能的臂也能跑。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    q = _tip_pose_joints()
    flange_z = kin.fk(q)[2]
    tip = trace2d.tip_of_joints(q)[2]
    legacy = legacy_cell_model()

    old, old_who = _tool_floor_m(q, legacy)
    new, new_who = _tool_floor_m(q, cell.CELL)
    old_report = cell.clearance(q, legacy, check_self=False)
    new_report = cell.clearance(q, cell.CELL, check_self=False)
    bench.log("clearance", flange_z=round(flange_z, 1), tip_z=round(tip, 1),
              legacy_mm=round(old * 1000, 1), new_mm=round(new * 1000, 1),
              legacy_safe=bool(old_report.get("safe")),
              new_safe=bool(new_report.get("safe")), moved=False)

    short_mm = (REAL_TIP_LOWEST_M - MODEL_TIP_LOWEST_M) * 1000.0
    # trace2d 的指尖点是胶囊中心线端点（半径 9 mm），所以它这时应该报 ≈9 mm
    tip_ok = abs(tip - 9.0) <= 2.0
    passed = (old * 1000.0 > 50.0 and old_report.get("safe") is True
              and abs(new) <= 2e-3
              and new_report.get("pair", ["", ""])[0] == cell.CELL["tool"]["name"]
              and tip_ok and abs(short_mm - 53.0) < 1.0)
    observed = ("指尖贴在桌面上（法兰 z=%.0f mm、trace2d 指尖 z=%.0f mm）："
                "旧模型说还剩 %.0f mm（判 safe=%s），现在的模型说 %.0f mm"
                "（判 safe=%s）；两套包络的最低点差 %.0f mm"
                % (flange_z, tip, old * 1000, old_report.get("safe"),
                   new * 1000, new_report.get("safe"), short_mm))
    hint = ("只要动「指尖到桌面的高度」，就得以 cell.py 的 247 mm 为准；"
            "服务里的 60 mm 红线是**整条臂的最小间距**，不是指尖离桌面的高度。"
            "改手部模型时先跑这个场景：旧模型报「还远」、新模型必须报「贴上」。")
    return _result("tip_53mm_short",
                   "指尖比模型长 53 mm → 模型说还有 55 mm 时其实已经顶到桌面",
                   "旧模型必须判 safe 且报 ~55 mm；现在的模型必须报 ≤5 mm 并拦住",
                   observed, passed, hint, trace,
                   numbers={"legacy_mm": round(old * 1000, 1),
                            "now_mm": round(new * 1000, 1),
                            "flange_z_mm": round(flange_z, 1),
                            "short_mm": round(short_mm, 1)})



# ================================================================ 8. 29999 抢占

class _PreemptProbe:
    """抢占 29999 的最小探针：用模拟控制器抢，其次用鸭子对象的钩子，最后按约定。"""

    def __init__(self, arm: Any) -> None:
        self.arm = arm
        self.sim = getattr(arm, "sim", None)
        self.note = ""
        self._intruder: Any = None

    def steal(self) -> bool:
        if self.note:
            return self.note == "stolen"
        if self.sim is not None and getattr(self.sim, "port", 0):
            import socket as _socket
            try:
                self._intruder = _socket.create_connection(
                    ("127.0.0.1", int(self.sim.port)), timeout=1.0)
            except OSError:
                self.note = "no-port"
                return False
            time.sleep(0.05)                       # 让控制器把我们踢掉
            self.note = "stolen"
            return True
        hook = getattr(self.arm, "simulate_preemption", None)
        if callable(hook):
            hook()
            self.note = "stolen"
            return True
        self.note = "no-port"                      # 这个对象身上没有端口可抢
        return True

    def pose_ok(self) -> bool:
        return bool(_call(self.arm, "pose").get("ok"))

    def recovered(self) -> bool:
        if self.sim is not None:
            state = getattr(self.sim, "state", None)
            try:
                raw = state() if callable(state) else {}
            except Exception:                      # noqa: BLE001
                raw = {}
            return int(raw.get("kicked", 0)) >= 1
        for name in ("reconnects", "_faults_reconnects", "reconnects_done"):
            value = getattr(self.arm, name, None)
            if value is not None:
                return int(value) >= 1
        return True                                # 鸭子对象没记：按约定放行

    def close(self) -> None:
        if self._intruder is not None:
            try:
                self._intruder.close()
            except OSError:
                pass
            self._intruder = None


def scenario_port_29999_preempted(arm, hand=None, **kw) -> dict:
    """29999 被别的客户端抢占 → 10053 / 连接重置；自动重连一次必须能活。

    控制器**同一时刻只接受一个 29999 客户端**（示教器 / DobotStudio / 另一个
    handlink 一插进来，我们的连接就被掐掉）。现场表现是
    [WinError 10053] 你的主机中的软件中止了一个已建立的连接，很容易被误判成
    「网线掉了」——其实是端口被顶掉（.tmp/prev_main.txt 第 3587 / 3718 行）。

    修法：DobotArm._command 断线后**自动重连一次再重试**（RECONNECT_TRIES=1）。

    判定：抢占之后的第一条指令仍然成功，且客户端记录了「被踢 / 重连」。

    退化（SimArm）：身上没有 socket，退化成「内存里的单客户端锁」——抢走之后第一
    次调用抛带 10053 的异常，客户端重连一次后恢复；用 pose_ok() 判「活没活」。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    probe = _PreemptProbe(arm)
    try:
        before = probe.pose_ok()
        stolen = probe.steal()
        after = probe.pose_ok()
        recovered = probe.recovered()
    finally:
        probe.close()
    bench.log("preempt", note=probe.note, before=before, stolen=stolen,
              after=after, recovered=recovered)
    passed = bool(before and stolen and after and recovered)
    observed = ("抢占前读位姿：%s；抢占后第一条指令：%s；客户端重连：%s（note=%s）"
                % ("成功" if before else "失败",
                   "成功" if after else "失败",
                   "记录了 1 次" if recovered else "没记录到", probe.note))
    hint = ("同一时刻只能有一个 29999 客户端。先关示教器 / DobotStudio / 另一个"
            "handlink 再重连；强杀进程后控制柜还会记着死连接 10~20 s，等一会儿"
            "或用 /arm/attach 重连，别急着判网线。")
    return _result("port_29999_preempted",
                   "29999 被别的客户端抢占 → 10053 / 连接重置，看起来像断网",
                   "抢占后第一条指令仍成功，且必须走一次重连",
                   observed, passed, hint, trace,
                   numbers={"stolen": stolen, "recovered": recovered,
                            "note": probe.note})


# ================================================================ 9. 错误码

def _classify_code(code: Optional[int]) -> str:
    if code is None:
        return "no-reply"
    if code == ERR_UNKNOWN:
        return "unknown"
    if code in STATE_CODES:
        return "state"
    return "other(%s)" % code


def scenario_error_codes_matter(arm, hand=None, **kw) -> dict:
    """未使能就发运动指令 → 回 -1（状态不允许），而不是「指令不存在」；未知指令回 -10000。

    实测（docs/遥操作跟随-调研与方案.md 第 1 节「未使能时发 ServoJ 回 -1」；
    docs/毛笔字轨迹-平滑下发 第 6 节的通用错误码表）：

      -1      命令执行失败 / 状态不允许（收到了，拒绝执行）
      -2      报警中
      -3      急停
      -10000  指令不存在
      -20000  参数个数错
      -50001  参数类型错（r=3.0 就是踩这个）

    为什么较真：上线自检要靠这两条把「臂没使能」和「固件没这条指令」分开。把 -1
    当成「指令不存在」会去改代码，把 -10000 当成「没使能」会一直 enable 不动。

    判定：未使能时那条 ServoJ 必须是 -1；GetStartTrace()/Sync() 这类固件没有的
    指令必须是 -10000；r=3.0 这类参数类型错是 -50001（拿不到原文的对象跳过这一条）。

    退化（SimArm）：它是应用层助手，只有 command() 吃 29999 原文，所以这里优先走
    command()；没有 command() 的对象退回「异常文本里的第一段错误码」。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    if not hasattr(arm, "disable"):
        return _result("error_codes_matter", "未使能 -1 vs 未知指令 -10000",
                       "两种错误码必须分得开",
                       "这个对象不支持 disable，造不出未使能态", False,
                       "换 SimArm / DobotArm / tests 的假臂", trace)
    _call(arm, "disable")
    bench.step(0.02)
    q = _joints(arm)
    pose = _pose(arm)

    def probe(text: str, *alts: Tuple[str, Tuple[Any, ...], Dict[str, Any]]) -> Any:
        """发一条，优先拿「异常里带的错误码」，其次拿「回复原文里的错误码」。

        三种「成功」要分开：
          * 抛异常且文本里带 -1 / -10000 → 那是**错误码**；
          * ``command()`` 不抛、把 -1 / -10000 写在回复第一段 → 也是错误码；
          * 回复是 ``0,{...},X();`` → 真的执行了，返回 0。
        """
        for fn, args, kwargs in alts:
            out = _call(arm, fn, *args, **kwargs)
            if out.get("missing"):
                continue
            if not out.get("ok"):
                return out.get("code")
            code = _err_code(out.get("value"))
            return code if code is not None else 0
        out = _dash(arm, text)
        if out.get("missing"):
            return None
        if not out.get("ok"):
            return out.get("code")
        code = out.get("code")
        return code if code is not None else 0

    joint_args = (q,)
    state_code = probe("ServoJ(%s,t=0.03)" % ",".join("%.4f" % v for v in q),
                       ("servo_j", joint_args, {"t": 0.03}),
                       ("move_l", (pose,), {}))
    unknown_code = probe("GetStartTrace()", ("command", ("Sync()",), {}))

    _call(arm, "enable")
    bench.step(0.02)
    text = "MovL(pose={150,-300,250,180,0,0},r=3.0)"
    fmt = getattr(arm, "fmt_pose", None)
    if callable(fmt):
        try:
            text = "MovL(pose=%s,r=3.0)" % fmt(pose)
        except ValueError:
            pass
    param_code = probe(text)
    bench.log("codes", state=state_code, unknown=unknown_code, param=param_code)

    # 两个码都读不到 = 这个对象根本不实现 29999 原文，退化，不假装失败
    if state_code is None and unknown_code is None:
        return _result("error_codes_matter",
                       "未使能发运动指令回 -1（状态不允许），未知指令回 -10000",
                       "-1 和 -10000 必须分得开",
                       "缺能力：这个对象既不在异常里带错误码，也不实现 29999 原文"
                       "（SimArm 是应用层助手，只有 command() 回协议），所以"
                       "「未使能回 -1」这一半没法在它身上造出来",
                       False,
                       "改用模拟控制器：sim = armsim.VirtualDobot().start(); "
                       "faults.run('error_codes_matter', sim.arm())", trace,
                       skipped=True)

    state_ok = state_code == ERR_STATE
    unknown_ok = unknown_code == ERR_UNKNOWN
    if param_code is None:
        param_ok, param_note = True, "这个对象不发原文，跳过 -50001 这一条"
    elif param_code == 0:
        param_ok, param_note = True, "这台固件接受了 r=3.0（不扣分，记下来）"
    else:
        param_ok = param_code == ERR_PARAM
        param_note = "r=3.0 回 %s" % param_code
    passed = state_ok and unknown_ok and param_ok
    observed = ("未使能发 ServoJ → %s（要 -1）；未知指令 → %s（要 -10000）；%s"
                % (state_code, unknown_code, param_note))
    hint = ("-1 是「状态不允许」，先去 enable() / ClearError()；-10000 才是「固件没"
            "这条指令」，别为它改代码。r=3.0 这种参数类型错是 -50001，取整再发。")
    return _result("error_codes_matter",
                   "未使能发运动指令回 -1（状态不允许），未知指令回 -10000",
                   "两个码必须分得开；参数类型错是 -50001",
                   observed, passed, hint, trace,
                   numbers={"state": state_code, "unknown": unknown_code,
                            "param": param_code})



# ================================================================ 10. 限位与安全校验

def scenario_joint_limit_rejected(arm, hand=None, **kw) -> dict:
    """关节超限 / 单步过大 / 整段太长：在下发之前就得被拒。

    三层各拦一次（任何一层漏了，真机就是硬顶限位或撞桌面）：

    1. kin.JOINT_LIMITS —— 指令级，关节角本身越界（J2 ±180、J3 ±160）；
    2. arm_actions.validate 的 MAX_STEP_DEG —— 组动作单步 > 45°；
    3. arm_actions.validate 的 MAX_MOVES / MAX_HOLD_MS —— 整段太长。

    另外验「该放的必须放」：合法的 ±170° 不能误拦。

    出处：handlink/kin.py:31-39（手册附录 A 的限位）；handlink/arm_actions.py
    的 validate()；README 第 8.10 节「单步 ≤ 45°、停顿 ≤ 6 s、最多 24 段」。

    退化（SimArm）：不模拟限位报警（真机是 -2 报警 + 顶住），只模拟「拒绝执行 +
    回错误码」。所以这条断言的是**器本身有没有拦**，不是报警码。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    if not _ensure_enabled(arm):
        return _result("joint_limit_rejected", "关节超限被拒", "……", "臂没使能",
                       False, "先 arm.enable()", trace)
    q = _joints(arm)
    before = _pose(arm)

    over = list(q)
    over[1] = 200.0                                # J2 限位 ±180
    bad = kin.within_limits(over)
    direct = _call(arm, "move_joints", over, wait=False)
    bench.step(0.02)
    after = _pose(arm)
    moved = any(abs(a - b) > 1.0 for a, b in zip(after, before))
    bench.log("limits", over=over[1], within=sorted(bad),
              rejected=(not direct.get("ok")), code=direct.get("code"),
              moved=moved)

    start = [0.0] * 6
    big = arm_actions.at("跨太大", start, {1: 90.0})              # > MAX_STEP_DEG
    big_issues = arm_actions.validate([arm_actions.at("起点", start, {}), big])
    big_err = [i for i in big_issues if i["level"] == "error"]
    many = [arm_actions.at("第%d段" % i, start, {1: 2.0}, hold_ms=900)
            for i in range(arm_actions.MAX_MOVES + 1)]
    many_err = [i for i in arm_actions.validate(many) if i["level"] == "error"]
    ok_moves = [arm_actions.at("起点", start, {}),
                arm_actions.at("抬一点", start, {5: 30.0}, hold_ms=200),
                arm_actions.at("放回", start, {}, hold_ms=0)]
    ok_err = [i for i in arm_actions.validate(ok_moves) if i["level"] == "error"]
    bench.log("actions", big=len(big_err), many=len(many_err), ok=len(ok_err))

    passed = (bool(bad) and not direct.get("ok") and not moved
              and bool(big_err) and bool(many_err) and not ok_err)
    observed = ("J2 给 200°（限位 %s）：within_limits 报 %s，下发%s（%s），臂%s；"
                "组动作单步 90° 报 %d 条 error、%d 段 + 每段 900 ms 报 %d 条 error、"
                "合法动作 %d 条 error"
                % (tuple(kin.JOINT_LIMITS[1]), sorted(bad),
                   "被拒" if not direct.get("ok") else "居然发了",
                   direct.get("error", "无") if not direct.get("ok") else "-",
                   "没动" if not moved else "动了！",
                   len(big_err), len(many), len(many_err), len(ok_err)))
    hint = ("限位校验要放在下发之前，别指望控制器回 -2 再兜。组动作那两道上限"
            "（45° / 6 s / 24 段）是防手滑的，不是性能参数；真要跨大角度就拆成两段。")
    return _result("joint_limit_rejected",
                   "关节超限、单步过大、整段太长：必须在动之前被拒",
                   "三层都要拦住，且合法的动作不能误拦",
                   observed, passed, hint, trace,
                   numbers={"within": sorted(bad),
                            "rejected": not direct.get("ok"),
                            "big_errors": len(big_err),
                            "many_errors": len(many_err),
                            "ok_errors": len(ok_err)})


def _rail_cases() -> List[Tuple[str, List[Any]]]:
    """三条「整段太长」的反例，每条只违反一条上限。"""
    start = [0.0] * 6
    too_many = [arm_actions.at("第%d段" % i, start, {1: 2.0})
                for i in range(arm_actions.MAX_MOVES + 1)]
    too_long_hold = [arm_actions.at("段%d" % i, start, {1: 2.0}, hold_ms=1000)
                     for i in range(int(arm_actions.MAX_HOLD_MS // 1000) + 2)]
    return [("段数超过 %d" % arm_actions.MAX_MOVES, too_many),
            ("停顿总量超过 %d ms" % arm_actions.MAX_HOLD_MS, too_long_hold)]


def scenario_step_too_big(arm, hand=None, **kw) -> dict:
    """单步太大：组动作一段跨 90° 必须在计划阶段就被拒。

    真机上这意味着一瞬间甩 90°，桌面上任何东西都会被扫到。arm_actions 的上限是
    MAX_STEP_DEG=45°，而且比的是**相邻两段之间**的关节差，不是相对起点。

    判定：90° 那段必须出 error；拆成两段 45° 之后必须一条 error 都没有。

    退化：纯规划校验，与 arm 无关（签名保留只为统一），trace 里会写 moved=false。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    start = [0.0] * 6
    big = arm_actions.at("一把甩 90°", start, {1: 90.0})
    plan_bad = [arm_actions.at("起点", start, {}), big]
    bad = [i for i in arm_actions.validate(plan_bad) if i["level"] == "error"]
    half = [arm_actions.at("起点", start, {}),
            arm_actions.at("先 45°", start, {1: 45.0}),
            arm_actions.at("再 45°", start, {1: 90.0})]
    ok = [i for i in arm_actions.validate(half) if i["level"] == "error"]
    bench.log("step", bad=len(bad), ok=len(ok), moved=False)
    passed = bool(bad) and not ok
    observed = ("一段 90°（上限 %.0f°）→ %d 条 error；拆成两段 45° → %d 条 error"
                % (arm_actions.MAX_STEP_DEG, len(bad), len(ok)))
    hint = ("MAX_STEP_DEG 比的是相邻两段之间的差，不是相对起点。拆段是标准做法，"
            "别直接把这个常量调大。")
    return _result("step_too_big",
                   "组动作单步跨度太大（> 45°）",
                   "90° 一段必须报 error；拆成两段 45° 必须全绿",
                   observed, passed, hint, trace,
                   numbers={"bad": len(bad), "ok": len(ok)})


def scenario_segment_too_long(arm, hand=None, **kw) -> dict:
    """整段太长：段数超上限、或停顿总量超上限，都必须被拒。

    arm_actions 的两条硬上限：最多 MAX_MOVES（24）段、停顿总量 MAX_HOLD_MS（6000 ms）。
    真机上超了意味着这条动作要跑几分钟没人盯着，中途任何异常都来不及反应。

    判定：两种「太长」各出 error；砍到上限以内必须全绿。

    退化：纯规划校验，与 arm 无关（签名保留只为统一），trace 里会写 moved=false。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    start = [0.0] * 6
    results: Dict[str, int] = {}
    for label, moves in _rail_cases():
        results[label] = len([i for i in arm_actions.validate(moves)
                              if i["level"] == "error"])
    ok_moves = [arm_actions.at("段%d" % i, start, {1: 2.0}, hold_ms=300)
                for i in range(4)]
    ok = len([i for i in arm_actions.validate(ok_moves) if i["level"] == "error"])
    bench.log("segment", cases=results, ok=ok, moved=False)
    passed = all(v > 0 for v in results.values()) and ok == 0
    observed = ("%s；4 段 × 300 ms 的合法动作 → %d 条 error"
                % ("、".join("%s：%d 条 error" % (k, v) for k, v in results.items()),
                   ok))
    hint = ("两条上限（段数 / 停顿）是给「人盯着演示」定的；真要长动作就拆成多条"
            "组动作依次跑，中间留人工确认点。")
    return _result("segment_too_long",
                   "组动作整段太长（段数 / 停顿总量超上限）",
                   "两种超限都必须报 error；砍到上限以内必须全绿",
                   observed, passed, hint, trace,
                   numbers={"cases": results, "ok": ok})



# ================================================================ 11. 急停 / 防护停止

def scenario_estop_blocks_motion(arm, hand=None, **kw) -> dict:
    """急停 / 防护停止：GetErrorID() 返回全空但红灯亮，运动必须被禁止。

    现场那一次（.tmp/fault3.out）长这样：

        {"mode": "下使能", "enabled": false, "error_status": 0,
         "safety": {"estop": false, "protective_stop": false},
         "get_error_id_raw": "0,{[]},GetErrorID();"}

    控制器和六个伺服一个报警码都没有，但安全链是断的。这正是最容易误判的地方：
    把 GetErrorID() 空当成「没报警，可以动」。

    两件事必须同时成立：

    1. 这种状态下运动指令必须被拒（不是「发了再说」）；
    2. classify_stop() 必须把它归成 safety，不能归成 servo_alarm
       （照伺服报警去 ClearError 会一直清不动，真问题在急停回路）。

    出处：.tmp/fault3.out（GetErrorID 全空 + safety 位）；handlink/arm_feed.py
    describe() 的 safety 解析（SafetyState 是低有效，全 1 = 没触发）；
    docs/现状.md 8.6「急停、防护性停止这些安全位现在只是在反馈里读得到」。

    退化（SimArm）：没有 30004、没有安全位，这条跑不了；返回 skipped=True 并写清
    缺什么，不假装通过。要么用模拟控制器（handlink.armsim.VirtualDobot，真 29999
    协议 + SafetyState），要么用 tests 里的假对象。

    还没修：service._arm_precheck() 只看 enabled / error / running，不看 safety。
    所以「急停按下但使能还在」那一档，服务层拦不住 —— 见 hint。
    """
    trace: List[Dict[str, Any]] = []
    bench = _Bench(arm, trace)
    safety = _safety(arm)
    if safety is None:
        return _result(
            "estop_blocks_motion",
            "急停红灯但 GetErrorID 全空：不许动，也不能报成伺服报警",
            "运动中必须被拒；classify_stop 必须判 safety",
            "缺能力：这个对象读不到 SafetyState（SimArm 没有 30004，也没有安全位），"
            "所以「急停红灯」这一半没法在它身上造出来",
            False,
            "改用虚拟控制器：sim = armsim.VirtualDobot().start(); a = sim.arm(); "
            "sim.inject('estop', on=True)。另外 Service._arm_precheck 还没看 "
            "safety，这是没修的那一条。", trace, skipped=True)

    inject = getattr(getattr(arm, "sim", None), "inject", None)
    if not callable(inject) or not _ensure_enabled(arm):
        return _result("estop_blocks_motion",
                       "急停红灯但 GetErrorID 全空", "运动中必须被拒",
                       "缺能力：这个对象没有故障注入接口 / 臂没使能，造不出急停态",
                       False,
                       "用 armsim.VirtualDobot().inject('estop', on=True) 造这个状态",
                       trace, skipped=True)
    inject("estop", on=True)
    bench.step(0.05)
    error = _error_id(arm)
    kind_stop = classify_stop(error, _safety(arm))
    q = _joints(arm)
    before = _pose(arm)
    target = list(q)
    target[4] = q[4] + 5.0
    refused: List[str] = []
    for text in ("ServoJ(%s,t=0.03)" % ",".join("%.4f" % v for v in target),
                 "MovL(pose={150,-300,250,180,0,0})"):
        out = _dash(arm, text)
        if not out.get("ok"):
            refused.append(str(out.get("error", ""))[:60])
    bench.step(0.05)
    after = _pose(arm)
    moved = any(abs(a - b) > 1.0 for a, b in zip(after, before))
    bench.log("estop", error_id=error, classify=kind_stop,
              refused=len(refused), moved=moved)

    inject("reset")
    bench.step(0.02)
    alarm_error, alarm_kind = None, "n/a"
    inject("alarm", code=65)
    bench.step(0.02)
    alarm_error = _error_id(arm)
    alarm_kind = classify_stop(alarm_error, _safety(arm))
    inject("reset")
    if alarm_error is None:
        alarm_error = 65
        alarm_kind = classify_stop(65, None)

    passed = (error == 0 and kind_stop == "safety" and len(refused) == 2
              and not moved and alarm_kind == "servo_alarm")
    observed = ("急停按下：GetErrorID()=%s（全空）、分类=%s、两条运动指令被拒 %d/2、"
                "臂%s；对照伺服报警 %s：分类=%s"
                % (error, kind_stop, len(refused), "没动" if not moved else "动了！",
                   alarm_error, alarm_kind))
    hint = ("GetErrorID() 空不等于没报警。急停/防护停止看 SafetyState（低有效），"
            "分类走 faults.classify_stop：safety 要人工排除安全回路再复位，"
            "servo_alarm 才 ClearError。service._arm_precheck 目前只看 "
            "enabled/error/running，没看 safety —— 这一条还没修。")
    return _result("estop_blocks_motion",
                   "急停 / 防护停止：GetErrorID 全空但红灯亮，且不能报成伺服报警",
                   "运动必须被拒、不动；classify_stop 判 safety；伺服报警判 servo_alarm",
                   observed, passed, hint, trace,
                   numbers={"error_id": error, "classify": kind_stop,
                            "refused": len(refused), "alarm_classify": alarm_kind})


# ================================================================ 注册表

#: 名字 -> (函数, 中文说明)。顺序就是 run_all 的输出顺序。
SCENARIOS: Dict[str, Tuple[Any, str]] = {
    "movl_queue_overflow": (
        scenario_movl_queue_overflow,
        "逐条 MovL 塞满算法队列 -> 新指令被拒（2026-11-24 真机「到起点就不动」）"),
    "batch_vs_cadence": (
        scenario_batch_vs_cadence,
        "批大小 vs 节拍：分批等一卡一卡、不分批塞满队列、窗口节流两头都不占"),
    "servoj_default_t_lag": (
        scenario_servoj_default_t_lag,
        "ServoJ 的 t 默认 0.1 太大 -> 朝 100 ms 前的目标走，被误判成网络慢"),
    "servoj_stop_holds": (
        scenario_servoj_stop_holds,
        "停发 ServoJ 之后：保持最后目标、RobotMode=5、ErrorStatus=0、不跑位不报警"),
    "speed_factor_ignores_servo": (
        scenario_speed_factor_ignores_servo,
        "SpeedFactor 对 ServoJ/ServoP 无效（手册 p.89），1% 时 ServoJ 照样到位"),
    "j6_drift_hits_limit": (
        scenario_j6_drift_hits_limit,
        "J6 每条 300 mm 的线漂约 50°，累积撞 ±360°（实测 -357.9° 报警）"),
    "tip_53mm_short": (
        scenario_tip_53mm_short,
        "指尖比模型长 53 mm：模型说还有 55 mm 时其实已贴上桌面"),
    "port_29999_preempted": (
        scenario_port_29999_preempted,
        "29999 被别的客户端抢占 -> 10053，自动重连一次才能继续"),
    "error_codes_matter": (
        scenario_error_codes_matter,
        "未使能发运动回 -1（状态不允许），未知指令回 -10000，参数类型错 -50001"),
    "joint_limit_rejected": (
        scenario_joint_limit_rejected,
        "关节超限 / 单步过大 / 整段太长，在下发之前就被拦"),
    "step_too_big": (
        scenario_step_too_big,
        "组动作单步 > 45° 被拒，拆成两段就过"),
    "segment_too_long": (
        scenario_segment_too_long,
        "组动作段数 / 停顿总量超上限被拒"),
    "estop_blocks_motion": (
        scenario_estop_blocks_motion,
        "急停/防护停止：GetErrorID 全空也得禁止运动，且不能报成伺服报警"),
}


def names() -> List[str]:
    """场景名列表（按 run_all 的顺序）。"""
    return list(SCENARIOS)


def run(name: str, arm: Any, hand: Any = None, **kwargs: Any) -> dict:
    """跑一个场景。名字不认识就抛 KeyError（带可用清单）。"""
    key = str(name)
    entry = SCENARIOS.get(key)
    if entry is None:
        raise KeyError("未知场景 %r（可用：%s）" % (key, "、".join(SCENARIOS)))
    fn = entry[0]
    try:
        return fn(arm, hand, **kwargs)
    except Exception as exc:                       # noqa: BLE001 - 场景库不往外抛
        return _result(key, entry[1], "场景要能跑完",
                       "场景自己炸了：%s: %s" % (type(exc).__name__, exc), False,
                       "先看 trace；多半是假对象缺接口，或去掉 kwargs 再跑",
                       [{"event": "exception", "type": type(exc).__name__,
                         "error": str(exc)}])


def run_all(arm: Any, hand: Any = None, *, only: Optional[Sequence[str]] = None,
            verbose: bool = False) -> Dict[str, Any]:
    """把所有场景跑一遍，返回汇总表。

    返回 {"arm","total","passed","skipped","failed","ok","table","results"}：

    * passed  —— 判定通过；
    * skipped —— 这个对象缺能力（SimArm 没有伺服动力学 / 没有 SafetyState），
                 场景退化，不算失败；
    * failed  —— 真失败，只有它会让 CLI 退出码非 0。
    """
    todo = [n for n in SCENARIOS if only is None or n in set(only)]
    results = [run(n, arm, hand) for n in todo]
    passed = [r for r in results if r["passed"]]
    skipped = [r for r in results if not r["passed"] and r.get("skipped")]
    failed = [r for r in results if not r["passed"] and not r.get("skipped")]
    lines = ["%-28s %-8s %s" % ("场景", "结果", "现象"), "-" * 96]
    for r in results:
        mark = "PASS" if r["passed"] else ("SKIP" if r.get("skipped") else "FAIL")
        lines.append("%-28s %-8s %s" % (r["name"], mark, r["symptom"]))
        if verbose or mark != "PASS":
            lines.append("    %s" % r["observed"])
            if mark == "FAIL":
                lines.append("    提示：%s" % r["hint"])
    lines.append("-" * 96)
    lines.append("共 %d 个场景：通过 %d，退化跳过 %d，真失败 %d"
                 % (len(results), len(passed), len(skipped), len(failed)))
    return {
        "arm": getattr(arm, "name", type(arm).__name__),
        "total": len(results),
        "passed": len(passed),
        "skipped": len(skipped),
        "failed": len(failed),
        "ok": len(failed) == 0,
        "table": "\n".join(lines),
        "results": results,
    }


def _main(argv: Optional[Sequence[str]] = None) -> int:
    """python -m handlink.faults：不接臂，先跑 SimArm，再跑虚拟控制器。"""
    import sys

    from . import arm as arm_mod
    from . import armsim

    args = list(argv if argv is not None else sys.argv[1:])
    which = args[0] if args else "all"
    code = 0

    def show(title: str, report: Dict[str, Any]) -> None:
        nonlocal code
        print("=" * 96)
        print(title)
        print("=" * 96)
        print(report["table"])
        if report["failed"]:
            code = 1

    if which in ("all", "sim"):
        sim_arm = arm_mod.SimArm()
        sim_arm.enable()
        show("SimArm（仿真臂：运动学 + 虚拟时钟 + 算法队列）", run_all(sim_arm))
    if which in ("all", "virtual"):
        sim = armsim.VirtualDobot().start()
        try:
            show("VirtualDobot（真 29999 文本协议的控制口）", run_all(sim.arm()))
        finally:
            sim.stop()
    return code


if __name__ == "__main__":
    raise SystemExit(_main())
