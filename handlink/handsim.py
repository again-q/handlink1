"""灵巧手总线模型：比 :class:`handlink.bus.SimBus` 更接近真机。

为什么需要它
------------
``SimBus`` 是「一阶位置伺服 + 拍脑袋估流」：position 直接按 speed*dt 走，
没有加减速、没有电流约束、没有丢帧、没有校准，凡是真机上会翻车的地方它都是绿
的。集成测试（动作编排、抓取力控、心跳看门狗、适配器抢占）需要一棵**会失败**的
树，于是有了 ``HandSim``。

模型的数字来源，逐条标注：

=============== ==========================================================
项目            依据
=============== ==========================================================
speed→°/s       [实测] docs/现状.md 第 2 节：speed=200 → 约 55°/s，
                90° 行程 1.6 s、2 s 内稳定
加减速段        [推断] 梯形速度规划，加速度上限 2400 °/s²（0→56°/s 约 23 ms），
                制动速度 v=√(2a·误差)，所以有加速段、不瞬移、到位不爬行
电流需求        [实测] 0.6 A 多指同动触发 I-Lim 卡在 118°；≥1.0 A 走完。
                据此反解 i_req = 0.10 + 0.00411·angle + 0.005·(n_moving-1)
电流上限        [手册] 拇指 1/2 = 0.8 A，其余 3.5 A（spec.MOTORS.current_max）
心跳            [实测] 每指约 500 Hz，六指合计约 3000 帧/秒
心跳位          [手册] 0x01 运行、0x10 电流限幅、0x20 位置限幅、
                0x02/0x04 硬/软过流、0x08 电压异常；calib & 0x08 = 校准成功
重复性          [实测] 回零后落在 0.5–1.5°
断电            [实测] 断 24V 手指变软、没有保持力，再上电要重新校准
适配器          [实测] 同一时刻只有一个进程能占用 USBCAN 适配器
下发延迟        [实测] 单帧 Transmit 1–15 ms（带抖动，可丢帧）
=============== ==========================================================

用法
----
    sim = HandSim(seed=7)
    sim.open()
    sim.attach()                      # 独占适配器
    sim.apply(spec.blank_pose(speed=200, torque=1.5), reason="home")
    for _ in range(1000):
        sim.step(0.002)               # 虚拟时间推进，测试里不要真 sleep

    sim.inject("power_off")           # 断电变软
    sim.inject("can_loss", rate=0.3)  # 三成丢帧
    sim.inject("undervoltage", seconds=2.0)
    sim.inject("stall", motor=103)
    sim.inject("uncalibrated")

所有状态推进都在 :meth:`step` 里，``open()`` **不**起后台线程，因此测试是
确定性的：同一个 seed、同一串 step，结果逐位相同。
"""
from __future__ import annotations

import math
import random
import threading
from typing import Dict, Optional

from . import bus, spec

__all__ = ["HandSim"]

# ---------------------------------------------------------------- 运动参数
# [实测] speed=200 → 90° 行程 1.6 s，即平均约 56.3°/s；巡航段 55–57°/s
# （现状文档的「约 55°/s」是取整值，二者差 3%，这里按行程时间标定）。
SPEED_DPS_PER_UNIT = 56.5 / 200.0
# [推断] 加速度上限（°/s²）：0→56°/s 约 23 ms，给出可见的加减速段，
# 配合下面的制动曲线就是梯形速度规划，不会瞬移也没有速度阶跃。
ACC_MAX = 2400.0
# [推断] 静摩擦 + 传动阻力折算的恒定电流（A）。
I_FRIC = 0.10
# [实测拟合] 弹簧/腱绳的回复力矩折算到电流：0.6 A 时正好在 118° 顶住。
I_SPRING_PER_DEG = 0.00411
# [推断] 同动耦合：每多一根手指在动，电源/总线上的压降多摊 5 mA。
I_COUPLING = 0.005
# [推断] 断电后位置塌陷的时间常数（s）：1 s 掉到 44%，6 s 基本躺平。
SAG_TAU = 1.2
# [实测] 单帧下发延迟 1–15 ms。
LATENCY_MIN = 0.001
LATENCY_MAX = 0.015
# [实测] 真机上电到校准完成约 10 s；仿真缩短到 0.6 s，测试才跑得完。
CALIB_DELAY = 0.6
# [推断] 校准落点范围（°），[实测] 回零后 0.5–1.5°。
REPEAT_MIN = 0.5
REPEAT_MAX = 1.5
# 仿真内部最大子步长（s），保证 ωn·h 足够小、大步长调用也稳定。
MAX_SUBSTEP = 0.002

# 心跳状态位（[手册] 协议速查）
BIT_RUNNING = 0x01      # 运行
BIT_HW_OC = 0x02        # 硬件过流
BIT_SW_OC = 0x04        # 软件过流
BIT_UNDERVOLT = 0x08    # 电压异常
BIT_I_LIMIT = 0x10      # 电流限幅（I-Lim）
BIT_POS_LIMIT = 0x20    # 位置限幅

# 适配器是**进程级独占**的（真机上 OpenDevice 返回 0 就是有人占着），
# 所以放在模块级而不是实例级：两个 HandSim 也不能同时 attach。
_ADAPTER_LOCK = threading.Lock()
_ADAPTER_OWNER: Optional["HandSim"] = None


class HandSim(bus.HandBus):
    """六自由度灵巧手的总线级仿真（继承 :class:`bus.HandBus`）。"""

    name = "handsim"

    #: 供测试/调用方读的默认常数（与模块级常量同值）
    SPEED_DPS_PER_UNIT = SPEED_DPS_PER_UNIT

    def __init__(self, *, seed: int = 0, hz: float = 500.0, can_loss: float = 0.0) -> None:
        """
        参数
        ----
        seed      : 随机种子。校准落点、下发延迟抖动、丢帧都由它派生，可复现。
        hz        : 每指心跳频率（[实测] 约 500 Hz，六指合计约 3000 帧/秒）。
        can_loss  : 初始丢帧率 0–1（[实测] CAN 上会丢帧，默认 0 便于日常开发）。
        """
        self.seed = int(seed)
        self.hz = float(hz)
        self.can_loss = _clamp(float(can_loss), 0.0, 1.0)

        self._lock = threading.RLock()
        self._ids = tuple(m.motor_id for m in spec.MOTORS)

        # 两条独立的随机流：校准落点 / 网络（延迟 + 丢帧），互不干扰。
        self._rng_cal = random.Random(self.seed * 7919 + 13)
        self._rng_net = random.Random(self.seed * 104729 + 7)

        # 虚拟时钟（秒）。测试靠 :meth:`step` 推进它，不依赖真实时间。
        self.t = 0.0

        self._opened = False
        self._attached = False
        self._powered = False
        self._calibrated = False
        self._calib_t = 0.0          # >0 表示正在等校准完成（倒计时）

        self._pos = {mid: 0.0 for mid in self._ids}
        self._vel = {mid: 0.0 for mid in self._ids}
        self._target_cmd = {mid: 0.0 for mid in self._ids}   # 下发的原始角度
        self._offset = {mid: 0.0 for mid in self._ids}       # 校准零偏（0.5–1.5°）
        self._speed = {mid: spec.FIRST_RUN_SPEED for mid in self._ids}
        self._limit = {mid: 0.3 for mid in self._ids}        # 电流上限（A）
        self._cur = {mid: 0.0 for mid in self._ids}
        self._limited = {mid: False for mid in self._ids}    # 本步是否被 I-Lim 顶住
        self._move_at = {mid: 0.0 for mid in self._ids}      # 延迟后的起始时刻

        self._stall = set()          # inject("stall") 的电机
        self._pos_limit = set()      # 下发角度超 pos_max（粘住到下次 apply）
        self._soft_oc = set()        # 下发电流超硬件上限（粘住到下次 apply）
        self._undervolt_until: Optional[float] = None

        self._hb_acc = {mid: 0.0 for mid in self._ids}
        self._hb_count = {mid: 0 for mid in self._ids}
        self._hb_total = 0

        self._tx = 0                 # 下发帧计数
        self._tx_lost = 0            # 丢帧计数
        self._applies = 0
        self._last_reason = ""

    # ------------------------------------------------------------ 生命周期
    def open(self) -> None:
        """打开总线（幂等）。上电即视为校准完成，零偏按 seed 抽样。"""
        with self._lock:
            if self._opened:
                return
            self._opened = True
            self._powered = True
            self._calibrate()

    def close(self) -> None:
        """关闭总线，同时释放适配器（[实测] 关掉进程别人才能占用）。"""
        with self._lock:
            self._opened = False
            self._detach_locked()
            for mid in self._ids:
                self._vel[mid] = 0.0

    # ------------------------------------------------------------ 适配器独占
    def attach(self) -> None:
        """独占适配器。已被占用（本实例或别的实例）时抛 :class:`RuntimeError`。"""
        global _ADAPTER_OWNER
        with _ADAPTER_LOCK:
            if _ADAPTER_OWNER is not None and _ADAPTER_OWNER is not self:
                raise RuntimeError("适配器已被占用：同一时刻只能有一个进程占用 USBCAN")
            if self._attached:
                raise RuntimeError("适配器已被本会话占用：不要重复 attach")
            _ADAPTER_OWNER = self
            self._attached = True

    def detach(self) -> None:
        """释放适配器（幂等）。close() 也会释放。"""
        with _ADAPTER_LOCK:
            self._detach_locked()

    def _detach_locked(self) -> None:
        global _ADAPTER_OWNER
        if _ADAPTER_OWNER is self:
            _ADAPTER_OWNER = None
        self._attached = False

    @property
    def attached(self) -> bool:
        return self._attached

    # ------------------------------------------------------------ 指令下发
    def apply(self, pose: spec.Pose, reason: str = "") -> Dict[str, object]:
        """下发姿态。

        每帧带 1–15 ms 的延迟抖动（[实测]），按 ``can_loss`` 概率整帧丢弃
        （[实测] CAN 会丢帧，丢了就是丢了，电机保持旧目标）。被钳位的项会在返回
        值 ``clamped`` 里列出来，同时点亮心跳的 0x20 / 0x04 位。
        """
        if not self._opened:
            self.open()
        unknown = [mid for mid in pose if mid not in spec.MOTOR_BY_ID]
        if unknown:
            raise ValueError(f"未知电机: {unknown}")

        with self._lock:
            latency = self._rng_net.uniform(LATENCY_MIN, LATENCY_MAX)
            dropped = self._rng_net.random() < self.can_loss
            self._applies += 1
            self._last_reason = reason
            base = {
                "ok": True,
                "bus": self.name,
                "reason": reason,
                "latency_ms": round(latency * 1000.0, 3),
                "dropped": bool(dropped),
                "lost_frames": self._tx_lost,
                "applied": [],
                "clamped": [],
            }
            if dropped:
                self._tx_lost += 1
                base["lost_frames"] = self._tx_lost
                return base

            self._tx += 1
            start = self.t + latency
            for mid, joint in pose.items():
                m = spec.MOTOR_BY_ID[mid]
                angle = float(joint.angle)
                speed = float(joint.speed)
                torque = float(joint.torque)

                if angle < 0.0 or angle > m.pos_max:
                    self._pos_limit.add(mid)
                    base["clamped"].append({"motor_id": mid, "field": "angle",
                                            "value": angle, "limit": m.pos_max})
                    angle = _clamp(angle, 0.0, m.pos_max)
                else:
                    self._pos_limit.discard(mid)

                if torque < 0.0 or torque > m.current_max:
                    # [手册] 越过硬件电流上限：钳位并点亮 0x04（软件过流）
                    self._soft_oc.add(mid)
                    base["clamped"].append({"motor_id": mid, "field": "torque",
                                            "value": torque, "limit": m.current_max})
                    torque = _clamp(torque, 0.0, m.current_max)
                else:
                    self._soft_oc.discard(mid)

                if speed < 0.0 or speed > spec.SPEED_MAX:
                    base["clamped"].append({"motor_id": mid, "field": "speed",
                                            "value": speed, "limit": spec.SPEED_MAX})
                    speed = _clamp(speed, 0.0, spec.SPEED_MAX)

                self._target_cmd[mid] = angle
                self._limit[mid] = torque
                self._speed[mid] = speed
                self._move_at[mid] = start
                base["applied"].append(mid)

            base["applied"].sort()
            return base

    def stop(self) -> None:
        self.apply(spec.blank_pose(speed=spec.FIRST_RUN_SPEED, torque=1.0), reason="stop")

    # ------------------------------------------------------------ 状态查询
    def state(self) -> Dict[int, bus.MotorState]:
        with self._lock:
            out = {}
            for mid in self._ids:
                out[mid] = bus.MotorState(
                    motor_id=mid,
                    angle=round(self._pos[mid], 3),
                    speed=round(abs(self._vel[mid]), 3),
                    current=round(self._cur[mid], 3),
                    target=round(self._target_cmd[mid], 3),
                    error=self._error(mid),
                )
            return out

    def frames(self) -> int:
        """累计心跳帧数（六指合计）。1 s 虚拟时间 ≈ 3000 帧。"""
        with self._lock:
            return self._hb_total

    def heartbeat(self) -> Dict[str, object]:
        """最近一次心跳快照：count / calib 位 / running 状态位（[手册]）。"""
        with self._lock:
            motors: Dict[int, Dict[str, object]] = {}
            for mid in self._ids:
                m = spec.MOTOR_BY_ID[mid]
                bits = self._bits(mid)
                calib = 0x08 if self._calibrated else 0x00
                motors[mid] = {
                    "motor_id": mid,
                    "name": m.name,
                    "count": self._hb_count[mid],
                    "calib": calib,
                    "calibrated": bool(calib & 0x08),
                    "running": bits,
                    "running_bits": _bit_names(bits),
                    "angle": round(self._pos[mid], 3),
                    "current": round(self._cur[mid], 3),
                    "current_limit": m.current_max,
                }
            return {
                "bus": self.name,
                "t": round(self.t, 4),
                "hz": self.hz,
                "frames": self._hb_total,
                "frames_per_s": round(self.hz * len(self._ids), 3),
                "calibrated": self._calibrated,
                "powered": self._powered,
                "attached": self._attached,
                "can_loss": self.can_loss,
                "motors": motors,
            }

    # ------------------------------------------------------------ 故障注入
    def inject(self, fault: str, **kwargs) -> Dict[str, object]:
        """注入故障，返回注入后的关键状态。

        * ``power_off``      —— ``on=True`` 断电（手指变软、位置塌陷、丢校准）；
          ``on=False`` 上电，需要重新校准（真机约 10 s [实测]，仿真 0.6 s）。
        * ``can_loss``      —— ``rate=0..1``，或 ``on=False`` 恢复 0。
        * ``undervoltage``  —— ``on=True`` / ``seconds=2.0``，置 0x08 并降速到 70%。
        * ``stall``         —— ``motor=103``（或 "index"），``on=False`` 解除。
        * ``uncalibrated``  —— ``on=True`` 丢掉校准；``on=False`` 立即重新校准。
        """
        key = str(fault).strip().lower()
        with self._lock:
            if key == "power_off":
                on = bool(kwargs.get("on", True))
                self._powered = not on
                if on:
                    # [实测] 断电后没有保持力，编码器参考也丢了 → 必须重新校准
                    self._calibrated = False
                    self._calib_t = 0.0
                    for mid in self._ids:
                        self._vel[mid] = 0.0
                        self._cur[mid] = 0.0
                        self._limited[mid] = False
                else:
                    self._calibrated = False
                    self._calib_t = float(kwargs.get("calib_after", CALIB_DELAY))
                return {"fault": key, "on": on, "powered": self._powered,
                        "calibrated": self._calibrated,
                        "recalib_in_s": round(self._calib_t, 3)}

            if key == "can_loss":
                if "on" in kwargs and not bool(kwargs["on"]):
                    rate = 0.0
                else:
                    rate = kwargs.get("rate", kwargs.get("value", self.can_loss))
                self.can_loss = _clamp(float(rate), 0.0, 1.0)
                return {"fault": key, "rate": self.can_loss,
                        "lost_frames": self._tx_lost}

            if key == "undervoltage":
                if not bool(kwargs.get("on", True)):
                    self._undervolt_until = None
                elif "seconds" in kwargs:
                    self._undervolt_until = self.t + max(0.0, float(kwargs["seconds"]))
                else:
                    self._undervolt_until = math.inf
                return {"fault": key, "on": self.undervoltage,
                        "bit": BIT_UNDERVOLT if self.undervoltage else 0}

            if key == "stall":
                motor = kwargs.get("motor", kwargs.get("motor_id"))
                if motor is None:
                    raise ValueError("stall 需要 motor=102..107 之一")
                m = spec.resolve(motor)
                on = bool(kwargs.get("on", True))
                if on:
                    self._stall.add(m.motor_id)
                else:
                    self._stall.discard(m.motor_id)
                return {"fault": key, "motor_id": m.motor_id, "on": on,
                        "stalled": sorted(self._stall)}

            if key == "uncalibrated":
                on = bool(kwargs.get("on", True))
                if on:
                    self._calibrated = False
                    self._calib_t = 0.0
                else:
                    self._calibrate()
                return {"fault": key, "on": on, "calibrated": self._calibrated}

            raise ValueError(f"未知故障: {fault!r}（可用: power_off / can_loss / "
                             "undervoltage / stall / uncalibrated）")

    def calibrate(self) -> None:
        """立即校准（等价于 inject("uncalibrated", on=False)）。"""
        with self._lock:
            self._calibrate()

    # ------------------------------------------------------------ 虚拟时间
    def step(self, dt: float) -> None:
        """推进 dt 秒虚拟时间（内部按 ≤2 ms 子步积分，大步长调用也稳定）。"""
        if dt is None:
            return
        dt = float(dt)
        if dt <= 0.0:
            return
        with self._lock:
            remaining = dt
            while remaining > 1e-12:
                h = min(remaining, MAX_SUBSTEP)
                self._substep(h)
                remaining -= h

    # ------------------------------------------------------------ 内部实现
    def _substep(self, h: float) -> None:
        self.t += h
        # 同动根数先算（与遍历顺序无关），每多一根多摊一点电流
        moving = 0
        for mid in self._ids:
            if not self._powered or self._speed[mid] <= 0.0:
                continue
            if self.t < self._move_at[mid]:
                continue
            if abs(self._target_eff(mid) - self._pos[mid]) > 0.5:
                moving += 1
        for mid in self._ids:
            self._advance(mid, h, moving)
        self._tick_heartbeat(h)

        if self._powered and not self._calibrated and self._calib_t > 0.0:
            self._calib_t -= h
            if self._calib_t <= 0.0:
                self._calibrate()
        if self._undervolt_until is not None and self.t >= self._undervolt_until:
            self._undervolt_until = None

    def _advance(self, mid: int, h: float, moving: int) -> None:
        m = spec.MOTOR_BY_ID[mid]

        if not self._powered:
            # [实测] 断电手指变软：按时间常数慢慢塌陷，没有保持力
            self._pos[mid] = max(0.0, self._pos[mid] * math.exp(-h / SAG_TAU))
            self._vel[mid] = 0.0
            self._cur[mid] = 0.0
            self._limited[mid] = False
            return

        target = self._target_eff(mid)
        pos = self._pos[mid]
        err = target - pos
        limit = self._limit[mid]

        # 电流需求随角度上升（腱绳/弹簧回复力矩），随同动根数上升（电源压降）
        i_req = I_FRIC + I_SPRING_PER_DEG * pos + I_COUPLING * max(0, moving - 1)
        limited = i_req > limit + 1e-9
        self._limited[mid] = limited
        self._cur[mid] = min(i_req, limit) if limit > 0.0 else 0.0

        blocked = (
            limited
            or mid in self._stall
            or self._speed[mid] <= 0.0
            or self.t < self._move_at[mid]
            or limit <= 0.0
        )
        vmax = 0.0
        if not blocked:
            vmax = self._speed[mid] * SPEED_DPS_PER_UNIT
            if self.undervoltage:
                vmax *= 0.7      # [推断] 母线掉压，转速打折

        # 梯形速度规划：v_des = min(vmax, √(2·a·|误差|)) 是「刚好能刹住」的速度，
        # 所以离目标越近越慢，加减速段天然存在，末端不会冲过头也不会爬行。
        if vmax > 0.0 and abs(err) > 1e-9:
            v_des = math.copysign(min(vmax, math.sqrt(2.0 * ACC_MAX * abs(err))), err)
        else:
            v_des = 0.0
        vel = self._vel[mid]
        vel += _clamp(v_des - vel, -ACC_MAX * h, ACC_MAX * h)
        pos = pos + vel * h
        if abs(target - pos) <= 0.01 or (err != 0.0 and (target - pos) * err < 0.0):
            pos, vel = target, 0.0   # 吸附，不抖动

        self._pos[mid] = _clamp(pos, 0.0, m.pos_max)
        self._vel[mid] = vel

    def _target_eff(self, mid: int) -> float:
        """实际驱动的角度 = 指令角 + 校准零偏，且不超过机械上限。"""
        m = spec.MOTOR_BY_ID[mid]
        return min(self._target_cmd[mid] + self._offset[mid], m.pos_max)

    def _tick_heartbeat(self, h: float) -> None:
        for mid in self._ids:
            self._hb_acc[mid] += h * self.hz
            n = int(self._hb_acc[mid])
            if n > 0:
                self._hb_acc[mid] -= n
                self._hb_count[mid] += n
                self._hb_total += n

    def _calibrate(self) -> None:
        """校准 = 重新找零点：手指顶到零位，落点 0.5–1.5°。

        所以校准会在零位附近重新落座（位置会被拉回 offset），这也是真机
        「回零」的物理含义；落点由 seed 决定，可复现。
        """
        for mid in self._ids:
            self._offset[mid] = self._rng_cal.uniform(REPEAT_MIN, REPEAT_MAX)
            # [实测] 回零后落在 0.5–1.5°
            self._pos[mid] = self._offset[mid]
            self._vel[mid] = 0.0
            # 回零后控制器里没有待执行的姿态，手指停在落点上
            self._target_cmd[mid] = 0.0
        self._calibrated = True
        self._calib_t = 0.0

    def _bits(self, mid: int) -> int:
        bits = 0
        if self._powered and (abs(self._vel[mid]) > 0.5
                              or abs(self._target_eff(mid) - self._pos[mid]) > 0.5):
            bits |= BIT_RUNNING
        if self._limited[mid] or mid in self._soft_oc or mid in self._stall:
            bits |= BIT_I_LIMIT
        if mid in self._pos_limit:
            bits |= BIT_POS_LIMIT
        if mid in self._stall:
            bits |= BIT_HW_OC
        if mid in self._soft_oc:
            bits |= BIT_SW_OC
        if self.undervoltage:
            bits |= BIT_UNDERVOLT
        return bits

    def _error(self, mid: int) -> Optional[str]:
        tags = []
        if not self._powered:
            tags.append("POWER_OFF")
        if not self._calibrated:
            tags.append("UNCALIBRATED")
        if mid in self._stall:
            tags.append("STALL")
        if self._limited[mid]:
            tags.append("I-LIM")
        if mid in self._soft_oc:
            tags.append("SW-OVERCURRENT")
        if mid in self._pos_limit:
            tags.append("POS-LIMIT")
        if self.undervoltage:
            tags.append("UNDERVOLTAGE")
        return "|".join(tags) if tags else None

    @property
    def undervoltage(self) -> bool:
        return self._undervolt_until is not None and self.t < self._undervolt_until


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


_BIT_TEXT = (
    (BIT_RUNNING, "running"),
    (BIT_HW_OC, "hw-overcurrent"),
    (BIT_SW_OC, "sw-overcurrent"),
    (BIT_UNDERVOLT, "undervoltage"),
    (BIT_I_LIMIT, "i-limit"),
    (BIT_POS_LIMIT, "pos-limit"),
)


def _bit_names(bits: int) -> list:
    return [name for bit, name in _BIT_TEXT if bits & bit]
