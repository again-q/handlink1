"""协作臂驱动：越疆 Dobot Nova 5（DT-NV050-1）。

臂和手是两条独立链路：

* 手 —— CAN（``handlink.canbus``），1000 kbps，3000 帧/秒心跳；
* 臂 —— 网口 TCP 29999（控制口），另有 30004 反馈口。

这一层只回答一个问题：**怎么让臂动起来、并且知道它到了没有**。
``SimArm`` 让整条流程在没有臂的时候也能跑通（dry-run / 单元测试），
``DobotArm`` 是照 Dobot CR/Nova 的 dashboard 协议写的，**尚未在 Nova 5 真机上验证**。

安全默认值：首次上线 ``SpeedFactor`` 不超过 20%，移动前必须显式 ``enable()``。
"""
from __future__ import annotations

import math
import socket
import threading
import time
from typing import Dict, Iterable, List, Optional, Sequence

DEFAULT_IP = "192.168.5.1"
CONTROL_PORT = 29999
FEEDBACK_PORT = 30004

#: 首次调试的最大速度因子（%），改大之前先确认工作空间里没有人
SAFE_SPEED_FACTOR = 20.0

#: 关节/笛卡尔到位判据
POS_TOL_MM = 1.0
ANG_TOL_DEG = 1.0


class ArmError(RuntimeError):
    """臂的通信或指令错误。"""


def _num(v: float) -> str:
    """Dobot 协议里的数字：不要科学计数法、不要多余 0。"""
    s = f"{float(v):.3f}".rstrip("0").rstrip(".")
    return s or "0"


def fmt_pose(pose: Sequence[float]) -> str:
    """``{x,y,z,rx,ry,rz}``。"""
    if len(pose) != 6:
        raise ValueError(f"姿态需要 6 个数 (x,y,z,rx,ry,rz)，收到 {len(pose)} 个")
    return "{" + ",".join(_num(v) for v in pose) + "}"


def fmt_joints(joints: Sequence[float]) -> str:
    """``{j1,j2,j3,j4,j5,j6}``，关节空间的 MovJ 用这个。

    手册 V4.6 的 ``MovJ(P,...)`` 里 P 既可以是 ``pose={x,y,z,rx,ry,rz}``，也可以是
    ``joint={j1..j6}``。关节空间不需要逆解，第一次试动作用它最稳。
    """
    if len(joints) != 6:
        raise ValueError(f"关节角需要 6 个数 (j1..j6)，收到 {len(joints)} 个")
    return "{" + ",".join(_num(v) for v in joints) + "}"


def parse_pose(text: str) -> List[float]:
    """从 ``0,{400,0,300,180,0,0},GetPose();`` 里取出 6 个数。"""
    start, end = text.find("{"), text.rfind("}")
    if start < 0 or end <= start:
        raise ArmError(f"返回值里没有姿态: {text!r}")
    body = text[start + 1:end]
    values = [float(x) for x in body.replace("，", ",").split(",") if x.strip()]
    if len(values) != 6:
        raise ArmError(f"姿态字段数不对（{len(values)}）: {text!r}")
    return values


class Arm:
    """接口：``SimArm`` 与 ``DobotArm`` 共用。"""

    name = "abstract"

    # -- 生命周期 ---------------------------------------------------------
    def connect(self) -> None:  # pragma: no cover - 接口
        pass

    def close(self) -> None:  # pragma: no cover
        pass

    # -- 状态 -------------------------------------------------------------
    def enabled(self) -> bool:
        raise NotImplementedError

    def pose(self) -> List[float]:
        raise NotImplementedError

    def joints(self) -> List[float]:
        raise NotImplementedError

    def ik(self, pose: Sequence[float],
           near: Optional[Sequence[float]] = None) -> Optional[List[float]]:
        """位姿 → 关节角。默认用本地数值反解；真机子类走控制器自己的 IK。"""
        raise NotImplementedError

    # -- 指令 -------------------------------------------------------------
    def enable(self) -> None:
        raise NotImplementedError

    def disable(self) -> None:
        raise NotImplementedError

    def speed_factor(self, percent: float) -> float:
        raise NotImplementedError

    def move_j(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        raise NotImplementedError

    def move_l(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        raise NotImplementedError

    def move_joints(self, joints: Sequence[float], wait: bool = True,
                    timeout: float = 30.0) -> Dict[str, object]:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    # -- 通用 -------------------------------------------------------------
    @staticmethod
    def _ang_diff(a: float, b: float) -> float:
        """两角之差，归一到 0～180。

        180° 和 −180° 是同一个朝向，但 ``abs(a-b)`` 会算出 360。控制器有时
        返回 −180、有时返回 +180（取决于 IK 选了哪支），直接用减法会让
        ``wait_until`` 永远等不到，白等到 timeout —— 实测白等了 40 s。
        """
        return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)

    def wait_until(self, target: Sequence[float], tol: float = POS_TOL_MM,
                   ang_tol: float = ANG_TOL_DEG, timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            p = self.pose()
            if all(abs(p[i] - target[i]) <= tol for i in range(3)) and \
                    all(self._ang_diff(p[i], target[i]) <= ang_tol for i in range(3, 6)):
                return True
            time.sleep(0.05)
        return False

    def wait_until_joints(self, target: Sequence[float], tol: float = ANG_TOL_DEG,
                          timeout: float = 30.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            j = self.joints()
            if all(abs(j[i] - target[i]) <= tol for i in range(6)):
                return True
            time.sleep(0.05)
        return False

    def status(self) -> Dict[str, object]:
        return {
            "arm": self.name,
            "enabled": self.enabled(),
            "pose": [round(v, 3) for v in self.pose()],
            "joints": [round(v, 3) for v in self.joints()],
        }


class SimArm(Arm):
    """仿真臂：记录目标位姿，按速度因子估算耗时，不真的等。"""

    name = "sim"

    def __init__(self, pose: Sequence[float] = (400.0, 0.0, 300.0, 180.0, 0.0, 0.0),
                 max_speed_mm_s: float = 100.0) -> None:
        self._pose = [float(v) for v in pose]
        self._joints = [0.0] * 6
        self._enabled = False
        self._factor = SAFE_SPEED_FACTOR
        self.max_speed_mm_s = float(max_speed_mm_s)
        self.trace: List[Dict[str, object]] = []

    # 生命周期
    def connect(self) -> None:
        self.trace.append({"t": time.time(), "event": "connect"})

    def close(self) -> None:
        self.trace.append({"t": time.time(), "event": "close"})

    # 状态
    def enabled(self) -> bool:
        return self._enabled

    def pose(self) -> List[float]:
        return list(self._pose)

    def joints(self) -> List[float]:
        return list(self._joints)

    def ik(self, pose: Sequence[float],
           near: Optional[Sequence[float]] = None) -> Optional[List[float]]:
        """仿真没有控制器，用本地的数值反解。"""
        from . import kin

        return kin.ik(pose, near or self._joints)

    # 指令
    def enable(self) -> None:
        self._enabled = True
        self.trace.append({"t": time.time(), "event": "enable"})

    def disable(self) -> None:
        self._enabled = False
        self.trace.append({"t": time.time(), "event": "disable"})

    def speed_factor(self, percent: float) -> float:
        self._factor = max(1.0, min(float(percent), 100.0))
        return self._factor

    def estimate_seconds(self, target: Sequence[float]) -> float:
        dist = math.dist(self._pose[:3], list(target)[:3])
        speed = self.max_speed_mm_s * self._factor / 100.0
        return dist / speed if speed > 0 else 0.0

    def _goto(self, target: Sequence[float], kind: str) -> Dict[str, object]:
        if len(target) != 6:
            raise ValueError("目标姿态需要 6 个数")
        if not self._enabled:
            raise ArmError("臂未使能（先 enable()）")
        eta = self.estimate_seconds(target)
        self._pose = [float(v) for v in target]
        step = {"t": time.time(), "event": kind, "to": list(self._pose), "eta_s": round(eta, 2)}
        self.trace.append(step)
        return {"ok": True, "arm": self.name, "cmd": kind, "to": list(self._pose), "eta_s": round(eta, 2)}

    def move_j(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._goto(pose, "MovJ")

    def move_l(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._goto(pose, "MovL")

    def move_joints(self, joints: Sequence[float], wait: bool = True,
                    timeout: float = 30.0) -> Dict[str, object]:
        if len(joints) != 6:
            raise ValueError("关节角需要 6 个数")
        if not self._enabled:
            raise ArmError("臂未使能（先 enable()）")
        self._joints = [float(v) for v in joints]
        step = {"t": time.time(), "event": "MovJ(joint)", "to": list(self._joints)}
        self.trace.append(step)
        return {"ok": True, "arm": self.name, "cmd": "MovJ(joint)", "joints": list(self._joints)}

    def stop(self) -> None:
        self.trace.append({"t": time.time(), "event": "stop"})


class DobotArm(Arm):
    """真机：TCP 29999 发 dashboard 指令（行文本、一问一答）。

    未在 Nova 5 上验证；第一次接真机先只跑 ``check()``（只读 GetPose/RobotMode），
    确认能读再 enable。指令名集中在 ``CMD_*`` 常量里，固件不一致时改这里。
    """

    name = "dobot"

    CMD_ENABLE = "EnableRobot()"
    CMD_DISABLE = "DisableRobot()"
    CMD_CLEAR = "ClearError()"
    CMD_REQUEST_CONTROL = "RequestControl()"
    CMD_POWER_ON = "PowerOn()"
    CMD_SPEED = "SpeedFactor({:.0f})"
    CMD_MOV_J = "MovJ(pose={})"
    CMD_MOV_L = "MovL(pose={})"
    CMD_MOV_JOINT = "MovJ(joint={})"
    CMD_POSE = "GetPose()"
    CMD_ANGLE = "GetAngle()"
    CMD_MODE = "RobotMode()"
    CMD_STOP = "Stop()"
    # 拖动示教（背驱）：切进去后电机变柔顺，人可以直接用手把臂掰到任意姿态，
    # 不需要写任何动作指令。退出时保持当前姿态。
    CMD_DRAG_START = "StartDrag()"
    CMD_DRAG_STOP = "StopDrag()"

    def __init__(self, ip: str = DEFAULT_IP, port: int = CONTROL_PORT,
                 timeout: float = 3.0, read_timeout: float = 2.0) -> None:
        self.ip = ip
        self.port = int(port)
        self.timeout = float(timeout)
        self.read_timeout = float(read_timeout)
        self._sock: Optional[socket.socket] = None
        self._lock = threading.RLock()
        self._last_error: Optional[str] = None
        self._factor = SAFE_SPEED_FACTOR

    # -- 连接 -------------------------------------------------------------
    def connect(self) -> None:
        if self._sock is not None:
            return
        try:
            sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        except OSError as exc:
            raise ArmError(f"连不上 {self.ip}:{self.port}（{exc}）—— "
                           f"先确认网线、PC 是 192.168.5.x、臂在 TCP 模式") from exc
        sock.settimeout(self.read_timeout)
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._sock = sock

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    # -- 一问一答 ---------------------------------------------------------
    def _command(self, text: str) -> str:
        with self._lock:
            if self._sock is None:
                self.connect()
            assert self._sock is not None
            try:
                self._sock.sendall((text + "\n").encode("ascii", "ignore"))
            except OSError as exc:
                raise ArmError(f"发送失败（{exc}）") from exc
            return self._check(self._read())

    def _read(self) -> str:
        assert self._sock is not None
        chunks: List[bytes] = []
        deadline = time.time() + self.read_timeout
        while time.time() < deadline:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                break
            except OSError as exc:
                raise ArmError(f"接收失败（{exc}）") from exc
            if not data:
                break
            chunks.append(data)
            text = b"".join(chunks).decode("utf-8", "ignore")
            if ";" in text or "{" in text and "}" in text and "\n" in text:
                break
        text = b"".join(chunks).decode("utf-8", "ignore").strip()
        if not text:
            raise ArmError("臂没有回复（超时）—— 检查是否别的程序占着 29999")
        return text

    @staticmethod
    def _check(text: str) -> str:
        low = text.lower()
        if "control mode" in low and "tcp" in low:
            raise ArmError("臂不在 TCP 模式：在示教器 / DobotStudio 里把控制模式切到 TCP")
        if text.startswith("-"):
            raise ArmError(text)
        return text

    def _idx(self, text: str, cmd: str) -> str:
        return f"{text} [{cmd}]"

    # -- 状态 -------------------------------------------------------------
    def enabled(self) -> bool:
        try:
            reply = self._command(self.CMD_MODE)
        except ArmError:
            return False
        # RobotMode() -> 0,{5},... 5 = 使能空闲；4 = 未使能
        try:
            start, end = reply.find("{"), reply.find("}", reply.find("{"))
            if start >= 0 and end > start:
                return int(float(reply[start + 1:end])) in (5, 6, 7, 8, 9, 10, 11)
        except (ValueError, TypeError):
            pass
        return False

    def pose(self) -> List[float]:
        return parse_pose(self._check(self._command(self.CMD_POSE)))

    def joints(self) -> List[float]:
        text = self._check(self._command(self.CMD_ANGLE))
        start, end = text.find("{"), text.rfind("}")
        if start < 0 or end <= start:
            raise ArmError(f"GetAngle 返回异常: {text!r}")
        return [float(x) for x in text[start + 1:end].split(",") if x.strip()]

    # -- 指令 -------------------------------------------------------------
    def ik(self, pose: Sequence[float],
           near: Optional[Sequence[float]] = None) -> Optional[List[float]]:
        """控制器自己的 ``InverseKin``：真机的解和它自己执行时完全一致。

        带 ``useJointNear=1``，让控制器挑离当前关节最近的那个支管——不然同一个
        位姿有多个解，臂可能突然翻腕。返回 ``None`` 表示控制器说无解。
        """
        p = [float(v) for v in pose]
        near = [float(v) for v in (near if near is not None else self.joints())]
        joints = ",".join(f"{v:.3f}" for v in near)
        text = self._command(
            f"InverseKin({p[0]:.3f},{p[1]:.3f},{p[2]:.3f},"
            f"{p[3]:.3f},{p[4]:.3f},{p[5]:.3f},"
            f"useJointNear=1,jointNear={{{joints}}})")
        start, end = text.find("{"), text.find("}")
        if start < 0 or end <= start:
            return None
        try:
            vals = [float(x) for x in text[start + 1:end].split(",") if x.strip()]
        except ValueError:
            return None
        return vals if len(vals) == 6 else None

    def take_control(self) -> None:
        """请求切到 TCP 模式（不必再开 DobotStudio）。

        协议规定只在「未上电」或「下使能（非暂停、抱闸未松）」时才允许切换。
        """
        self._check(self._command(self.CMD_REQUEST_CONTROL))

    def power_on(self) -> None:
        self._check(self._command(self.CMD_POWER_ON))

    def enable(self) -> None:
        self._check(self._command(self.CMD_ENABLE))

    def disable(self) -> None:
        self._check(self._command(self.CMD_DISABLE))

    def clear_error(self) -> None:
        self._check(self._command(self.CMD_CLEAR))

    def speed_factor(self, percent: float) -> float:
        pct = max(1.0, min(float(percent), 100.0))
        self._check(self._command(self.CMD_SPEED.format(pct)))
        self._factor = pct
        return pct

    def move_j(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._move(self.CMD_MOV_J, "MovJ", pose, wait, timeout)

    def move_l(self, pose: Sequence[float], wait: bool = True, timeout: float = 30.0) -> Dict[str, object]:
        return self._move(self.CMD_MOV_L, "MovL", pose, wait, timeout)

    def move_joints(self, joints: Sequence[float], wait: bool = True,
                    timeout: float = 30.0) -> Dict[str, object]:
        """关节空间 MovJ：不需要逆解，轨迹只取决于关节插值。"""
        if len(joints) != 6:
            raise ValueError("关节角需要 6 个数")
        self._check(self._command(self.CMD_MOV_JOINT.format(fmt_joints(joints))))
        result: Dict[str, object] = {"ok": True, "arm": self.name, "cmd": "MovJ(joint)",
                                     "joints": [round(float(v), 3) for v in joints]}
        if wait:
            result["arrived"] = self.wait_until_joints(joints, timeout=timeout)
        return result

    def _move(self, template: str, kind: str, pose: Sequence[float],
              wait: bool, timeout: float) -> Dict[str, object]:
        cmd = template.format(fmt_pose(pose))
        self._check(self._command(cmd))
        result: Dict[str, object] = {"ok": True, "arm": self.name, "cmd": kind, "to": list(pose)}
        if wait:
            result["arrived"] = self.wait_until(pose, timeout=timeout)
        return result

    def stop(self) -> None:
        self._check(self._command(self.CMD_STOP))

    def start_drag(self) -> None:
        """进入拖动示教（背驱）：电机变柔顺，人可以手扶着把臂摆到任意姿态。"""
        self._check(self._command(self.CMD_DRAG_START))

    def stop_drag(self) -> None:
        """退出拖动示教，保持当前姿态，重新由指令控制。"""
        self._check(self._command(self.CMD_DRAG_STOP))

    # -- 只读自检 ---------------------------------------------------------
    def check(self) -> Dict[str, object]:
        """第一次接真机跑这个：只读，不动。"""
        self.connect()
        out: Dict[str, object] = {"ip": self.ip, "port": self.port}
        out["mode_raw"] = self._command(self.CMD_MODE)
        out["pose"] = self.pose()
        out["joints"] = self.joints()
        out["enabled"] = self.enabled()
        return out
