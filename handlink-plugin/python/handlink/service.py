"""本地 HTTP 服务：把机械手变成一组可调用的 JSON 接口。

设计目标是让「我」（或任何脚本、Notebook、MCP 客户端）不用关心底下是仿真还是
真机，只用 HTTP 就能看状态、下发姿态、跑动作、录制与分析。

默认只监听 127.0.0.1，不对外暴露。
"""
from __future__ import annotations

import json
import math
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from . import ai as ai_mod
from . import arm as arm_mod
from . import arm_actions
from . import arm_feed
from . import arm_poses
from . import armview
from . import audio as audio_mod
from . import boxcv
from . import bus as bus_mod
from . import camera as cam_mod
from . import camui
from . import cable as cable_mod
from . import cell as cell_mod
from . import cubecv
from . import cubeui
from . import glove as glove_mod
from . import kin
from . import poses as poses_mod
from . import recorder as rec_mod
from . import scene as scene_mod
from . import skills, spec, store, tablescan, webui

# 网页 / JSON 接口的默认端口。新版统一用 8766（旧版是 8765），
# 两个端口可以同时各起一个服务，互不抢占。
DEFAULT_PORT = 8766


class App:
    """服务状态与业务动作。"""

    def __init__(self, hand_bus: bus_mod.HandBus, root: Optional[Path] = None,
                 points_dir: Optional[Path] = None, workdir: Optional[Path] = None) -> None:
        self.bus = hand_bus
        self.dirs = store.dirs(root)
        self.dirs.points = Path(points_dir) if points_dir else self.dirs.points
        self.workdir = Path(workdir) if workdir else Path.cwd()
        self.lock = threading.RLock()
        self.camera = cam_mod.Camera()
        self.cube = cubecv.shared()          # 魔方视觉：和 /camera 的识别器共用
        # 相机实时环境：桌面平面标定 + 物体表（见 scene.py）。后台线程懒启动，
        # 只有真的开过相机/读过 /scene/state 才跑，测试里不会凭空起线程。
        self.scene = scene_mod.Scene()
        # 整桌扫描：一个摄像头把桌上所有东西分割出来（见 tablescan.py）。
        # 背景图/开关存在 config/scan.json，服务重启不会自己开始扫。
        self.scan = tablescan.TableScan(self.scene)
        self.scene_hub: Optional[scene_mod.VisionHub] = None
        self.audio = audio_mod.MicRecorder()
        self.audio_dir = self.dirs.root / "recordings"
        self.recorder: Optional[rec_mod.Recorder] = None
        self.sampler: Optional[rec_mod.Sampler] = None
        self.player_thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.last_reason = ""
        self.last_action_error = ""
        self.started_at = time.time()
        # 协作臂：真机走 30004 反馈，没接就退回仿真关节
        self.arm_ip: Optional[str] = None
        self.arm_feed: Optional[arm_feed.Feed] = None
        self.arm_error = ""
        self.arm_sim_joints: List[float] = [0.0] * 6
        # 使能/下使能要开 29999 的短连接，串行化，避免两个请求同时抢控制权
        self._arm_lock = threading.Lock()
        # 手腕走线防缠：J4/J6 的累计扭转和等效角选择（见 cable.py）
        self.cable = cable_mod.TwistGuard()
        # 示教时存下来的绝对姿态（见 arm_poses.py），和防缠/组动作都无关
        self.arm_poses = arm_poses.PoseStore()
        # 数据手套（MotionVenus，UDP）直接收在服务里：见 glove.py。
        # 这里只建对象不占端口，真开监听要 /glove/start（插件加载后会自动开）。
        self.glove = glove_mod.GloveReceiver(apply=self.apply_glove)
        # 手到底连上没有：看 CAN 帧最近还在不在涨（见 hand_status / attach_hand）
        self._hand_frames: Optional[int] = None
        self._hand_seen_at: Optional[float] = None

    # -- 指令 -------------------------------------------------------------
    def apply(self, pose: spec.Pose, reason: str = "",
              held: Optional[list] = None) -> Dict[str, Any]:
        current = self.bus.pose()
        issues = spec.check(pose, reference=current)
        if spec.has_errors(issues):
            return {"ok": False, "error": "安全检查未通过", "issues": issues}
        with self.lock:
            result = self.bus.apply(pose, reason=reason)
            self.last_reason = reason
            if self.recorder and self.recorder.open:
                self.recorder.cmd(pose, reason)
        return {"ok": True, "issues": issues, "held": held or [], **result}

    def apply_point(self, name: str, reason: Optional[str] = None) -> Dict[str, Any]:
        path = store.resolve_point(name, self.dirs.points)
        pose = store.load_point(path)
        return self.apply(pose, reason=reason or f"point:{name}")

    def play_track(self, name: str, scale: float = 1.0, dry_run: bool = False) -> Dict[str, Any]:
        path = self.dirs.tracks / f"{name}.txt"
        if not path.is_file():
            return {"ok": False, "error": f"动作不存在: {path}"}
        track = store.load_track(path)
        player = bus_mod.Player(self.bus, self.dirs.points)
        if dry_run:
            return {"ok": True, "dry_run": True, "track": track.as_dict(),
                    "timeline": player.preview(track), "duration_ms": track.duration_ms}
        if self.player_thread and self.player_thread.is_alive():
            return {"ok": False, "error": "已有动作在播放，先 /stop"}
        self.stop_event.clear()
        if self.recorder and self.recorder.open:
            self.recorder.event("track_start", name=name, scale=scale)

        def run() -> None:
            try:
                result = player.play(track, speed_scale=scale, stop_event=self.stop_event,
                                     on_step=lambda s: self.recorder and self.recorder.open
                                     and self.recorder.event("track_step", **s))
                if self.recorder and self.recorder.open:
                    self.recorder.event("track_done", name=name, **result)
            finally:
                self.bus.stop()

        self.player_thread = threading.Thread(target=run, name="track-player", daemon=True)
        self.player_thread.start()
        return {"ok": True, "started": True, "track": track.as_dict(),
                "duration_ms": track.duration_ms}

    def stop(self) -> Dict[str, Any]:
        self.stop_event.set()
        with self.lock:
            self.bus.stop()
        return {"ok": True, "stopped": True}

    # -- 数据手套 ---------------------------------------------------------
    def apply_glove(self, motors: Dict[int, float], speed: float, torque: float) -> Dict[str, Any]:
        """手套线程每帧调一次：六指角度 → Pose → 和 /pose 走同一条安全检查。"""
        pose: spec.Pose = {
            int(mid): spec.Joint(float(angle), float(speed), float(torque))
            for mid, angle in motors.items()
        }
        return self.apply(pose, reason="glove")

    # -- 预设动作组 -------------------------------------------------------
    def action_list(self) -> Dict[str, Any]:
        return {"ok": True, "actions": poses_mod.catalogue()}

    def _settle_seconds(self, pose: spec.Pose) -> float:
        """按角度差估一个等待时间：实测 speed=200 约 55°/s。"""
        current = self.bus.pose()
        delta = max((abs(j.angle - current[mid].angle) for mid, j in pose.items()), default=0.0)
        return min(4.0, delta / 50.0 + 0.35)

    def run_action(self, name: str) -> Dict[str, Any]:
        meta = poses_mod.ACTIONS.get(name)
        if meta is None:
            return {"ok": False, "error": f"没有这个动作组: {name}"}
        if self.player_thread and self.player_thread.is_alive():
            return {"ok": False, "error": "已有动作在播放，先 /stop"}
        self.stop_event.clear()
        self.last_action_error = ""

        def run() -> None:
            try:
                for pose_name in meta["poses"]:
                    if self.stop_event.is_set():
                        break
                    pose = poses_mod.pose_of(pose_name)
                    wait = self._settle_seconds(pose)
                    result = self.apply(pose, reason=f"action:{name}/{pose_name}")
                    if not result.get("ok"):
                        self.last_action_error = str(result.get("error") or "未知错误")
                        break
                    if self.recorder and self.recorder.open:
                        self.recorder.event("action_step", name=name,
                                            pose=pose_name, ok=result.get("ok"))
                    if self.stop_event.wait(wait):
                        break
                    # 真到位再走下一段：拇指1 没停稳就合五指会互相顶住
                    poses_mod.wait_pose(self.bus, pose, timeout=max(0.5, wait),
                                        stop=self.stop_event)
                    if self.stop_event.wait(float(meta.get("hold", 0.0))):
                        break
            finally:
                pass

        self.player_thread = threading.Thread(target=run, name="action", daemon=True)
        self.player_thread.start()
        return {"ok": True, "started": True, "action": name,
                "label": meta["label"], "poses": meta["poses"]}

    # -- AI 控制 ----------------------------------------------------------
    def ai_plan(self, text: str, prefer_llm: bool = False) -> ai_mod.Plan:
        return ai_mod.make_plan(text, prefer_llm=prefer_llm)

    def ai_run(self, text: str, dry_run: bool = False, prefer_llm: bool = False,
               speed_scale: float = 1.0) -> Dict[str, Any]:
        plan = self.ai_plan(text, prefer_llm=prefer_llm)
        base = {"plan": plan.as_dict(), "explain": ai_mod.explain(plan)}
        if dry_run:
            return {"ok": True, "dry_run": True, **base}
        if self.player_thread and self.player_thread.is_alive():
            return {"ok": False, "error": "已有动作在播放，先 /stop", **base}
        self.stop_event.clear()
        executor = ai_mod.Executor(self.apply, stop_event=self.stop_event)
        result = executor.run(plan, speed_scale=speed_scale)
        if self.recorder and self.recorder.open:
            self.recorder.event("ai", text=text, ok=result.get("ok"), steps=len(plan.steps))
        return {**base, **result}

    # -- 录制 -------------------------------------------------------------
    def start_recording(self, path: Optional[str] = None, hz: float = 20.0) -> Dict[str, Any]:
        if self.recorder and self.recorder.open:
            return {"ok": False, "error": "已经在录制", "path": str(self.recorder.path)}
        out = Path(path) if path else (Path("recordings") / time.strftime("%Y%m%d-%H%M%S.jsonl"))
        if not out.is_absolute():
            out = (self.workdir if path else self.dirs.root) / out
        self.recorder = rec_mod.Recorder(out, meta={"bus": self.bus.name, "host": "handlink"})
        self.sampler = rec_mod.Sampler(self.bus, self.recorder, hz=hz).start()
        return {"ok": True, "path": str(out), "hz": hz}

    def stop_recording(self) -> Dict[str, Any]:
        if not (self.recorder and self.recorder.open):
            return {"ok": False, "error": "当前没有录制"}
        path = self.recorder.path
        if self.sampler:
            self.sampler.stop()
            self.sampler = None
        self.recorder.close()
        self.recorder = None
        return {"ok": True, "path": str(path), "stats": rec_mod.analyze(path)}

    def recording_stats(self) -> Dict[str, Any]:
        if not (self.recorder and self.recorder.open):
            return {"ok": False, "error": "当前没有录制"}
        return {"ok": True, "path": str(self.recorder.path), "stats": rec_mod.analyze(self.recorder.path)}

    # -- 麦克风 -----------------------------------------------------------
    def audio_devices(self) -> Dict[str, Any]:
        return {"ok": True, "windows": audio_mod.IS_WINDOWS,
                "devices": [d.as_dict() for d in audio_mod.list_inputs()]}

    def audio_start(self, data: Dict[str, Any]) -> Dict[str, Any]:
        raw = data.get("path")
        out = Path(raw) if raw else (self.audio_dir / time.strftime("audio-%Y%m%d-%H%M%S.wav"))
        if not out.is_absolute():
            out = (self.workdir if raw else self.audio_dir) / out
        try:
            return self.audio.start(
                index=int(data.get("index", 0)),
                rate=int(data.get("rate", audio_mod.DEFAULT_RATE)),
                channels=int(data.get("channels", audio_mod.DEFAULT_CHANNELS)),
                out=out,
                max_seconds=float(data.get("seconds", 30)),
            )
        except (RuntimeError, OSError) as exc:
            return {"ok": False, "error": str(exc)}

    def audio_test(self, data: Dict[str, Any]) -> Dict[str, Any]:
        raw = data.get("path")
        out = Path(raw) if raw else (self.audio_dir / time.strftime("audio-test-%Y%m%d-%H%M%S.wav"))
        if not out.is_absolute():
            out = (self.workdir if raw else self.audio_dir) / out
        try:
            return self.audio.test(index=int(data.get("index", 0)),
                                   seconds=float(data.get("seconds", 2)), out=out)
        except (RuntimeError, OSError) as exc:
            return {"ok": False, "error": str(exc)}

    # -- 信息 -------------------------------------------------------------
    def health(self) -> Dict[str, Any]:
        cam = self.camera.state()
        mic = self.audio.state()
        return {
            "ok": True,
            "bus": self.bus.name,
            "uptime_s": round(time.time() - self.started_at, 1),
            "motors": len(spec.MOTORS),
            "root": str(self.dirs.root),
            "recording": bool(self.recorder and self.recorder.open),
            "playing": bool(self.player_thread and self.player_thread.is_alive()),
            "camera": {"running": cam["running"], "fps": cam["fps"],
                       "size": [cam["width"], cam["height"]],
                       "recognition": cam["recognizer"]["enabled"]},
            "audio": {"running": mic["running"], "device": mic["device"],
                      "seconds": mic["seconds"]},
            "hand": self.hand_status(),
            "glove": self.glove.summary(),
        }

    # -- 手（连接 / 确认是否连上） ------------------------------------------
    def hand_status(self) -> Dict[str, Any]:
        """手到底连上没有：真手看 CAN 帧，仿真就明确说「没接」。

        判断依据不是「适配器打开过」，而是**最近 3 秒内还收到过帧**：线松了、
        手断电了，帧就停了，这里立刻变成未连接。
        """
        now = time.time()
        status: Dict[str, Any] = {"bus": self.bus.name, "connected": False}
        try:
            snap = self.bus.snapshot()
        except Exception as exc:                       # noqa: BLE001
            status["error"] = str(exc)
            status["detail"] = f"读总线失败：{exc}"
            return {"ok": False, **status}

        status["motors"] = len(snap.get("motors") or {})
        if self.bus.name != "can":
            status["detail"] = "仿真总线：还没有接真手"
            return {"ok": True, **status}

        frames = int(snap.get("frames") or 0)
        hand_id = snap.get("hand_id")
        calibrated = bool(snap.get("calibrated"))
        if frames > (self._hand_frames or 0):
            self._hand_frames = frames
            self._hand_seen_at = now
        silent = now - (self._hand_seen_at or now)
        status.update({
            "hand_id": hand_id,
            "frames": frames,
            "calibrated": calibrated,
            "silent_s": round(silent, 2),
            "connected": bool(frames) and silent < 3.0,
        })
        who = f"CAN 0x{hand_id:02X}" if hand_id else "CAN（没认出 ID）"
        if status["connected"]:
            status["detail"] = (f"{who} · {frames} 帧 · 校准"
                                f"{'正常' if calibrated else '未完成'}")
        else:
            status["detail"] = f"{who} · 最近 {silent:.1f}s 没收到帧（线松了或手断电）"
        return {"ok": True, **status}

    def attach_hand(self, bus_name: str = "can", dll: Optional[str] = None,
                    hand_id: Optional[int] = None) -> Dict[str, Any]:
        """运行时连手：仿真换成真手，或者掉线后重连。

        同一时刻只有一个进程能打开 CAN 适配器，所以顺序必须是「先关旧的、再开新的」。
        新总线起不来就退回仿真——一次连接失败不能把整个服务弄死。
        """
        if self.player_thread is not None and self.player_thread.is_alive():
            self.stop_event.set()
            self.player_thread.join(timeout=2.0)
        with self.lock:
            try:
                self.bus.close()
            except Exception:                          # noqa: BLE001
                pass
            self._hand_frames = None
            self._hand_seen_at = None
            if bus_name != "can":
                self.bus = bus_mod.open_sim()
                return {"ok": True, "bus": "sim", "connected": False,
                        "detail": "已切回仿真总线"}

            from .canbus import DEFAULT_DLL, CanBus

            try:
                new_bus = CanBus(dll=dll or DEFAULT_DLL, hand_id=hand_id)
                new_bus.open()
                snap = new_bus.snapshot()
                frames = int(snap.get("frames") or 0)
                if frames <= 0:
                    raise RuntimeError("CAN 适配器打开了，但一帧都没收到（手通电了吗？线插好了吗？）")
                self.bus = new_bus
                self._hand_frames = frames
                self._hand_seen_at = time.time()
            except Exception as exc:                   # noqa: BLE001
                self.bus = bus_mod.open_sim()
                self._hand_frames = None
                self._hand_seen_at = None
                return {"ok": False, "bus": "sim", "connected": False,
                        "error": str(exc),
                        "detail": f"连不上真手：{exc}（已退回仿真）"}
        status = self.hand_status()
        status["detail"] = "已连上真手 · " + str(status.get("detail", ""))
        return status

    # -- 臂（三维视图的数据源） -------------------------------------------
    def attach_arm(self, ip: str, attempts: int = 3) -> bool:
        """接上 30004 反馈口。连不上不算错误——视图退回仿真。

        控制柜对 30004 的并发连接很敏感：上一个客户端被强杀后，会有一小段时间连不上，
        所以这里重试几次，别一次失败就放弃（实测确实遇到过）。
        """
        last = ""
        for i in range(max(1, attempts)):
            try:
                feed = arm_feed.Feed(ip).open()
                feed.wait_frame(timeout=3.0)
            except arm_feed.FeedbackError as exc:
                last = str(exc)
                time.sleep(1.0)
                continue
            self.arm_feed = feed
            self.arm_ip = ip
            self.arm_error = ""
            return True
        self.arm_error = last
        return False

    def close_arm(self) -> None:
        if self.arm_feed is not None:
            self.arm_feed.close()
            self.arm_feed = None

    def _arm_live(self) -> bool:
        return self.arm_feed is not None and self.arm_feed.age() < 1.5

    def _wait_arm_enabled(self, on: bool, timeout: float = 6.0) -> bool:
        """等 30004 的 EnableStatus 真的变成 on，而不是发完指令就当成功。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                snap = self.arm_feed.snapshot()
            except arm_feed.FeedbackError:
                return False
            if (int(snap["EnableStatus"]) == 1) == on:
                return True
            time.sleep(0.05)
        return False

    def arm_enable(self, on: bool = True, speed: Optional[float] = None) -> Dict[str, Any]:
        """网页上的「使能 / 下使能」。

        顺序照 scripts/arm_up.py：先读 30004 的真实状态，需要时 RequestControl()
        接管 TCP，设速度因子，再 EnableRobot() / DisableRobot()，最后用反馈复核
        EnableStatus——每一步都不靠 sleep 猜。

        * 真机不在线、有报警、急停未复位、正在运动时**不发指令**；
        * 使能时的速度因子默认 SAFE_SPEED_FACTOR（20%），可以用 speed 覆盖。
        """
        if not self._arm_live():
            return {"ok": False, "online": False, "enabled": False,
                    "error": "真机不在线（30004 没有数据）—— 先点「重连真机」"}
        with self._arm_lock:
            try:
                snap = self.arm_feed.snapshot()
            except arm_feed.FeedbackError as exc:
                return {"ok": False, "online": False, "enabled": False, "error": str(exc)}
            described = arm_feed.describe(snap)
            base = {"online": True, "mode": described["mode"],
                    "mode_name": described["mode_name"],
                    "speed_scaling": described["speed_scaling"]}
            live_enabled = bool(described["enabled"])

            if described["safety"]["estop"] or described["safety"]["protective_stop"]:
                return {**base, "ok": False, "enabled": live_enabled,
                        "error": "急停/安全输入未复位，先排除再操作使能"}
            if on and live_enabled:
                return {**base, "ok": True, "enabled": True, "on": True,
                        "already": True, "note": "已经处于使能状态"}
            if not on and not live_enabled:
                return {**base, "ok": True, "enabled": False, "on": False,
                        "already": True, "note": "已经是下使能状态"}
            if described["running"] or described["mode"] in (7, 8, 11):
                return {**base, "ok": False, "enabled": live_enabled,
                        "error": "臂正在运动，先停下再操作使能"}
            if on and described["error"]:
                return {**base, "ok": False, "enabled": live_enabled,
                        "error": (f"臂有报警 {described['error']}，先清报警"
                                  "（ClearError / 示教器）再使能")}
            if on and described["mode"] not in (3, 4, 5):
                return {**base, "ok": False, "enabled": live_enabled,
                        "error": f"当前模式「{described['mode_name']}」不允许使能"}

            robot = arm_mod.DobotArm(ip=self.arm_ip or arm_mod.DEFAULT_IP)
            try:
                robot.connect()
                if on:
                    if described["mode"] in arm_feed.CONTROLLABLE_MODES:
                        robot.take_control()
                        robot.pose()          # 复核：控制权真的到手了
                    pct = arm_mod.SAFE_SPEED_FACTOR if speed is None else float(speed)
                    pct = max(1.0, min(100.0, pct))
                    robot.speed_factor(pct)
                    robot.enable()
                else:
                    robot.disable()
            except (arm_mod.ArmError, ValueError) as exc:
                return {**base, "ok": False, "enabled": live_enabled, "error": str(exc)}
            finally:
                robot.close()

            want = bool(on)
            reached = self._wait_arm_enabled(want, timeout=6.0)
            try:
                after = arm_feed.describe(self.arm_feed.snapshot())
            except arm_feed.FeedbackError:
                after = described
            result: Dict[str, Any] = {
                "ok": reached, "online": True, "on": want,
                "enabled": bool(after["enabled"]),
                "mode": after["mode"], "mode_name": after["mode_name"],
                "speed_scaling": after["speed_scaling"],
            }
            if not reached:
                result["error"] = ("指令已发，但 6 s 内没在 30004 里看到使能状态变化 —— "
                                   "检查示教器 / 急停 / 报警")
            return result

    def arm_drag(self, on: bool = True) -> Dict[str, Any]:
        """网页上的「拖动示教」：进 ``StartDrag()`` 后电机变柔顺，人直接用手掰。

        这条路不需要写任何动作指令——掰到哪就是哪，退出（``StopDrag()``）时
        保持掰到的姿态，之后可以用 ``/arm/state`` 读出来、再用 MovJ 复现。

        * 未使能 / 急停未复位 / 真机不在线时**不发指令**；
        * 进之前要求已经在「使能空闲」这类可控模式里。
        """
        if not self._arm_live():
            return {"ok": False, "online": False,
                    "error": "真机不在线（30004 没有数据）—— 先点「重连真机」"}
        with self._arm_lock:
            try:
                described = arm_feed.describe(self.arm_feed.snapshot())
            except arm_feed.FeedbackError as exc:
                return {"ok": False, "online": False, "error": str(exc)}
            base = {"online": True, "mode": described["mode"],
                    "mode_name": described["mode_name"],
                    "enabled": bool(described["enabled"])}
            if described["safety"]["estop"] or described["safety"]["protective_stop"]:
                return {**base, "ok": False,
                        "error": "急停/安全输入未复位，不能进拖动示教"}
            if on and not described["enabled"]:
                return {**base, "ok": False,
                        "error": "臂未使能——先点「使能」，再进拖动示教"}
            if on and described["error"]:
                return {**base, "ok": False,
                        "error": f"臂有报警 {described['error']}，先清报警再进拖动示教"}
            robot = arm_mod.DobotArm(ip=self.arm_ip or arm_mod.DEFAULT_IP)
            try:
                robot.connect()
                if on:
                    robot.start_drag()
                else:
                    robot.stop_drag()
            except (arm_mod.ArmError, ValueError) as exc:
                return {**base, "ok": False, "error": str(exc)}
            finally:
                robot.close()
            time.sleep(0.4)
            try:
                after = arm_feed.describe(self.arm_feed.snapshot())
            except arm_feed.FeedbackError:
                after = described
            return {"ok": True, "online": True, "on": bool(on),
                    "mode": after["mode"], "mode_name": after["mode_name"],
                    "enabled": bool(after["enabled"])}

    # -- 保存的姿态 -------------------------------------------------------
    def arm_pose_list(self) -> Dict[str, Any]:
        """``GET /arm/poses``：已存的姿态（含文件路径，页面直接显示）。"""
        return self.arm_poses.state()

    def arm_pose_save(self, name: str, joints: Optional[List[float]] = None,
                      note: str = "", source: str = "") -> Dict[str, Any]:
        """``POST /arm/poses``：把一组关节角存下来（重名覆盖）。

        ``joints`` 不给就用「现在这个姿态」：真机在线读 30004，否则用视图里的仿真角。
        存之前按手册限位夹一遍，并顺手算出末端位姿一起写进文件。
        """
        if joints is None:
            joints = self._arm_now()
            source = source or ("robot" if self._arm_live() else "model")
        return self.arm_poses.save(name, joints, note=note, source=source)

    def arm_pose_delete(self, name: str) -> Dict[str, Any]:
        """``POST /arm/poses/delete``：删一个保存的姿态。"""
        return self.arm_poses.delete(name)

    def hand_enable(self, on: bool = True) -> Dict[str, Any]:
        """手部驱动器总开关。``on=False`` = 泄力，手指变软可以手掰。

        为什么单独做一条：``/pose`` 里把 ``torque`` 设成 0 **不够**——电流上限确实
        变 0 了，但驱动器仍使能、用内部闭环保持位置，手掰起来还是硬的。要真正
        变软必须把驱动器关掉（``enable(motor, False)``），这就是这条路。
        """
        bus = self.bus
        fn = getattr(bus, "enable_all", None)
        if fn is None:
            return {"ok": False, "bus": getattr(bus, "name", "?"),
                    "error": "当前是仿真总线，没有真手可泄力"}
        try:
            fn(bool(on))
        except Exception as exc:                       # noqa: BLE001
            return {"ok": False, "error": str(exc)}
        return {"ok": True, "on": bool(on), "bus": getattr(bus, "name", "?"),
                "note": ("已泄力：驱动器关闭，手指可用手掰" if not on
                         else "已恢复出力，可以下发 /pose")}

    def ensure_scene_hub(self) -> scene_mod.VisionHub:
        """懒启动视觉线程：只有开过相机 / 显式配置时才跑，离线测试不受影响。"""
        if self.scene_hub is None:
            self.scene_hub = scene_mod.VisionHub(self.camera, self.cube, self.scene,
                                                 scan=self.scan)
            self.scene_hub.start()
        return self.scene_hub

    def scene_summary(self) -> Dict[str, Any]:
        out = self.scene.summary()
        out["hub"] = (self.scene_hub.state() if self.scene_hub
                      else {"running": False, "hz": 0.0, "frames": 0, "reused": 0,
                            "errors": 0, "last_error": "", "mode": "idle",
                            "last_ms": 0.0})
        out["scan"] = self.scan.state()
        return out

    def hand_curl(self) -> Dict[str, Any]:
        """把六指角度折算成三维视图里五根手指的弯曲量 0~1。"""
        motors = self.bus.snapshot()["motors"]
        curl: List[float] = []
        for mid in (103, 104, 105, 106):
            m = spec.MOTOR_BY_ID[mid]
            angle = float(motors.get(str(mid), {}).get("angle") or 0.0)
            curl.append(max(0.0, min(1.0, angle / max(1.0, m.pos_max))))
        thumb = []
        for mid in (102, 107):
            m = spec.MOTOR_BY_ID[mid]
            angle = float(motors.get(str(mid), {}).get("angle") or 0.0)
            thumb.append(max(0.0, min(1.0, angle / max(1.0, m.pos_max))))
        curl.append(sum(thumb) / len(thumb))
        avg = sum(curl[:4]) / 4.0
        shape = "张开" if avg < 0.15 else ("握拳" if avg > 0.75 else "半握")
        return {"curl": [round(v, 3) for v in curl], "shape": shape,
                "source": f"总线 /state（{self.bus.name}）"}

    def arm_snapshot(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "ok": True,
            "online": False,
            "ip": self.arm_ip,
            "error": self.arm_error,
            "mode_name": "仿真",
            "mode": 0,
            "enabled": False,
            "hand": self.hand_curl(),
        }
        if self._arm_live():
            try:
                snap = self.arm_feed.snapshot()
            except arm_feed.FeedbackError:
                snap = None
            if snap is not None:
                described = arm_feed.describe(snap)
                joints = [round(float(v), 3) for v in snap["QActual"]]
                tcp = snap.get("TCPSpeedActual") or [0.0, 0.0, 0.0]
                self.cable.observe(joints)
                out.update({
                    "online": True,
                    "joints": joints,
                    "pose": [round(float(v), 3) for v in snap["ToolVectorActual"]],
                    "pose_source": "robot",
                    "mode": described["mode"],
                    "mode_name": described["mode_name"],
                    "enabled": described["enabled"],
                    "error_id": described["error"],
                    "speed_scaling": described["speed_scaling"],
                    "tcp_speed_m_s": round(math.sqrt(sum(float(v) ** 2 for v in tcp[:3])) / 1000.0, 3),
                    "joint_speed_dps": [round(v, 1) for v in (snap.get("QDActual") or [0.0] * 6)],
                    "cable": self.cable.state(),
                })
                return out
        joints = [round(float(v), 3) for v in self.arm_sim_joints]
        out.update({
            "joints": joints,
            "pose": [round(v, 3) for v in kin.fk(joints)],
            "pose_source": "model",
            "speed_scaling": 0.0,
            "cable": self.cable.state(),
        })
        return out

    def _arm_gate(self, current: List[float], target: List[float],
                  force: bool = False) -> Tuple[Optional[List[float]], Dict[str, Any]]:
        """下发前的闸门：手腕防缠 + 碰撞间距 + 视觉物体是否过期。

        返回 ``(实际要下发的关节角 或 None, 说明)``；``None`` 表示拒绝，原因在
        ``说明["error"]``。抽出来的原因：单步下发（``arm_move``）和组动作
        （``arm_action``）的每一段走的必须是**同一道**闸门，规则只能有一份。
        """
        info: Dict[str, Any] = {}
        # 手腕防缠：J4/J6 换成「累计扭转更小」的等效角，必要时反方向转。
        self.cable.observe(current)
        planned, cable_notes, cable_blocked = self.cable.choose(current, kin.clamp(target))
        info["cable"] = {**self.cable.state(), "notes": cable_notes}
        if cable_blocked:
            info["error"] = "；".join(cable_notes) or "手腕扭转会超过上限"
            return None, info
        # 安全校验用「静态工位 + 相机现在看到的物体」，不是写死的 CELL。
        path = cell_mod.check_path(current, planned, cell=self.scene.cell())
        info["clearance"] = path
        scene_now = self.scene.summary()
        info["scene"] = {"count": scene_now["count"],
                         "calibrated": scene_now["calibrated"],
                         "vision_age": scene_now["vision_age"],
                         "vision_stale": scene_now["vision_stale"]}
        if self.scene.calibrated and not force:
            if scene_now["vision_age"] is None:
                info["error"] = ("已标定桌面平面，但相机视觉没在跑（服务端不知道桌上有什么）；"
                                 "先打开相机，或确认现场后加 force=true")
                return None, info
            if scene_now["vision_stale"]:
                info["error"] = (f"相机视觉已停 {scene_now['vision_age']:.1f} s，桌面物体状态未知；"
                                 "确认现场无新增障碍可加 force=true")
                return None, info
        if not path.get("ok"):
            info["error"] = f"安全校验没跑起来：{path.get('error', '')}"
            return None, info
        if not path["safe"] and not force:
            pair = path.get("pair") or ["", ""]
            info["error"] = (f"路径最小间距 {path['min_m'] * 1000:.0f} mm"
                             f"（{pair[0]} ↔ {pair[1]}），低于 {cell_mod.SAFE_MM:.0f} mm 红线；"
                             "确定安全可加 force=true")
            return None, info
        return list(planned), info

    def _arm_precheck(self) -> Optional[Dict[str, Any]]:
        """真机能不能接指令：在线 / 使能 / 无报警 / 没在动。通过返回 ``None``。"""
        if not self._arm_live():
            return {"ok": False, "reason": "真机不在线（30004 没有数据）"}
        described = arm_feed.describe(self.arm_feed.snapshot())
        if not described["enabled"]:
            return {"ok": False, "reason": "臂未使能"}
        if described["error"]:
            return {"ok": False, "reason": f"臂有报警 {described['error']}"}
        if described["running"]:
            return {"ok": False, "reason": "臂正在运动"}
        return None

    def arm_move(self, joints: List[float], apply: bool = False,
                 timeout: float = 20.0, force: bool = False) -> Dict[str, Any]:
        """把视图里的姿态记下来；``apply`` 且真机在线时才真的动臂。

        ``apply`` 之前先走一遍 :func:`handlink.cell.check_path`：MovJ 是关节空间
        直线插值，所以整条路径都要扫，不能只看终点。低于安全间距就拒绝下发——
        这条闸门在服务端，前端绕不过去（``force=True`` 才会放行，留给现场调试）。
        """
        if len(joints) != 6:
            return {"ok": False, "error": "需要 6 个关节角"}
        clamped = kin.clamp(joints)
        self.arm_sim_joints = [float(v) for v in clamped]
        result: Dict[str, Any] = {"ok": True, "moved": False,
                                  "joints": self.arm_sim_joints,
                                  "clamped": clamped != [float(v) for v in joints]}
        if not apply:
            result["reason"] = "preview（没下发）"
            return result
        denied = self._arm_precheck()
        if denied:
            result.update(denied)
            return result

        current = [float(v) for v in arm_feed.describe(self.arm_feed.snapshot())["joints"]]
        planned, info = self._arm_gate(current, clamped, force=force)
        for key in ("cable", "clearance", "scene"):
            if key in info:
                result[key] = info[key]
        if planned is None:
            result.update({"ok": False, "reason": info.get("error", "安全闸门拒绝")})
            return result

        robot = arm_mod.DobotArm(ip=self.arm_ip or "192.168.5.1")
        try:
            robot.connect()
            robot.move_joints(planned, wait=True, timeout=timeout)
            result["moved"] = True
            result["joints"] = [round(v, 3) for v in planned]
            result["pose"] = [round(v, 3) for v in robot.pose()]
        except arm_mod.ArmError as exc:
            result["ok"] = False
            result["error"] = str(exc)
        finally:
            robot.close()
        return result

    def _arm_now(self) -> List[float]:
        """当前关节角：真机在线读 30004，否则用视图里的仿真值。"""
        if self._arm_live():
            try:
                return [float(v) for v in self.arm_feed.snapshot()["QActual"]]
            except arm_feed.FeedbackError:
                pass
        return [float(v) for v in self.arm_sim_joints]

    def arm_action(self, name: str, apply: bool = False,
                   start: Optional[List[float]] = None,
                   speed: Optional[float] = None, force: bool = False,
                   timeout: float = 20.0, **params: Any) -> Dict[str, Any]:
        """跑一条组动作（``handlink/arm_actions.py``），一段一段地走。

        * ``apply=False``：只把计划算出来，并把视图挪到最后一帧（和 ``/arm/joints``
          的 preview 一样，方便网页先看动画），不发任何指令。
        * ``apply=True``：真机在线且已使能才会动。**每一段**都过
          :meth:`_arm_gate`（手腕防缠 + 安全间距 + 视觉时效），任一段不过就
          ``Stop()`` 停在原地，响应里带 ``steps_done`` 说明做到哪了。

        起点默认是「现在这个姿态」，所以挥手从哪开始都能做；``start`` 只给调试和
        测试用（比如指定从某个姿态复现）。
        """
        seed = self._arm_now() if start is None else [float(v) for v in start]
        if len(seed) != 6:
            return {"ok": False, "error": "start 需要 6 个关节角"}
        try:
            moves = arm_actions.build(name, seed, **params)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        issues = arm_actions.validate(moves)
        errors = [i["message"] for i in issues if i["level"] == "error"]
        pct = float(speed) if speed not in (None, "") else arm_mod.SAFE_SPEED_FACTOR
        pct = max(1.0, min(100.0, pct))
        out: Dict[str, Any] = {
            "ok": not errors,
            "action": name,
            "label": arm_actions.ACTIONS.get(name, {}).get("label", name),
            "start": [round(v, 3) for v in seed],
            "steps": [m.as_dict() for m in moves],
            "plan": arm_actions.describe(moves, seed, pct),
            "estimate_s": arm_actions.estimate_seconds(moves, seed, pct),
            "issues": issues,
            "moved": False,
        }
        if errors:
            out["error"] = "；".join(errors)
            return out
        if not apply:
            self.arm_sim_joints = [float(v) for v in moves[-1].joints]
            out["reason"] = "preview（没下发）"
            return out
        denied = self._arm_precheck()
        if denied:
            out.update(denied)
            return out

        robot = arm_mod.DobotArm(ip=self.arm_ip or "192.168.5.1")
        done: List[Dict[str, Any]] = []
        min_mm: Optional[float] = None
        out["speed_factor"] = pct
        try:
            robot.connect()
            robot.speed_factor(pct)
            for i, move in enumerate(moves, 1):
                current = self._arm_now()
                planned, info = self._arm_gate(current, move.joints, force=force)
                if planned is None:
                    out.update({"ok": False, "step": i, "label_step": move.label,
                                "error": info.get("error", "安全闸门拒绝"),
                                "steps_done": done})
                    try:
                        robot.stop()
                    except arm_mod.ArmError:
                        pass
                    break
                path = info.get("clearance") or {}
                if path.get("min_m") is not None:
                    mm = float(path["min_m"]) * 1000.0
                    min_mm = mm if min_mm is None else min(min_mm, mm)
                robot.move_joints(planned, wait=True, timeout=timeout)
                done.append({"step": i, "label": move.label,
                             "joints": [round(v, 3) for v in planned],
                             "hold_ms": int(move.hold_ms)})
                # MovJ 的到位判据到得比 30004 的 RunningStatus 早，停一下再查下一段，
                # 不然下一段会被「臂正在运动」误挡。
                time.sleep(arm_actions.SETTLE_S + move.hold_ms / 1000.0)
            else:
                out.update({"ok": True, "steps_done": done})
        except arm_mod.ArmError as exc:
            out.update({"ok": False, "error": str(exc), "steps_done": done})
            try:
                robot.stop()
            except arm_mod.ArmError:
                pass
        finally:
            robot.close()
        out["moved"] = bool(done)
        out["complete"] = len(done) == len(moves)
        if min_mm is not None:
            out["min_clearance_mm"] = round(min_mm)
        last = done[-1]["joints"] if done else moves[-1].joints
        self.arm_sim_joints = [float(v) for v in last]
        return out


# ---------------------------------------------------------------- HTTP

class Handler(BaseHTTPRequestHandler):
    app: App
    server_version = "handlink/0.1"

    def log_message(self, fmt: str, *args: Any) -> None:  # 静音
        pass

    # -- 工具 -------------------------------------------------------------
    def _send(self, code: int, obj: Any) -> None:
        body = json.dumps(obj, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_bytes(self, body: bytes, content_type: str, filename: str = "") -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        if filename:
            self.send_header("Content-Disposition", f'inline; filename="{filename}"')
        self.end_headers()
        self.wfile.write(body)

    def _query(self) -> Dict[str, str]:
        raw = self.path.split("?", 1)
        if len(raw) < 2:
            return {}
        out: Dict[str, str] = {}
        for item in raw[1].split("&"):
            if "=" in item:
                k, v = item.split("=", 1)
                out[k] = v
        return out

    def _send_mjpeg(self) -> None:
        """multipart/x-mixed-replace 推流：浏览器 <img> 直接就能显示。"""
        cam = self.app.camera
        if not cam.running:
            return self._send(503, {"ok": False, "error": "摄像头没打开"})
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        last = -1
        try:
            while True:
                seq, jpg = cam.wait_frame(last, timeout=1.0)
                if jpg is None:
                    if not cam.running:
                        break
                    time.sleep(0.02)      # 还没编出第一帧，别空转
                    continue
                if seq == last:
                    continue
                last = seq
                head = (f"--frame\r\nContent-Type: image/jpeg\r\n"
                        f"Content-Length: {len(jpg)}\r\n\r\n").encode()
                self.wfile.write(head)
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except Exception:
            pass          # 客户端关页面了（BrokenPipe / 已关闭的 socket），正常收场
        self.close_connection = True

    def _body(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise ValueError(f"请求体不是合法 JSON: {exc}") from exc
        if not isinstance(data, dict):
            raise ValueError("请求体必须是 JSON 对象")
        return data

    # -- 路由 -------------------------------------------------------------
    def do_GET(self) -> None:
        try:
            self._route_get()
        except Exception as exc:
            self._send(500, {"ok": False, "error": str(exc)})

    def do_POST(self) -> None:
        try:
            self._route_post()
        except ValueError as exc:
            self._send(400, {"ok": False, "error": str(exc)})
        except Exception as exc:
            self._send(500, {"ok": False, "error": str(exc)})

    def _route_get(self) -> None:
        app = self.app
        path = self.path.split("?", 1)[0]
        if path in ("/demo", "/demo/"):
            return self._send_html(webui.HTML)
        if path in ("/camera", "/camera/"):
            return self._send_html(camui.HTML)
        if path in ("/arm3d", "/arm3d/"):
            return self._send_html(armview.HTML)
        if path in ("/vision", "/vision/"):
            return self._send_html(cubeui.HTML)
        if path == "/vision/state":
            return self._send(200, app.cube.state())
        if path == "/vision/snapshot.png":
            frame = app.camera.read_raw()
            if frame is None:
                return self._send(404, {"ok": False, "error": "没有画面（先打开摄像头）"})
            try:
                png = app.cube.annotated_png(frame)
            except Exception as exc:            # noqa: BLE001
                return self._send(500, {"ok": False, "error": str(exc)})
            return self._send_bytes(png, "image/png", "cube-vision.png")
        if path.startswith("/static/"):
            rel = path[len("/static/"):]
            base = Path(__file__).resolve().parent / "static"
            target = (base / rel).resolve()
            if (not rel or ".." in rel or not str(target).startswith(str(base))
                    or not target.is_file()):
                return self._send(404, {"ok": False, "error": f"没有这个静态文件: {rel}"})
            suffix = target.suffix.lower()
            ctype = {".js": "application/javascript", ".css": "text/css",
                     ".json": "application/json", ".stl": "model/stl",
                     ".png": "image/png", ".html": "text/html; charset=utf-8",
                     }.get(suffix, "application/octet-stream")
            return self._send_bytes(target.read_bytes(), ctype, target.name)
        if path == "/arm/model":
            return self._send(200, {"ok": True, **kin.model_dict()})
        if path == "/arm/cell":
            # ?live=1 返回「静态工位 + 相机看到的物体」，和下发闸门用的是同一份。
            live = self._query().get("live") in ("1", "true", "yes")
            return self._send(200, {"ok": True,
                                    **cell_mod.cell_dict(app.scene.cell() if live else None)})
        if path == "/scene/state":
            cam = app.camera.state()
            return self._send(200, {
                **app.scene.state(),
                "camera": {"running": bool(cam.get("running")), "fps": cam.get("fps", 0.0),
                           "size": [cam.get("width", 0), cam.get("height", 0)]},
                "hub": app.scene_summary()["hub"],
                "scan": app.scan.state(),
            })
        if path == "/scene/scan":
            return self._send(200, {**app.scan.state(),
                                    "summary": app.scene.summary(),
                                    "hub": app.scene_summary()["hub"]})
        if path == "/scene/scan/annotated.jpg":
            frame = app.camera.read_raw()
            jpg = app.scan.annotate(frame) if frame is not None else None
            if jpg is None:
                return self._send(404, {"ok": False, "error": "没有画面（先打开摄像头）"})
            return self._send_bytes(jpg, "image/jpeg", "scan-annotated.jpg")
        if path == "/scene/scan/map.jpg":
            frame = app.camera.read_raw()
            jpg = app.scan.map_image(frame) if frame is not None else None
            if jpg is None:
                return self._send(404, {"ok": False,
                                        "error": "没有画面或桌面没标定"})
            return self._send_bytes(jpg, "image/jpeg", "scan-map.jpg")
        if path == "/arm/state":
            return self._send(200, app.arm_snapshot())
        if path == "/arm/poses":
            return self._send(200, app.arm_pose_list())
        if path == "/arm/actions":
            return self._send(200, {"ok": True, "actions": arm_actions.catalogue(),
                                    "max_step_deg": arm_actions.MAX_STEP_DEG,
                                    "max_hold_ms": arm_actions.MAX_HOLD_MS})
        if path == "/arm/cable":
            return self._send(200, {"ok": True, **app.cable.state()})
        if path == "/camera.mjpg":
            return self._send_mjpeg()
        if path == "/camera/state":
            return self._send(200, app.camera.state())
        if path == "/camera/devices":
            force = self._query().get("force") in ("1", "true", "yes")
            return self._send(200, {"ok": True, "cv2": cam_mod.HAS_CV2,
                                    "devices": cam_mod.list_devices(),
                                    "streams": cam_mod.probe_devices(force=force)})
        if path == "/camera/snapshot.png":
            png = app.camera.snapshot_png()
            if png is None:
                return self._send(404, {"ok": False, "error": "没有画面（先打开摄像头）"})
            return self._send_bytes(png, "image/png", "handlink.png")
        if path == "/camera/frame.jpg":
            # 给三维视图当桌面贴图用：小、快，不追求无损（PNG 一张 400 KB，
            # 2 Hz 叠加就吃掉 1 MB/s；JPEG q72 只有 30～50 KB）。
            frame = app.camera.read_raw()
            if frame is None or cam_mod.cv2 is None:
                return self._send(404, {"ok": False, "error": "没有画面（先打开摄像头）"})
            ok, buf = cam_mod.cv2.imencode(
                ".jpg", frame, [int(cam_mod.cv2.IMWRITE_JPEG_QUALITY), 72])
            if not ok:
                return self._send(500, {"ok": False, "error": "JPEG 编码失败"})
            return self._send_bytes(buf.tobytes(), "image/jpeg", "scene.jpg")
        if path == "/audio/devices":
            return self._send(200, app.audio_devices())
        if path == "/audio/state":
            return self._send(200, app.audio.state())
        if path == "/audio/clips":
            return self._send(200, {"ok": True, "dir": str(app.audio_dir),
                                    "clips": audio_mod.clips(app.audio_dir)})
        if path.startswith("/audio/file/"):
            name = path.rsplit("/", 1)[-1]
            target = app.audio_dir / name
            if ("/" in name or "\\" in name or not name.lower().endswith(".wav")
                    or not target.is_file()):
                return self._send(404, {"ok": False, "error": f"没有这段录音: {name}"})
            return self._send_bytes(target.read_bytes(), "audio/wav", name)
        if path in ("/plan", "/plan/"):
            page = Path(__file__).resolve().parent.parent / "docs" / "plan.html"
            if page.is_file():
                return self._send_html(page.read_text(encoding="utf-8"))
            return self._send(404, {"ok": False, "error": "docs/plan.html 不存在"})
        if path in ("/health", "/"):
            return self._send(200, app.health())
        if path == "/limits":
            return self._send(200, {"ok": True, "motors": spec.describe_table()})
        if path == "/actions":
            return self._send(200, app.action_list())
        if path == "/ai/skills":
            return self._send(200, {"ok": True, "skills": skills.schema(),
                                    "max_steps": skills.MAX_STEPS,
                                    "max_total_ms": skills.MAX_TOTAL_MS})
        if path == "/hand/status":
            return self._send(200, app.hand_status())
        if path == "/glove/state":
            return self._send(200, {"ok": True, **app.glove.state()})
        if path == "/glove/limits":
            return self._send(200, {"ok": True, "fingers": glove_mod.finger_table()})
        if path == "/state":
            return self._send(200, {"ok": True, **app.bus.snapshot()})
        if path == "/points":
            return self._send(200, {"ok": True, "dir": str(app.dirs.points),
                                    "points": store.list_points(app.dirs.points)})
        if path.startswith("/points/"):
            name = path.rsplit("/", 1)[-1]
            pose = store.load_point(store.resolve_point(name, app.dirs.points))
            return self._send(200, {"ok": True, "name": name,
                                    "motors": spec.to_vendor(pose)})
        if path == "/tracks":
            return self._send(200, {"ok": True, "dir": str(app.dirs.tracks),
                                    "tracks": store.list_tracks(app.dirs.tracks)})
        if path.startswith("/tracks/"):
            name = path.rsplit("/", 1)[-1]
            track = store.load_track(app.dirs.tracks / f"{name}.txt")
            return self._send(200, {"ok": True, "name": name, **track.as_dict()})
        if path == "/record/stats":
            return self._send(200, app.recording_stats())
        return self._send(404, {"ok": False, "error": f"未知路径 {path}"})

    def _route_post(self) -> None:
        app = self.app
        path = self.path.split("?", 1)[0]
        data = self._body()

        if path == "/hand/attach":
            raw_id = data.get("hand_id")
            if isinstance(raw_id, str):
                raw_id = int(raw_id, 0) if raw_id.strip() else None
            result = app.attach_hand(bus_name=str(data.get("bus", "can")),
                                     dll=data.get("dll") or None,
                                     hand_id=raw_id)
            return self._send(200 if result.get("ok") else 422, result)

        if path == "/hand/enable":
            on = data.get("on", True)
            if isinstance(on, str):
                on = on.strip().lower() in ("1", "true", "yes", "on")
            result = app.hand_enable(bool(on))
            return self._send(200 if result.get("ok") else 422, result)

        if path == "/glove/start":
            try:
                result = app.glove.start(
                    udp=int(data.get("udp", glove_mod.DEFAULT_UDP)),
                    bind=str(data.get("bind") or glove_mod.DEFAULT_BIND),
                    side=str(data.get("side") or "right"),
                    gain=float(data.get("gain", 1.0)),
                    rate=float(data.get("rate", glove_mod.DEFAULT_RATE)),
                    speed=float(data.get("speed", glove_mod.DEFAULT_SPEED)),
                    torque=float(data.get("torque", glove_mod.DEFAULT_TORQUE)),
                    full_scale=float(data.get("full_scale", glove_mod.FULL_SCALE_DEG)),
                    frame=str(data.get("frame") or glove_mod.DEFAULT_FRAME),
                    deadband=data.get("deadband") or None,
                    dry_run=bool(data.get("dry_run", False)),
                )
            except (TypeError, ValueError) as exc:
                return self._send(400, {"ok": False, "error": f"参数不对：{exc}"})
            return self._send(200 if result.get("ok") else 422, result)

        if path == "/glove/stop":
            return self._send(200, app.glove.stop())

        if path == "/glove/mode":
            try:
                return self._send(200, app.glove.set_mode(str(data.get("mode", ""))))
            except ValueError as exc:
                return self._send(400, {"ok": False, "error": str(exc)})

        if path == "/glove/toggle":
            return self._send(200, app.glove.toggle_mode())

        if path == "/glove/zero":
            return self._send(200, app.glove.zero())

        if path == "/glove/frame":
            try:
                return self._send(200, app.glove.set_frame(str(data.get("frame", ""))))
            except ValueError as exc:
                return self._send(400, {"ok": False, "error": str(exc)})

        if path == "/glove/deadband":
            try:
                return self._send(200, app.glove.set_deadband(data.get("deadband") or {}))
            except ValueError as exc:
                return self._send(400, {"ok": False, "error": str(exc)})

        if path == "/glove/save":
            name = str(data.get("name") or "").strip()
            if not name or any(c in name for c in '\\/:*?"<>|'):
                return self._send(400, {"ok": False, "error": f"点位名不合法：{name!r}"})
            state = app.glove.state()
            if not state["live"]:
                return self._send(409, {"ok": False,
                                        "error": "手套没在跟手（没有新帧），先把手指摆成要的点位",
                                        "hint": state["hint"]})
            frames = int(data.get("frames", glove_mod.SNAPSHOT_FRAMES))
            motors = app.glove.snapshot(frames)
            if len(motors) < len(spec.MOTOR_IDS):
                return self._send(409, {"ok": False, "error": "数据不全，再摆一次"})
            speed = float(data.get("speed", app.glove.speed))
            torque = float(data.get("torque", app.glove.torque))
            pose = spec.clamp({mid: spec.Joint(deg, speed, torque)
                               for mid, deg in motors.items()})
            target = app.dirs.points / f"{name}.txt"
            if target.exists() and not data.get("overwrite"):
                return self._send(409, {"ok": False,
                                        "error": f"点位已存在：{target.name}"
                                                 "（要覆盖就加 \"overwrite\": true）"})
            store.save_point(target, pose)
            hit = [str(mid) for mid, deg in motors.items()
                   if deg >= app.glove.limits[mid] - 0.05]
            return self._send(200, {
                "ok": True, "name": name, "path": str(target), "frames": frames,
                "motors": {str(k): round(v, 1) for k, v in motors.items()},
                "saturated": hit, "issues": spec.check(pose),
            })

        if path == "/glove/teach":
            action = str(data.get("action") or "status").strip().lower()
            if action == "start":
                return self._send(200, app.glove.teach_start())
            if action == "stop":
                return self._send(200, app.glove.teach_stop())
            if action == "clear":
                return self._send(200, app.glove.clear_axes())
            if action == "status":
                return self._send(200, {"ok": True, "global": app.glove.teaching,
                                        "axes": {str(k): [round(c, 4) for c in v]
                                                 for k, v in app.glove.axes.items()}})
            return self._send(400, {"ok": False, "error": f"action 只能是 start/stop/clear，收到 {action!r}"})

        if path == "/glove/gain":
            try:
                return self._send(200, app.glove.set_gain(float(data.get("gain", 1.0))))
            except (TypeError, ValueError) as exc:
                return self._send(400, {"ok": False, "error": f"参数不对：{exc}"})

        if path == "/pose":
            payload = data.get("motors")
            if payload is None:
                payload = {k: v for k, v in data.items() if k not in ("reason", "dry_run")}
            pose = spec.from_vendor(payload)
            held = [mid for mid in spec.MOTOR_IDS if mid not in pose]
            pose = spec.complete(pose, fallback=app.bus.pose())
            result = app.apply(pose, reason=data.get("reason", "http:/pose"), held=held)
            return self._send(200 if result["ok"] else 422, result)

        if path == "/point":
            result = app.apply_point(data["name"], reason=data.get("reason"))
            return self._send(200 if result["ok"] else 422, result)

        if path == "/point/nudge":
            name = str(data.get("name") or "")
            try:
                target = store.resolve_point(name, app.dirs.points)
                pose = store.load_point(target)
            except (FileNotFoundError, ValueError) as exc:
                return self._send(404, {"ok": False, "error": str(exc)})
            try:
                for key, deg in (data.get("delta") or {}).items():
                    mid = spec.resolve(key).motor_id
                    joint = pose[mid]
                    pose[mid] = spec.Joint(float(joint.angle) + float(deg),
                                           joint.speed, joint.torque)
            except (KeyError, TypeError, ValueError) as exc:
                return self._send(400, {"ok": False, "error": f"delta 不对：{exc}"})
            pose = spec.clamp(pose)
            issues = spec.check(pose)
            store.save_point(target, pose)
            out = {"ok": True, "name": name,
                   "motors": {str(k): round(v.angle, 1) for k, v in pose.items()},
                   "issues": issues}
            if data.get("apply", True) and not spec.has_errors(issues):
                out["applied"] = app.apply(pose, reason=f"point:nudge:{name}")
            return self._send(200, out)

        if path == "/track":
            result = app.play_track(
                data["name"],
                scale=float(data.get("scale", 1.0)),
                dry_run=bool(data.get("dry_run", False)),
            )
            return self._send(200 if result["ok"] else 422, result)

        if path == "/action":
            result = app.run_action(str(data.get("name", "")))
            return self._send(200 if result["ok"] else 422, result)

        if path == "/stop":
            return self._send(200, app.stop())

        if path == "/arm/joints":
            joints = data.get("joints") or []
            result = app.arm_move(joints, apply=bool(data.get("apply", False)),
                                  timeout=float(data.get("timeout", 20.0)),
                                  force=bool(data.get("force", False)))
            return self._send(200 if result["ok"] else 422, result)

        if path == "/arm/action":
            # 组动作：{"name":"wave","apply":false,"speed":20,"cycles":3}
            # params 只接受 arm_actions 里登记的键；未知键会被 build() 明确报错。
            name = str(data.get("name") or "")
            if not name:
                return self._send(400, {"ok": False, "error": "缺 name（GET /arm/actions 有清单）"})
            start = data.get("start")
            if start is not None and (not isinstance(start, list) or len(start) != 6):
                return self._send(400, {"ok": False, "error": "start 需要 6 个关节角"})
            known = set(arm_actions.ACTIONS.get(name, {}).get("defaults", {}))
            params = {k: data[k] for k in known if k in data}
            try:
                speed = data.get("speed")
                result = app.arm_action(
                    name, apply=bool(data.get("apply", False)),
                    start=[float(v) for v in start] if start else None,
                    speed=None if speed in (None, "") else float(speed),
                    force=bool(data.get("force", False)),
                    timeout=float(data.get("timeout", 20.0)),
                    **params)
            except (TypeError, ValueError) as exc:
                return self._send(400, {"ok": False, "error": f"参数不对：{exc}"})
            return self._send(200 if result["ok"] else 422, result)

        if path == "/arm/check":
            joints = data.get("joints") or []
            if len(joints) != 6:
                return self._send(400, {"ok": False, "error": "需要 6 个关节角"})
            start = data.get("from")
            if not start:
                if app._arm_live():
                    start = [float(v) for v in app.arm_feed.snapshot()["QActual"]]
                else:
                    start = list(app.arm_sim_joints)
            from_joints = [float(v) for v in start]
            planned, notes, blocked = app.cable.choose(from_joints,
                                                       kin.clamp([float(v) for v in joints]))
            result = cell_mod.check_path(from_joints, planned, cell=app.scene.cell())
            result["scene"] = app.scene.summary()
            result["cable"] = {**app.cable.state(), "notes": notes}
            result["adjusted_joints"] = [round(v, 3) for v in planned]
            if blocked:
                result.update({"ok": False, "safe": False, "error": "；".join(notes)})
            result["ok"] = result.get("ok", False)
            return self._send(200 if result["ok"] else 422, result)

        if path == "/arm/cable/mark":
            if app._arm_live():
                joints = [float(v) for v in app.arm_feed.snapshot()["QActual"]]
            else:
                joints = list(app.arm_sim_joints)
            app.cable.mark(joints)
            return self._send(200, {"ok": True, **app.cable.state()})

        if path == "/arm/poses":
            # 保存姿态：{"name":"桌面抓取","joints":[...],"note":"..."}；
            # joints 不给就存「现在这个姿态」（真机在线读 30004）。
            name = data.get("name")
            raw = data.get("joints")
            joints: Optional[List[float]] = None
            if raw is not None:
                if not isinstance(raw, list) or len(raw) != 6:
                    return self._send(400, {"ok": False, "error": "joints 需要 6 个数"})
                try:
                    joints = [float(v) for v in raw]
                except (TypeError, ValueError):
                    return self._send(400, {"ok": False, "error": "joints 里有非数字"})
            try:
                result = app.arm_pose_save(
                    "" if name is None else str(name),
                    joints=joints,
                    note=str(data.get("note") or ""),
                    source=str(data.get("source") or ""))
            except (TypeError, ValueError) as exc:
                return self._send(422, {"ok": False, "error": str(exc)})
            return self._send(200, result)

        if path == "/arm/poses/delete":
            try:
                result = app.arm_pose_delete(str(data.get("name") or ""))
            except KeyError as exc:
                return self._send(404, {"ok": False, "error": str(exc.args[0])})
            except (TypeError, ValueError) as exc:
                return self._send(400, {"ok": False, "error": str(exc)})
            return self._send(200, result)

        if path == "/arm/attach":
            ip = str(data.get("ip") or app.arm_ip or "192.168.5.1")
            app.close_arm()
            ok = app.attach_arm(ip)
            return self._send(200 if ok else 503,
                              {"ok": ok, "ip": ip, "error": app.arm_error})

        if path == "/arm/enable":
            on = data.get("on", True)
            if isinstance(on, str):
                on = on.strip().lower() in ("1", "true", "yes", "on")
            speed = data.get("speed")
            try:
                speed_val = None if speed in (None, "") else float(speed)
            except (TypeError, ValueError):
                return self._send(400, {"ok": False, "error": "speed 需要是数字"})
            result = app.arm_enable(bool(on), speed=speed_val)
            code = 200 if result.get("ok") else (503 if not result.get("online", True) else 422)
            return self._send(code, result)

        if path == "/arm/drag":
            on = data.get("on", True)
            if isinstance(on, str):
                on = on.strip().lower() in ("1", "true", "yes", "on")
            result = app.arm_drag(bool(on))
            code = 200 if result.get("ok") else (503 if not result.get("online", True) else 422)
            return self._send(code, result)

        if path == "/scene/calibrate":
            pixels = data.get("pixels") or []
            world = data.get("world")
            try:
                result = app.scene.calibrate(
                    pixels, world,
                    rotate=int(data.get("rotate") or 0),
                    mirror=bool(data.get("mirror", False)),
                    note=str(data.get("note") or ""),
                )
            except ValueError as exc:
                return self._send(422, {"ok": False, "error": str(exc)})
            app.ensure_scene_hub()
            return self._send(200, result)

        if path == "/scene/place":
            try:
                size = data.get("size") or ([0.06, 0.12] if data.get("kind") == "cylinder"
                                            else [0.06, 0.06, 0.06])
                result = app.scene.place(
                    name=str(data.get("name") or "物体"),
                    u=data.get("u"), v=data.get("v"),
                    x=data.get("x"), y=data.get("y"),
                    size=size, kind=str(data.get("kind") or "box"),
                    yaw=float(data.get("yaw") or 0.0),
                    ttl=float(data.get("ttl") or 0.0),
                    meta=data.get("meta") if isinstance(data.get("meta"), dict) else None,
                )
            except ValueError as exc:
                return self._send(422, {"ok": False, "error": str(exc)})
            return self._send(200, {"ok": True, "object": result})

        if path == "/scene/scan":
            action = str(data.get("action") or "").strip()
            if action == "background":
                frame = app.camera.read_raw()
                if frame is None:
                    return self._send(409, {"ok": False,
                                            "error": "没有画面（先打开摄像头）"})
                try:
                    app.scan.set_background(frame)
                except RuntimeError as exc:
                    return self._send(409, {"ok": False, "error": str(exc)})
                return self._send(200, {**app.scan.state(),
                                        "hint": "空桌背景已保存；现在放东西，再点开始扫描"})
            if action == "start":
                if not app.scene.calibrated:
                    return self._send(409, {"ok": False,
                                            "error": "桌面还没标定（先点桌面四角）"})
                app.scan.start()
                app.ensure_scene_hub()
                return self._send(200, app.scan.state())
            if action == "stop":
                return self._send(200, app.scan.stop())
            if action == "clear_background":
                return self._send(200, app.scan.clear_background())
            if action == "once":
                frame = app.camera.read_raw()
                if frame is None:
                    return self._send(409, {"ok": False,
                                            "error": "没有画面（先打开摄像头）"})
                result = app.scan.run(frame)
                return self._send(200, {"ok": True, "result": result,
                                        "scan": app.scan.state(),
                                        "summary": app.scene_summary()})
            return self._send(422, {"ok": False, "error": f"未知动作：{action}"})

        if path == "/scene/object":
            # 页面上点物体改名 / 改尺寸 / 删掉
            name = str(data.get("name") or "")
            if not name:
                return self._send(422, {"ok": False, "error": "缺 name"})
            if data.get("remove"):
                return self._send(200, {"ok": True,
                                        "removed": app.scene.remove_object(name)})
            try:
                obj = app.scene.update_object(
                    name, new_name=data.get("new_name"),
                    size=data.get("size"), yaw=data.get("yaw"),
                    kind=data.get("kind"))
            except ValueError as exc:
                return self._send(422, {"ok": False, "error": str(exc)})
            return self._send(200, {"ok": True, "object": obj})

        if path == "/scene/clear":
            source = data.get("source")
            n = app.scene.clear_objects(str(source) if source else None)
            return self._send(200, {"ok": True, "cleared": n})

        if path == "/scene/calibration/clear":
            return self._send(200, app.scene.clear_calibration())

        if path == "/scene/config":
            hub = app.ensure_scene_hub()
            hz = hub.set_hz(float(data.get("hz", hub.hz)))
            return self._send(200, {"ok": True, "hz": hz, "hub": hub.state()})

        if path == "/scene/detect":
            # 立刻跑一次（不依赖后台线程的节奏），用来验证标定和读数。
            frame = app.camera.read_raw()
            if frame is None:
                return self._send(409, {"ok": False, "error": "没有画面（先打开摄像头）"})
            reading = app.cube.analyze(frame)
            app.scene.update_cube(reading)
            box = None
            # 读 9 格失败时退回「只当它是一个立方体」：抓取只要位置。
            if not reading.get("found"):
                det = boxcv.find_box(frame)
                if det.get("found"):
                    loc = boxcv.locate(app.scene, det)
                    det["located"] = loc
                    if loc.get("ok"):
                        obj = app.scene.update_box(
                            loc["center"],
                            size=(loc["size_m"], loc["size_m"], loc["size_m"]),
                            yaw=float(loc.get("yaw") or 0.0),
                            meta={"origin": "box", "px": det.get("center_px"),
                                  "bbox": det.get("bbox"), "fill": det.get("fill"),
                                  "ms": det.get("ms")},
                        )
                        det["object"] = obj.get("object")
                box = det
            return self._send(200, {"ok": True, "reading": reading, "box": box,
                                    "summary": app.scene_summary()})

        if path == "/camera/open":
            cfg = cam_mod.CameraConfig.from_dict(data)
            app.camera.config = cfg
            try:
                result = app.camera.open()
            except RuntimeError as exc:
                return self._send(409, {"ok": False, "error": str(exc)})
            if result.get("ok"):
                app.ensure_scene_hub()      # 相机一开，标定过的场景就开始跑视觉
            return self._send(200, result)

        if path == "/camera/close":
            return self._send(200, app.camera.close())

        if path == "/camera/detect":
            result = app.camera.enable_recognition(
                enabled=bool(data.get("enable", True)),
                every=int(data.get("every", 6)),
                mode=str(data.get("mode", "hand")),
            )
            return self._send(200 if result.get("ok") else 422, result)

        if path == "/camera/track":
            rect = [data.get("x", 0), data.get("y", 0),
                    data.get("w", 140), data.get("h", 110)]
            result = app.camera.set_template(rect)
            return self._send(200 if result.get("ok") else 422, result)

        if path == "/vision/calibrate":
            return self._vision_calibrate(app, data)

        if path == "/vision/quad":
            return self._send(200, app.cube.set_manual(data.get("points")))

        if path == "/vision/analyze":
            frame = app.camera.read_raw()
            if frame is None:
                return self._send(409, {"ok": False, "error": "没有画面（先打开摄像头）"})
            reading = app.cube.analyze(frame)
            return self._send(200, {"ok": True, **reading})

        if path == "/audio/start":
            result = app.audio_start(data)
            return self._send(200 if result.get("ok") else 409, result)

        if path == "/audio/stop":
            return self._send(200, app.audio.stop())

        if path == "/audio/test":
            result = app.audio_test(data)
            return self._send(200 if result.get("ok") else 409, result)

        if path == "/ai":
            text = str(data.get("text", "")).strip()
            if not text:
                return self._send(400, {"ok": False, "error": "text 不能为空"})
            try:
                result = app.ai_run(
                    text,
                    dry_run=bool(data.get("dry_run", False)),
                    prefer_llm=bool(data.get("prefer_llm", False)),
                    speed_scale=float(data.get("speed_scale", 1.0)),
                )
            except ValueError as exc:
                return self._send(422, {"ok": False, "error": str(exc)})
            return self._send(200 if result.get("ok") else 422, result)

        if path == "/record/start":
            return self._send(200, app.start_recording(data.get("path"), float(data.get("hz", 20))))

        if path == "/record/stop":
            return self._send(200, app.stop_recording())

        if path == "/analyze":
            target = Path(data["path"])
            if not target.is_absolute():
                target = app.workdir / target
            stats = rec_mod.analyze(target)
            return self._send(200, {"ok": True, "stats": stats,
                                    "markdown": rec_mod.to_markdown(stats)})

        return self._send(404, {"ok": False, "error": f"未知路径 {path}"})


def _vision_calibrate(app: App, data: Dict[str, Any]) -> None:
    """魔方视觉的标定动作：学一个面的颜色 / 设托盘基准 / 清空。"""
    op = str(data.get("op") or "").strip().lower()
    if op == "reset":
        app.cube.reset()
        return app._send(200, {"ok": True, "reset": True})
    frame = app.camera.read_raw()
    if frame is None:
        return app._send(409, {"ok": False, "error": "没有画面（先打开摄像头）"})
    if op == "palette":
        face = str(data.get("face") or "").strip().upper()
        if face not in cubecv.LETTERS:
            return app._send(400, {"ok": False,
                                   "error": f"face 只能是 {'/'.join(cubecv.LETTERS)}"})
        result = app.cube.learn(face, frame)
        return app._send(200 if result.get("ok") else 422, result)
    if op == "reference":
        result = app.cube.set_reference(frame)
        return app._send(200 if result.get("ok") else 422, result)
    return app._send(400, {"ok": False, "error": f"未知 op: {op or '(空)'}（palette/reference/reset）"})


class _Server(ThreadingHTTPServer):
    # Windows 上 SO_REUSEADDR 允许两个进程绑同一端口，会让旧进程继续应答，
    # 所以这里关掉，端口占用就明确报错。
    allow_reuse_address = os.name != "nt"
    daemon_threads = True


def make_server(app: App, host: str = "127.0.0.1", port: int = DEFAULT_PORT) -> ThreadingHTTPServer:
    handler = type("BoundHandler", (Handler,), {"app": app})
    httpd = _Server((host, port), handler)
    return httpd


def serve(app: App, host: str = "127.0.0.1", port: int = DEFAULT_PORT) -> None:  # pragma: no cover
    httpd = make_server(app, host, port)
    print(f"handlink 服务已启动: http://{host}:{httpd.server_address[1]}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
