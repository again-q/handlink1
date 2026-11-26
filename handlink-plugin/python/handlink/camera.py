"""摄像头：把 USB 摄像头变成「随时能拿到最新一帧」的对象。

网页 `/camera` 看到的画面就是这个模块推出去的，识别结果（如果有）也叠在这上面。

设计要点
--------
* **只留最新帧**：后台线程一直 ``read()``，旧帧直接丢，不在内存里堆积——摄像头
  不丢帧、网页也不会越看越慢。
* **一次编码，多人共用**：JPEG 在采集线程里编好，所有 ``/camera.mjpg`` 客户端拿同一份。
* **现场装歪很常见**：``rotate`` / ``mirror`` 就在这里处理，别让每个下游自己转。
* **没有 cv2 也能用**：``probe_devices()`` 只读注册表，页面照样能列设备；
  真正取流时才要求 opencv。

依赖（只在真正取流时需要）::

    pip install opencv-python

本机如果装不上（沙箱里 pip 写临时目录会 EACCES），可以下 wheel 解压到某个目录，
再用 ``PYTHONPATH`` 指过去；``candidate_paths()`` 会自动找 ``.tmp/pylibs``。
"""
from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def candidate_paths() -> List[Path]:
    """opencv 可能被手动解压在哪些目录（按优先级）。"""
    here = Path(__file__).resolve().parent.parent
    out = []
    env = os.environ.get("HANDLINK_PYLIB")
    if env:
        out.append(Path(env))
    out += [here / ".tmp" / "pylibs", here / "vendor" / "pylibs"]
    return out


# ---------------------------------------------------------------- opencv

# 摄像头掉线时 OpenCV 会按帧刷 WARN（一秒几十条，日志能刷到几百 KB），
# 在 import 之前把日志级别压下去；状态我们自己在 /camera/state 里报。
os.environ.setdefault("OPENCV_LOG_LEVEL", "SILENT")
os.environ.setdefault("OPENCV_VIDEOIO_DEBUG", "0")


def load_cv2():
    """导入 opencv；找不到时返回 None，并把可尝试的路径记下来。"""
    try:
        import cv2  # type: ignore

        return cv2
    except ImportError:
        pass
    for path in candidate_paths():
        if (path / "cv2").is_dir() and str(path) not in sys.path:
            sys.path.insert(0, str(path))
            try:
                import cv2  # type: ignore

                return cv2
            except ImportError:
                continue
    return None


cv2 = load_cv2()
HAS_CV2 = cv2 is not None

BACKENDS: Dict[str, str] = {}
BACKEND_LABELS = {"any": "auto", "dshow": "DirectShow", "msmf": "MSMF"}
if HAS_CV2:
    BACKENDS = {"any": cv2.CAP_ANY, "dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}

ROTATIONS = (0, 90, 180, 270)
QUALITY = 80


def _rotate_flag(deg: int):
    return {
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }.get(deg % 360)


# ---------------------------------------------------------------- 配置

@dataclass
class CameraConfig:
    index: int = 0
    backend: str = "any"          # any / dshow / msmf
    width: int = 640
    height: int = 480
    rotate: int = 0               # 0 / 90 / 180 / 270（顺时针）
    mirror: bool = False

    def __post_init__(self) -> None:
        self.rotate = int(self.rotate) % 360
        if self.rotate not in ROTATIONS:
            self.rotate = 0
        if self.backend not in ("any", "dshow", "msmf"):
            self.backend = "any"
        self.index = max(0, int(self.index))
        self.width = max(0, int(self.width))
        self.height = max(0, int(self.height))

    def as_dict(self) -> Dict[str, Any]:
        return {
            "index": self.index, "backend": self.backend,
            "width": self.width, "height": self.height,
            "rotate": self.rotate, "mirror": bool(self.mirror),
        }

    @classmethod
    def from_dict(cls, data: Optional[Dict[str, Any]]) -> "CameraConfig":
        data = data or {}
        return cls(
            index=data.get("index", 0),
            backend=str(data.get("backend", "any")),
            width=data.get("width", 1280),
            height=data.get("height", 800),
            rotate=data.get("rotate", 0),
            mirror=bool(data.get("mirror", False)),
        )


# ---------------------------------------------------------------- 采集

class Camera:
    """一路摄像头的采集线程 + 最新帧缓存。同一时间只开一路。"""

    def __init__(self, config: Optional[CameraConfig] = None) -> None:
        self.config = config or CameraConfig()
        self.recognizer: Any = None
        self._cap: Any = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._cond = threading.Condition()
        self._frame: Any = None          # 最新原始帧（BGR ndarray）
        self._jpeg: Optional[bytes] = None
        self._seq = 0
        self.frames = 0
        self.errors = 0
        self.reconnects = 0
        self.last_error = ""
        self.opened_at = 0.0
        self.width = 0
        self.height = 0
        self.fps = 0.0
        self._stamps: List[float] = []
        self._detect_every = 6
        self._last_detect_ms = 0.0
        self._last_result: Dict[str, Any] = {}
        self._overlay_error = ""

    # -- 生命周期 ---------------------------------------------------------
    @property
    def running(self) -> bool:
        return bool(self._thread and self._thread.is_alive())

    def open(self) -> Dict[str, Any]:
        if not HAS_CV2:
            raise RuntimeError(
                "没有 opencv：先 pip install opencv-python"
                + "（或设 HANDLINK_PYLIB 指向装好的目录，例如 .tmp\\pylibs）"
            )
        self.close()
        time.sleep(0.4)          # 后端刚释放就立刻重开会崩（DShow 尤其）
        cfg = self.config
        wanted = (cfg.backend, cfg.index)
        # 只在本后端内换 index：跨后端（MSMF↔DirectShow）切同一台设备时，
        # OpenCV 的 DirectShow 后端会抛 native 异常，整个进程都会没，救不回来。
        tries: List[Tuple[str, int]] = [wanted]
        for idx in range(4):                       # 索引会漂：同后端都试一遍
            if (cfg.backend, idx) not in tries:
                tries.append((cfg.backend, idx))

        failed: List[str] = []
        # 第一轮按页面要的分辨率开；全都出不了图就再来一轮「不改分辨率」。
        # 摄像头不一定支持页面存的那个尺寸，硬 set 之后 read 会一直空。
        for force_size in (True, False):
            if not force_size:
                time.sleep(0.3)
            for backend, index in tries:
                cap, frame = self._try_open(backend, index, force_size=force_size)
                if cap is None:
                    failed.append(f"{backend}#{index}")
                    continue
                if (backend, index) != wanted:
                    self.config = CameraConfig(index=index, backend=backend,
                                               width=cfg.width, height=cfg.height,
                                               rotate=cfg.rotate, mirror=cfg.mirror)
                self._cap = cap
                self.width = int(frame.shape[1])
                self.height = int(frame.shape[0])
                self._frame = frame
                self._stop.clear()
                self.frames = 0
                self.errors = 0
                self.last_error = ""
                self._stamps.clear()
                self.opened_at = time.time()
                self._thread = threading.Thread(target=self._run, name="camera", daemon=True)
                self._thread.start()
                return {"ok": True, "fallback": (backend, index) != wanted,
                        "native_size": not force_size,
                        "used": f"{backend}#{index}", **self.state()}
        raise RuntimeError("这些组合都出不了图：" + "、".join(sorted(set(failed))) +
                           "（被占用？设备在别的程序里开着？）")

    def _try_open(self, backend: str, index: int, force_size: bool = True) -> Tuple[Any, Any]:
        """开一个（后端, index），1.5 秒内拿不到帧就放弃。

        有的 UVC（比如新接的 USB2.0 PC CAMERA）只认 640×360，强行 set 1280×720 以后
        OpenCV 会在 ``read()`` 里抛断言；所以带 size 的那一轮失败后，再不带 size 试一次。
        """
        cfg = self.config
        cap = cv2.VideoCapture(index, BACKENDS.get(backend, cv2.CAP_ANY))
        if not cap.isOpened():
            cap.release()
            return None, None
        if force_size and cfg.width and cfg.height:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        deadline = time.time() + 1.5          # 有些后端首帧要等（曝光/启动）
        while time.time() < deadline:
            try:
                ok, frame = cap.read()
            except Exception:                 # 后端断言（分辨率不支持）当成没帧
                ok, frame = False, None
            if ok and frame is not None and frame.size:
                return cap, frame
            time.sleep(0.03)
        cap.release()
        if force_size and cfg.width and cfg.height:
            return self._try_open(backend, index, force_size=False)
        return None, None

    def close(self) -> Dict[str, Any]:
        self._stop.set()
        thread = self._thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._thread = None
        cap, self._cap = self._cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        with self._cond:
            self._seq += 1
            self._cond.notify_all()
        return {"ok": True, "running": False}

    # -- 采集线程 ---------------------------------------------------------
    def _reopen(self, cap):
        """设备掉线后自己接一次：同一个后端里换个 index 重开。

        摄像头 USB 松动、MSMF 丢设备时，read() 会一直返回 False。
        这里重开最多 3 轮、间隔 1 秒，接不上才真的放弃。
        """
        try:
            cap.release()
        except Exception:
            pass
        time.sleep(0.4)
        self.reconnects += 1
        cfg = self.config
        order = [cfg.index] + [i for i in range(4) if i != cfg.index]
        for _ in range(3):
            if self._stop.is_set():
                return None
            for index in order:
                cap2, frame = self._try_open(cfg.backend, index)
                if cap2 is not None:
                    self.config = CameraConfig(index=index, backend=cfg.backend,
                                               width=cfg.width, height=cfg.height,
                                               rotate=cfg.rotate, mirror=cfg.mirror)
                    self.width = int(frame.shape[1])
                    self.height = int(frame.shape[0])
                    self.last_error = ""
                    return cap2
            time.sleep(1.0)
        return None

    def _run(self) -> None:
        cap = self._cap
        bad = 0
        while not self._stop.is_set() and cap is not None:
            try:
                ok, frame = cap.read()
            except Exception as exc:      # 后端断言不能让线程崩掉
                ok, frame = False, None
                self.last_error = f"读帧异常: {exc}"
            if not ok or frame is None:
                self.errors += 1
                bad += 1
                if not self.last_error:
                    self.last_error = "读帧失败（设备被占用或已拔出）"
                if bad >= 60:              # 连续 2 秒没帧：先尝试自己接回来
                    cap = self._reopen(cap)
                    bad = 0
                    if cap is None:
                        self.last_error = self.last_error or "连续取不到画面，已停止取流"
                        break
                time.sleep(0.02)   # 别空转把 CPU 吃满
                continue
            bad = 0
            self.frames += 1
            self.last_error = ""
            if self.config.rotate:
                frame = cv2.rotate(frame, _rotate_flag(self.config.rotate))
            if self.config.mirror:
                frame = cv2.flip(frame, 1)

            view = frame
            if self.recognizer is not None and self.frames % max(1, self._detect_every) == 0:
                view = self._apply_recognition(frame)
            elif self._last_result:
                view = self.recognizer.draw(frame, self._last_result) if self.recognizer else frame

            ok, buf = cv2.imencode(".jpg", view, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])
            if not ok:
                self.errors += 1
                continue
            jpeg = buf.tobytes()
            now = time.time()
            self.width = int(view.shape[1])
            self.height = int(view.shape[0])
            self._stamps.append(now)
            if len(self._stamps) > 30:
                del self._stamps[:-30]
            if len(self._stamps) > 1:
                span = self._stamps[-1] - self._stamps[0]
                self.fps = round((len(self._stamps) - 1) / span, 1) if span > 0 else 0.0
            with self._cond:
                self._frame = frame
                self._jpeg = jpeg
                self._seq += 1
                self._cond.notify_all()
        if not self._stop.is_set() and cap is not None:
            try:                       # 连续没帧自己退出：把设备放掉，别占着
                cap.release()
            except Exception:
                pass

    def _apply_recognition(self, frame):
        t0 = time.perf_counter()
        try:
            result = self.recognizer.detect(frame)
            self._last_result = result or {}
            self._overlay_error = ""
        except Exception as exc:                    # 识别挂了不能拖垮取流
            self._last_result = {}
            self._overlay_error = str(exc)
            return frame
        finally:
            self._last_detect_ms = (time.perf_counter() - t0) * 1000.0
        try:
            return self.recognizer.draw(frame, self._last_result)
        except Exception as exc:
            self._overlay_error = f"overlay: {exc}"
            return frame

    # -- 取帧 -------------------------------------------------------------
    def wait_frame(self, after: int = -1, timeout: float = 1.0) -> Tuple[int, Optional[bytes]]:
        """等一个比 ``after`` 新的 JPEG；超时就把手上最新的那份还回去。"""
        with self._cond:
            if self._seq <= after and self._jpeg is not None:
                self._cond.wait(timeout)
            return self._seq, self._jpeg

    def read_raw(self):
        with self._cond:
            return None if self._frame is None else self._frame.copy()

    def snapshot(self) -> Optional[bytes]:
        with self._cond:
            return self._jpeg

    def snapshot_png(self) -> Optional[bytes]:
        """干净的一帧（不带识别叠加），存 PNG 用。"""
        frame = self.read_raw()
        if frame is None or not HAS_CV2:
            return None
        ok, buf = cv2.imencode(".png", frame)
        return buf.tobytes() if ok else None

    # -- 识别 -------------------------------------------------------------
    def enable_recognition(self, enabled: bool = True, every: int = 6,
                           mode: str = "hand") -> Dict[str, Any]:
        if enabled:
            from . import vision

            if mode == "robot":
                if not isinstance(self.recognizer, vision.RobotHand):
                    return {"ok": False,
                            "error": "还没有标定：先在画面上点一下手掌（/camera/track）",
                            "recognizer": vision.RobotHand().info()}
            elif mode == "cube":
                # 魔方视觉：服务和脚本共用同一个实例（标定状态都在它身上）
                from . import cubecv

                rec = cubecv.shared()
                if not isinstance(self.recognizer, cubecv.CubeVision):
                    self.recognizer = rec
            else:
                if not isinstance(self.recognizer, vision.HandLandmarks):
                    self.recognizer = vision.make_recognizer(mode)
            self._detect_every = max(1, int(every))
            info = self.recognizer.info()
            if not info.get("available"):
                self.recognizer = None
                return {"ok": False, "error": info.get("hint") or "识别器不可用", **info}
            return {"ok": True, "recognizer": info, "every": self._detect_every}
        self.recognizer = None
        self._last_result = {}
        return {"ok": True, "recognizer": {"name": "none", "available": True}}

    def set_template(self, rect) -> Dict[str, Any]:
        """用当前这一帧的某块区域当机械手的模板（点手掌用的接口）。"""
        from . import vision

        frame = self.read_raw()
        if frame is None:
            return {"ok": False, "error": "没有画面：先把摄像头打开"}
        rec = self.recognizer if isinstance(self.recognizer, vision.RobotHand) else vision.RobotHand()
        try:
            info = rec.set_template(frame, rect)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        self.recognizer = rec
        self._last_result = {}
        self.last_error = ""
        return {"ok": True, "recognizer": rec.info(), **info}

    # -- 状态 -------------------------------------------------------------
    def state(self) -> Dict[str, Any]:
        rec = self.recognizer.info() if self.recognizer is not None else {"name": "none"}
        return {
            "ok": True,
            "running": self.running,
            "cv2": HAS_CV2,
            "config": self.config.as_dict(),
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "frames": self.frames,
            "errors": self.errors,
            "reconnects": self.reconnects,
            "last_error": self.last_error,
            "uptime_s": round(time.time() - self.opened_at, 1) if self.opened_at else 0.0,
            "seq": self._seq,
            "recognizer": {
                **rec,
                "enabled": self.recognizer is not None,
                "detect_ms": round(self._last_detect_ms, 1),
                "error": self._overlay_error,
                "result": self._short_result(),
            },
        }

    def _short_result(self) -> Dict[str, Any]:
        """识别结果里给网页看的那一小块（关键点太多，不往 JSON 里塞）。"""
        res = self._last_result or {}
        hands = res.get("hands") or []
        out: Dict[str, Any] = {"hands": len(hands)}
        if hands:
            h = hands[0]
            out["bbox"] = h.get("bbox")
            out["center"] = h.get("center")
            out["score"] = round(float(h.get("score", 0.0)), 3)
            out["flex"] = {str(k): round(float(v), 3) for k, v in (h.get("flex") or {}).items()}
        elif res.get("score") is not None:
            out["score"] = res.get("score")
        cube = res.get("cube")
        if isinstance(cube, dict):
            out["cube"] = {
                "found": bool(cube.get("found")),
                "reliable": cube.get("reliable", True),
                "angle": cube.get("angle"),
                "cells": cube.get("cells"),
                "center": cube.get("center"),
                "cell_px": cube.get("cell"),
                "ms": cube.get("ms"),
                "source": cube.get("source"),
                "gap": cube.get("gap"),
                "good": cube.get("good"),
                "score": cube.get("score"),
            }
        return out


# ---------------------------------------------------------------- 设备

def probe_devices(indices: Tuple[int, ...] = (0, 1, 2, 3),
                  backends: Tuple[str, ...] = ("msmf",),
                  force: bool = False) -> List[Dict[str, Any]]:
    """逐个 index × 后端试一遍，看谁能出图。

    默认只试 Media Foundation：同一台设备刚被 MSMF 放掉、DirectShow 立刻去开，
    某些 UVC 设备（奥比这种）会直接在 C++ 里崩掉，整个进程都没。所以
    DirectShow 只在 MSMF 一路都出不了图时才补一轮。

    没有 cv2 时返回空列表（页面还能显示注册表里的物理设备）。
    结果缓存 60 秒：每试一次要开关设备，别让页面一刷新就把摄像头折腾一轮。
    """
    global _PROBE_CACHE
    if not HAS_CV2:
        return []
    now = time.time()
    if not force and _PROBE_CACHE and now - _PROBE_CACHE[0] < 60:
        return _PROBE_CACHE[1]
    out = _probe_pass(indices, backends)
    if not any(d["frame"] for d in out) and backends != ("dshow",):
        time.sleep(0.5)
        extra = _probe_pass(indices, ("dshow",))
        out += extra
        out.sort(key=lambda d: (not d["frame"], d["measured_fps"] < 15, -d["pixels"]))
    _PROBE_CACHE = (now, out)
    return out


def _probe_pass(indices: Tuple[int, ...], backends: Tuple[str, ...]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for backend in backends:
        for idx in indices:
            flag = BACKENDS.get(backend, cv2.CAP_ANY)
            cap = cv2.VideoCapture(idx, flag)
            if not cap.isOpened():
                cap.release()
                continue
            frame = None
            t0 = time.time()
            while time.time() - t0 < 1.2:
                ok, f = cap.read()
                if ok and f is not None and f.size:
                    frame = f
                    break
            measured = 0.0
            if frame is not None:
                # 实测帧率比 CAP_PROP_FPS 靠谱：后端经常报 0 或 -1。
                # 先空读几帧，避开 DSHOW 刚开始那几帧特别慢的预热期。
                for _ in range(4):
                    cap.read()
                n, t1 = 10, time.perf_counter()
                got = sum(1 for _ in range(n) if cap.read()[0])
                span = time.perf_counter() - t1
                measured = round(got / span, 1) if span > 0 else 0.0
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or (0 if frame is None else frame.shape[1])
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or (0 if frame is None else frame.shape[0])
            fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)
            cap.release()
            time.sleep(0.15)        # 让 MSMF 把设备放干净，不然下一路容易读空
            out.append({
                "backend": backend,
                "backend_label": BACKEND_LABELS.get(backend, backend),
                "index": idx,
                "width": w,
                "height": h,
                "fps": round(fps, 1),
                "measured_fps": measured,
                # 只认「拿到帧 + 还能继续读」的，深度口那种首帧后就没动静的不算
                "frame": frame is not None and measured > 0,
                "pixels": w * h,
            })
    # 能出图的排前面；再看帧率够不够 15，最后比分辨率
    out.sort(key=lambda d: (not d["frame"], d["measured_fps"] < 15, -d["pixels"]))
    return out


_PROBE_CACHE: Tuple[float, List[Dict[str, Any]]] = (0.0, [])


def list_devices() -> List[Dict[str, Any]]:
    """注册表里的摄像头（物理设备，跟 OpenCV 的 index 不是一回事）。

    用 cfgmgr32 判断「现在插着没」，因为注册表里的记录拔了也不会删。
    """
    if os.name != "nt":
        return []
    try:
        import winreg
    except ImportError:
        return []
    import ctypes

    cfg = ctypes.WinDLL("cfgmgr32", use_last_error=True)
    CM_LOCATE_DEVNODE_NORMAL = 0
    CR_SUCCESS = 0
    CM_PROB_PHANTOM = 45        # 拔掉的设备会留在注册表里，问题代码是 45

    def present(instance: str) -> bool:
        dev = ctypes.c_uint32(0)
        if cfg.CM_Locate_DevNodeW(ctypes.byref(dev), ctypes.c_wchar_p(instance),
                                  CM_LOCATE_DEVNODE_NORMAL) != CR_SUCCESS:
            return False
        status = ctypes.c_uint32(0)
        problem = ctypes.c_uint32(0)
        if cfg.CM_Get_DevNode_Status(ctypes.byref(status), ctypes.byref(problem), dev, 0) != CR_SUCCESS:
            return False
        return int(problem.value) != CM_PROB_PHANTOM

    root = r"SYSTEM\CurrentControlSet\Enum\USB"
    found: List[Dict[str, Any]] = []
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, root) as usb:
            i = 0
            while True:
                try:
                    vidpid = winreg.EnumKey(usb, i)
                except OSError:
                    break
                i += 1
                if not vidpid.startswith("VID_"):
                    continue
                try:
                    with winreg.OpenKey(usb, vidpid) as vk:
                        j = 0
                        while True:
                            try:
                                inst = winreg.EnumKey(vk, j)
                            except OSError:
                                break
                            j += 1
                            try:
                                with winreg.OpenKey(vk, inst) as ik:
                                    def get(name):
                                        try:
                                            return winreg.QueryValueEx(ik, name)[0]
                                        except OSError:
                                            return None

                                    service = get("Service")
                                    if service != "usbvideo":
                                        continue
                                    name = get("FriendlyName") or \
                                        str(get("DeviceDesc") or "").split(";")[-1] or vidpid
                                    full = f"USB\\{vidpid}\\{inst}"
                                    found.append({
                                        "name": name,
                                        "instance": full,
                                        "present": present(full),
                                        "vidpid": vidpid,
                                    })
                            except OSError:
                                continue
                except OSError:
                    continue
    except OSError:
        return []
    found.sort(key=lambda d: (not d["present"], d["name"]))
    return found


# ---------------------------------------------------------------- 单例

_CAMERA: Optional[Camera] = None


def get_camera() -> Camera:
    """服务里共用同一个 Camera；页面打开/关闭都落在它身上。"""
    global _CAMERA
    if _CAMERA is None:
        _CAMERA = Camera()
    return _CAMERA
