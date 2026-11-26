"""一个摄像头看懂整张桌子：整桌分割 + 物体成表 + 单目估高。

为什么单独一个模块
------------------
``scene.py`` 只管「像素 ↔ 桌面」的映射和物体表，``cubecv.py`` 只认魔方。真实
工位上不可能只有魔方：纸盒、水杯、胶带、工具、手…… 这些都要进安全模型。这个
模块补上「整桌」那一段：

    一帧画面 ──空桌背景差──► 桌面内的前景像素 ──连通块──► 一个个物体
             ──单应 H──► 基座坐标 ──► 位置 / 朝向 / 长宽 / 高度 / 类别

只用一个普通彩色摄像头，不需要深度相机、不需要模型文件。

高度只有单目，怎么来的
----------------------
桌面是平面 ``z = 0``，所以像素反投影到桌面是一个单应；但**物体顶端不在桌面上**，
直接反投影会把顶面投到桌面后方（这就是「高度视差」）。反过来用这个视差就能
估高。相机中心 ``C`` 离桌面 ``hc``，看一个竖直物体：同一条视线上的桌面点是

    X' = C + s·(X − C),     s = hc / (hc − h)

所以同一 (u, v) 列上，「底边像素」给真实底面 ``X_b``，「顶边像素」给虚点 ``X'``：

    s = |X'−C| / |X_b−C|      h = hc·(1 − 1/s)

``hc`` 不用卷尺量：桌面四角是 1.4×0.9 m 的**已知矩形**，从「世界→像素」单应
可以解出焦距 ``f`` 和相机位姿（假定主点在画面中心、方形像素），相机高度就是
位姿里的 Z。见 ``camera_pose()``。

单目的边界
----------
* 物体必须是**竖直柱体**、底面看得见：斜放的、悬空的、被挡住的，高度会偏；
* 只能看正面和顶面，背面/底面没有：俯视角度越小（相机越平），顶面越看不见，
  估高越差；
* 单应只在桌面平面成立：桌外的东西不反投影，一律当「未知」，由 ``scene`` 的
  保守策略兜底。

所以这里的高度是**下限估计**，碰撞盒按偏大取（``pad_m``、``MAX_H_M`` 截断），
宁可多拦一次。

怎么用
------
    scan = TableScan(scene)
    scan.set_background(空桌那一帧)     # 只做一次，桌面清空时拍
    scan.start()                        # 置 enabled
    scan.run(当前帧)                    # 后台线程每秒调一次
    scan.annotate(当前帧)               # 带中文标签的叠图（JPEG）

「空桌背景差」比「学习桌面颜色」稳：桌面是蓝的，纸盒/白纸/胶带颜色都接近，
靠颜色分不开；靠「和空桌那张图不一样」就能分开。没有背景时退化成桌面颜色
模型，会把手、阴影、反光也算进来，``state()["mode"]`` 会标出来。
"""
from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .camera import HAS_CV2, cv2

try:                                        # 常规路径
    import numpy as np  # type: ignore
except Exception:                           # noqa: BLE001
    from .cubecv import np  # type: ignore  # cubecv 已经找过 pylibs

_HERE = Path(__file__).resolve().parent.parent
_CFG_PATH = _HERE / "config" / "scan.json"
_BG_NAME = "scan_bg.png"

#: 小于这个面积（平方毫米）的连通块当噪声。桌面上 2 cm × 2 cm 的东西仍然能认出来。
MIN_AREA_MM2 = 420.0

#: 背景差在 Lab 空间的阈值下限。实测桌面噪声在 4～8，物体边缘在 25 以上。
DIFF_THR_MIN = 16.0

#: 没有背景时的「桌面颜色模型」阈值，比背景差宽一点（阴影/反光会进来）。
TABLE_THR = 26.0

#: 碰撞盒每边留的余量（米）。视觉尺寸只会偏小，不会偏大。
PAD_M = 0.006

#: 估计不出高度时的默认值（米）。宁可当个矮盒子，也别当没有。
DEFAULT_H_M = 0.08

#: 高度上限（米）。防止视差公式在遮挡/反光时解出离谱的值。
MAX_H_M = 0.45

#: 低于这个高度算「平铺物」（纸、布），碰撞盒给 4 mm 就够。
FLAT_H_M = 0.012

#: 俯视地图的分辨率（像素/米）。
MAP_PX_M = 300.0

_BEV_LABEL_COLORS = {
    "hand": (60, 60, 240),
    "paper": (230, 210, 140),
    "long": (90, 200, 250),
    "cup": (250, 170, 90),
    "box": (120, 220, 120),
    "other": (200, 160, 255),
}


# ================================================================ 单目几何
#
# 下面这些函数不依赖相机，给定矩阵就能算，方便单测。

def camera_pose(world_to_pixel: Sequence[Sequence[float]],
                width: int, height: int) -> Optional[Dict[str, Any]]:
    """从「桌面(米) → 像素」单应解相机位姿和离桌高度。

    模型：``H = K · [r1 r2 t]``，``K`` 主点在画面中心、方形像素、焦距 ``f`` 未知。
    两个约束（旋转列正交、等长）给两个 ``f²`` 估计，取中位数。解不出来（点病态、
    焦距离谱）返回 ``None``，调用方就把高度当未知处理。
    """
    if not HAS_CV2:
        return None
    try:
        Hm = np.asarray(world_to_pixel, dtype=float)
        if Hm.shape != (3, 3) or not np.all(np.isfinite(Hm)):
            return None
        h1, h2, h3 = Hm[:, 0], Hm[:, 1], Hm[:, 2]
        cx, cy = float(width) / 2.0, float(height) / 2.0
        a1x, a1y = h1[0] - cx * h1[2], h1[1] - cy * h1[2]
        a2x, a2y = h2[0] - cx * h2[2], h2[1] - cy * h2[2]

        cands: List[float] = []
        den = h1[2] * h2[2]
        if abs(den) > 1e-12:
            f2 = -(a1x * a2x + a1y * a2y) / den
            if f2 > 0.0:
                cands.append(float(f2))
        den = h2[2] ** 2 - h1[2] ** 2
        if abs(den) > 1e-12:
            f2 = ((a1x ** 2 + a1y ** 2) - (a2x ** 2 + a2y ** 2)) / den
            if f2 > 0.0:
                cands.append(float(f2))
        if not cands:
            return None
        f = math.sqrt(float(np.median(cands)))
        if not (0.25 * width < f < 8.0 * width):
            return None

        b1 = np.array([a1x / f, a1y / f, h1[2]])
        b2 = np.array([a2x / f, a2y / f, h2[2]])
        b3 = np.array([(h3[0] - cx * h3[2]) / f, (h3[1] - cy * h3[2]) / f, h3[2]])
        lam = 2.0 / (float(np.linalg.norm(b1)) + float(np.linalg.norm(b2)))
        r1, r2, t = lam * b1, lam * b2, lam * b3
        r3 = np.cross(r1, r2)
        R = np.stack([r1, r2, r3], axis=1)
        U, _S, Vt = np.linalg.svd(R)          # 再正交化一次，消掉噪声
        R = U @ Vt
        if np.linalg.det(R) < 0.0:
            U[:, -1] *= -1.0
            R = U @ Vt
        C = -R.T @ t
        if float(C[2]) < 0.0:                # 单应整体差一个符号：翻过来
            r1, r2, t = -r1, -r2, -t
            r3 = np.cross(r1, r2)
            R = np.stack([r1, r2, r3], axis=1)
            U, _S, Vt = np.linalg.svd(R)
            R = U @ Vt
            if np.linalg.det(R) < 0.0:
                U[:, -1] *= -1.0
                R = U @ Vt
            C = -R.T @ t
        hc = float(C[2])
        if not (0.15 <= hc <= 3.5):
            return None
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
        return {"f_px": round(f, 2), "center": [round(float(C[0]), 4),
                                                round(float(C[1]), 4),
                                                round(hc, 4)],
                "height_m": round(hc, 4), "tilt_deg": round(tilt, 1),
                "rotation": [[float(v) for v in row] for row in R],
                "translation": [float(v) for v in t]}
    except Exception:                       # noqa: BLE001
        return None


def height_from_parallax(base_xy: Sequence[float], top_xy: Sequence[float],
                         camera_xy: Sequence[float], hc: float) -> Optional[float]:
    """两点（底面点、顶端像素反投影到桌面的虚点）→ 高度（米）。

    ``base_xy`` 是物体和桌面接触那个点；``top_xy`` 是顶边像素按 ``z=0`` 反投影
    出来的点（一定比 ``base_xy`` 离相机更远）。两者和相机中心的距离比就是
    ``s = hc/(hc−h)``。
    """
    db = math.hypot(base_xy[0] - camera_xy[0], base_xy[1] - camera_xy[1])
    dt = math.hypot(top_xy[0] - camera_xy[0], top_xy[1] - camera_xy[1])
    if db < 1e-6 or dt <= db + 1e-9:
        return None
    s = dt / db
    h = hc * (1.0 - 1.0 / s)
    return h if math.isfinite(h) else None


# ================================================================ 分类

def classify(feat: Dict[str, Any]) -> Tuple[str, str]:
    """按「长宽高 + 颜色 + 形状」给一个粗糙类别。返回 ``(标签, 形状)``。

    只分到工位上够用的粒度：手、纸/薄片、细长物（笔/线缆/工具）、杯/筒、盒子、
    其它。分类错了不影响安全（尺寸是量出来的），只影响显示和命名；页面上可以改名。
    """
    L = max(float(feat.get("l_m") or 0.0), 1e-3)
    D = max(float(feat.get("d_m") or 0.0), 1e-3)
    h = float(feat.get("h_m") or 0.0)
    skin = float(feat.get("skin") or 0.0)
    val = float(feat.get("val") or 0.0)
    area_mm2 = float(feat.get("area_mm2") or 0.0)
    long_side, short_side = max(L, D), min(L, D)
    elong = long_side / max(short_side, 1e-3)

    if skin >= 0.42 and area_mm2 >= 3000.0:
        return "手", "box"
    if h <= FLAT_H_M and long_side >= 0.05:
        return "纸/薄片", "box"
    if elong >= 3.0 or short_side <= 0.018:
        return ("线缆" if val < 85.0 else "细长物"), "box"
    if h >= 1.15 * long_side and float(feat.get("fill") or 0.0) >= 0.35:
        return "杯/筒", "cylinder"
    if 0.45 <= short_side / long_side <= 1.7 and h >= 0.014:
        return "盒子", "box"
    return "物块", "box"


def _kind_key(label: str) -> str:
    return {"手": "hand", "纸/薄片": "paper", "细长物": "long", "线缆": "long",
            "杯/筒": "cup", "盒子": "box"}.get(label, "other")


# ================================================================ 整桌扫描

class TableScan:
    """整桌扫描器：背景、分割、跟踪、把结果写进 ``Scene``。

    线程安全：``run()`` 由 ``VisionHub`` 的线程调，``state()`` / ``annotate()``
    由 HTTP 线程调，共用一把 ``RLock``。
    """

    def __init__(self, scene, path: Optional[Path] = None,
                 bg_path: Optional[Path] = None) -> None:
        self.scene = scene
        self.cfg_path = Path(path) if path else _CFG_PATH
        self.bg_path = Path(bg_path) if bg_path else self.cfg_path.parent / _BG_NAME
        self.enabled = False
        self.captured = 0.0
        self.bg = None                       # BGR ndarray
        self._lock = threading.RLock()
        self._geom_sig: Any = None
        self._table_mask = None
        self._tracks: List[Dict[str, Any]] = []
        self._counters: Dict[str, int] = {}
        self.frames = 0
        self.errors = 0
        self.last_error = ""
        self.last_ms = 0.0
        self.last: Dict[str, Any] = {}
        self.last_frame: Any = None          # 最近一帧，标注图用
        self._load()

    # -- 配置 -------------------------------------------------------------
    def _load(self) -> None:
        try:
            data = json.loads(self.cfg_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            data = {}
        self.enabled = bool(data.get("enabled", False))
        self.captured = float(data.get("captured") or 0.0)
        if not self.enabled:                 # 服务重启后别自动开始扫
            self.enabled = False
        if HAS_CV2 and self.bg_path.exists():
            img = cv2.imread(str(self.bg_path))
            if img is not None:
                self.bg = img

    def _save(self) -> None:
        self.cfg_path.parent.mkdir(parents=True, exist_ok=True)
        data = {"enabled": bool(self.enabled), "captured": float(self.captured),
                "bg": self.bg_path.name if self.bg is not None else None}
        self.cfg_path.write_text(json.dumps(data, ensure_ascii=False, indent=2),
                                 encoding="utf-8")

    # -- 开关 / 背景 -------------------------------------------------------
    def start(self) -> Dict[str, Any]:
        with self._lock:
            self.enabled = True
            self._save()
        return self.state()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            self.enabled = False
            self._save()
        return self.state()

    def has_background(self) -> bool:
        return self.bg is not None

    def set_background(self, frame) -> Dict[str, Any]:
        """把当前帧当空桌背景。桌面清空时拍一次，之后物体出现就会被认出来。"""
        if not HAS_CV2 or frame is None:
            raise RuntimeError("没有画面（先打开摄像头）")
        with self._lock:
            self.bg = frame.copy()
            self.captured = time.time()
            self.bg_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(self.bg_path), self.bg)
            self._save()
            self._tracks.clear()
            self._counters.clear()
        return self.state()

    def clear_background(self) -> Dict[str, Any]:
        with self._lock:
            self.bg = None
            self.captured = 0.0
            try:
                self.bg_path.unlink()
            except OSError:
                pass
            self.enabled = False
            self._save()
        return self.state()

    def state(self) -> Dict[str, Any]:
        now = time.time()
        age = (now - self.captured) if self.captured else None
        pose = None
        if self.scene.calibrated and self.last_frame is not None:
            pose = camera_pose(self.scene.Hinv, self.last_frame.shape[1],
                               self.last_frame.shape[0])
        return {
            "ok": True,
            "available": HAS_CV2,
            "enabled": bool(self.enabled),
            "has_background": self.bg is not None,
            "background_age": None if age is None else round(age, 1),
            "mode": ("background" if self.bg is not None else "tablecolor"),
            "frames": self.frames, "errors": self.errors,
            "last_error": self.last_error, "last_ms": round(self.last_ms, 1),
            "objects": list(self.last.get("objects") or []),
            "count": int(self.last.get("count") or 0),
            "camera": pose,
            "thr": self.last.get("thr"),
            "updated": self.last.get("t"),
        }

    # -- 几何 -------------------------------------------------------------
    def _ensure_geometry(self, shape: Tuple[int, ...]) -> bool:
        """标定/画面尺寸变了就重算桌面多边形掩膜。"""
        h, w = int(shape[0]), int(shape[1])
        poly = self.scene.table_polygon()
        sig = (w, h, self.scene.calib.get("updated"), tuple(poly))
        if sig == self._geom_sig and self._table_mask is not None:
            return True
        if len(poly) != 4:
            return False
        arr = np.array([[float(u), float(v)] for u, v in poly], dtype=np.float32)
        if abs(cv2.contourArea(arr)) < 400.0:
            return False
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [arr.astype(np.int32)], 255)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8))
        if mask.sum() == 0:
            return False
        self._table_mask = mask
        self._geom_sig = sig
        return True

    def _world(self, xs, ys) -> "np.ndarray":
        H = np.asarray(self.scene.H, dtype=float)
        p = H @ np.stack([np.asarray(xs, dtype=float), np.asarray(ys, dtype=float),
                          np.ones(len(xs))])
        w = p[2]
        w[np.abs(w) < 1e-12] = 1e-12
        return (p[:2] / w).T

    # -- 主流程 -----------------------------------------------------------
    def run(self, frame) -> Dict[str, Any]:
        """跑一帧：分割 → 连通块 → 估尺寸 → 写进 ``Scene``。"""
        t0 = time.time()
        if not HAS_CV2:
            return {"ok": False, "error": "没有 opencv/numpy"}
        if frame is None:
            return {"ok": False, "error": "没有画面（先打开摄像头）"}
        with self._lock:
            self.last_frame = frame.copy()
        if not self.scene.calibrated:
            return {"ok": False, "error": "桌面还没标定（先 /scene/calibrate）"}
        if not self._ensure_geometry(frame.shape):
            return {"ok": False, "error": "桌面多边形太小/不在画面里，重新标定"}

        try:
            mask, thr = self._foreground(frame)
            dets = self._components(frame, mask)
            objs = self._objects(dets)
            now = time.time()
            with self._lock:
                self.last = {"objects": objs, "count": len(objs), "thr": round(thr, 1),
                             "t": time.strftime("%H:%M:%S"), "mode":
                             "background" if self.bg is not None else "tablecolor"}
                self.frames += 1
                self.last_ms = (time.time() - t0) * 1000.0
            self.scene.update_scan(objs)
            return {"ok": True, "count": len(objs), "objects": objs,
                    "thr": round(thr, 1), "ms": round(self.last_ms, 1),
                    "mode": "background" if self.bg is not None else "tablecolor"}
        except Exception as exc:              # noqa: BLE001
            with self._lock:
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
            return {"ok": False, "error": self.last_error}

    def _foreground(self, frame) -> Tuple["np.ndarray", float]:
        """桌面内的前景掩膜（uint8）+ 用到的阈值。"""
        h, w = frame.shape[:2]
        cur = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
        tbl = self._table_mask
        if self.bg is not None:
            bg = self.bg
            if bg.shape[:2] != (h, w):
                bg = cv2.resize(bg, (w, h), interpolation=cv2.INTER_AREA)
            bg_lab = cv2.cvtColor(bg, cv2.COLOR_BGR2LAB).astype(np.float32)
            # 曝光/自动白平衡漂移：用桌面区域的中位差逐通道校正，只校小漂移。
            step = 4
            sub = np.zeros((h, w), dtype=bool)
            sub[::step, ::step] = True
            sel = sub & (tbl > 0)
            if sel.any():
                for c in range(3):
                    dv = float(np.median(cur[:, :, c][sel] - bg_lab[:, :, c][sel]))
                    if abs(dv) < 14.0:
                        cur[:, :, c] -= dv
            d = np.sqrt(((cur - bg_lab) ** 2).sum(axis=2))
            vals = d[tbl > 0]
            med = float(np.median(vals)) if vals.size else 0.0
            mad = float(np.median(np.abs(vals - med))) if vals.size else 0.0
            thr = max(DIFF_THR_MIN, med + 6.0 * 1.4826 * mad)
        else:
            # 没有空桌背景：用桌面自己的颜色当模型（蓝桌子比较均匀）。
            vals_lab = cur[tbl > 0]
            med = np.median(vals_lab, axis=0) if vals_lab.size else np.zeros(3)
            d = np.sqrt(((cur - med.reshape(1, 1, 3)) ** 2).sum(axis=2))
            thr = TABLE_THR

        m = (d > thr).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        m = cv2.bitwise_and(m, tbl)
        return m, thr

    def _components(self, frame, mask) -> List[Dict[str, Any]]:
        """连通块 → 每个块的像素统计（还没定尺寸/类别）。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ycc = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        n, labels, stats, _cent = cv2.connectedComponentsWithStats(mask, 8)
        h, w = mask.shape[:2]
        # 每像素对应的桌面面积（mm²）：用单应局部尺度算，边缘会大一点。
        mm2_per_px = self._mm2_per_px()
        out: List[Dict[str, Any]] = []
        for i in range(1, n):
            x, y, bw, bh, area = (int(stats[i, 0]), int(stats[i, 1]), int(stats[i, 2]),
                                  int(stats[i, 3]), int(stats[i, 4]))
            if area < 25 or bw < 3 or bh < 3:
                continue
            area_mm2 = area * mm2_per_px
            if area_mm2 < MIN_AREA_MM2:
                continue
            sub = labels[y:y + bh, x:x + bw] == i
            ys, xs = np.nonzero(sub)
            ys = ys + y
            xs = xs + x
            if len(xs) < 12:
                continue
            px_area = float(len(xs))
            # 颜色/皮肤色比例
            hh = hsv[ys, xs].astype(np.float32)
            yy = ycc[ys, xs]
            cr, cb = yy[:, 1].astype(np.float32), yy[:, 2].astype(np.float32)
            skin = ((cr >= 133) & (cr <= 178) & (cb >= 75) & (cb <= 130)
                    & (hh[:, 2] >= 60)).mean()
            out.append({"x0": x, "y0": y, "bw": bw, "bh": bh, "area_px": px_area,
                        "xs": xs, "ys": ys, "area_mm2": area_mm2,
                        "hue": float(np.median(hh[:, 0])), "sat": float(np.median(hh[:, 1])),
                        "val": float(np.median(hh[:, 2])), "skin": float(skin)})
        return out

    def _mm2_per_px(self) -> float:
        """桌面平面的平均像素面积（mm²/px）。拿单应的行列式开平方估。"""
        try:
            H = np.asarray(self.scene.H, dtype=float)
            # H 的雅可比：把像素小方块映射成平行四边形，用行列式的平方根当尺度。
            j = np.linalg.det(H[:2, :2] - np.outer(H[:2, 2], H[2, :2])) * 1e6
            if not math.isfinite(float(j)) or j <= 0:
                raise ValueError
            return float(j)
        except Exception:                    # noqa: BLE001
            return 4.0

    def _objects(self, dets: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """每个连通块 → 场景物体（世界坐标 + 尺寸 + 类别 + 稳定名字）。"""
        pose = None
        if self.scene.calibrated:
            pose = camera_pose(self.scene.Hinv, self.last_frame.shape[1],
                               self.last_frame.shape[0])
        C = pose["center"][:2] if pose else None
        hc = pose["height_m"] if pose else None
        objs: List[Dict[str, Any]] = []
        used_tracks: List[Dict[str, Any]] = []
        for det in dets:
            xs, ys = det["xs"], det["ys"]
            P = self._world(xs, ys)                      # (N,2) 世界
            if P is None or len(P) < 12 or not np.all(np.isfinite(P)):
                continue
            ymin, ymax = int(ys.min()), int(ys.max())
            xmin, xmax = int(xs.min()), int(xs.max())
            band = ys >= ymax - max(2, int(0.18 * (ymax - ymin)))
            if not band.any():
                continue
            # 底面朝向：底边带在主方向上的投影（PCA）
            B = P[band]
            c0 = B.mean(axis=0)
            cov = np.cov(B.T) if len(B) > 2 else np.eye(2)
            vals, vecs = np.linalg.eigh(cov)
            e = vecs[:, int(np.argmax(vals))]
            e = e / (np.linalg.norm(e) + 1e-12)
            t = B @ e
            L = float(t.max() - t.min())
            if L < 0.012:
                L = 0.012
            # 高度：同一 (u,v) 列上底边像素 vs 顶边像素
            u0 = int(round(float(np.median(xs[band]))))
            span = max(3, int(0.10 * (xmax - xmin)))
            col = np.abs(xs - u0) <= span
            h_est = None
            if col.any():
                vb, vt = int(ys[col].max()), int(ys[col].min())
                base = self.scene.unproject(float(u0), float(vb))
                top = self.scene.unproject(float(u0), float(vt))
                if base is not None and top is not None and hc:
                    h_est = height_from_parallax(base, top, C, hc)
            if h_est is None or not math.isfinite(h_est) or h_est < 0.0:
                h = DEFAULT_H_M
                height_ok = False
            else:
                h = min(MAX_H_M, max(0.0, h_est))
                height_ok = True
            # 深度：远端是顶面投到桌面的虚点，除以 s 才是真正的后边
            D = L
            if C is not None and hc and h < hc:
                r = np.linalg.norm(P - np.asarray(C, dtype=float), axis=1)
                d0, d1 = float(r.min()), float(r.max())
                s = hc / max(1e-6, hc - h)
                D = d1 / s - d0
                D = min(0.6, max(0.012, D))
            n = np.array([-e[1], e[0]])
            if C is not None:
                if float(n @ (c0 - np.asarray(C, dtype=float))) < 0:
                    n = -n
            center = c0 + n * (D / 2.0)
            yaw = math.degrees(math.atan2(float(e[1]), float(e[0]))) % 180.0
            fill = float(det["area_px"]) / max(1.0, float(det["bw"] * det["bh"]))
            L_pad = L + 2 * PAD_M
            D_pad = D + 2 * PAD_M
            feat = {"l_m": L, "d_m": D, "h_m": h, "area_mm2": det["area_mm2"],
                    "skin": det["skin"], "val": det["val"], "fill": fill}
            label, shape = classify(feat)
            name = self._track_name(label, center, (L_pad, D_pad))
            objs.append({
                "name": name, "kind": shape,
                "center": [round(float(center[0]), 4), round(float(center[1]), 4)],
                "size": [round(float(L_pad), 4), round(float(D_pad), 4),
                         round(float(max(h, 0.004)), 4)],
                "yaw": round(float(yaw), 1),
                "source": "vision", "ttl": 3.0,
                "meta": {"origin": "scan", "label": label,
                         "px": [int((det["x0"] + det["bw"] / 2)),
                                int((det["y0"] + det["bh"] / 2))],
                         "area_mm2": round(float(det["area_mm2"])),
                         "height_ok": bool(height_ok), "fill": round(fill, 2),
                         "hue": round(det["hue"], 1), "val": round(det["val"], 1)},
            })
            used_tracks.append({"name": name, "center": [float(center[0]),
                                                         float(center[1])],
                                "label": label})
        with self._lock:
            self._tracks = used_tracks
        return objs

    def _track_name(self, label: str, center: "np.ndarray",
                    size: Sequence[float]) -> str:
        """跨帧保持名字：离上次同名物体 9 cm 以内就沿用。"""
        best = None
        best_d = 0.09
        for tr in self._tracks:
            d = math.hypot(float(center[0]) - tr["center"][0],
                           float(center[1]) - tr["center"][1])
            if d < best_d:
                best_d, best = d, tr
        if best is not None:
            return str(best["name"])
        with self._lock:
            self._counters[label] = self._counters.get(label, 0) + 1
            k = self._counters[label]
        return f"{label} {k}"

    # -- 画图 -------------------------------------------------------------
    def annotate(self, frame) -> Optional[bytes]:
        """当前帧 + 桌面框 + 物体轮廓 + 中文标签，返回 JPEG 字节。"""
        if not HAS_CV2 or frame is None:
            return None
        img = frame.copy()
        poly = self.scene.table_polygon()
        if len(poly) == 4:
            cv2.polylines(img, [np.array(poly, dtype=np.int32)], True,
                          (120, 200, 120), 1, cv2.LINE_AA)
        items: List[Tuple[Tuple[int, int], str, str]] = []
        for o in (self.last.get("objects") or []):
            pts = self._box_pixels(o)
            if len(pts) != 4:
                continue
            key = _kind_key(str(o.get("meta", {}).get("label") or ""))
            color = _BEV_LABEL_COLORS.get(key, _BEV_LABEL_COLORS["other"])
            cv2.polylines(img, [np.array(pts, dtype=np.int32)], True, color, 2,
                          cv2.LINE_AA)
            h_mm = round(float(o["size"][2]) * 1000)
            w_mm = round(float(o["size"][0]) * 1000)
            d_mm = round(float(o["size"][1]) * 1000)
            text = f"{o['name']}  {w_mm}×{d_mm}×{h_mm} mm"
            u, v = pts[0]
            items.append(((int(u), max(18, int(v)) - 6), text, key))
        return _encode_jpeg(_draw_labels(img, items))

    def map_image(self, frame) -> Optional[bytes]:
        """俯视地图：桌面按毫米摆正，物体画成带标签的框。"""
        if not HAS_CV2 or frame is None or not self.scene.calibrated:
            return None
        try:
            from .scene import TABLE_SIZE
            tw, th = float(TABLE_SIZE[0]), float(TABLE_SIZE[1])
            bw = int(round(tw * MAP_PX_M))
            bh = int(round(th * MAP_PX_M))
            A = np.array([[MAP_PX_M, 0.0, MAP_PX_M * tw / 2.0],
                          [0.0, -MAP_PX_M, MAP_PX_M * th / 2.0],
                          [0.0, 0.0, 1.0]], dtype=float)
            M = A @ np.asarray(self.scene.H, dtype=float)
            bev = cv2.warpPerspective(frame, M, (bw, bh), flags=cv2.INTER_AREA)
            bev = cv2.addWeighted(bev, 0.85, np.full_like(bev, 20), 0.15, 0)
            for x in np.arange(-0.7, 0.71, 0.1):
                u = int(round(x * MAP_PX_M + bw / 2))
                cv2.line(bev, (u, 0), (u, bh), (70, 70, 70), 1)
            for y in np.arange(-0.45, 0.46, 0.1):
                v = int(round(bh / 2 - y * MAP_PX_M))
                cv2.line(bev, (0, v), (bw, v), (70, 70, 70), 1)
            cv2.rectangle(bev, (0, 0), (bw - 1, bh - 1), (110, 180, 110), 2)
            items = []
            for o in (self.last.get("objects") or []):
                pts = []
                for x, y in self._box_corners(o):
                    pts.append((x * MAP_PX_M + bw / 2, bh / 2 - y * MAP_PX_M))
                arr = np.array(pts, dtype=np.int32)
                key = _kind_key(str(o.get("meta", {}).get("label") or ""))
                color = _BEV_LABEL_COLORS.get(key, _BEV_LABEL_COLORS["other"])
                cv2.polylines(bev, [arr], True, color, 2, cv2.LINE_AA)
                items.append(((int(arr[0][0]), max(16, int(arr[0][1]) - 4)),
                              f"{o['name']} h{round(float(o['size'][2]) * 1000)}",
                              key))
            return _encode_jpeg(_draw_labels(bev, items, font_size=15))
        except Exception:                    # noqa: BLE001
            return None

    def _box_corners(self, o: Dict[str, Any]) -> List[Tuple[float, float]]:
        w, d = float(o["size"][0]), float(o["size"][1])
        yaw = math.radians(float(o.get("yaw") or 0.0))
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for dx, dy in ((-w / 2, -d / 2), (w / 2, -d / 2), (w / 2, d / 2),
                       (-w / 2, d / 2)):
            out.append((o["center"][0] + dx * c - dy * s,
                        o["center"][1] + dx * s + dy * c))
        return out

    def _box_pixels(self, o: Dict[str, Any]) -> List[Tuple[int, int]]:
        out = []
        for x, y in self._box_corners(o):
            p = self.scene.project(x, y)
            if p is None:
                return []
            out.append((int(round(p[0])), int(round(p[1]))))
        return out


# ================================================================ 画图工具

_FONT_CACHE: Dict[int, Any] = {}


def _font(size: int):
    """中文字体：Windows 自带微软雅黑，找不到就用 PIL 默认（显示方块也算能用）。"""
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    from PIL import ImageFont
    for name in ("msyh.ttc", "msyhbd.ttc", "simhei.ttf", "simsun.ttc"):
        try:
            f = ImageFont.truetype(str(Path("C:/Windows/Fonts") / name), size)
            _FONT_CACHE[size] = f
            return f
        except Exception:                    # noqa: BLE001
            continue
    try:
        f = ImageFont.load_default()
    except Exception:                        # noqa: BLE001
        f = None
    _FONT_CACHE[size] = f
    return f


def _draw_labels(img, items: Sequence[Tuple[Tuple[int, int], str, str]],
                 font_size: int = 16):
    """在 BGR 图上画带底色的小标签。``items`` = [(左上角, 文字, 颜色键)]。"""
    try:
        from PIL import Image, ImageDraw
        pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil)
        font = _font(font_size)
        for (x, y), text, key in items:
            color = _BEV_LABEL_COLORS.get(key, _BEV_LABEL_COLORS["other"])
            try:
                box = draw.textbbox((x, y), text, font=font)
            except Exception:                # noqa: BLE001
                box = (x, y, x + 8 * len(text), y + font_size)
            pad = 2
            draw.rectangle([box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad],
                           fill=(12, 18, 30))
            draw.text((x, y), text, font=font, fill=(color[2], color[1], color[0]))
        return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
    except Exception:                        # noqa: BLE001
        return img


def _encode_jpeg(img, quality: int = 84) -> Optional[bytes]:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return buf.tobytes() if ok else None
