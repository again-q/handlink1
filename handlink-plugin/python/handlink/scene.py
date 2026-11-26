"""把摄像头的实时环境接进**仿真与安全模型**。

解决的问题
----------
``/arm3d`` 里的工位是写死的（``cell.py`` 的 ``CELL``）：桌子、料盒、人都在固定
位置，相机拍到的真实物体一个都进不来。这个模块补上中间那一层：

    相机像素 ──单应 H──► 臂基座坐标 (x, y, z=0) ──► 场景物体 ──► cell / 三维视图

坐标一律用 **URDF 基座系**（Z 向上、原点在臂底座安装面、单位米），和
``cell.py``、``kin.py`` 保持同一套。

为什么先做「平面单应」而不是完整手眼标定
----------------------------------------
臂只在桌面上方工作，桌子就是一个平面 ``z = 0``。相机固定（eye-to-hand）时，
桌面上的点从像素到基座的映射是一个 3×3 单应矩阵，**四对点就能解**，不需要
内参、不需要 `solvePnP`。代价是只对桌面平面成立：

* 桌面上的魔方 / 料盒 / 水杯 —— 准（本机约 2～4 mm/px，见 §精度）；
* 站着的人、桌下的地面 —— 不准，要等第二阶段补内参 + 外参（见文档）。

接口分三层
----------
* ``solve_homography()`` / ``apply_h()``：纯数学，不依赖相机；
* ``Scene``：标定、反投影、物体表（带 TTL）、把物体合并进 ``CELL``；
* ``VisionHub``：后台线程，按 ``hz`` 从相机取帧、跑魔方视觉、更新场景。

「最后已知」与失败安全
----------------------
视觉丢帧时**不立刻删物体**：保留 ``ttl`` 秒（默认 3 s），``cell()`` 仍然把它
当障碍物——宁可多拦一次，也不要因为一帧没认出来就放行。超过 TTL 才移除，
并在 ``summary()`` 里报 ``vision_age``，让上层决定是否降级。
"""
from __future__ import annotations

import copy
import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import cell as cell_mod
from . import boxcv

# numpy 可能装在 .tmp/pylibs（camera.load_cv2 会处理），所以不能假定 import 成功。
np = None
try:                                        # 常规路径
    import numpy as _np  # type: ignore

    np = _np
except Exception:                           # noqa: BLE001
    try:
        from .cubecv import np as _np       # type: ignore  # cubecv 已经找过 pylibs

        np = _np
    except Exception:                       # noqa: BLE001
        np = None

HAS_NUMPY = np is not None

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]

_SETTINGS_PATH = Path(__file__).resolve().parent.parent / "config" / "scene.json"

#: 桌面尺寸（和 ``cell.TABLE_SIZE`` 同源）。标定「点四个桌角」时用它换算世界坐标。
TABLE_SIZE = cell_mod.TABLE_SIZE[:2]

#: 标准竞速魔方边长（米）。视觉不给物理尺寸，只给像素中心，所以这里用量出来的
#: 实物尺寸，而不是拿 ``cell`` 像素反推（那样会把标定误差算进物体尺寸）。
CUBE_M = 0.057

#: 视觉物体在「最后一帧之后」还保留多久（秒）。丢帧不等于物体消失。
CUBE_TTL = 3.0

#: 超过这么久没有新的视觉结果就判「视觉失效」。比 TTL 短：物体还没过期时
#: 就要先报失效，否则 TTL 到期到判失效之间会有一段「桌子看起来是空的」的窗口。
VISION_STALE_S = 2.0

MIN_CALIB_POINTS = 4


# ================================================================ 单应矩阵
#
# 像素 (u, v) → 基座 (x, y)：  [x', y', w]ᵀ = H · [u, v, 1]ᵀ，x = x'/w, y = y'/w。
# H 同时吸收了相机的内参、外参和「图像 y 轴朝下」的翻转，所以不要再单独翻 v。

def _points(points: Sequence[Sequence[float]], dim: int, name: str) -> List[List[float]]:
    out: List[List[float]] = []
    for i, p in enumerate(points):
        if len(p) != dim:
            raise ValueError(f"{name}[{i}] 需要 {dim} 个数，收到 {list(p)}")
        try:
            vals = [float(v) for v in p]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name}[{i}] 不是数字: {p}") from exc
        if not all(math.isfinite(v) for v in vals):
            raise ValueError(f"{name}[{i}] 含 nan/inf: {p}")
        out.append(vals)
    return out


def _normalizer(pts: "np.ndarray") -> "np.ndarray":
    """Hartley 归一化：平移到质心、缩放到平均距离 √2。四点解 H 不做这步会很病态。"""
    c = pts.mean(axis=0)
    d = float(np.sqrt(((pts - c) ** 2).sum(axis=1)).mean())
    s = math.sqrt(2.0) / d if d > 1e-12 else 1.0
    return np.array([[s, 0.0, -s * c[0]],
                     [0.0, s, -s * c[1]],
                     [0.0, 0.0, 1.0]], dtype=float)


def solve_homography(pixels: Sequence[Sequence[float]],
                     world: Sequence[Sequence[float]]) -> List[List[float]]:
    """四对（或更多）点解单应：像素 → 桌面基座坐标。

    点数 > 4 时是最小二乘（照相机装歪、点得不准时更稳）。返回 3×3 的 list，
    方便直接塞进 JSON / 存盘。
    """
    if not HAS_NUMPY:
        raise RuntimeError("没有 numpy，算不了单应矩阵（先装 numpy 或解压到 .tmp/pylibs）")
    src = np.asarray(_points(pixels, 2, "pixels"), dtype=float)
    dst = np.asarray(_points(world, 2, "world"), dtype=float)
    if len(src) != len(dst):
        raise ValueError(f"像素点和世界点数量不一致：{len(src)} vs {len(dst)}")
    n = len(src)
    if n < MIN_CALIB_POINTS:
        raise ValueError(f"至少要 {MIN_CALIB_POINTS} 对点，收到 {n} 对")
    if _collinear(src) or _collinear(dst):
        raise ValueError("点几乎共线，解不出单应（换几个不在一条线上的点）")

    Ts = _normalizer(src)
    Td = _normalizer(dst)
    sn = np.hstack([src, np.ones((n, 1))]) @ Ts.T
    dn = np.hstack([dst, np.ones((n, 1))]) @ Td.T

    A = np.zeros((2 * n, 9), dtype=float)
    for i in range(n):
        x, y = sn[i, 0], sn[i, 1]
        u, v = dn[i, 0], dn[i, 1]
        A[2 * i] = [x, y, 1.0, 0.0, 0.0, 0.0, -u * x, -u * y, -u]
        A[2 * i + 1] = [0.0, 0.0, 0.0, x, y, 1.0, -v * x, -v * y, -v]
    _u, _s, vt = np.linalg.svd(A)
    Hn = vt[-1].reshape(3, 3)
    H = np.linalg.inv(Td) @ Hn @ Ts
    if abs(H[2, 2]) > 1e-12:
        H = H / H[2, 2]
    if not np.all(np.isfinite(H)):
        raise ValueError("单应矩阵退化（点太少或几乎重合）")
    return [[float(v) for v in row] for row in H]


def invert_h(H: Sequence[Sequence[float]]) -> List[List[float]]:
    if not HAS_NUMPY:
        raise RuntimeError("没有 numpy，算不了单应矩阵")
    return [[float(v) for v in row] for row in np.linalg.inv(np.asarray(H, dtype=float))]


def apply_h(H: Sequence[Sequence[float]], u: float, v: float) -> Vec2:
    """把一个像素点映射过去（H 自己决定方向）。"""
    w = H[2][0] * u + H[2][1] * v + H[2][2]
    if abs(w) < 1e-12:
        raise ValueError("点在单应的无穷远线上，投不出来")
    return ((H[0][0] * u + H[0][1] * v + H[0][2]) / w,
            (H[1][0] * u + H[1][1] * v + H[1][2]) / w)


def _collinear(pts: "np.ndarray", eps: float = 1e-6) -> bool:
    """任意三点都近似共线时判死（H 的病态来源）。"""
    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                ax, ay = pts[j] - pts[i]
                bx, by = pts[k] - pts[i]
                area = abs(ax * by - ay * bx)
                if area > eps:
                    return False
    return True


def reprojection(H: Sequence[Sequence[float]],
                 pixels: Sequence[Sequence[float]],
                 world: Sequence[Sequence[float]]) -> Dict[str, float]:
    """标定残差：把像素映射回世界，和给的毫米坐标比。"""
    errs: List[float] = []
    for (u, v), (x, y) in zip(pixels, world):
        px, py = apply_h(H, float(u), float(v))
        errs.append(math.hypot(px - float(x), py - float(y)))
    n = max(1, len(errs))
    rms = math.sqrt(sum(e * e for e in errs) / n)
    return {"rms_m": rms, "max_m": max(errs) if errs else 0.0,
            "rms_mm": rms * 1000.0, "max_mm": (max(errs) if errs else 0.0) * 1000.0}


def mm_per_px(pixels: Sequence[Sequence[float]],
              world: Sequence[Sequence[float]]) -> float:
    """标尺换算：相邻两对点的世界距离 / 像素距离，取中位数。

    只在标定点附近可靠——离得越远、镜头畸变越大（见 docs 的精度一节）。
    """
    ratios: List[float] = []
    n = len(pixels)
    for i in range(n):
        j = (i + 1) % n
        du = float(pixels[i][0]) - float(pixels[j][0])
        dv = float(pixels[i][1]) - float(pixels[j][1])
        dx = float(world[i][0]) - float(world[j][0])
        dy = float(world[i][1]) - float(world[j][1])
        dpx = math.hypot(du, dv)
        if dpx > 1e-6:
            ratios.append(math.hypot(dx, dy) * 1000.0 / dpx)
    ratios.sort()
    return ratios[len(ratios) // 2] if ratios else 0.0


def preset_world(rotate: int = 0, mirror: bool = False,
                 size: Sequence[float] = TABLE_SIZE) -> List[Vec2]:
    """「点四个桌角」用的世界坐标预设。

    约定：按**画面里**左上 → 右上 → 右下 → 左下 的顺序点。默认假设相机从
    正上方看、画面 x 约等于基座 x、画面上方是基座 +y，于是：

        画面左上 = (-w/2, +h/2)   画面右上 = (+w/2, +h/2)
        画面右下 = (+w/2, -h/2)   画面左下 = (-w/2, -h/2)

    ``rotate`` 把世界坐标整体转 90° 的整数倍，``mirror`` 再左右翻一次——相机
    装反、装转了先试这 8 种组合，不用重装。真机最好还是用 ``world`` 显式给点。
    """
    hx, hy = float(size[0]) / 2.0, float(size[1]) / 2.0
    pts: List[Vec2] = [(-hx, hy), (hx, hy), (hx, -hy), (-hx, -hy)]
    k = (int(rotate) // 90) % 4
    pts = pts[k:] + pts[:k]
    if mirror:
        pts = [(-x, y) for x, y in pts]
    return pts


# ================================================================ 场景物体

@dataclass
class SceneObject:
    """桌面上的一个物体。坐标是基座系，``center`` 的 z 是**底面高度**（桌面 = 0）。"""

    name: str
    kind: str                       # box / cylinder
    center: Vec2                    # 底面中心 (x, y)
    size: Tuple[float, ...]         # box: (w, d, h)；cylinder: (r, h)
    yaw: float = 0.0                # 绕 Z 的朝向（度），只影响显示与碰撞膨胀
    source: str = "manual"          # manual / vision
    ttl: float = 0.0                # 0 = 常驻；>0 = 最后一帧之后保留的秒数
    t: float = field(default_factory=time.time)
    meta: Dict[str, Any] = field(default_factory=dict)

    def age(self, now: Optional[float] = None) -> float:
        return max(0.0, (now if now is not None else time.time()) - self.t)

    def alive(self, now: Optional[float] = None) -> bool:
        return self.ttl <= 0.0 or self.age(now) <= self.ttl

    def aabb_size(self) -> Tuple[float, float, float]:
        """把带 yaw 的盒子包成轴对齐盒（碰撞用，只会更大）。"""
        if self.kind == "cylinder":
            r, h = self.size[0], self.size[1]
            return (2.0 * r, 2.0 * r, h)
        w, d, h = self.size
        c, s = abs(math.cos(math.radians(self.yaw))), abs(math.sin(math.radians(self.yaw)))
        return (w * c + d * s, w * s + d * c, h)

    def to_obstacle(self) -> Dict[str, Any]:
        """转成 ``cell.CELL["obstacles"]`` 的条目（AABB 外扩，保守）。"""
        if self.kind == "cylinder":
            r, h = self.size[0], self.size[1]
            return {"name": self.name, "type": "cylinder", "radius": float(r),
                    "height": float(h), "base": [float(self.center[0]),
                                                 float(self.center[1]), 0.0],
                    "source": self.source}
        w, d, h = self.aabb_size()
        return {"name": self.name, "type": "box",
                "size": [round(w, 4), round(d, 4), round(h, 4)],
                "center": [float(self.center[0]), float(self.center[1]), float(h) / 2.0],
                "source": self.source}

    def as_dict(self, now: Optional[float] = None) -> Dict[str, Any]:
        return {
            "name": self.name, "kind": self.kind,
            "center": [round(float(self.center[0]), 4), round(float(self.center[1]), 4)],
            "size": [round(float(v), 4) for v in self.size],
            "yaw": round(float(self.yaw), 2),
            "source": self.source,
            "age": round(self.age(now), 2),
            "ttl": round(float(self.ttl), 2),
            "meta": self.meta,
        }


# ================================================================ 场景

class Scene:
    """标定状态 + 物体表。服务里只有一个实例，网页和脚本读同一份。"""

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(path) if path else _SETTINGS_PATH
        self.H: Optional[List[List[float]]] = None      # 像素 → 基座
        self.Hinv: Optional[List[List[float]]] = None   # 基座 → 像素
        self.calib: Dict[str, Any] = {}
        self._objects: Dict[str, SceneObject] = {}
        self._lock = threading.RLock()
        self.last_vision: Dict[str, Any] = {}
        self.last_vision_t = 0.0
        self._load()

    # -- 标定 -------------------------------------------------------------
    @property
    def calibrated(self) -> bool:
        return self.H is not None and self.Hinv is not None

    def _load(self) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        try:
            H = data.get("homography")
            if isinstance(H, list) and len(H) == 3 and all(len(r) == 3 for r in H):
                self.H = [[float(v) for v in row] for row in H]
                self.Hinv = invert_h(self.H)
                self.calib = {k: data.get(k) for k in
                              ("pixels", "world", "rms_mm", "max_mm", "mm_per_px",
                               "rotate", "mirror", "updated")}
        except (TypeError, ValueError, RuntimeError):
            self.H = self.Hinv = None
            self.calib = {}

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {"homography": self.H, **self.calib,
                "updated": time.strftime("%Y-%m-%d %H:%M:%S")}
        self.path.write_text(json.dumps(data, ensure_ascii=False, indent=2),
                             encoding="utf-8")

    def calibrate(self, pixels: Sequence[Sequence[float]],
                  world: Optional[Sequence[Sequence[float]]] = None,
                  rotate: int = 0, mirror: bool = False,
                  note: str = "") -> Dict[str, Any]:
        """用 4 对（或更多）点标定；``world`` 不给就用桌角预设。"""
        pix = _points(pixels, 2, "pixels")
        if world is None:
            if len(pix) != 4:
                raise ValueError("不给 world 时必须正好 4 个点（对应四个桌角）")
            wld = [list(p) for p in preset_world(rotate, mirror)]
        else:
            wld = _points(world, 2, "world")
        H = solve_homography(pix, wld)
        rp = reprojection(H, pix, wld)
        scale = mm_per_px(pix, wld)
        with self._lock:
            self.H, self.Hinv = H, invert_h(H)
            self.calib = {
                "pixels": pix, "world": wld,
                "rms_mm": round(rp["rms_mm"], 2), "max_mm": round(rp["max_mm"], 2),
                "mm_per_px": round(scale, 3),
                "rotate": int(rotate), "mirror": bool(mirror), "note": note,
            }
            # 旧物体是按旧映射算出来的，换标定后位置全错：清掉，别让它继续参与安全判断。
            self._objects.clear()
            self.last_vision = {}
            self.last_vision_t = 0.0
            self.save()
        return self.summary()

    def clear_calibration(self) -> Dict[str, Any]:
        with self._lock:
            self.H = self.Hinv = None
            self.calib = {}
            self._objects.clear()
            self.last_vision = {}
            self.last_vision_t = 0.0
            try:
                self.path.unlink()
            except OSError:
                pass
        return self.summary()

    # -- 投影 -------------------------------------------------------------
    def unproject(self, u: float, v: float) -> Optional[Vec2]:
        """像素 → 桌面 (x, y)；没标定或落在无穷远线外时给 None。"""
        if not self.calibrated:
            return None
        try:
            x, y = apply_h(self.H, float(u), float(v))
        except ValueError:
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return (x, y)

    def project(self, x: float, y: float) -> Optional[Vec2]:
        """桌面 (x, y) → 像素。画叠加图、验证对位时用。"""
        if not self.calibrated:
            return None
        try:
            u, v = apply_h(self.Hinv, float(x), float(y))
        except ValueError:
            return None
        if not (math.isfinite(u) and math.isfinite(v)):
            return None
        return (u, v)

    def table_polygon(self) -> List[Vec2]:
        """桌面四角在当前画面里的像素位置（标定后才知道）。"""
        if not self.calibrated:
            return []
        hx, hy = TABLE_SIZE[0] / 2.0, TABLE_SIZE[1] / 2.0
        out: List[Vec2] = []
        for x, y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            p = self.project(x, y)
            if p:
                out.append((round(p[0], 1), round(p[1], 1)))
        return out

    # -- 物体 -------------------------------------------------------------
    def _put(self, obj: SceneObject) -> SceneObject:
        with self._lock:
            name, i = obj.name, 2
            while name in self._objects:
                name = f"{obj.name}#{i}"
                i += 1
            obj.name = name
            self._objects[name] = obj
        return obj

    def place(self, name: str = "物体", u: Optional[float] = None,
              v: Optional[float] = None, x: Optional[float] = None,
              y: Optional[float] = None, size: Sequence[float] = (0.06, 0.06, 0.06),
              kind: str = "box", yaw: float = 0.0, ttl: float = 0.0,
              meta: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """手动放一个物体：给像素 (u,v) 或已经算好的世界 (x,y)。

        ``ttl=0`` 表示常驻（人放的料盒、杯子）；视觉物体才用 TTL。
        """
        if kind not in ("box", "cylinder"):
            raise ValueError("kind 只能是 box / cylinder")
        if len(size) not in (2, 3):
            raise ValueError("size：box 要 [w,d,h]，cylinder 要 [r,h]")
        if x is None or y is None:
            if u is None or v is None:
                raise ValueError("要么给 u/v（像素），要么给 x/y（基座坐标）")
            p = self.unproject(float(u), float(v))
            if p is None:
                raise ValueError("没有标定，没法把像素换成基座坐标（先 /scene/calibrate）")
            x, y = p
        dims = tuple(float(s) for s in size)
        if kind == "box" and len(dims) == 2:
            dims = (dims[0], dims[1], dims[1])
        if kind == "cylinder" and len(dims) == 3:
            dims = (dims[0], dims[2])
        obj = self._put(SceneObject(name=str(name), kind=kind, center=(float(x), float(y)),
                                    size=dims, yaw=float(yaw), source="manual",
                                    ttl=float(ttl), meta=dict(meta or {})))
        return obj.as_dict()

    def update_cube(self, result: Dict[str, Any], cube_m: float = CUBE_M) -> Dict[str, Any]:
        """把一次 ``CubeVision.analyze()`` 的读数变成桌面上的一个立方体。

        没找到时**只更新心跳**、不动物体：让旧的按 TTL 自然过期。这样臂挡住
        相机的一两帧不会让障碍物从安全模型里消失。
        """
        now = time.time()
        self.last_vision_t = now
        self.last_vision = {k: result.get(k) for k in
                            ("found", "reliable", "angle", "score", "ms")}
        if not result.get("found") or not self.calibrated:
            return {"ok": False, "reason": "没找到魔方" if self.calibrated else "没标定"}
        center = result.get("center") or []
        if len(center) != 2:
            return {"ok": False, "reason": "读数里没有 center"}
        p = self.unproject(float(center[0]), float(center[1]))
        if p is None:
            return {"ok": False, "reason": "center 反投影失败"}
        # 朝向：取「中心 + 一个格子边长」这个点，再反投影，两点连线就是世界朝向。
        # 正方形 90° 一循环，所以 yaw 只按 0..90 用。
        yaw = 0.0
        cell_px = float(result.get("cell") or 0.0)
        theta = math.radians(float(result.get("theta") or 0.0))
        if cell_px > 1e-6:
            du, dv = math.cos(theta) * cell_px, math.sin(theta) * cell_px
            q = self.unproject(float(center[0]) + du, float(center[1]) + dv)
            if q is not None:
                yaw = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))
        meta = {"px": [round(float(center[0]), 1), round(float(center[1]), 1)],
                "theta": round(float(result.get("theta") or 0.0), 2),
                "angle": result.get("angle"), "cells": result.get("cells"),
                "reliable": bool(result.get("reliable")),
                "score": result.get("score"), "ms": result.get("ms"),
                "seen": round(now, 3)}
        with self._lock:
            old = self._objects.get("相机·魔方")
            obj = old or SceneObject(name="相机·魔方", kind="box", center=p,
                                     size=(cube_m, cube_m, cube_m))
            obj.center = p
            obj.size = (cube_m, cube_m, cube_m)
            obj.yaw = yaw
            obj.source = "vision"
            obj.ttl = CUBE_TTL
            obj.t = now
            obj.meta = meta
            self._objects[obj.name] = obj
        return {"ok": True, "object": obj.as_dict(now)}

    def update_box(self, center: Sequence[float],
                   size: Sequence[float] = (CUBE_M, CUBE_M, CUBE_M),
                   yaw: float = 0.0, name: str = "相机·魔方",
                   ttl: float = CUBE_TTL,
                   meta: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """把「只认位置」的立方体检测结果写进物体表。

        和 ``update_cube`` 共用同一个名字和 TTL：面读数成功就用面读数，
        失败就退回这里，表里不会出现两个魔方。
        """
        now = time.time()
        self.last_vision_t = now
        with self._lock:
            old = self._objects.get(name)
            obj = old or SceneObject(name=name, kind="box",
                                     center=(float(center[0]), float(center[1])),
                                     size=tuple(float(v) for v in size))
            obj.kind = "box"
            obj.center = (float(center[0]), float(center[1]))
            obj.size = tuple(float(v) for v in size)
            obj.yaw = float(yaw)
            obj.source = "vision"
            obj.ttl = float(ttl)
            obj.t = now
            obj.meta = dict(meta or {})
            self._objects[obj.name] = obj
        return {"ok": True, "object": obj.as_dict(now)}

    def update_scan(self, objects: Sequence[Dict[str, Any]],
                    now: Optional[float] = None,
                    source: str = "scan") -> Dict[str, Any]:
        """把「整桌扫描」的结果换进物体表。

        和 ``update_cube`` 的区别：魔方视觉只给一个物体，扫描给**一整张表**。
        所以这里不是 upsert 单个名字，而是把上一轮扫描出来的整组换掉——桌子
        上少了一个东西，它就该从碰撞模型里消失。手动放的物体（``source`` 不是
        vision）不动。

        ``objects`` 里每个是 ``{name, kind, center, size, yaw, meta}``；``ttl``
        统一给 ``CUBE_TTL``，和魔方视觉一样「最后一帧之后还留 3 秒」。
        """
        now = now if now is not None else time.time()
        built: List[SceneObject] = []
        for raw in objects:
            try:
                center = [float(v) for v in (raw.get("center") or [])]
                size = [float(v) for v in (raw.get("size") or [])]
            except (TypeError, ValueError):
                continue
            if len(center) != 2 or len(size) not in (2, 3):
                continue
            kind = str(raw.get("kind") or "box")
            if kind not in ("box", "cylinder"):
                kind = "box"
            dims: Tuple[float, ...] = tuple(size)
            if kind == "cylinder":
                # 扫描给的是 [宽, 深, 高] 的外接盒；圆柱只要 (半径, 高)
                dims = (max(dims[0], dims[1]) / 2.0, dims[-1])
            elif len(dims) == 2:
                dims = (dims[0], dims[1], dims[1])
            obj = SceneObject(name=str(raw.get("name") or "物体"), kind=kind,
                              center=(center[0], center[1]), size=dims,
                              yaw=float(raw.get("yaw") or 0.0), source="vision",
                              ttl=float(raw.get("ttl") or CUBE_TTL),
                              t=now, meta=dict(raw.get("meta") or {}))
            built.append(obj)
        with self._lock:
            for name, obj in list(self._objects.items()):
                if obj.source == "vision" and obj.meta.get("origin") == source:
                    del self._objects[name]
            for obj in built:
                name, i = obj.name, 2
                while name in self._objects:
                    name = f"{obj.name}#{i}"
                    i += 1
                obj.name = name
                self._objects[name] = obj
        self.last_vision_t = now
        self.last_vision = {"found": len(built), "mode": "scan", "source": source,
                            "count": len(built)}
        return {"ok": True, "count": len(built),
                "objects": [o.as_dict(now) for o in built]}

    def update_object(self, name: str, new_name: Optional[str] = None,
                      size: Optional[Sequence[float]] = None,
                      yaw: Optional[float] = None,
                      kind: Optional[str] = None) -> Dict[str, Any]:
        """改一个已有物体的名字/尺寸/朝向（页面上点物体改名用）。"""
        with self._lock:
            obj = self._objects.get(name)
            if obj is None:
                raise ValueError(f"没有这个物体：{name}")
            if new_name and new_name != name:
                if new_name in self._objects:
                    raise ValueError(f"名字已被占用：{new_name}")
                del self._objects[name]
                obj.name = str(new_name)
                self._objects[obj.name] = obj
            if kind in ("box", "cylinder"):
                if kind != obj.kind:
                    obj.kind = kind
                    obj.size = ((obj.size[0], obj.size[-1]) if kind == "cylinder"
                                else (obj.size[0], obj.size[0], obj.size[1]))
            if size is not None:
                dims = tuple(float(v) for v in size)
                if obj.kind == "cylinder":
                    if len(dims) == 3:
                        dims = (dims[0], dims[2])
                    if len(dims) != 2:
                        raise ValueError("圆柱尺寸要 [r, h]")
                else:
                    if len(dims) == 2:
                        dims = (dims[0], dims[1], dims[1])
                    if len(dims) != 3:
                        raise ValueError("方块尺寸要 [w, d, h]")
                obj.size = dims
            if yaw is not None:
                obj.yaw = float(yaw)
            return obj.as_dict()

    def remove_object(self, name: str) -> bool:
        with self._lock:
            return self._objects.pop(name, None) is not None

    def clear_objects(self, source: Optional[str] = None) -> int:
        with self._lock:
            names = [n for n, o in self._objects.items()
                     if source is None or o.source == source]
            for n in names:
                del self._objects[n]
        return len(names)

    def objects(self, now: Optional[float] = None) -> List[Dict[str, Any]]:
        now = now if now is not None else time.time()
        with self._lock:
            dead = [n for n, o in self._objects.items() if not o.alive(now)]
            for n in dead:
                del self._objects[n]
            return [o.as_dict(now) for o in self._objects.values()]

    def vision_age(self, now: Optional[float] = None) -> Optional[float]:
        if self.last_vision_t <= 0:
            return None
        return max(0.0, (now if now is not None else time.time()) - self.last_vision_t)

    # -- 合并进 cell ------------------------------------------------------
    def cell(self, now: Optional[float] = None) -> Dict[str, Any]:
        """静态工位 + 当前物体 = 安全校验和三维视图共用的那一份几何。"""
        now = now if now is not None else time.time()
        merged = copy.deepcopy(cell_mod.CELL)
        with self._lock:
            for obj in self._objects.values():
                if obj.alive(now):
                    merged["obstacles"].append(obj.to_obstacle())
        return merged

    def summary(self, now: Optional[float] = None) -> Dict[str, Any]:
        now = now if now is not None else time.time()
        objs = self.objects(now)
        age = self.vision_age(now)
        return {
            "ok": True,
            "calibrated": self.calibrated,
            "rms_mm": self.calib.get("rms_mm"),
            "max_mm": self.calib.get("max_mm"),
            "mm_per_px": self.calib.get("mm_per_px"),
            "rotate": self.calib.get("rotate"),
            "mirror": self.calib.get("mirror"),
            "updated": self.calib.get("updated"),
            "objects": objs,
            "count": len(objs),
            "vision_age": None if age is None else round(age, 2),
            "vision_stale": bool(age is not None and age > VISION_STALE_S),
            "table_polygon": self.table_polygon(),
        }

    def state(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "available": HAS_NUMPY,
            "calibrated": self.calibrated,
            "homography": self.H,
            "inverse": self.Hinv,
            "calibration": {
                "pixels": self.calib.get("pixels"),
                "world": self.calib.get("world"),
                "rms_mm": self.calib.get("rms_mm"),
                "max_mm": self.calib.get("max_mm"),
                "mm_per_px": self.calib.get("mm_per_px"),
                "rotate": self.calib.get("rotate"),
                "mirror": self.calib.get("mirror"),
                "updated": self.calib.get("updated"),
            },
            "table": {"size": list(TABLE_SIZE), "top": cell_mod.TABLE_TOP},
            "objects": self.objects(),
            "vision": {"age": self.vision_age(), "last": self.last_vision},
            "summary": self.summary(),
        }


# ================================================================ 台线程

class VisionHub:
    """后台线程：按 ``hz`` 从相机取帧，跑视觉，喂给 ``Scene``。

    两种模式，按 ``scan`` 是否开启切换：

    * ``scan``（整桌扫描，``TableScan`` 在 ``tablescan.py``）：一帧里把桌上
      所有东西分割出来，全部写进场景——这是「看到什么是什么」那条路；
    * ``cube``（老路）：只认魔方，``CubeVision.analyze()`` 一次，更新一个物体。

    * 没标定就不跑（省 CPU）；
    * 一次扫描约 20～60 ms、魔方分析约 100～180 ms，所以默认 1 Hz，别拿它当推流；
    * 如果 ``/camera`` 那边的识别器刚出过读数（``< 0.45 s``），直接复用，
      不重复分析同一帧——两个线程各跑一遍会把 CPU 吃满；
    * 相机没开 / 读帧失败就安静等待，错误记在 ``errors`` 里，不刷屏。
    """

    def __init__(self, camera, cube, scene: Scene, scan=None, hz: float = 1.0) -> None:
        self.camera = camera
        self.cube = cube
        self.scene = scene
        self.scan = scan
        self.hz = max(0.2, min(5.0, float(hz)))
        self.frames = 0
        self.reused = 0
        self.errors = 0
        self.last_error = ""
        self.last_ms = 0.0
        self.last_mode = "idle"
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="scene-vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def set_hz(self, hz: float) -> float:
        self.hz = max(0.2, min(5.0, float(hz)))
        return self.hz

    def _run(self) -> None:  # pragma: no cover - 线程循环，离线测试不跑
        while not self._stop.is_set():
            t0 = time.time()
            try:
                if self.scene.calibrated:
                    frame = None
                    if self.scan is not None and self.scan.enabled:
                        frame = self.camera.read_raw()
                        if frame is not None:
                            result = self.scan.run(frame)
                            self.last_mode = "scan"
                            self.last_ms = float(result.get("ms") or 0.0)
                            if result.get("ok"):
                                self.frames += 1
                    else:
                        result = None
                        last = self.cube.last or {}
                        stamp = float(last.get("stamp") or 0.0)
                        if stamp and time.time() - stamp < 0.45:
                            result = last          # 识别器刚跑过，复用同一份读数
                            self.reused += 1
                        else:
                            frame = self.camera.read_raw()
                            if frame is not None:
                                result = self.cube.analyze(frame)
                        if result is not None:
                            self.last_mode = "cube"
                            self.last_ms = float(result.get("ms") or 0.0)
                            self.scene.update_cube(result)
                            # 9 格读不出来时退回「只当它是一个立方体」，抓取只要位置。
                            if frame is not None and not result.get("found"):
                                det = boxcv.find_box(frame)
                                if det.get("found"):
                                    loc = boxcv.locate(self.scene, det)
                                    if loc.get("ok"):
                                        self.scene.update_box(
                                            loc["center"],
                                            size=(loc["size_m"], loc["size_m"],
                                                  loc["size_m"]),
                                            yaw=float(loc.get("yaw") or 0.0),
                                            meta={"origin": "box", "px": det.get("center_px"),
                                                  "bbox": det.get("bbox"),
                                                  "fill": det.get("fill"),
                                                  "ms": det.get("ms")},
                                        )
                                        self.last_mode = "box"
                            self.frames += 1
                else:
                    self.last_mode = "idle"
            except Exception as exc:            # noqa: BLE001
                self.errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
            dt = 1.0 / self.hz - (time.time() - t0)
            self._stop.wait(max(0.05, dt))

    def state(self) -> Dict[str, Any]:
        return {"running": bool(self._thread and self._thread.is_alive()),
                "hz": self.hz, "frames": self.frames, "reused": self.reused,
                "errors": self.errors, "last_error": self.last_error,
                "mode": self.last_mode, "last_ms": round(self.last_ms, 1)}
