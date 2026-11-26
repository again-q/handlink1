"""Gemini 336L 深度相机：取 RGB-D、拟合桌面、量出物体的长宽高。

为什么加这一层
--------------
单目只能靠视差「估」高度（±10～20%），深度相机直接给毫米级的三维点，桌面上
物体的长宽高就变成**测量**而不是估计：

    深度图 + 内参 ──► 相机系点云 ──► 拟合桌面平面 ──► 平面以上的点
                                                       └─► 聚类 ──► 每个物体的
                                                           长/宽/高/中心

坐标：这一层只输出**相机系**（米），因为「物体多大」不需要机器人坐标；
物体的 (x, y) 仍然走 ``scene.py`` 的桌面单应（或者以后用机械臂探针标定）。
这样两块可以分开验证：尺寸对不对看深度，位置对不对看单应。

相机怎么调
----------
深度相机认的是自己的坐标系，但**必须看得到桌面**：采集到的有效点少于 30%、
或者中位距离大于 3 m，基本就是没对着桌面（``state()["hint"]`` 会写）。

接口
----
    cam = DepthCamera()
    cam.open()                      # 只开深度（彩色走 USB 摄像头）
    color, depth, intr = cam.read()
    plane = fit_plane(depth_points(depth, intr))
    objs = measure_objects(depth, intr, plane)
    cam.close()

颜色流说明：Gemini 的彩色是 1280×720 MJPG，Windows 上个别机器会报
``hardware MFT failed``（缺 MJPG 解码器），这时 ``open(with_color=False)``
只开深度；彩色用旁边的 USB 摄像头，标定和尺寸互不影响。
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:                                        # SDK 是可选的：没装也能 import 本模块
    import pyorbbecsdk as ob  # type: ignore
    HAS_SDK = True
except Exception:                           # noqa: BLE001
    ob = None                               # type: ignore
    HAS_SDK = False

#: 桌面拟合的 RANSAC 内点阈值（米）。
PLANE_TOL_M = 0.006

#: 物体高度下限（米）：低于这个算桌面噪声/纸屑。
MIN_OBJ_H_M = 0.008

#: 物体高度上限（米）：高于这个多半是把人/墙算进来了。
MAX_OBJ_H_M = 0.60

#: 每个物体最少点数，滤掉噪点。
MIN_POINTS = 120


@dataclass
class DepthObject:
    """相机系里的一个物体（桌面上方）。"""

    center: Tuple[float, float, float]     # 点云质心（米，相机系）
    size: Tuple[float, float, float]       # (长, 宽, 高)，米；长/宽是桌面内的外接矩形
    points: int
    height_m: float                        # 顶面高于桌面的量
    label: str = "物体"

    def as_dict(self) -> Dict[str, Any]:
        return {"center": [round(v, 4) for v in self.center],
                "size": [round(v, 4) for v in self.size],
                "height_m": round(self.height_m, 4),
                "points": int(self.points), "label": self.label}


def depth_points(depth: np.ndarray, intr: Dict[str, float],
                 step: int = 2) -> np.ndarray:
    """深度图（uint16 毫米）→ 相机系点云 (N,3)，米。``step`` 是下采样步长。"""
    h, w = depth.shape[:2]
    fx, fy = float(intr["fx"]), float(intr["fy"])
    cx, cy = float(intr["cx"]), float(intr["cy"])
    vs, us = np.mgrid[0:h:step, 0:w:step]
    z = depth[::step, ::step].astype(np.float32) / 1000.0
    valid = z > 0.05
    us = us[valid].astype(np.float32)
    vs = vs[valid].astype(np.float32)
    z = z[valid]
    x = (us - cx) / fx * z
    y = (vs - cy) / fy * z
    return np.stack([x, y, z], axis=1)


def fit_plane(points: np.ndarray, tol: float = PLANE_TOL_M,
              iters: int = 200, seed: int = 0) -> Optional[Dict[str, Any]]:
    """RANSAC 拟合桌面平面：返回 ``{normal, d, inliers, ratio}``，平面是 n·p = d。"""
    if len(points) < 50:
        return None
    if len(points) > 12000:                 # 拟合不需要全量点，抽稀到 1.2 万
        idx = np.random.default_rng(seed).choice(len(points), 12000, replace=False)
        points = points[idx]
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        idx = rng.choice(len(points), 3, replace=False)
        p0, p1, p2 = points[idx]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        d = float(n @ p0)
        dist = np.abs(points @ n - d)
        inl = dist < tol
        count = int(inl.sum())
        if best is None or count > best[0]:
            best = (count, n, d, inl)
    if best is None:
        return None
    count, n, d, inl = best
    # 用内点做一次最小二乘精修
    P = points[inl]
    if len(P) >= 3:
        c = P.mean(axis=0)
        _u, _s, vt = np.linalg.svd(P - c)
        n = vt[-1]
        d = float(n @ c)
    if n[2] > 0:                            # 法向朝相机（z 负方向），便于理解
        n, d = -n, -d
    return {"normal": [float(v) for v in n], "d": float(d),
            "inliers": int(count), "ratio": round(count / len(points), 3)}


def plane_basis(normal: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """平面内两个正交方向（长/宽方向），朝向任意，只用于量尺寸。"""
    n = np.asarray(normal, dtype=float)
    n = n / (np.linalg.norm(n) + 1e-12)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(float(n @ ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    e1 = np.cross(n, ref)
    e1 /= np.linalg.norm(e1) + 1e-12
    e2 = np.cross(n, e1)
    return e1, e2


def _label(size: Tuple[float, float, float]) -> str:
    L, D, H = size
    long_side, short_side = max(L, D), min(L, D)
    if H <= 0.012 and long_side >= 0.05:
        return "平铺物"
    if long_side / max(short_side, 1e-3) >= 3.0 or short_side <= 0.018:
        return "细长物"
    if H >= 1.15 * long_side:
        return "杯/筒"
    if 0.45 <= short_side / long_side <= 1.7:
        return "盒子"
    return "物块"


def measure_objects(depth: np.ndarray, intr: Dict[str, float],
                    plane: Optional[Dict[str, Any]] = None,
                    step: int = 2) -> Dict[str, Any]:
    """量出桌面上所有物体：返回 ``{plane, objects, valid_ratio}``。

    做法：点云 → 拟合桌面 → 桌面以上的点 → 按图像网格做连通域 → 每个连通域
    投影到桌面平面，外接矩形给长/宽，最高点给高。不需要彩色，也不需要机器人坐标。
    """
    pts = depth_points(depth, intr, step=step)
    if len(pts) < 50:
        return {"ok": False, "error": "深度点太少（相机没对着桌面？）",
                "objects": [], "valid_ratio": 0.0}
    # 先看一眼距离：Gemini 336L 的有效量程大约 0.25～3 m，中位 5 m 基本是
    # 对着房间/走廊了，不是桌面——直接报错，别拿房间算出「1.5 米的盒子」。
    z_med = float(np.median(pts[:, 2]))
    if z_med > 3.0:
        return {"ok": False,
                "error": f"深度中位 {z_med:.1f} m：相机没对着桌面，先把它转到桌面正上方",
                "objects": [], "valid_ratio": round(len(pts) / max(1.0, depth.size / (step * step)), 3)}
    if plane is None:
        plane = fit_plane(pts)
    if plane is None:
        return {"ok": False, "error": "拟合不出桌面平面", "objects": [],
                "valid_ratio": round(len(pts) / (depth.size / (step * step)), 3)}
    if plane["ratio"] < 0.25:
        return {"ok": False,
                "error": f"桌面平面只占 {plane['ratio'] * 100:.0f}% 的点：画面里没有大块平面，"
                         f"相机可能对着墙/地面，或者桌面被挡住了",
                "objects": [], "plane": plane, "valid_ratio": round(plane["ratio"], 3)}
    n = np.asarray(plane["normal"], dtype=float)
    d = float(plane["d"])
    e1, e2 = plane_basis(n)
    height = pts @ n - d                       # 离桌面；桌面上的点≈0
    above = (height > MIN_OBJ_H_M) & (height < MAX_OBJ_H_M)
    ratio = float(above.mean())

    # 用「投影到平面」的坐标做二维直方图/连通域，避免只靠图像相邻
    uv = np.stack([pts @ e1, pts @ e2], axis=1)
    # 2 cm 网格聚类（栅格 + 连通域），不依赖 scipy
    cell = 0.02
    ij = np.floor(uv[above] / cell).astype(np.int64)
    if len(ij) < MIN_POINTS:
        return {"ok": True, "plane": plane, "objects": [], "valid_ratio": round(ratio, 3)}
    # 归一化到从 0 开始
    ij -= ij.min(axis=0)
    gh, gw = int(ij[:, 0].max()) + 1, int(ij[:, 1].max()) + 1
    grid = np.zeros((gh, gw), dtype=np.uint8)
    grid[ij[:, 0], ij[:, 1]] = 1
    # 4 邻域连通域（手写 BFS，避免依赖 cv2/scipy）
    objects: List[Dict[str, Any]] = []
    lab = np.zeros((gh, gw), dtype=np.int32)
    next_label = 0
    for a, b in zip(*np.nonzero(grid)):
        if lab[a, b]:
            continue
        next_label += 1
        stack = [(int(a), int(b))]
        lab[a, b] = next_label
        while stack:
            x, y = stack.pop()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < gh and 0 <= ny < gw and grid[nx, ny] and not lab[nx, ny]:
                    lab[nx, ny] = next_label
                    stack.append((nx, ny))

    labels_pt = lab[ij[:, 0], ij[:, 1]]      # 每个点落在哪个连通域
    for k in range(1, next_label + 1):
        sel = labels_pt == k
        P = pts[above][sel]
        if len(P) < MIN_POINTS:
            continue
        # 平面内尺寸
        u = P @ e1
        v = P @ e2
        L = float(u.max() - u.min())
        D = float(v.max() - v.min())
        h = float((P @ n - d).max())
        c = P.mean(axis=0)
        size = (max(L, 0.01), max(D, 0.01), max(h, 0.004))
        objects.append({"center": [float(v) for v in c],
                        "size": [float(v) for v in size],
                        "height_m": float(h), "points": int(len(P)),
                        "label": _label(size)})
    objects.sort(key=lambda o: o["points"], reverse=True)
    return {"ok": True, "plane": plane, "objects": objects,
            "valid_ratio": round(ratio, 3)}


class DepthCamera:
    """Gemini 336L：只开深度（默认），可选彩色。"""

    def __init__(self) -> None:
        self.pipeline = None
        self.device = None
        self.intr: Optional[Dict[str, float]] = None
        self.scale = 1.0
        self.last_error = ""
        self.frames = 0

    @property
    def available(self) -> bool:
        return HAS_SDK

    def open(self, with_color: bool = False) -> Dict[str, Any]:
        if not HAS_SDK:
            raise RuntimeError("没有 pyorbbecsdk：pip install pyorbbecsdk2 的 cp313 轮子")
        ctx = ob.Context()
        devs = ctx.query_devices()
        if devs.get_count() < 1:
            raise RuntimeError("没找到 Orbbec 相机（USB 插好、别被别的程序占着）")
        self.device = devs.get_device_by_index(0)
        self.pipeline = ob.Pipeline(self.device)
        cfg = ob.Config()
        dp = self.pipeline.get_stream_profile_list(
            ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
        cfg.enable_stream(dp)
        if with_color:
            cp = self.pipeline.get_stream_profile_list(
                ob.OBSensorType.COLOR_SENSOR).get_default_video_stream_profile()
            cfg.enable_stream(cp)
        self.pipeline.start(cfg)
        param = self.pipeline.get_camera_param()
        self.intr = {"fx": float(param.depth_intrinsic.fx),
                     "fy": float(param.depth_intrinsic.fy),
                     "cx": float(param.depth_intrinsic.cx),
                     "cy": float(param.depth_intrinsic.cy),
                     "width": int(param.depth_intrinsic.width),
                     "height": int(param.depth_intrinsic.height)}
        self.scale = 1.0
        return {"ok": True, "intr": self.intr}

    def read(self, timeout_ms: int = 1500):
        """返回 ``(color_bgr | None, depth_mm uint16, intrinsics)``。"""
        if self.pipeline is None:
            raise RuntimeError("相机没打开（先 open()）")
        frames = self.pipeline.wait_for_frames(timeout_ms)
        if frames is None:
            return None, None, self.intr
        d = frames.get_depth_frame()
        if d is None:
            return None, None, self.intr
        depth = np.frombuffer(d.get_data(), dtype=np.uint16).reshape(
            d.get_height(), d.get_width()).copy()
        self.frames += 1
        color = None
        c = frames.get_color_frame()
        if c is not None:
            import cv2
            buf = np.frombuffer(c.get_data(), dtype=np.uint8)
            color = cv2.imdecode(buf, cv2.IMREAD_COLOR)   # MJPG
        return color, depth, self.intr

    def close(self) -> None:
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:                # noqa: BLE001
                pass
        self.pipeline = None
        self.device = None

    def hint(self, depth: Optional[np.ndarray]) -> str:
        """给页面/命令行的「相机没对好」提示。"""
        if depth is None:
            return "没有深度帧"
        valid = float((depth > 0).mean())
        if valid < 0.30:
            return f"有效深度只有 {valid * 100:.0f}%：相机没对着桌面，或者被挡住了"
        return ""


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse
    import cv2

    ap = argparse.ArgumentParser(description="Gemini 336L：取深度、量桌面物体")
    ap.add_argument("--grab", action="store_true", help="抓一帧深度，存图并打印质量")
    ap.add_argument("--measure", action="store_true", help="拟合桌面并量出所有物体")
    ap.add_argument("--out", default=".tmp/scan/orb", help="输出前缀")
    args = ap.parse_args(argv)
    cam = DepthCamera()
    if not cam.available:
        print("没有 pyorbbecsdk")
        return 2
    cam.open()
    try:
        color, depth, intr = cam.read()
        if depth is None:
            print("没有深度帧：", cam.hint(depth))
            return 1
        hint = cam.hint(depth)
        print(f"深度 {depth.shape[1]}×{depth.shape[0]}  "
              f"有效 {float((depth > 0).mean()) * 100:.1f}%  {hint}")
        if args.grab:
            norm = np.clip((depth.astype(np.float32) - 200) / 2000.0, 0, 1)
            img = cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
            cv2.imwrite(args.out + "-depth.png", img)
            np.save(args.out + "-depth.npy", depth)
            print("已存", args.out + "-depth.png")
        if args.measure:
            res = measure_objects(depth, intr)
            if not res.get("ok"):
                print("测量失败：", res.get("error"))
                return 1
            plane = res["plane"]
            print(f"桌面：法向 {[round(v, 2) for v in plane['normal']]}  "
                  f"内点 {plane['ratio'] * 100:.0f}%")
            for i, o in enumerate(res["objects"], 1):
                L, D, H = [v * 1000 for v in o["size"]]
                print(f"  {i}. {o['label']:<4} {L:.0f} × {D:.0f} × {H:.0f} mm  "
                      f"点数 {o['points']}")
            if not res["objects"]:
                print("  （桌面上没量到东西）")
        return 0
    finally:
        cam.close()


if __name__ == "__main__":
    import sys

    sys.exit(main())
