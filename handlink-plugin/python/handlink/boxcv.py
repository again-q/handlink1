"""把一个立方体/盒子当成盒子：只给位置，不读六个面的状态。

为什么单独一个模块
------------------
``cubecv`` 要回答的是「朝上一面 9 格是什么颜色、转了多少度」，前提是
**垂直俯视 + 纯色贴纸 + 顶面接近正方形**。真实工位经常不满足：相机是斜的、
魔方是图案款、顶面还过曝。抓取其实只需要「它在哪、多大、朝哪」，不需要知道
哪面是白。

这个模块就做这一件事：

    一帧画面 ──高饱和 & 非蓝桌面──► 立方体色块 ──连通块──► 包围框
             ──底边接触点──► 桌面坐标 ──半个边长──► 立方体底面中心

它**不依赖** ``cubecv`` 的调色板、基准和 9 格检测，也不要求顶面是正方形；
斜视、图案贴纸、顶面过曝都能用，只要立方体本身和桌面/纸有颜色差。

坐标约定
--------
* 输入是 BGR 帧；输出像素坐标 ``(u, v)``，图像 y 向下。
* ``locate()`` 返回基座坐标（米），``center`` 是**底面中心**，不是顶面中心。
* 底边接触点在 ``z=0`` 桌面上，所以它没有「高度视差」；再沿画面「上方」
  （远离相机）走半个边长，就得到立方体底面中心。这样不用相机内参，也不用
  知道相机高度。

边界
----
* 只能处理**立方体/竖直盒子**：斜放、叠放、被大面积遮住的会偏。
* 画面里同时有多个高饱和物体时，取**离支撑面中心最近**的那个；要指定就传 ``roi``。
* 机械臂从画面上沿伸进来时，贴着顶边的橙色标签会被排除；桌面上的物体不受影响。
* 桌面如果是红/绿/黄等高饱和颜色，``exclude_blue`` 不够用，得换背景差或 ROI。
"""
from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, Sequence, Tuple

from .camera import HAS_CV2, cv2

try:                                        # 常规路径
    import numpy as np  # type: ignore
except Exception:                           # noqa: BLE001
    from .cubecv import np  # type: ignore  # cubecv 已经找过 pylibs

#: 小于这个像素面积的连通块当噪声。640×480 下约 30×30 px。
MIN_AREA_PX = 900.0

#: 超过画面这个比例的大色块不是桌上的立方体（多半是整张桌布/背景）。
MAX_AREA_FRAC = 0.30

#: 包围框填充率下限。星形/箭头贴纸被闭运算填掉后仍在 0.7 以上。
MIN_FILL = 0.45

#: 包围框长宽比范围。立方体斜视时横向会变宽，放宽到 2.2。
MIN_ASPECT = 0.45
MAX_ASPECT = 2.2

#: 贴纸/物体的饱和度、亮度下限。黄色顶面过曝时 S 会掉到 50 左右，所以取 45。
SAT_MIN = 45
VAL_MIN = 40

#: 蓝色桌面的色相范围（OpenCV H 0..179）。桌面蓝、机械手暗蓝灰都在这里面。
BLUE_HUE = (95, 135)

#: 默认立方体边长（米）。标准竞速魔方 57 mm。
DEFAULT_SIZE_M = 0.057

#: 底边带占包围框高度的比例。用来估「立方体与桌面的接触边」。
BOTTOM_BAND = 0.08


def _as_mask(frame, exclude_blue: bool = True):
    """高饱和、非白、非蓝桌面的二值掩膜。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    m = (s >= SAT_MIN) & (v >= VAL_MIN)
    if exclude_blue:
        m &= ~((h >= BLUE_HUE[0]) & (h <= BLUE_HUE[1]))
    return m.astype(np.uint8) * 255


def _focus_point(frame) -> Tuple[float, float]:
    """画面里最大的一块「白色支撑面」（纸/托盘）的中心。

    立方体通常放在纸上；机械臂的橙色标签、桌上的彩色杂物可能在画面边缘，
    用这个点做距离先验，避免「谁面积大谁就是立方体」。
    """
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m = ((hsv[:, :, 1] <= 45) & (hsv[:, :, 2] >= 150)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < 0.02 * w * h:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw < 0.15 * w or bh < 0.15 * h:
            continue
        if best is None or area > best[0]:
            best = (area, x + bw / 2.0, y + bh / 2.0)
    if best is None:
        return (w / 2.0, h / 2.0)
    return (best[1], best[2])


def find_box(frame, roi: Optional[Sequence[int]] = None,
             min_area: float = MIN_AREA_PX,
             exclude_blue: bool = True,
             focus: Optional[Sequence[float]] = None) -> Dict[str, Any]:
    """在画面里找最大的立方体/盒子色块。

    参数
    ----
    frame : BGR ndarray
    roi : ``(x, y, w, h)``，只在这个矩形里找；默认整幅。
    min_area : 最小像素面积。
    exclude_blue : 是否排除蓝色桌面/机械手。
    focus : 物体预期的像素位置；默认取画面里最大的白色支撑面中心（纸/托盘）。

    返回
    ----
    ``{"found": bool, "bbox": [x, y, w, h], "center_px": [u, v],
       "contact": [u, v], "area": px, "fill": 0..1, "aspect": w/h,
       "score": ..., "ms": ...}``
    """
    t0 = time.perf_counter()
    if not HAS_CV2 or frame is None:
        return {"found": False, "error": "没有 opencv 或没有画面",
                "ms": round((time.perf_counter() - t0) * 1000.0, 1)}
    h, w = frame.shape[:2]
    mask = _as_mask(frame, exclude_blue=exclude_blue)

    x0 = y0 = 0
    rw = rh = 0
    if roi is not None:
        if len(roi) != 4:
            raise ValueError("roi 要 (x, y, w, h)")
        x0, y0, rw, rh = (int(v) for v in roi)
        x0 = max(0, min(x0, w - 8))
        y0 = max(0, min(y0, h - 8))
        rw = max(8, min(rw, w - x0))
        rh = max(8, min(rh, h - y0))
        keep = np.zeros_like(mask)
        keep[y0:y0 + rh, x0:x0 + rw] = 255
        mask = cv2.bitwise_and(mask, keep)

    if focus is None:
        if roi is not None:
            focus = (x0 + rw / 2.0, y0 + rh / 2.0)
        else:
            focus = _focus_point(frame)
    fx, fy = float(focus[0]), float(focus[1])
    sigma = max(40.0, 0.45 * min(h, w))

    # 开运算去噪；闭运算把贴纸图案（箭头/星星）内部的黑洞填掉。
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = MAX_AREA_FRAC * w * h
    best: Optional[Dict[str, Any]] = None
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < min_area or area > max_area:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw < 12 or bh < 12:
            continue
        # 机械臂从画面上边伸进来，橙色标签会是一个大色块；它贴着顶边，
        # 而立方体在桌面上，不会贴着画面最上沿。
        if y <= 2 and (y + bh / 2.0) < 0.25 * h:
            continue
        fill = area / max(1.0, float(bw * bh))
        if fill < MIN_FILL:
            continue
        aspect = bw / max(1.0, float(bh))
        if aspect < MIN_ASPECT or aspect > MAX_ASPECT:
            continue
        # 面积大、填充实、接近正方形、离支撑面中心近的优先。
        # 只按面积排会选到机械臂上的橙色标签（它比魔方大，但在画面边上）。
        dist = math.hypot(x + bw / 2.0 - fx, y + bh / 2.0 - fy)
        prior = math.exp(-(dist / sigma) ** 2)
        score = (area * fill
                 * (1.0 - min(0.5, abs(math.log(aspect)) / 2.0))
                 * prior)
        if best is None or score > best["score"]:
            best = {"x": x, "y": y, "w": bw, "h": bh, "area": area,
                    "fill": fill, "aspect": aspect, "score": score, "cnt": cnt}

    ms = round((time.perf_counter() - t0) * 1000.0, 1)
    if best is None:
        return {"found": False, "ms": ms, "count": len(contours)}

    pts = best["cnt"].reshape(-1, 2)
    v_bottom = int(pts[:, 1].max())
    band_h = max(2, int(BOTTOM_BAND * best["h"]))
    band = pts[pts[:, 1] >= v_bottom - band_h]
    u_contact = float(np.median(band[:, 0])) if len(band) else best["x"] + best["w"] / 2.0
    # 立方体转 45° 时最低处是一个角，底边带会很窄；这时退回包围框中心。
    if len(band) < 4 or (band[:, 0].max() - band[:, 0].min()) < 0.25 * best["w"]:
        u_contact = best["x"] + best["w"] / 2.0
    u_center = best["x"] + best["w"] / 2.0
    v_center = best["y"] + best["h"] / 2.0

    return {
        "found": True,
        "bbox": [int(best["x"]), int(best["y"]), int(best["w"]), int(best["h"])],
        "center_px": [round(u_center, 1), round(v_center, 1)],
        "contact": [round(u_contact, 1), float(v_bottom)],
        "area": int(best["area"]),
        "fill": round(best["fill"], 3),
        "aspect": round(best["aspect"], 3),
        "score": round(best["score"], 1),
        "ms": ms,
    }


def locate(scene, det: Dict[str, Any], size_m: float = DEFAULT_SIZE_M,
           toward: str = "up") -> Dict[str, Any]:
    """把 ``find_box`` 的像素结果换成基座坐标里的底面中心。

    ``scene`` 需要已标定（有 ``unproject``）。``toward="up"`` 表示画面「上方」
    是远离相机的那一侧；相机倒装（旋转 180°）时传 ``"down"``。
    """
    if not det.get("found"):
        return {"ok": False, "error": "画面上没找到立方体"}
    if not getattr(scene, "calibrated", False):
        return {"ok": False, "error": "桌面还没标定（先 /vision 点桌面四角或 /scene/calibrate）"}
    u, v = det["contact"]
    base = scene.unproject(float(u), float(v))
    if base is None:
        return {"ok": False, "error": "底边接触点反投影失败"}
    bh = float(det.get("bbox", [0, 0, 0, 40])[3])
    up_px = max(20.0, 0.25 * bh)
    above = scene.unproject(float(u), float(v) - up_px)
    if above is None:
        return {"ok": False, "error": "画面‘上方’反投影失败"}
    dx, dy = above[0] - base[0], above[1] - base[1]
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return {"ok": False, "error": "底边方向退化，换个位置再试"}
    if toward == "down":
        dx, dy = -dx, -dy
    half = float(size_m) / 2.0
    center = [base[0] + dx / n * half, base[1] + dy / n * half]

    # 底边朝向：把底边带两端点投到世界，连线就是立方体正面（宽度方向）。
    yaw = 0.0
    bbox = det.get("bbox") or [0, 0, 0, 0]
    if len(bbox) == 4 and bbox[2] > 0:
        # 用包围框左右两端、底边高度处两个点估方向；比取轮廓极值稳。
        left = scene.unproject(float(bbox[0]), float(v))
        right = scene.unproject(float(bbox[0] + bbox[2]), float(v))
        if left is not None and right is not None:
            ex, ey = right[0] - left[0], right[1] - left[1]
            if math.hypot(ex, ey) > 1e-6:
                # 立方体 90° 一循环，折到 [-45, 45) 给抓取用更直观。
                yaw = (math.degrees(math.atan2(ey, ex)) + 45.0) % 90.0 - 45.0

    return {
        "ok": True,
        "center": [round(center[0], 5), round(center[1], 5)],
        "center_mm": [round(center[0] * 1000.0, 1), round(center[1] * 1000.0, 1)],
        "contact_mm": [round(base[0] * 1000.0, 1), round(base[1] * 1000.0, 1)],
        "size_m": float(size_m),
        "yaw": round(float(yaw), 1),
        "up": [round(dx / n, 4), round(dy / n, 4)],
    }


def annotate(frame, det: Dict[str, Any], located: Optional[Dict[str, Any]] = None):
    """把检测结果画在帧上（返回 BGR，不改原帧）。"""
    img = frame.copy()
    if not det.get("found"):
        cv2.putText(img, "box: not found", (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 255), 2, cv2.LINE_AA)
        return img
    x, y, w, h = det["bbox"]
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 220, 255), 2)
    u, v = det["contact"]
    cv2.circle(img, (int(round(u)), int(round(v))), 5, (0, 0, 255), -1)
    label = f"box {w}x{h} px  fill {det.get('fill', 0):.2f}"
    cv2.putText(img, label, (x, max(18, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 220, 255), 2, cv2.LINE_AA)
    if located and located.get("ok"):
        mm = located.get("center_mm") or [0, 0]
        text = f"base {mm[0]:.0f},{mm[1]:.0f} mm"
        cv2.putText(img, text, (x, min(img.shape[0] - 10, y + h + 22)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2, cv2.LINE_AA)
    return img
