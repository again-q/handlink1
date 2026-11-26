"""魔方视觉：俯视相机 → 「朝上那面」的 9 格颜色 + 层转角。

这一层只做纯计算，不碰相机、不碰服务：给它一帧 BGR 画面，它回答三个问题

1. 魔方的 U 面（朝上那面）在哪：一个正方形（四个角像素坐标）；
2. 那 9 格里各是什么颜色：U/R/F/D/L/B 中的一个（或 ?）；
3. 这一层相对「基准朝向」转了多少度：[-45, 45)，用来判 90° 到没到、补推多少。

怎么认出来的
------------
不训练模型，靠几何 + 颜色：

* 彩色贴纸是一块块高饱和的小方块。用 HSV 的饱和度把「非白」贴纸挑出来，
  每块的 minAreaRect 自带边长和角度——边长给出格子间距，角度给出这一层的朝向。
* 9 格中心必然落在「中心 ± 一格」的 3x3 点阵上。于是拿任意一块贴纸当锚点，
  枚举它在 3x3 里的 9 种位置，就能反推整张面的中心；再用「格子是否平整 +
  格线是否发黑」给每个假设打分，分最高的就是真面。
* 白色贴纸不靠阈值找，而是整面摆正之后在 9 格里直接取样：白贴纸自然就是
  低饱和、高亮度的那几格，颜色分类交给 Lab 距离。

标定
----
* 调色板：把复原的魔方六个面依次怼到相机下，各学一次颜色参考。学了就用学
  的，没学就用标准配色。换灯光/换相机要重学（Lab 距离对白平衡很敏感）。
* 基准朝向：托盘装好后，把魔方摆正（U 面与托盘边平行），点一次「设基准」，
  之后读到的角度都是相对它的。

对外接口
--------
CubeVision 同时实现 camera.Camera 需要的识别器接口（info/detect/draw），
所以网页上开 mode="cube" 就能直接在推流画面上看到框和读数。
"""
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

# opencv 可能装在 .tmp/pylibs 里（camera.load_cv2 会处理），所以这里不能直接
# 假定 import cv2 成功。
cv2 = None
np = None
_IMPORT_ERROR = ""
try:                                  # 先按常规路径找
    import cv2 as _cv2  # type: ignore
    import numpy as _np  # type: ignore

    cv2, np = _cv2, _np
except Exception:                     # noqa: BLE001 - 缺依赖不该让整包 import 失败
    try:
        from .camera import load_cv2

        cv2 = load_cv2()
        if cv2 is not None:
            import numpy as _np  # type: ignore

            np = _np
    except Exception as exc:          # noqa: BLE001
        _IMPORT_ERROR = f"{type(exc).__name__}: {exc}"

HAS_CV2 = cv2 is not None and np is not None

# ---------------------------------------------------------------- 常量

LETTERS = ("U", "R", "F", "D", "L", "B")

# 标准配色（BGR）。U 白 / R 红 / F 绿 / D 黄 / L 橙 / B 蓝
DEFAULT_BGR: Dict[str, Tuple[int, int, int]] = {
    "U": (238, 238, 240),
    "R": (40, 30, 196),
    "F": (70, 160, 30),
    "D": (0, 214, 250),
    "L": (25, 120, 240),
    "B": (200, 70, 25),
}

COLOR_NAME = {"U": "白", "R": "红", "F": "绿", "D": "黄", "L": "橙", "B": "蓝"}

CELLS = 3
# 摆正时多摆两圈：5×5 的窗口里，外面那一圈必须「不是贴纸」，才能否掉整个框
# 平移一格的错误假设（3×3 点阵是周期性的，只看里面 9 格分不出来）。
GRID = 5
WARP_PX = 240
CELL_PX = WARP_PX // GRID         # 60
INNER0 = 1                        # 内 3×3 从第 1 格开始

# 魔方贴纸边长 / 格子边长。竞速魔方约 17.5/19 ≈ 0.92。
STICKER_RATIO = 0.92
MIN_BLOB_AREA = 30
# Lab 下「这一格就是某个面色」的距离上限（ΔE76）。
GOOD_DE = 38.0
# 检测合格线
MIN_GAP_DARK = 0.65               # 内部四条格线至少八成是黑的（歪一格会掉到 ~0.5）
MIN_GOOD = 0.85                   # 9 格必须几乎全是「贴着黑缝的贴纸」

_SETTINGS_PATH = Path(os.environ.get("HANDLINK_VISION")
                       or (Path(__file__).resolve().parent.parent / "config" / "vision.json"))


# ---------------------------------------------------------------- 颜色

def bgr_to_lab(bgr: Sequence[float]) -> Tuple[float, float, float]:
    """BGR(0..255) → CIELab，L 归一到 0..100。"""
    if not HAS_CV2:
        raise RuntimeError("没有 opencv / numpy")
    px = np.array([[[max(0, min(255, int(round(float(v))))) for v in bgr[:3]]]], dtype=np.uint8)
    lab = cv2.cvtColor(px, cv2.COLOR_BGR2LAB)[0, 0].astype(np.float64)
    return (float(lab[0]) * 100.0 / 255.0, float(lab[1]) - 128.0, float(lab[2]) - 128.0)


def delta_e(a: Sequence[float], b: Sequence[float]) -> float:
    """ΔE76：Lab 空间的欧氏距离，够用且好解释。"""
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


@dataclass
class Palette:
    """六种面色的 Lab 参考。learned 记录哪些是真机上学来的。"""

    refs: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    learned: set = field(default_factory=set)

    @classmethod
    def default(cls) -> "Palette":
        if not HAS_CV2:
            return cls({})
        return cls({k: bgr_to_lab(v) for k, v in DEFAULT_BGR.items()}, set())

    def classify(self, lab: Sequence[float]) -> Tuple[str, float]:
        """最近的面色，返回 (字母, ΔE)。"""
        best, best_d = "?", float("inf")
        for letter, ref in self.refs.items():
            d = delta_e(lab, ref)
            if d < best_d:
                best, best_d = letter, d
        return best, best_d

    def learn(self, letter: str, lab: Sequence[float]) -> None:
        if letter not in LETTERS:
            raise ValueError(f"没有这个面: {letter}（只能是 {''.join(LETTERS)}）")
        self.refs[letter] = (float(lab[0]), float(lab[1]), float(lab[2]))
        self.learned.add(letter)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "refs": {k: [round(v, 2) for v in lab] for k, lab in self.refs.items()},
            "learned": sorted(self.learned),
            "complete": len(self.learned) >= 6,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Palette":
        refs = {k: tuple(v) for k, v in (data.get("refs") or {}).items() if k in LETTERS}
        return cls(refs, set(data.get("learned") or []))


# ---------------------------------------------------------------- 小工具

def _rot(x: float, y: float, deg: float) -> Tuple[float, float]:
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return x * c - y * s, x * s + y * c


def _circular_median_90(angles: Sequence[float]) -> float:
    """把一组角度按 90° 周期性平均（贴纸是正方形，转 90° 看着一样）。"""
    if not angles:
        return 0.0
    sx = sum(math.cos(math.radians(4 * a)) for a in angles)
    sy = sum(math.sin(math.radians(4 * a)) for a in angles)
    if abs(sx) < 1e-9 and abs(sy) < 1e-9:
        return float(angles[0]) % 90.0
    return (math.degrees(math.atan2(sy, sx)) / 4.0) % 90.0


def quad_of(center: Sequence[float], cell: float, theta: float) -> List[List[float]]:
    """由「面中心 + 格子边长 + 朝向」生成正方形四角（TL, TR, BR, BL）。"""
    half = 1.5 * cell
    out = []
    for u, v in ((-half, -half), (half, -half), (half, half), (-half, half)):
        dx, dy = _rot(u, v, theta)
        out.append([float(center[0] + dx), float(center[1] + dy)])
    return out


def wrap_angle(deg: float) -> float:
    """把角度折到 [-45, 45)：离最近的 90° 整倍数差多少。"""
    return ((float(deg) + 45.0) % 90.0) - 45.0


def _warp(frame, quad) -> Optional[Any]:
    """把一张 3×3 候选面（四角）连同外面一圈一起摆正成 5×5 的正方形。"""
    if not HAS_CV2:
        return None
    src3 = np.array(quad, dtype=np.float32)
    if abs(float(cv2.contourArea(src3))) < 4.0:
        return None
    centre = src3.mean(axis=0)
    src5 = centre + (src3 - centre) * (GRID / float(CELLS))
    dst = np.array([[0, 0], [WARP_PX - 1, 0], [WARP_PX - 1, WARP_PX - 1], [0, WARP_PX - 1]],
                   dtype=np.float32)
    m = cv2.getPerspectiveTransform(src5, dst)
    return cv2.warpPerspective(frame, m, (WARP_PX, WARP_PX), flags=cv2.INTER_AREA)


_INNER_WINDOWS: List[Tuple[int, int, int, int]] = []


def _inner_windows() -> List[Tuple[int, int, int, int]]:
    """内 3×3 九个格子的取样窗（y0, y1, x0, x1）：格子中心 56% 的一块。"""
    global _INNER_WINDOWS
    if not _INNER_WINDOWS:
        half = max(3, int(CELL_PX * 0.28))
        for r in range(CELLS):
            for c in range(CELLS):
                x = (INNER0 + c) * CELL_PX + CELL_PX // 2
                y = (INNER0 + r) * CELL_PX + CELL_PX // 2
                _INNER_WINDOWS.append((y - half, y + half, x - half, x + half))
    return _INNER_WINDOWS


def _v_base(face):
    """亮度通道 + 这一面的「贴纸亮度基准」。黑缝的判据都以它为准。"""
    v = cv2.cvtColor(face, cv2.COLOR_BGR2HSV)[:, :, 2]
    vv = [float(np.median(v[y0:y1, x0:x1])) for (y0, y1, x0, x1) in _inner_windows()]
    return v, (float(np.median(vv)) or 1.0)


def _dark_mask(v, base: float):
    """黑缝掩码：比贴纸基准暗一半以上的像素。整张图一次算完，别再逐点采样。"""
    return v < 0.5 * base


def _dark_edges(dark, x: int, y: int, half: int = CELL_PX // 2) -> int:
    """这一格的上下左右四条边有几条是黑的（黑缝）。"""
    hit = 0
    for dx, dy in ((0, -half), (0, half), (-half, 0), (half, 0)):
        px, py = x + dx, y + dy
        patch = dark[max(0, py - 2):py + 3, max(0, px - 2):px + 3]   # 5×5，容 ±2 px 对齐误差
        if patch.size and float(patch.mean()) > 0.25:
            hit += 1
    return hit


def _gap_dark(dark) -> float:
    """内 3×3 的四道格线有多黑：0（和贴纸一样亮）～1（全黑）。

    魔方黑色骨架在贴纸之间留下约 2 mm 的缝；5×5 摆正之后就是 x/y = 一格、两格
    的两条线。真面在这四条线上必然发黑；把桌面当魔方，这里就露馅。
    """
    lo = INNER0 * CELL_PX
    hi = (INNER0 + CELLS) * CELL_PX
    # 先把每条列/行在内层长度上的黑像素比例算出来，再在 ±6 px 里取最黑的一条。
    # 格子边长是估的，真缝可能偏几个像素；只取 1 px 线（缝摆正后只有 2~3 px 宽）。
    col = dark[lo:hi, :].mean(axis=0)
    row = dark[:, lo:hi].mean(axis=1)
    fracs = []
    for k in (1, 2):
        line = (INNER0 + k) * CELL_PX
        fracs.append(float(col[max(0, line - 6):line + 7].max()))
        fracs.append(float(row[max(0, line - 6):line + 7].max()))
    return sum(fracs) / len(fracs) if fracs else 0.0


def _face_stats(face, dark):
    """按格子切 5×5 摆正图。

    返回 dict：内 9 格的中位 BGR、Lab、平整度（灰度标准差）、四条边有几条黑缝。
    「贴纸」= 颜色像某个面色 + 平整 + 至少两条边是黑缝；桌面/白纸没有黑缝，
    所以它不会被当成贴纸——这是否掉「整个框平移一格」的关键。
    """
    lab = cv2.cvtColor(face, cv2.COLOR_BGR2LAB)
    gray = cv2.cvtColor(face, cv2.COLOR_BGR2GRAY)
    bgr_out: List[Any] = []
    lab_out: List[Tuple[float, float, float]] = []
    std_out: List[float] = []
    edges_out: List[int] = []
    for y0, y1, x0, x1 in _inner_windows():
        roi = face[y0:y1, x0:x1]
        cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
        if roi.size == 0:
            bgr_out.append(np.zeros(3))
            lab_out.append((0.0, 0.0, 0.0))
            std_out.append(255.0)
            edges_out.append(0)
            continue
        bgr_out.append(np.median(roi.reshape(-1, 3), axis=0))
        med = np.median(lab[y0:y1, x0:x1].reshape(-1, 3), axis=0).astype(np.float64)
        lab_out.append((float(med[0]) * 100.0 / 255.0, float(med[1]) - 128.0, float(med[2]) - 128.0))
        std_out.append(float(gray[y0:y1, x0:x1].std()))
        edges_out.append(_dark_edges(dark, cx, cy))
    return {"bgr": bgr_out, "lab": lab_out, "std": std_out, "edges": edges_out}


# ---------------------------------------------------------------- 检测

@dataclass
class Blob:
    center: Tuple[float, float]
    side: float          # 贴纸短边
    angle: float         # 0..90
    area: float


def _sticker_blobs(frame) -> List[Blob]:
    """高饱和的贴纸块（不含白）。面积/长宽比/填充率三道过滤。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    mask = ((s >= 60) & (v >= 40)).astype(np.uint8) * 255
    # 只做 3×3 开运算去掉噪点；不要用 5×5 闭运算——贴纸缝只有 4~5 px，
    # 一闭就把 9 块贴纸连成一整块，边长直接变成整张面。
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out: List[Blob] = []
    for cnt in contours:
        area = float(cv2.contourArea(cnt))
        if area < MIN_BLOB_AREA:
            continue
        (cx, cy), (w, h), ang = cv2.minAreaRect(cnt)
        w, h = max(float(w), float(h)), min(float(w), float(h))
        if w <= 1.0 or h <= 1.0:
            continue
        if h / w < 0.55:                      # 长条不是贴纸
            continue
        if area / (w * h) < 0.55:             # 空心的不要
            continue
        out.append(Blob((float(cx), float(cy)), h, float(ang) % 90.0, area))
    if not out:
        return []
    sides = sorted(b.side for b in out)
    med = sides[len(sides) // 2]
    return [b for b in out if 0.55 * med <= b.side <= 1.8 * med]


class CubeVision:
    """魔方面识别器 + 标定状态。服务、脚本、网页共用同一个实例。"""

    name = "cube-face"

    def __init__(self, settings_path: Optional[Path] = None) -> None:
        self.path = Path(settings_path) if settings_path else _SETTINGS_PATH
        self.palette = Palette.default()
        self.ref_angle: Optional[float] = None      # 托盘基准（原始 0..90）
        self.manual_quad: Optional[List[List[float]]] = None
        self.roi: Optional[List[int]] = None        # x, y, w, h
        self.last: Dict[str, Any] = {}
        self.last_ms = 0.0
        self.detect_count = 0
        self.fail_count = 0
        self._load()

    # -- 标定状态 ---------------------------------------------------------
    def _load(self) -> None:
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        try:
            pal = data.get("palette")
            if pal:
                self.palette = Palette.from_dict(pal)
            ref = data.get("ref_angle")
            self.ref_angle = None if ref is None else float(ref)
            quad = data.get("manual_quad")
            self.manual_quad = [[float(x), float(y)] for x, y in quad] if quad else None
            roi = data.get("roi")
            self.roi = [int(v) for v in roi] if roi else None
        except (TypeError, ValueError):
            pass

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            "palette": self.palette.as_dict(),
            "ref_angle": self.ref_angle,
            "manual_quad": self.manual_quad,
            "roi": self.roi,
            "updated": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    def info(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "available": HAS_CV2,
            "hint": "" if HAS_CV2 else ("没有 opencv/numpy" + (f"（{_IMPORT_ERROR}）" if _IMPORT_ERROR else "")),
            "palette": sorted(self.palette.learned),
            "ref_angle": self.ref_angle,
            "manual": self.manual_quad is not None,
        }

    # -- 检测 -------------------------------------------------------------
    def _score(self, frame, quad, theta, strict: bool = True,
               relaxed: bool = False) -> Optional[Dict[str, Any]]:
        """把一张候选面摆正、读 9 格，并给出「像不像魔方面」的分数。

        及格线有两道：内部格线够黑（gap），9 格都是「颜色像面色 + 平整 +
        至少两条边是黑缝」的贴纸（good）。桌面/白纸没有黑缝，所以哪怕颜色像白色
        也过不了 good——这是否掉「整个框平移一格」的关键。

        relaxed=True 时不要求颜色落在调色板附近（学配色时调色板还不可信）。
        """
        face = _warp(frame, quad)
        if face is None:
            return None
        v, base = _v_base(face)
        dark = _dark_mask(v, base)
        gap = _gap_dark(dark)
        if strict and gap < MIN_GAP_DARK - 0.02:
            # 格线都不黑，后面算 Lab/灰度纯属浪费：这个假设直接否掉。
            return {"face": face, "stats": [], "labs": [], "labels": ["?"] * 9,
                    "dists": [999.0] * 9, "gap": gap, "good": 0.0,
                    "score": 0.65 * gap, "theta": theta, "rejected": True}
        feats = _face_stats(face, dark)
        labels: List[str] = []
        dists: List[float] = []
        like = 0
        for lab, std, edges in zip(feats["lab"], feats["std"], feats["edges"]):
            letter, d = self.palette.classify(lab)
            dists.append(d)
            color_ok = d <= GOOD_DE
            labels.append(letter if (color_ok or relaxed) else "?")
            if (color_ok or relaxed) and std < 20.0 and edges >= 2:
                like += 1
        good = like / 9.0
        score = 0.65 * gap + 0.35 * good
        return {
            "face": face, "stats": feats["bgr"], "labs": feats["lab"], "labels": labels,
            "dists": dists, "gap": gap, "good": good, "score": score, "theta": theta,
            "edges": feats["edges"],
        }

    def _gap_only(self, frame, quad) -> Optional[float]:
        """只算「内部格线黑不黑」——比完整读 9 格便宜一个数量级，用来粗排。"""
        face = _warp(frame, quad)
        if face is None:
            return None
        v, base = _v_base(face)
        return _gap_dark(_dark_mask(v, base))

    def _candidates(self, blobs: Sequence[Blob], cell0: float) -> List[Tuple[float, float]]:
        """中心假设。拿每块贴纸当锚点，枚举它在 3x3 里的位置。"""
        theta = _circular_median_90([b.angle for b in blobs])
        raw: List[Tuple[float, float]] = []
        for b in blobs:
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    dx, dy = _rot(i * cell0, j * cell0, theta)
                    raw.append((b.center[0] - dx, b.center[1] - dy))
        uniq: List[Tuple[float, float]] = []
        for c in raw:
            if all(math.dist(c, u) > cell0 * 0.3 for u in uniq):
                uniq.append(c)
        prev = self.last.get("center") if isinstance(self.last, dict) else None
        if prev:
            uniq.sort(key=lambda c: math.dist(c, prev))
        return uniq[:10]

    def detect_face(self, frame, relaxed: bool = False) -> Optional[Dict[str, Any]]:
        """自动找 U 面。返回 {quad, center, cell, theta, ...} 或 None。

        relaxed=True 时只看几何（格线黑不黑），不要求颜色落在调色板附近——
        学配色本身就得先找到面，这时调色板还不可信。
        """
        if not HAS_CV2 or frame is None:
            return None
        work = frame
        off = (0, 0)
        if self.roi:
            x, y, w, h = self.roi
            x = max(0, min(int(x), frame.shape[1] - 8))
            y = max(0, min(int(y), frame.shape[0] - 8))
            w = max(8, min(int(w), frame.shape[1] - x))
            h = max(8, min(int(h), frame.shape[0] - y))
            work = frame[y:y + h, x:x + w]
            off = (x, y)

        theta = 0.0
        cell0 = 0.0
        if self.manual_quad:
            quad = [tuple(p) for p in self.manual_quad]
            cx = sum(p[0] for p in quad) / 4.0
            cy = sum(p[1] for p in quad) / 4.0
            side = math.dist(quad[0], quad[1])
            theta = math.degrees(math.atan2(quad[1][1] - quad[0][1],
                                            quad[1][0] - quad[0][0])) % 90.0
            scored = self._score(work, quad, theta)
            if scored is None:
                return None
            return {"quad": [list(p) for p in quad], "center": [cx, cy],
                    "cell": side / 3.0, "theta": theta, **scored, "manual": True}

        min_good = 0.0 if relaxed else MIN_GOOD
        best = self._from_stickers(work, min_good, relaxed)
        if best is None:
            # 全白面没有彩色贴纸可抓，退回「黑色骨架」这条线
            best = self._from_body(work, min_good, relaxed)
        if best is None:
            return None
        if off != (0, 0):
            ox, oy = off
            best["quad"] = [[p[0] + ox, p[1] + oy] for p in best["quad"]]
            best["center"] = [best["center"][0] + ox, best["center"][1] + oy]
        return best

    def _from_stickers(self, work, min_good: float = MIN_GOOD,
                       relaxed: bool = False) -> Optional[Dict[str, Any]]:
        """主路线：彩色贴纸 → 面中心 + 格子边长。

        难点是「一块贴纸」和「九块贴纸糊成一块」在尺寸上只差 3 倍，光看面积分不清。
        所以对同一块色斑试三种解释：单块（÷0.92）、两连（÷1.84）、整面（÷2.88），
        交给打分去否定错的那个——整面边框的格线只有真面才黑得下来。
        """
        blobs = _sticker_blobs(work)
        if not blobs:
            return None
        size = float(np.median([b.side for b in blobs]))
        theta = _circular_median_90([b.angle for b in blobs])
        h, w = work.shape[:2]

        rough: List[Tuple[float, float, float, float]] = []
        min_side = 0.10 * min(h, w)
        max_side = 0.98 * max(h, w)
        for ratio in (STICKER_RATIO, 1.84, 2.88):
            cell0 = size / ratio
            if cell0 < 5 or not (min_side <= 3.0 * cell0 <= max_side):
                continue
            for cx, cy in self._candidates(blobs, cell0):
                if not (0 <= cx < w and 0 <= cy < h):
                    continue
                quad = quad_of((cx, cy), cell0, theta)
                xs = [p[0] for p in quad]
                ys = [p[1] for p in quad]
                margin = 0.05
                if (min(xs) < -margin * w or max(xs) > (1 + margin) * w
                        or min(ys) < -margin * h or max(ys) > (1 + margin) * h):
                    continue
                # 贴纸必须都落在框里：歪一格的框会把边上的贴纸甩出去
                poly = np.array(quad, dtype=np.float32)
                if any(cv2.pointPolygonTest(poly, (b.center[0], b.center[1]), False) < -2.0
                       for b in blobs):
                    continue
                gap = self._gap_only(work, quad)
                if gap is None or gap < MIN_GAP_DARK:
                    continue
                rough.append((float(gap), cx, cy, cell0))
        rough.sort(key=lambda t: -t[0])
        best: Optional[Dict[str, Any]] = None
        for _g, cx, cy, cell0 in rough[:3]:          # 格线最黑的前三名再完整读
            cand = self._refine(work, cx, cy, cell0, theta, min_good, relaxed)
            if cand is None:
                continue
            if best is None or cand["score"] > best["score"]:
                best = cand
        return best

    def _refine(self, work, cx, cy, cell, theta, min_good: float = MIN_GOOD,
                relaxed: bool = False) -> Optional[Dict[str, Any]]:
        """在 (中心, 格子边长, 朝向) 附近做一次小范围搜索，取分数最高的那张。

        先用原尺度试：对得上就直接收工（大多数情况如此，省掉 14 次重算）；
        对不上才在 ±12% 的尺度里找——那是「九块贴纸糊成一块」时估出来的格子略大。
        """
        best: Optional[Dict[str, Any]] = None
        for scale in (1.0, 0.96, 1.04, 0.92, 1.08):
            for dtheta in (0.0, -2.0, 2.0):
                quad = quad_of((cx, cy), cell * scale, theta + dtheta)
                scored = self._score(work, quad, theta + dtheta, relaxed=relaxed)
                if scored is None:
                    continue
                if scored["gap"] < MIN_GAP_DARK or scored["good"] < min_good:
                    continue
                cand = {"quad": quad, "center": [cx, cy], "cell": cell * scale,
                        "theta": (theta + dtheta) % 90.0, **scored,
                        "manual": False, "source": "stickers"}
                if best is None or cand["score"] > best["score"]:
                    best = cand
            if scale == 1.0 and best is not None:
                break
        return best

    def _from_body(self, work, min_good: float = MIN_GOOD,
                   relaxed: bool = False) -> Optional[Dict[str, Any]]:
        """兜底：整面全白时靠黑色骨架的外框。

        黑色骨架的灰度远低于贴纸和桌面，取最大的一块近似正方形的暗区，外框边长
        就是 3 格。它的朝向与 U 层一致；但 U 层被转开时，下层露出的角会让外框
        偏大——所以这条路的置信度单独标出来（source="body"）。
        """
        hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2].astype(np.uint8)
        # 骨架「黑到什么程度」随曝光/贴纸颜色变，固定阈值不好使：试几档，
        # 每档都过一遍打分。Otsu 那一档应对「白贴纸把缝挤得很亮」的情况。
        otsu, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        thresholds = {70, 95, 120, max(60, min(160, int(otsu)))}
        best: Optional[Dict[str, Any]] = None
        limit = 0.95 * min(work.shape[:2])
        for thr in thresholds:
            mask = (v < thr).astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = float(cv2.contourArea(cnt))
                if area < 400:
                    continue
                (cx, cy), (w, h), ang = cv2.minAreaRect(cnt)
                w, h = max(float(w), float(h)), min(float(w), float(h))
                if w <= 1 or h / w < 0.62:
                    continue
                if area / (w * h) < 0.55:
                    continue
                if w > limit:
                    continue
                theta = float(ang) % 90.0
                quad = quad_of((cx, cy), w / 3.0, theta)
                scored = self._score(work, quad, theta, relaxed=relaxed)
                if scored is None or scored["gap"] < MIN_GAP_DARK or scored["good"] < min_good:
                    continue
                cand = {"quad": quad, "center": [cx, cy], "cell": w / 3.0, "theta": theta,
                        **scored, "manual": False, "source": f"body@{thr}"}
                if best is None or cand["score"] > best["score"]:
                    best = cand
        return best

    # -- 读数 -------------------------------------------------------------
    def analyze(self, frame) -> Dict[str, Any]:
        t0 = time.perf_counter()
        found = self.detect_face(frame)
        ms = (time.perf_counter() - t0) * 1000.0
        self.last_ms = ms
        if not found:
            self.fail_count += 1
            self.last = {"found": False, "ms": round(ms, 1), "stamp": time.time()}
            return self.last
        self.detect_count += 1
        theta = float(found["theta"])
        ref = self.ref_angle if self.ref_angle is not None else 0.0
        # 正方形转 90° 看着一样，所以「相对转角」只能折到 [-45, 45)。
        # 格子顺序必须跟基准连续：用 ref+转角 重摆一次正，而不是沿用检测器
        # 随手给的 0..90（否则同一层转 90° 后 9 格会整体错位）。
        theta_rel = wrap_angle(theta - ref)
        # 9 格永远按「托盘/魔方本体」的朝向取样，而不是跟着这一层转：层转 90°
        # 之后，本体格子里装的本来就是别的颜色——这才是要读的「状态」。
        # 转到 45° 附近时格子会落在缝上，颜色不可信，只有角度还有意义。
        grid_theta = ref
        quad = quad_of(found["center"], found["cell"], grid_theta)
        # strict=False：面已经找到了，这里只是换个朝向重读 9 格；层没完全对正时
        # 格线会变浅，但格子中心还在贴纸上，颜色照样可信。
        scored = self._score(frame, quad, grid_theta, strict=False) if frame is not None else None
        if scored is None:
            scored = found
            quad = found["quad"]
        reliable = abs(theta_rel) <= 12.0
        self.last = {
            "found": True,
            "reliable": reliable,
            # quad 是贴着这一层的框（画面上看得出来）；read_quad 是读数用的本体朝向，
            # 层转到一半时两者会差一个角度，颜色只有在可靠区间里才当真。
            "quad": [[round(float(x), 1), round(float(y), 1)] for x, y in found["quad"]],
            "read_quad": [[round(float(x), 1), round(float(y), 1)] for x, y in quad],
            "center": [round(float(found["center"][0]), 1), round(float(found["center"][1]), 1)],
            "cell": round(float(found["cell"]), 2),
            "theta": round(theta, 2),
            "angle": round(theta_rel, 2),
            "ref_angle": round(float(ref), 2),
            "cells": scored["labels"],
            "colors": [[int(c) for c in bgr] for bgr in scored["stats"]],
            "conf": [round(float(d), 1) for d in scored["dists"]],
            "labs": [[round(float(v), 1) for v in lab] for lab in scored["labs"]],
            "gap": round(float(scored["gap"]), 3),
            "good": round(float(scored["good"]), 3),
            "score": round(float(scored["score"]), 3),
            "source": found.get("source", "manual" if found.get("manual") else "stickers"),
            "manual": bool(found.get("manual")),
            "ms": round(ms, 1),
            "stamp": time.time(),
        }
        return self.last

    # -- 标定 -------------------------------------------------------------
    def learn(self, letter: str, frame) -> Dict[str, Any]:
        """把一个面（复原状态的纯色面）的实测颜色记进调色板。"""
        found = self.detect_face(frame, relaxed=True)
        if not found:
            return {"ok": False, "error": "没找到魔方面：让魔方以复原面朝上、占画面 1/4 左右再试"}
        labs = found["labs"]
        med = tuple(float(np.median([lab[k] for lab in labs])) for k in range(3))
        self.palette.learn(letter, med)
        self.save()
        return {"ok": True, "face": letter, "lab": [round(v, 1) for v in med],
                "palette": self.palette.as_dict()}

    def set_reference(self, frame) -> Dict[str, Any]:
        # 只关心朝向，颜色是不是准确无所谓
        found = self.detect_face(frame, relaxed=True)
        if not found:
            return {"ok": False, "error": "没找到魔方面：把魔方摆正、U 面朝上再点一次"}
        self.ref_angle = float(found["theta"])
        self.save()
        return {"ok": True, "ref_angle": round(self.ref_angle, 2)}

    def set_manual(self, points: Optional[Sequence[Sequence[float]]]) -> Dict[str, Any]:
        if not points:
            self.manual_quad = None
            self.save()
            return {"ok": True, "manual": False}
        if len(points) != 4:
            return {"ok": False, "error": "要 4 个角点 [[x,y], ...]"}
        self.manual_quad = [[float(p[0]), float(p[1])] for p in points]
        self.save()
        return {"ok": True, "manual": True, "quad": self.manual_quad}

    def reset(self) -> Dict[str, Any]:
        self.palette = Palette.default()
        self.ref_angle = None
        self.manual_quad = None
        self.roi = None
        self.save()
        return {"ok": True, "reset": True}

    def state(self) -> Dict[str, Any]:
        return {
            "ok": True,
            "available": HAS_CV2,
            "last": self.last,
            "palette": self.palette.as_dict(),
            "ref_angle": self.ref_angle,
            "manual_quad": self.manual_quad,
            "roi": self.roi,
            "detect_ms": round(self.last_ms, 1),
            "stats": {"ok": self.detect_count, "fail": self.fail_count},
        }

    # -- 导出 -------------------------------------------------------------
    def annotated_png(self, frame, analyze_frame: bool = True) -> bytes:
        """在画面上画出框/格/字母，返回 PNG 字节（存证据、发给别人看都用它）。"""
        if not HAS_CV2:
            raise RuntimeError("没有 opencv/numpy")
        if analyze_frame:
            self.analyze(frame)
        canvas = frame.copy()
        self.draw(canvas, {"cube": self.last})
        ok, buf = cv2.imencode(".png", canvas)
        if not ok:
            raise RuntimeError("PNG 编码失败")
        return buf.tobytes()

    # -- 识别器接口（camera.Camera 用） -----------------------------------
    def detect(self, frame) -> Dict[str, Any]:
        res = self.analyze(frame)
        h, w = (frame.shape[0], frame.shape[1]) if frame is not None else (0, 0)
        return {"cube": res, "width": w, "height": h}

    def draw(self, frame, result: Dict[str, Any]):
        if not HAS_CV2 or frame is None or not result:
            return frame
        cube = result.get("cube") or result.get("last") or {}
        if not cube.get("found"):
            cv2.putText(frame, "cube: not found", (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (80, 80, 220), 1, cv2.LINE_AA)
            return frame
        quad = [tuple(int(round(v)) for v in p) for p in cube["quad"]]
        cv2.polylines(frame, [np.array(quad, dtype=np.int32)], True, (80, 220, 120), 2, cv2.LINE_AA)
        for k in (1, 2):
            a = (quad[0][0] + (quad[1][0] - quad[0][0]) * k / 3,
                 quad[0][1] + (quad[1][1] - quad[0][1]) * k / 3)
            b = (quad[3][0] + (quad[2][0] - quad[3][0]) * k / 3,
                 quad[3][1] + (quad[2][1] - quad[3][1]) * k / 3)
            c = (quad[0][0] + (quad[3][0] - quad[0][0]) * k / 3,
                 quad[0][1] + (quad[3][1] - quad[0][1]) * k / 3)
            d = (quad[1][0] + (quad[2][0] - quad[1][0]) * k / 3,
                 quad[1][1] + (quad[2][1] - quad[1][1]) * k / 3)
            cv2.line(frame, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (80, 220, 120), 1, cv2.LINE_AA)
            cv2.line(frame, (int(c[0]), int(c[1])), (int(d[0]), int(d[1])), (80, 220, 120), 1, cv2.LINE_AA)
        cells = cube.get("cells") or []
        # 字母按「读数框」摆（本体朝向）；框本身用贴着这一层的 quad，看着才贴合
        read_quad = [tuple(int(round(v)) for v in p) for p in
                     (cube.get("read_quad") or cube["quad"])]
        for idx in range(9):
            r, c = divmod(idx, 3)
            u = (c + 0.5) / 3.0
            v = (r + 0.5) / 3.0
            top_x = read_quad[0][0] + (read_quad[1][0] - read_quad[0][0]) * u
            top_y = read_quad[0][1] + (read_quad[1][1] - read_quad[0][1]) * u
            bot_x = read_quad[3][0] + (read_quad[2][0] - read_quad[3][0]) * u
            bot_y = read_quad[3][1] + (read_quad[2][1] - read_quad[3][1]) * u
            x = top_x + (bot_x - top_x) * v
            y = top_y + (bot_y - top_y) * v
            letter = cells[idx] if idx < len(cells) else "?"
            bgr = DEFAULT_BGR.get(letter, (40, 40, 40))
            cv2.putText(frame, letter, (int(x) - 6, int(y) + 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 255, 255) if sum(bgr) < 380 else (20, 20, 20), 2, cv2.LINE_AA)
        a = cube.get("angle")
        txt = f"U {''.join(cells)}  angle {a:+.1f} deg" if a is not None else f"U {''.join(cells)}"
        cv2.putText(frame, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 220, 120), 2, cv2.LINE_AA)
        if not cube.get("reliable", True):
            cv2.putText(frame, "layer mid-turn: colors unreliable", (10, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 200, 250), 1, cv2.LINE_AA)
        return frame


# ---------------------------------------------------------------- 单例

_VISION: Optional[CubeVision] = None


def shared() -> CubeVision:
    global _VISION
    if _VISION is None:
        _VISION = CubeVision()
    return _VISION
