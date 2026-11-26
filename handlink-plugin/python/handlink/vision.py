"""视觉识别：给摄像头画面接一个「看懂」的钩子。

现在只有一条路：MediaPipe Hands（21 个手部关键点 → 每根手指弯了多少）。
没有装 mediapipe 时 ``make_recognizer()`` 返回一个「不可用」的占位器，
摄像头照常出图，只是不叠骨架——不会因为缺依赖就把预览搞挂。

接自己的模型
------------
实现三个方法就够了::

    class MyRecognizer:
        name = "my-model"
        def info(self) -> dict            # {"name":..., "available": bool, "hint": ...}
        def detect(self, frame_bgr) -> dict   # {"hands": [{"bbox": [x,y,w,h], "flex": {103: 0.4}}]}
        def draw(self, frame_bgr, result) -> frame_bgr

然后把 ``make_recognizer()`` 改成返回它。``camera.Camera`` 每 N 帧调用一次
``detect``，把结果叠到推流画面上，同时通过 ``/camera/state`` 给网页。
"""
from __future__ import annotations

import math
import os
import shutil
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# MediaPipe 的 21 个关键点编号：每根手指 (根, 中节, 尖)
FINGERS = {103: (5, 6, 8), 104: (9, 10, 12), 105: (13, 14, 16), 106: (17, 18, 20)}
THUMB = (2, 3, 4)

# 骨架连线（自己写一份，省得为了画线去 import mp.solutions）
BONES = [
    (0, 1), (1, 2), (2, 3), (3, 4),            # 拇指
    (0, 5), (5, 6), (6, 7), (7, 8),            # 食指
    (5, 9), (9, 10), (10, 11), (11, 12),       # 中指
    (9, 13), (13, 14), (14, 15), (15, 16),     # 无名指
    (13, 17), (17, 18), (18, 19), (19, 20),    # 小指
    (0, 17),
]


def angle_at(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    """b 处的夹角（度）。手指伸直 ≈ 180°，握紧 ≈ 30°。"""
    v1 = (a[0] - b[0], a[1] - b[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 180.0
    cos = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cos))


def flex_from_landmarks(lm) -> Dict[int, float]:
    """21 个归一化关键点 → 每个电机的弯曲量 0..1（与 demo/mirror.py 同一套换算）。"""
    out: Dict[int, float] = {}
    for mid, (a, b, c) in FINGERS.items():
        out[mid] = _clamp01((165.0 - angle_at(lm[a], lm[b], lm[c])) / 135.0)
    thumb = _clamp01((170.0 - angle_at(lm[THUMB[0]], lm[THUMB[1]], lm[THUMB[2]])) / 140.0)
    out[102] = thumb
    out[107] = thumb * 0.9
    return out


def _clamp01(v: float) -> float:
    return 0.0 if v < 0 else 1.0 if v > 1 else v


class NullRecognizer:
    """没装依赖时的占位：永远不产生结果，但接口齐全。"""

    name = "none"

    def __init__(self, hint: str = "") -> None:
        self.hint = hint or "未安装识别依赖"

    def info(self) -> Dict[str, Any]:
        return {"name": self.name, "available": False, "hint": self.hint}

    def detect(self, frame) -> Dict[str, Any]:
        return {}

    def draw(self, frame, result):
        return frame


MODEL_PATH = Path(__file__).resolve().parent / "models" / "hand_landmarker.task"
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/1/hand_landmarker.task")

_ROOT = Path(__file__).resolve().parent.parent


def _prepare_env() -> None:
    """matplotlib 默认往 %LOCALAPPDATA% 写字体缓存，沙箱/受限账号下会直接报错。"""
    if not os.environ.get("MPLCONFIGDIR"):
        cache = _ROOT / ".tmp" / "mplcache"
        try:
            cache.mkdir(parents=True, exist_ok=True)
        except OSError:
            cache = Path(tempfile.gettempdir()) / "handlink-mpl"
        os.environ["MPLCONFIGDIR"] = str(cache)


def _safe_model_path() -> Path:
    """TFLite 的 C++ 层打不开非 ASCII 路径（工作区叫「机械手实验」），拷一份到临时目录。"""
    if MODEL_PATH.is_file() and str(MODEL_PATH).isascii():
        return MODEL_PATH
    dst = Path(tempfile.gettempdir()) / "handlink-models" / MODEL_PATH.name
    dst.parent.mkdir(parents=True, exist_ok=True)
    if not dst.is_file() or dst.stat().st_size != MODEL_PATH.stat().st_size:
        shutil.copyfile(MODEL_PATH, dst)
    return dst


_prepare_env()


class HandLandmarks:
    """MediaPipe 手部关键点：认出 21 个点，换算成 6 个电机的弯曲量。

    mediapipe 1.x 移除了老的 ``mp.solutions.hands``，改用 Tasks API + 一个
    ``hand_landmarker.task`` 模型文件；0.10.x 还能走老接口，两条路都留着。
    """

    name = "mediapipe-hands"

    def __init__(self, max_hands: int = 1, complexity: int = 0) -> None:
        self.max_hands = max_hands
        self.complexity = complexity
        self._hands = None          # 0.10.x 的 solutions 对象
        self._landmarker = None     # 1.x 的 Tasks 对象
        self._cv2 = None
        self._mp = None
        self._error = ""

    # -- 能力探测 ---------------------------------------------------------
    def _load(self) -> bool:
        if self._hands is not None or self._landmarker is not None:
            return True
        try:
            import cv2  # type: ignore
            import mediapipe as mp  # type: ignore
        except Exception as exc:            # noqa: BLE001 - 缺依赖就是缺依赖
            self._error = f"{type(exc).__name__}: {exc}"
            return False
        self._cv2 = cv2
        self._mp = mp
        try:
            if hasattr(mp, "solutions"):                 # mediapipe 0.10.x
                self._hands = mp.solutions.hands.Hands(
                    max_num_hands=self.max_hands,
                    model_complexity=self.complexity,
                    min_detection_confidence=0.6,
                    min_tracking_confidence=0.5,
                )
            else:                                        # mediapipe 1.x
                model_file = _safe_model_path() if MODEL_PATH.is_file() else None
                if model_file is None:
                    self._error = f"缺模型文件 {MODEL_PATH}（下载见 MODEL_URL）"
                    return False
                from mediapipe.tasks import python as mp_python  # type: ignore
                from mediapipe.tasks.python import vision as mp_vision  # type: ignore

                self._mp_python, self._mp_vision = mp_python, mp_vision
                options = mp_vision.HandLandmarkerOptions(
                    base_options=mp_python.BaseOptions(model_asset_path=str(model_file)),
                    running_mode=mp_vision.RunningMode.IMAGE,
                    num_hands=self.max_hands,
                    min_hand_detection_confidence=0.5,
                    min_hand_presence_confidence=0.5,
                    min_tracking_confidence=0.5,
                )
                self._landmarker = mp_vision.HandLandmarker.create_from_options(options)
        except Exception as exc:            # noqa: BLE001
            self._error = f"初始化失败 {type(exc).__name__}: {exc}"
            return False
        return True

    def info(self) -> Dict[str, Any]:
        ok = self._load()
        engine = ("mp.solutions" if self._hands is not None
                  else "tasks.HandLandmarker" if self._landmarker is not None else "-")
        return {
            "name": self.name,
            "available": ok,
            "engine": engine,
            "model": str(MODEL_PATH) if MODEL_PATH.is_file() else "",
            "hint": "" if ok else (self._error or "pip install mediapipe opencv-python"),
        }

    # -- 识别 -------------------------------------------------------------
    def detect(self, frame) -> Dict[str, Any]:
        if not self._load():
            return {}
        cv2 = self._cv2
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self._landmarker is not None:
            import numpy as np
            image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                                   data=np.ascontiguousarray(rgb))
            result = self._landmarker.detect(image)
            groups = result.hand_landmarks or []
        else:
            result = self._hands.process(rgb)
            groups = [item.landmark for item in (result.multi_hand_landmarks or [])]
        hands: List[Dict[str, Any]] = []
        for group in groups[: self.max_hands]:
            pts = [(p.x, p.y) for p in group]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            box = [int(min(xs) * w), int(min(ys) * h),
                   int((max(xs) - min(xs)) * w), int((max(ys) - min(ys)) * h)]
            hands.append({
                "landmarks": pts,
                "bbox": box,
                "score": 1.0,
                "flex": flex_from_landmarks(pts),
            })
        return {"hands": hands, "width": w, "height": h}

    # -- 叠加 -------------------------------------------------------------
    def draw(self, frame, result: Dict[str, Any]):
        if not result or not result.get("hands"):
            return frame
        cv2 = self._cv2
        h, w = frame.shape[:2]
        for hand in result["hands"]:
            pts = [(int(x * w), int(y * h)) for x, y in hand.get("landmarks", [])]
            for a, b in BONES:
                if a < len(pts) and b < len(pts):
                    cv2.line(frame, pts[a], pts[b], (80, 220, 120), 2, cv2.LINE_AA)
            for p in pts:
                cv2.circle(frame, p, 3, (255, 255, 255), -1, cv2.LINE_AA)
            x, y, bw, bh = hand.get("bbox") or (0, 0, 0, 0)
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (80, 220, 120), 2)
            flex = hand.get("flex") or {}
            label = " ".join(f"{k}:{v:.2f}" for k, v in sorted(flex.items()))
            if label:
                cv2.putText(frame, label, (x, max(18, y - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 220, 120), 1, cv2.LINE_AA)
        return frame


def make_recognizer(mode: str = "hand") -> Any:
    """当前可用的识别器。

    mode="hand"  —— MediaPipe 人手关键点（人手专用，机械手认不出来）
    mode="robot" —— 模板锚点，定位机械手（要先 set_template）
    """
    if mode == "robot":
        return RobotHand()
    rec = HandLandmarks()
    info = rec.info()
    if info.get("available"):
        return rec
    return NullRecognizer(hint=info.get("hint") or "")


class RobotHand:
    """模板锚点：记住手掌那块像素，之后每帧在全图里找它。

    MediaPipe 是人手专用模型，机械手它一定认不出来（实测 0 个手）。
    这里换成「定位」：模板取手掌（刚性部分），手指怎么张合都不影响匹配，
    输出手掌的框、掌心坐标和置信度。

    用法：``set_template(frame, [x, y, w, h])`` 之后 ``detect(frame)`` 就行。
    """

    name = "palm-template"

    def __init__(self) -> None:
        self._tpl = None
        self._rect: Optional[Tuple[int, int, int, int]] = None
        self._cv2 = None
        self.last_score = 0.0
        self.misses = 0

    # -- 标定 -------------------------------------------------------------
    def set_template(self, frame, rect) -> Dict[str, Any]:
        import cv2  # type: ignore

        h_frame, w_frame = frame.shape[:2]
        x, y, w, h = (int(v) for v in rect)
        w = max(16, min(w, w_frame - 1))
        h = max(16, min(h, h_frame - 1))
        x = max(0, min(x, w_frame - w))
        y = max(0, min(y, h_frame - h))
        tpl = frame[y:y + h, x:x + w].copy()
        if tpl.size == 0:
            raise ValueError("模板区域为空")
        self._tpl, self._rect, self._cv2 = tpl, (x, y, w, h), cv2
        self.last_score, self.misses = 1.0, 0
        return {"ok": True, "rect": [x, y, w, h]}

    def reset(self) -> None:
        self._tpl, self._rect = None, None
        self.last_score, self.misses = 0.0, 0

    def info(self) -> Dict[str, Any]:
        ok = self._tpl is not None
        return {
            "name": self.name,
            "available": ok,
            "engine": "matchTemplate" if ok else "-",
            "rect": list(self._rect) if self._rect else None,
            "hint": "" if ok else "先在画面上点一下手掌（点「框住机械手」）",
        }

    # -- 识别 -------------------------------------------------------------
    def detect(self, frame, threshold: float = 0.45) -> Dict[str, Any]:
        if self._tpl is None:
            return {}
        cv2 = self._cv2
        h_frame, w_frame = frame.shape[:2]
        res = cv2.matchTemplate(frame, self._tpl, cv2.TM_CCOEFF_NORMED)
        _min_v, max_v, _min_loc, loc = cv2.minMaxLoc(res)
        score = float(max_v)
        self.last_score = score
        hands: List[Dict[str, Any]] = []
        if score >= threshold:
            self.misses = 0
            w, h = self._tpl.shape[1], self._tpl.shape[0]
            hands.append({
                "bbox": [int(loc[0]), int(loc[1]), int(w), int(h)],
                "center": [int(loc[0] + w / 2), int(loc[1] + h / 2)],
                "score": round(score, 3),
                "flex": {},
            })
        else:
            self.misses += 1
        return {"hands": hands, "width": w_frame, "height": h_frame,
                "score": round(score, 3), "found": bool(hands)}

    # -- 叠加 -------------------------------------------------------------
    def draw(self, frame, result: Dict[str, Any]):
        if not result:
            return frame
        cv2 = self._cv2
        if not result.get("hands"):
            cv2.putText(frame, f"no hand ({result.get('score', 0):.2f})", (12, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (60, 60, 220), 2, cv2.LINE_AA)
            return frame
        for hand in result["hands"]:
            x, y, w, h = hand["bbox"]
            cx, cy = hand["center"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (90, 220, 120), 2)
            cv2.drawMarker(frame, (cx, cy), (90, 220, 120), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(frame, f"hand {hand['score']:.2f}", (x, max(16, y - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (90, 220, 120), 2, cv2.LINE_AA)
        return frame
