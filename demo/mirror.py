"""摄像头手部追踪 → 灵巧手实时跟随（不用买数据手套的方案）。

官方方案是「数据手套 + MotionVenus」（见 README 第 5 节），零代码但要那副手套。
这里给的是替代路线：一个普通摄像头，MediaPipe 抓 21 个手部关键点，换算成 6 个
电机角度，按 20 Hz 通过 handlink 下发。

依赖（只这条路需要，在本机装不了，得在你电脑上装）::

    pip install mediapipe opencv-python

用法::

    python demo/mirror.py --test          # 不接摄像头，验证映射与平滑（现在就能跑）
    python demo/mirror.py                 # 开摄像头，内置仿真手
    python demo/mirror.py --server        # 开摄像头，推到正在跑的 handlink 服务
    python demo/mirror.py --dry           # 只打印角度，不发

局限（先看清楚再演示）：
* 这只手只有 6 个电机，人手 20+ 自由度，复现的是「每根手指弯多少 + 拇指收多少」，
  不是每个指节的独立动作，也不分左右张开（没有内收外展）。
* 手指正对镜头时 2D 关键点会被压缩，角度偏小——演示时手要侧一点，别正对。
* 拇指对掌（107）摄像头测不准，这里用 102 的 0.9 倍耦合，属于近似。
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import urllib.request
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from handlink import bus, service, spec  # noqa: E402

# MediaPipe 关键点：食指/中指/无名指/小指的 (根, 中节, 尖)
FINGER_TIPS = {103: (5, 6, 8), 104: (9, 10, 12), 105: (13, 14, 16), 106: (17, 18, 20)}
THUMB_TIP = (2, 3, 4)

SPEED = 100.0
TORQUE = 0.3
MAX_STEP = 25.0     # 每帧最多变 25°，防止抖动甩手
ALPHA = 0.45        # 指数平滑


def clamp01(v: float) -> float:
    return 0.0 if v < 0 else 1.0 if v > 1 else v


def angle_at(a, b, c) -> float:
    """b 处的夹角（度）。手指伸直 ≈ 180°，握紧 ≈ 30°。"""
    v1 = (a[0] - b[0], a[1] - b[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 180.0
    cos = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cos))


def flex_from_landmarks(lm) -> dict:
    """21 个关键点 → 每个电机的归一化弯曲量 0..1。"""
    out = {}
    for mid, (a, b, c) in FINGER_TIPS.items():
        out[mid] = clamp01((165.0 - angle_at(lm[a], lm[b], lm[c])) / 135.0)
    thumb = clamp01((170.0 - angle_at(lm[THUMB_TIP[0]], lm[THUMB_TIP[1]], lm[THUMB_TIP[2]])) / 140.0)
    out[102] = thumb
    out[107] = thumb * 0.9
    return out


class Mirror:
    """把 0..1 的弯曲量变成安全的电机指令（限速 + 平滑 + 夹紧）。"""

    def __init__(self, alpha: float = ALPHA, max_step: float = MAX_STEP,
                 speed: float = SPEED, torque: float = TORQUE) -> None:
        self.alpha = alpha
        self.max_step = max_step
        self.speed = speed
        self.torque = torque
        self.value = {mid: 0.0 for mid in spec.MOTOR_IDS}

    def update(self, flex: dict) -> spec.Pose:
        for mid, f in flex.items():
            m = spec.MOTOR_BY_ID[mid]
            target = clamp01(f) * m.pos_max
            cur = self.value[mid]
            target = cur + max(-self.max_step, min(self.max_step, target - cur))
            self.value[mid] = cur + self.alpha * (target - cur)
        return {mid: spec.Joint(round(v, 2), self.speed, self.torque)
                for mid, v in self.value.items()}


def send_http(base: str, pose: spec.Pose) -> None:
    body = json.dumps({
        "motors": {str(mid): {"angle": j.angle, "speed": j.speed, "torque": j.torque}
                   for mid, j in pose.items()},
        "reason": "mirror",
    }).encode()
    req = urllib.request.Request(base.rstrip("/") + "/pose", data=body, method="POST",
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=2) as resp:
            resp.read()
    except Exception as exc:  # 服务没起就只打印，不中断演示
        print(f"[warn] 下发失败: {exc}")


def run_test() -> int:
    mirror = Mirror()
    print("每档弯 25 帧（≈0.8 秒，30fps）后的稳态角度\n")
    print(f"{'弯曲量':<8}{'拇指1':<8}{'拇指2':<8}{'食指':<8}{'中指':<8}{'无名指':<8}小指")
    last = None
    for f in (0.0, 0.25, 0.5, 0.75, 1.0):
        pose = None
        for _ in range(25):
            pose = mirror.update({mid: f for mid in spec.MOTOR_IDS})
        row = "".join(f"{pose[mid].angle:<8.1f}" for mid in (102, 107, 103, 104, 105, 106))
        print(f"{f:<8.2f}{row}")
        if last:
            for mid in spec.MOTOR_IDS:
                assert pose[mid].angle >= last[mid].angle - 1e-6, f"{mid} 不单调"
        last = pose
    for mid, j in pose.items():
        assert 0.0 <= j.angle <= spec.MOTOR_BY_ID[mid].pos_max + 1e-6, f"{mid} 越界 {j.angle}"
    assert pose[103].angle > 175 and pose[102].angle > 95, "满弯曲没到位"
    print("\n映射/平滑/限速/夹紧 校验通过：五档单调递增，满弯曲贴住行程上限。")
    print("摄像头路径需要另装 mediapipe + opencv-python（本机沙箱装不了）。")
    return 0


def run_camera(args) -> int:
    try:
        import cv2  # type: ignore
        import mediapipe as mp  # type: ignore
    except ImportError as exc:
        print("缺少依赖:", exc)
        print("请先安装：pip install mediapipe opencv-python")
        print("或者用 --test 验证映射逻辑。")
        return 2

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"打不开摄像头 {args.camera}")
        return 2

    hands = mp.solutions.hands.Hands(
        max_num_hands=1, model_complexity=0,
        min_detection_confidence=0.6, min_tracking_confidence=0.5,
    )
    mirror = Mirror()
    app = None
    if not args.server and not args.dry:
        sim = bus.open_sim()
        app = service.App(sim)
    last_send = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if args.flip:
                frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(rgb)
            if result.multi_hand_landmarks:
                lm = [(p.x, p.y) for p in result.multi_hand_landmarks[0].landmark]
                flex = flex_from_landmarks(lm)
                pose = mirror.update(flex)
                now = time.time()
                if now - last_send > 0.05:  # 20 Hz
                    last_send = now
                    if args.dry:
                        print("  ".join(f"{spec.MOTOR_BY_ID[m].name}:{pose[m].angle:.0f}"
                                        for m in spec.MOTOR_IDS))
                    elif args.server:
                        send_http(args.server, pose)
                    elif app:
                        app.apply(pose, reason="mirror")
            cv2.imshow("handlink mirror  (q 退出)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        hands.close()
        if app:
            app.bus.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="摄像头手势 → 灵巧手跟随")
    ap.add_argument("--test", action="store_true", help="不接摄像头，验证映射")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--server", nargs="?", const="http://127.0.0.1:8766", default=None)
    ap.add_argument("--dry", action="store_true", help="只打印角度")
    ap.add_argument("--flip", action="store_true", default=True, help="镜像画面（默认开）")
    args = ap.parse_args()
    return run_test() if args.test else run_camera(args)


if __name__ == "__main__":
    raise SystemExit(main())
