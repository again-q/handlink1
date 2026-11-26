"""魔方视觉的离线/在线命令行：不开网页也能取一帧、读一次、学配色。

用法
----
    python scripts/vision_check.py --list                    # 有哪些相机能出图
    python scripts/vision_check.py --open 0 --seconds 2      # 取一帧、读一次、存标注图
    python scripts/vision_check.py --image shot.png           # 读一张现成的图
    python scripts/vision_check.py --learn U --image u.png    # 把这一面学进调色板
    python scripts/vision_check.py --set-ref --image aligned.png
    python scripts/vision_check.py --self-test                # 合成图自检（不接相机）

默认把标注图写到 .tmp/frames/vision-<时间>.png，读数以 JSON 打到标准输出。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from handlink import camera, cubecv  # noqa: E402

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:  # noqa: BLE001
        pass

OUT_DIR = ROOT / ".tmp" / "frames"


def _load_image(path: str):
    frame = cubecv.cv2.imread(path)
    if frame is None:
        raise SystemExit(f"读不了这张图: {path}")
    return frame


def _grab(index: int, backend: str, seconds: float, size):
    cfg = camera.CameraConfig(index=index, backend=backend,
                              width=size[0], height=size[1])
    cam = camera.Camera(cfg)
    print(f"打开相机 {backend} #{index} …", file=sys.stderr)
    info = cam.open()
    print(f"  → {info.get('used')} {cam.width}x{cam.height}", file=sys.stderr)
    deadline = time.time() + max(1.0, seconds)
    frame = None
    while time.time() < deadline:
        frame = cam.read_raw()
        if frame is not None:
            break
        time.sleep(0.05)
    time.sleep(min(0.5, seconds / 2))       # 让自动曝光稳一下
    frame = cam.read_raw()
    cam.close()
    if frame is None:
        raise SystemExit("相机没有出图")
    return frame


def _summary(reading: dict) -> dict:
    keys = ("found", "cells", "angle", "center", "cell", "gap", "good", "score",
            "source", "ms")
    return {k: reading.get(k) for k in keys if k in reading}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="魔方视觉：取帧、读数、学配色")
    ap.add_argument("--list", action="store_true", help="列出能出图的相机")
    ap.add_argument("--open", type=int, metavar="INDEX", help="按 OpenCV 编号开相机")
    ap.add_argument("--backend", default="msmf", choices=("msmf", "dshow", "any"))
    ap.add_argument("--seconds", type=float, default=2.0, help="取帧前等多久（曝光稳定）")
    ap.add_argument("--size", default="640x480", help="分辨 宽x高")
    ap.add_argument("--image", help="直接读一张图")
    ap.add_argument("--learn", metavar="FACE", help="把这一面（U/R/F/D/L/B）学进调色板")
    ap.add_argument("--set-ref", action="store_true", help="把当前朝向设为托盘基准")
    ap.add_argument("--reset", action="store_true", help="清空调色板与基准")
    ap.add_argument("--out", help="标注图输出路径（默认 .tmp/frames/vision-*.png）")
    ap.add_argument("--self-test", action="store_true", help="合成图自检，不接硬件")
    ap.add_argument("--json", action="store_true", help="只输出 JSON")
    args = ap.parse_args(argv)

    if args.list:
        for d in camera.probe_devices(force=True):
            print(f"{d['backend']} #{d['index']}  {d['width']}x{d['height']}  "
                  f"{d['measured_fps']} fps  {'有画面' if d['frame'] else '无画面'}")
        return 0

    vision = cubecv.shared()

    if args.reset:
        vision.reset()
        print("已清空调色板与基准")
        return 0

    if args.self_test:
        return _self_test(vision)

    if args.image:
        frame = _load_image(args.image)
    elif args.open is not None:
        w, h = (int(v) for v in args.size.lower().split("x"))
        frame = _grab(args.open, args.backend, args.seconds, (w, h))
    else:
        ap.print_help()
        return 2

    if args.learn:
        result = vision.learn(args.learn.upper(), frame)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("ok") else 1
    if args.set_ref:
        result = vision.set_reference(frame)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result.get("ok") else 1

    reading = vision.analyze(frame)
    out = Path(args.out) if args.out else OUT_DIR / f"vision-{time.strftime('%H%M%S')}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(vision.annotated_png(frame, analyze_frame=False))
    if args.json:
        print(json.dumps(reading, ensure_ascii=False, indent=2))
    else:
        print(json.dumps(_summary(reading), ensure_ascii=False, indent=2))
        print(f"标注图: {out}", file=sys.stderr)
    return 0 if reading.get("found") else 1


def _self_test(vision) -> int:
    """合成一张「俯视魔方」，确认几何/颜色/角度这条链没坏。"""
    import math

    sys.path.insert(0, str(ROOT))
    from tests.test_cubecv import render           # 复用测试里的合成图

    def rotated(pattern, deg):
        """整层顺时针转 deg 度后，本体格子里应该看到的图案。"""
        out = list(pattern)
        for _ in range((int(deg) // 90) % 4):
            out = [out[(2 - c) * 3 + r] for r in range(3) for c in range(3)]
        return out

    face = list("URFDLBURF")
    bad = 0
    for angle in (0, 15, 45, -30, 90):
        reading = vision.analyze(render(face, angle=angle))
        got = reading.get("angle")
        diff = 0.0 if got is None else (got - cubecv.wrap_angle(angle) + 45.0) % 90.0 - 45.0
        ok = bool(reading.get("found")) and abs(diff) < 3.0
        cells = reading.get("cells") or []
        if angle % 90 == 0:
            # 整 90° 的层：颜色在本体格子里换位，必须逐格对上
            ok = ok and cells == rotated(face, angle)
        if not ok:
            bad += 1
        print(f"  {angle:>4}°  found={reading.get('found')}  angle={got}  "
              f"cells={''.join(cells)}  {'ok' if ok else 'FAIL'}")
    print("自检通过" if bad == 0 else f"自检失败 {bad} 项")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
