"""把连接器里的真实姿态画成图：张开 / 石头 / 剪刀 / 半握。

姿态不是硬编码的：先通过 SimBus 下发、等电机到位，再读回实际角度来画。

    <bundled python> demo/render_poses.py
"""
from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from handlink import bus, spec, store  # noqa: E402

S = 2  # 超采样倍数
W, H = 400, 470
BG = (11, 16, 32)
PANEL = (20, 28, 51)
LINE = (44, 58, 94)
PALM = (27, 36, 64)
FINGER = (59, 79, 133)
TIP = (96, 120, 190)
TXT = (230, 236, 255)
DIM = (141, 155, 189)
ACCENT = (96, 165, 250)

PKG = HERE.parent / "pkg" / "LG-603"
NAMES = {102: "拇指1", 107: "拇指2", 103: "食指", 104: "中指", 105: "无名指", 106: "小指"}
ORDER = [102, 107, 103, 104, 105, 106]

# 手指几何：基准点（掌顶边缘）、方向（-90 = 朝上）、两节长度、粗细
FINGERS = {
    103: dict(x=147, y=232, base=-90, l1=62, l2=42, w=22),   # 食指
    104: dict(x=188, y=228, base=-90, l1=70, l2=46, w=22),   # 中指
    105: dict(x=229, y=232, base=-90, l1=62, l2=42, w=22),   # 无名指
    106: dict(x=266, y=240, base=-90, l1=50, l2=34, w=20),   # 小指
}


def font(size: int):
    for name in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\msyhbd.ttc", r"C:\Windows\Fonts\simhei.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def rot(x: float, y: float, deg: float, length: float):
    import math

    a = math.radians(deg)
    return x + length * math.cos(a), y + length * math.sin(a)


def draw_hand(d: ImageDraw.ImageDraw, angles: dict) -> None:
    d.rounded_rectangle([118 * S, 230 * S, 286 * S, 352 * S], radius=34 * S, fill=PALM, outline=LINE, width=2 * S)
    d.rounded_rectangle([132 * S, 352 * S, 272 * S, 368 * S], radius=9 * S, fill=(22, 31, 56), outline=LINE, width=2 * S)

    for mid, g in FINGERS.items():
        flex = min(1.0, max(0.0, angles.get(mid, 0.0) / spec.MOTOR_BY_ID[mid].pos_max))
        a1 = g["base"] + flex * 72
        a2 = a1 + flex * 76
        l1 = g["l1"] * (1 - 0.18 * flex)
        l2 = g["l2"] * (1 - 0.22 * flex)
        x1, y1 = g["x"], g["y"]
        x2, y2 = rot(x1, y1, a1, l1)
        x3, y3 = rot(x2, y2, a2, l2)
        for (xa, ya, xb, yb) in ((x1, y1, x2, y2), (x2, y2, x3, y3)):
            d.line([xa * S, ya * S, xb * S, yb * S], fill=FINGER, width=g["w"] * S)
        d.ellipse([x3 * S - g["w"] * 0.5 * S, y3 * S - g["w"] * 0.5 * S,
                   x3 * S + g["w"] * 0.5 * S, y3 * S + g["w"] * 0.5 * S], fill=TIP)

    # 拇指：从掌根左侧伸出，弯曲时收向掌心
    f1 = min(1.0, max(0.0, angles.get(102, 0.0) / 100.0))
    tx, ty = 122, 300
    a1 = 232 + f1 * 46
    a2 = a1 - 14
    x2, y2 = rot(tx, ty, a1, 56)
    x3, y3 = rot(x2, y2, a2, 34)
    for (xa, ya, xb, yb) in ((tx, ty, x2, y2), (x2, y2, x3, y3)):
        d.line([xa * S, ya * S, xb * S, yb * S], fill=FINGER, width=24 * S)
    d.ellipse([x3 * S - 11 * S, y3 * S - 11 * S, x3 * S + 11 * S, y3 * S + 11 * S], fill=TIP)

    # 指根关节
    for g in FINGERS.values():
        d.ellipse([g["x"] * S - 6 * S, g["y"] * S - 6 * S, g["x"] * S + 6 * S, g["y"] * S + 6 * S], fill=(38, 50, 84))


def panel(pose: dict, title: str, note: str) -> Image.Image:
    img = Image.new("RGB", (W * S, H * S), PANEL)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([8 * S, 8 * S, (W - 8) * S, (H - 8) * S], radius=18 * S, fill=PANEL, outline=LINE, width=2 * S)
    f_title = font(26 * S)
    f_small = font(15 * S)
    f_tiny = font(13 * S)
    d.text((28 * S, 24 * S), title, font=f_title, fill=TXT)
    d.text((28 * S, 58 * S), note, font=f_tiny, fill=ACCENT)
    draw_hand(d, pose)
    line1 = "  ".join(f"{NAMES[m]}{pose.get(m, 0):.0f}°" for m in (102, 107, 103))
    line2 = "  ".join(f"{NAMES[m]}{pose.get(m, 0):.0f}°" for m in (104, 105, 106))
    d.text((28 * S, (H - 76) * S), line1, font=f_tiny, fill=DIM)
    d.text((28 * S, (H - 52) * S), line2, font=f_tiny, fill=DIM)
    return img.resize((W, H), Image.LANCZOS)


def settle_pose(target: dict) -> dict:
    sim = bus.open_sim()
    try:
        pose = {mid: spec.Joint(a, 360, 0.5) for mid, a in target.items()}
        sim.apply(pose)
        sim.settle(timeout=4.0)
        return {mid: st.angle for mid, st in sim.state().items()}
    finally:
        sim.close()


def main() -> int:
    poses = []
    poses.append(("张开", "全部 0°", settle_pose({mid: 0 for mid in ORDER})))
    if (PKG / "points" / "rock.txt").is_file():
        for name, title in (("rock", "石头"), ("scissors", "剪刀")):
            p = store.load_point(PKG / "points" / f"{name}.txt")
            poses.append((title, f"points/{name}.txt", settle_pose({mid: j.angle for mid, j in p.items()})))
    poses.append(("半握", "自定义 /pose", settle_pose({102: 60, 107: 20, 103: 90, 104: 100, 105: 110, 106: 120})))

    panels = [panel(p, t, n) for t, n, p in poses]
    header_h = 84
    out = Image.new("RGB", (W * len(panels), H + header_h), BG)
    d = ImageDraw.Draw(out)
    d.text((28, 20), "handlink · LG-603 姿态（仿真总线，角度由连接器读回）", font=font(26), fill=TXT)
    d.text((28, 52), "6 主动自由度：拇指1=102  拇指2=107  食指=103  中指=104  无名指=105  小指=106",
           font=font(14), fill=DIM)
    for i, p in enumerate(panels):
        out.paste(p, (W * i, header_h))

    target = HERE / "poses.png"
    out.save(target)
    print(f"saved {target}  {out.size[0]}x{out.size[1]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
