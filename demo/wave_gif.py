"""把「招手」画成图和 GIF：食指反复勾动，其余手指收着。

    <bundled python> demo/wave_gif.py     # 需要 Pillow
"""
from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from handlink import skills  # noqa: E402
from render_poses import BG, DIM, PANEL, TXT, W, H, S, draw_hand, font  # noqa: E402

LINE = (44, 58, 94)


def frame(index_angle: float, title: str) -> Image.Image:
    angles = dict(skills.POSE_LIB["beckon_base"])
    angles[103] = index_angle
    img = Image.new("RGB", (W * S, H * S), PANEL)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([8 * S, 8 * S, (W - 8) * S, (H - 8) * S],
                        radius=18 * S, fill=PANEL, outline=LINE, width=2 * S)
    d.text((28 * S, 26 * S), title, font=font(23 * S), fill=TXT)
    draw_hand(d, angles)
    d.text((28 * S, (H - 54) * S), "其余四指收拢，只有食指在动", font=font(13 * S), fill=DIM)
    return img.resize((W, H), Image.LANCZOS)


def main() -> int:
    frames = []
    for angle in (0, 25, 50, 75, 100, 75, 50, 25):
        frames.append(frame(angle, f"招手 · 食指 {angle}°"))
    gif = HERE / "wave.gif"
    frames[0].save(gif, save_all=True, append_images=frames[1:],
                   duration=150, loop=0, disposal=2, optimize=True)

    strip_angles = (0, 40, 80, 100)
    strip = [frame(a, f"食指 {a}°") for a in strip_angles]
    out = Image.new("RGB", (W * len(strip), H), BG)
    for i, im in enumerate(strip):
        out.paste(im, (W * i, 0))
    png = HERE / "wave_strip.png"
    out.save(png)
    print(f"saved {gif} ({gif.stat().st_size // 1024} KB), {png} ({out.size[0]}x{out.size[1]})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
