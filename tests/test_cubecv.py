"""魔方视觉的单元测试：全部用合成图，不接相机、不接硬件。

合成图的画法刻意贴近真机：黑色骨架 + 贴纸（0.92 格宽）+ 轻微噪声与模糊，
贴纸按给定的层转角整体旋转。真机照片到了以后，先跑这个确认几何没退化，再调阈值。
"""
from __future__ import annotations

import math
import unittest
from pathlib import Path

from handlink import cubecv
from handlink.cubecv import DEFAULT_BGR, LETTERS

np = cubecv.np
cv2 = cubecv.cv2


def _rot_pts(points, cx, cy, deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    out = []
    for x, y in points:
        dx, dy = x - cx, y - cy
        out.append((int(round(cx + dx * c - dy * s)), int(round(cy + dx * s + dy * c))))
    return np.array(out, np.int32)


def _square(cx, cy, side, deg):
    h = side / 2.0
    return _rot_pts([(cx - h, cy - h), (cx + h, cy - h), (cx + h, cy + h), (cx - h, cy + h)],
                    cx, cy, deg)


def render(face, angle=0.0, center=(320, 240), side=171.0, bg=(205, 205, 208),
           noise=4.0, blur=1.1):
    """画一张「俯视魔方」：黑骨架 + 9 张贴纸，整面按 angle 旋转。"""
    img = np.full((480, 640, 3), bg, np.uint8)
    # 一点点桌面纹理，避免纯色太理想
    img[:, :, 0] = np.clip(img[:, :, 0].astype(np.int16) + np.random.randint(-4, 5, (480, 640)), 0, 255)
    cv2.fillConvexPoly(img, _square(center[0], center[1], side, angle), (30, 30, 32),
                       lineType=cv2.LINE_AA)
    cell = side / 3.0
    for idx, letter in enumerate(face):
        r, c = divmod(idx, 3)
        dx, dy = cubecv._rot((c - 1) * cell, (r - 1) * cell, angle)
        cx, cy = center[0] + dx, center[1] + dy
        cv2.fillConvexPoly(img, _square(cx, cy, cell * 0.92, angle), DEFAULT_BGR[letter],
                           lineType=cv2.LINE_AA)
    if blur:
        img = cv2.GaussianBlur(img, (3, 3), blur)
    if noise:
        img = np.clip(img.astype(np.int16) + np.random.normal(0, noise, img.shape), 0, 255).astype(np.uint8)
    return img


# 标定文件写到工作区的 .tmp 下：沙箱里 %TEMP% 不可写（chmod 会被拒）。
_TMP = Path(__file__).resolve().parent.parent / ".tmp" / "test-cubecv"


def _vision(name: str) -> cubecv.CubeVision:
    d = _TMP / name
    d.mkdir(parents=True, exist_ok=True)
    path = d / "vision.json"
    try:
        path.unlink()
    except OSError:
        pass
    return cubecv.CubeVision(path)


class TestGeometry(unittest.TestCase):
    def setUp(self):
        np.random.seed(7)
        self.vision = _vision(self._testMethodName)

    def test_solved_mixed_face(self):
        face = list("URFDLBURF")
        img = render(face)
        got = self.vision.analyze(img)
        self.assertTrue(got["found"], "合成图都没认出来")
        self.assertEqual(got["cells"], face)
        self.assertAlmostEqual(got["angle"], 0.0, delta=1.5)
        self.assertGreater(got["gap"], 0.5)
        # 四个角应该在真实正方形附近
        expected = _square(320, 240, 171.0, 0.0)
        for p, q in zip(got["quad"], expected):
            self.assertLess(math.dist(p, q), 6.0)

    def test_all_white_face_without_color(self):
        # 复原的白面：9 格全白，没有彩色贴纸可抓。靠黑色骨架兜底。
        img = render(list("UUUUUUUUU"))
        got = self.vision.analyze(img)
        self.assertTrue(got["found"], "全白面应该靠骨架兜底认出来")
        self.assertEqual(got["cells"], list("UUUUUUUUU"))

    def test_rotation_angles(self):
        face = list("URFDLBURF")
        for angle in (0, 8, 20, 33, 45, 60, 72, -25):
            img = render(face, angle=angle)
            got = self.vision.analyze(img)
            self.assertTrue(got["found"], f"{angle}° 没认出来")
            want = cubecv.wrap_angle(angle)
            # ±45° 是同一个正方形的两种写法，按 90° 周期比
            diff = (got["angle"] - want + 45.0) % 90.0 - 45.0
            self.assertLess(abs(diff), 2.5, f"{angle}°：读到 {got['angle']}，期望 {want}")
            # 转到一半时格子会压在缝隙上，颜色本就不该信；只有接近对齐（±10°）
            # 才要求 9 格全对。
            if abs(cubecv.wrap_angle(angle)) <= 10:
                self.assertEqual(got["cells"], face, f"{angle}° 格子顺序变了")

    def test_turned_layer_shifts_colors_in_body_frame(self):
        # 层转 90°：本体格子里换成了别的颜色，读数应该整体转一格，而不是「看起来没转」
        face = ["R"] + ["F"] * 8
        got = self.vision.analyze(render(face, angle=90))
        self.assertTrue(got["found"])
        self.assertEqual(got["cells"][2], "R")
        self.assertEqual(got["cells"].count("F"), 8)

    def test_reference_angle(self):
        img = render(list("URFDLBURF"), angle=10)
        self.assertTrue(self.vision.set_reference(img)["ok"])
        self.assertAlmostEqual(self.vision.ref_angle, 10.0, delta=1.5)
        got = self.vision.analyze(render(list("URFDLBURF"), angle=35))
        self.assertTrue(got["found"])
        self.assertAlmostEqual(got["angle"], 25.0, delta=2.5)

    def test_quarter_turn_relabels_cells(self):
        face = ["R"] + ["F"] * 8
        got = self.vision.analyze(render(face, angle=90))
        self.assertTrue(got["found"])
        # 顺时针转 90°：左上角跑到右上角
        self.assertEqual(got["cells"][2], "R")
        self.assertEqual(got["cells"].count("F"), 8)

    def test_background_is_not_a_cube(self):
        img = np.full((480, 640, 3), (205, 205, 208), np.uint8)
        got = self.vision.analyze(img)
        self.assertFalse(got["found"])
        # 桌面 + 一张白纸，也不能当魔方
        cv2.rectangle(img, (120, 100), (520, 380), (250, 250, 250), -1)
        self.assertFalse(self.vision.analyze(img)["found"])

    def test_manual_quad(self):
        face = list("URFDLBURF")
        img = render(face, angle=0)
        quad = _square(320, 240, 171.0, 0).tolist()
        self.assertTrue(self.vision.set_manual(quad)["ok"])
        got = self.vision.analyze(img)
        self.assertTrue(got["found"])
        self.assertTrue(got["manual"])
        self.assertEqual(got["cells"], face)


class TestPalette(unittest.TestCase):
    def setUp(self):
        np.random.seed(11)
        self.vision = _vision("palette-" + self._testMethodName)

    def test_learn_six_faces(self):
        for letter in LETTERS:
            img = render([letter] * 9, angle=0)
            res = self.vision.learn(letter, img)
            self.assertTrue(res["ok"], f"{letter}: {res}")
            self.assertIn(letter, self.vision.palette.learned)
        # 学完之后，混色面必须 9 格全对
        got = self.vision.analyze(render(list("URFDLBURF")))
        self.assertEqual(got["cells"], list("URFDLBURF"))
        # 而且存盘了
        again = cubecv.CubeVision(self.vision.path)
        self.assertEqual(sorted(again.palette.learned), sorted(LETTERS))

    def test_colors_are_separable(self):
        pal = cubecv.Palette.default()
        labs = {k: cubecv.bgr_to_lab(v) for k, v in DEFAULT_BGR.items()}
        for letter, lab in labs.items():
            got, d = pal.classify(lab)
            self.assertEqual(got, letter)
            self.assertLess(d, 1e-6)
        # 六色两两可分：最近的一对也要有 ΔE > 30
        pairs = [(cubecv.delta_e(a, b), x, y) for i, (x, a) in enumerate(labs.items())
                 for y, b in list(labs.items())[i + 1:]]
        pairs.sort()
        self.assertGreater(pairs[0][0], 30.0, pairs[0])


if __name__ == "__main__":
    unittest.main()
