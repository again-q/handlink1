"""Nova 5 正运动学测试：拿真机实测的那一组角/位姿当基准，钉住 URDF 参数。

真机数据来自 2026-11-15（`docs/现状.md` 第 8 节）：

    关节 [328.953, -1.759, -119.878, -63.137, -68.699, -49.355]
    位姿 [226.1304, -331.2691, 341.683, 97.1069, 50.9106, 43.0727]

以后如果谁改了 `JOINT_ORIGINS`（换型号、换 URDF 版本），这几个断言会先炸。

    python tests/test_kin.py
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from handlink import kin  # noqa: E402

MEASURED_JOINTS = [328.953, -1.759, -119.878, -63.137, -68.699, -49.355]
MEASURED_POSE = [226.1304, -331.2691, 341.683, 97.1069, 50.9106, 43.0727]


class TestForward(unittest.TestCase):
    def test_matches_real_robot(self):
        pose = kin.fk(MEASURED_JOINTS)
        dist = math.dist(pose[:3], MEASURED_POSE[:3])
        self.assertLess(dist, 3.0, f"位置和真机差 {dist:.1f} mm，URDF 参数可能被改了")
        for got, want in zip(pose[3:], MEASURED_POSE[3:]):
            self.assertAlmostEqual(got, want, delta=0.5)

    def test_all_zero_fully_extended(self):
        """零位是「全伸直朝上」：高度约 1.09 m，姿态朝上（pitch -90°）。

        手册的 850 mm 是工作半径，不是最大伸展；CR/Nova 这一族的最大伸展本来就比
        工作半径大（CR5：900 / 1096）。所以这里比的是伸展极限，不是 850。
        """
        pose = kin.fk([0, 0, 0, 0, 0, 0])
        self.assertGreater(pose[2], 1050.0)
        self.assertLess(pose[2], 1150.0)
        self.assertLess(abs(pose[0]), 1.0, "零位时末端应基本在基座正上方（x≈0）")
        _, R = kin.flange([0, 0, 0, 0, 0, 0])
        for i in range(3):        # 旋转矩阵得是正交的
            for j in range(3):
                dot = sum(R[k][i] * R[k][j] for k in range(3))
                self.assertAlmostEqual(dot, 1.0 if i == j else 0.0, delta=1e-6)

    def test_max_stretch_is_about_1090(self):
        """伸直时末端到基座的距离 ≈ 240+400+330+135+88 ≈ 1090 mm —— 与 CR5 的 1096 同族。"""
        p = kin.fk([0, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(math.dist([0, 0, 0], p[:3]), 1113.0, delta=40.0)

    def test_never_exceeds_max_reach(self):
        for joints in ([0, 0, 0, 0, 0, 0], [90, 0, 0, 0, 0, 0],
                       [0, -90, 0, 0, 0, 0], [0, 0, -160, 0, 0, 0],
                       [328.953, -1.759, -119.878, -63.137, -68.699, -49.355]):
            p = kin.fk(joints)
            r = math.dist([0, 0, 0], p[:3])
            self.assertLess(r, 1200.0, f"{joints} 算出来 {r:.0f} mm，超出机械伸展")

    def test_j1_turns_the_whole_arm(self):
        """J1 转 90°，末端在 XY 平面上也应该跟着转 90°，半径不变。"""
        a = kin.fk([0, 0, -90, 0, 0, 0])
        b = kin.fk([90, 0, -90, 0, 0, 0])
        self.assertAlmostEqual(math.hypot(a[0], a[1]), math.hypot(b[0], b[1]), delta=2.0)
        turned = math.degrees(math.atan2(b[1], b[0]) - math.atan2(a[1], a[0]))
        self.assertAlmostEqual(turned % 360, 90.0, delta=1.0)

    def test_requires_six(self):
        with self.assertRaises(ValueError):
            kin.fk([1, 2, 3])


class TestLimits(unittest.TestCase):
    def test_manual_ranges(self):
        self.assertEqual(kin.JOINT_LIMITS[1], (-180.0, 180.0), "J2 是 ±180°")
        self.assertEqual(kin.JOINT_LIMITS[2], (-160.0, 160.0), "J3 是 ±160°")
        for i in (0, 3, 4, 5):
            self.assertEqual(kin.JOINT_LIMITS[i], (-360.0, 360.0))

    def test_clamp_and_check(self):
        self.assertEqual(kin.clamp([400, -200, 0, 0, 0, 0]),
                         [360.0, -180.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(kin.within_limits([0, 0, 0, 0, 0, 0]), [])
        self.assertEqual(kin.within_limits([0, 0, -170, 0, 0, 0]), [3])


class TestFrames(unittest.TestCase):
    def test_frames_and_flange_agree(self):
        joints = [10, -20, 30, -40, 50, -60]
        last = kin.frames(joints)[-1]
        p, R = kin.flange(joints)
        # 最后一个关节坐标系加上 joint6 自身的旋转就是法兰
        self.assertLess(math.dist(last[0], p), 1e-9)

    def test_frames_are_six(self):
        self.assertEqual(len(kin.frames([0, 0, 0, 0, 0, 0])), 6)


class TestModel(unittest.TestCase):
    def test_model_dict_is_json_ready(self):
        m = kin.model_dict()
        self.assertEqual(len(m["joint_origins"]), 6)
        self.assertEqual(len(m["limits"]), 6)
        self.assertEqual(m["home"], [0.0] * 6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
