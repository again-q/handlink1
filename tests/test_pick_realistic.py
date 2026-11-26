"""抓取流程在「会翻车」的环境里还能不能跑通：HandSim + SimArm，真时序。

tests/test_arm 里那套用 FakeHand（状态直接等于目标），测的是流程编排；
这里手会卡电流、臂要走时间，测的是**整条链路到底会不会掉**。

注意：HandSim 的所有推进都在 step() 里（这是它可复现的来源），而 pick.run
是照真机写成「发一条、等一会儿、再读状态」的。所以这里加一层
RealtimeHand：后台线程按墙钟喂 step()，pick.run 一行都不用改。
"""
from __future__ import annotations

import threading
import time
import unittest

from handlink import arm, handsim, kin, pick, poses, spec


class RealtimeHand:
    """把虚拟时钟的 HandSim 挂到墙钟上（测试专用，不进 handlink 包）。"""

    def __init__(self, sim, dt: float = 0.002) -> None:
        self.sim = sim
        self.dt = dt
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self._stop.is_set():
            self.sim.step(self.dt)
            time.sleep(self.dt)

    def start(self):
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def __getattr__(self, name):
        return getattr(self.sim, name)


def apply_fist(sim, torque: float) -> None:
    """按指定电流上限合手，然后等它走完（真机上这一步约 2 s）。"""
    pose = {mid: spec.Joint(poses.POSES["fist"][mid], 200, torque)
            for mid in poses.POSES["fist"]}
    sim.apply(pose, reason="close:fist")


class TestPickOnRealisticHardware(unittest.TestCase):
    def setUp(self):
        self._grace = pick.CLENCH_GRACE
        pick.CLENCH_GRACE = 0.6
        self.sim = handsim.HandSim(seed=3)
        self.sim.open()
        self.sim.attach()
        self.hand = RealtimeHand(self.sim).start()
        self.arm = arm.SimArm(joints=[0.0, -25.0, -40.0, 0.0, 65.0, 0.0])
        self.arm.enable()

    def tearDown(self):
        pick.CLENCH_GRACE = self._grace
        self.hand.stop()
        self.sim.detach()
        self.sim.close()

    def test_full_pick_runs_on_the_realistic_pair(self):
        result = pick.run(self.arm, self.hand,
                           pick.PickPlan(force=1.5, hold=0.0, timeout=10.0),
                           log=lambda *_: None)
        self.assertTrue(result["ok"], result)
        names = [s["step"] for s in result["steps"]]
        self.assertEqual(names[0], "手张开")
        self.assertIn("抬升", names)

    def test_arm_moves_take_simulated_time_not_instantly(self):
        """臂不能瞬移：每段移动都要有非零时长。"""
        pick.run(self.arm, self.hand,
                 pick.PickPlan(force=1.5, hold=0.0), log=lambda *_: None)
        moves = [t for t in self.arm.trace if t["event"] in ("MovJ", "MovL")]
        self.assertGreaterEqual(len(moves), 5)
        for move in moves:
            self.assertGreater(move["eta_s"], 0.0)

    def test_pose_and_joints_stay_consistent_through_the_whole_pick(self):
        """真机上 GetPose 和 GetAngle 是同一台臂；仿真里也必须自洽。"""
        pick.run(self.arm, self.hand,
                 pick.PickPlan(force=1.5, hold=0.0), log=lambda *_: None)
        self.assertAlmostEqual(kin.fk(self.arm.joints())[2], self.arm.pose()[2], delta=3.0)

    def test_power_loss_mid_grasp_is_detectable(self):
        """抓着东西的时候手断电：位置会塌，下游必须看得出「没抓住」。"""
        apply_fist(self.sim, 1.5)
        time.sleep(3.0)
        self.assertTrue(pick.clenched(self.hand, poses.POSES["fist"]))
        self.sim.inject("power_off")
        time.sleep(2.5)
        self.assertFalse(pick.clenched(self.hand, poses.POSES["fist"]))


class TestKnownInterlockHole(unittest.TestCase):
    """**已发现的真实缺陷**：电流不足卡在半途时，pick.clenched 仍然判「合上了」。

    真机实测（docs/现状.md 第 2 节）：多指同动 0.6 A 会触发 I-LIM 卡在 118°，
    而 CLENCH_RATIO=0.6 对 140° 的目标只需要 ≥84°，118° 轻松过线。
    也就是说：手其实卡住了，互锁却放行，臂会把一只半开的手抬起来。

    这两条用例把现状钉住：模拟器的行为是**对的**（它复现了真机），
    错的是判据。等 pick.clenched 改成「看 I-Lim 状态位 / 看最后一段是否还在动」，
    把 test_stall_would_be_caught_by_a_motion_check 打开，再把上一条删掉。
    """

    def setUp(self):
        self.sim = handsim.HandSim(seed=3)
        self.sim.open()
        self.sim.attach()

    def tearDown(self):
        self.sim.detach()
        self.sim.close()

    def _run(self, torque: float, seconds: float = 3.0) -> dict:
        apply_fist(self.sim, torque)
        for _ in range(int(seconds / 0.002)):
            self.sim.step(0.002)
        return self.sim.state()

    def test_stall_at_118_passes_the_ratio_interlock(self):
        state = self._run(0.6)
        self.assertAlmostEqual(state[103].angle, 118.6, delta=3.0)
        self.assertLess(state[103].angle, poses.POSES["fist"][103] * 0.95)
        self.assertTrue(pick.clenched(self.sim, poses.POSES["fist"]),
                        "现判据会放过卡在 118 度的手——这正是要修的地方")

    def test_a_motion_check_would_catch_it(self):
        """替代判据的可行性：卡住时「最近一段没位移」，正常合手时还在动。"""
        apply_fist(self.sim, 0.6)
        for _ in range(int(4.0 / 0.002)):          # 先让它彻底卡死（不再爬）
            self.sim.step(0.002)
        stuck = self.sim.state()[103].angle
        for _ in range(int(1.0 / 0.002)):
            self.sim.step(0.002)
        self.assertAlmostEqual(self.sim.state()[103].angle, stuck, delta=0.5)

        apply_fist(self.sim, 1.5)
        for _ in range(int(0.2 / 0.002)):
            self.sim.step(0.002)
        early = self.sim.state()[103].angle
        for _ in range(int(0.3 / 0.002)):
            self.sim.step(0.002)
        self.assertGreater(self.sim.state()[103].angle - early, 5.0)


if __name__ == "__main__":
    unittest.main()