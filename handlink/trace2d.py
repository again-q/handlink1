"""二维手绘轨迹 → 机械臂关节序列（仿真优先，参数全部可调）。

这一层只回答一个问题：在纸面上画一条线，六个关节该怎么动。

四个转换
--------
1. 画布归一化坐标 (u, v) → 纸面毫米 (x, y)：canvas_to_paper
2. 指尖点 → 法兰位姿：减去 tip_offset，姿态恒为 (180, 0, rz)
   （工具轴竖直朝下，rz=0 时手指朝向固定）
3. 位姿 → 关节角：warm start 的数值反解（solve_path）
4. 画布上的折线 → 等弧长重采样（resample）

为什么不用 kin.ik()
-------------------
kin.ik() 一次会试 7 个种子再挑最优，单点约 300 ms；一条 200 点的线要一分钟。
轨迹的相邻点只差 1 mm，用上一点的解当种子一次迭代就收敛（实测 1.1 ms/点，
指尖误差 0.15 mm）。所以这里直接调 kin._ik_once。

参数为什么这么多
----------------
现在全套是仿真，纸面的真实位置、笔长、纸高都还没量过。所以 TraceConfig 里
凡是接真机后一定会改的量都单独列出来，改配置即可，不用动算法。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any, Dict, List, Optional, Sequence, Tuple

from . import kin

Point = Tuple[float, float]


@dataclass(frozen=True)
class TraceConfig:
    """一整套可调参数。默认值按可达性实测挑，接真机时逐项改。"""

    # ---- 指尖（工具系，毫米）------------------------------------------
    #: 中指指尖相对法兰的位置。工具朝下时基座系就是 (dx, dy, -238)。
    #: 来源：handlink/cell.py 的手部包络 [0.016, 0, 0.238]。
    tip_offset: Tuple[float, float, float] = (16.0, 0.0, -238.0)

    # ---- 工具姿态 ------------------------------------------------------
    #: 法兰 rpy 的 roll / pitch。实测 (180, 0) 时工具 Z 轴 = (0, 0, -1)，正朝下。
    tool_roll: float = 180.0
    tool_pitch: float = 0.0
    #: 绕竖直轴的朝向（度）。只转手指、不动指尖；0 最省 J6。
    tool_yaw: float = 0.0

    # ---- 纸面 ----------------------------------------------------------
    #: 指尖平面高度（毫米，基座系 z）。仿真先用 250，接真机后改成纸面高度。
    paper_z: float = 250.0

    # ---- 可达区域（毫米，基座系）--------------------------------------
    #: 实测「全部可解」的矩形：820x380 是极限，内收 40 mm 得到 740x300。
    region_x: Tuple[float, float] = (-320.0, 420.0)
    region_y: Tuple[float, float] = (-480.0, -180.0)
    #: 画布顶边对应纸面的哪一边。
    #: False（默认）= 顶边是**近端**（y 较大）。这是按 /arm3d 默认相机实测的：
    #: 把纸面四角投影到屏幕，y0=-480 落在屏幕下方、y1=-180 落在上方，
    #: 所以画布顶边必须对 y1，否则画布和三维视图上下相反。
    #: 站在桌子另一侧看纸面的话，这个值和 mirror_y 都可能要翻。
    canvas_top_is_far: bool = False
    #: 镜像。站在臂的哪一侧看纸面，左右上下会反过来；对不上就翻这里，
    #: 不用改任何算法。改的是"画布 → 纸面"这一步，轨迹和关节流不变。
    mirror_x: bool = False
    mirror_y: bool = False

    # ---- 采样与速度 ----------------------------------------------------
    #: 重采样步长（毫米）。原来的 5 mm 会把一条 300 mm 的线切成 60 段，每段都很短，
    #: 控制器的加减速来不及跑完就进了下一段，现场就是「又慢又卡」。
    #: 2026-11-25 真机对照（corner 轨迹，速度 40%）：
    #:
    #: * 10 mm + r=3：25~37 点，书写段仍有 2~6 次掉到 0，均速 137 mm/s
    #: * **15 mm + r=5：25 点，掉速 0 次，均速 167 mm/s**（当前默认）
    #:
    #: 关节跳变在 15 mm 下约 2.7°（红线 12°），几何余量仍然充足。
    max_step_mm: float = 15.0
    #: 指尖名义速度（毫米/秒），只用来估时长。
    tip_speed_mm_s: float = 40.0
    #: 单段关节跳变上限（度）：超了说明重采样太稀或过了奇异点。
    max_joint_step_deg: float = 12.0
    #: 拐角判定角（度）：输入折线里方向突变超过这个量的点，重采样后会**吸附**回去，
    #: 免得手绘的直角被等距采样斜切成一段弧。0 = 不吸附。
    #:
    #: 为什么要有：等距重采样会把采样点放在离拐角最多半个步长的地方，直角就被
    #: 削掉一个角（15 mm 步长时最多削 7.5 mm）。吸附只挪纵向位置、不改点数，
    #: 段长仍落在 [0.6, 1.5]×步长 内。
    corner_deg: float = 50.0
    #: 真机下发时的**队列窗口**：控制器里最多积压多少条 MovL 没走完。
    #: 一口气推几百条会塞满队列、机械臂在起点附近卡住（实测）；窗口就是「边执行
    #: 边下发」的节流上限。用算法队列 ID 判断（GetCurrentCommandID），不能用分批。
    batch_size: int = 40
    #: 中间点的平滑过渡比例 cp（0~100，手册「平滑过渡参数」）。0 = 每点停车（一抖一
    #: 抖）；30~50 一般够让臂带速穿过。最后一个点永远强制 cp=0，收笔才停得稳。
    smooth_cp: int = 30
    #: 拐角平滑半径（毫米，手册 MovL 的 ``r``，与 cp 互斥且 r 优先）。0 = 不用 r，
    #: 按 smooth_cp 走。
    #:
    #: 2026-11-25 用 30004 速度曲线实测（同一台机器、同一条 296 mm 直线、31 点）：
    #:
    #: * ``cp=30``：速度曲线是**等幅锯齿**——每个点都掉到 0 再爬起来，就是「一卡一卡」；
    #:   整段 12.1 s，运动中均速 69 mm/s。
    #: * ``r=3``：速度曲线是**一条连续曲线**，点与点之间没有掉速；整段 8.4 s，
    #:   运动中均速 106 mm/s。
    #:
    #: 也就是说这台固件对 ``cp`` 基本不做混合，``r`` 才是真正生效的那个。
    #: 配上 15 mm 点距时用 r=5：corner 轨迹实测掉速 0 次、均速 167 mm/s。
    #: 注意控制器只认**整数** r（``r=3`` 可以，``r=3.0`` 回 -50001），arm.movl_r 会取整。
    blend_r_mm: float = 5.0
    #: 每条 MovL 的加速度比例 ``a``（1~100，手册「加速度比例」）。0 = 不下发，用控制器
    #: 默认值。默认加速度偏低时，5 mm 的小段几乎全程在加减速，看起来又慢又卡。
    accel_pct: int = 0

    # ---- 归位（执行前的安全高位）--------------------------------------
    #: 执行轨迹前先走到这个关节角。选它是因为实测：法兰 z=927 mm，
    #: 自身最小间距 123 mm，从这里到任何起点的路径最小 41 mm（红线 30 mm）。
    home_joints: Tuple[float, ...] = (0.0, -15.0, -25.0, 0.0, 50.0, 0.0)
    #: 关掉就不归位，直接从当前姿态去起点。
    home_enabled: bool = False
    #: 「安全地走到起点」的抬升高度（毫米）：先竖直抬到「纸面 + 这个高度」，平移到起点
    #: 正上方，再竖直落下去。不绕固定归位位姿，省一次大动作，竖直段也不会横扫桌面。
    approach_lift_mm: float = 80.0
    #: 腕部预归中：下发前若 |J6| 超过 ``wrist_limit_deg``，先把它绕回 ±180° 内的等价角度。
    #: 不带这一步，J6 会一轮一轮累积到限位（2026-11-25 实测 -357.9° 报警）。
    #:
    #: **代价要说清楚**：等价角度差 360°，所以「绕回」本身是一次整圈腕部自转（实测约
    #: 15 s），并不是想象中零点几度的小动作。因此阈值必须卡在限位附近（300°，即离
    #: 限位 60° 才动手），把它当成「快撞限位了才做一次的大复位」，而不是每次都做。
    #: J6 每写一条 300 mm 的线约漂 50°，所以大约每 5~6 条笔画才触发一次。
    wrist_unwind: bool = True
    wrist_limit_deg: float = 300.0
    #: 灵巧手也归位到某个预设姿态（见 handlink/poses.py 的 POSES）。
    #: 手是夹着笔的，**整段书写过程中必须保持不变**，否则笔会被带偏，
    #: 指尖偏移也就对不上了。所以只在这里定一次，执行中不再动它。
    hand_home_enabled: bool = True
    hand_home_pose: str = "fist"

    # ---- 反解 ----------------------------------------------------------
    #: 首次反解的种子关节角（腕部大致朝下的构型）。
    seed_joints: Tuple[float, ...] = (68.0, 24.76, 69.88, 175.27, 89.98, -22.0)
    #: 单点 warm start 最大迭代次数。
    ik_iters: int = 60

    def as_dict(self) -> Dict[str, Any]:
        return {
            "tip_offset": [round(v, 3) for v in self.tip_offset],
            "tool_rpy": [self.tool_roll, self.tool_pitch, self.tool_yaw],
            "paper_z": self.paper_z,
            "region": {"x": list(self.region_x), "y": list(self.region_y)},
            "canvas_top_is_far": self.canvas_top_is_far,
            "mirror": {"x": bool(self.mirror_x), "y": bool(self.mirror_y)},
            "home": {"enabled": bool(self.home_enabled),
                     "joints": [round(v, 3) for v in self.home_joints]},
            "hand": {"enabled": bool(self.hand_home_enabled),
                     "pose": self.hand_home_pose},
            "max_step_mm": self.max_step_mm,
            "corner_deg": float(self.corner_deg),
            "tip_speed_mm_s": self.tip_speed_mm_s,
            "max_joint_step_deg": self.max_joint_step_deg,
            "batch_size": int(self.batch_size),
            "smooth_cp": int(self.smooth_cp),
            "blend_r_mm": float(self.blend_r_mm),
            "approach_lift_mm": float(self.approach_lift_mm),
            "wrist": {"unwind": bool(self.wrist_unwind),
                      "limit_deg": float(self.wrist_limit_deg)},
            "accel_pct": int(self.accel_pct),
            "region_width_mm": round(self.region_x[1] - self.region_x[0], 1),
            "region_depth_mm": round(self.region_y[1] - self.region_y[0], 1),
        }


DEFAULT = TraceConfig()

#: 环境变量，指向一份 JSON 覆盖默认配置（接真机时不用改代码）
ENV_CONFIG = "HANDLINK_TRACE_CONFIG"


def config_from_env(path: Optional[str] = None) -> TraceConfig:
    """读一份 JSON 覆盖 DEFAULT；文件不在或字段不认识就保持默认。"""
    import json
    import os

    target = path or os.environ.get(ENV_CONFIG)
    if not target:
        return DEFAULT
    try:
        with open(target, encoding="utf-8") as fh:
            raw = json.load(fh)
    except (OSError, ValueError):
        return DEFAULT
    if not isinstance(raw, dict):
        return DEFAULT
    known = {k: v for k, v in raw.items() if k in TraceConfig.__dataclass_fields__}
    try:
        return replace(DEFAULT, **{k: _coerce(k, v) for k, v in known.items()})
    except (TypeError, ValueError):
        return DEFAULT


def _coerce(key: str, value: Any) -> Any:
    """JSON 里的 list 要还原成 tuple，数值统一成 float。"""
    if key in ("region_x", "region_y"):
        return tuple(float(v) for v in value)
    if key in ("tip_offset", "seed_joints", "home_joints"):
        return tuple(float(v) for v in value)
    if key in ("canvas_top_is_far", "mirror_x", "mirror_y", "home_enabled",
               "hand_home_enabled", "wrist_unwind"):
        return bool(value)
    if key == "hand_home_pose":
        return str(value)
    if key in ("ik_iters", "batch_size", "smooth_cp", "accel_pct"):
        return int(value)
    if key in ("tool_roll", "tool_pitch", "tool_yaw", "paper_z", "max_step_mm",
               "tip_speed_mm_s", "max_joint_step_deg", "blend_r_mm",
               "approach_lift_mm", "wrist_limit_deg", "corner_deg"):
        return float(value)
    return value


# ---------------------------------------------------------------- 坐标映射

def canvas_to_paper(u: float, v: float, cfg: TraceConfig = DEFAULT) -> Point:
    """画布归一化坐标 (0~1) → 纸面毫米。

    u 从左到右，v 从上到下（浏览器画布的坐标系）。
    canvas_top_is_far=True 时，画布顶边是纸的远处。
    """
    x0, x1 = cfg.region_x
    y0, y1 = cfg.region_y
    u = min(1.0, max(0.0, float(u)))
    v = min(1.0, max(0.0, float(v)))
    if cfg.mirror_x:
        u = 1.0 - u
    if cfg.mirror_y:
        v = 1.0 - v
    x = x0 + (x1 - x0) * u
    # 画布顶边（v=0）对应 region_y 的远端：臂底座在 y=0，所以 y0（更负）才是远处。
    y = (y0 + (y1 - y0) * v) if cfg.canvas_top_is_far else (y1 - (y1 - y0) * v)
    return (x, y)


def paper_to_canvas(x: float, y: float, cfg: TraceConfig = DEFAULT) -> Point:
    """canvas_to_paper 的逆，用来把算出来的轨迹画回画布。"""
    x0, x1 = cfg.region_x
    y0, y1 = cfg.region_y
    u = (float(x) - x0) / (x1 - x0) if x1 != x0 else 0.0
    if cfg.canvas_top_is_far:
        v = (float(y) - y0) / (y1 - y0) if y1 != y0 else 0.0
    else:
        v = (y1 - float(y)) / (y1 - y0) if y1 != y0 else 0.0
    if cfg.mirror_x:
        u = 1.0 - u
    if cfg.mirror_y:
        v = 1.0 - v
    return (u, v)


def tip_to_flange(x: float, y: float, z: float,
                  cfg: TraceConfig = DEFAULT) -> List[float]:
    """指尖目标点（基座系毫米）→ 法兰位姿 [x, y, z, rx, ry, rz]。

    工具朝下时，指尖相对法兰的偏移在基座系里就是 tip_offset 原样。
    """
    dx, dy, dz = cfg.tip_offset
    return [float(x) - dx, float(y) - dy, float(z) - dz,
            cfg.tool_roll, cfg.tool_pitch, cfg.tool_yaw]


def tip_of_joints(joints: Sequence[float], cfg: TraceConfig = DEFAULT) -> List[float]:
    """关节角 → 指尖点在基座系里的位置（毫米）。仿真回放和校验都用它。"""
    p, _R = kin.flange(joints)
    dx, dy, dz = cfg.tip_offset
    return [p[0] * 1000.0 + dx, p[1] * 1000.0 + dy, p[2] * 1000.0 + dz]


# ---------------------------------------------------------------- 重采样

def resample(points: Sequence[Sequence[float]], step_mm: float,
             corner_deg: float = 0.0) -> List[Point]:
    """按弧长**等距**重采样，相邻点间距尽量等于 step_mm（且永不超过它）。

    旧版只保证「不超过 step」：手绘时鼠标点本身就比 step 密（拖慢一点就 3~4 mm
    一个点），于是输出全是 3~4 mm 的小段。段长一小，``r``（圆角半径）就放不下——
    控制器要求 ``r ≤ 段长/2``，放不下就回 ``-1``，服务把那个点退回 ``cp=0`` 停车。
    一条 48 点的圆因此变成 48 次急停，现场就是「画个圆直接卡住」。

    现在按总弧长 ``L`` 取 ``n = ceil(L / step)``，输出 ``n + 1`` 个严格等距的点，
    间距 ``L / n`` 落在 ``(约 0.9·step, step]``，圆角才有地方落。

    ``corner_deg > 0`` 时还会把「方向突变超过这个角度」的原始拐点**吸附**回来，
    免得手绘的直角被等距采样斜切成一小段弧（15 mm 步长最多削掉 7.5 mm 的角）。
    """
    pts = [(float(p[0]), float(p[1])) for p in points]
    if len(pts) < 2 or step_mm <= 0:
        return pts

    # 去掉零长度段，否则后面按弧长插值会除零
    clean: List[Point] = [pts[0]]
    for p in pts[1:]:
        if math.dist(p, clean[-1]) > 1e-9:
            clean.append(p)
    if len(clean) < 2:
        return clean

    cum: List[float] = [0.0]
    for i in range(1, len(clean)):
        cum.append(cum[-1] + math.dist(clean[i - 1], clean[i]))
    total = cum[-1]
    if total <= 1e-9:
        return [clean[0]]

    n = max(1, int(math.ceil(total / float(step_mm) - 1e-9)))
    out: List[Point] = []
    j = 0
    for k in range(n + 1):
        s = total * k / n
        while j < len(cum) - 2 and cum[j + 1] < s:
            j += 1
        seg = cum[j + 1] - cum[j]
        t = 0.0 if seg <= 1e-9 else (s - cum[j]) / seg
        t = min(1.0, max(0.0, t))
        a, b = clean[j], clean[j + 1]
        out.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))

    if corner_deg and corner_deg > 0:
        out = _snap_corners(out, clean, cum, total, n, step_mm, corner_deg)
    return out


def _turn_deg(a: Point, b: Point, c: Point) -> float:
    """在 b 点的方向突变角（度）：0 = 一条直线，90 = 直角，180 = 原路折返。"""
    v1 = (b[0] - a[0], b[1] - a[1])
    v2 = (c[0] - b[0], c[1] - b[1])
    n1 = math.hypot(*v1)
    n2 = math.hypot(*v2)
    if n1 < 1e-9 or n2 < 1e-9:
        return 0.0
    dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def _snap_corners(out: List[Point], clean: Sequence[Point], cum: Sequence[float],
                  total: float, n: int, step_mm: float,
                  corner_deg: float) -> List[Point]:
    """把原始折线上的拐点吸附到最近的等距采样点，保住直角。

    只挪采样点的位置、不增删点，而且挪完必须仍然「离前后邻居不太近也不太远」，
    否则宁可不吸附（保持均匀比保住一个角更重要）。
    """
    res = list(out)
    # 只挡住「会产生退化段」的情况。下限刻意放得比半个采样间隔还松：吸附必然让
    # 一段变长、一段变短（角被拉回来就是这么回事），短的那段如果容不下大圆角，
    # 下发的 radius_at() 会按实际段长自动降半径。**保住角**比保住均匀更重要——
    # 均匀只是速度好看，角削掉了写出来的字就是圆的、而且补不回来。
    lo = 0.45 * step_mm
    hi = 1.55 * step_mm
    for i in range(1, len(clean) - 1):
        if _turn_deg(clean[i - 1], clean[i], clean[i + 1]) < corner_deg:
            continue
        k = int(round(cum[i] / total * n))
        if k <= 0 or k >= n:
            continue
        p = clean[i]
        prev_p = res[k - 1]
        next_p = res[k + 1] if k + 1 < len(res) else clean[-1]
        if math.dist(p, prev_p) < lo or math.dist(p, next_p) < lo:
            continue
        if math.dist(p, prev_p) > hi or math.dist(p, next_p) > hi:
            continue
        res[k] = (p[0], p[1])
    return res


def path_length(points: Sequence[Sequence[float]]) -> float:
    return sum(math.dist(points[i], points[i + 1]) for i in range(len(points) - 1))


# ---------------------------------------------------------------- 反解

def solve_path(paper_points: Sequence[Sequence[float]], cfg: TraceConfig = DEFAULT,
               seed: Optional[Sequence[float]] = None) -> Dict[str, Any]:
    """指尖平面点串 → 关节序列（warm start 逐点递推）。

    解不出的点跳过但记序号，不会中断整条轨迹。
    """
    q = kin.clamp([float(v) for v in (seed if seed is not None else cfg.seed_joints)])
    joints: List[List[float]] = []
    failed: List[int] = []
    max_step = 0.0
    for i, p in enumerate(paper_points):
        pose = tip_to_flange(p[0], p[1], cfg.paper_z, cfg)
        sol = kin._ik_once(pose, q, iters=cfg.ik_iters)
        if sol is None:
            failed.append(i)
            continue
        if joints:
            max_step = max(max_step, max(abs(sol[k] - q[k]) for k in range(6)))
        q = sol
        joints.append(list(q))

    spans: Dict[str, float] = {}
    for k in range(6):
        col = [j[k] for j in joints]
        spans["J%d" % (k + 1)] = round(max(col) - min(col), 2) if col else 0.0
    j6_net = round(joints[-1][5] - joints[0][5], 2) if len(joints) > 1 else 0.0
    return {"joints": joints, "failed": failed,
            "max_step": round(max_step, 3), "spans": spans, "j6_net": j6_net}


# ---------------------------------------------------------------- 主入口

def trace_to_joints(points_uv: Sequence[Sequence[float]],
                    cfg: TraceConfig = DEFAULT,
                    seed: Optional[Sequence[float]] = None,
                    closed: bool = False) -> Dict[str, Any]:
    """画布点串（归一化 0~1）→ 完整计划。

    points_uv 是浏览器里画的折线，每项 [u, v]，取值 0~1。
    closed=True 时首尾相连（画圆圈用）。

    返回的 steps 结构与 arm_actions 一致（{"label", "joints"}），
    所以页面上的预览播放器可以直接复用。
    """
    raw = [[float(p[0]), float(p[1])] for p in points_uv
           if isinstance(p, (list, tuple)) and len(p) >= 2]
    if closed and len(raw) > 2:
        raw.append(list(raw[0]))
    if len(raw) < 2:
        return {"ok": False, "error": "至少要两个点才能连成线"}

    paper = [canvas_to_paper(u, v, cfg) for u, v in raw]
    x0, x1 = cfg.region_x
    y0, y1 = cfg.region_y
    outside = sum(1 for x, y in paper if not (x0 <= x <= x1 and y0 <= y <= y1))

    sampled = resample(paper, cfg.max_step_mm, corner_deg=cfg.corner_deg)
    solved = solve_path(sampled, cfg, seed=seed)

    joints = solved["joints"]
    if not joints:
        return {"ok": False, "error": "整条轨迹都反解失败：检查纸面高度和可达区域",
                "config": cfg.as_dict()}

    tip_path = [tip_of_joints(q, cfg) for q in joints]
    steps = [{"label": "轨迹 %d/%d" % (i + 1, len(joints)),
              "joints": [round(v, 3) for v in q]}
             for i, q in enumerate(joints)]

    length = path_length(sampled)
    duration = length / cfg.tip_speed_mm_s if cfg.tip_speed_mm_s > 0 else 0.0
    warnings: List[str] = []
    if outside:
        warnings.append("%d 个采样点在可达区外" % outside)
    if solved["failed"]:
        warnings.append("%d 个点反解失败，已跳过" % len(solved["failed"]))
    if solved["max_step"] > cfg.max_joint_step_deg:
        warnings.append("相邻点最大关节跳变 %.1f°，超过 %.0f°"
                        % (solved["max_step"], cfg.max_joint_step_deg))
    if abs(solved["j6_net"]) > 180:
        warnings.append("J6 净转 %.0f°，注意手腕线缆" % solved["j6_net"])

    return {
        "ok": True,
        "config": cfg.as_dict(),
        "paper_points": [[round(x, 2), round(y, 2)] for x, y in sampled],
        "canvas_points": [[round(u, 5), round(v, 5)]
                          for u, v in (paper_to_canvas(x, y, cfg) for x, y in sampled)],
        "steps": steps,
        "tip_path": [[round(v, 2) for v in p] for p in tip_path],
        "report": {
            "n_points": len(sampled),
            "n_joints": len(joints),
            "failed": len(solved["failed"]),
            "length_mm": round(length, 1),
            "duration_s": round(duration, 2),
            "spans_deg": solved["spans"],
            "j6_net_deg": solved["j6_net"],
            "max_joint_step_deg": solved["max_step"],
            "outside_region": outside,
        },
        "warnings": warnings,
    }
