"""安全环境模型：工作台 / 障碍物 / 人 / 保持距离，以及机械臂的碰撞代理。

这个模块是 ``/arm3d`` 三维页和 ``POST /arm/joints``（下发前的安全闸门）共用的
唯一一套几何。坐标一律用 **URDF 基座系**：Z 向上、原点在臂底座安装面、单位米。

三层东西：

1. ``CELL``：工位本身的描述（桌面、障碍物、人、禁入区）。页面拿 ``cell_dict()``
   画出来，服务端拿同一份做校验——不存在"画面里避开了、下发时没避开"。
2. ``PROXIES``：每段连杆的胶囊链（``static/nova/proxies.json``，由
   ``scripts/build_nova_assets.py`` 从厂家 STL 生成）。胶囊比三角面便宜三个数量级，
   而且外扩一点，做安全距离刚好是**保守**的。
3. ``clearance()`` / ``check_path()``：当前姿态和一条 MovJ 关节插值路径上的
   最小间距；``ssm_required()`` 是 ISO/TS 15066 的速度-间距监控（SSM）简化式。

安全的判定线（``SAFE_MM`` / ``WARN_MM``）是工程值，不是标准值：真机验收时按
现场风险评估改这两个数，别照抄。
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

from . import kin

Vec3 = Tuple[float, float, float]
Capsule = Tuple[Vec3, Vec3, float]

_HERE = Path(__file__).resolve().parent
PROXY_FILE = _HERE / "static" / "nova" / "proxies.json"

#: 最小安全间距（毫米）：低于它就拦，0 表示已经接触。
SAFE_MM = 30.0
#: 提醒线（毫米）：黄灯，允许但要在日志里留痕。
WARN_MM = 100.0

#: 桌面高度相对臂底座安装面（臂装在桌面上，所以桌面就是 z=0）
TABLE_TOP = 0.0
TABLE_SIZE = (1.40, 0.90, 0.04)          # 长 x 宽 x 厚
FLOOR_Z = -0.72                           # 地面（桌下）

#: 工位里的东西。box 用中心 + 尺寸；cylinder 用底面中心 + 半径/高；human 用胶囊。
CELL: Dict[str, Any] = {
    "table": {"size": list(TABLE_SIZE), "center": [0.0, 0.0, -TABLE_SIZE[2] / 2]},
    "floor_z": FLOOR_Z,
    "obstacles": [
        # 「料盒」已删除：真实工位上没有这个东西（现场核对过）。它原来在
        # (0.46,-0.26) 挡住过一条本来可行的路径，是凭空的障碍物。
        {"name": "相机立柱", "type": "cylinder", "radius": 0.032, "height": 0.42,
         "base": [-0.58, 0.34, 0.0]},
        {"name": "防护栏", "type": "box", "size": [1.70, 0.03, 1.65],
         "center": [0.0, 1.15, 0.10]},
    ],
    "human": {
        "name": "操作员",
        # 站在桌子右前方；躯干/肩/手臂/头/腿都是胶囊，做安全距离用同一套
        "capsules": [
            [[0.95, -1.05, FLOOR_Z + 0.76], [0.95, -1.05, FLOOR_Z + 1.27], 0.16],   # 躯干
            [[0.95, -1.05, FLOOR_Z + 1.27], [0.95, -1.05, FLOOR_Z + 1.34], 0.10],   # 颈
            [[0.95, -1.05, FLOOR_Z + 1.40], [0.95, -1.05, FLOOR_Z + 1.56], 0.11],   # 头
            [[0.80, -1.05, FLOOR_Z + 1.27], [1.10, -1.05, FLOOR_Z + 1.27], 0.09],   # 肩
            [[0.79, -1.05, FLOOR_Z + 1.24], [0.78, -1.05, FLOOR_Z + 0.92], 0.06],   # 左臂
            [[1.11, -1.05, FLOOR_Z + 1.24], [1.12, -1.05, FLOOR_Z + 0.92], 0.06],   # 右臂
            [[0.87, -1.05, FLOOR_Z + 0.02], [0.87, -1.05, FLOOR_Z + 0.78], 0.095],  # 腿
            [[1.03, -1.05, FLOOR_Z + 0.02], [1.03, -1.05, FLOOR_Z + 0.78], 0.095],
        ],
        "keepout_radius": 0.50,      # 以人为中心的安全柱半径
        "keepout_top": FLOOR_Z + 1.85,
    },
    #: 末端灵巧手的包络（tool_link 系，+Z 是工具方向）。
    #:
    #: 尺寸按厂家手部图（``pkg/LG-603/images/left_hand.jpg``）估的：手掌宽约
    #: 100 mm、厚约 55 mm，四指长约 75 mm、指粗约 18 mm，拇指从掌心侧面伸出。
    #: 以前是「一个大胖胶囊、指尖半径 38 mm」，抓盒子时模型会认为手指戳进了
    #: 桌面；现在把手掌和手指分开，指尖段细到 9 mm，才可能真正抓到 60 mm 的盒子。
    #: 仍然是**张开**手的包络（最坏情况），合手时实际更小。
    #:
    #: ⚠ 2026-11-18 实测修正：真实指尖最低点在法兰下方 247 mm（z=286 降 z=220
    #: 时在 z≈247 顶到桌面，量得「约 240」）。而下面这套胶囊最低点只有
    #: 0.185+0.009=0.194，比真实短 53 mm —— 于是模型报「离桌面还有 55 mm」时
    #: 指尖其实已经贴上了，臂因此顶到桌子。已把手指段拉长到最低点 0.247。
    #: 真实工具可能是两指夹爪（照片上是一片片宽扁指垫），宽度待实测后重画。
    "tool": {
        "name": "灵巧手",
        "capsules": [
            [[0.0, 0.0, 0.000], [0.0, 0.0, 0.030], 0.040],      # 腕
            [[0.0, 0.0, 0.030], [0.0, 0.0, 0.100], 0.050],      # 手掌
            [[0.0, 0.0, 0.100], [0.0, 0.0, 0.115], 0.034],      # 指根横梁
            [[-0.048, 0.0, 0.110], [-0.048, 0.0, 0.212], 0.009],  # 小指（最短）
            [[-0.016, 0.0, 0.110], [-0.016, 0.0, 0.228], 0.009],  # 无名指
            [[0.016, 0.0, 0.110], [0.016, 0.0, 0.238], 0.009],    # 中指（最长 = 最低点）
            [[0.048, 0.0, 0.110], [0.048, 0.0, 0.230], 0.009],    # 食指
            [[-0.055, 0.018, 0.095], [-0.068, 0.042, 0.213], 0.011],  # 拇指
        ],
    },
}

_DEFAULT_PROXIES: List[Dict[str, Any]] = []


def load_proxies(path: Path | None = None) -> List[Dict[str, Any]]:
    """读胶囊代理；文件不在就返回空列表（页面退化成只做显示，不静默造数据）。"""
    global _DEFAULT_PROXIES
    if _DEFAULT_PROXIES and path is None:
        return _DEFAULT_PROXIES
    target = path or PROXY_FILE
    if not target.is_file():
        return []
    data = json.loads(target.read_text(encoding="utf-8"))
    links = data.get("links", [])
    if path is None:
        _DEFAULT_PROXIES = links
    return links


# ---------------------------------------------------------------- 几何小工具

def _matvec(R: Sequence[Sequence[float]], v: Sequence[float]) -> Vec3:
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def _add(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a: Sequence[float]) -> float:
    return math.sqrt(_dot(a, a))


def _clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def seg_seg_distance(p1: Sequence[float], q1: Sequence[float],
                     p2: Sequence[float], q2: Sequence[float]) -> float:
    """两条线段的最近距离（经典的 Ericson 解法，含平行/退化分支）。"""
    d1 = _sub(q1, p1)
    d2 = _sub(q2, p2)
    r = _sub(p1, p2)
    a = _dot(d1, d1)
    e = _dot(d2, d2)
    f = _dot(d2, r)
    eps = 1e-12

    if a <= eps and e <= eps:
        return _norm(r)
    if a <= eps:
        s = 0.0
        t = _clamp01(f / e)
    else:
        c = _dot(d1, r)
        if e <= eps:
            t = 0.0
            s = _clamp01(-c / a)
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = _clamp01((b * f - c * e) / denom) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = _clamp01(-c / a)
            elif t > 1.0:
                t = 1.0
                s = _clamp01((b - c) / a)
    c1 = _add(p1, (d1[0] * s, d1[1] * s, d1[2] * s))
    c2 = _add(p2, (d2[0] * t, d2[1] * t, d2[2] * t))
    return _norm(_sub(c1, c2))


def capsule_distance(c1: Capsule, c2: Capsule) -> float:
    return seg_seg_distance(c1[0], c1[1], c2[0], c2[1]) - c1[2] - c2[2]


def point_aabb_distance(p: Sequence[float], center: Sequence[float],
                        size: Sequence[float]) -> float:
    dx = max(abs(p[0] - center[0]) - size[0] / 2, 0.0)
    dy = max(abs(p[1] - center[1]) - size[1] / 2, 0.0)
    dz = max(abs(p[2] - center[2]) - size[2] / 2, 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def capsule_box_distance(cap: Capsule, center: Sequence[float],
                         size: Sequence[float], samples: int = 5) -> float:
    """线段到 AABB：沿线采样取最小，再减半径。5 个点对 3 cm 的胶囊足够。"""
    a, b, r = cap
    best = 1e9
    for i in range(samples):
        t = i / (samples - 1)
        p = (a[0] + (b[0] - a[0]) * t,
             a[1] + (b[1] - a[1]) * t,
             a[2] + (b[2] - a[2]) * t)
        best = min(best, point_aabb_distance(p, center, size))
    return best - r


def capsule_plane_distance(cap: Capsule, z: float, above: bool = True) -> float:
    """线段到水平面 z=const 的距离；above=True 表示臂在面上方（下面算负）。"""
    a, b, r = cap
    if above:
        return min(a[2], b[2]) - z - r
    return z - max(a[2], b[2]) - r


# ---------------------------------------------------------------- 正解与代理

def link_frames(joints: Sequence[float]) -> List[Tuple[Vec3, List[List[float]]]]:
    """7 个连杆坐标系：base_link + Link1..Link6（**已含该关节转动**）。"""
    if len(joints) != 6:
        raise ValueError(f"需要 6 个关节角，收到 {len(joints)} 个")
    R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    p: Vec3 = (0.0, 0.0, 0.0)
    out: List[Tuple[Vec3, List[List[float]]]] = [(p, [row[:] for row in R])]
    for (xyz, rp), q in zip(kin.JOINT_ORIGINS, joints):
        p = _add(p, _matvec(R, xyz))
        R = kin._matmul(R, kin.rpy_to_matrix(*rp))
        R = kin._matmul(R, kin._rotz(math.radians(q)))
        out.append((p, [row[:] for row in R]))
    return out


def world_capsules(joints: Sequence[float],
                   proxies: Sequence[Dict[str, Any]] | None = None,
                   include_tool: bool = True) -> List[Dict[str, Any]]:
    """把每段连杆的胶囊变换到基座系，返回带 link/name 的列表。"""
    links = proxies if proxies is not None else load_proxies()
    frames = link_frames(joints)
    out: List[Dict[str, Any]] = []
    for i, link in enumerate(links):
        if i >= len(frames):
            break
        p, R = frames[i]
        for cap in link.get("capsules", []):
            a = _add(p, _matvec(R, cap[0:3]))
            b = _add(p, _matvec(R, cap[3:6]))
            out.append({"link": link.get("name", f"L{i}"), "index": i,
                        "a": a, "b": b, "r": float(cap[6])})
    if include_tool:
        p, R = frames[6]
        for cap in CELL.get("tool", {}).get("capsules", []):
            a = _add(p, _matvec(R, cap[0]))
            b = _add(p, _matvec(R, cap[1]))
            out.append({"link": CELL["tool"].get("name", "工具"), "index": 6,
                        "a": a, "b": b, "r": float(cap[2])})
    return out


def self_pairs(caps: Sequence[Dict[str, Any]]) -> Iterable[Tuple[Dict[str, Any], Dict[str, Any]]]:
    """自碰撞候选：只查隔了两段以上的连杆。

    相邻连杆本来就通过关节嵌套在一起（胶囊必然"重叠"），腕部 4/5/6 也是同轴
    套在一起的组件；把这些算进来，正常姿态下永远是负距离，指标就废了。
    真正需要盯的是「工具撞本体」「大臂撞小臂」这类跨段的挤压。
    """
    for i in range(len(caps)):
        for j in range(i + 1, len(caps)):
            if abs(caps[i]["index"] - caps[j]["index"]) <= 2:
                continue
            yield caps[i], caps[j]


def _obstacle_capsules(cell: Dict[str, Any]) -> List[Tuple[str, Capsule]]:
    """圆柱和人用胶囊表示；盒子在 ``clearance`` 里单独用点-盒距离算。"""
    out: List[Tuple[str, Capsule]] = []
    for ob in cell.get("obstacles", []):
        if ob["type"] == "cylinder":
            bx, by, bz = ob["base"]
            out.append((ob["name"],
                        ((bx, by, bz), (bx, by, bz + ob["height"]), ob["radius"])))
    human = cell.get("human") or {}
    for cap in human.get("capsules", []):
        out.append((human.get("name", "人"), (tuple(cap[0]), tuple(cap[1]), cap[2])))
    return out


def _obstacle_boxes(cell: Dict[str, Any]) -> List[Tuple[str, Tuple[float, float, float],
                                                          Tuple[float, float, float]]]:
    return [(ob["name"], tuple(ob["center"]), tuple(ob["size"]))
            for ob in cell.get("obstacles", []) if ob["type"] == "box"]


# ---------------------------------------------------------------- 间距计算

def clearance(joints: Sequence[float], cell: Dict[str, Any] | None = None,
              check_self: bool = True,
              safe_mm: float | None = None) -> Dict[str, Any]:
    """当前姿态的最小间距（米）和最接近的对象。

    ``safe_mm`` 不给就用模块默认 ``SAFE_MM``。抓取这种「已经规划好、速度很慢」
    的场合可以传更小的值——但那是**按不确定性和制动距离算**的，不是随手调。
    """
    cell = cell or CELL
    caps = world_capsules(joints)
    if not caps:
        return {"ok": False, "error": "没有碰撞代理（先跑 scripts/build_nova_assets.py）"}

    best = 1e9
    pair: Tuple[str, str] = ("", "")
    boxes = _obstacle_boxes(cell)
    for cap in caps:
        cap_t = (cap["a"], cap["b"], cap["r"])
        for name, oc in _obstacle_capsules(cell):
            d = capsule_distance(cap_t, oc)
            if d < best:
                best, pair = d, (cap["link"], name)
        for name, center, size in boxes:
            d = capsule_box_distance(cap_t, center, size)
            if d < best:
                best, pair = d, (cap["link"], name)
        # 桌面 / 地面：底座和肩柱本来就贴着安装面（胶囊还外扩了一圈），
        # 把它们算进来会得到一个恒定的 80 mm 假告警，掩盖真正的问题。
        if cap["link"] not in ("base_link", "Link1"):
            d = capsule_plane_distance(cap_t, TABLE_TOP)
            if d < best:
                best, pair = d, (cap["link"], "桌面")
            d = capsule_plane_distance(cap_t, cell["floor_z"])
            if d < best:
                best, pair = d, (cap["link"], "地面")

    if check_self:
        for c1, c2 in self_pairs(caps):
            d = capsule_distance((c1["a"], c1["b"], c1["r"]), (c2["a"], c2["b"], c2["r"]))
            if d < best:
                best, pair = d, (c1["link"], c2["link"])

    safe = (safe_mm if safe_mm is not None else SAFE_MM) / 1000.0
    return {"ok": True, "min_m": round(best, 4), "pair": list(pair),
            "safe": best >= safe, "warn": best < WARN_MM / 1000.0}


def check_path(start: Sequence[float], target: Sequence[float],
               cell: Dict[str, Any] | None = None, step_deg: float = 2.0,
               max_samples: int = 400,
               safe_mm: float | None = None) -> Dict[str, Any]:
    """关节空间直线插值（MovJ 的走法）扫一遍，返回整条路径的最小间距。"""
    cell = cell or CELL
    delta = [float(b) - float(a) for a, b in zip(start, target)]
    span = max(abs(v) for v in delta)
    n = max(1, min(max_samples, int(math.ceil(span / step_deg))))
    worst = {"min_m": 1e9, "pair": ["", ""], "index": 0}
    for i in range(n + 1):
        t = i / n
        q = [a + d * t for a, d in zip(start, delta)]
        c = clearance(q, cell, safe_mm=safe_mm)
        if not c.get("ok"):
            return c
        if c["min_m"] < worst["min_m"]:
            worst = {"min_m": c["min_m"], "pair": c["pair"], "index": i}
    safe = (safe_mm if safe_mm is not None else SAFE_MM) / 1000.0
    worst.update({
        "ok": True,
        "samples": n + 1,
        "safe": worst["min_m"] >= safe,
        "warn": worst["min_m"] < WARN_MM / 1000.0,
        "step_deg": step_deg,
    })
    return worst


def ssm_required(tcp_speed_m_s: float, react_s: float = 0.20,
                 brake_m_s2: float = 3.0, intrusion_m: float = 0.10) -> Dict[str, float]:
    """ISO/TS 15066 速度-间距监控的简化式。

    ``S = v·(t_反应) + v²/(2a) + C``。t/а/C 都是**这里假设的工程值**，
    真机要按风险评估和实测制动距离标定；返回值带上参数就是为了别说成标准值。
    """
    v = max(0.0, float(tcp_speed_m_s))
    distance = v * react_s + (v * v) / (2.0 * brake_m_s2) + intrusion_m
    return {"v_m_s": round(v, 3), "required_m": round(distance, 4),
            "react_s": react_s, "brake_m_s2": brake_m_s2, "intrusion_m": intrusion_m}


def cell_dict(cell: Dict[str, Any] | None = None) -> Dict[str, Any]:
    """给前端的完整工位描述：几何 + 代理 + 判定线。

    ``cell`` 可以传 ``scene.Scene.cell()``（静态工位 + 相机看到的物体），
    这样页面和安全校验拿到的就是同一份"现在的"环境，而不是出厂模型。
    """
    return {
        "cell": cell or CELL,
        "proxies": load_proxies(),
        "thresholds": {"safe_mm": SAFE_MM, "warn_mm": WARN_MM},
        "table_top": TABLE_TOP,
        "units": "m（基座系，Z 向上）",
    }
