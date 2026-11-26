"""Nova 5 正运动学：由六个关节角算出末端位姿。

参数不是猜的，来自越疆官方 URDF（``nova5_2f85_description/urdf/nova5_2f85.urdf``，
joint1~joint6 的 ``origin``）。用 2026-11-15 真机上实测的一组角/位姿校过：

    关节 [328.953, -1.759, -119.878, -63.137, -68.699, -49.355]
    实测 [226.130, -331.269, 341.683, 97.107, 50.911, 43.073]

本模块算出 **[226.8, -331.5, 340.3], 97.1/50.9/43.1** —— 位置差 1.6 mm、姿态差 0.1°。
差的那 1.6 mm 是 URDF 与实机的微小几何差异，对这个量级的可视化无所谓；
要更准就得在真机上多采几组点做最小二乘标定（见 ``docs/现状.md`` 待办）。

约定：角度用**度**，长度用**毫米**输出；内部按米算，和 URDF 一致。
"""
from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

#: 每个关节相对父关节的固定变换 ``(xyz 米, rpy 弧度)``，抄自 URDF 的 joint origin
JOINT_ORIGINS: Tuple[Tuple[Tuple[float, float, float], Tuple[float, float, float]], ...] = (
    ((0.0, 0.0, 0.240000000000178), (0.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (-1.57080287682252, 1.53586622836832, 3.14159265358979)),
    ((-0.399756009268664, -0.0139690033141953, 0.0), (0.0, 0.0, 0.0)),
    ((-0.329798707647103, -0.0115244277211674, 0.134999532858734),
     (0.0, 0.0, -1.53586622840712)),
    ((0.0, -0.12, 0.0), (1.5708, 0.0, 0.0)),
    ((0.0, 0.088328, 0.0), (-1.5708, 0.0, 0.0)),
)

#: 关节限位（度），手册附录 A：J1/J4/J5/J6 ±360，J2 ±180，J3 ±160
JOINT_LIMITS: Tuple[Tuple[float, float], ...] = (
    (-360.0, 360.0),
    (-180.0, 180.0),
    (-160.0, 160.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
    (-360.0, 360.0),
)

#: 关节最大速度（度/秒），手册：Nova 5 六轴都是 100°/s
JOINT_MAX_SPEED = 100.0

#: 工作半径（毫米）
REACH_MM = 850.0


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """URDF 的 rpy：``R = Rz(yaw) · Ry(pitch) · Rx(roll)``（固定轴，先 roll）。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def matrix_to_rpy(R: Sequence[Sequence[float]]) -> Tuple[float, float, float]:
    """``rpy_to_matrix`` 的逆，返回弧度。接近万向锁时 roll/yaw 会退化，够用。"""
    sp = max(-1.0, min(1.0, -R[2][0]))
    pitch = math.asin(sp)
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return roll, pitch, yaw


def _rotz(angle: float) -> List[List[float]]:
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mulvec(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def frames(joints: Sequence[float]) -> List[Tuple[List[float], List[List[float]]]]:
    """算每个关节坐标系在基座系里的位置与朝向（米）。

    返回 6 个 ``(原点, 旋转矩阵)``：第 i 个是关节 i 的坐标系，
    也就是「画出第 i 段连杆」时的起点。
    """
    if len(joints) != 6:
        raise ValueError(f"需要 6 个关节角，收到 {len(joints)} 个")
    R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    p = [0.0, 0.0, 0.0]
    out: List[Tuple[List[float], List[List[float]]]] = []
    for (xyz, rp), q in zip(JOINT_ORIGINS, joints):
        p = [p[i] + _mulvec(R, xyz)[i] for i in range(3)]
        R = _matmul(R, rpy_to_matrix(*rp))
        out.append((list(p), [row[:] for row in R]))     # 关节轴坐标系（转动前）
        R = _matmul(R, _rotz(math.radians(q)))
    return out


def flange(joints: Sequence[float]) -> Tuple[List[float], List[List[float]]]:
    """末端法兰（URDF 的 tool_link，与 joint6 同原点）在基座系里的位姿，单位米。"""
    if len(joints) != 6:
        raise ValueError(f"需要 6 个关节角，收到 {len(joints)} 个")
    R = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    p = [0.0, 0.0, 0.0]
    for (xyz, rp), q in zip(JOINT_ORIGINS, joints):
        p = [p[i] + _mulvec(R, xyz)[i] for i in range(3)]
        R = _matmul(R, rpy_to_matrix(*rp))
        R = _matmul(R, _rotz(math.radians(q)))
    return p, R


def fk(joints: Sequence[float]) -> List[float]:
    """关节角（度）→ 末端位姿 ``[x, y, z, rx, ry, rz]``，毫米 + 度，与 ``GetPose()`` 同序。"""
    p, R = flange(joints)
    roll, pitch, yaw = matrix_to_rpy(R)
    return [p[0] * 1000.0, p[1] * 1000.0, p[2] * 1000.0,
            math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def clamp(joints: Sequence[float]) -> List[float]:
    """按手册限位夹一遍，返回新列表。"""
    return [max(lo, min(hi, float(q))) for q, (lo, hi) in zip(joints, JOINT_LIMITS)]


def within_limits(joints: Sequence[float], tol: float = 0.0) -> List[int]:
    """返回越界的关节序号（1 起），空列表表示都在范围内。"""
    return [i + 1 for i, (q, (lo, hi)) in enumerate(zip(joints, JOINT_LIMITS))
            if q < lo - tol or q > hi + tol]


def joints_of_pose(*args):  # pragma: no cover - 占位，避免误用
    raise NotImplementedError("反解还没做：先只做可视化，IK 等真机在手边时再上")


# ================================================================ 数值反解
#
# 控制器自己有 ``InverseKin``，但那是真机在线用的；离线规划/安全检查需要在
# 电脑上先把「目标位姿」变成「关节角」，否则 ``cell.check_path()`` 无从采样。
# 这里用阻尼最小二乘（DLS）迭代，雅可比用数值差分——6 个变量，够快也够稳。

def _rotvec(R: Sequence[Sequence[float]]) -> List[float]:
    """旋转矩阵 → 轴角向量（弧度）。"""
    tr = R[0][0] + R[1][1] + R[2][2]
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    ang = math.acos(c)
    s = math.sin(ang)
    if ang < 1e-9 or abs(s) < 1e-9:
        return [0.0, 0.0, 0.0]
    return [ang * (R[2][1] - R[1][2]) / (2 * s),
            ang * (R[0][2] - R[2][0]) / (2 * s),
            ang * (R[1][0] - R[0][1]) / (2 * s)]


def _pose_error(pose: Sequence[float], joints: Sequence[float]) -> List[float]:
    """位姿误差：前三个是毫米、后三个是度（基座系）。"""
    p, R = flange(joints)
    target = rpy_to_matrix(math.radians(pose[3]), math.radians(pose[4]),
                           math.radians(pose[5]))
    # R_err = R_target · Rᵀ，轴角就是「基座系里还差多少」
    Rt = [[sum(target[i][k] * R[j][k] for k in range(3)) for j in range(3)]
          for i in range(3)]
    return [float(pose[0]) - p[0] * 1000.0, float(pose[1]) - p[1] * 1000.0,
            float(pose[2]) - p[2] * 1000.0] + [math.degrees(v) for v in _rotvec(Rt)]


def _solve6(J: Sequence[Sequence[float]], e: Sequence[float],
            damping: float = 0.05) -> List[float]:
    """解 (JᵀJ + λI)·dq = Jᵀe，高斯消元（6×6，不用 numpy）。

    ``J`` 的行是「关节」，列是「误差分量」：``J[j][k] = ∂e_k/∂q_j``。
    """
    A = [[sum(J[i][k] * J[j][k] for k in range(6)) + (damping if i == j else 0.0)
          for j in range(6)] for i in range(6)]
    b = [-sum(J[i][k] * e[k] for k in range(6)) for i in range(6)]
    M = [A[i][:] + [b[i]] for i in range(6)]
    for c in range(6):
        p = max(range(c, 6), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-12:
            return []
        for r in range(6):
            if r != c:
                f = M[r][c] / M[c][c]
                for k in range(c, 7):
                    M[r][k] -= f * M[c][k]
    return [M[i][6] / M[i][i] for i in range(6)]


def _jacobian(joints: Sequence[float]) -> List[List[float]]:
    """几何雅可比：行是关节，列是误差分量（毫米/度、度/度）。

    数值差分每步要 7 次 FK，这里用关节轴直接写：位置列 ``z × (p − p_i)``，
    旋转列就是转轴 ``z``。快一个量级，也没有步长误差。
    """
    fr = frames(joints)
    p, _R = flange(joints)
    k = math.pi / 180.0
    out: List[List[float]] = []
    for p_i, R_i in fr:
        z = [R_i[0][2], R_i[1][2], R_i[2][2]]
        rx, ry, rz = p[0] - p_i[0], p[1] - p_i[1], p[2] - p_i[2]
        out.append([-(z[1] * rz - z[2] * ry) * 1000.0 * k,
                    -(z[2] * rx - z[0] * rz) * 1000.0 * k,
                    -(z[0] * ry - z[1] * rx) * 1000.0 * k,
                    -z[0], -z[1], -z[2]])
    return out


def _ik_once(pose: Sequence[float], seed: Sequence[float],
             iters: int = 90) -> Optional[List[float]]:
    q = clamp([float(v) for v in seed])
    best = float("inf")
    stall = 0
    for _ in range(iters):
        e = _pose_error(pose, q)
        if max(abs(v) for v in e[:3]) < 0.5 and max(abs(v) for v in e[3:]) < 0.5:
            return q
        norm = sum(abs(v) for v in e)
        if norm < best - 1e-4:
            best, stall = norm, 0
        else:
            stall += 1
            if stall > 12:                # 卡在限位/奇异点上了，换个种子
                return None
        dq = _solve6(_jacobian(q), e)
        if not dq:
            return None
        m = max(abs(v) for v in dq)
        if m > 3.0:                       # 限制单步（度），别一步跨到另一个支管
            dq = [v * 3.0 / m for v in dq]
        q = clamp([q[i] + dq[i] for i in range(6)])
    return None


def ik(pose: Sequence[float], seed: Optional[Sequence[float]] = None,
       iters: int = 300) -> Optional[List[float]]:
    """目标位姿（毫米/度，法兰系）→ 关节角（度）；解不出返回 ``None``。

    多试几个种子，取「离种子最近」的那个解：同一个位姿常有多个解（腕部翻转），
    选关节走位最小的，路径才不会突然甩一下。
    """
    seeds: List[Sequence[float]] = []
    if seed is not None:
        seeds.append(seed)
    seeds += [[0.0, -30.0, 60.0, 0.0, 30.0, 0.0],
              [0.0, -60.0, 120.0, 0.0, 60.0, 0.0],
              # 「工具朝下、伸手去够」的几个种子：腕部 J4/J5 在 90° 附近，
              # 这是桌面抓取最常见的支管，数值反解从这个角度出发最稳。
              [180.0, 10.0, 100.0, 150.0, 90.0, 120.0],
              [210.0, 10.0, 100.0, 150.0, 90.0, 120.0],
              [150.0, 10.0, 100.0, 150.0, 90.0, 120.0],
              [120.0, 20.0, 100.0, 140.0, 90.0, 120.0]]
    best: Optional[Tuple[float, List[float]]] = None
    for s in seeds:
        q = _ik_once(pose, s, iters)
        if q is None:
            continue
        cost = sum(abs(q[i] - float(s[i])) for i in range(6))
        if max(abs(q[i] - float(s[i])) for i in range(6)) > 110.0:
            continue                      # 离种子太远：多半是另一个支管，别用
        if best is None or cost < best[0]:
            best = (cost, q)
    return best[1] if best else None


def model_dict() -> dict:
    """给前端用的模型常量：关节原点、限位、每段连杆的粗细。"""
    return {
        "joint_origins": [{"xyz": list(xyz), "rpy": list(rp)} for xyz, rp in JOINT_ORIGINS],
        "limits": [[lo, hi] for lo, hi in JOINT_LIMITS],
        "max_speed": JOINT_MAX_SPEED,
        "reach_mm": REACH_MM,
        "names": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "home": [0.0] * 6,
    }
