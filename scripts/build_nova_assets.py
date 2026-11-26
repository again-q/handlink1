"""从厂家 STL 生成碰撞代理（胶囊链），供 /arm3d 的安全距离计算用。

用法::

    python scripts/build_nova_assets.py

输出 ``handlink/static/nova/proxies.json``：每段连杆一组胶囊
``[ax, ay, az, bx, by, bz, r]``（米，连杆自身坐标系，和 URDF 的 Link 系一致）。

为什么是胶囊：机械臂的碰撞检测不需要精确三角面求交，需要的是**保守而且快**。
每段连杆沿长轴切成薄片，取每片点的质心连成骨架、半径取 90 分位，得到一串
把整段连杆包住的胶囊。距离全是解析解（点-线段），一帧几万个胶囊对也不卡。

只依赖标准库：跑一次生成静态 JSON，运行时不再需要 numpy。
"""
from __future__ import annotations

import json
import math
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
NOVA = ROOT / "handlink" / "static" / "nova"
LINKS = ["base_link", "Link1", "Link2", "Link3", "Link4", "Link5", "Link6"]


def load_stl(path: Path) -> list[tuple[float, float, float]]:
    """读二进制 STL，返回去重后的顶点列表（米）。"""
    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"{path} 不是二进制 STL")
    n = struct.unpack_from("<I", raw, 80)[0]
    verts: list[tuple[float, float, float]] = []
    off = 84
    seen: set[tuple[int, int, int]] = set()
    for _ in range(n):
        off += 12                                   # 法向
        for _v in range(3):
            x, y, z = struct.unpack_from("<fff", raw, off)
            off += 12
            key = (round(x * 1e5), round(y * 1e5), round(z * 1e5))
            if key in seen:
                continue
            seen.add(key)
            verts.append((x, y, z))
        off += 2                                    # 属性字节
    return verts


def percentile(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    i = min(len(s) - 1, max(0, int(round(q / 100.0 * (len(s) - 1)))))
    return s[i]


def capsule_chain(verts: list[tuple[float, float, float]],
                  bin_mm: float = 22.0, percentile_q: float = 90.0) -> list[list[float]]:
    """把一段连杆压成一串胶囊。长轴取包围盒最长边，足够稳。"""
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    ext = [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)]
    ax = max(range(3), key=lambda i: ext[i])

    t = [v[ax] for v in verts]
    lo, hi = min(t), max(t)
    span = hi - lo
    if span < 1e-6:
        c = [sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)]
        return [[*c, *c, 0.05]]

    n_bins = max(1, int(math.ceil(span / (bin_mm / 1000.0))))
    step = span / n_bins
    bins: list[list[tuple[float, float, float]]] = [[] for _ in range(n_bins)]
    for v in verts:
        i = min(n_bins - 1, int((v[ax] - lo) / step))
        bins[i].append(v)

    slices: list[tuple[list[float], float]] = []
    for pts in bins:
        if len(pts) < 8:
            continue
        c = [sum(p[k] for p in pts) / len(pts) for k in range(3)]
        radial = []
        for p in pts:
            d = [p[k] - c[k] for k in range(3)]
            d[ax] = 0.0                       # 只算到轴的径向距离
            radial.append(math.sqrt(sum(x * x for x in d)))
        r = percentile(radial, percentile_q) * 1.02
        slices.append((c, r))

    if not slices:
        c = [sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs)]
        return [[*c, *c, 0.05]]

    caps: list[list[float]] = []
    for i, (a, ra) in enumerate(slices):
        if i + 1 < len(slices):
            b, rb = slices[i + 1]
            r = max(ra, rb)
        else:
            b, r = a, ra                      # 最后一片退化成球
        caps.append([round(float(v), 4) for v in (*a, *b, r)])

    merged: list[list[float]] = []
    for cap in caps:
        if merged:
            prev = merged[-1]
            if (math.dist(prev[0:3], cap[0:3]) < 1e-4
                    and math.dist(prev[3:6], cap[3:6]) < 2e-3):
                prev[6] = max(prev[6], cap[6])
                continue
        merged.append(cap)
    return merged


def main() -> None:
    out = {"links": [], "meta": {
        "source": "Dobot Nova 5 URDF meshes (nova5_2f85_description)",
        "method": "longest-axis slabs -> capsule chain, radius = P90 radial",
        "units": "m, link frame",
        "generated_by": "scripts/build_nova_assets.py",
    }}
    total = 0
    for name in LINKS:
        verts = load_stl(NOVA / f"{name}.STL")
        caps = capsule_chain(verts)
        total += len(caps)
        rmax = max(c[6] for c in caps)
        print(f"{name:10s} verts={len(verts):6d} capsules={len(caps):2d} rmax={rmax*1000:3.0f} mm")
        out["links"].append({"name": name, "capsules": caps})
    (NOVA / "proxies.json").write_text(json.dumps(out, ensure_ascii=False, indent=1),
                                       encoding="utf-8")
    print(f"wrote {NOVA / 'proxies.json'}  ({total} capsules)")


if __name__ == "__main__":
    main()
