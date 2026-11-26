# -*- coding: utf-8 -*-
"""arm3d 界面重构示意图：用 Pillow 直接画，不依赖浏览器。

输出：
  docs/arm3d-redesign-ui.png   重构后的界面示意图（带 5 处结构标注）
  docs/arm3d-redesign-ia.png   信息架构 before / after 对照

跑法：
  python scripts/arm3d_mockup.py
"""
from __future__ import annotations

import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs"

# ---------------------------------------------------------------- 调色板（与 armview.py 一致）
BG = "#0a0f1c"
PANEL = "#121a2d"
CARD = "#0f1728"
LINE = "#233150"
TXT = "#e8eeff"
DIM = "#8fa0c4"
DIM2 = "#64748b"
OK = "#4ade80"
WARN = "#fbbf24"
ERR = "#f87171"
ACCENT = "#5b9bff"
BLUE = "#3d72c4"
DEEP = "#080d18"
PURPLE = "#7c5cff"

MSYH = "C:/Windows/Fonts/msyh.ttc"
MSYHBD = "C:/Windows/Fonts/msyhbd.ttc"
CONS = "C:/Windows/Fonts/consola.ttf"

_cache: dict = {}


def font(size: int, bold: bool = False, mono: bool = False):
    key = (size, bold, mono)
    if key in _cache:
        return _cache[key]
    if mono:
        for p in (CONS, "C:/Windows/Fonts/cour.ttf", MSYH):
            try:
                f = ImageFont.truetype(p, size)
                break
            except OSError:
                continue
        else:
            f = ImageFont.load_default()
    else:
        try:
            f = ImageFont.truetype(MSYHBD if bold else MSYH, size)
        except OSError:
            f = ImageFont.load_default()
    _cache[key] = f
    return f


def tw(d: ImageDraw.ImageDraw, s: str, size: int, bold=False, mono=False) -> float:
    return d.textlength(s, font=font(size, bold, mono))


def text(d, xy, s, size=13, color=TXT, bold=False, mono=False, anchor="la"):
    # Consolas 没有中文字形，混排会出豆腐块：含非 ASCII 就退回 MSYH。
    if mono and not s.isascii():
        mono = False
    d.text(xy, s, font=font(size, bold, mono), fill=color, anchor=anchor)


def chev(d, cx, cy, on=False, color="#5f6d8f"):
    """折叠箭头：自己画三角形，不依赖字体里的 ▶。on=展开 → 朝下。"""
    if on:
        d.polygon([(cx - 4.5, cy - 2.5), (cx + 4.5, cy - 2.5), (cx, cy + 4)], fill=color)
    else:
        d.polygon([(cx - 2.5, cy - 4.5), (cx + 4, cy), (cx - 2.5, cy + 4.5)], fill=color)


def rr(d, box, r=10, fill=None, outline=None, w=1):
    d.rounded_rectangle(box, radius=r, fill=fill, outline=outline, width=w)


def card(d, box, r=12, fill=CARD, outline=LINE):
    rr(d, box, r, fill=fill, outline=outline)


def chip(d, x, y, label, color=DIM, bg=None, size=12, pad=9, h=23, dot=None, border=None):
    """胶囊标签，返回右边界 x。"""
    wdt = tw(d, label, size) + pad * 2 + (14 if dot else 0)
    bd = border or (color + "55" if color != DIM else LINE)
    rr(d, (x, y, x + wdt, y + h), h // 2, fill=bg, outline=bd)
    tx = x + pad
    if dot:
        cy = y + h // 2
        d.ellipse((tx, cy - 3.5, tx + 7, cy + 3.5), fill=dot)
        tx += 13
    text(d, (tx, y + h // 2 + 0.5), label, size, color, anchor="lm")
    return x + wdt


def btn(d, x, y, label, on=False, danger=False, size=12, pad=11, h=27, dimmed=False, w=None):
    wdt = w if w is not None else tw(d, label, size) + pad * 2
    if danger:
        fill, bd, fg = "#3a1620", "#7f1d1d", "#fecaca"
    elif on:
        fill, bd, fg = "#1d4ed8", "#2563eb", "#ffffff"
    else:
        fill, bd, fg = "#1a2540", LINE, TXT
    if dimmed:
        fill, bd, fg = "#141d31", LINE, "#4d5f7d"
    rr(d, (x, y, x + wdt, y + h), 8, fill=fill, outline=bd)
    text(d, (x + wdt / 2, y + h / 2 + 0.5), label, size, fg, anchor="mm")
    return x + wdt


def kv(d, x, y, wdt, k, v, vcolor=TXT, size=12):
    text(d, (x, y), k, size, DIM)
    text(d, (x + wdt, y), v, size, vcolor, mono=True, anchor="ra")


def badge_num(d, cx, cy, n, r=11):
    d.ellipse((cx - r, cy - r, cx + r, cy + r), fill="#1d4ed8", outline="#60a5fa", width=1)
    text(d, (cx, cy + 0.5), str(n), 12, "#ffffff", bold=True, anchor="mm")


def icon(d, cx, cy, kind, color=TXT, s=1.0):
    """左栏图标：全部手绘，避免字体缺字出豆腐块。"""
    c = color
    w = max(1, int(round(1.5 * s)))
    if kind == "trace":                      # 一支笔
        d.line((cx - 6 * s, cy + 6 * s, cx + 5 * s, cy - 5 * s), fill=c, width=w + 1)
        d.polygon([(cx + 4 * s, cy - 7 * s), (cx + 7 * s, cy - 4 * s),
                   (cx + 5.5 * s, cy - 2.5 * s), (cx + 2.5 * s, cy - 5.5 * s)], fill=c)
        d.line((cx - 6 * s, cy + 6 * s, cx - 8 * s, cy + 8 * s), fill=c, width=1)
    elif kind == "pose":                     # 一面旗
        d.line((cx - 4 * s, cy + 8 * s, cx - 4 * s, cy - 8 * s), fill=c, width=w)
        d.polygon([(cx - 3 * s, cy - 8 * s), (cx + 7 * s, cy - 5 * s),
                   (cx - 3 * s, cy - 1 * s)], fill=c)
    elif kind == "actions":                  # 三条横杠
        for i in range(3):
            d.rounded_rectangle((cx - 7 * s, cy - 6 * s + i * 5 * s,
                                 cx + 7 * s, cy - 4 * s + i * 5 * s), 1, fill=c)
    elif kind == "safety":                   # 盾牌
        d.polygon([(cx, cy - 8 * s), (cx + 7 * s, cy - 5 * s), (cx + 7 * s, cy + 1 * s),
                   (cx, cy + 8 * s), (cx - 7 * s, cy + 1 * s), (cx - 7 * s, cy - 5 * s)],
                  outline=c, width=w)
    elif kind == "vision":                   # 眼睛/光圈
        d.ellipse((cx - 8 * s, cy - 5 * s, cx + 8 * s, cy + 5 * s), outline=c, width=w)
        d.ellipse((cx - 2.5 * s, cy - 2.5 * s, cx + 2.5 * s, cy + 2.5 * s), fill=c)
    elif kind == "params":                   # 齿轮（简化）
        d.ellipse((cx - 6 * s, cy - 6 * s, cx + 6 * s, cy + 6 * s), outline=c, width=w)
        for k in range(6):
            a = k * 3.14159 / 3
            import math
            d.line((cx + 6 * s * math.cos(a), cy + 6 * s * math.sin(a),
                    cx + 8.5 * s * math.cos(a), cy + 8.5 * s * math.sin(a)), fill=c, width=w)


# ================================================================ 界面示意图
def draw_ui(path: Path):
    W, H = 1600, 1140
    UI_H = 1010                      # 界面本体高度，下面是图例条
    TOP = 56
    DOCK = 128
    RAIL = 78
    PANEL_W = 432

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    px0 = W - PANEL_W
    sy0, sy1 = TOP, UI_H - DOCK

    # ---------- 背景分区 ----------
    d.rectangle((0, 0, W, TOP), fill="#0c1322")
    d.line((0, TOP, W, TOP), fill=LINE)
    d.rectangle((0, 0, RAIL, sy1), fill="#0c1322")
    d.line((RAIL, TOP, RAIL, sy1), fill=LINE)
    d.rectangle((RAIL, TOP, px0, sy1), fill=DEEP)
    d.rectangle((px0, TOP, W, sy1), fill=PANEL)
    d.line((px0, TOP, px0, sy1), fill=LINE)
    d.rectangle((0, sy1, W, H), fill="#0c1322")
    d.line((0, sy1, W, sy1), fill=LINE)

    # ================================================== 顶栏
    text(d, (18, TOP / 2 + 1), "三维机械臂 · 安全环境", 15, TXT, bold=True, anchor="lm")
    x = 18 + tw(d, "三维机械臂 · 安全环境", 15, bold=True) + 14
    x = chip(d, x, 16, "仿真", "#c4b5fd", "#7c5cff1a", dot="#7c5cff") + 8
    x = chip(d, x, 16, "未使能", DIM, dot="#64748b") + 8

    # 使能胶囊：内嵌速度输入 + 按钮
    rr(d, (x, 12, x + 188, 44), 16, fill="#141d31", outline=LINE)
    text(d, (x + 12, 28), "速度", 11.5, DIM, anchor="lm")
    rr(d, (x + 44, 17, x + 92, 39), 6, fill="#0e1526", outline=LINE)
    text(d, (x + 68, 28), "20", 12, TXT, mono=True, anchor="mm")
    text(d, (x + 98, 28), "%", 11.5, DIM, anchor="lm")
    btn(d, x + 112, 15, "使能", on=True, size=12, pad=9, h=26)

    # 右侧：全局状态胶囊（只读）
    rx = W - 18
    for lab, val, col in [("手", "握拳", TXT), ("手腕", "+18°", TXT), ("最小间距", "412 mm", OK)]:
        wdt = tw(d, f"{lab} {val}", 12) + 22
        rx -= wdt
        rr(d, (rx, 16, rx + wdt, 40), 12, fill=None, outline=LINE)
        text(d, (rx + 11, 28), lab, 12, DIM, anchor="lm")
        text(d, (rx + 11 + tw(d, lab, 12) + 6, 28), val, 12, col, mono=True, anchor="lm")
        rx -= 8
    btn(d, rx - 78, 16, "灵巧手 →", size=11.5, pad=8, h=23)
    btn(d, rx - 168, 16, "摄像头 →", size=11.5, pad=8, h=23)

    # ================================================== 左图标栏
    rail_items = [("trace", "轨迹", "38", ACCENT), ("pose", "姿态", None, None),
                  ("actions", "动作库", None, None), ("SEP",), ("safety", "安全", "OK", OK),
                  ("vision", "视觉", "!", WARN), ("SEP",), ("params", "参数", None, None)]
    ry = TOP + 10
    idx = 0
    for it in rail_items:
        if it[0] == "SEP":
            d.line((RAIL / 2 - 16, ry + 6, RAIL / 2 + 16, ry + 6), fill=LINE)
            ry += 18
            continue
        ic, lab, tag, tcol = it
        on = idx == 0
        if on:
            rr(d, (8, ry, RAIL - 8, ry + 54), 10, fill="#17233d")
            d.rounded_rectangle((5, ry + 9, 8, ry + 45), 2, fill=ACCENT)
        icon(d, RAIL / 2, ry + 16, ic, TXT if on else DIM, 1.05)
        text(d, (RAIL / 2, ry + 40), lab, 10.5, TXT if on else DIM2, anchor="mm")
        if tag:
            tws = tw(d, tag, 9) + 8
            bgc = tcol if tcol else "#1d4ed8"
            rr(d, (RAIL - 21, ry + 3, RAIL - 21 + tws, ry + 15), 6, fill=bgc)
            text(d, (RAIL - 21 + tws / 2, ry + 9), tag, 9,
                 "#0a0f1c" if tcol in (OK, WARN) else "#ffffff", anchor="mm")
        ry += 58
        idx += 1

    # ================================================== 三维舞台
    st_cx = (RAIL + px0) / 2
    # 地面
    d.ellipse((st_cx - 350, sy1 - 180, st_cx + 350, sy1 - 40), fill="#0d1422")
    gcol = "#16304f"
    for k in range(1, 8):
        yy = sy1 - 60 - k * 14
        half = 300 + k * 22
        d.line((st_cx - half, yy, st_cx + half, yy), fill=gcol)
    for k in range(-6, 7):
        d.line((st_cx + k * 50, sy1 - 172, st_cx + k * 96, sy1 - 60), fill=gcol)

    # 桌面（透视四边形）
    tx0, ty0 = st_cx - 268, sy1 - 210
    tx1, ty1 = st_cx + 268, sy1 - 210
    bx0, by0 = st_cx - 312, sy1 - 150
    bx1, by1 = st_cx + 312, sy1 - 150
    d.polygon([(tx0, ty0), (tx1, ty1), (bx1, by1), (bx0, by0)], fill="#1a2436", outline="#5d6b85")
    d.line((bx0, by0, bx1, by1), fill="#7a879e", width=2)
    # 桌腿
    for lx in (bx0 + 34, bx1 - 34):
        d.line((lx, by0, lx - 4, sy1 - 68), fill="#59637a", width=5)

    # 纸面区域（虚线框）
    pxx0, pxy0 = st_cx - 148, ty0 + 16
    pxx1, pxy1 = st_cx + 148, ty0 + 16
    pbx0, pby0 = st_cx - 172, ty0 + 58
    pbx1, pby1 = st_cx + 172, ty0 + 58
    d.polygon([(pxx0, pxy0), (pxx1, pxy1), (pbx1, pby1), (pbx0, pby0)], fill="#2a2410")
    for seg in [((pxx0, pxy0), (pxx1, pxy1)), ((pxx1, pxy1), (pbx1, pby1)),
                ((pbx1, pby1), (pbx0, pby0)), ((pbx0, pby0), (pxx0, pxy0))]:
        d.line(seg, fill="#a1801a", width=1)
    # 纸上的圆（已规划轨迹）
    ccx, ccy = st_cx, ty0 + 38
    d.ellipse((ccx - 96, ccy - 17, ccx + 96, ccy + 17), outline=OK, width=2)
    d.ellipse((ccx - 96, ccy - 16, ccx + 96, ccy + 16), outline="#86efac")

    # 机械臂
    basex, basey = st_cx - 300, ty0 + 30
    p0 = (basex, basey)
    p1 = (basex, ty0 - 80)
    p2 = (basex + 108, ty0 - 220)
    p3 = (basex + 236, ty0 - 120)
    p4 = (basex + 322, ty0 - 58)
    p5 = (ccx - 96, ccy)          # 指尖落在圆的起点
    for a, b in ((p0, p1), (p1, p2), (p2, p3), (p3, p4)):
        d.line((a, b), fill=BLUE, width=11)
        d.line((a, b), fill="#93b4e8", width=3)
    d.line((p4, p5), fill="#b7bec9", width=3)
    d.rectangle((basex - 24, basey - 8, basex + 24, basey + 8), fill=BLUE)
    for p, rad in ((p1, 10), (p2, 8), (p3, 7), (p4, 6)):
        d.ellipse((p[0] - rad, p[1] - rad, p[0] + rad, p[1] + rad), fill=BLUE, outline="#93b4e8")

    # 人 + 禁入柱
    hx = st_cx + 330
    d.ellipse((hx - 58, by0 - 22, hx + 58, by0 + 12), outline="#f87171")
    d.rounded_rectangle((hx - 15, ty0 + 6, hx + 15, by0 - 6), 12, fill="#37456b")
    d.ellipse((hx - 13, ty0 - 20, hx + 13, ty0 + 6), fill="#a98a72")

    # HUD 左上
    hudx, hudy = RAIL + 16, TOP + 14
    rr(d, (hudx, hudy, hudx + 232, hudy + 76), 12, fill="#0b1220", outline=LINE)
    for i, (k, v) in enumerate([("J  ", "  0.0 -25.0 -40.0   0.0  65.0   0.0"),
                                ("XYZ", "  611  -172   720"),
                                ("R  ", "  180.0   -0.0  -180.0")]):
        text(d, (hudx + 11, hudy + 14 + i * 20), k, 11, DIM2, mono=True)
        text(d, (hudx + 40, hudy + 14 + i * 20), v, 11, "#9fb0d0", mono=True)

    # HUD 右上
    hux = px0 - 250
    rr(d, (hux, hudy, px0 - 16, hudy + 122), 12, fill="#0b1220", outline=LINE)
    text(d, (hux + 14, hudy + 20), "SAFE", 11, DIM, anchor="lm")
    text(d, (hux + 13, hudy + 42), "412", 30, "#86efac", mono=True, anchor="lm")
    text(d, (hux + 13 + tw(d, "412", 30, mono=True) + 5, hudy + 48), "mm", 13, DIM, anchor="lm")
    text(d, (hux + 14, hudy + 66), "J6 ↔ 料盒", 11, DIM, anchor="lm")
    rr(d, (hux + 14, hudy + 78, px0 - 32, hudy + 83), 3, fill="#1b2438")
    rr(d, (hux + 14, hudy + 78, hux + 14 + (px0 - 46 - hux) * 0.62, hudy + 83), 3, fill=OK)
    text(d, (hux + 14, hudy + 101), "SSM 要求 96 mm · TCP 62 mm/s", 10.5, DIM, anchor="lm")

    # 视角条
    vlabels = ["斜视", "俯视", "前视", "侧视", "看纸面"]
    tot = sum(tw(d, s, 12) + 22 for s in vlabels) + 8
    vx = st_cx - tot / 2
    vy = sy1 - 46
    rr(d, (vx - 4, vy - 4, vx + tot + 4, vy + 35), 18, fill="#0b1220", outline=LINE)
    for s in vlabels:
        wdt = tw(d, s, 12) + 22
        if s == "斜视":
            rr(d, (vx, vy, vx + wdt, vy + 27), 14, fill="#1d4ed8")
            text(d, (vx + wdt / 2, vy + 14), s, 12, "#ffffff", anchor="mm")
        else:
            text(d, (vx + wdt / 2, vy + 14), s, 12, DIM, anchor="mm")
        vx += wdt
    text(d, (RAIL + 16, sy1 - 24), "左键旋转 · 滚轮缩放 · 右键平移", 11, "#3f4d6b")

    # ================================================== 右面板
    text(d, (px0 + 46, TOP + 24), "二维手绘轨迹", 13, TXT, bold=True, anchor="lm")
    icon(d, px0 + 26, TOP + 24, "trace", TXT, 0.95)
    text(d, (px0 + 46, TOP + 44), "画 → 规划 → 执行", 11.5, DIM, anchor="lm")
    btn(d, W - 16 - 70, TOP + 12, "收起全部", size=11, pad=8, h=23)
    d.line((px0, TOP + 58, W, TOP + 58), fill=LINE)

    bx = px0 + 14
    bw = PANEL_W - 28

    # —— 卡片：画板（展开）
    cy = TOP + 68
    ch = 196
    card(d, (bx, cy, bx + bw, cy + ch))
    chev(d, bx + 15, cy + 17, on=True)
    text(d, (bx + 28, cy + 17), "画板", 12.5, TXT, bold=True, anchor="lm")
    text(d, (bx + bw - 11, cy + 17), "38 点 · 15.0 mm", 11, DIM, mono=True, anchor="rm")
    d.line((bx, cy + 31, bx + bw, cy + 31), fill="#1b2540")
    padx0, pady0 = bx + 11, cy + 41
    padw = bw - 22
    rr(d, (padx0, pady0, padx0 + padw, pady0 + 120), 10, fill="#0b1220", outline="#2a3a5c")
    d.line((padx0 + padw / 2, pady0, padx0 + padw / 2, pady0 + 120), fill="#16233b")
    d.line((padx0, pady0 + 60, padx0 + padw, pady0 + 60), fill="#16233b")
    pcx = padx0 + padw / 2
    d.ellipse((pcx - 84, pady0 + 20, pcx + 84, pady0 + 98), outline=PURPLE, width=2)
    d.ellipse((pcx - 3, pady0 + 16, pcx + 4, pady0 + 23), fill="#c4b5fd")
    # 小 × 清空
    rr(d, (padx0 + padw - 26, pady0 + 6, padx0 + padw - 6, pady0 + 26), 6,
       fill="#0b1220", outline=LINE)
    text(d, (padx0 + padw - 16, pady0 + 16), "×", 12, DIM, anchor="mm")
    text(d, (bx + 11, cy + ch - 12), "横向 u 从左到右，纵向 v 从上到下，映射到实测可解纸面矩形",
         10.5, DIM2, anchor="lm")

    # —— 卡片：轨迹参数（展开，表）  只放本域相关参数
    cy += ch + 8
    ch = 296
    card(d, (bx, cy, bx + bw, cy + ch))
    chev(d, bx + 15, cy + 17, on=True)
    text(d, (bx + 28, cy + 17), "轨迹参数", 12.5, TXT, bold=True, anchor="lm")
    text(d, (bx + bw - 11, cy + 17), "TraceConfig", 11, DIM, mono=True, anchor="rm")
    d.line((bx, cy + 31, bx + bw, cy + 31), fill="#1b2540")

    c1, c2 = bx + 11, bx + 218          # 名称列 / 值列
    c3 = bx + bw - 45                   # 「可改」标签列
    th = cy + 42
    text(d, (c1, th), "参数", 10.5, DIM2)
    text(d, (c2, th), "值", 10.5, DIM2)
    d.line((c1, th + 15, bx + bw - 11, th + 15), fill="#1b2540")
    rows = [("步长 max_step_mm", "15.0 mm", "改"),
            ("圆角 blend_r_mm", "5.0 mm", "改"),
            ("拐角吸附 corner_deg", "50.0°", "改"),
            ("书写速度", "40 mm/s", "改"),
            ("加速度 accel_pct", "0（不设）", "改"),
            ("纸面高度 paper_z", "250.0 mm", "读"),
            ("可达区域 x", "-320 .. 420", "读"),
            ("可达区域 y", "-480 .. -180", "读")]
    ry2 = th + 24
    for k, v, kind in rows:
        text(d, (c1, ry2), k, 12, DIM)
        text(d, (c2, ry2), v, 12, "#cbd6f0", mono=True)
        if kind == "改":
            rr(d, (c3, ry2 - 2, c3 + 34, ry2 + 15), 4, fill=None, outline="#3f6212")
            text(d, (c3 + 17, ry2 + 6), "可改", 9.5, "#a3e635", anchor="mm")
        else:
            rr(d, (c3, ry2 - 2, c3 + 34, ry2 + 15), 4, fill=None, outline="#334155")
            text(d, (c3 + 17, ry2 + 6), "只读", 9.5, "#64748b", anchor="mm")
        d.line((c1, ry2 + 19, bx + bw - 11, ry2 + 19), fill="#151f31")
        ry2 += 25
    # 脚注：再让出 12px，别和最后一行、卡片边框挤在一起
    text(d, (c1, ry2 + 4), "出处 handlink/trace2d.py · TraceConfig",
         10.5, DIM2, anchor="la")

    # —— 卡片：方向与闭合（收起）
    cy += ch + 8
    card(d, (bx, cy, bx + bw, cy + 36))
    chev(d, bx + 15, cy + 17)
    text(d, (bx + 28, cy + 17), "方向与闭合", 12.5, DIM, bold=True, anchor="lm")
    text(d, (bx + bw - 11, cy + 17), "正", 11, DIM2, mono=True, anchor="rm")

    # —— 卡片：播放与安全（收起）
    cy += 44
    card(d, (bx, cy, bx + bw, cy + 36))
    chev(d, bx + 15, cy + 17)
    text(d, (bx + 28, cy + 17), "播放与安全", 12.5, DIM, bold=True, anchor="lm")
    text(d, (bx + bw - 11, cy + 17), "4×", 11, DIM2, mono=True, anchor="rm")

    # ================================================== 底部动作台
    dy0 = sy1
    text(d, (18, dy0 + 20), "当前目标", 10.5, "#5f6d8f", anchor="lm")
    icon(d, 26, dy0 + 42, "trace", TXT, 0.9)
    text(d, (40, dy0 + 42), "二维轨迹", 13, TXT, bold=True, anchor="lm")
    text(d, (40, dy0 + 63), "38 点 · 15.0 mm · 1932 mm", 12, OK, mono=True, anchor="lm")
    text(d, (18, dy0 + 84), "来自左侧「轨迹」面板", 11, DIM2, anchor="lm")

    # 步骤条
    steps = [("1", "绘制", "done"), ("2", "规划预览", "done"),
             ("3", "播放 / 下发", "now"), ("4", "完成", "todo")]
    sx = 320
    syc = dy0 + 46
    for i, (n, lab, st) in enumerate(steps):
        if st == "done":
            d.ellipse((sx, syc - 11, sx + 22, syc + 11), fill="#15803d", outline="#22c55e")
            text(d, (sx + 11, syc + 0.5), n, 11, "#ffffff", mono=True, anchor="mm")
            lc = DIM
        elif st == "now":
            d.ellipse((sx - 4, syc - 15, sx + 26, syc + 15), fill=None, outline="#1d4ed8")
            d.ellipse((sx, syc - 11, sx + 22, syc + 11), fill="#1d4ed8", outline="#3b82f6")
            text(d, (sx + 11, syc + 0.5), n, 11, "#ffffff", mono=True, anchor="mm")
            lc = "#ffffff"
        else:
            d.ellipse((sx, syc - 11, sx + 22, syc + 11), fill="#111a2c", outline=LINE)
            text(d, (sx + 11, syc + 0.5), n, 11, DIM2, mono=True, anchor="mm")
            lc = DIM2
        text(d, (sx + 30, syc), lab, 12, lc, bold=(st == "now"), anchor="lm")
        sx += 30 + tw(d, lab, 12, bold=(st == "now")) + 18
        if i < 3:
            d.line((sx - 22, syc, sx + 10, syc), fill=LINE)
            sx += 32

    # 动作按钮组
    bw2 = 106
    byt = dy0 + 24
    bx2 = W - 18 - bw2
    btn(d, bx2, byt, "下发真机", danger=True, dimmed=True, w=bw2)
    bx2 -= bw2 + 6
    btn(d, bx2, byt, "安全抬升", w=bw2)
    bx2 -= bw2 + 6
    btn(d, bx2, byt, "播放 4×", on=True, w=bw2)
    bx2 -= bw2 + 6
    btn(d, bx2, byt, "重新规划", w=bw2)

    text(d, (W - 18, dy0 + 72), "已规划 38/38 · 失败 0 · 关节跳变 2.84°",
         11.5, DIM, anchor="rm")
    rr(d, (bx2, dy0 + 86, W - 18, dy0 + 90), 2, fill="#1b2438")
    rr(d, (bx2, dy0 + 86, W - 18, dy0 + 90), 2, fill=OK)
    text(d, (W - 18, dy0 + 104), "首尾闭合 0.000 mm · 全程 1932.7 mm",
         11, DIM2, anchor="rm")

    # ================================================== 图例条（在界面下方，不压内容）
    d.rectangle((0, UI_H, W, H), fill="#0c1322")
    d.line((0, UI_H, W, UI_H), fill="#3f6212")
    text(d, (40, UI_H + 24), "这次只动布局与信息组织，以下全部原样保留：",
         13, TXT, bold=True, anchor="lm")
    keep = [
        ("三维场景", "原厂 Nova 5 STL（base_link + Link1~Link6）、螺钉线槽、关节罩"),
        ("工位", "桌面 1.4×0.9 m、料盒、相机立柱、防护栏、操作员、禁入柱"),
        ("碰撞", "胶囊代理、最小间距、SSM 计算、红点标注最近的一对"),
        ("相机", "桌面单应反投影贴图、视觉物体进碰撞模型"),
        ("手势", "灵巧手五指几何与弯曲联动"),
        ("渲染", "three.js 光照 / 阴影 / ACES 色调映射 / OrbitControls"),
    ]
    for i, (k, v) in enumerate(keep):
        col = 40 + (i % 2) * 780
        row_y = UI_H + 50 + (i // 2) * 26
        # 手绘对勾，避免字体缺字
        d.line((col, row_y + 2, col + 3, row_y + 6), fill=OK, width=2)
        d.line((col + 3, row_y + 6, col + 10, row_y - 4), fill=OK, width=2)
        text(d, (col + 18, row_y), k, 11.5, TXT, bold=True, anchor="lm")
        text(d, (col + 18 + tw(d, k, 11.5, bold=True) + 10, row_y), v, 11.5, DIM2, anchor="lm")

    img.save(path)
    return path


# ================================================================ 信息架构对照
def draw_ia(path: Path):
    W, H = 1600, 1010
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    text(d, (40, 34), "arm3d 信息架构：现在 vs 重构后", 22, TXT, bold=True)
    text(d, (40, 68), "同一批功能，从「12 张卡片单列纵向滚动」改成「6 个工作域 + 1 个动作台」",
         13, DIM)

    # ---------------- 左：现在
    lx = 40
    text(d, (lx, 118), "现在", 15, WARN, bold=True)
    text(d, (lx + 52, 121), "12 张卡片一列排到底，画板排在第 8 位", 12, DIM)

    cards = [
        ("数据源", ""), ("机械臂使能", "拖动示教"), ("安全状态", ""),
        ("相机实时环境", ""), ("关节角（度）", "5 个预设姿态"),
        ("保存的姿态", "保存 / 刷新 / 删除"), ("组动作", "每个动作一个按钮"),
        ("二维手绘轨迹", "清空 / 预览 / 播放 / 下发 / 归位 + 视角 + 倍速 + 镜像"),
        ("末端位姿", ""), ("安全环境（显示开关）", ""), ("灵巧手", ""),
        ("手腕走线", "标记为没缠"), ("校验后下发", "校验 / 下发 / 强制下发"),
    ]
    cy = 146
    for name, sub in cards:
        h = 32 if not sub else 48
        hot = name in ("二维手绘轨迹", "校验后下发", "组动作")
        card(d, (lx, cy, lx + 440, cy + h), 8,
             fill="#161f33" if hot else CARD, outline="#7c5cff66" if hot else LINE)
        text(d, (lx + 12, cy + 9), name, 12, TXT if hot else DIM, bold=hot)
        if sub:
            text(d, (lx + 12, cy + 27), sub, 10.5, DIM2)
        cy += h + 4

    text(d, (lx, cy + 10), "执行入口散在 3 张卡片里（组动作 / 轨迹 / 校验后下发），", 11.5, ERR)
    text(d, (lx, cy + 28), "每张都有自己的「下发」按钮和一套状态文案。", 11.5, ERR)
    text(d, (lx, cy + 54), "参数没有表：纸面几何、步长、圆角、速度混在正文说明里；", 11.5, DIM2)
    text(d, (lx, cy + 72), "画板被挤到第 8 位，得滚动才找得到。", 11.5, DIM2)

    # ---------------- 中：箭头
    ax = 500
    d.line((ax, 380, ax + 96, 380), fill=ACCENT, width=2)
    d.polygon([(ax + 96, 380), (ax + 84, 374), (ax + 84, 386)], fill=ACCENT)
    text(d, (ax + 48, 366), "重组", 12, ACCENT, anchor="mm")

    # ---------------- 右：重构后
    rx = 620
    RW = W - 40 - rx
    text(d, (rx, 118), "重构后", 15, OK, bold=True)
    text(d, (rx + 60, 121), "6 个工作域互斥展开 + 底部统一动作台", 12, DIM)

    # 顶栏
    card(d, (rx, 146, W - 40, 200), 8)
    text(d, (rx + 12, 162), "顶栏", 11.5, ACCENT, bold=True)
    cx2 = rx + 62
    for s in ["仿真", "未使能", "速度 20%", "最小间距", "手腕", "手型"]:
        cx2 = chip(d, cx2, 152, s, DIM, size=10.5) + 6
    text(d, (rx + 12, 186), "只放「状态」和「全局开关」，不放任何动作", 10.5, DIM2)

    # 左栏
    card(d, (rx, 212, rx + 128, 600), 8)
    text(d, (rx + 10, 226), "左图标栏", 11.5, ACCENT, bold=True)
    rail2 = [("trace", "轨迹"), ("pose", "姿态"), ("actions", "动作库"),
             ("safety", "安全"), ("vision", "视觉"), ("params", "参数")]
    for i, (icn, s) in enumerate(rail2):
        on = i == 0
        yb = 244 + i * 38
        if on:
            rr(d, (rx + 8, yb - 4, rx + 120, yb + 28), 7, fill="#17233d")
        icon(d, rx + 24, yb + 12, icn, TXT if on else DIM, 0.85)
        text(d, (rx + 42, yb + 12), s, 11, TXT if on else DIM, bold=on, anchor="lm")
    text(d, (rx + 10, 480), "一次只展开一个域，", 10.5, DIM2)
    text(d, (rx + 10, 496), "其余不占屏幕。", 10.5, DIM2)
    text(d, (rx + 10, 520), "带角标：点数、告警、", 10.5, DIM2)
    text(d, (rx + 10, 536), "视觉状态。", 10.5, DIM2)

    # 舞台
    card(d, (rx + 140, 212, rx + 428, 600), 8)
    text(d, (rx + 152, 226), "三维舞台（原样保留）", 11.5, ACCENT, bold=True)
    d.polygon([(rx + 210, 380), (rx + 358, 380), (rx + 374, 412), (rx + 194, 412)],
              fill="#1a2436", outline="#5d6b85")
    d.ellipse((rx + 248, 386, rx + 348, 408), outline=OK, width=2)
    for a, b in [((rx + 200, 404), (rx + 200, 340)), ((rx + 200, 340), (rx + 250, 296)),
                 ((rx + 250, 296), (rx + 300, 340)), ((rx + 300, 340), (rx + 348, 384))]:
        d.line((a, b), fill=BLUE, width=7)
    text(d, (rx + 152, 440), "原厂 STL · 工位 · 碰撞代理", 10.5, DIM2)
    text(d, (rx + 152, 456), "全部不动，只把位置让出来", 10.5, DIM2)
    text(d, (rx + 152, 480), "视角条 / HUD 浮在上面", 10.5, DIM2)

    # 右面板
    card(d, (rx + 440, 212, W - 40, 600), 8)
    text(d, (rx + 452, 226), "上下文面板（只显示当前域）", 11.5, ACCENT, bold=True)
    py = 244
    for name, state in [("画板", "展开"), ("轨迹参数（表）", "展开"),
                        ("方向与闭合", "收起"), ("播放与安全", "收起")]:
        h = 62 if state == "展开" else 26
        rr(d, (rx + 452, py, W - 54, py + h), 7,
           fill="#131d33" if state == "展开" else "#0b1220", outline=LINE)
        chev(d, rx + 464, py + 13, on=(state == "展开"), color="#5f6d8f")
        text(d, (rx + 476, py + 13), name, 11, TXT if state == "展开" else DIM2, anchor="lm")
        text(d, (W - 64, py + 13), state, 10, DIM2, anchor="rm")
        if state == "展开":
            rr(d, (rx + 462, py + 26, W - 64, py + 54), 5, fill="#0b1220", outline="#2a3a5c")
        py += h + 6
    text(d, (rx + 452, py + 12), "参数一律用表：名称 · 值 · 可改 / 只读", 10.5, DIM2)
    text(d, (rx + 452, py + 30), "不再把配置写进说明正文。", 10.5, DIM2)

    # 底部动作台
    card(d, (rx, 612, W - 40, 726), 8, fill="#131d33", outline="#3b82f666")
    text(d, (rx + 12, 628), "底部动作台 · 所有动作的唯一入口", 12, "#93c5fd", bold=True)
    for i, s in enumerate(["1 绘制", "2 规划预览", "3 播放 / 下发", "4 完成"]):
        on = i == 2
        rr(d, (rx + 14 + i * 100, 646, rx + 104 + i * 100, 670), 7,
           fill="#1d4ed8" if on else "#1a2540", outline="#3b82f6" if on else LINE)
        text(d, (rx + 59 + i * 100, 658), s, 10, "#ffffff" if on else DIM, anchor="mm")
    bx3 = W - 46
    for i, s in enumerate(["下发真机", "安全抬升", "播放 4×", "重新规划"]):
        danger = s == "下发真机"
        w3 = 74
        bx3 -= w3 + 5
        rr(d, (bx3, 646, bx3 + w3, 670), 7,
           fill="#3a1620" if danger else ("#1d4ed8" if s == "播放 4×" else "#1a2540"),
           outline="#7f1d1d" if danger else LINE)
        text(d, (bx3 + w3 / 2, 658), s, 9.5,
             "#fecaca" if danger else TXT, anchor="mm")
    text(d, (rx + 12, 690),
         "组动作、手动姿态、二维轨迹 → 都汇到这里：同一套步骤条、同一套结果文案、同一道安全闸门。",
         10.5, DIM2)
    text(d, (rx + 12, 708), "安全闸门仍是服务端那套：/arm/check + /arm/joints 二次拦截。", 10.5, DIM2)

    # 底部结论（让开左栏注释，另起一段）
    text(d, (40, 836), "合并的冗余", 13, TXT, bold=True)
    合并 = [
        ("5 个「下发 / 执行」按钮", "1 个动作台，4 步固定流程"),
        ("3 套结果文案（actMsg / padMsg / sendMsg）", "1 个结果行 + 统一的颜色语义"),
        ("分散的镜像开关 + 对调按钮", "1 个「方向」四选一"),
        ("3 处显示开关散在两张卡片", "收进对应域，舞台上留一个总开关"),
        ("参数散落在说明正文里", "「参数」一张全量表 + 各域只显示相关的"),
    ]
    yy = 864
    for a, b in 合并:
        text(d, (56, yy), "·", 13, ACCENT, bold=True)
        text(d, (70, yy), a, 11.5, DIM)
        text(d, (410, yy), "→", 11.5, ACCENT)
        text(d, (436, yy), b, 11.5, TXT)
        yy += 24

    img.save(path)
    return path


if __name__ == "__main__":
    targets = sys.argv[1:] or ["ui", "ia"]
    OUT.mkdir(parents=True, exist_ok=True)
    if "ui" in targets:
        print(draw_ui(OUT / "arm3d-redesign-ui.png"))
    if "ia" in targets:
        print(draw_ia(OUT / "arm3d-redesign-ia.png"))
