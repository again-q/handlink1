"""边干活边记录：跑一轮真实任务，回头看得见问题在哪。

    python tools/glove_soak.py --seconds 180 --out .tmp/soak.csv

它只读 /glove/state（10 Hz），不碰手套、不碰总线，任务照常跑。结束时给一张小结：
掉线、跳变、饱和、零点漂移、帧率——真实任务里"抓一半突然合手""拇指抽一下"这类
现象，基本都是这几项里的某一项在动。
"""
from __future__ import annotations

import argparse
import csv
import json
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional

FIELDS = ("frames", "packets", "applied", "errors", "fps", "live", "frame_age_s",
          "spikes", "baseline_ready", "zeroed_at")


def fetch(base: str, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
    try:
        with urllib.request.urlopen(base + "/glove/state", timeout=timeout) as r:
            return json.loads(r.read())
    except Exception:            # noqa: BLE001 - 采集脚本，读不到就跳过这一拍
        return None


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="跑任务时顺手记录手套状态")
    ap.add_argument("--base", default="http://127.0.0.1:8766")
    ap.add_argument("--seconds", type=float, default=180.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--out", default=".tmp/soak.csv")
    args = ap.parse_args(argv)

    period = 1.0 / max(1.0, args.hz)
    rows: List[Dict[str, Any]] = []
    t0 = time.monotonic()
    drops = 0
    prev_frames = None
    prev_jumps = None
    stale_since: Optional[float] = None
    stale_total = 0.0
    sat_total = 0.0
    first: Optional[Dict[str, Any]] = None
    last: Optional[Dict[str, Any]] = None

    print(f"开始记录 {args.seconds:g}s → {args.out}（任务照常跑）")
    while time.monotonic() - t0 < args.seconds:
        now = time.monotonic() - t0
        st = fetch(args.base)
        if st is None:
            drops += 1
        else:
            first = first or st
            last = st
            frames = st.get("frames") or 0
            if prev_frames is not None and frames <= prev_frames and st.get("running"):
                drops += 1                      # 计数不涨 = 这一拍没新帧
            prev_frames = frames
            prev_jumps = st.get("spikes")
            if st.get("live"):
                stale_since = None
            elif stale_since is None:
                stale_since = now
            if stale_since is not None:
                stale_total += period
            if st.get("saturated"):
                sat_total += period
            rows.append({"t": round(now, 2), **{k: st.get(k) for k in FIELDS},
                         "motors": json.dumps(st.get("motors"), ensure_ascii=False),
                         "saturated": ",".join(st.get("saturated") or [])})
        time.sleep(period)

    if not rows:
        print("一条状态都没读到：服务没起？端口不对？")
        return 2

    path = Path(args.out)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    span = max(rows[-1]["t"] - rows[0]["t"], 1e-6)
    print()
    print(f"采样      {len(rows)} 条 / {span:.0f}s")
    print(f"读状态失败 {drops} 次（含没新帧的拍子）")
    print(f"手套在线   {span - stale_total:.0f}s / {span:.0f}s"
          + (f"，掉线 {stale_total:.0f}s" if stale_total > 0.5 else ""))
    print(f"饱和时长   {sat_total:.1f}s" + ("   ← 有指头顶到行程，点位会被夹" if sat_total > 0.5 else ""))
    if first is not None and last is not None:
        print(f"帧数        {first.get('frames')} → {last.get('frames')}")
        print(f"毛刺次数    {last.get('spikes')}（跳上去又弹回，多半是传感器松/丢包）")
        print(f"下发次数    {first.get('applied')} → {last.get('applied')}")
        print(f"零点重置    {last.get('zeroed_at')}")
    print(f"明细见 {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
