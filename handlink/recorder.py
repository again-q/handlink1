"""数据录制与离线分析。

录制格式：JSONL，一行一个事件，便于追加写、崩溃不丢数据、事后用 pandas 分析。

* ``{"t": 1762..., "kind": "status", "motors": {...}}``   —— 状态采样
* ``{"t": 1762..., "kind": "cmd", "reason": "...", "motors": {...}}`` —— 指令
* ``{"t": 1762..., "kind": "event", "event": "record_start", ...}`` —— 事件
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from . import spec


class Recorder:
    """线程安全的 JSONL 录制器。"""

    def __init__(self, path: Path, meta: Optional[Dict[str, Any]] = None) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._fh = self.path.open("a", encoding="utf-8")
        self.started_at = time.time()
        self.meta = dict(meta or {})
        self.event("record_start", **self.meta)

    def _write(self, obj: Dict[str, Any]) -> None:
        with self._lock:
            if self._fh.closed:
                return
            self._fh.write(json.dumps(obj, ensure_ascii=False) + "\n")
            self._fh.flush()

    def event(self, name: str, **kw: Any) -> None:
        self._write({"t": time.time(), "kind": "event", "event": name, **kw})

    def status(self, motors: Dict[str, Any]) -> None:
        self._write({"t": time.time(), "kind": "status", "motors": motors})

    def cmd(self, pose: spec.Pose, reason: str = "") -> None:
        self._write({
            "t": time.time(),
            "kind": "cmd",
            "reason": reason,
            "motors": {str(mid): j.as_dict() for mid, j in pose.items()},
        })

    @property
    def open(self) -> bool:
        return not self._fh.closed

    def close(self) -> None:
        if not self._fh.closed:
            self.event("record_stop")
            with self._lock:
                self._fh.close()


class Sampler:
    """后台按固定频率把总线状态写进录制文件。"""

    def __init__(self, bus, recorder: Recorder, hz: float = 20.0) -> None:
        self.bus = bus
        self.recorder = recorder
        self.period = 1.0 / max(hz, 0.1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="sampler", daemon=True)

    def start(self) -> "Sampler":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                snap = self.bus.snapshot()
                self.recorder.status(snap["motors"])
            except Exception as exc:  # 采样失败不应该杀掉服务
                self.recorder.event("sample_error", error=str(exc))
            self._stop.wait(self.period)


# ---------------------------------------------------------------- 分析

def analyze(path: Path) -> Dict[str, Any]:
    """读取录制文件，输出可直接看的统计。"""
    rows: List[Dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))

    status = [r for r in rows if r.get("kind") == "status"]
    cmds = [r for r in rows if r.get("kind") == "cmd"]
    events = [r for r in rows if r.get("kind") == "event"]

    per: Dict[str, Dict[str, Any]] = {}
    for mid in spec.MOTOR_IDS:
        key = str(mid)
        series = [r["motors"][key] for r in status if key in r.get("motors", {})]
        if not series:
            continue
        angles = [s.get("angle", 0.0) for s in series]
        speeds = [abs(s.get("speed", 0.0)) for s in series]
        currents = [s.get("current", 0.0) for s in series]
        limit = spec.MOTOR_BY_ID[mid].current_max
        per[key] = {
            "name": spec.MOTOR_BY_ID[mid].name,
            "samples": len(series),
            "angle_min": round(min(angles), 2),
            "angle_max": round(max(angles), 2),
            "rom": round(max(angles) - min(angles), 2),
            "speed_max": round(max(speeds), 2),
            "current_max": round(max(currents), 3),
            "current_peak_ratio": round(max(currents) / limit, 3) if limit else None,
            "errors": sum(1 for s in series if s.get("error")),
        }

    t0 = min((r["t"] for r in rows), default=None)
    t1 = max((r["t"] for r in rows), default=None)
    duration = round((t1 - t0), 3) if t0 and t1 else 0.0

    return {
        "file": str(path),
        "duration_s": duration,
        "samples": len(status),
        "commands": len(cmds),
        "events": [e.get("event") for e in events],
        "motors": per,
        "command_poses": [
            {"t": round(c["t"] - (t0 or c["t"]), 3), "reason": c.get("reason", ""),
             "angles": {k: v.get("angle") for k, v in c.get("motors", {}).items()}}
            for c in cmds[:20]
        ],
    }


def to_markdown(stats: Dict[str, Any]) -> str:
    lines = [
        f"# 录制分析：{Path(stats['file']).name}",
        "",
        f"- 时长：{stats['duration_s']} s",
        f"- 状态采样：{stats['samples']} 帧，指令：{stats['commands']} 条",
        f"- 事件：{', '.join(str(e) for e in stats['events'])}",
        "",
        "| 手指 | 最小角 | 最大角 | 行程 | 峰值速度 | 峰值电流 | 电流占用 | 异常 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for mid, row in stats["motors"].items():
        ratio = row["current_peak_ratio"]
        lines.append(
            f"| {row['name']}({mid}) | {row['angle_min']}° | {row['angle_max']}° | "
            f"{row['rom']}° | {row['speed_max']} | {row['current_max']} A | "
            f"{'-' if ratio is None else f'{ratio * 100:.0f}%'} | {row['errors']} |"
        )
    return "\n".join(lines)
