"""点位与动作文件的读写，与 LG-603 的 points / gesturePoints / tracks 目录兼容。

厂家格式（实测）
----------------
points/rock.txt::

    [{"motorId":102,"angle":40,"speed":360,"torque":0.5}, ...]

tracks/动作一.txt::

    points=[paper,scissors,rock,paper]
    delays=[600,1500,1500,600]
    loops=2
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

from . import spec


# ---------------------------------------------------------------- 目录

def find_root(start: Optional[Path] = None) -> Optional[Path]:
    """找 LG-603 软件包根目录（含 LG-603.exe，或 points/ + tracks/）。

    依次看：HANDLINK_ROOT、当前目录及各级父目录、它们的下一层子目录。
    """
    env = os.environ.get("HANDLINK_ROOT")
    roots = [Path(env)] if env else []
    here = Path(start or Path.cwd()).resolve()
    roots += [here, *here.parents]

    def is_root(p: Path) -> bool:
        return (p / "points").is_dir() and (p / "tracks").is_dir()

    for r in roots:
        for cand in (r, r / "LG-603"):
            if is_root(cand):
                return cand
        try:
            children = list(r.iterdir())
        except OSError:
            continue
        for child in children[:200]:
            if not child.is_dir():
                continue
            for cand in (child, child / "LG-603"):
                if is_root(cand):
                    return cand
    return None


def default_root() -> Path:
    """找软件包根目录：环境变量 HANDLINK_ROOT > 自动搜索 > 当前目录。"""
    return find_root() or Path.cwd()


@dataclass
class Dirs:
    root: Path
    points: Path
    gestures: Path
    tracks: Path

    def ensure(self) -> "Dirs":
        for p in (self.points, self.gestures, self.tracks):
            p.mkdir(parents=True, exist_ok=True)
        return self


def dirs(root: Optional[Path] = None) -> Dirs:
    r = Path(root) if root else default_root()
    return Dirs(r, r / "points", r / "gesturePoints", r / "tracks")


# ---------------------------------------------------------------- 点位

def load_point(path: Path) -> spec.Pose:
    text = Path(path).read_text(encoding="utf-8-sig").strip()
    if not text:
        raise ValueError(f"点位文件为空: {path}")
    return spec.from_vendor(json.loads(text))


def dumps_point(pose: spec.Pose) -> str:
    """生成与厂家完全一致的紧凑 JSON。"""
    return json.dumps(spec.to_vendor(pose), ensure_ascii=False, separators=(",", ":"))


def save_point(path: Path, pose: spec.Pose) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(dumps_point(pose), encoding="utf-8")
    return p


def list_points(directory: Path) -> List[str]:
    d = Path(directory)
    if not d.is_dir():
        return []
    return sorted(p.stem for p in d.glob("*.txt"))


def resolve_point(name: str, directory: Path) -> Path:
    d = Path(directory)
    p = d / name
    if p.suffix != ".txt":
        p = p.with_suffix(".txt")
    if not p.is_file():
        raise FileNotFoundError(f"点位不存在: {p}（目录内: {list_points(d)}）")
    return p


# ---------------------------------------------------------------- 动作

@dataclass
class Track:
    points: List[str] = field(default_factory=list)
    delays: List[int] = field(default_factory=list)
    loops: int = 1

    def __post_init__(self) -> None:
        if len(self.delays) < len(self.points):
            self.delays = list(self.delays) + [1000] * (len(self.points) - len(self.delays))
        self.delays = self.delays[: len(self.points)]
        self.loops = max(1, int(self.loops))

    def as_dict(self) -> Dict[str, object]:
        return {"points": list(self.points), "delays": list(self.delays), "loops": self.loops}

    @property
    def duration_ms(self) -> int:
        return int(sum(self.delays)) * self.loops


_RE_POINTS = re.compile(r"points\s*=\s*\[([^\]]*)\]")
_RE_DELAYS = re.compile(r"delays\s*=\s*\[([^\]]*)\]")
_RE_LOOPS = re.compile(r"loops\s*=\s*(\d+)")


def load_track(path: Path) -> Track:
    text = Path(path).read_text(encoding="utf-8-sig")
    m = _RE_POINTS.search(text)
    if not m:
        raise ValueError(f"动作文件缺少 points=: {path}")
    names = [s.strip() for s in m.group(1).split(",") if s.strip()]
    d = _RE_DELAYS.search(text)
    delays = [int(float(s)) for s in d.group(1).split(",") if s.strip()] if d else []
    lp = _RE_LOOPS.search(text)
    loops = int(lp.group(1)) if lp else 1
    return Track(names, delays, loops)


def dumps_track(track: Track) -> str:
    return (
        f"points=[{','.join(track.points)}]\n"
        f"delays=[{','.join(str(int(x)) for x in track.delays)}]\n"
        f"loops={int(track.loops)}\n"
    )


def save_track(path: Path, track: Track) -> Path:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(dumps_track(track), encoding="utf-8")
    return p


def list_tracks(directory: Path) -> List[str]:
    d = Path(directory)
    if not d.is_dir():
        return []
    return sorted(p.stem for p in d.glob("*.txt"))


def load_track_steps(track: Track, points_dir: Path) -> List[Dict[str, object]]:
    """把动作展开成 (点位名, 姿态, 延时) 序列，便于预览与播放。"""
    steps: List[Dict[str, object]] = []
    for name, delay in zip(track.points, track.delays):
        p = resolve_point(name, points_dir)
        steps.append({"name": name, "pose": load_point(p), "delay_ms": int(delay)})
    return steps
