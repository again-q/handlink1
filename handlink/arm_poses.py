"""机械臂姿态存档：把「现在这个姿态」存下来，以后一键回填。

为什么单独存一份 JSON
---------------------
``poses.py`` 是灵巧手的角度表（厂家 points 格式），``arm_actions.py`` 是**相对增量**
的组动作，都不能直接当「协作臂的绝对姿态」用。示教时最常做的一件事其实是：

    把臂掰到位 / 拖滑块到位 ──► 存起来 ──► 下次回到这里

所以这里只做一件很小的事，文件里同时存两样东西：

* **关节角**（6 个，度）——回填滑块、MovJ 下发用的**权威值**；
* **末端位姿**（mm / deg）——给人看的（「桌面左侧 490,-518,104」比一串角度好认），
  读文件时不用重算，对不上也能立刻发现。

保存在 ``handlink/config/arm_poses.json``（和 ``tablescan`` 的 ``config/scan.json``
同一个目录）；``HANDLINK_ARM_POSES`` 可以换位置，测试和多套工位各存一份都用它。

名字按 UTF-8 原样保存（中文没问题），重名就是**覆盖**：示教时反复微调同一个姿态，
覆盖比存出十个「姿态 1 副本」更符合直觉，响应里的 ``replaced`` 告诉页面是新建还是
更新。写入走「临时文件 + ``os.replace``」，写到一半断电也不会把整个文件弄坏。
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from . import kin

#: 默认存档位置（包内 ``config/``，和 scan.json 放一起）
DEFAULT_PATH = Path(__file__).resolve().parent / "config" / "arm_poses.json"
#: 换存档位置的环境变量
ENV_PATH = "HANDLINK_ARM_POSES"

MAX_NAME = 40
MAX_NOTE = 200
MAX_POSES = 100


def _stamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def _clean_name(name: Any) -> str:
    """去掉首尾和多余空白；斜杠、超长直接拒绝（名字只当键用，不拼路径）。"""
    text = " ".join(str(name or "").split())
    if not text:
        return ""
    if len(text) > MAX_NAME:
        raise ValueError(f"名字太长了（最多 {MAX_NAME} 个字）")
    if "/" in text or "\\" in text:
        raise ValueError("名字里不能有斜杠")
    return text


def _clean_note(note: Any) -> str:
    return str(note or "").strip()[:MAX_NOTE]


class PoseStore:
    """一组「机械臂姿态」，落盘成一个 JSON 文件。线程安全（HTTP 是多线程的）。"""

    def __init__(self, path: Optional[Path] = None) -> None:
        self.path = Path(path or os.environ.get(ENV_PATH) or DEFAULT_PATH)
        self._lock = threading.RLock()
        self._poses: Dict[str, Dict[str, Any]] = {}
        self._load()

    # -- 文件 -------------------------------------------------------------
    def _load(self) -> None:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return                      # 没存过 / 文件坏了：当空表，别让服务起不来
        items = raw.get("poses") if isinstance(raw, dict) else raw
        if isinstance(items, dict):      # 也认 {name: {...}} 的写法
            items = list(items.values())
        if not isinstance(items, list):
            return
        for item in items:
            rec = self._normalize(item)
            if rec is not None:
                self._poses[rec["name"]] = rec

    @staticmethod
    def _normalize(item: Any) -> Optional[Dict[str, Any]]:
        """读文件时把一条记录收拾干净；坏记录直接跳过（一条坏不能拖垮整份存档）。"""
        if not isinstance(item, dict):
            return None
        try:
            name = _clean_name(item.get("name"))
            joints = [float(v) for v in (item.get("joints") or [])]
        except (TypeError, ValueError):
            return None
        if not name or len(joints) != 6 or any(not math.isfinite(v) for v in joints):
            return None
        clamped = kin.clamp(joints)
        try:
            pose = [float(v) for v in (item.get("pose") or [])]
            if len(pose) != 6 or any(not math.isfinite(v) for v in pose):
                pose = []
        except (TypeError, ValueError):
            pose = []
        if not pose:                     # 老文件 / 手改丢了位姿：用正解补
            pose = kin.fk(clamped)
        return {
            "name": name,
            "joints": [round(v, 3) for v in clamped],
            "pose": [round(v, 3) for v in pose],
            "note": _clean_note(item.get("note", "")),
            "source": str(item.get("source") or ""),
            "created": str(item.get("created") or ""),
            "updated": str(item.get("updated") or ""),
        }

    def _flush(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {"version": 1, "saved": _stamp(), "poses": self._poses}
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n",
                       encoding="utf-8")
        os.replace(tmp, self.path)

    # -- 查询 -------------------------------------------------------------
    @staticmethod
    def _public(rec: Dict[str, Any]) -> Dict[str, Any]:
        return {**rec, "joints": list(rec["joints"]), "pose": list(rec["pose"])}

    def _listing(self) -> List[Dict[str, Any]]:
        return [self._public(rec) for rec in self._poses.values()]

    def state(self) -> Dict[str, Any]:
        with self._lock:
            return {"ok": True, "path": str(self.path), "count": len(self._poses),
                    "max": MAX_POSES, "poses": self._listing()}

    def get(self, name: str) -> Optional[Dict[str, Any]]:
        name = _clean_name(name)
        with self._lock:
            rec = self._poses.get(name)
            return self._public(rec) if rec is not None else None

    def _auto_name(self) -> str:
        for i in range(1, MAX_POSES + 1):
            cand = f"姿态 {i}"
            if cand not in self._poses:
                return cand
        return f"姿态 {int(time.time())}"

    # -- 写 ---------------------------------------------------------------
    def save(self, name: Any, joints: Sequence[float], note: Any = "",
             source: Any = "") -> Dict[str, Any]:
        """存一个姿态；``name`` 为空自动编号，重名覆盖。返回给页面用的结果。"""
        clean = _clean_name(name)
        if len(joints) != 6:
            raise ValueError("需要 6 个关节角")
        try:
            values = [float(v) for v in joints]
        except (TypeError, ValueError):
            raise ValueError("关节角必须是数字") from None
        if any(not math.isfinite(v) for v in values):
            raise ValueError("关节角里有 nan / inf")

        clamped = kin.clamp(values)
        with self._lock:
            if not clean:
                clean = self._auto_name()
            replaced = clean in self._poses
            if not replaced and len(self._poses) >= MAX_POSES:
                raise ValueError(f"最多存 {MAX_POSES} 个姿态，先删掉几个再存")
            now = _stamp()
            rec = {
                "name": clean,
                "joints": [round(v, 3) for v in clamped],
                "pose": [round(v, 3) for v in kin.fk(clamped)],
                "note": _clean_note(note),
                "source": str(source or ""),
                "created": self._poses[clean]["created"] if replaced else now,
                "updated": now,
            }
            self._poses[clean] = rec
            self._flush()
            return {
                "ok": True,
                "replaced": replaced,
                "clamped": any(abs(a - b) > 1e-9 for a, b in zip(values, clamped)),
                "pose": self._public(rec),
                "count": len(self._poses),
                "path": str(self.path),
                "poses": self._listing(),
            }

    def delete(self, name: Any) -> Dict[str, Any]:
        clean = _clean_name(name)
        with self._lock:
            if not clean or clean not in self._poses:
                raise KeyError(f"没有这个姿态：{clean or '(空)'}")
            removed = self._public(self._poses.pop(clean))
            self._flush()
            return {"ok": True, "deleted": removed["name"], "pose": removed,
                    "count": len(self._poses), "path": str(self.path),
                    "poses": self._listing()}
