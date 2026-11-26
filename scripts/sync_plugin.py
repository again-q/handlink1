"""把 handlink/ 源码同步进插件包里的 vendored 副本。

插件 `handlink-plugin/python/handlink/` 是**独立副本**（打包用），不是软链接。
改完源码忘了同步，装上插件跑的仍是旧代码——本项目实际踩过。

用法：
    python scripts/sync_plugin.py          # 同步
    python scripts/sync_plugin.py --check  # 只检查，不一致就退出码 1
"""
from __future__ import annotations

import argparse
import filecmp
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "handlink"
DST = ROOT / "handlink-plugin" / "python" / "handlink"

# models/ 有 7.5 MB 的 mediapipe 模型，config/ 是本机点位，都不进包
SKIP_DIRS = {"__pycache__", "models", "config"}
SKIP_SUFFIX = {".pyc", ".pyo"}


def _wanted(root: Path):
    for p in sorted(root.rglob("*")):
        if not p.is_file():
            continue
        rel = p.relative_to(root)
        if set(rel.parts) & SKIP_DIRS:
            continue
        if p.suffix in SKIP_SUFFIX:
            continue
        yield rel


def main(argv) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="只检查不同步")
    args = ap.parse_args(argv)

    if not DST.is_dir():
        print(f"找不到插件副本目录：{DST}")
        return 2

    added, changed, same = [], [], 0
    for rel in _wanted(SRC):
        s, d = SRC / rel, DST / rel
        if not d.exists():
            added.append(rel)
        elif not filecmp.cmp(s, d, shallow=False):
            changed.append(rel)
        else:
            same += 1

    if not added and not changed:
        print(f"已同步：{same} 个文件一致")
        return 0

    for rel in added:
        print(f"  新增  {rel}")
    for rel in changed:
        print(f"  改动  {rel}")

    if args.check:
        print(f"\n共 {len(added) + len(changed)} 个文件需要同步，跑 python scripts/sync_plugin.py")
        return 1

    for rel in added + changed:
        (DST / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(SRC / rel, DST / rel)
    print(f"\n已同步 {len(added) + len(changed)} 个文件 → {DST}")
    print("记得重新 pnpm pack 才生效")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
