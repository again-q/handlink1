"""实时盯总线：收到帧就打印，攒够 --frames 或超时即退出。

    python scripts/can_watch.py --seconds 300 --frames 30
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from handlink import screw  # noqa: E402
from handlink.hardware import ZlgCan  # noqa: E402

DLL = HERE / "pkg" / "LG-603" / "src" / "main" / "resources" / "native" / "Windows-x86_64" / "ECanVci64.dll"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=300.0)
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--baud", type=int, default=1000)
    ap.add_argument("--wait", type=float, default=120.0, help="等适配器释放的秒数")
    ap.add_argument("--log", default="", help="同时把每帧追加到该文件（便于后台查看）")
    args = ap.parse_args()

    log = None
    if args.log:
        log = Path(args.log)
        log.parent.mkdir(parents=True, exist_ok=True)
        log.write_text("", encoding="utf-8")

    bus = ZlgCan(DLL, device_type=4, device_index=0, channel=0, baud=args.baud)
    deadline = time.time() + args.wait
    while True:
        try:
            bus.open()
            break
        except RuntimeError:
            if time.time() > deadline:
                print("适配器一直打不开", flush=True)
                return 1
            time.sleep(1.0)
    print(f"监听中（{args.seconds:.0f}s，{args.baud}kbps）…", flush=True)

    seen = 0
    started = time.time()
    try:
        while time.time() - started < args.seconds and seen < args.frames:
            for f in bus.read(max_frames=64, wait_ms=50):
                seen += 1
                data = f["data"]
                parsed = screw.parse(data)
                line = (
                    f"{time.time() - started:7.2f}s  ID 0x{f['id']:03X} DLC {f['dlc']} "
                    f"[{data.hex(' ')}]  {parsed}"
                )
                print(line, flush=True)
                if log is not None:
                    with log.open("a", encoding="utf-8") as fh:
                        fh.write(line + "\n")
    finally:
        bus.close()
    print(f"共 {seen} 帧", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
