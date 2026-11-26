"""只读抓取 GCAN USBCAN 上的帧，打印并落 JSONL。

    python scripts/can_capture.py --seconds 5 --dump 20
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.wintypes as wt
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))

from scripts.can_probe import CAN_OBJ, INIT_CONFIG, exports  # noqa: E402

DLL = HERE / "pkg" / "LG-603" / "src" / "main" / "resources" / "native" / "Windows-x86_64" / "ECanVci64.dll"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--dump", type=int, default=20)
    ap.add_argument("--channel", type=int, default=0)
    ap.add_argument("--device-type", type=int, default=4)
    ap.add_argument("--baud", type=int, default=1000)
    ap.add_argument("--out", default="demo/can-raw.jsonl")
    args = ap.parse_args()

    timing = {1000: (0x00, 0x14), 500: (0x00, 0x1C), 250: (0x01, 0x1C)}[args.baud]
    dll = ctypes.WinDLL(str(DLL))
    assert dll.OpenDevice(args.device_type, 0, 0) == 1, "打不开适配器"
    cfg = INIT_CONFIG(0, 0xFFFFFFFF, 0, 0, *timing, 0)
    assert dll.InitCAN(args.device_type, 0, args.channel, ctypes.byref(cfg)) == 1, "InitCAN 失败"
    assert dll.StartCAN(args.device_type, 0, args.channel) == 1, "StartCAN 失败"

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    buf = (CAN_OBJ * 64)()
    seen = {}
    dumped = 0
    started = time.time()
    with out.open("w", encoding="utf-8") as fh:
        while time.time() - started < args.seconds:
            n = dll.Receive(args.device_type, 0, args.channel, buf, 64, 50)
            for i in range(max(0, n)):
                o = buf[i]
                can_id = o.ID & 0x1FFFFFFF
                data = bytes(o.Data[: o.DataLen])
                seen[can_id] = seen.get(can_id, 0) + 1
                fh.write(json.dumps({"t": round(time.time() - started, 4), "id": can_id,
                                     "ext": bool(o.ID & 0x80000000), "dlc": o.DataLen,
                                     "data": data.hex(" ")}) + "\n")
                if dumped < args.dump:
                    dumped += 1
                    print(f"{time.time() - started:7.3f}  ID 0x{can_id:03X}  DLC {o.DataLen}  "
                          f"[{data.hex(' ')}]  | {' '.join(f'{b:3d}' for b in data)}")

    total = sum(seen.values())
    print(f"\n{args.seconds}s 共 {total} 帧，{len(seen)} 个 ID")
    for can_id, count in sorted(seen.items(), key=lambda kv: -kv[1]):
        print(f"  0x{can_id:03X} ({can_id:>3})  {count:>5} 帧  ≈{count / args.seconds:5.1f}/s")
    print(f"已写入 {out}")
    dll.ResetCAN(args.device_type, 0, args.channel)
    dll.CloseDevice(args.device_type, 0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
