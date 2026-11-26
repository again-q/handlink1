"""探测 GCAN USBCAN 适配器：找对 DLL、设备类型和通道，只读听帧。

    python scripts/can_probe.py            # 探测
    python scripts/can_probe.py --listen 5 # 听 5 秒

不发任何报文。
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.wintypes as wt
import struct
import sys
import time
from pathlib import Path

CANDIDATES = [
    Path(r"C:\Program Files\GCANTools\ECanUsb.dll"),
    Path(r"C:\Program Files\GCANTools\ECANFDVCI64.dll"),
    Path(r"C:\Program Files\GCANTools\drv\ECanUsb.dll"),
]

PKG_DLL = Path(__file__).resolve().parent.parent / "pkg" / "LG-603" / "src" / "main" / "resources" / "native" / "Windows-x86_64" / "ECanVci64.dll"
if PKG_DLL.is_file():
    CANDIDATES.insert(0, PKG_DLL)


class INIT_CONFIG(ctypes.Structure):
    _fields_ = [("AccCode", wt.DWORD), ("AccMask", wt.DWORD), ("Reserved", wt.DWORD),
                ("Filter", wt.BYTE), ("Timing0", wt.BYTE), ("Timing1", wt.BYTE), ("Mode", wt.BYTE)]


class CAN_OBJ(ctypes.Structure):
    _fields_ = [("ID", wt.DWORD), ("TimeStamp", wt.DWORD), ("TimeFlag", wt.BYTE),
                ("SendType", wt.BYTE), ("RemoteFlag", wt.BYTE), ("ExternFlag", wt.BYTE),
                ("DataLen", wt.BYTE), ("Data", wt.BYTE * 8), ("Reserved", wt.BYTE * 3)]


def exports(path: Path):
    try:
        data = path.read_bytes()
    except OSError:
        return []
    e = struct.unpack_from("<I", data, 0x3C)[0]
    nsec = struct.unpack_from("<H", data, e + 6)[0]
    opt = struct.unpack_from("<H", data, e + 20)[0]
    magic = struct.unpack_from("<H", data, e + 24)[0]
    dd = e + 24 + (112 if magic == 0x20B else 96)
    exp_rva, _ = struct.unpack_from("<II", data, dd)
    secs, so = [], e + 24 + opt
    for i in range(nsec):
        vs, va, rs, ra = struct.unpack_from("<IIII", data, so + i * 40 + 8)
        secs.append((va, vs, ra, rs))
    if not exp_rva:
        return []
    def off(rva):
        for va, vs, ra, rs in secs:
            if va <= rva < va + max(vs, rs):
                return ra + (rva - va)
    o = off(exp_rva)
    if o is None:
        return []
    n = struct.unpack_from("<I", data, o + 24)[0]
    names_rva = struct.unpack_from("<I", data, o + 32)[0]
    no = off(names_rva)
    out = []
    for i in range(n):
        rva = struct.unpack_from("<I", data, no + i * 4)[0]
        p = off(rva)
        end = data.index(b"\0", p)
        out.append(data[p:end].decode("latin1"))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", type=float, default=2.0)
    ap.add_argument("--baud", type=int, default=1000)
    args = ap.parse_args()

    timing = {1000: (0x00, 0x14), 500: (0x00, 0x1C), 250: (0x01, 0x1C)}[args.baud]
    for dll_path in CANDIDATES:
        if not dll_path.is_file():
            continue
        print(f"\n=== {dll_path} ===")
        names = exports(dll_path)
        print("  exports:", ", ".join(names[:14]) + ("…" if len(names) > 14 else ""))
        try:
            dll = ctypes.WinDLL(str(dll_path))
        except OSError as exc:
            print("  加载失败:", exc)
            continue

        fn = {}
        for key in ("OpenDevice", "InitCAN", "StartCAN", "Receive", "CloseDevice", "ResetCAN"):
            for cand in (key, "VCI_" + key, "ECanVci_" + key):
                if hasattr(dll, cand):
                    fn[key] = getattr(dll, cand)
                    break
        if "OpenDevice" not in fn:
            print("  找不到 OpenDevice")
            continue

        opened = None
        for devtype in (4, 3, 2, 1, 21, 20):
            try:
                rc = fn["OpenDevice"](devtype, 0, 0)
            except Exception as exc:
                print(f"  OpenDevice({devtype}) 异常: {exc}")
                continue
            print(f"  OpenDevice(type={devtype}) -> {rc}")
            if rc == 1:
                opened = devtype
                break
        if opened is None:
            print("  这个 DLL 打不开设备")
            continue

        print(f"  ✅ 用设备类型 {opened} 打开了")
        for ch in (0, 1):
            cfg = INIT_CONFIG(0, 0xFFFFFFFF, 0, 0, *timing, 0)
            rc = fn["InitCAN"](opened, 0, ch, ctypes.byref(cfg))
            print(f"  InitCAN(ch={ch}) -> {rc}")
            if rc != 1:
                continue
            print(f"  StartCAN(ch={ch}) -> {fn['StartCAN'](opened, 0, ch)}")
            buf = (CAN_OBJ * 64)()
            seen = {}
            deadline = time.time() + args.listen
            while time.time() < deadline:
                n = fn["Receive"](opened, 0, ch, buf, 64, 50)
                for i in range(max(0, n)):
                    o = buf[i]
                    seen.setdefault(o.ID & 0x1FFFFFFF, 0)
                    seen[o.ID & 0x1FFFFFFF] += 1
            print(f"  {args.listen}s 内收到 {sum(seen.values())} 帧，ID:",
                  {hex(k): v for k, v in sorted(seen.items())} or "无")
            fn.get("ResetCAN", lambda *a: None)(opened, 0, ch)
        fn["CloseDevice"](opened, 0)
        return 0
    print("\n没有可用的 DLL")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
