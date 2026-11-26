"""Inspect a jar without a decompiler.

Usage:
  python jar_scan.py list <jar> [regex]
  python jar_scan.py strings <jar> <entry-substr> [<entry-substr> ...]
  python jar_scan.py pyc <file.pyc>
"""
from __future__ import annotations

import io
import re
import struct
import sys
import zipfile


def entries(jar: str) -> list[str]:
    with zipfile.ZipFile(jar) as z:
        return z.namelist()


def cmd_list(jar: str, pattern: str = "") -> None:
    rx = re.compile(pattern, re.I) if pattern else None
    names = entries(jar)
    hits = [n for n in names if rx is None or rx.search(n)]
    print(f"total entries: {len(names)}  matched: {len(hits)}")
    for n in hits:
        print(" ", n)


# ---------------------------------------------------------------- class parsing
def read_utf8(cp: list[tuple[int, object]], idx: int) -> str:
    tag, val = cp[idx - 1]
    return val if tag == 1 else ""


def read_name(cp: list[tuple[int, object]], idx: int) -> str:
    """Resolve a Class/String entry to its Utf8 text."""
    tag, val = cp[idx - 1]
    if tag in (7, 8):
        return read_utf8(cp, int(val))
    return read_utf8(cp, idx)


def parse_constant_pool(data: bytes, pos: int):
    (count,) = struct.unpack_from(">H", data, pos)
    pos += 2
    cp: list[tuple[int, object]] = []
    i = 1
    while i < count:
        tag = data[pos]
        pos += 1
        if tag == 1:
            (ln,) = struct.unpack_from(">H", data, pos)
            pos += 2
            cp.append((1, data[pos:pos + ln].decode("utf-8", "replace")))
            pos += ln
        elif tag in (3, 4):
            cp.append((tag, struct.unpack_from(">i" if tag == 3 else ">f", data, pos)[0]))
            pos += 4
        elif tag in (5, 6):
            cp.append((tag, struct.unpack_from(">q" if tag == 5 else ">d", data, pos)[0]))
            pos += 8
            cp.append((0, None))
            i += 1
        elif tag in (7, 8, 16, 19, 20):
            cp.append((tag, struct.unpack_from(">H", data, pos)[0]))
            pos += 2
        elif tag in (9, 10, 11, 12, 17, 18):
            cp.append((tag, struct.unpack_from(">HH", data, pos)))
            pos += 4
        elif tag == 15:
            cp.append((tag, (data[pos], struct.unpack_from(">H", data, pos + 1)[0])))
            pos += 3
        else:
            raise ValueError(f"bad cp tag {tag} at {pos}")
        i += 1
    return cp, pos


def skip_attributes(data: bytes, pos: int) -> int:
    (n,) = struct.unpack_from(">H", data, pos)
    pos += 2
    for _ in range(n):
        pos += 2
        (ln,) = struct.unpack_from(">I", data, pos)
        pos += 4 + ln
    return pos


def class_info(data: bytes):
    assert data[:4] == b"\xca\xfe\xba\xbe"
    cp, pos = parse_constant_pool(data, 8)
    _, this_i, super_i = struct.unpack_from(">HHH", data, pos)
    pos += 6
    (nif,) = struct.unpack_from(">H", data, pos)
    pos += 2 + 2 * nif
    (nfields,) = struct.unpack_from(">H", data, pos)
    pos += 2
    for _ in range(nfields):
        pos += 6
        pos = skip_attributes(data, pos)
    (nmethods,) = struct.unpack_from(">H", data, pos)
    pos += 2
    methods = []
    for _ in range(nmethods):
        flags, name_i, desc_i = struct.unpack_from(">HHH", data, pos)
        pos += 6
        (na,) = struct.unpack_from(">H", data, pos)
        pos += 2
        code = None
        for _ in range(na):
            an_i, alen = struct.unpack_from(">HI", data, pos)
            pos += 6
            if read_utf8(cp, an_i) == "Code":
                max_stack, max_locals = struct.unpack_from(">HH", data, pos)
                (clen,) = struct.unpack_from(">I", data, pos + 4)
                code = data[pos + 8:pos + 8 + clen]
            pos += alen
        methods.append((read_utf8(cp, name_i), read_utf8(cp, desc_i), code))
    return cp, read_name(cp, this_i), read_name(cp, super_i), methods


CP_OPS = {0x12: 1, 0x13: 2, 0x14: 2}  # ldc, ldc_w, ldc2_w


def decode_strings(cp, code: bytes) -> list[str]:
    out, i = [], 0
    while i < len(code):
        op = code[i]
        if op in CP_OPS:
            n = CP_OPS[op]
            idx = int.from_bytes(code[i + 1:i + 1 + n], "big")
            tag, val = cp[idx - 1]
            if tag == 8:  # String -> Utf8
                out.append(read_utf8(cp, int(val)))
            i += 1 + n
            continue
        i += 1
    return out


def push_values(code: bytes) -> list[tuple[int, int]]:
    """(offset, value) for bipush/sipush/iconst-like pushes."""
    out, i = [], 0
    while i < len(code):
        op = code[i]
        if op == 0x10:
            out.append((i, struct.unpack_from(">b", code, i + 1)[0]))
            i += 2
        elif op == 0x11:
            out.append((i, struct.unpack_from(">h", code, i + 1)[0]))
            i += 3
        elif 0x02 <= op <= 0x08:
            out.append((i, op - 0x03))
            i += 1
        else:
            i += 1
    return out


def cmd_strings(jar: str, needles: list[str]) -> None:
    with zipfile.ZipFile(jar) as z:
        for name in z.namelist():
            if not name.endswith(".class"):
                continue
            if needles and not any(n.lower() in name.lower() for n in needles):
                continue
            try:
                cp, this, sup, methods = class_info(z.read(name))
            except Exception as exc:  # noqa: BLE001
                print(f"!! {name}: {exc}")
                continue
            print(f"\n=== {this}  (super {sup}) ===")
            for mname, mdesc, code in methods:
                if code is None:
                    print(f"  -- {mname}{mdesc}")
                    continue
                ss = decode_strings(cp, code)
                pushes = [v for _, v in push_values(code) if 0 <= v <= 255]
                print(f"  -- {mname}{mdesc}  len={len(code)}")
                if ss:
                    print(f"     strings: {ss}")
                if org := [v for v in pushes if v not in (0, 1, 2, 3, 4, 5)]:
                    print(f"     consts : {org}")


def cmd_pyc(path: str) -> None:
    import dis
    import marshal

    raw = open(path, "rb").read()
    code = marshal.loads(raw[16:])
    dis.dis(code)


def main(argv: list[str]) -> int:
    if len(argv) < 3:
        print(__doc__)
        return 2
    cmd, jar = argv[1], argv[2]
    if cmd == "list":
        cmd_list(jar, argv[3] if len(argv) > 3 else "")
    elif cmd == "strings":
        cmd_strings(jar, argv[3:])
    elif cmd == "pyc":
        cmd_pyc(jar)
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
