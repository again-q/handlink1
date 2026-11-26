"""Linear bytecode walker: dump strings / calls / constants of a class in a jar.

Usage:
  python classdump.py <jar> <class-substr> [--all]

No decompiler needed: we walk the instruction stream with correct operand
lengths, then print an ordered event list per method. Good enough to reverse
a CAN frame layout.
"""
from __future__ import annotations

import struct
import sys
import zipfile

# ---------------------------------------------------------------- cp parsing
def parse_cp(d: bytes, pos: int):
    (count,) = struct.unpack_from(">H", d, pos)
    pos += 2
    cp: list[tuple[int, object]] = [(0, None)]  # 1-based
    i = 1
    while i < count:
        tag = d[pos]
        pos += 1
        if tag == 1:
            (ln,) = struct.unpack_from(">H", d, pos)
            pos += 2
            cp.append((1, d[pos:pos + ln].decode("utf-8", "replace")))
            pos += ln
        elif tag in (3, 4):
            cp.append((tag, struct.unpack_from(">i" if tag == 3 else ">f", d, pos)[0]))
            pos += 4
        elif tag in (5, 6):
            cp.append((tag, struct.unpack_from(">q" if tag == 5 else ">d", d, pos)[0]))
            pos += 8
            cp.append((0, None))
            i += 1
        elif tag in (7, 8, 16, 19, 20):
            cp.append((tag, struct.unpack_from(">H", d, pos)[0]))
            pos += 2
        elif tag in (9, 10, 11, 12, 17, 18):
            cp.append((tag, struct.unpack_from(">HH", d, pos)))
            pos += 4
        elif tag == 15:
            cp.append((tag, (d[pos], struct.unpack_from(">H", d, pos + 1)[0])))
            pos += 3
        else:
            raise ValueError(f"cp tag {tag} @ {pos}")
        i += 1
    return cp, pos


def utf8(cp, idx: int) -> str:
    if idx <= 0 or idx >= len(cp):
        return ""
    tag, val = cp[idx]
    return val if tag == 1 else ""


def name_of(cp, idx: int) -> str:
    tag, val = cp[idx]
    if tag in (7, 8):
        return utf8(cp, int(val))
    return utf8(cp, idx)


def cls_of(cp, idx: int) -> str:
    tag, val = cp[idx]
    if tag == 7:
        return utf8(cp, int(val))
    return utf8(cp, idx)


def ref_of(cp, idx: int) -> str:
    """Methodref/Fieldref -> 'owner.name:desc'"""
    tag, val = cp[idx]
    if tag in (9, 10, 11):
        ci, nti = val
        tag2, ntval = cp[nti]
        n_i, d_i = ntval
        return f"{cls_of(cp, ci)}.{utf8(cp, n_i)}:{utf8(cp, d_i)}"
    return utf8(cp, idx)


def skip_attrs(d: bytes, pos: int):
    (n,) = struct.unpack_from(">H", d, pos)
    pos += 2
    for _ in range(n):
        (an,) = struct.unpack_from(">H", d, pos)
        (ln,) = struct.unpack_from(">I", d, pos + 2)
        pos += 6 + ln
    return pos


def parse_class(d: bytes):
    cp, pos = parse_cp(d, 8)
    _, this_i, super_i = struct.unpack_from(">HHH", d, pos)
    pos += 6
    (nif,) = struct.unpack_from(">H", d, pos)
    pos += 2 + 2 * nif
    (nf,) = struct.unpack_from(">H", d, pos)
    pos += 2
    for _ in range(nf):
        pos = skip_attrs(d, pos + 6)
    (nm,) = struct.unpack_from(">H", d, pos)
    pos += 2
    methods = []
    for _ in range(nm):
        flags, n_i, d_i = struct.unpack_from(">HHH", d, pos)
        pos += 6
        (na,) = struct.unpack_from(">H", d, pos)
        pos += 2
        code = None
        for _ in range(na):
            an_i, alen = struct.unpack_from(">HI", d, pos)
            body = pos + 6
            if utf8(cp, an_i) == "Code":
                (clen,) = struct.unpack_from(">I", d, body + 4)
                code = d[body + 8:body + 8 + clen]
            pos = body + alen
        methods.append((utf8(cp, n_i), utf8(cp, d_i), code))
    return cp, cls_of(cp, this_i), cls_of(cp, super_i), methods


# ---------------------------------------------------------------- opcode sizes
def instr_len(code: bytes, i: int) -> int:
    op = code[i]
    if op in (0x10, 0x12, 0x15, 0x16, 0x17, 0x18, 0x19, 0x36, 0x37, 0x38, 0x39,
              0x3a, 0xa9, 0xbc):
        return 2
    if op in (0x11, 0x13, 0x14, 0x84, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
              0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xb2, 0xb3,
              0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xbb, 0xbd, 0xc0, 0xc1, 0xc6, 0xc7):
        return 3
    if op in (0xb9, 0xba, 0xc8, 0xc9):
        return 5
    if op == 0xc5:
        return 4
    if op == 0xc4:
        return 6 if code[i + 1] == 0x84 else 4
    if op == 0xaa:  # tableswitch
        base = i + 1
        pad = (4 - (base % 4)) % 4
        p = base + pad
        low, high = struct.unpack_from(">ii", code, p + 4)
        return p + 12 + 4 * (high - low + 1) - i
    if op == 0xab:  # lookupswitch
        base = i + 1
        pad = (4 - (base % 4)) % 4
        p = base + pad
        (npairs,) = struct.unpack_from(">i", code, p + 4)
        return p + 8 + 8 * npairs - i
    return 1


def walk(cp, code: bytes):
    """Yield ('str'|'push'|'call'|'field', text) in bytecode order."""
    i = 0
    n = len(code)
    while i < n:
        op = code[i]
        try:
            ln = instr_len(code, i)
        except Exception:  # noqa: BLE001
            return
        if ln <= 0:
            return
        if op in (0x12, 0x13):  # ldc / ldc_w
            idx = code[i + 1] if op == 0x12 else struct.unpack_from(">H", code, i + 1)[0]
            tag, val = cp[idx]
            if tag == 8:
                yield ("str", utf8(cp, int(val)))
            elif tag == 3:
                yield ("push", int(val))
        elif op == 0x14:
            idx = struct.unpack_from(">H", code, i + 1)[0]
            tag, val = cp[idx]
            if tag == 5:
                yield ("push", int(val))
        elif op == 0x10:
            yield ("push", struct.unpack_from(">b", code, i + 1)[0])
        elif op == 0x11:
            yield ("push", struct.unpack_from(">h", code, i + 1)[0])
        elif 0x02 <= op <= 0x08:
            yield ("push", op - 0x03)
        elif op in (0xb6, 0xb7, 0xb8, 0xb9):
            idx = struct.unpack_from(">H", code, i + 1)[0]
            yield ("call", ref_of(cp, idx))
        elif op in (0xb2, 0xb3, 0xb4, 0xb5):
            idx = struct.unpack_from(">H", code, i + 1)[0]
            yield ("field", ref_of(cp, idx))
        i += ln


def main(argv: list[str]) -> int:
    jar, needle = argv[1], argv[2]
    show_all = "--all" in argv
    with zipfile.ZipFile(jar) as z:
        names = [n for n in z.namelist() if n.endswith(".class") and needle in n]
        for name in sorted(names):
            cp, this, sup, methods = parse_class(z.read(name))
            print(f"\n===== {this}  (super {sup}) =====")
            for mname, mdesc, code in methods:
                if code is None:
                    continue
                events = list(walk(cp, code))
                if not events:
                    continue
                print(f"\n  --- {mname}{mdesc}")
                buf = []
                for kind, val in events:
                    if kind == "str":
                        buf.append(f'"{val}"')
                    elif kind == "call":
                        buf.append(f"->{val.split('/')[-1]}")
                    elif kind == "field":
                        buf.append(f"#{val.split('/')[-1]}")
                    elif show_all:
                        buf.append(f"[{val}]")
                print("     " + " ".join(buf))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
