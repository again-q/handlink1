"""魔方走步工具：解析、取反、只保留正向 90°、按面分组。

这里**不建 54 格状态模型**（那是求解器的事）；只处理“走步序列”，给
“手只推正向 90°、每换一个面要重抓一次”的规划用。

术语：

* 一个 move 是 ``(face, power)``，face ∈ ``U R F D L B``，power ∈ 1/2/3；
  power=3 就是逆时针 90°（等价于 ``U'``），手仍然只做正向推，推三次。
* segment 是同一面的连续转动：``("U", 3)`` 表示 U U U。
* 换面次数 = segment 数 - 1（第一段如果已经朝上则不用换）。

命令行自测::

    python -m handlink.cube "R U R' U'"
"""
from __future__ import annotations

import sys
from typing import Iterable, List, Sequence, Tuple

FACES = "URFDLB"
Move = Tuple[str, int]

_PRIMES = ("'", "’", "′")


def parse_moves(text: str) -> List[Move]:
    """``"R U R' U2"`` → ``[("R",1),("U",1),("R",3),("U",2)]``。

    允许空白分隔；也允许 ``R2``、``R'``、``R’``；不支持的记号直接报错。
    """
    moves: List[Move] = []
    for token in text.replace(",", " ").split():
        face = token[0].upper()
        if face not in FACES:
            raise ValueError(f"无法识别的走步 {token!r}（面必须是 U/R/F/D/L/B）")
        rest = token[1:]
        if rest == "":
            power = 1
        elif rest == "2":
            power = 2
        elif rest in _PRIMES:
            power = 3
        else:
            raise ValueError(f"无法识别的走步 {token!r}（只支持 U、U2、U'）")
        moves.append((face, power))
    return moves


def format_moves(moves: Iterable[Move]) -> str:
    """统一用正向写法：``U`` / ``U2`` / ``U3``（U3 就是 U'）。"""
    out = []
    for face, power in moves:
        power = int(power) % 4
        if power == 0:
            continue
        out.append(face if power == 1 else f"{face}{power}")
    return " ".join(out)


def invert(moves: Sequence[Move]) -> List[Move]:
    """逆序列：倒序 + 每一步取反（1↔3，2→2）。"""
    out: List[Move] = []
    for face, power in reversed(list(moves)):
        inv = (4 - int(power) % 4) % 4
        if inv:
            out.append((face, inv))
    return out


def group_by_face(moves: Sequence[Move]) -> List[Move]:
    """合并相邻同面转动：``R R' U U U`` → ``U3``；power 为 0 的丢掉。"""
    out: List[Move] = []
    for face, power in moves:
        power = int(power) % 4
        if power == 0:
            continue
        if out and out[-1][0] == face:
            merged = (out[-1][1] + power) % 4
            if merged:
                out[-1] = (face, merged)
            else:
                out.pop()
        else:
            out.append((face, power))
    return out


def segments(moves: Sequence[Move]) -> List[Move]:
    """按面切段；每段是一组连续的正向 90°。"""
    return group_by_face(moves)


def quarter_turns(moves: Sequence[Move]) -> int:
    """总正向 90° 次数（U2 算 2 次，U3 算 3 次）。"""
    return sum(int(power) % 4 for _, power in moves)


def regrasps(moves: Sequence[Move], first_face: str | None = None) -> int:
    """要换几次面。

    默认 ``first_face=None``：假设可以先把魔方摆成第一段的面朝上，
    所以是“段数 - 1”。如果指定了当前朝上的面且它不是第一段的面，
    第一段也要换一次，结果就是段数。
    """
    segs = segments(moves)
    if not segs:
        return 0
    if first_face is None or first_face == segs[0][0]:
        return len(segs) - 1
    return len(segs)


def estimate_seconds(
    moves: Sequence[Move],
    turn_s: float = 3.0,
    regrasp_s: float = 20.0,
    first_face: str | None = None,
) -> dict:
    """粗估时间：每次 90° 推 turn_s 秒，每次换面 regrasp_s 秒。"""
    turns = quarter_turns(moves)
    changes = regrasps(moves, first_face=first_face)
    return {
        "quarter_turns": turns,
        "segments": len(segments(moves)),
        "regrasps": changes,
        "turn_s": round(turns * turn_s, 1),
        "regrasp_s": round(changes * regrasp_s, 1),
        "total_s": round(turns * turn_s + changes * regrasp_s, 1),
    }


def _main(argv: Sequence[str]) -> int:
    text = " ".join(argv) if argv else "R U R' U'"
    moves = parse_moves(text)
    print(f"输入 : {text}")
    print(f"规范 : {format_moves(moves)}")
    print(f"取反 : {format_moves(invert(moves))}")
    print(f"分段 : {segments(moves)}")
    print(f"估算 : {estimate_seconds(moves)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv[1:]))
