# -*- coding: utf-8 -*-
"""把汇报提纲.md 转成可打印的 docx（A4，微软雅黑，带页码）。"""
import re
import sys

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

SRC = r"C:\Users\WaterAITech01\Downloads\机械手实验\汇报提纲-2026-11-19.md"
DST = r"C:\Users\WaterAITech01\Downloads\机械手实验\汇报提纲-2026-11-19.docx"

NAVY = RGBColor(0x0F, 0x27, 0x40)
INK = RGBColor(0x1E, 0x2A, 0x36)
MUTED = RGBColor(0x60, 0x6E, 0x80)
ACCENT = RGBColor(0x1F, 0x6F, 0xEB)
FONT = "微软雅黑"


def set_run_font(run, size=None, bold=None, color=None, font=FONT):
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if color is not None:
        run.font.color.rgb = color
    run.font.name = font
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    rfonts.set(qn("w:eastAsia"), font)
    rfonts.set(qn("w:ascii"), font)
    rfonts.set(qn("w:hAnsi"), font)


def add_inline(par, text, size=10.5, color=INK, base_bold=False):
    """处理 **加粗** 与 `代码`。"""
    for piece in re.split(r"(\*\*[^*]+\*\*|`[^`]+`)", text):
        if not piece:
            continue
        if piece.startswith("**") and piece.endswith("**"):
            run = par.add_run(piece[2:-2])
            set_run_font(run, size, True, color)
        elif piece.startswith("`") and piece.endswith("`"):
            run = par.add_run(piece[1:-1])
            set_run_font(run, size - 0.5, base_bold, RGBColor(0x1B, 0x4B, 0x7A), "Consolas")
        else:
            run = par.add_run(piece)
            set_run_font(run, size, base_bold, color)


def shade(cell, hex_color):
    tcpr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_color)
    tcpr.append(shd)


def cell_margins(table, top=60, bottom=60, left=100, right=100):
    tblpr = table._tbl.tblPr
    mar = OxmlElement("w:tblCellMar")
    for tag, val in (("top", top), ("left", left), ("bottom", bottom), ("right", right)):
        el = OxmlElement(f"w:{tag}")
        el.set(qn("w:w"), str(val))
        el.set(qn("w:type"), "dxa")
        mar.append(el)
    tblpr.append(mar)


def h1(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(18)
    p.paragraph_format.space_after = Pt(6)
    add_inline(p, text, 19, NAVY, True)
    # 标题下的细线
    pPr = p._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), "8")
    bottom.set(qn("w:space"), "4")
    bottom.set(qn("w:color"), "1F6FEB")
    pbdr.append(bottom)
    pPr.append(pbdr)


def h2(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(14)
    p.paragraph_format.space_after = Pt(4)
    p.paragraph_format.keep_with_next = True
    add_inline(p, text, 13.5, NAVY, True)


def h3(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after = Pt(2)
    p.paragraph_format.keep_with_next = True
    add_inline(p, text, 11.5, ACCENT, True)


def body(doc, text, indent=0.0):
    p = doc.add_paragraph()
    fmt = p.paragraph_format
    fmt.space_before = Pt(2)
    fmt.space_after = Pt(4)
    fmt.line_spacing = 1.45
    fmt.left_indent = Cm(indent)
    add_inline(p, text)


def bullet(doc, text, numbered=False, idx=0, mark=None):
    p = doc.add_paragraph()
    fmt = p.paragraph_format
    fmt.left_indent = Cm(0.75)
    fmt.first_line_indent = Cm(-0.45)
    fmt.space_before = Pt(1)
    fmt.space_after = Pt(2)
    fmt.line_spacing = 1.4
    if mark is None:
        mark = f"{idx}. " if numbered else "·  "
    if mark == "□  ":
        fmt.space_before = Pt(0)
        fmt.space_after = Pt(1)
        fmt.line_spacing = 1.3
    run = p.add_run(mark)
    set_run_font(run, 10.5, True, ACCENT if mark != "□  " else MUTED)
    add_inline(p, text)


def quote(doc, text):
    p = doc.add_paragraph()
    fmt = p.paragraph_format
    fmt.left_indent = Cm(0.5)
    fmt.space_before = Pt(4)
    fmt.space_after = Pt(4)
    fmt.line_spacing = 1.4
    pPr = p._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr")
    left = OxmlElement("w:left")
    left.set(qn("w:val"), "single")
    left.set(qn("w:sz"), "18")
    left.set(qn("w:space"), "8")
    left.set(qn("w:color"), "1F6FEB")
    pbdr.append(left)
    pPr.append(pbdr)
    add_inline(p, text, 10.5, RGBColor(0x24, 0x3B, 0x53), True)


def hr(doc):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(8)
    p.paragraph_format.space_after = Pt(2)
    pPr = p._p.get_or_add_pPr()
    pbdr = OxmlElement("w:pBdr")
    bottom = OxmlElement("w:bottom")
    bottom.set(qn("w:val"), "single")
    bottom.set(qn("w:sz"), "4")
    bottom.set(qn("w:space"), "1")
    bottom.set(qn("w:color"), "D5DCE6")
    pbdr.append(bottom)
    pPr.append(pbdr)


def table(doc, rows):
    cols = len(rows[0])
    t = doc.add_table(rows=len(rows), cols=cols)
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    cell_margins(t)
    layout = OxmlElement("w:tblLayout")
    layout.set(qn("w:type"), "fixed")
    t._tbl.tblPr.append(layout)
    if cols == 2:
        widths = [Cm(3.4), Cm(13.0)]
    elif cols == 3:
        widths = [Cm(3.2), Cm(6.0), Cm(7.2)]
    else:
        widths = [Cm(16.4 / cols)] * cols
    for r_i, row in enumerate(rows):
        for c_i, text in enumerate(row):
            cell = t.cell(r_i, c_i)
            cell.width = widths[c_i]
            par = cell.paragraphs[0]
            par.paragraph_format.space_before = Pt(1)
            par.paragraph_format.space_after = Pt(1)
            par.paragraph_format.line_spacing = 1.25
            clean = text.replace("**", "")
            run = par.add_run(clean)
            if r_i == 0:
                set_run_font(run, 9.5, True, NAVY)
                shade(cell, "EEF3FA")
            else:
                set_run_font(run, 9.5, False, INK)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def add_page_number(par):
    run = par.add_run()
    for instr in ("begin", "PAGE", "end"):
        el = OxmlElement("w:fldChar") if instr != "PAGE" else OxmlElement("w:instrText")
        if instr == "PAGE":
            el.set(qn("xml:space"), "preserve")
            el.text = " PAGE "
        else:
            el.set(qn("w:fldCharType"), instr)
        run._r.append(el)
    set_run_font(run, 8.5, False, MUTED)


def build():
    doc = Document()
    sec = doc.sections[0]
    sec.page_width = Cm(21.0)
    sec.page_height = Cm(29.7)
    sec.top_margin = Cm(2.0)
    sec.bottom_margin = Cm(1.8)
    sec.left_margin = Cm(2.3)
    sec.right_margin = Cm(2.3)

    normal = doc.styles["Normal"]
    normal.font.name = FONT
    normal.font.size = Pt(10.5)
    normal.element.rPr.rFonts.set(qn("w:eastAsia"), FONT)

    # 页脚：左标题 + 右页码
    footer = sec.footer
    fp = footer.paragraphs[0]
    fp.paragraph_format.tab_stops.add_tab_stop(Cm(16.4), WD_ALIGN_PARAGRAPH.RIGHT)
    r = fp.add_run("智衍 AI 社 · 机械臂与灵巧手项目组")
    set_run_font(r, 8.5, False, MUTED)
    r2 = fp.add_run("\t")
    set_run_font(r2, 8.5, False, MUTED)
    add_page_number(fp)

    with open(SRC, encoding="utf-8") as fh:
        lines = fh.read().splitlines()

    i = 0
    ordered = 0
    while i < len(lines):
        line = lines[i].rstrip()
        if not line.strip():
            ordered = 0
            i += 1
            continue
        if line.strip() == "<!-- pagebreak -->":
            doc.add_page_break()
            i += 1
            continue
        elif line.startswith("| "):
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                raw = lines[i].strip().strip("|")
                cells = [c.strip() for c in raw.split("|")]
                if not all(re.fullmatch(r":?-{2,}:?", c) for c in cells):
                    rows.append(cells)
                i += 1
            if rows:
                table(doc, rows)
            continue
        if line.startswith("---"):
            hr(doc)
        elif line.startswith("### "):
            h3(doc, line[4:])
        elif line.startswith("## "):
            h2(doc, line[3:])
        elif line.startswith("# "):
            h1(doc, line[2:])
        elif line.startswith("> "):
            quote(doc, line[2:])
        elif re.match(r"^\d+\.\s", line):
            ordered += 1
            bullet(doc, re.sub(r"^\d+\.\s+", "", line), numbered=True, idx=ordered)
        elif line.startswith("- [ ] "):
            bullet(doc, line[6:], mark="□  ")
        elif line.startswith("- "):
            ordered = 0
            bullet(doc, line[2:])
        elif line.startswith("* "):
            ordered = 0
            bullet(doc, line[2:])
        else:
            body(doc, line)
        i += 1

    doc.save(DST)
    print("saved", DST)


if __name__ == "__main__":
    build()
