"""
Drawing. Every screen is built from the same grid:

  row 0     status bar: state, link, camera, AP, e-stop, clock
  rows 1-17 BED MAP (62 cols) + right column (38 cols)
  rows 18-27 JOB (62) + UART (38), or a full-width panel
  row -2    key bar for the current screen
  row -1    command line, prompts and one-line errors

Drawing goes through a Canvas so tests can render to text. Roles are the
theme.py names. Set canvas.dim to draw everything in the dimmed role
(the dashboard behind the e-stop modal).
"""

import os
import platform
import time

import commands
from app import CALIB_FIELDS, CONFIG_SECTIONS, STEP_FIELDS
from boot_art import ART, RIGHT_COL, TOP_ROW
from controller import mmss
from model import MAP_COLS, MAP_ROWS
from switches import NAMES as SWITCH_NAMES

FULL_W, FULL_H = 100, 30
MIN_W, MIN_H = 80, 24
LEFT_W, RIGHT_W = 62, 38
NAME = "GANTRY CLI"


class Canvas:
    """A text grid; also the base for the curses canvas."""

    def __init__(self, h: int, w: int):
        self.h, self.w = h, w
        self.dim = False
        self.cells = [[(" ", "t") for _ in range(w)] for _ in range(h)]

    def put(self, y: int, x: int, text: str, role: str = "t") -> int:
        if self.dim:
            role = "z"
        if 0 <= y < self.h:
            for i, ch in enumerate(text):
                if 0 <= x + i < self.w:
                    self._set(y, x + i, ch, role)
        return x + len(text)

    def _set(self, y, x, ch, role):
        self.cells[y][x] = (ch, role)

    def segs(self, y: int, x: int, parts, width: int | None = None, fill_role: str = "t") -> int:
        start = x
        for text, role in parts:
            if width is not None:
                room = start + width - x
                if room <= 0:
                    break
                text = text[:room]
            x = self.put(y, x, text, role)
        if width is not None and x < start + width:
            x = self.put(y, x, " " * (start + width - x), fill_role)
        return x

    def text(self) -> str:
        return "\n".join("".join(c for c, _ in row).rstrip() for row in self.cells)

    def roles(self, y: int) -> str:
        return "".join(r[0] for _, r in self.cells[y])


class CursesCanvas(Canvas):
    def __init__(self, scr, attr):
        h, w = scr.getmaxyx()
        self.scr, self.attr = scr, attr
        self.h, self.w = h, w
        self.dim = False

    def put(self, y: int, x: int, text: str, role: str = "t") -> int:
        if self.dim:
            role = "z"
        if 0 <= y < self.h and x < self.w:
            if x < 0:
                text, x0 = text[-x:], 0
            else:
                x0 = x
            text = text[: self.w - x0]
            try:
                self.scr.addstr(y, x0, text, self.attr(role))
            except Exception:
                pass    # writing the bottom-right cell always "fails"
        return x + len(text)


# --- small helpers ----------------------------------------------------
def hms(t: float | None) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t)) if t else "--:--:--"


def bar(frac: float, width: int, on_role: str) -> list:
    n = max(0, min(width, round(frac * width)))
    return [("█" * n, on_role), ("░" * (width - n), "f")]


def box(c: Canvas, y: int, x: int, h: int, w: int, title: str | None = None,
        right: str | None = None, role: str = "f", title_role: str = "h", right_role: str = "d") -> None:
    parts = [("┌─", role)]
    used = 3
    if title:
        parts.append((f" {title} ", title_role))
        used += len(title) + 2
    rt = f" {right} " if right else ""
    parts.append(("─" * max(0, w - used - len(rt)), role))
    if rt:
        parts.append((rt, right_role))
    parts.append(("┐", role))
    c.segs(y, x, parts)
    for yy in range(y + 1, y + h - 1):
        c.put(yy, x, "│", role)
        c.put(yy, x + w - 1, "│", role)
    c.put(y + h - 1, x, "└" + "─" * (w - 2) + "┘", role)


def sep(c: Canvas, y: int, x: int, w: int, title: str | None = None, role: str = "f") -> None:
    parts = [("├─", role)]
    used = 3
    if title:
        parts.append((f" {title} ", "h"))
        used += len(title) + 2
    parts.append(("─" * (w - used) + "┤", role))
    c.segs(y, x, parts)


def clear(c: Canvas, y: int, x: int, h: int, w: int) -> None:
    for yy in range(y, y + h):
        c.put(yy, x, " " * w)


def line(c: Canvas, y: int, x: int, w: int, parts) -> None:
    """One panel row: parts written from x, padded/clipped to w."""
    c.segs(y, x, parts, width=w)


def short_path(p: str) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.relpath(p, here) if os.path.abspath(p).startswith(here) else p


def homed_all(g) -> bool:
    return all(a.homed for a in g.axes.values())


# --- screen entry -----------------------------------------------------
def draw(app, c: Canvas) -> None:
    if c.w < MIN_W or c.h < MIN_H:
        c.put(0, 0, f"terminal too small ({c.w}×{c.h}) · needs {MIN_W}×{MIN_H} · SPACE is still E-STOP"[: c.w], "a")
        return
    g = app.g
    compact = c.w < FULL_W or c.h < FULL_H
    status_bar(app, c)
    if app.mode == "boot":
        boot_screen(app, c)
    elif app.mode == "log":
        log_screen(app, c)
    elif app.mode == "config":
        config_screen(app, c)
    elif app.mode == "jog" and not g.estop:
        jog_screen(app, c)
    elif app.mode == "review" and not g.estop:
        review_screen(app, c)
    elif app.mode == "calib" and not g.estop:
        calib_screen(app, c)
    else:
        c.dim = g.estop
        dashboard(app, c, compact)
        c.dim = False
        if g.estop:
            estop_modal(app, c)
    if app.cmd is not None and app.mode != "boot":
        commands_popup(app, c)
    if app.help_open:
        help_overlay(app, c)
    key_bar(app, c)
    command_line(app, c)


# --- status bar -------------------------------------------------------
def status_bar(app, c: Canvas) -> None:
    g = app.g
    label, role = app.state_label()
    left = [(" ", "inv"), (NAME, "invb"), ("  ", "inv"), ("▸", role), (f" {label} ", role)]
    if g.link_lost:
        esp = [("●", "invr"), (" esp32 LOST  ", "inv")]
    elif g.latency_ms is None:
        esp = [("○", "inv"), (" esp32 ----  ", "inv")]
    else:
        esp = [("●", "invg"), (f" esp32 {g.latency_ms}ms  ", "inv")]
    sw_ok = g.switches.available and not g.switch_fault()
    sw = [("●", "invg") if sw_ok else ("○", "invr"), (" sw  ", "inv")]
    cam = [("●", "invg") if g.vision.camera else ("○", "inv"), (" cam  ", "inv")]
    clients = app.ap_clients_count()
    ap_up = app.service(g.s.ap_service)
    ap = [("●", "invg") if ap_up else ("○", "inv"), (f" ap ·{clients if clients is not None else '?'}  ", "inv")]
    stop = [(" E-STOP ", "rbg")] if g.estop else [("○", "inv"), (" e-stop", "inv")]
    right = esp + sw + cam + ap + stop + [(f"   {hms(app.now())} ", "inv")]
    rlen = sum(len(t) for t, _ in right)
    x = c.segs(0, 0, left)
    c.put(0, x, " " * max(0, c.w - x - rlen), "inv")
    c.segs(0, max(x, c.w - rlen), right)


# --- bed map ----------------------------------------------------------
def map_cells(app, cols: int = MAP_COLS, rows: int = MAP_ROWS, beam: bool = True,
              texture: bool = True, weeds_mode: str | None = None) -> dict:
    """{(row, col): (glyph, role)} for the bed in plan view."""
    g = app.g
    bed = g.bed
    cells: dict = {}
    scanning = bool(g.job and g.job.kind == "scan")
    if texture:
        for r in range(1, rows, 2):
            for col in range(2, cols, 4):
                if g.revealed(*bed.point(r, col, cols, rows)):
                    cells[(r, col)] = ("·", "s")
    tr, tc = bed.cell(g.axes["X"].pos, g.axes["Y"].pos, cols, rows)
    if beam:
        for r in range(rows):
            cells[(r, tc)] = ("┊", "c") if scanning else ("┆", "f")
    for crop in bed.crops:
        if not g.revealed(crop.x, crop.y):
            continue
        r, col = bed.cell(crop.x, crop.y, cols, rows)
        k = bed.kind(crop.kind)
        glyph = k["glyph"]
        start = col - len(glyph) // 2
        for i, ch in enumerate(glyph):
            if 0 <= start + i < cols:
                cells[(r, start + i)] = (ch, k.get("role", "p"))
    mode = weeds_mode or weed_mode(app)
    sel_id = app.g.weeds[app.review_sel].id if mode == "review" and app.g.weeds else None
    for w in g.weeds:
        if not g.revealed(w.x, w.y):
            continue
        r, col = bed.cell(w.x, w.y, cols, rows)
        if w.status == "sprayed":
            cells[(r, col)] = ("o", "g")
        elif w.status == "untreated" or (mode == "map" and w.guarded):
            cells[(r, col)] = ("!", "r")
        elif mode == "map":
            cells[(r, col)] = ("x", "a")
        elif mode == "review":
            cells[(r, col)] = (w.glyph(), "sel" if w.id == sel_id else ("a" if w.ticked else "d"))
        else:  # spray pass
            role = {"spraying": "sel", "queued": "a"}.get(w.status, "d")
            cells[(r, col)] = (w.glyph(), role)
    cells[(tr, tc)] = ("@", "cb")
    return cells


def weed_mode(app) -> str:
    if app.mode == "review":
        return "review"
    j = app.g.job
    if j and j.kind == "weed":
        return "weed"
    return "map"


def bed_map(app, c: Canvas, y: int, x: int, w: int = LEFT_W) -> None:
    g = app.g
    bed = g.bed
    mode = weed_mode(app)
    if mode == "review":
        n = len(g.weeds)
        k = sum(1 for wd in g.weeds if wd.ticked and not wd.guarded and wd.status != "sprayed")
        right = f"{n} weed{'s' if n != 1 else ''} · {k} to spray"
    elif mode == "weed" and g.job:
        order = g.job.targets
        cur = g.job.current
        i = order.index(cur) + 1 if cur in order else 0
        right = f"target {i} of {len(order)}" if i else f"{len(order)} targets"
    else:
        right = f"{bed.width:g} × {bed.height:g} mm · {bed.name}"
    box(c, y, x, 17, w, "BED MAP", right)
    inner = w - 2
    for r in range(15):
        c.put(y + 1 + r, x + 1, " " * inner)
    # horizontal axis labels
    ticks = 200 if bed.h_size >= 800 else 100
    v = 0.0
    while v <= bed.h_size + 0.1:
        lab = f"{v:g}"
        col = 6 + bed.col_of(v)
        if v >= bed.h_size - ticks + 0.1 and v > 0:
            start = col - len(lab) + 1
        else:
            start = col - (len(lab) + 1) // 2 + (1 if len(lab) == 3 else 0)
        c.put(y + 2, x + 1 + start, lab, "f")
        v += ticks
    c.put(y + 1, x + 2, f"{bed.v_axis}↑ {bed.h_axis}→ mm", "f")
    for val in (bed.v_size, bed.v_size / 2, 0):
        r = bed.row_of(val)
        c.put(y + 3 + r, x + 2, f"{val:>4g}", "f")
    for (r, col), (ch, role) in map_cells(app).items():
        c.put(y + 3 + r, x + 7 + col, ch, role)
    c.segs(y + 15, x + 7, legend(app, mode))


def legend(app, mode: str) -> list:
    g = app.g
    if mode == "review":
        ticked = next((w for w in g.weeds if w.ticked and not w.guarded), None)
        skipped = next((w for w in g.weeds if not w.ticked and not w.guarded), None)
        parts = []
        if ticked:
            parts += [(ticked.glyph(), "a"), (" spray  ", "d")]
        if skipped:
            parts += [(skipped.glyph(), "d"), (" skip  ", "d")]
        if any(w.guarded for w in g.weeds):
            parts += [("!", "r"), (" too close  ", "d")]
        if g.weeds:
            parts += [(g.weeds[app.review_sel].glyph(), "sel"), (" selected", "d")]
        return parts
    parts = [("@", "cb"), (" nozzle ", "d")]
    for name, k in g.bed.kinds.items():
        parts += [(k["glyph"], k.get("role", "p")), (f" {k.get('short', name[:4])} ", "d")]
    parts += [("o", "g"), (" sprayed", "d")] if mode == "weed" else [("x", "a"), (" weed ", "d"), ("!", "r"), (" close", "d")]
    return parts


# --- right column panels ----------------------------------------------
def axes_panel(app, c: Canvas, y: int, x: int, w: int = RIGHT_W) -> None:
    g = app.g
    box(c, y, x, 7, w, "AXES")
    inner = w - 2
    rows = []
    for a in "XYZ":
        ax = g.axes[a]
        lo, hi = g.s.travel(a)
        if g.link_lost:
            rows.append([(f" {a}", "h"), (f"{ax.pos:8.1f} ", "t"), ("mm", "d"), ("  last known", "d")])
            continue
        frac = (min(hi, max(lo, ax.pos)) - lo) / (hi - lo) if hi > lo else 0
        mark = [("●", "g"), (" homed", "g")] if ax.homed else [("○", "a"), (" unhomed", "a")]
        rows.append([(f" {a}", "h"), (f"{ax.pos:8.1f} ", "t"), ("mm ", "d")] + bar(frac, 12, "c") + [(" ", "t")] + mark)
    j = g.job
    if g.link_lost:
        rows.append([(" position not trusted until re-home", "a")])
        rows.append([])
    else:
        z = g.axes["Z"].pos
        if j and j.kind == "weed":
            rows.append([(" spray ", "d"), (f"{g.s.spray_ms} ms at Z {g.s.spray_z:g} mm", "t")])
        elif abs(z - g.s.spray_z) < 1:
            rows.append([(" nozzle ", "d"), ("at spray height", "a")])
        else:
            rows.append([(" nozzle ", "d"), (f"Z {z:g} mm · ", "t"), ("step ", "d"), (f"{g.s.jog_step_mm():g} mm", "t")])
        soft = homed_all(g)
        sw_ok = g.switches.available and not g.switch_fault()
        rows.append([(" limits ", "d"), ("soft ", "g" if soft else "a"), ("●" if soft else "○", "g" if soft else "a"),
                     (" · switches ", "g" if sw_ok else "r"), ("●" if sw_ok else "○", "g" if sw_ok else "r")])
    for i, parts in enumerate(rows):
        line(c, y + 1 + i, x + 1, inner, parts)


def detection_panel(app, c: Canvas, y: int, x: int, w: int = RIGHT_W) -> None:
    g = app.g
    box(c, y, x, 10, w, "DETECTION")
    v = g.vision
    j = g.job
    found = list(g.weeds)
    close = sum(1 for wd in found if wd.guarded)
    infer = sum(g.infer_times[-10:]) / len(g.infer_times[-10:]) if g.infer_times else v.infer_s
    if j and j.kind == "scan":
        rows = [
            [(" frame   ", "d"), (f"{j.frame_now:02d} / {len(j.frames):02d}", "h")],
            [(" capture ", "d"), (f"{v.camera or 'no camera'} · {g.s.settle:g} s", "t")],
            [(" infer   ", "d"), ("dry run · no detection" if j.dry else f"{infer or 0:.1f} s · {(v.model or '?').replace('.tflite', '')}", "t")],
            [(" thresh  ", "d"), (f"{j.thresh:.2f}", "t")],
            [(" so far  ", "d"), (f"{len(found)} weed{'s' if len(found) != 1 else ''} · {close} too close", "a")],
            [], [],
            [(f" last frame  {j.last_frame[0]} box{'es' if j.last_frame[0] != 1 else ''}"
              + (f" · max {j.last_frame[1]:.2f}" if j.last_frame[0] else ""), "f")] if j.last_frame else [],
        ]
    else:
        sm = g.scan_summary
        if sm:
            last = f"{time.strftime('%H:%M', time.localtime(sm['wall']))} · {sm['frames']} frames · {sm['secs']:.0f} s"
            fnd = ([(f"{sm['found']} weed{'s' if sm['found'] != 1 else ''} · {sm['close']} too close", "a")]
                   if sm["found"] else [("no weeds", "g")])
        else:
            last, fnd = None, [("—", "d")]
        rows = [
            [(" camera  ", "d"), (f"{v.camera} · ok", "t") if v.camera else ("not wired", "a")],
            [(" model   ", "d"), (v.model, "t") if v.model else ("not wired", "a")],
            [(" thresh  ", "d"), (f"{g.s.thresh:.2f}", "t")],
            [(" clear   ", "d"), (f"no spray within {g.s.guard:g} mm of a crop", "t")],
            [(" last    ", "d"), (last, "t") if last else ("no scan yet", "d")],
            [(" found   ", "d")] + fnd,
            [],
            [(f" infer {infer:.1f} s/frame on Pi", "f")] if infer else [],
        ]
    for i, parts in enumerate(rows):
        line(c, y + 1 + i, x + 1, w - 2, parts)


def link_panel(app, c: Canvas, y: int, x: int, w: int = RIGHT_W) -> None:
    g = app.g
    box(c, y, x, 10, w, "LINK", role="r", title_role="r")
    ago = f" ({int(app.now() - g.last_ok_wall)} s ago)" if g.last_ok_wall else ""
    rows = [
        [(" port    ", "d"), (g.link.device, "t")],
        [(" baud    ", "d"), (f"{g.link.baud} 8N1", "t")],
        [(" last ok ", "d"), (f"{hms(g.last_ok_wall)}{ago}" if g.last_ok_wall else "never", "a")],
        [(" timeout ", "d"), (f"{g.s.missed_for_lost} in a row · {g.s.reply_timeout:g} s each", "r")],
        [(" garbled ", "d"), (f"{g.link.garbled} lines", "t")],
        [(" retry   ", "d"), (f"PING every {g.s.ping_every:g} s · {g.retry_n}/{g.s.ping_tries}", "c")],
        [],
        [(" ESP32 reset? check its 5 V supply", "t")],
    ]
    for i, parts in enumerate(rows):
        line(c, y + 1 + i, x + 1, w - 2, parts)


def targets_weed_panel(app, c: Canvas, y: int, x: int, w: int = RIGHT_W) -> None:
    g = app.g
    j = g.job
    order = j.targets if j else []
    cur = j.current if j else None
    i = order.index(cur) + 1 if cur in order else 0
    box(c, y, x, 10, w, "TARGETS", f"{i} of {len(order)}" if i else None)
    inner = w - 2
    rows = []
    for wd in g.weeds:
        if wd.status == "sprayed":
            parts, right = [("●", "g"), (" sprayed", "t")], f"{wd.spray_s:.1f} s"
        elif wd.status == "skipped":
            parts, right = [("○", "t"), (" skipped", "t")], "unticked"
        elif wd.status == "untreated":
            parts, right = [("!", "r"), (" untreated", "t")], f"{wd.near_mm:.0f} mm"
        elif wd.status == "spraying":
            rows.append(("sel", f" {wd.glyph()} ▸ spraying · Z {g.s.spray_z:g} · {g.s.spray_ms} ms"))
            continue
        elif wd.status == "queued":
            parts, right = [("·", "f"), (" queued", "t")], ""
        else:
            parts, right = [("×", "r"), (f" {wd.status}", "t")], ""
        rows.append((parts, right, wd))
    for k, row in enumerate(rows[:6]):
        yy = y + 1 + k
        if row[0] == "sel":
            c.put(yy, x + 1, row[1].ljust(inner), "sel")
            continue
        parts, right, wd = row
        line(c, yy, x + 1, inner, [(f" {wd.glyph()} ", "h")] + parts)
        c.put(yy, x + 1 + inner - 2 - len(right), right, "t")
    nxt = None
    if j and cur in order and order.index(cur) + 1 < len(order):
        n = order[order.index(cur) + 1]
        nxt = f"MOVE X{n.x:g} Y{n.y:g}"
    done = [wd.spray_s for wd in order if wd.status == "sprayed"]
    per = f"~{sum(done) / len(done):.0f} s" if done else "~8 s"
    line(c, y + 8, x + 1, inner, [(f" per target {per}" + (f" · next {nxt}" if nxt else ""), "f")])


def uart_panel(app, c: Canvas, y: int, x: int, w: int = RIGHT_W, h: int = 10) -> None:
    g = app.g
    box(c, y, x, h, w, f"UART · {g.link.port}")
    rows = [ln for ln in g.log if ln.mark != "#"][-(h - 2):]
    marks = {"→": "c", "←": "g", "!": "a", "×": "r", "·": "f"}
    for i, ln in enumerate(rows):
        text_role = "f" if ln.mark == "·" else "t"
        line(c, y + 1 + i, x + 1, w - 2, [(f" {hms(ln.t)} ", "f"), (ln.mark, marks.get(ln.mark, "t")),
                                         (f" {ln.text}", text_role)])
    for i in range(len(rows), h - 2):
        line(c, y + 1 + i, x + 1, w - 2, [])


# --- job panel --------------------------------------------------------
def job_panel(app, c: Canvas, y: int, x: int, w: int = LEFT_W) -> None:
    g = app.g
    j = g.job
    box(c, y, x, 10, w, "JOB")
    inner = w - 2
    stage = None
    if j and j.kind == "scan":
        stage = "SCAN"
    elif j and j.kind == "weed":
        stage = "SPRAY"
    elif app.mode == "review":
        stage = "REVIEW"

    def plan():
        out = [(" plan   ", "d")]
        for k, name in enumerate(("SCAN", "REVIEW", "SPRAY")):
            if k:
                out.append((" ▸ ", "t"))
            out.append((name, ("cb" if name == "SCAN" else "a") if name == stage else "t"))
        return out

    scan_n = len(j.frames) if j and j.kind == "scan" else (g.scan_summary or {}).get("frames", 0)
    scan_done = j.frames_done if j and j.kind == "scan" else scan_n
    scan_frac = scan_done / scan_n if scan_n else 0
    scan_role = "c" if j and j.kind == "scan" else "g"
    scan_row = [(" scan   ", "d")] + bar(scan_frac, 32, scan_role) + [
        (f"{round(scan_frac * 100):>5d} %  ", "t" if scan_n else "d"),
        (f"{scan_done}/{scan_n} frames" if j and j.kind == "scan" else (f"{scan_n} frames" if scan_n else "no scan"), "t" if scan_n else "d")]
    open_ = [wd for wd in g.weeds if wd.status in ("found", "queued", "spraying", "sprayed", "failed")]
    to_spray = j.targets if j and j.kind == "weed" else [wd for wd in open_ if wd.ticked and not wd.guarded]
    sprayed = sum(1 for wd in to_spray if wd.status == "sprayed")
    frac = sprayed / len(to_spray) if to_spray else 0
    on = bool(j and j.kind == "weed")
    spray_row = [(" spray  ", "d")] + bar(frac, 32, "a" if on else "g") + [
        (f"{round(frac * 100):>5d} %  ", "t" if on else "d"),
        (f"{sprayed}/{len(to_spray)} sprays" if (to_spray or on) else ("waiting" if j else "0 sprays"), "t" if on else "d")]
    untreated = sum(1 for wd in g.weeds if wd.status == "untreated")
    skipped = sum(1 for wd in g.weeds if wd.status == "skipped")
    foot = [(f" spray {g.s.spray_ms} ms at Z {g.s.spray_z:g} mm · clearance {g.s.guard:g} mm (ACT-09)", "f")]
    proc = g.proc if g.proc and not g.proc.done else None

    if g.link_lost:
        rows = [[(" plan   ", "d"), ("nothing running" if not j else f"{j.kind} stopped", "t")],
                [(" state  ", "d"), ("motion blocked: ESP32 not answering", "r")],
                [], [],
                [(" hint   ", "d"), ("boot text on GPIO20/21 means it rebooted;" if g.boot_text_seen
                                     else "check the ESP32's power and the UART wires;", "t")],
                [("        it will answer PONG once ready", "t")],
                [],
                [(" ESP32 stops every axis after 1 s without the Pi", "f")]]
    elif proc:
        detail = {"homing": "fast approach, back off, slow approach, zero, park",
                  "span": "switch to switch at the approach voltage",
                  "step": "open-loop step, every slot pulse timed"}.get(proc.kind, "")
        rows = [[(" proc   ", "d"), (proc.label(), "c")],
                [(" phase  ", "d"), (proc.phase or "-", "t")],
                [(" how    ", "d"), (detail, "t")],
                [], [], [],
                [(" a or :abort stops it · SPACE is the e-stop", "f")], []]
    elif j and j.kind == "scan" and j.state in ("running", "pausing", "finishing"):
        state = ("last frames in inference" if j.state == "finishing" else
                 "pausing after this frame" if j.state == "pausing" else "scanning along Y, snaking in X")
        rows = [plan(), [(" state  ", "d"), (state, "c")], scan_row,
                [(" spray  ", "d")] + bar(0, 32, "g") + [("    0 %  waiting", "d")],
                [(" eta    ", "d"), (f"{mmss(g.eta())} left · started {hms(j.started_wall)}", "t")],
                [], [],
                [(f" camera stops for {g.s.settle:g} s at each frame", "f")]]
    elif j and j.kind == "weed" and j.state in ("running", "pausing"):
        cur = j.current
        state = (f"spraying weed {cur.id} ({cur.near_mm:.0f} mm from a {cur.near_kind})" if cur and cur.status == "spraying"
                 else "moving to the first target")
        if j.state == "pausing":
            state = "stopping after this spray"
        rows = [plan(), [(" state  ", "d"), (state, "a")], scan_row, spray_row,
                [(" eta    ", "d"), (f"{mmss(g.eta())} left · {sprayed} sprayed · {untreated} untreated", "t")],
                [], [],
                [(" p stops after this spray · SPACE stops now", "f")]]
    elif j:
        rows = [plan(), [(" state  ", "d"), (f"{j.kind} stopped: {j.reason}" if j.state == "stopped" else j.reason, "a")],
                scan_row, spray_row,
                [(" next   ", "d"), ("r or :resume carries on · a or :abort ends the job", "c")],
                [], [], foot]
    else:
        n_open = sum(1 for wd in g.weeds if wd.status in ("found", "failed"))
        if not homed_all(g):
            state, nxt = [("not homed · scan and spray are blocked", "a")], [("press h or type :home", "c")]
        elif g.scan_summary and n_open:
            state = [(f"scan done · {n_open} weed{'s' if n_open != 1 else ''} waiting for review", "a")]
            nxt = [("press w or type :review to check targets", "c")]
        elif g.scan_summary:
            state = [("scan done · nothing left to spray", "g")]
            nxt = [("press s to scan again", "c")]
        else:
            state, nxt = [("idle · no scan yet", "t")], [("press s or type :scan to capture the bed", "c")]
        ws = g.weed_summary
        last = ([(time.strftime("%Y-%m-%d %H:%M", time.localtime(ws["wall"]))
                  + f" · {ws.get('sprayed', 0)} sprayed · {ws.get('untreated', 0)} untreated · {ws.get('skipped', 0)} skipped", "t")]
                if ws else [("no spray pass yet", "d")])
        rows = [plan(), [(" state  ", "d")] + state, scan_row, spray_row,
                [(" last   ", "d")] + last, [(" next   ", "d")] + nxt, [], foot]
    for i, parts in enumerate(rows):
        line(c, y + 1 + i, x + 1, inner, parts)


# --- dashboards -------------------------------------------------------
def dashboard(app, c: Canvas, compact: bool = False) -> None:
    g = app.g
    j = g.job
    if compact:
        return compact_dashboard(app, c)
    bed_map(app, c, 1, 0)
    axes_panel(app, c, 1, LEFT_W)
    if g.link_lost:
        link_panel(app, c, 8, LEFT_W)
    elif j and j.kind == "weed":
        targets_weed_panel(app, c, 8, LEFT_W)
    else:
        detection_panel(app, c, 8, LEFT_W)
    job_panel(app, c, 18, 0)
    uart_panel(app, c, 18, LEFT_W)


def compact_dashboard(app, c: Canvas) -> None:
    """80×24: the right column moves under the map; UART log only on l."""
    g = app.g
    bed_map(app, c, 1, 0)
    rw = c.w - LEFT_W
    if rw >= 16:
        box(c, 1, LEFT_W, 17, rw, "AXES")
        for i, a in enumerate("XYZ"):
            ax = g.axes[a]
            line(c, 2 + i, LEFT_W + 1, rw - 2, [(f" {a}", "h"), (f"{ax.pos:7.1f} ", "t"),
                                                ("●" if ax.homed else "○", "g" if ax.homed else "a")])
        line(c, 6, LEFT_W + 1, rw - 2, [(" esp32 ", "d"), ("lost", "r") if g.link_lost else ("ok", "g")])
        sw_ok = g.switches.available and not g.switch_fault()
        line(c, 7, LEFT_W + 1, rw - 2, [(" sw    ", "d"), ("ok", "g") if sw_ok else ("fault", "r")])
        n = sum(1 for wd in g.weeds if wd.status in ("found", "failed"))
        line(c, 8, LEFT_W + 1, rw - 2, [(" weeds ", "d"), (str(n), "a" if n else "t")])
    h = c.h - 2 - 18
    if h >= 3:
        box(c, 18, 0, h, c.w, "JOB")
        j = g.job
        if g.link_lost:
            state = [("motion blocked: ESP32 not answering", "r")]
        elif g.proc and not g.proc.done:
            state = [(g.proc.label(), "c")]
        elif j:
            state = [(f"{j.kind} · {j.state}" + (f" · {j.frames_done}/{len(j.frames)} frames" if j.kind == "scan" else ""), "c")]
        else:
            state = [("idle · l shows the UART log", "t")]
        line(c, 19, 1, c.w - 2, [(" state  ", "d")] + state)


def estop_modal(app, c: Canvas) -> None:
    g = app.g
    y, x, w, h = 6, 18, 64, 17
    box(c, y, x, h, w, role="r")
    inner = w - 2
    for yy in range(y + 1, y + h - 1):
        c.put(yy, x + 1, " " * inner)
    head = "  E-STOP LATCHED"
    t = hms(g.estop_wall) + "   "
    c.put(y + 1, x + 1, head + " " * (inner - len(head) - len(t)) + t, "rbg")
    if g.estop_confirmed:
        conf = [("  ESP32 confirmed: motors braked, pump and valve off.", "t")]
    elif g.estop_confirmed is None:
        conf = [("  waiting for the ESP32 to confirm…", "a")]
    else:
        conf = [("  ESP32 did not confirm · cut the motor supply", "r")]
    z = g.axes["Z"].pos
    pos = f"X{g.axes['X'].pos:g} Y{g.axes['Y'].pos:g}"
    nozzle = [(f"  nozzle at Z {z:g} mm · {pos}", "t")]
    j = g.job
    rows = [
        [], [(f"  {g.estop_reason}"[:inner], "h")], conf, [], nozzle, [],
        [("  to recover", "d")],
        [("  1  ", "d"), ("check nobody is touching the bed", "t")],
        [("  2  ", "d"), ("if a limit switch tripped, free the carriage by hand", "t")],
        [("  3  ", "d"), ("type  ", "t"), (":reset", "c"), ("   then ", "t"), ("h", "c"), (" to re-home if in doubt", "t")],
    ]
    if j:
        where = f"weed {j.current.id}" if j.kind == "weed" and j.current else f"frame {j.frames_done + 1}"
        rows.append([("  4  ", "d"), (":resume", "c"), (f"  picks up at {where}, or  ", "t"), (":abort", "c"), ("  ends the job", "t")])
    else:
        rows.append([])
    rows += [[], [("  motion commands are refused until :reset", "t")]]
    for i, parts in enumerate(rows[: h - 3]):
        line(c, y + 2 + i, x + 1, inner, parts)


# --- review -----------------------------------------------------------
def review_screen(app, c: Canvas) -> None:
    g = app.g
    bed_map(app, c, 1, 0)
    x, w = LEFT_W, RIGHT_W
    box(c, 1, x, 17, w, "TARGETS", "↑↓ space")
    inner = w - 2
    line(c, 2, x + 1, inner, [(" #     x    y  conf  near", "d")])
    weeds = g.weeds
    top = max(0, min(app.review_sel - 5, len(weeds) - 6))
    for k in range(6):
        yy = 3 + k
        i = top + k
        if i >= len(weeds):
            line(c, yy, x + 1, inner, [])
            continue
        wd = weeds[i]
        short = g.bed.kind(wd.near_kind).get("short", wd.near_kind[:4]) if wd.near_kind else "----"
        done = wd.status in ("sprayed", "untreated")
        tick = "o" if wd.status == "sprayed" else ("!" if wd.guarded else ("x" if wd.ticked else " "))
        text = f" {wd.glyph():<2}{wd.x:>5.0f}{wd.y:>5.0f}  {wd.conf:.2f}  {short:<4}{wd.near_mm:>3.0f}mm "
        if i == app.review_sel:
            c.put(yy, x + 1, (text + f"[{tick}]").ljust(inner), "sel")
        else:
            role = "r" if wd.guarded else ("g" if wd.ticked and not done else "t")
            line(c, yy, x + 1, inner, [(text, "d" if done else "t"), (f"[{tick}]", role)])
    sep(c, 9, x, w)
    rows: list = []
    close = next((wd for wd in weeds if wd.guarded and wd.status != "sprayed"), None)
    if close:
        rows += [
            [(f" #{close.id} is {close.near_mm:.0f} mm from a {close.near_kind}:", "a")],
            [(f" inside the {g.s.guard:g} mm clearance, so it", "t")],
            [(" is never sprayed and is logged", "t")],
            [(" as untreated (ACT-09).", "t")],
            [],
        ]
    else:
        rows += [[(f" Every target is more than {g.s.guard:g} mm", "t")], [(" from the nearest crop canopy.", "t")], []]
    if weeds:
        wd = weeds[app.review_sel]
        rows += [
            [(f" #{wd.id}  conf {wd.conf:.2f} · {wd.near_kind} {wd.near_mm:.0f} mm", "h")],
            [(f" path X{wd.x:g} Y{wd.y:g} · Z {g.s.spray_z:g} · {g.s.spray_ms} ms", "t")],
        ]
    for i in range(7):
        line(c, 10 + i, x + 1, inner, rows[i] if i < len(rows) else [])
    if app.preview and weeds:
        frame_panel(app, c, 18)
    else:
        job_panel(app, c, 18, 0)
        uart_panel(app, c, 18, LEFT_W)


SPRITES = {
    "lettuce": [" ,%%%, ", "%%%%%%%", " '%%%' ", "  |||  "],
    "bokchoy": [" \\|||/ ", "  |||  ", "  |||  ", "  |||  "],
    "radish":  ["  \\|/  ", "   *   ", "  /|\\  ", "   |   "],
    "basil":   [" .o.o. ", " o.o.o ", "  \\|/  ", "   |   "],
}


def frame_panel(app, c: Canvas, y: int) -> None:
    """Zoomed view around the selected weed, drawn from the bed map (a schematic, not the camera crop)."""
    g = app.g
    bed = g.bed
    wd = g.weeds[app.review_sel]
    w, h = c.w if c.w < FULL_W else FULL_W, 10
    box(c, y, 0, h, w, f"FRAME {wd.frame} · around #{wd.id}", f"schematic · {bed.h_axis}→ {bed.v_axis}↑")
    inner_w, inner_h = w - 2, h - 2
    mm_col, mm_row = 3.0, 25.0
    cx, cy = inner_w // 2, 3
    for r in range(inner_h):
        c.put(y + 1 + r, 1, " " * inner_w)
        for col in range(inner_w):
            if (r in (0, inner_h - 2) and col % 3 == 1) or (r not in (0, inner_h - 2, inner_h - 1) and col % 13 == (r * 5) % 13):
                c.put(y + 1 + r, 1 + col, ".", "s")
    wh, wv = bed.hv(wd.x, wd.y)

    def cell(xm, ym):
        hm, vm = bed.hv(xm, ym)
        return cx + round((hm - wh) / mm_col), cy - round((vm - wv) / mm_row)

    for crop in bed.crops:
        col, row = cell(crop.x, crop.y)
        if not -8 < col < inner_w + 8 or not -3 < row < inner_h:
            continue
        row = max(1, min(inner_h - 5, row))
        sprite = SPRITES.get(crop.kind, [f"  {bed.kind(crop.kind)['glyph']:^3}  "])
        role = bed.kind(crop.kind).get("role", "p")
        for k, s in enumerate(sprite + [crop.kind[:7].center(7)]):
            rr = row - 1 + k
            if 0 <= rr < inner_h - 1:
                for i, ch in enumerate(s):
                    cc = col - 3 + i
                    if ch != " " and 0 <= cc < inner_w:
                        c.put(y + 1 + rr, 1 + cc, ch, "d" if k == len(sprite) else role)
    label = f" {wd.glyph()} · {wd.conf:.2f} "
    bw = max(12, len(label) + 2)
    rows = ["┌" + label.center(bw - 2, " ") + "┐", "│" + "\\|/".center(bw - 2) + "│",
            "│" + "--*--".center(bw - 2) + "│", "│" + "/|\\".center(bw - 2) + "│", "└" + "─" * (bw - 2) + "┘"]
    for k, s in enumerate(rows):
        c.put(y + 2 + k, 1 + cx - bw // 2, s, "r" if wd.guarded else "a")


# --- jog --------------------------------------------------------------
def jog_pad(h: str, v: str) -> list:
    return [
        "                 ┌─────────┐",
        "                 │    ↑    │",
        f"                 │   {v}+    │",
        "       ┌─────────┼─────────┼─────────┐        ┌─────────┐",
        "       │  ←      │         │      →  │        │ Z- up   │",
        f"       │  {h}-     │    @    │     {h}+  │        │    w    │",
        "       └─────────┼─────────┼─────────┘        ├─────────┤",
        f"                 │   {v}-    │                  │    s    │",
        "                 │    ↓    │                  │ Z+ down │",
        "                 └─────────┘                  └─────────┘",
    ]


def jog_screen(app, c: Canvas) -> None:
    g = app.g
    motors = "motors off" if g.estop or g.link_lost else "motors on"
    box(c, 1, 0, 27, LEFT_W, "JOG · manual move", motors)
    inner = LEFT_W - 2
    for r in range(25):
        c.put(2 + r, 1, " " * inner)
    for i, row in enumerate(jog_pad(g.bed.h_axis, g.bed.v_axis)):
        yy = 3 + i
        for k, ch in enumerate(row):
            if ch == " ":
                continue
            role = "c" if ch in "←→↑↓" else ("cb" if ch == "@" else "t")
            c.put(yy, 1 + k, ch, role)
    parts = [(" step   ", "d")]
    for i, s in enumerate(g.s.jog_steps):
        parts += [(f" {s:g} ", "sel" if i == g.s.jog_step else "t"), (" ", "t")]
    own = g.s.jog_custom
    parts += [(f" {own:g} " if own > 0 else " own ", "sel" if g.s.jog_step >= len(g.s.jog_steps) else "t"), (" ", "t")]
    parts += [(" mm   ", "d"), ("keys 1–6", "d")]
    line(c, 14, 1, inner, parts)
    line(c, 15, 1, inner, [(" fine   ", "d"), ("hold shift for 1 mm", "t"), ("   unhomed: 50 mm max per jog", "d")])
    sep(c, 17, 0, LEFT_W, "WHERE")
    mw = MAP_COLS
    mx = 1 + (inner - mw - 2) // 2
    c.put(19, mx, "┌" + "─" * mw + "┐", "f")
    cells = map_cells(app, rows=5, texture=False, weeds_mode="map")
    for r in range(5):
        c.put(20 + r, mx, "│", "f")
        c.put(20 + r, mx + mw + 1, "│", "f")
    for (r, col), (ch, role) in cells.items():
        c.put(20 + r, mx + 1 + col, ch, role)
    c.put(25, mx, "└" + "─" * mw + "┘", "f")

    x, w = LEFT_W, RIGHT_W
    box(c, 1, x, 9, w, "POSITION")
    rows = []
    for a in "XYZ":
        ax = g.axes[a]
        lo, hi = g.s.limits(a)
        tol = max(0.5, g.s.mmpc(a))          # the ESP32 stops within one count of a target
        inside = lo - g.s.soft_margin - tol <= ax.pos <= hi + tol      # homed on the switch (0) counts as in limits
        rows.append([(f" {a}", "h"), (f"{ax.pos:7.1f}", "h"), (" mm   ", "d"), (f"{lo:g} – {hi:g}", "t")])
        if not ax.homed:
            rows.append([("    ", "t"), ("○", "a"), (" unhomed · limits not trusted", "a")])
        else:
            rows.append([("    ", "t"), ("●", "g" if inside else "r"), (" homed · in limits" if inside else " homed · OUTSIDE limits", "g" if inside else "r")])
    rows.append([])
    for i, parts in enumerate(rows):
        line(c, 2 + i, x + 1, w - 2, parts)
    box(c, 10, x, 8, w, "NOZZLE")
    trows = [
        [(" f f ", "h"), (f"test spray {g.s.spray_ms} ms here", "t")],
        [(" g   ", "h"), ("go to x,y …", "t")],
        [],
        [(f" no spray within {g.s.guard:g} mm of a crop", "a")],
    ]
    for i, parts in enumerate(trows):
        line(c, 11 + i, x + 1, w - 2, parts)
    uart_panel(app, c, 18, x)


# --- calibration --------------------------------------------------------
def calib_value_text(app, i: int) -> str:
    kind = CALIB_FIELDS[i][2]
    v = app.calib_value(i)
    g = app.g
    if kind == "axis":
        return v.upper()
    if kind == "span_axis":
        return v
    if kind == "volts":
        return f"{v:g} V   duty {100 * v / g.s.rail_v:.0f} % of {g.s.rail_v:g} V"
    if kind == "dir":
        return "+ (away from home)" if v > 0 else "- (towards home)"
    if kind == "pulses":
        a = g.s.step_axis.upper()
        return f"{v}   ≈{v * g.s.mmpc(a):.0f} mm of travel"
    if kind == "repeats":
        return str(v)
    if kind == "bool":
        return "yes" if v else "no"
    if kind == "secs":
        return f"{v:g} s"
    if kind == "span_mm":
        return f"{v:g} mm" if v else "— (counts only)"
    return str(v)


def calib_screen(app, c: Canvas) -> None:
    g = app.g
    proc = g.proc if g.proc and not g.proc.done else None
    box(c, 1, 0, 17, LEFT_W, "STEP TEST · system ID", "s runs")
    inner = LEFT_W - 2
    for r in range(15):
        c.put(2 + r, 1, " " * inner)
    for i in range(STEP_FIELDS):
        label = CALIB_FIELDS[i][1]
        text = f"  {label:<14}{calib_value_text(app, i)}"
        if i == app.calib_sel:
            c.put(3 + i, 1, ("▸" + text[1:]).ljust(inner), "sel")
        else:
            line(c, 3 + i, 1, inner, [(f"  {label:<14}", "d"), (calib_value_text(app, i), "t")])
    a = g.s.step_axis.upper()
    homed = g.axes[a].homed
    line(c, 11, 1, inner, [(" axis    ", "d"), (f"{a} at {g.axes[a].pos:.1f} mm · ", "t"),
                           ("homed: soft limits stop it" if homed else "NOT homed: only the switches stop it", "g" if homed else "a")])
    nm = f"{a}_{float(g.s.step_volts):.1f}_{'pos' if g.s.step_dir > 0 else 'neg'}_test.csv"
    line(c, 12, 1, inner, [(" saved   ", "d"), (short_path(g.s.sysid_dir) + "/" + nm, "t")])
    line(c, 13, 1, inner, [(" format  ", "d"), ("edge,t_us · every raw sensor edge, us from the step", "t")])
    line(c, 15, 1, inner, [(" ↑↓ field  ←→ change  ↵ type a value  s start", "f")])

    x, w = LEFT_W, RIGHT_W
    box(c, 1, x, 17, w, "SPAN · mm per count", "m runs")
    rinner = w - 2
    for r in range(15):
        c.put(2 + r, x + 1, " " * rinner)
    for k, i in enumerate(range(STEP_FIELDS, len(CALIB_FIELDS))):
        label = CALIB_FIELDS[i][1]
        val = calib_value_text(app, i)
        if i == app.calib_sel:
            c.put(3 + k, x + 1, f"▸ {label:<14}{val}"[:rinner].ljust(rinner), "sel")
        else:
            line(c, 3 + k, x + 1, rinner, [(f"  {label:<14}", "d"), (val, "t")])
    sa = app.span_axis
    line(c, 7, x + 1, rinner, [(f"  in use {g.s.mmpc(sa):.4f} mm/count", "d")])
    r = g.span_result
    if r:
        rows = [[(f"  last {r['axis']}: ", "d"), (" ".join(str(s) for s in r["spans"])[:rinner - 10], "t")],
                [(f"  mean {r['mean']:.1f} counts · spread {r['spread']}", "g" if r["spread"] <= 1 else "a")],
                [(f"  = {r['nominal_mm']:.1f} mm at the mm/count in use", "t")],
                [(f"  → {r['mmpc']:.4f} mm/count · a applies", "c")] if r["mmpc"] else
                [(f"  set span_{r['axis'].lower()}_mm for mm/count", "a")]]
    else:
        rows = [[("  no span run yet", "d")], [("  needs the axis homed", "d")], [], []]
    for k, parts in enumerate(rows):
        line(c, 9 + k, x + 1, rinner, parts)
    line(c, 13, x + 1, rinner, [("  measured span: steel rule,", "f")])
    line(c, 14, x + 1, rinner, [("  switch trip to trip (set span_x_mm)", "f")])
    line(c, 15, x + 1, rinner, [("  pass: spread ≤ 1 count (ATP-05)", "f")])

    # results
    box(c, 18, 0, 10, LEFT_W, "STEP RESULTS", proc.label() if proc else None, right_role="c")
    line(c, 19, 1, inner, [("  axis volts dir  pulses   end       rpm  mm/s  file", "d")])
    res = g.step_results[-7:]
    for k in range(7):
        if k < len(res):
            s = res[k]
            rpm = f"{s['rpm']:5.1f}" if s["rpm"] else "    -"
            mms = f"{s['mm_s']:5.1f}" if s["mm_s"] else "    -"
            line(c, 20 + k, 1, inner, [(f"  {s['axis']:<4}{s['volts']:5g}  {s['dir']:+d}  {s['n']:>3}/{s['req']:<4} {s['end']:<8} {rpm} {mms}  ", "t"),
                                       ((s["file"] or "not saved")[:22], "f")])
        else:
            line(c, 20 + k, 1, inner, [])
    uart_panel(app, c, 18, x)


# --- config -----------------------------------------------------------
def config_screen(app, c: Canvas) -> None:
    g = app.g
    lw = 24
    box(c, 1, 0, 27, lw, "CONFIG")
    for r in range(25):
        c.put(2 + r, 1, " " * (lw - 2))
    for i, name in enumerate(CONFIG_SECTIONS):
        yy = 3 + 2 * i
        if i == app.config_sec:
            c.put(yy, 1, f"  ▸ {name}".ljust(lw - 2), "sel")
        else:
            c.put(yy, 1, f"    {name}", "t")
    c.put(26, 1, "  ↑↓ section  ↵ edit", "d")
    rw = c.w - lw if c.w < FULL_W else FULL_W - lw
    sec = CONFIG_SECTIONS[app.config_sec]
    box(c, 1, lw, 27, rw, sec.upper())
    for r in range(25):
        c.put(2 + r, lw + 1, " " * (rw - 2))
    if sec == "network":
        return network_section(app, c, lw, rw)
    s = g.s
    bed = g.bed
    st = g.sw_state or {}

    def sw(name):
        v = st.get(name)
        return "?" if v is None else ("PRESSED" if v else "released")

    groups = {
        "esp32 link": [("port", s.port), ("device", g.link.device), ("baud", f"{s.baud} 8N1"),
                       ("reply_timeout", f"{s.reply_timeout:g} s"), ("missed_for_lost", s.missed_for_lost),
                       ("keepalive", f"{s.keepalive:g} s (ESP stops after 1 s)"), ("heartbeat", f"{s.heartbeat:g} s"),
                       ("latency", f"{g.latency_ms} ms" if g.latency_ms is not None else "—"),
                       ("settings sent", "yes" if g.config_pushed else "no · press u")],
        "axes": [("travel_x", f"{s.travel_x[0]:g} – {s.travel_x[1]:g} mm"), ("travel_y", f"{s.travel_y[0]:g} – {s.travel_y[1]:g} mm"),
                 ("travel_z", f"{s.travel_z[0]:g} – {s.travel_z[1]:g} mm (down)"), ("soft_margin", f"{s.soft_margin:g} mm"),
                 ("mm_per_count", f"X {s.mm_per_count_x:g} · Y {s.mm_per_count_y:g} · Z {s.mm_per_count_z:g}"),
                 ("gain_x (kp kd vlim v0)", " ".join(f"{v:g}" for v in s.gain_x)),
                 ("gain_y", " ".join(f"{v:g}" for v in s.gain_y)), ("gain_z", " ".join(f"{v:g}" for v in s.gain_z)),
                 ("rail_v", f"{s.rail_v:g} V (no stall detection: moves time out instead)"),
                 ("home", f"order {s.home_order} · {s.home_fast_v:g} V to the switch · homed when pressed"),
                 ("jog_steps", " ".join(f"{v:g}" for v in s.jog_steps) + " mm")],
        "switches": [("backend", g.switches.backend + (f" · {g.switches.error}" if g.switches.error else ""))]
                    + [(n, f"BCM {s.switch_pins.get(n, '?')} · {sw(n)}") for n in SWITCH_NAMES]
                    + [("wiring", "NC to GND, pull-up: broken wire = pressed" if s.switch_nc else "NO to GND, pull-up: broken wire looks released"), ("fault", g.switch_fault() or "none")],
        "spray": [("safe_z", f"{s.safe_z:g} mm"), ("spray_z", f"{s.spray_z:g} mm (TBC, ATP-08)"),
                  ("spray_ms", f"{s.spray_ms} ms (TBC, ATP-08)"), ("guard", f"{s.guard:g} mm (2 × {s.tolerance:g} + {s.weed_radius:g})"),
                  ("park", f"X{s.park_x:g} Y{s.park_y:g}"), ("spray log", os.path.join(short_path(s.log_dir), "sprays.csv"))],
        "detection": [("camera", g.vision.camera or "not wired"), ("model", g.vision.model or "not wired"),
                      ("thresh", f"{s.thresh:.2f}"), ("scan_cols", f"{s.scan_cols} across X"),
                      ("scan_rows", f"{s.scan_rows} along Y"), ("camera_z", f"{s.camera_z:g} mm"), ("settle", f"{s.settle:g} s")],
        "bed layout": [("bed_file", os.path.basename(s.bed_file)), ("name", bed.name),
                       ("size", f"{bed.width:g} × {bed.height:g} mm"),
                       ("crops", " · ".join(f"{sum(1 for cr in bed.crops if cr.kind == k)} {k}" for k in bed.kinds)),
                       ("map_view", s.map_view)],
        "calibration": [("sysid_dir", short_path(s.sysid_dir)), ("calib_dir", short_path(s.calib_dir)),
                        ("span_x_mm", f"{s.span_x_mm:g}" if s.span_x_mm else "not measured"),
                        ("span_y_mm", f"{s.span_y_mm:g}" if s.span_y_mm else "not measured"),
                        ("screen", "press k on the dashboard or :calib")],
        "logs": [("log_dir", s.log_dir), ("today", time.strftime("gantry-%Y%m%d.log")),
                 ("sprays", "sprays.csv (ACT-06)"), ("in memory", f"{len(g.log)} lines"), ("full view", "press l or type :log")],
        "about": [("gantry cli", app.version), ("firmware", "firmware/gantry_fw (ESP32-C3)"), ("host", g.system.hostname()),
                  ("python", platform.python_version()), ("terminal", f"{c.w}×{c.h}"),
                  ("colours", os.environ.get("TERM", "?"))],
    }
    items = groups.get(sec, [])
    for i, (k, v) in enumerate(items):
        line(c, 3 + i, lw + 1, rw - 2, [(f"  {k:<18}", "d"), (str(v), "t")])
    tail = [("  change one with  ", "d"), (":set <key> <value>", "c"), ("  · or edit gantry.json", "d")]
    if sec == "esp32 link":
        tail = [("  u ", "h"), ("send the axis settings to the ESP32 now", "t")]
    line(c, 3 + len(items) + 1, lw + 1, rw - 2, tail)


def network_section(app, c: Canvas, lw: int, rw: int) -> None:
    g = app.g
    s = g.s
    sysm = g.system
    info = app.net_info()
    up = app.service(s.ap_service)
    inner = rw - 2
    x = lw + 1
    radio = []
    if info.get("hw_mode"):
        radio.append("5 GHz" if info["hw_mode"] == "a" else "2.4 GHz")
    if info.get("channel"):
        radio.append(f"channel {info['channel']}")
    if info.get("wpa"):
        radio.append(f"WPA{info['wpa']}-{info.get('wpa_key_mgmt', '?').replace('WPA-', '')}/{info.get('rsn_pairwise', '?')}")
    if info.get("power_save"):
        radio.append(f"power-save {info['power_save']}")

    def svc(name):
        st = app.service(name)
        return [(f"{name} ", "t"), ("●" if st else "○", "g" if st else "a"),
                (" active   " if st else (" inactive   " if st is not None else " ?   "), "t")]

    rows = [
        [],
        [("  mode       ", "d")] + ([("●", "g"), (f" access point  {info.get('ssid', s.ap_ssid)}", "g")] if up
                                    else [("○", "a"), (" access point off" if up is not None else " unknown", "a")]),
        [("  address    ", "d"), (info.get("address", "?"), "t")],
        [("  radio      ", "d"), (" · ".join(radio) or "?", "t")],
        [("  services   ", "d")] + svc(s.ap_service) + svc("hostapd") + svc("dnsmasq"),
        [("  internet   ", "d"), ("none while the AP is up (one radio)", "t")],
        [("  fallback   ", "d"), (f"hotspot {s.hotspot} via NetworkManager", "t")],
        [],
    ]
    for i, parts in enumerate(rows):
        line(c, 2 + i, x, inner, parts)
    sep(c, 10, lw, rw, "CLIENTS")
    line(c, 11, x, inner, [("  ip             host         signal   connected", "d")])
    clients = app.clients() or []
    me = sysm.this_ssh_ip()
    for i, cl in enumerate(clients[:1]):
        parts = [(f"  {cl.ip:<15}{cl.host:<13}{cl.signal:<9}{cl.connected:<11}", "t")]
        if cl.ip == me:
            parts += [("←", "c"), (" this ssh session", "c")]
        line(c, 12 + i, x, inner, parts)
    if not clients:
        line(c, 12, x, inner, [("  no clients" if clients is not None else "  leases file not readable", "d")])
    line(c, 13, x, inner, [])
    sep(c, 14, lw, rw, "ACTIONS")
    actions = [
        [("   o  ", "h"), ("go online         ", "t"), ("stop the AP and join the hotspot for apt", "t")],
        [("                        ", "t"), ("your ssh session drops · reconnect on the hotspot", "a")],
        [],
        [("   r  ", "h"), ("restart AP        ", "t"), ("restart hostapd + dnsmasq (≈10 s offline)", "t")],
        [],
        [("   p  ", "h"), ("change password   ", "t"), ("edit wpa_passphrase, restart hostapd", "t")],
    ]
    actions += [[("                        ", "t"), (f"! open item: {item}", "a")] for item in s.open_items]
    actions += [[], [("   k  ", "h"), ("add ssh key       ", "t"), ("ssh-copy-id from your laptop to skip passwords", "t")]]
    for i, parts in enumerate(actions[:12]):
        line(c, 15 + i, x, inner, parts)


# --- boot -------------------------------------------------------------
def boot_screen(app, c: Canvas) -> None:
    g = app.g
    for i, row in enumerate(ART):
        for col, text, role in row:
            c.put(TOP_ROW + i, col, text, role)
    x = RIGHT_COL
    c.put(TOP_ROW, x, NAME, "h")
    c.put(TOP_ROW + 1, x, f"spot-spray gantry · {app.version}", "d")
    c.put(TOP_ROW + 2, x, f"pi zero w · {g.system.hostname()}", "d")
    c.put(TOP_ROW + 4, x, "STARTUP CHECKS", "h")
    marks = {"ok": ("●", "g"), "warn": ("○", "a"), "fail": ("×", "r"), "busy": ("○", "c"), "wait": ("○", "f")}
    names = list(app.boot_checks)
    for i, name in enumerate(names):
        state, value = app.boot_checks[name]
        m, role = marks[state]
        c.segs(TOP_ROW + 5 + i, x, [(m, role), (" ", "t"), (f"{name:<8} ", "d"),
                                    (value if state != "wait" else "", "t" if state == "ok" else role)])
    homed = homed_all(g)
    yy = TOP_ROW + 5 + len(names)
    c.segs(yy, x, [("●" if homed else "○", "g" if homed else "a"), (" ", "t"), ("homed    ", "d"),
                   ("yes" if homed else ("homing…" if app.boot_phase == "homing" else "not yet"), "g" if homed else "a")])
    if app.boot_phase == "prompt":
        c.put(yy + 2, x, "Home all axes now?" if g.s.switches_connected else "Set datum here?", "h")
        c.segs(yy + 3, x, [(" y ", "key"), (" home  ", "d"), (" n ", "key"), (" skip", "d")])
    elif app.boot_phase == "homing" and g.proc:
        c.put(yy + 2, x, g.proc.label()[:20], "c")
        c.put(yy + 3, x, (g.proc.phase or "")[:20], "d")
    if app.boot_phase in ("prompt", "homing"):
        c.put(yy + 5, x, "Keep hands clear", "a")
        c.put(yy + 6, x, "of the bed.", "a")
    up = g.system.uptime_s()
    svc = []
    for name in (g.s.ap_service, "hostapd", "dnsmasq"):
        st = app.service(name)
        svc.append(f"{name} {'active' if st else ('inactive' if st is not None else '?')}")
    boot = f"boot {int(up // 60)} m {int(up % 60)} s · " if up is not None else ""
    c.put(c.h - 4, 1, boot + " · ".join(svc), "f")


# --- log --------------------------------------------------------------
def log_screen(app, c: Canvas) -> None:
    g = app.g
    h = c.h - 3
    box(c, 1, 0, h, c.w if c.w < FULL_W else FULL_W, "LOG", f"{len(g.log)} lines · newest last")
    w = (c.w if c.w < FULL_W else FULL_W) - 2
    rows = h - 2
    end = max(0, len(g.log) - app.log_scroll)
    show = g.log[max(0, end - rows):end]
    marks = {"→": "c", "←": "g", "!": "a", "×": "r", "·": "f", "#": "h"}
    for i in range(rows):
        if i < len(show):
            ln = show[i]
            line(c, 2 + i, 1, w, [(f" {hms(ln.t)} ", "f"), (ln.mark, marks.get(ln.mark, "t")),
                                  (f" {ln.text}", "f" if ln.mark == "·" else ("d" if ln.mark == "#" else "t"))])
        else:
            line(c, 2 + i, 1, w, [])


# --- overlays ---------------------------------------------------------
def commands_popup(app, c: Canvas) -> None:
    y, x, w, h = c.h - 12, 0, LEFT_W, 10
    box(c, y, x, h, w, "COMMANDS", "TAB complete · ↑↓ pick", role="c", title_role="c")
    inner = w - 2
    rows = commands.matches(app.cmd or "")[:5]
    pick = commands.best(app.cmd or "", rows, app.pick)
    for i in range(5):
        yy = y + 1 + i
        if i >= len(rows):
            line(c, yy, x + 1, inner, [])
            continue
        text, desc = rows[i]
        if i == pick:
            c.put(yy, x + 1, f"  {text:<22}{desc}".ljust(inner)[:inner], "sel")
        else:
            word, _, rest = text.partition(" ")
            line(c, yy, x + 1, inner, [("  ", "t"), (word, "h"), (" " + rest if rest else "", "c"),
                                       (" " * max(1, 22 - len(text)), "t"), (desc, "d")])
    for i, also in enumerate(commands.ALSO):
        parts = [("  ", "t"), ("also", "f"), ("  " + also, "d")] if i == 0 else [("  ", "t"), (also, "d")]
        line(c, y + 6 + i, x + 1, inner, parts)


def help_overlay(app, c: Canvas) -> None:
    w, h = 64, 20
    y, x = 4, (min(c.w, FULL_W) - w) // 2
    box(c, y, x, h, w, "HELP", "esc closes", role="c", title_role="c")
    rows = [[(" this screen", "h")]]
    for k, label in keys_for(app):
        rows.append([(f"   {k:<8}", "c"), (label, "t")])
    rows += [[], [(" everywhere", "h")],
             [("   SPACE   ", "r"), ("E-STOP, read before any other key", "t")],
             [("   :       ", "c"), ("command line · help <cmd> for details", "t")],
             [("   h s w j ", "c"), ("home · scan · review/spray · jog", "t")],
             [("   k c l p ", "c"), ("calibrate · config · full log · pause", "t")],
             [("   esc ? q ", "c"), ("back/close · help · quit (asks if a job runs)", "t")]]
    for i in range(h - 2):
        line(c, y + 1 + i, x + 1, w - 2, rows[i] if i < len(rows) else [])


# --- bottom rows ------------------------------------------------------
def keys_for(app) -> list[tuple[str, str]]:
    g = app.g
    j = g.job
    if app.prompt:
        return [("y", "yes"), ("n", "no")]
    if app.ask:
        return [("↵", "set"), ("esc", "cancel")]
    if app.cmd is not None:
        return [("TAB", "complete"), ("↵", "run"), ("esc", "close"), ("↑↓", "history")]
    if app.mode == "boot":
        return ([("y", "home now"), ("n", "skip")] if app.boot_phase == "prompt" else []) + [("?", "help"), ("q", "quit")]
    if app.mode == "log":
        return [("↑↓", "scroll"), ("PgUp/Dn", "page"), ("esc", "back")]
    if app.mode == "config":
        sec = CONFIG_SECTIONS[app.config_sec]
        if sec == "network":
            return [("o", "online"), ("r", "restart"), ("p", "password"), ("k", "ssh key"), ("esc", "back")]
        if sec == "esp32 link":
            return [("↑↓", "section"), ("u", "send settings"), ("↵", "edit"), ("esc", "back")]
        return [("↑↓", "section"), ("↵", "edit"), ("esc", "back")]
    if g.estop:
        return [(":", "cmd"), ("l", "log"), ("?", "help")]
    if app.mode == "calib":
        if g.proc and not g.proc.done:
            return [("a", "abort"), ("esc", "back")]
        return [("↑↓", "field"), ("←→", "change"), ("↵", "type"), ("s", "step test"), ("m", "span"), ("a", "apply"), ("v", "check"), ("esc", "back")]
    if app.mode == "jog":
        return [("←→↑↓", "move"), ("w s", "Z"), ("1-5", "step"), ("6", "own step"), ("f f", "spray"), ("esc", "done"), ("c", "photo")]
    if app.mode == "review":
        return [("↑↓", "select"), ("space", "tick"), ("v", "view"), ("↵", "confirm"), ("esc", "back")]
    if g.link_lost:
        return [("r", "retry now"), ("l", "log"), ("c", "config"), (":", "cmd"), ("?", "help")]
    if g.proc and not g.proc.done:
        return [("a", "abort"), ("k", "calibration"), ("l", "log"), (":", "cmd")]
    if j and j.state in ("running", "pausing", "finishing"):
        if j.kind == "scan":
            return [("p", "pause"), ("a", "abort"), ("l", "log"), (":", "cmd"), ("?", "help")]
        return [("p", "stop after spray"), ("l", "log"), (":", "cmd"), ("?", "help")]
    if j:
        return [("r", "resume"), ("a", "abort"), ("h", "home"), ("l", "log"), (":", "cmd"), ("?", "help")]
    return [("h", "home"), ("s", "scan"), ("w", "spray"), ("j", "jog"), ("k", "calib"), ("c", "config"), (":", "cmd"), ("q", "quit")]


def key_bar(app, c: Canvas) -> None:
    """Keys valid right now; SPACE E-STOP always fits, other keys drop off the end first."""
    y = c.h - 2
    c.put(y, 0, " " * c.w)
    right = [(" SPACE ", "rbg"), (" ", "t"), (" LATCHED" if app.g.estop else " E-STOP", "r")]
    rx = min(c.w, FULL_W) - sum(len(t) for t, _ in right) - 1
    parts, used = [(" ", "t")], 1
    for k, label in keys_for(app):
        n = len(k) + len(label) + 5
        if used + n > rx - 1:
            break
        parts += [(f" {k} ", "key"), (f" {label} ", "d"), (" ", "t")]
        used += n
    c.segs(y, 0, parts)
    c.segs(y, rx, right)


def command_line(app, c: Canvas) -> None:
    y = c.h - 1
    c.put(y, 0, " " * (c.w - 1))
    g = app.g
    if app.prompt:
        c.segs(y, 0, [(" ", "t"), ("?", "a"), (" ", "t"), (app.prompt[0], "h"), ("   ", "t"), ("[y/N]", "a"), (" ", "t"), ("█", "c")])
    elif app.ask:
        label, text, _ = app.ask
        c.segs(y, 0, [(" ", "t"), ("?", "a"), (f" {label}: ", "t"), (text, "h"), ("█", "c")])
    elif app.cmd is not None:
        c.segs(y, 0, [(" ", "t"), (":", "c"), (" ", "t"), (app.cmd, "h"), ("█", "c")])
    elif g.notice:
        mark, text = g.notice
        if mark == "·":
            c.segs(y, 0, [(" ", "t"), (":", "f"), (" ", "t"), (text, "t")])
        else:
            role = "r" if mark == "×" else "a"
            c.segs(y, 0, [(" ", "t"), (mark, role), (" ", "t"), (text, role)])
    else:
        c.segs(y, 0, [(" ", "t"), (":", "f"), (" ", "t"), (hint(app), "f")])


def hint(app) -> str:
    g = app.g
    j = g.job
    if app.mode == "boot":
        return "SPACE is E-STOP on every screen, even while typing a command"
    if g.estop:
        return "type :reset once the bed is clear"
    if app.mode == "config":
        return "or type  net online · net ap · set spray_ms 300"
    if app.mode == "jog":
        return f"arrows move {g.s.jog_step_mm():g} mm along the axes drawn on the map · shift for 1 mm"
    if app.mode == "calib":
        return "step tests run open loop · keep the carriage clear of both ends"
    if j and j.kind == "scan" and j.state == "running":
        return "scanning · p pauses after this frame · a aborts and parks the camera"
    if j and j.kind == "weed" and j.state == "running":
        return "spraying · keep hands off the bed"
    return "press : for commands · TAB completes · ↑↓ history"
