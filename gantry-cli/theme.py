"""
Colour roles from the design's spec board (screen 11), mapped onto curses.

Role names are the design's CSS class names so a screen can be checked
against the mockup cell by cell:

  t text   f frame   d label   h heading   g ok   a attention   r fault
  c accent cb accent bold   p crop   u beet   s soil   z dimmed (under a modal)
  key key cap   sel selection   rbg e-stop banner
  inv* status bar (reverse video) with coloured variants
  a? the boot art palette (wood, metal, belt, yellow, orange, grey, ...)

With 256 colours the hexes are matched to the nearest xterm index and the
background is painted #111413 so the look does not depend on the terminal
theme. With 8 colours the spec board's fallback column is used. With no
colour at all only bold/dim/reverse survive.
"""

import curses

BG = "#111413"

# role: (fg hex, bg hex or None, bold, (fallback fg, fallback bg, fallback attr))
ROLES = {
    "t":    ("#d6d9d4", None, False, ("white", None, 0)),
    "f":    ("#56605a", None, False, ("white", None, "dim")),
    "d":    ("#8a948e", None, False, ("white", None, "dim")),
    "h":    ("#f4f1e8", None, True,  ("white", None, "bold")),
    "g":    ("#7fd18b", None, False, ("green", None, 0)),
    "a":    ("#f2b84b", None, False, ("yellow", None, 0)),
    "r":    ("#ff6b5e", None, True,  ("red", None, "bold")),
    "c":    ("#6cc7d9", None, False, ("cyan", None, 0)),
    "cb":   ("#6cc7d9", None, True,  ("cyan", None, "bold")),
    "p":    ("#8fcf7a", None, False, ("green", None, 0)),
    "u":    ("#d77be0", None, False, ("magenta", None, 0)),
    "s":    ("#6b5444", None, False, ("yellow", None, "dim")),
    "z":    ("#3a403d", None, False, ("white", None, "dim")),
    "key":  ("#f4f1e8", "#2c3330", False, ("black", "white", 0)),
    "sel":  ("#0f1211", "#6cc7d9", False, ("black", "cyan", 0)),
    "rbg":  ("#111413", "#ff6b5e", True,  ("black", "red", "bold")),
    "inv":  ("#111413", "#d6d9d4", False, ("white", None, "reverse")),
    "invb": ("#111413", "#d6d9d4", True,  ("white", None, "reverse|bold")),
    "invg": ("#1f6e31", "#d6d9d4", True,  ("green", None, "reverse|bold")),
    "inva": ("#7a4f00", "#d6d9d4", True,  ("yellow", None, "reverse|bold")),
    "invr": ("#a3221a", "#d6d9d4", True,  ("red", None, "reverse|bold")),
    "invc": ("#0b5f70", "#d6d9d4", True,  ("cyan", None, "reverse|bold")),
    # boot art
    "aw":   ("#d6965a", None, False, ("yellow", None, 0)),
    "am":   ("#8c9eaa", None, False, ("white", None, 0)),
    "ap":   ("#8fcf7a", None, False, ("green", None, 0)),
    "au":   ("#d77be0", None, False, ("magenta", None, 0)),
    "ab":   ("#5a8cf0", None, False, ("blue", None, 0)),
    "ay":   ("#f2be46", None, False, ("yellow", None, "bold")),
    "ar":   ("#ff6b5e", None, False, ("red", None, 0)),
    "ao":   ("#f08c3c", None, False, ("yellow", None, 0)),
    "ac":   ("#6cc7d9", None, False, ("cyan", None, 0)),
    "ag":   ("#c9cec8", None, False, ("white", None, 0)),
    "as":   ("#6b5444", None, False, ("yellow", None, "dim")),
}

_attrs: dict[str, int] = {}

_LEVELS = (0, 95, 135, 175, 215, 255)


def _rgb(hexcode: str) -> tuple[int, int, int]:
    h = hexcode.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def xterm256(hexcode: str) -> int:
    """Nearest xterm-256 palette index (6x6x6 cube or grey ramp)."""
    r, g, b = _rgb(hexcode)

    def nearest_level(v: int) -> int:
        return min(range(6), key=lambda i: abs(_LEVELS[i] - v))

    ci = [nearest_level(v) for v in (r, g, b)]
    cube = tuple(_LEVELS[i] for i in ci)
    cube_idx = 16 + 36 * ci[0] + 6 * ci[1] + ci[2]
    grey_i = max(0, min(23, round((sum((r, g, b)) / 3 - 8) / 10)))
    grey = 8 + 10 * grey_i

    def dist(c: tuple[int, int, int]) -> int:
        return (c[0] - r) ** 2 + (c[1] - g) ** 2 + (c[2] - b) ** 2

    return cube_idx if dist(cube) <= dist((grey, grey, grey)) else 232 + grey_i


def _flags(spec) -> int:
    if not spec:
        return 0
    names = {"dim": curses.A_DIM, "bold": curses.A_BOLD, "reverse": curses.A_REVERSE}
    out = 0
    for part in str(spec).split("|"):
        out |= names[part]
    return out


def init() -> None:
    """Set up colour pairs. Call once inside curses.wrapper."""
    _attrs.clear()
    if not curses.has_colors():
        for role, (_, _, bold, (_, _, fb)) in ROLES.items():
            _attrs[role] = _flags(fb) | (curses.A_BOLD if bold else 0)
        return
    curses.start_color()
    try:
        curses.use_default_colors()
        default_bg = -1
    except curses.error:
        default_bg = curses.COLOR_BLACK
    rich = curses.COLORS >= 256 and curses.COLOR_PAIRS > len(ROLES) + 1
    colours = {
        "black": curses.COLOR_BLACK, "red": curses.COLOR_RED,
        "green": curses.COLOR_GREEN, "yellow": curses.COLOR_YELLOW,
        "blue": curses.COLOR_BLUE, "magenta": curses.COLOR_MAGENTA,
        "cyan": curses.COLOR_CYAN, "white": curses.COLOR_WHITE,
    }
    bg_idx = xterm256(BG) if rich else default_bg
    for n, (role, (fg, bg, bold, (ffg, fbg, fattr))) in enumerate(ROLES.items(), start=1):
        if rich:
            curses.init_pair(n, xterm256(fg), xterm256(bg) if bg else bg_idx)
            _attrs[role] = curses.color_pair(n) | (curses.A_BOLD if bold else 0)
        else:
            curses.init_pair(n, colours[ffg], colours[fbg] if fbg else default_bg)
            _attrs[role] = curses.color_pair(n) | _flags(fattr)


def attr(role: str) -> int:
    return _attrs.get(role, 0)
