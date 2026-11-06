"""
The : command line. Commands use the same words as the keys, so a run can
also be scripted over ssh (SW-05).

COMPLETIONS feeds the popup that opens over the JOB panel: TAB completes the
highlighted row, the arrows pick a row, and with an empty line they walk
history instead.
"""

COMPLETIONS = [
    ("home", "home all axes (Z, X, then Y)"),
    ("home y", "home one axis"),
    ("scan", "capture the bed, detect weeds"),
    ("scan --dry", "move the camera only, no detection"),
    ("scan --rows 1-2", "only frames near these crop rows"),
    ("scan --thresh 0.7", "override confidence for this run"),
    ("scan --resume", "continue the last stopped scan"),
    ("review", "tick targets before any spray"),
    ("weed all", "spray every ticked target"),
    ("weed 1,2", "spray only these ids"),
    ("jog", "manual move with the arrow keys"),
    ("calib", "calibration: step tests and mm/count"),
    ("step y 3 + 100", "step test: axis volts dir pulses"),
    ("step y 3 + 100 3 alt", "3 runs, alternating direction"),
    ("check", "sensor check: does each axis count?"),
    ("span x", "switch to switch: true mm/count"),
    ("apply", "use the last span result"),
    ("spray", "one test spray here"),
    ("photo", "take a photo here (logs/photos)"),
    ("pause", "stop at the next safe point"),
    ("resume", "carry on a paused or stopped job"),
    ("abort", "end the job or procedure and park"),
    ("reset", "clear the e-stop latch"),
    ("status", "ask the ESP32 where it is"),
    ("log 50", "show the last n log lines"),
    ("net ap", "start the gantry-net access point"),
    ("net online", "stop the AP and join the hotspot"),
    ("set spray_ms 300", "change a setting for this session"),
    ("config", "settings and network"),
    ("help step", "help for one command"),
    ("quit", "leave the CLI"),
]

ALSO = [
    "home [x|y|z|all]  jog  review  weed <ids|all>",
    "      calib  step  span  apply  pause  resume  abort",
    "      status  log  net  set <key> <value>  help <cmd>",
]

HELP = {
    "home": "home [x|y|z|all] · on the ESP32: drive each axis to its switch (Z top, X min, Y min), homed when pressed · no switches: datum here",
    "photo": "photo · one photo at the current position, saved as logs/photos/photo_<time>_X.._Y.._Z...jpg (c on the jog screen)",
    "scan": "scan [--rows a-b] [--thresh v] [--dry] [--resume] · MOVE + capture per frame",
    "review": "review · tick targets; weeds inside the crop clearance are never sprayed (ACT-09)",
    "weed": "weed <ids|all> · asks y/N, then per target: MOVE X Y · MOVE Z spray height · SPRAY · MOVE Z safe",
    "jog": "jog · arrows move the axes as drawn on the map, w/s Z up/down, 1-5 step (6 = type your own), shift = 1 mm, f f spray, c photo",
    "calib": "calib · step tests (system ID) and the switch-to-switch mm/count calibration",
    "step": "step <x|y|z> <volts> <+|-> <ticks> [repeats] [alt] · open-loop step test, raw edges saved as <axis>_<volts>_<pos|neg>_test.csv",
    "check": "check [xyz] · drive each axis check_ticks at check_v and see if its sensor answers (calibration screen: v)",
    "span": "span <x|y> [passes] · drive between the two switches and count; set span_x_mm / span_y_mm for mm/count",
    "apply": "apply · write the last span result's mm/count to gantry.json and the ESP32",
    "spray": "spray · one spray of spray_ms at the current position (refused inside a crop clearance)",
    "pause": "pause · stops after the current frame or spray",
    "resume": "resume · restarts the current frame or spray from its first move",
    "abort": "abort · ends the job or procedure, raises the nozzle and parks",
    "reset": "reset · clears the e-stop latch (RESET)",
    "status": "status · POS?",
    "log": "log [n] · full log view, newest last",
    "net": "net ap | net online · runs net_ap_cmd / net_online_cmd from gantry.json",
    "set": "set <key> <value> · e.g. set spray_ms 300, set guard 45, set span_x_mm 612.5",
    "help": "help <cmd>",
    "quit": "quit · asks first if a job is running",
    "config": "config · settings and network",
}

WORDS = sorted({c.split()[0] for c, _ in COMPLETIONS})


def matches(text: str) -> list[tuple[str, str]]:
    """Completion rows for what has been typed so far: every row whose
    command word starts with the typed word, so `scan --` still lists `scan`."""
    t = text.strip()
    if not t:
        return COMPLETIONS[:5]
    first = t.split()[0]
    return [c for c in COMPLETIONS if c[0].split()[0].startswith(first)]


def best(text: str, rows: list, pick: int) -> int:
    """Row to highlight: the user's pick once they have moved, else the first
    row the typed text is a prefix of."""
    if pick or not rows:
        return min(pick, len(rows) - 1) if rows else 0
    t = text.strip()
    for i, (c, _) in enumerate(rows):
        if c.startswith(t) and c != t:
            return i
    return 0


def parse_ids(arg: str) -> list[int]:
    out = []
    for part in arg.replace(" ", "").split(","):
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def parse_rows(arg: str) -> tuple[int, int]:
    if "-" in arg:
        a, b = arg.split("-", 1)
        return int(a), int(b)
    return int(arg), int(arg)


def parse_dir(arg: str) -> int:
    t = str(arg).strip().lower()
    if t in ("+", "+1", "1", "f", "fwd", "forward"):
        return 1
    if t in ("-", "-1", "r", "rev", "reverse", "back"):
        return -1
    raise ValueError("direction must be + or -")
