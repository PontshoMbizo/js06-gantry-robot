"""
The gantry itself: link health, the command queue, homing, scan/spray jobs, calibration, e-stop.

No curses here. app.py turns keys and typed commands into calls on Gantry, render.py draws
its state. One command is outstanding on the UART at a time; ESTOP and ABORT are written straight away.

Timeouts (EMBED-08): every command must hear something within reply_timeout (1 s). Quick
commands are dropped after one silent second; long ones (MOVE, JOG, DRIVE, STEP, SPRAY) keep
waiting, because the firmware sends PROG every 0.5 s while they run. Each silent second counts
as a miss, and three in a row means LINK LOST: motion is blocked, position becomes "last
known" and needs a re-home. While a long command runs the Pi sends KA every 0.5 s; the
firmware stops every axis if it hears nothing from the Pi for 1 s.

Firmware gantry_fw 2.x runs homing, the span calibration, the end-switch stops and all the
filtering on the ESP32. The Pi reads the limit switches and forwards them (switches.Forwarder);
here it only sends HOME / SPAN / STEP and follows the replies.
"""

import csv
import json
import math
import os
import re
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from model import Axis, Bed, LogLine, Settings, Weed, footprint, in_frames, nearest_neighbour, scan_frames, vtag
from switches import FAR_SWITCH, HOME_SWITCH

QUICK = {"PING", "POS?", "PAUSE", "RESUME", "ABORT", "RESET", "ESTOP", "STOP", "SET", "CFG?"}
FW_WANTED = "2."
POS_RE = re.compile(r"(?<![A-Z=])([XYZ])(-?\d+(?:\.\d+)?)\b")
HOMED_RE = re.compile(r"HOMED=([XYZ-]+)")
SLOTS_PER_REV = 30


@dataclass
class Pending:
    line: str
    after: Callable | None = None   # called with the OK/PONG line
    err: Callable | None = None     # called with the ERR line
    prog: Callable | None = None    # called with each PROG line
    data: Callable | None = None    # called with each D line (step-test data)
    silent: bool = False            # heartbeat: not shown unless it fails
    sent_at: float = 0.0
    last_reply: float = 0.0

    @property
    def quick(self) -> bool:
        return self.line.split()[0] in QUICK


@dataclass
class Step:
    line: str
    unit: int                       # first step of this frame/spray: where :resume restarts
    safe: bool = False              # a pause may stop just before this step
    before: Callable | None = None
    after: Callable | None = None
    prog: Callable | None = None


@dataclass
class Job:
    kind: str                       # "scan" | "weed"
    steps: list
    started: float                  # monotonic
    started_wall: float
    i: int = 0
    state: str = "running"          # running | pausing | paused | stopped | finishing
    reason: str = ""
    waiting: bool = False
    # scan
    frames: list = field(default_factory=list)
    frames_done: int = 0
    start_frame: int = 0
    frame_now: int = 0
    captured: list = field(default_factory=list)
    dry: bool = False
    thresh: float = 0.6
    last_frame: tuple | None = None   # (boxes, max_conf)
    # weed (spray pass)
    targets: list = field(default_factory=list)
    current: Weed | None = None


# ----------------------------------------------------------------------
# procedures: homing and calibration react to the switches, so they are small state machines
# ----------------------------------------------------------------------
class Proc:
    kind = ""

    def __init__(self, g):
        self.g = g
        self.done = False
        self.error = ""
        self.phase = ""

    def label(self) -> str:
        return self.kind

    def tick(self) -> None:
        pass

    def fail(self, why: str) -> None:
        g = self.g
        self.error = why
        self.done = True
        g.say("!", why)
        g.event(f"{self.kind} failed: {why}")

    def abort(self, why: str) -> None:
        self.error = why
        self.done = True


def explain(line: str) -> str:
    """Firmware errors in words."""
    if line.startswith("ERR LIMIT"):
        return f"{line}: stopped at the end switch"
    if line.startswith("ERR TIMEOUT"):
        return "move timed out: blocked axis, motor not turning or sensor not counting · k → v checks the sensors"
    if line.startswith("ERR STALL"):
        return f"{line}: driven but no sensor pulses (old firmware) · re-home"
    if line.startswith("ERR SW_LOST"):
        return "ERR SW_LOST: the ESP32 stopped getting the switch states from the Pi (switch wiring or backend)"
    if line.startswith("ERR HOME"):
        return f"{line} (CTRL-06)"
    if line.startswith("ERR SOFT_LIMIT"):
        return f"{line}: outside the soft limits"
    return line


class HomeProc(Proc):
    """HOME on the ESP32: Z, X, Y; each axis drives to its home switch (z_top, x_min, y_min) and is homed
    the moment that switch is pressed: zero there, no back-off, no parking. With no switches (SET SW 0)
    the ESP makes this spot 0."""

    kind = "homing"

    def __init__(self, g, axes: str):
        super().__init__(g)
        self.axes = [a for a in g.s.home_order if a in axes]
        self.datum = not g.s.switches_connected
        self.phase = "set datum" if self.datum else "→ " + " ".join(self.axes)
        for a in self.axes:
            g.axes[a].homed = False
        g.event("datum here" if self.datum else f"homing {' '.join(self.axes)}: start")
        g.send("HOME " + "".join(self.axes), after=self.finished, err=self.failed, prog=self.progress)

    def label(self) -> str:
        return "set datum" if self.datum else "homing " + " ".join(self.axes)

    def progress(self, line: str) -> None:
        g = self.g
        self.phase = " ".join(f"{a}{g.axes[a].pos:.0f}" for a in self.axes)

    def finished(self, line: str) -> None:
        g = self.g
        self.done = True
        g.pos_trusted = all(x.homed for x in g.axes.values())
        if self.datum:
            g.say("·", f"datum set {' '.join(self.axes)} here · no switches, so soft limits are relative to this spot")
        else:
            g.say("·", f"homed {' '.join(self.axes)}")
        g.event(line)

    def failed(self, line: str) -> None:
        self.fail(f"homing: {explain(line)}")


class SpanProc(Proc):
    """SPAN on the ESP32: switch to switch on X or Y, the raw count at each trip (CTRL-06 note: true mm per count, belt slip)."""

    kind = "span"

    def __init__(self, g, axis: str, passes: int):
        super().__init__(g)
        self.axis = axis
        self.passes = passes
        self.trips: list[tuple[str, int]] = []
        self.legs = 2 * passes + 1                  # min, max, min, max, ... ending on min
        self.result = None
        self.phase = "running"
        g.event(f"span {axis}: start, {passes} passes")
        g.send(f"SPAN {axis} {passes}", after=self.finished, err=lambda l: self.fail(f"span {axis}: {explain(l)}"))

    def label(self) -> str:
        return f"span {self.axis} · {self.legs} legs"

    def finished(self, line: str) -> None:
        m = re.search(r"C=(-?\d+(?:,-?\d+)*)", line)
        if not m:
            return self.fail(f"span {self.axis}: no counts in '{line}'")
        counts = [int(c) for c in m.group(1).split(",")]
        names = (HOME_SWITCH[self.axis], FAR_SWITCH[self.axis])
        self.trips = [(names[k % 2], c) for k, c in enumerate(counts)]
        for sw, c in self.trips:
            self.g.event(f"span {self.axis}: {sw} at count {c}")
        self.finish()

    def finish(self) -> None:
        g, a = self.g, self.axis
        spans = [abs(self.trips[k + 1][1] - self.trips[k][1]) for k in range(len(self.trips) - 1)]
        mean = sum(spans) / len(spans)
        mm = g.s.span_mm(a)
        self.result = {
            "axis": a, "spans": spans, "mean": mean, "spread": max(spans) - min(spans),
            "nominal_mm": mean * g.s.mmpc(a), "span_mm": mm,
            "mmpc": (mm / mean) if mm > 0 else None, "wall": g.wall(),
        }
        g.span_result = self.result
        self._save()
        self.phase = "park"
        g.send(f"MOVE {a}{g.s.home_offset:g}", after=lambda _l: self._parked(),
               err=lambda l: self.fail(f"span {a}: {explain(l)}"))

    def _parked(self) -> None:
        r = self.result
        self.done = True
        txt = f"span {r['axis']}: {r['mean']:.1f} counts (spread {r['spread']})"
        txt += f" · {r['mmpc']:.4f} mm/count · a applies it" if r["mmpc"] else " · set span_" + r["axis"].lower() + "_mm to get mm/count"
        self.g.say("·", txt)
        self.g.event(txt)

    def _save(self) -> None:
        g, r = self.g, self.result
        try:
            os.makedirs(g.s.calib_dir, exist_ok=True)
            path = os.path.join(g.s.calib_dir, f"span_{r['axis'].lower()}_{time.strftime('%Y%m%d_%H%M%S', time.localtime(g.wall()))}.csv")
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["leg", "switch", "raw_count", "span_counts"])
                for k, (sw, c) in enumerate(self.trips):
                    w.writerow([k, sw, c, r["spans"][k - 1] if k else ""])
                f.write(f"# mean {r['mean']:.2f} counts, spread {r['spread']}, mm/count in use {g.s.mmpc(r['axis'])}, "
                        f"measured span {r['span_mm']} mm, mm/count {r['mmpc']}\n")
            r["file"] = path
        except OSError as e:
            g.say("!", f"span result not saved: {e}")


class StepProc(Proc):
    """Open-loop step tests (system ID). The ESP logs every raw sensor edge; each run is saved
    unprocessed as <axis>_<volts>_<pos|neg>_test.csv (edge,t_us)."""

    kind = "step"

    def __init__(self, g, axis: str, volts: float, d: int, pulses: int, repeats: int, alternate: bool, timeout: float):
        super().__init__(g)
        self.axis, self.volts, self.dir, self.pulses = axis, volts, d, pulses
        self.repeats, self.alternate, self.timeout = repeats, alternate, timeout
        self.run_i = 0
        self.rows: list[str] = []
        self.saved: list[str] = []
        self.problems: list[str] = []
        self.start_run()

    def label(self) -> str:
        return f"step test {self.axis} {self.volts:g} V · run {self.run_i + 1}/{self.repeats}"

    def run_dir(self) -> int:
        return -self.dir if self.alternate and self.run_i % 2 else self.dir

    def start_run(self) -> None:
        g, a = self.g, self.axis
        self.rows = []
        d = self.run_dir()
        self.phase = "run"
        g.send(f"STEP {a} {self.volts:g} {'+' if d > 0 else '-'} {self.pulses} {self.timeout:g}", after=self.finished,
               err=lambda l: self.fail(f"step test: {explain(l)}"), data=self.rows.append)

    def finished(self, line: str) -> None:
        g = self.g
        kv = dict(p.split("=", 1) for p in line.split() if "=" in p)
        d = self.run_dir()
        res = save_step_run(g, self.axis, self.volts, d, self.pulses, self.rows, kv)
        g.step_results.append(res)
        del g.step_results[:-12]
        g.event(f"step test {self.axis} {self.volts:g} V dir {d:+d}: {res['edges']} raw edges, {res['n']} pulses, end={res['end']}"
                + (f" · {res['path']}" if res.get("path") else f" · NOT SAVED: {res.get('why')}"))
        if res.get("path"):
            self.saved.append(res["file"])
        else:
            self.problems.append(res.get("why", "not saved"))
        self.run_i += 1
        if self.run_i < self.repeats and res["end"] not in ("stop", "abort", "limit", "sw_lost") and res.get("path"):
            self.start_run()
        else:
            self.done = True
            if self.saved:
                where = os.path.dirname(res.get("path") or os.path.join(g.s.sysid_dir, "x"))
                txt = f"saved {', '.join(self.saved)} in {where}"
                if res["end"] not in ("ticks",):
                    txt += f" · last run ended: {res['end']}"
                if self.problems:
                    txt += f" · {self.problems[-1]}"
                g.say("·" if not self.problems else "!", txt)
            else:
                g.say("!", f"step test NOT saved: {self.problems[-1] if self.problems else 'no data'}")


class SensorCheckProc(Proc):
    """Calibration only: drive each axis a few ticks with a short step test and see whether its
    sensor answers. Nothing is saved. The ESP ends a step test with end=stall when no pulse comes
    for 2 s, which is the only stall check left (moves, homing and the span don't have one)."""

    kind = "sensor check"

    def __init__(self, g, axes: list):
        super().__init__(g)
        self.axes = axes
        self.i = 0
        self.rows: list[str] = []
        self.results: dict = {}
        self.start_axis()

    @property
    def axis(self) -> str:
        return self.axes[self.i]

    def label(self) -> str:
        return f"sensor check {self.axis} ({self.i + 1}/{len(self.axes)})"

    def direction(self, a: str) -> int:
        g = self.g
        st = g.sw_state or {}
        if st.get(FAR_SWITCH.get(a, "")):
            return -1
        if st.get(HOME_SWITCH[a]):
            return 1
        if g.axes[a].homed:
            lo, hi = g.s.limits(a)
            room = g.s.check_ticks * g.s.mmpc(a) + 5
            return 1 if g.axes[a].pos + room <= hi else -1
        return 1

    def start_axis(self) -> None:
        g, a = self.g, self.axis
        self.rows = []
        d = self.direction(a)
        self.phase = f"{a} {'+' if d > 0 else '-'}"
        g.send(f"STEP {a} {g.s.check_v:g} {'+' if d > 0 else '-'} {g.s.check_ticks} {g.s.check_timeout:g}",
               after=self.finished, err=self.failed, data=self.rows.append)

    def judge(self, kv: dict) -> tuple[str, str]:
        try:
            n, edges = int(kv.get("pulses", 0)), int(kv.get("edges", 0))
        except ValueError:
            n, edges = 0, 0
        end = kv.get("end", "?")
        if end == "limit" and n == 0:
            return "skip", "at an end switch or soft limit: move it away and check again"
        if n == 0:
            if edges:
                return "fail", f"no clean pulses, {edges} noise edges: sensor wire picking up motor noise"
            return "fail", "no pulses in 2 s: sensor not seeing the slots (unplugged, no 3V3, misaligned) or motor not turning"
        if n < self.g.s.check_ticks:
            return "warn", f"only {n} of {self.g.s.check_ticks} pulses (end={end}): slow or sticking"
        if edges > 3 * n:
            return "warn", f"ok, but noisy: {edges} raw edges for {n} pulses"
        return "ok", f"{n} pulses, clean ({edges} raw edges)"

    def finished(self, line: str) -> None:
        g, a = self.g, self.axis
        kv = dict(p.split("=", 1) for p in line.split() if "=" in p)
        status, text = self.judge(kv)
        self.results[a] = (status, text)
        g.sensor_check_results[a] = (status, text)
        sps = step_speed(self.rows)
        g.step_results.append({"axis": a, "volts": g.s.check_v, "dir": 1 if "+" in self.phase else -1,
                               "n": int(kv.get("pulses", 0) or 0), "edges": len(self.rows), "req": g.s.check_ticks,
                               "end": kv.get("end", "?"), "rpm": sps / SLOTS_PER_REV * 60 if sps else None,
                               "mm_s": sps * g.s.mmpc(a) if sps else None,
                               "file": {"ok": "sensor ok", "warn": "sensor: check", "fail": "SENSOR FAIL", "skip": "not checked"}[status]})
        del g.step_results[:-12]
        g.event(f"sensor check {a}: {status} · {text}")
        self.next()

    def failed(self, line: str) -> None:
        a = self.axis
        self.results[a] = ("fail", explain(line))
        self.g.sensor_check_results[a] = self.results[a]
        self.g.event(f"sensor check {a}: {line}")
        if line.startswith(("ERR ESTOP", "ERR SW_LOST", "ERR BUSY")):
            return self.fail(f"sensor check: {explain(line)}")
        self.next()

    def next(self) -> None:
        self.i += 1
        if self.i < len(self.axes):
            return self.start_axis()
        self.done = True
        word = {"ok": "ok", "warn": "check", "fail": "FAIL", "skip": "skipped"}
        summary = " · ".join(f"{a} {word[s]}" for a, (s, _t) in self.results.items())
        bad = [f"{a}: {t}" for a, (s, t) in self.results.items() if s in ("fail", "warn", "skip")]
        self.g.say("·" if not bad else "!", f"sensors: {summary}" + (f" · {bad[0]}" if bad else ""))


def step_speed(rows: list[str], min_gap_us: int = 3000) -> float | None:
    """Steady speed in slots/s for the screen, from the raw edges with noise edges (closer than
    min_gap_us to the previous one) skipped. The CSV keeps every edge."""
    ts, last = [], None
    for r in rows:
        try:
            t = int(r.split(",")[1])
        except (IndexError, ValueError):
            continue
        if last is None or t - last >= min_gap_us:
            ts.append(t)
            last = t
    if len(ts) < 6:
        return None
    half = ts[len(ts) // 2:]
    dt = (half[-1] - half[0]) / 1e6
    return (len(half) - 1) / dt if dt > 0 else None


def free_name(folder: str, base: str) -> str:
    p = os.path.join(folder, base + ".csv")
    k = 2
    while os.path.exists(p):
        p = os.path.join(folder, f"{base}_{k}.csv")
        k += 1
    return p


def step_file_base(axis: str, volts: float, d: int) -> str:
    return f"{axis.upper()}_{volts:.1f}_{'pos' if d > 0 else 'neg'}_test"


def save_step_run(g, axis: str, volts: float, d: int, pulses_req: int, rows: list[str], kv: dict) -> dict:
    """{axis}_{volts}_{pos|neg}_test.csv: edge,t_us, one row per raw sensor edge, nothing else.
    Repeats get _2, _3 ...; the run's details go in runs_log.csv next to it."""
    sps = step_speed(rows)
    mmpc = g.s.mmpc(axis)
    try:
        n = int(kv.get("pulses", 0))
    except ValueError:
        n = 0
    res = {"axis": axis, "volts": volts, "dir": d, "n": n, "edges": len(rows), "req": pulses_req,
           "end": kv.get("end", "?"), "lost": kv.get("lost", "0"),
           "rpm": sps / SLOTS_PER_REV * 60 if sps else None, "mm_s": sps * mmpc if sps else None, "file": None, "path": None}
    started = bool(rows) or kv.get("t_end", "0") not in ("", "0")
    if not started:
        res["why"] = {"limit": f"{axis} is at an end switch or soft limit in that direction: move it away first",
                      "sw_lost": "the ESP32 stopped getting the switch states",
                      "stop": "stopped before the voltage was applied"}.get(res["end"], f"the test did not start (end={res['end']})")
        return res
    try:
        os.makedirs(g.s.sysid_dir, exist_ok=True)
        path = free_name(g.s.sysid_dir, step_file_base(axis, volts, d))
        stamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(g.wall()))
        with open(path, "w") as f:
            f.write("edge,t_us\n" + "\n".join(rows) + "\n")
        logp = os.path.join(g.s.sysid_dir, "runs_log.csv")
        new = not os.path.exists(logp)
        with open(logp, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["file", "axis", "volts", "dir", "ticks_req", "raw_edges", "pulses", "end", "t_end_us",
                            "edges_lost", "travel_mm", "rpm", "rail_v", "time"])
            w.writerow([os.path.basename(path), axis.upper(), volts, "+" if d > 0 else "-", pulses_req, len(rows), n,
                        res["end"], kv.get("t_end", ""), res["lost"], round(n * mmpc, 1),
                        round(res["rpm"], 2) if res["rpm"] else "", g.s.rail_v, stamp])
        res["file"] = os.path.basename(path)
        res["path"] = path
    except OSError as e:
        res["why"] = f"cannot write {g.s.sysid_dir}: {e}"
    return res


# ----------------------------------------------------------------------
class Gantry:
    def __init__(self, settings: Settings, bed: Bed, link, vision, system, switches,
                 clock=time.monotonic, wall=time.time, camera=None):
        self.s = settings
        self.camera = camera
        self.fw = ""
        self.bed = bed
        self.link = link
        self.vision = vision
        self.system = system
        self.switches = switches
        self.clock = clock
        self.wall = wall

        self.axes = {a: Axis(a) for a in "XYZ"}
        self.pos_trusted = False
        self.log: list[LogLine] = []
        self.notice: tuple[str, str] | None = None   # (mark, text) for row -1

        self.queue: deque[Pending] = deque()
        self.pending: Pending | None = None
        self.missed = 0
        self.last_contact = clock()
        self.last_ok_wall: float | None = None
        self.latency_ms: int | None = None
        self.last_ka = 0.0

        self.link_lost = False
        self.link_lost_wall: float | None = None
        self.retry_n = 0
        self.next_retry = 0.0
        self.boot_text_seen = False
        self.config_pushed = False

        self.estop = False
        self.estop_reason = ""
        self.estop_wall = 0.0
        self.estop_t = 0.0
        self.estop_confirmed: bool | None = None

        self.sw_state = {}
        self.sw_prev = {}

        self.weeds: list[Weed] = []
        self.reviewed = False
        self.scan_summary: dict | None = None
        self.weed_summary: dict | None = None
        self.job: Job | None = None
        self.proc: Proc | None = None
        self.resume_scan_from = 0
        self.heartbeat_on = True
        self.infer_times: list[float] = []
        self.step_results: list[dict] = []
        self.span_result: dict | None = None
        self.sensor_check_results: dict = {}

        self._logfile = None
        self._open_logfile()
        self._load_state()

    # --- log ----------------------------------------------------------
    def _open_logfile(self) -> None:
        try:
            os.makedirs(self.s.log_dir, exist_ok=True)
            name = time.strftime("gantry-%Y%m%d.log", time.localtime(self.wall()))
            self._logfile = open(os.path.join(self.s.log_dir, name), "a", encoding="utf-8")
        except OSError:
            self._logfile = None

    def _write_file(self, mark: str, text: str) -> None:
        if self._logfile:
            stamp = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.wall()))
            self._logfile.write(f"{stamp} {mark} {text}\n")
            self._logfile.flush()

    def uart(self, mark: str, text: str) -> None:
        last = self.log[-1] if self.log else None
        if mark == "←" and text.startswith("PROG") and last and last.mark == mark and last.text.split()[:2] == text.split()[:2]:
            last.t, last.text = self.wall(), text    # PROG every 0.5 s: refresh, don't scroll the panel
            return
        self.log.append(LogLine(self.wall(), mark, text))
        del self.log[:-500]
        self._write_file(mark, text)

    def event(self, text: str) -> None:
        """A processing-stage event (SW-04): file log only, plus the full log view."""
        self.log.append(LogLine(self.wall(), "#", text))
        del self.log[:-500]
        self._write_file("#", text)

    def say(self, mark: str, text: str) -> None:
        self.notice = (mark, text)

    def _state_path(self) -> str:
        return os.path.join(self.s.log_dir, "state.json")

    def _load_state(self) -> None:
        try:
            with open(self._state_path(), encoding="utf-8") as f:
                self.weed_summary = json.load(f).get("last_weed")
        except (OSError, ValueError):
            pass

    def _save_state(self) -> None:
        try:
            with open(self._state_path(), "w", encoding="utf-8") as f:
                json.dump({"last_weed": self.weed_summary}, f)
        except OSError:
            pass

    def spray_log(self, w: Weed, outcome: str, ms: int = 0) -> None:
        """ACT-06: every spray decision, with coordinates, time and outcome."""
        path = os.path.join(self.s.log_dir, "sprays.csv")
        try:
            new = not os.path.exists(path)
            with open(path, "a", newline="") as f:
                wr = csv.writer(f)
                if new:
                    wr.writerow(["time", "weed", "x_mm", "y_mm", "conf", "near_crop", "gap_mm", "outcome", "spray_ms", "spray_z_mm"])
                wr.writerow([time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self.wall())), w.id, f"{w.x:.1f}", f"{w.y:.1f}",
                             f"{w.conf:.2f}", w.near_kind, f"{w.near_mm:.0f}", outcome, ms, self.s.spray_z])
        except OSError:
            pass
        self.event(f"spray log: weed {w.id} at X{w.x:g} Y{w.y:g} · {outcome}")

    # --- queue --------------------------------------------------------
    def send(self, line: str, after=None, err=None, prog=None, silent=False, data=None) -> None:
        self.queue.append(Pending(line, after, err, prog, data, silent))

    def write_now(self, line: str) -> None:
        """Bypass the queue (ESTOP, ABORT)."""
        self.link.write(line)
        self.uart("→", line)

    def _pump(self) -> None:
        if self.pending or not self.queue:
            return
        p = self.queue.popleft()
        now = self.clock()
        p.sent_at = p.last_reply = now
        self.pending = p
        if not p.silent:
            self.uart("→", p.line)
        self.link.write(p.line)
        self.last_ka = now

    def _keepalive(self) -> None:
        p = self.pending
        if p and not p.quick and self.clock() - self.last_ka >= self.s.keepalive:
            self.last_ka = self.clock()
            self.link.write("KA")

    def _check_timeout(self) -> None:
        p = self.pending
        if not p:
            return
        limit = self.s.reply_timeout
        now = self.clock()
        if now - p.last_reply < limit:
            return
        self.missed += 1
        lost_now = not self.link_lost and self.missed >= self.s.missed_for_lost
        if p.quick or lost_now:
            self.pending = None
        else:
            p.last_reply = now      # a long command: keep waiting, but count the silence
        if p.silent:
            self.uart("→", p.line)
        self.uart("×", f"timeout {limit:g} s" + (" · link lost" if lost_now else ""))
        if lost_now:
            if self.job:
                self.job.waiting = False
            self._lose_link()

    def _lose_link(self) -> None:
        self.link_lost = True
        self.link_lost_wall = self.last_ok_wall
        self.queue.clear()
        self.retry_n = 0
        self.next_retry = self.clock() + self.s.ping_every
        self.pos_trusted = False
        for a in self.axes.values():
            a.homed = False
        if self.job:
            self._stop_job("link lost")
        if self.proc:
            self.proc.abort("link lost")
            self.proc = None
        when = time.strftime("%H:%M:%S", time.localtime(self.last_ok_wall)) if self.last_ok_wall else "boot"
        self.say("×", f"ESP32 stopped answering at {when} · retrying every {self.s.ping_every:g} s")
        self.event("link lost")

    def retry_link(self) -> None:
        self.retry_n = 0
        self.next_retry = self.clock()

    def _link_retry(self) -> None:
        if not self.link_lost or self.pending or self.retry_n >= self.s.ping_tries:
            return
        if self.clock() < self.next_retry:
            return
        self.retry_n += 1
        self.next_retry = self.clock() + self.s.ping_every
        when = time.strftime("%H:%M:%S", time.localtime(self.link_lost_wall)) if self.link_lost_wall else "boot"
        tail = f"({self.retry_n}/{self.s.ping_tries})"
        if self.retry_n >= self.s.ping_tries:
            tail += " · press r to retry"
        self.say("×", f"ESP32 stopped answering at {when} · retrying every {self.s.ping_every:g} s {tail}")
        self.send("PING", after=lambda _l: None)

    def _heartbeat(self) -> None:
        if not self.heartbeat_on or self.link_lost or self.pending or self.queue:
            return
        if self.clock() - self.last_contact >= self.s.heartbeat:
            self.send("POS?", silent=True)

    # --- firmware settings ----------------------------------------------
    def push_config(self, after=None) -> None:
        """Send the axis settings from gantry.json to the ESP (it starts from its own defaults)."""
        s = self.s
        lines = [f"SET RAIL {s.rail_v:g}",
                 f"SET HOME {s.home_fast_v:g} {s.home_slow_v:g} {s.home_offset:g}",
                 f"SET SW {1 if s.switches_connected else 0}"]
        for a in "XYZ":
            lo, hi = s.limits(a)
            kp, kd, vlim, v0 = s.gain(a)
            lines += [f"SET MMPC {a} {s.mmpc(a):g}", f"SET SOFT {a} {lo:g} {hi:g}",
                      f"SET GAIN {a} {kp:g} {kd:g} {vlim:g} {v0:g}", f"SET SIGN {a} {s.sign(a)}"]
        for i, ln in enumerate(lines):
            last = i == len(lines) - 1

            def ok(_l, last=last):
                if last:
                    self.config_pushed = True
                    self.event("settings sent to the ESP32")
                    if after:
                        after(True)
            self.send(ln, after=ok, err=lambda l, ln=ln: self.say("!", f"settings refused: {ln} → {l}") or (after and after(False)), silent=True)
        self.send("CFG?", after=self._got_cfg, silent=True)

    def _got_cfg(self, line: str) -> None:
        m = re.search(r"FW=(\S+)", line)
        self.fw = m.group(1) if m else "?"
        if not self.fw.startswith(FW_WANTED):
            self.say("!", f"ESP32 firmware is {self.fw}: flash firmware/gantry_fw (2.x) for this CLI")

    # --- replies ------------------------------------------------------
    def _dispatch(self, line: str, now: float | None = None) -> None:
        if not line:
            return
        now = self.clock() if now is None else now
        word = line.split()[0]
        p = self.pending
        if line == "ESTOP":
            self.uart("←", line)
            self.press_estop(button=True)
            return
        if word in ("OK", "PONG", "PROG", "ERR", "D"):
            self.last_contact = now
            self.missed = 0
            if self.link_lost and word in ("OK", "PONG"):
                self._regain_link()
        if word == "D":
            if p and p.data:
                p.last_reply = now
                p.data(line[2:].strip())
            return
        if word == "BOOT":
            self.uart("·", line)
            self.boot_text_seen = True
            self._esp_restarted()
            return
        if word == "OK" and "ESTOP" in line:
            self.uart("←", line)
            self.estop_confirmed = True
            if p and p.line == "ESTOP":
                self.pending = None
            return
        if word in ("OK", "PONG"):
            self.last_ok_wall = self.wall()
            self._read_position(line)
            if p and (word == "OK" or p.line == "PING"):
                self.pending = None
                self.latency_ms = round((now - p.sent_at) * 1000) if p.quick else self.latency_ms
                if not p.silent:
                    self.uart("←", line)
                if p.after:
                    p.after(line)
            elif not (p and p.silent):
                self.uart("←", line)
            return
        if word == "PROG":
            self.uart("←", line)
            self._read_position(line)
            if p:
                p.last_reply = now
                if p.prog:
                    p.prog(line)
            return
        if word == "ERR":
            self.uart("!", line)
            self.say("!", explain(line))
            m = re.match(r"ERR STALL ([XYZ])", line)
            if m:
                self.axes[m.group(1)].homed = False
                self.pos_trusted = False
                self.say("!", f"stall on {m.group(1)}: drive cut, axis needs a re-home (CTRL-08)")
            if line.startswith("ERR WATCHDOG"):
                self.say("!", "ESP32 stopped the axes: it heard nothing from the Pi for 1 s")
            self._read_position(line)
            if p:
                self.pending = None
                if p.err:
                    p.err(line)
            if "ESTOP" in line and not self.estop:
                # the ESP32 is latched but the Pi isn't (e.g. the Pi restarted): latch here too,
                # so the screen shows it and :reset sends RESET
                self.press_estop(button=True, reason="The ESP32 reports its e-stop latched.")
            return
        if line.startswith("ESP-ROM") or line.startswith("rst:"):
            self.boot_text_seen = True
        self.uart("·", line)

    def _esp_restarted(self) -> None:
        was = [a for a, x in self.axes.items() if x.homed]
        for x in self.axes.values():
            x.homed = False
        self.pos_trusted = False
        self.config_pushed = False
        if self.pending:
            p, self.pending = self.pending, None
            if p.err:
                p.err("ERR RESTARTED")
        if self.job:
            self._stop_job("ESP32 restarted")
        if self.proc:
            self.proc.abort("ESP32 restarted")
            self.proc = None
        self.push_config()
        self.say("!", "ESP32 restarted · settings re-sent · position lost, re-home (h)" if was else "ESP32 restarted · settings re-sent")
        self.event("ESP32 restarted")

    def _regain_link(self) -> None:
        self.link_lost = False
        self.say("·", "ESP32 is back · position not trusted until re-home (h)")
        self.event("link regained")
        self.push_config()

    def _read_position(self, line: str) -> None:
        for a, v in POS_RE.findall(line):
            self.axes[a].pos = float(v)
        m = HOMED_RE.search(line)
        if m:
            for a in "XYZ":
                self.axes[a].homed = a in m.group(1)
            self.pos_trusted = all(x.homed for x in self.axes.values())

    # --- switches -------------------------------------------------------
    def _check_switches(self) -> None:
        """For the screen only: the ESP32 stops at the switches itself (it gets them from the Forwarder)."""
        st = self.switches.read()
        self.sw_prev, self.sw_state = self.sw_state, st

    def switch_fault(self) -> str | None:
        st = self.sw_state or self.switches.read()
        if st.get("x_min") and st.get("x_max"):
            return "x_min and x_max both pressed"
        if st.get("y_min") and st.get("y_max"):
            return "y_min and y_max both pressed"
        return None

    # --- tick ---------------------------------------------------------
    def tick(self) -> None:
        for t, line in self.link.poll():
            self._dispatch(line.strip(), t)
        self._check_switches()
        self._check_timeout()
        if self.estop and self.estop_confirmed is None and self.clock() - self.estop_t > self.s.reply_timeout:
            self.estop_confirmed = False
        if self.proc:
            self.proc.tick()
            if self.proc.done and not self.pending and not self.queue:
                self.proc = None
        self._tick_capture()
        self._advance_job()
        self._poll_vision()
        self._link_retry()
        self._heartbeat()
        self._pump()
        self._keepalive()

    # --- guards -------------------------------------------------------
    def blocked(self, need_home: bool = False) -> str | None:
        """Why a motion command can't run right now, or None."""
        if self.estop:
            return "motion refused: e-stop latched · type :reset first"
        if self.link_lost:
            return "motion blocked: ESP32 not answering"
        if self.proc and not self.proc.done:
            return f"{self.proc.label()} is running · :abort ends it"
        if self.job and self.job.state not in ("paused", "stopped"):
            return f"a {'spray pass' if self.job.kind == 'weed' else self.job.kind} is running · p to pause, :abort to end it"
        if need_home and not all(a.homed for a in self.axes.values()):
            return "not homed · press h to home first"
        return None

    def motion_active(self) -> bool:
        if self.job and self.job.state in ("running", "pausing", "finishing"):
            return True
        if self.proc and not self.proc.done:
            return True
        p = self.pending
        return bool(p and not p.quick)

    def activity(self) -> str:
        j = self.job
        if self.proc and not self.proc.done:
            return self.proc.label()
        if j and j.kind == "weed" and j.current and j.state in ("running", "pausing"):
            return f"spraying weed {j.current.id}"
        if j and j.kind == "scan" and j.state in ("running", "pausing"):
            return f"scanning frame {j.frame_now}"
        p = self.pending
        if p and p.line.startswith("SPRAY"):
            return "spraying"
        if p and not p.quick:
            return "moving"
        return "idle"

    # --- e-stop -------------------------------------------------------
    def press_estop(self, button: bool = False, reason: str = "") -> None:
        if self.estop:
            return
        what = self.activity()
        self.estop = True
        self.estop_t = self.clock()
        self.estop_wall = self.wall()
        who = "The ESP32" if button else "SPACE"
        if reason:
            self.estop_reason = reason
        else:
            self.estop_reason = f"{who} was pressed while {what}." if what != "idle" else f"{who} was pressed."
        self.queue.clear()
        self.pending = None
        self.notice = None
        if self.job:
            self._stop_job("e-stop")
        if self.proc:
            self.proc.abort("e-stop")
            self.proc = None
        if button:
            self.estop_confirmed = True
        else:
            self.estop_confirmed = None
            self.uart("→", "ESTOP")
            self.link.write("ESTOP")
        self.event("e-stop latched: " + self.estop_reason)

    def reset(self) -> str | None:
        if not self.estop:
            return "nothing to reset: e-stop is not latched"

        def done(_line):
            self.estop = False
            self.say("·", "e-stop cleared")
            self.event("e-stop reset")
            self.send("POS?")

        self.send("RESET", after=done)
        return None

    # --- homing, status, jog -----------------------------------------
    def home(self, axes: str = "XYZ") -> str | None:
        why = self.blocked()
        if why:
            return why
        if self.s.switches_connected and not self.switches.available:
            return f"limit switches not readable ({self.switches.error or self.switches.backend}) · see README, switches"
        if self.s.switches_connected and self.switch_fault():
            return f"{self.switch_fault()}: a broken wire on an NC switch reads as pressed (CTRL-06)"
        self.pos_trusted = False
        self.proc = HomeProc(self, axes)
        return None

    def status(self) -> None:
        self.send("POS?")

    def check_move(self, target: dict) -> str | None:
        """Pi-side soft limits (the ESP checks them too once an axis is homed)."""
        for a, v in target.items():
            if not self.axes[a].homed:
                continue
            lo, hi = self.s.limits(a)
            here = self.axes[a].pos
            # moving back toward the range is allowed: a homed axis sits on its switch, below the soft limit
            if (v < lo - 1e-6 and v < here) or (v > hi + 1e-6 and v > here):
                return f"{a}{v:g} is outside the soft limit {lo:g} – {hi:g}"
        return None

    def jog(self, axis: str, delta: float) -> str | None:
        why = self.blocked()
        if why:
            return why
        if not self.axes[axis].homed and abs(delta) > 50:
            return f"{axis} is not homed: jog steps up to 50 mm only"
        target = {axis: round(self.axes[axis].pos + delta, 3)}
        sign = "+" if delta >= 0 else "-"
        label = f"{axis}{sign}{abs(delta):g}"
        ahead = HOME_SWITCH[axis] if delta < 0 else FAR_SWITCH.get(axis)
        if ahead and (self.sw_state or {}).get(ahead):
            return f"{label} refused: {ahead} is pressed (the carriage is at the end of travel)"
        bad = self.check_move(target)
        if bad:
            return f"last move refused: {label} {bad}"
        self.send(f"JOG {label}", err=lambda l: None)
        return None

    def goto(self, x: float, y: float) -> str | None:
        why = self.blocked(need_home=True)
        if why:
            return why
        bad = self.check_move({"X": x, "Y": y})
        if bad:
            return f"go to refused: {bad}"
        self.send(f"MOVE X{x:g} Y{y:g}", err=lambda l: None)
        return None

    def spray_here(self) -> str | None:
        """One test spray at the current position, refused inside a crop clearance (ACT-09)."""
        why = self.blocked()
        if why:
            return why
        x, y = self.axes["X"].pos, self.axes["Y"].pos
        crop, gap = self.bed.nearest_crop(x, y)
        if self.pos_trusted and crop and gap < self.s.guard:
            return f"spray refused: {gap:.0f} mm from a {crop.kind}, inside the {self.s.guard:g} mm clearance (ACT-09)"
        self.send(f"SPRAY {self.s.spray_ms}", err=lambda l: None)
        self.event(f"test spray at X{x:g} Y{y:g} · {self.s.spray_ms} ms")
        return None

    # --- calibration --------------------------------------------------
    def step_test(self, axis: str, volts: float, d: int, pulses: int, repeats: int = 1,
                  alternate: bool = False, timeout: float | None = None) -> str | None:
        why = self.blocked()
        if why:
            return why
        axis = axis.upper()
        if axis not in "XYZ" or len(axis) != 1:
            return "axis must be x, y or z"
        if not 0 < volts <= self.s.rail_v:
            return f"volts must be above 0 and at most the rail ({self.s.rail_v:g} V)"
        if d not in (1, -1):
            return "direction must be + or -"
        if not 1 <= pulses <= 5000:
            return "ticks must be 1 to 5000"
        timeout = self.s.step_timeout if timeout is None else timeout
        self.proc = StepProc(self, axis, volts, d, pulses, max(1, repeats), alternate, timeout)
        return None

    def sensor_check(self, axes: str = "ZXY") -> str | None:
        why = self.blocked()
        if why:
            return why
        axes = [a for a in axes.upper() if a in "XYZ"]
        if not axes:
            return "axes must be x, y and/or z"
        self.proc = SensorCheckProc(self, axes)
        return None

    def span_test(self, axis: str, passes: int | None = None) -> str | None:
        why = self.blocked()
        if why:
            return why
        axis = axis.upper()
        if axis not in ("X", "Y"):
            return "span needs a switch at each end: x or y only (Z has one switch)"
        if not self.axes[axis].homed:
            return f"home {axis} first (h or :home {axis.lower()})"
        if not self.s.switches_connected:
            return "span needs the limit switches (switches_connected is false in gantry.json)"
        if not self.switches.available:
            return f"limit switches not readable ({self.switches.backend})"
        if self.switch_fault():
            return f"switch fault: {self.switch_fault()}"
        self.proc = SpanProc(self, axis, passes or self.s.span_passes)
        return None

    def apply_span(self) -> str | None:
        r = self.span_result
        if not r:
            return "no span result yet · run a span test first"
        if not r.get("mmpc"):
            return f"set span_{r['axis'].lower()}_mm (the measured distance between the switches) first"
        key = f"mm_per_count_{r['axis'].lower()}"
        why = self.blocked()
        if why:
            return why
        self.s.set(key, round(r["mmpc"], 5))
        err = self.s.save({key: round(r["mmpc"], 5)})
        self.send(f"SET MMPC {r['axis']} {r['mmpc']:.5f}", after=lambda _l: None)
        self.event(f"applied {key} = {r['mmpc']:.5f}")
        self.say("·" if not err else "!", f"{key} = {r['mmpc']:.5f}" + (" saved to gantry.json · re-home recommended" if not err else f" (not saved: {err})"))
        return None

    # --- jobs ---------------------------------------------------------
    def start_scan(self, rows=None, thresh=None, dry=False, resume=False) -> str | None:
        why = self.blocked(need_home=True)
        if why:
            return why
        if not dry and not getattr(self.vision, "detects", False):
            return "weed detection not wired yet: see vision.py · :scan --dry still moves the camera"
        frames = scan_frames(self.s, self.bed, rows)
        start = self.resume_scan_from if resume else 0
        if resume and not start:
            return "no stopped scan to resume"
        if not resume:
            self.weeds = []
            self.reviewed = False
        job = Job("scan", [], self.clock(), self.wall(), frames=frames, dry=dry,
                  thresh=self.s.thresh if thresh is None else thresh)
        job.frames_done = job.start_frame = start
        job.captured = list(frames[:start])
        steps = [Step(f"MOVE Z{self.s.camera_z:g}", 0)]
        for f in frames[start:]:
            unit = len(steps)
            steps.append(Step(f"MOVE X{f.x:g} Y{f.y:g}", unit, safe=True,
                              before=lambda f=f: setattr(job, "frame_now", f.n)))
            steps.append(Step(f"CAPTURE {f.n:02d}", unit, after=lambda _l, f=f: self._captured(job, f)))
        job.steps = steps
        self.job = job
        self.say("·", "scanning · p pauses after this frame · a aborts and parks the camera")
        self.event(f"scan start · {len(frames) - start} frames" + (" · dry" if dry else ""))
        return None

    def _captured(self, job: Job, f) -> None:
        job.frames_done = f.n
        job.captured.append(f)
        self.resume_scan_from = f.n if f.n < len(job.frames) else 0
        fw, fh = footprint(self.s, self.bed)
        self.event(f"capture frame {f.n} at X{f.x:g} Y{f.y:g}")
        if not job.dry:
            self.vision.submit(f.n, f.x, f.y, fw, fh, job.thresh)

    def revealed(self, x: float, y: float) -> bool:
        """During a scan the map only shows what the camera has seen."""
        j = self.job
        if not (j and j.kind == "scan"):
            return True
        fw, fh = footprint(self.s, self.bed)
        return in_frames(x, y, j.captured, fw, fh)

    def _poll_vision(self) -> None:
        for frame, dets, infer_s, boxes, max_conf in self.vision.poll():
            self.infer_times.append(infer_s)
            if self.job and self.job.kind == "scan":
                self.job.last_frame = (boxes, max_conf)
            scanning = self.job and self.job.kind == "scan"
            for d in dets:
                dup = next((w for w in self.weeds if math.hypot(w.x - d.x, w.y - d.y) < self.s.merge_mm), None)
                if dup:      # the same weed seen again where frames overlap: keep the more confident one
                    if d.conf > dup.conf:
                        dup.x, dup.y, dup.conf, dup.frame = d.x, d.y, d.conf, frame
                    continue
                crop, gap = self.bed.nearest_crop(d.x, d.y)
                wid = 0 if scanning else max((w.id for w in self.weeds), default=0) + 1
                close = gap < self.s.guard
                self.weeds.append(Weed(wid, d.x, d.y, d.conf, frame, crop.kind if crop else "", gap,
                                       ticked=not close, guarded=close))
            self.event(f"inference frame {frame} · {len(dets)} weeds · {infer_s:.1f} s")

    def _finish_scan(self, job: Job, aborted: bool = False) -> None:
        """Number the weeds along the map and record the summary. An aborted
        scan keeps what it found so far and can still be --resumed."""
        self.weeds.sort(key=lambda w: self.bed.hv(w.x, w.y))
        for i, w in enumerate(self.weeds, start=1):
            w.id = i
        close = sum(1 for w in self.weeds if w.guarded)
        self.scan_summary = {"wall": job.started_wall, "frames": job.frames_done,
                             "secs": self.clock() - job.started, "found": len(self.weeds),
                             "close": close, "dry": job.dry}
        self.resume_scan_from = job.frames_done if aborted and job.frames_done < len(job.frames) else 0
        self.job = None
        n = len(self.weeds)
        word = "scan aborted" if aborted else "scan done"
        self.say("·", f"{word} · {n} weed{'s' if n != 1 else ''}" + (" · press w to review" if n else ""))
        self.event(f"{word} · {n} weeds · {close} inside the crop clearance · coordinates ready")

    def tick_target(self, wid: int, on: bool) -> str | None:
        w = next((w for w in self.weeds if w.id == wid), None)
        if not w:
            return f"no weed #{wid}"
        if on and w.guarded:
            return (f"#{w.id} is {w.near_mm:.0f} mm from a {w.near_kind}, inside the {self.s.guard:g} mm clearance: "
                    "never sprayed, logged as untreated (ACT-09)")
        w.ticked = on
        return None

    def start_weed(self) -> str | None:
        why = self.blocked(need_home=True)
        if why:
            return why
        open_ = [w for w in self.weeds if w.status in ("found", "queued", "failed")]
        targets = [w for w in open_ if w.ticked and not w.guarded]
        if not targets:
            return "no ticked targets"
        for w in open_:
            if w.guarded:
                w.status, w.ticked = "untreated", False
                self.spray_log(w, f"untreated: {w.near_mm:.0f} mm from {w.near_kind}, clearance {self.s.guard:g} mm (ACT-09)")
            elif not w.ticked:
                w.status = "skipped"
                self.spray_log(w, "skipped by the operator")
            else:
                w.status = "queued"
        start = (self.axes["X"].pos, self.axes["Y"].pos)
        order = nearest_neighbour(start, targets)
        job = Job("weed", [], self.clock(), self.wall(), targets=order)
        steps = []
        if abs(self.axes["Z"].pos - self.s.safe_z) > 0.5:
            steps.append(Step(f"MOVE Z{self.s.safe_z:g}", 0))
        for w in order:
            unit = len(steps)
            steps.append(Step(f"MOVE X{w.x:g} Y{w.y:g}", unit, safe=True, before=lambda w=w: self._spray_start(job, w)))
            steps.append(Step(f"MOVE Z{self.s.spray_z:g}", unit))
            steps.append(Step(f"SPRAY {self.s.spray_ms}", unit, after=lambda _l, w=w: self._sprayed(w)))
            steps.append(Step(f"MOVE Z{self.s.safe_z:g}", unit))
        job.steps = steps
        self.job = job
        self.say("·", "spraying · keep hands off the bed")
        self.event(f"removal pass start · {len(order)} targets · order {' '.join(str(w.id) for w in order)}")
        return None

    def _spray_start(self, job: Job, w: Weed) -> None:
        job.current = w
        w.status = "spraying"
        w.spray_s = self.clock()

    def _sprayed(self, w: Weed) -> None:
        w.status = "sprayed"
        w.spray_s = self.clock() - w.spray_s
        self.spray_log(w, "sprayed", self.s.spray_ms)

    def _finish_weed(self, job: Job) -> None:
        sprayed = sum(1 for w in self.weeds if w.status == "sprayed")
        skipped = sum(1 for w in self.weeds if w.status == "skipped")
        untreated = sum(1 for w in self.weeds if w.status == "untreated")
        self.weed_summary = {"wall": job.started_wall, "sprayed": sprayed, "skipped": skipped, "untreated": untreated}
        self._save_state()
        self.job = None
        self.say("·", f"spray pass done · {sprayed} sprayed · {untreated} untreated · {skipped} skipped")
        self.event(f"removal pass done · {sprayed} sprayed · {untreated} untreated · {skipped} skipped")

    def _advance_job(self) -> None:
        j = self.job
        if not j or self.pending or self.queue or j.waiting:
            return
        if j.state == "finishing" or (j.state in ("running", "pausing") and j.i >= len(j.steps)):
            if j.kind == "scan":
                if self.vision.busy():
                    j.state = "finishing"
                    return
                self._finish_scan(j)
            else:
                self._finish_weed(j)
            return
        if j.state not in ("running", "pausing"):
            return
        step = j.steps[j.i]
        if step.safe and j.state == "pausing":
            j.state = "paused"
            where = f"frame {j.frames_done}" if j.kind == "scan" else "this spray"
            j.reason = f"paused after {where}"
            self.say("·", f"{j.reason} · :resume or :abort")
            self.event(f"{j.kind} paused")
            self.send("PAUSE")
            return
        if step.before:
            step.before()
        j.waiting = True

        def ok(line, step=step):
            j.waiting = False
            j.i += 1
            if step.after:
                step.after(line)

        def err(line):
            j.waiting = False
            self._stop_job(line)

        if step.line.startswith("CAPTURE"):
            # captures run on the Pi: wait settle seconds at the frame, no UART command
            j.waiting = True
            due = self.clock() + self.s.settle
            self._capture_due = (due, ok)
            return
        self.send(step.line, after=ok, err=err, prog=step.prog)

    _capture_due = None

    def _tick_capture(self) -> None:
        if self._capture_due and self.clock() >= self._capture_due[0]:
            ok = self._capture_due[1]
            self._capture_due = None
            ok("OK CAPTURE")

    def _stop_job(self, reason: str) -> None:
        j = self.job
        if not j:
            return
        self._capture_due = None
        j.state = "stopped"
        j.reason = reason
        j.waiting = False
        if j.i < len(j.steps):
            j.i = j.steps[j.i].unit
        if j.kind == "weed" and j.current and j.current.status == "spraying":
            j.current.status = "queued"
        self.event(f"{j.kind} stopped: {reason}")

    def pause(self) -> str | None:
        j = self.job
        if not j or j.state not in ("running",):
            return "nothing running to pause"
        j.state = "pausing"
        what = "this frame" if j.kind == "scan" else "this spray"
        self.say("·", f"pausing after {what}")
        return None

    def resume(self) -> str | None:
        j = self.job
        if not j or j.state not in ("paused", "stopped"):
            return "nothing to resume"
        if self.estop or self.link_lost:
            return self.blocked()
        if not all(a.homed for a in self.axes.values()):
            return "not homed · press h to home, then :resume"
        if j.i < len(j.steps):
            j.i = j.steps[j.i].unit
            if abs(self.axes["Z"].pos - self.s.safe_z) > 0.5 and not j.steps[j.i].line.startswith("MOVE Z"):
                j.steps.insert(j.i, Step(f"MOVE Z{self.s.safe_z:g}", j.i))
                for k in range(j.i + 1, len(j.steps)):
                    j.steps[k].unit += 1
        j.state = "running"
        j.reason = ""
        self.send("RESUME")
        self.say("·", f"{'spray pass' if j.kind == 'weed' else j.kind} resumed")
        self.event(f"{j.kind} resumed")
        return None

    def abort(self) -> str | None:
        if self.proc and not self.proc.done:
            label = self.proc.label()
            self.proc.abort("aborted")
            self.proc = None
            self.queue.clear()
            now = self.clock()
            self.pending = Pending("ABORT", sent_at=now, last_reply=now)
            self.uart("→", "ABORT")
            self.link.write("ABORT")
            self.send("POS?")
            self.say("·", f"{label} aborted")
            self.event(f"{label} aborted")
            return None
        j = self.job
        if not j:
            return "nothing to abort"
        if self.estop:
            if j.kind == "scan":
                self._finish_scan(j, aborted=True)
            self.job = None
            self.event(f"{j.kind} aborted while latched")
            self.say("·", f"{j.kind} aborted")
            return None
        self.queue.clear()
        self.job = None
        self._capture_due = None
        for w in self.weeds:
            if w.status in ("queued", "spraying"):
                w.status = "found"
        # ABORT jumps the queue like ESTOP: it must stop the move in progress, not wait behind it
        now = self.clock()
        self.pending = Pending("ABORT", sent_at=now, last_reply=now)
        self.uart("→", "ABORT")
        self.link.write("ABORT")
        self.send("POS?")
        if not self.link_lost and all(a.homed for a in self.axes.values()):
            if abs(self.axes["Z"].pos - self.s.safe_z) > 0.5:
                self.send(f"MOVE Z{self.s.safe_z:g}")
            self.send(f"MOVE X{self.s.park_x:g} Y{self.s.park_y:g}")
        if j.kind == "scan":
            self._finish_scan(j, aborted=True)
        self.say("·", f"{'spray pass' if j.kind == 'weed' else j.kind} aborted · parking")
        self.event(f"{j.kind} aborted")
        return None

    def eta(self) -> float | None:
        j = self.job
        if not j:
            return None
        if j.kind == "scan":
            done = j.frames_done - j.start_frame
            if done <= 0:
                return None
            per = (self.clock() - j.started) / done
            return per * (len(j.frames) - j.frames_done)
        done = [w.spray_s for w in j.targets if w.status == "sprayed"]
        per = sum(done) / len(done) if done else 8.0
        left = sum(1 for w in j.targets if w.status in ("queued", "spraying"))
        return per * left

    def close(self) -> None:
        if self.camera:
            self.camera = None
        self.switches.close()
        self.link.close()
        if self._logfile:
            self._logfile.close()


def mmss(secs: float | None) -> str:
    if secs is None or math.isinf(secs):
        return "--:--"
    secs = int(round(secs))
    return f"{secs // 60:02d}:{secs % 60:02d}"
