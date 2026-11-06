"""
Screens, keys and the command line, on top of controller.Gantry.

Keys arrive as str (characters, from get_wch) or int (curses.KEY_*).
SPACE is read before any other key in a batch. It latches the e-stop on
every screen; the two places where it also has a local meaning (typing a
space on the command line, ticking a target in review) only get it while
nothing is moving. While anything moves, SPACE is always the e-stop and the
rest of the batch is dropped.
"""

import curses
import os

import commands
from controller import Gantry
from switches import NAMES as SWITCH_NAMES

ESC = "\x1b"
ENTER = ("\n", "\r", curses.KEY_ENTER)
BACKSPACE = ("\x7f", "\b", curses.KEY_BACKSPACE)

BOOT_CHECKS = ["uart", "esp32", "config", "switches", "camera", "model", "ap", "ssh"]
CONFIG_SECTIONS = ["network", "esp32 link", "axes", "switches", "spray", "detection", "bed layout", "calibration", "logs", "about"]

# calibration screen fields: (settings key, label, kind)
CALIB_FIELDS = [
    ("step_axis", "axis", "axis"),
    ("step_volts", "voltage", "volts"),
    ("step_dir", "direction", "dir"),
    ("step_pulses", "pulses", "pulses"),
    ("step_repeats", "repeats", "repeats"),
    ("step_alternate", "alternate dir", "bool"),
    ("step_timeout", "timeout", "secs"),
    ("span_axis", "axis", "span_axis"),
    ("span_passes", "passes", "repeats"),
    ("span_mm", "measured span", "span_mm"),
]
STEP_FIELDS = 7


class App:
    def __init__(self, gantry: Gantry, sim_link=None, version: str = "v1.0"):
        self.g = gantry
        self.sim_link = sim_link
        self.version = version
        self.mode = "boot"            # boot | dash | review | jog | config | log | calib
        self.running = True
        self.quitting_at: float | None = None

        self.cmd: str | None = None   # None: command line not focused
        self.hist: list[str] = []
        self.hist_pos: int | None = None
        self.pick = 0
        self.prompt: tuple[str, object] | None = None        # y/N question on row -1
        self.ask: tuple[str, str, object] | None = None      # (label, text, on_submit)
        self.help_open = False

        self.review_sel = 0
        self.preview = True
        self.config_sec = 0
        self.log_scroll = 0
        self.f_armed: float | None = None
        self.calib_sel = 0
        self.span_axis = "X"

        self._probes: dict = {}
        self.boot_checks = {name: ("wait", "") for name in BOOT_CHECKS}
        self.boot_i = 0
        self.boot_next = gantry.clock() + 0.25
        self.boot_phase = "checking"   # checking | prompt | homing
        gantry.heartbeat_on = False

    # --- per-tick -----------------------------------------------------
    def tick(self) -> None:
        self.g.tick()
        self._photo_tick()
        if self.mode == "boot":
            self._boot_tick()
        if self.quitting_at is not None:
            idle = not self.g.pending and not self.g.queue
            if idle or self.g.clock() - self.quitting_at > 5:
                self.running = False
        if self.mode == "review" and self.g.job and self.g.job.kind == "weed":
            self.mode = "dash"

    def _boot_tick(self) -> None:
        g = self.g
        if self.boot_phase == "homing":
            if not g.proc or g.proc.done:
                self._boot_done()
            return
        if self.boot_phase != "checking" or g.clock() < self.boot_next:
            return
        if self.boot_i >= len(BOOT_CHECKS):
            self.boot_phase = "prompt"
            return
        name = BOOT_CHECKS[self.boot_i]
        state, _ = self.boot_checks[name]
        if name == "esp32":
            if state == "wait":
                self.boot_checks[name] = ("busy", "")
                g.send("PING", after=lambda _l: self.boot_checks.update(esp32=("ok", f"PONG {g.latency_ms} ms")))
                return
            if state == "busy":
                if g.pending or g.queue:
                    return
                self.boot_checks[name] = ("fail", "no reply")
        elif name == "config":
            if self.boot_checks["esp32"][0] != "ok":
                self.boot_checks[name] = ("fail", "no ESP32")
            elif state == "wait":
                self.boot_checks[name] = ("busy", "")
                g.push_config(after=lambda ok: self.boot_checks.update(
                    config=("ok", "axis settings sent") if ok else ("fail", "refused")))
                return
            elif state == "busy":
                if g.pending or g.queue:
                    return
                if self.boot_checks[name][0] == "busy":
                    self.boot_checks[name] = ("fail", "no reply")
        elif name == "camera" and g.camera is not None and g.camera.state == "checking":
            if state == "wait":
                self.boot_checks[name] = ("busy", "looking")
                self._cam_wait = g.clock()
            if g.clock() - getattr(self, "_cam_wait", g.clock()) < 25:
                return
            self.boot_checks[name] = ("warn", "still looking")
        else:
            self.boot_checks[name] = self._probe(name)
        self.boot_i += 1
        self.boot_next = g.clock() + 0.25

    def _probe(self, name: str) -> tuple[str, str]:
        g = self.g
        if name == "uart":
            return "ok", f"{g.link.baud} 8N1"
        if name == "switches":
            if not g.s.switches_connected:
                return "warn", "not connected (h sets the datum here)"
            sw = g.switches
            if not sw.available:
                return "fail", f"not readable ({sw.error or 'no backend'})"[:40]
            st = sw.read()
            fault = g.switch_fault()
            if fault:
                return "fail", fault
            on = [n for n in SWITCH_NAMES if st.get(n)]
            if not g.s.switch_nc and not on:
                return "warn", f"{sw.backend} · NO switches: press each one to check"
            return ("ok", f"{sw.backend} · all released") if not on else ("warn", f"{sw.backend} · pressed: {' '.join(on)}")
        if name == "camera":
            cam = g.camera
            if cam is None:
                return ("ok", g.vision.camera) if g.vision.camera else ("warn", "not wired")
            if cam.state == "ok":
                return "ok", cam.name[:40]
            return "warn", ("off" if cam.state == "off" else "not found")
        if name == "model":
            m = g.vision.model
            return ("ok", m.replace(".tflite", "")) if m else ("warn", "not wired")
        if name == "ap":
            up = g.system.service(g.s.ap_service)
            if up is None:
                return "warn", "unknown"
            return ("ok", g.s.ap_ssid) if up else ("warn", "inactive")
        if name == "ssh":
            n = g.system.ssh_sessions()
            if n is None:
                return "warn", "unknown"
            return "ok", f"{n} client" + ("" if n == 1 else "s")
        return "warn", ""

    def _boot_done(self) -> None:
        self.mode = "dash"
        self.g.heartbeat_on = True
        self.g.event("boot done")

    # --- SPACE --------------------------------------------------------
    def space_is_estop(self) -> bool:
        if self.g.motion_active():
            return True
        if self.cmd is not None or self.ask is not None:
            return False
        if self.mode == "review" and not self.prompt:
            return False
        return True

    def handle_keys(self, keys: list) -> None:
        if not keys:
            return
        if " " in keys and self.space_is_estop():
            self.g.press_estop()
            self.prompt = None
            self.ask = None
            if self.mode in ("jog", "review", "config", "log", "calib"):
                self.mode = "dash"
            return
        for k in keys:
            self.g.notice = None if self.g.notice and self.g.notice[0] == "·" else self.g.notice
            self.key(k)

    # --- keys ---------------------------------------------------------
    def key(self, k) -> None:
        if k == curses.KEY_RESIZE:
            return
        if self.prompt:
            text, on_yes = self.prompt
            self.prompt = None
            if k in ("y", "Y"):
                on_yes()
            else:
                self.g.say("·", "cancelled")
            return
        if self.ask:
            self._ask_key(k)
            return
        if self.cmd is not None:
            self._cmd_key(k)
            return
        if self.help_open:
            if k in (ESC, "?", "q"):
                self.help_open = False
            return
        if k == ":":
            self.cmd = ""
            self.pick = 0
            self.hist_pos = None
            self.g.notice = None
            return
        if k == "?":
            self.help_open = True
            return
        handler = getattr(self, f"_{self.mode}_key")
        handler(k)

    def _boot_key(self, k) -> None:
        if self.boot_phase != "prompt":
            if k == "q":
                self.running = False
            return
        if k == "y":
            err = self.g.home("XYZ")
            if err:
                self.g.say("!", err)
                self._boot_done()
            else:
                self.boot_phase = "homing"
        elif k == "n":
            self.g.say("·", "skipped homing · scan and spray stay blocked until you home (h)")
            self._boot_done()
        elif k == "q":
            self.running = False

    def _dash_key(self, k) -> None:
        g = self.g
        j = g.job
        if g.estop:
            if k == "l":
                self._open_log()
            return
        if k == "l":
            return self._open_log()
        if g.link_lost:
            if k == "r":
                g.retry_link()
            elif k == "c":
                self.mode = "config"
            return
        if g.proc and not g.proc.done:
            if k == "a":
                self._err(g.abort())
            elif k == "k":
                self.mode = "calib"
            return
        if j and j.state in ("running", "pausing", "finishing"):
            if k == "p":
                self._err(g.pause())
            elif k == "a" and j.kind == "scan":
                self._err(g.abort())
            return
        if j and j.state in ("paused", "stopped"):
            if k == "r":
                return self._err(g.resume())
            if k == "a":
                return self._err(g.abort())
        if k == "h":
            self._err(g.home("XYZ"))
        elif k == "s":
            self._err(g.start_scan())
        elif k == "w":
            self.open_review()
        elif k == "j":
            self._open_jog()
        elif k == "k":
            self.mode = "calib"
        elif k == "c":
            self.mode = "config"
        elif k == "q":
            self.request_quit()

    def open_review(self) -> None:
        g = self.g
        if g.job and g.job.kind == "scan":
            return self._err("wait for the scan to finish")
        if not any(w.status in ("found", "queued", "failed") for w in g.weeds):
            return self._err("no weeds to review · press s to scan")
        self.mode = "review"
        g.reviewed = True
        self.review_sel = min(self.review_sel, len(g.weeds) - 1)

    def _review_key(self, k) -> None:
        g = self.g
        n = len(g.weeds)
        if k == curses.KEY_UP:
            self.review_sel = (self.review_sel - 1) % n
        elif k == curses.KEY_DOWN:
            self.review_sel = (self.review_sel + 1) % n
        elif k == " ":
            w = g.weeds[self.review_sel]
            self._err(g.tick_target(w.id, not w.ticked))
        elif k == "v":
            self.preview = not self.preview
        elif k in ENTER:
            self.confirm_weed()
        elif k == ESC:
            self.mode = "dash"

    def confirm_weed(self) -> None:
        g = self.g
        why = g.blocked(need_home=True)
        if why:
            return self._err(why)
        open_ = [w for w in g.weeds if w.status in ("found", "queued", "failed")]
        todo = [w for w in open_ if w.ticked and not w.guarded]
        if not todo:
            return self._err("nothing ticked · space ticks a target")
        skip = [f"#{w.id}" for w in open_ if not w.ticked and not w.guarded]
        close = [f"#{w.id}" for w in open_ if w.guarded]
        text = f"Spray {len(todo)} weed{'s' if len(todo) != 1 else ''}"
        if skip:
            text += f" (skip {', '.join(skip)})" if len(skip) <= 2 else f" (skip {len(skip)})"
        if close:
            text += f" · {len(close)} untreated (ACT-09)"
        text += f"? Nozzle lowers to Z {g.s.spray_z:g} mm."

        def go():
            err = g.start_weed()
            if err:
                self._err(err)
            else:
                self.mode = "dash"

        self.prompt = (text, go)

    def _open_jog(self) -> None:
        why = self.g.blocked()
        if why:
            return self._err(why)
        self.mode = "jog"
        self.f_armed = None

    def jog_moves(self) -> dict:
        """Arrow keys follow the map: horizontal arrows move the axis drawn left-right."""
        h, v = self.g.bed.h_axis, self.g.bed.v_axis
        return {
            curses.KEY_LEFT: (h, -1, False), curses.KEY_RIGHT: (h, 1, False),
            curses.KEY_UP: (v, 1, False), curses.KEY_DOWN: (v, -1, False),
            curses.KEY_SLEFT: (h, -1, True), curses.KEY_SRIGHT: (h, 1, True),
            curses.KEY_SR: (v, 1, True), curses.KEY_SF: (v, -1, True),
            "w": ("Z", -1, False), "s": ("Z", 1, False),      # Z is measured down: w raises, s lowers
            "W": ("Z", -1, True), "S": ("Z", 1, True),
        }

    def _jog_key(self, k) -> None:
        g = self.g
        step = g.s.jog_step_mm()
        moves = self.jog_moves()
        if k in moves:
            axis, sign, fine = moves[k]
            if g.pending:
                return      # one jog at a time; holding a key must not queue up moves
            return self._err(g.jog(axis, sign * (1.0 if fine else step)))
        if k in ("1", "2", "3", "4", "5"):
            g.s.jog_step = int(k) - 1
        elif k == "6":
            self.ask = ("jog step in mm", "", self._own_step)
        elif k == "f":
            now = g.clock()
            if self.f_armed and now - self.f_armed < 3:
                self.f_armed = None
                self._err(g.spray_here())
            else:
                self.f_armed = now
                g.say("!", f"press f again to spray {g.s.spray_ms} ms here")
        elif k == "g":
            self.ask = ("go to x,y", "", self._goto)
        elif k == "c":
            self.photo()
        elif k in (ESC, "q"):
            self.mode = "dash"

    def photo(self) -> None:
        g = self.g
        cam = g.camera
        if cam is None:
            return self._err("no camera here (--sim has none)")
        pos = {a: g.axes[a].pos for a in "XYZ"}
        why = cam.capture(pos)
        if why:
            return self._err(why)
        self._photo_seen = cam.done_count
        g.say("·", "taking a photo …")

    def _photo_tick(self) -> None:
        cam = self.g.camera
        if cam is None or getattr(self, "_photo_seen", None) is None or cam.done_count == self._photo_seen:
            return
        self._photo_seen = None
        if cam.last_error:
            self.g.say("!", f"photo failed: {cam.last_error}")
        else:
            here = os.path.dirname(os.path.abspath(__file__))
            p = cam.last_photo
            self.g.say("·", f"photo saved: {os.path.relpath(p, here) if p.startswith(here + os.sep) else p}")
            self.g.event(f"photo {cam.last_photo}")

    def _own_step(self, text: str) -> None:
        s = self.g.s
        try:
            v = float(text.strip().lower().removesuffix("mm").strip())
        except ValueError:
            return self._err("step wants a number of mm, e.g. 25")
        top = max(s.travel_x[1], s.travel_y[1], s.travel_z[1])
        if not 0 < v <= top:
            return self._err(f"step must be above 0 and at most {top:g} mm")
        s.jog_custom = v
        s.jog_step = len(s.jog_steps)
        self.g.say("·", f"jog step {v:g} mm (key 6 to change it)")

    def _goto(self, text: str) -> None:
        try:
            x, y = (float(v) for v in text.replace(" ", "").split(","))
        except ValueError:
            return self._err("go to wants x,y in mm, e.g. 310,500")
        self._err(self.g.goto(x, y))

    # --- calibration screen -------------------------------------------
    def calib_value(self, i: int):
        key, _, kind = CALIB_FIELDS[i]
        s = self.g.s
        if kind == "span_axis":
            return self.span_axis
        if kind == "span_mm":
            return s.span_mm(self.span_axis)
        return getattr(s, key)

    def _calib_set(self, i: int, value) -> None:
        key, _, kind = CALIB_FIELDS[i]
        s = self.g.s
        if kind == "span_axis":
            self.span_axis = value
        elif kind == "span_mm":
            s.set(f"span_{self.span_axis.lower()}_mm", float(value))
        else:
            s.set(key, value)

    def _calib_adjust(self, i: int, sign: int) -> None:
        kind = CALIB_FIELDS[i][2]
        v = self.calib_value(i)
        if kind == "axis":
            order = "xyz"
            v = order[(order.index(v.lower()) + sign) % 3]
        elif kind == "span_axis":
            v = "Y" if v == "X" else "X"
        elif kind == "volts":
            v = round(min(self.g.s.rail_v, max(0.5, v + 0.5 * sign)), 2)
        elif kind == "dir":
            v = -v
        elif kind == "pulses":
            v = int(min(5000, max(10, v + 10 * sign)))
        elif kind == "repeats":
            v = int(min(20, max(1, v + sign)))
        elif kind == "bool":
            v = not v
        elif kind == "secs":
            v = float(min(60, max(1, v + 5 * sign)))
        elif kind == "span_mm":
            v = round(max(0.0, v + 0.5 * sign), 1)
        self._calib_set(i, v)

    def _calib_type(self, i: int) -> None:
        label = CALIB_FIELDS[i][1]

        def submit(text, i=i):
            kind = CALIB_FIELDS[i][2]
            t = text.strip().lower()
            try:
                if kind == "axis":
                    if t not in ("x", "y", "z"):
                        raise ValueError("axis must be x, y or z")
                    self._calib_set(i, t)
                elif kind == "span_axis":
                    if t not in ("x", "y"):
                        raise ValueError("span axis must be x or y")
                    self._calib_set(i, t.upper())
                elif kind == "dir":
                    self._calib_set(i, commands.parse_dir(t))
                elif kind == "bool":
                    self._calib_set(i, t in ("y", "yes", "1", "on", "true"))
                else:
                    self._calib_set(i, t)
            except (ValueError, KeyError) as e:
                self._err(str(e) or f"bad value for {label}")

        self.ask = (label, "", submit)

    def _calib_key(self, k) -> None:
        g = self.g
        n = len(CALIB_FIELDS)
        if k == curses.KEY_UP:
            self.calib_sel = (self.calib_sel - 1) % n
        elif k == curses.KEY_DOWN:
            self.calib_sel = (self.calib_sel + 1) % n
        elif k in (curses.KEY_LEFT, curses.KEY_RIGHT):
            self._calib_adjust(self.calib_sel, -1 if k == curses.KEY_LEFT else 1)
        elif k in ENTER:
            self._calib_type(self.calib_sel)
        elif k == "s":
            self.confirm_step()
        elif k == "m":
            self.confirm_span()
        elif k == "v":
            self.confirm_check()
        elif k == "a":
            if g.proc and not g.proc.done:
                return self._err(g.abort())
            self._err(g.apply_span())
        elif k in (ESC, "q"):
            self.mode = "dash"

    def confirm_step(self, axis=None, volts=None, d=None, pulses=None, repeats=None, alternate=None) -> None:
        g, s = self.g, self.g.s
        axis = (axis or s.step_axis).upper()
        volts = s.step_volts if volts is None else volts
        d = s.step_dir if d is None else d
        pulses = s.step_pulses if pulses is None else pulses
        repeats = s.step_repeats if repeats is None else repeats
        alternate = s.step_alternate if alternate is None else alternate
        why = g.blocked()
        if why:
            return self._err(why)
        travel = pulses * s.mmpc(axis)
        where = "" if g.axes[axis].homed else " · axis NOT homed: no soft limits, switches still stop it"
        text = (f"Step test {axis} {volts:g} V {'+' if d > 0 else '-'} · {pulses} pulses (≈{travel:.0f} mm) × {repeats}"
                f"{' alternating' if alternate and repeats > 1 else ''}{where}. Clear of both ends?")

        def go():
            self._err(g.step_test(axis, volts, d, pulses, repeats, alternate))

        self.prompt = (text, go)

    def confirm_check(self, axes: str = "ZXY") -> None:
        g, s = self.g, self.g.s
        why = g.blocked()
        if why:
            return self._err(why)
        axes = "".join(a for a in "ZXY" if a in axes.upper())
        text = (f"Sensor check: drive {' '.join(axes)} {s.check_ticks} ticks each at {s.check_v:g} V "
                f"(a few mm) and see if each sensor answers. Clear of the ends?")
        self.prompt = (text, lambda: self._err(g.sensor_check(axes)))

    def confirm_span(self, axis=None, passes=None) -> None:
        g = self.g
        axis = (axis or self.span_axis).upper()
        passes = passes or g.s.span_passes
        why = g.blocked()
        if why:
            return self._err(why)
        span = g.s.travel(axis)[1] - g.s.travel(axis)[0]
        text = f"Span {axis}: drive switch to switch {2 * passes + 1} times (≈{span:g} mm each)? Bed clear?"

        def go():
            self._err(g.span_test(axis, passes))

        self.prompt = (text, go)

    def _config_key(self, k) -> None:
        g = self.g
        if k == curses.KEY_UP:
            self.config_sec = (self.config_sec - 1) % len(CONFIG_SECTIONS)
        elif k == curses.KEY_DOWN:
            self.config_sec = (self.config_sec + 1) % len(CONFIG_SECTIONS)
        elif k in ENTER:
            self.cmd = "set "
        elif k in (ESC, "q"):
            self.mode = "dash"
        elif CONFIG_SECTIONS[self.config_sec] == "network":
            if k == "o":
                self.net("online")
            elif k == "r":
                self._net_run(g.s.net_restart_cmd, "restart the AP (about 10 s offline)", "net_restart_cmd")
            elif k == "p":
                g.say("·", f"edit wpa_passphrase in {g.s.hostapd_conf}, then press r to restart the AP")
            elif k == "k":
                host = (g.system.ap_info(g.s.hostapd_conf, g.s.ap_iface).get("address") or "192.168.4.1").split("/")[0]
                g.say("·", f"from your laptop:  ssh-copy-id {g.system.hostname()}@{host}")
        elif CONFIG_SECTIONS[self.config_sec] == "esp32 link" and k == "u":
            g.push_config(after=lambda ok: g.say("·" if ok else "!", "settings sent to the ESP32" if ok else "settings refused"))

    def net(self, which: str) -> None:
        g = self.g
        if which == "online":
            self._net_run(g.s.net_online_cmd, f"stop the AP and join {g.s.hotspot}? Your ssh session drops",
                          "net_online_cmd")
        elif which == "ap":
            self._net_run(g.s.net_ap_cmd, f"start {g.s.ap_service}", "net_ap_cmd")
        else:
            self._err("net ap | net online")

    def _net_run(self, cmd: list, what: str, key: str) -> None:
        g = self.g
        if not cmd:
            return self._err(f"not configured: set {key} in gantry.json")

        def go():
            ok, out = g.system.run_action(cmd)
            g.say("·" if ok else "!", out or ("done" if ok else "failed"))
            g.event(f"net action {key}: {'ok' if ok else 'failed'} {out}")

        self.prompt = (f"{what[0].upper()}{what[1:]}? runs: {' '.join(cmd)}", go)

    def _open_log(self) -> None:
        self.mode = "log"
        self.log_scroll = 0

    def _log_key(self, k) -> None:
        if k in (curses.KEY_UP, "k"):
            self.log_scroll += 1
        elif k in (curses.KEY_DOWN, "j"):
            self.log_scroll = max(0, self.log_scroll - 1)
        elif k == curses.KEY_PPAGE:
            self.log_scroll += 20
        elif k == curses.KEY_NPAGE:
            self.log_scroll = max(0, self.log_scroll - 20)
        elif k in (ESC, "q", "l"):
            self.mode = "dash"

    # --- prompts and the command line ---------------------------------
    def _ask_key(self, k) -> None:
        label, text, on_submit = self.ask
        if k in ENTER:
            self.ask = None
            on_submit(text)
        elif k == ESC:
            self.ask = None
        elif k in BACKSPACE:
            self.ask = (label, text[:-1], on_submit)
        elif isinstance(k, str) and k.isprintable():
            self.ask = (label, text + k, on_submit)

    def _cmd_key(self, k) -> None:
        if k == ESC:
            self.cmd = None
        elif k in ENTER:
            text, self.cmd = self.cmd.strip(), None
            if text:
                self.hist.append(text)
                self.run(text)
        elif k in BACKSPACE:
            self.cmd = self.cmd[:-1]
            self.pick, self.hist_pos = 0, None
        elif k == "\t":
            rows = commands.matches(self.cmd)[:5]
            if rows:
                self.cmd = rows[commands.best(self.cmd, rows, self.pick)][0]
                self.pick = 0
        elif k in (curses.KEY_UP, curses.KEY_DOWN):
            up = k == curses.KEY_UP
            if self.cmd == "" or self.hist_pos is not None:
                if not self.hist:
                    return
                if self.hist_pos is None:
                    self.hist_pos = len(self.hist)
                self.hist_pos += -1 if up else 1
                if self.hist_pos >= len(self.hist):
                    self.hist_pos, self.cmd = None, ""
                else:
                    self.hist_pos = max(0, self.hist_pos)
                    self.cmd = self.hist[self.hist_pos]
            else:
                n = len(commands.matches(self.cmd))
                if n:
                    self.pick = (self.pick + (-1 if up else 1)) % min(n, 5)
        elif isinstance(k, str) and k.isprintable():
            self.cmd += k
            self.pick, self.hist_pos = 0, None

    def _err(self, msg: str | None) -> None:
        if msg:
            self.g.say("!", msg)

    def request_quit(self) -> None:
        g = self.g
        if g.job or (g.proc and not g.proc.done):
            what = g.proc.label() if g.proc and not g.proc.done else ("spray pass" if g.job.kind == "weed" else g.job.kind)

            def go():
                g.abort()
                self.quitting_at = g.clock()
            self.prompt = (f"A {what} is running. Abort it and quit?", go)
        else:
            self.running = False

    def run(self, text: str) -> None:
        """Run one typed command."""
        g = self.g
        words = text.split()
        cmd, args = words[0].lower(), words[1:]
        try:
            if cmd == "home":
                axes = (args[0] if args else "all").upper()
                axes = "XYZ" if axes == "ALL" else axes
                if not axes or any(a not in "XYZ" for a in axes):
                    return self._err("home [x|y|z|all]")
                self._err(g.home(axes))
            elif cmd == "scan":
                rows = thresh = None
                dry = resume = False
                i = 0
                while i < len(args):
                    a = args[i]
                    if a == "--rows" and i + 1 < len(args):
                        rows = commands.parse_rows(args[i + 1])
                        i += 1
                    elif a == "--thresh" and i + 1 < len(args):
                        thresh = float(args[i + 1])
                        i += 1
                    elif a == "--dry":
                        dry = True
                    elif a == "--resume":
                        resume = True
                    else:
                        return self._err(commands.HELP["scan"])
                    i += 1
                self._err(g.start_scan(rows, thresh, dry, resume))
            elif cmd == "review":
                self.open_review()
            elif cmd == "weed":
                ids = list(args)
                if not ids:
                    return self._err(commands.HELP["weed"])
                open_ = [w for w in g.weeds if w.status in ("found", "queued", "failed")]
                if ids[0] == "all":
                    for w in open_:
                        w.ticked = not w.guarded
                else:
                    wanted = set(commands.parse_ids(",".join(ids)))
                    for w in open_:
                        if w.id in wanted:
                            err = g.tick_target(w.id, True)
                            if err:
                                return self._err(err)
                        else:
                            w.ticked = False
                self.open_review()
                if self.mode == "review":
                    self.confirm_weed()
            elif cmd == "jog":
                self._open_jog()
            elif cmd == "calib":
                self.mode = "calib"
            elif cmd == "step":
                # step <axis> <volts> <+|-> <pulses> [repeats] [alt]
                if len(args) < 4:
                    return self._err(commands.HELP["step"])
                rep = int(args[4]) if len(args) > 4 else 1
                alt = len(args) > 5 and args[5].lower().startswith("alt")
                self.confirm_step(args[0], float(args[1]), commands.parse_dir(args[2]), int(args[3]), rep, alt)
            elif cmd == "check":
                self.confirm_check("".join(args) if args else "ZXY")
            elif cmd == "span":
                if not args:
                    return self._err(commands.HELP["span"])
                self.span_axis = args[0].upper() if args[0].upper() in ("X", "Y") else self.span_axis
                self.confirm_span(args[0], int(args[1]) if len(args) > 1 else None)
            elif cmd == "apply":
                self._err(g.apply_span())
            elif cmd == "photo":
                self.photo()
            elif cmd == "spray":
                self.prompt = (f"Spray {g.s.spray_ms} ms here (X{g.axes['X'].pos:g} Y{g.axes['Y'].pos:g})?",
                               lambda: self._err(g.spray_here()))
            elif cmd == "pause":
                self._err(g.pause())
            elif cmd == "resume":
                self._err(g.resume())
            elif cmd == "abort":
                self._err(g.abort())
            elif cmd == "reset":
                self._err(g.reset())
            elif cmd == "status":
                g.status()
            elif cmd == "log":
                self._open_log()
            elif cmd == "net":
                self.net(args[0] if args else "")
            elif cmd == "set":
                self._set(args)
            elif cmd == "config":
                self.mode = "config"
            elif cmd == "help":
                if args and args[0] in commands.HELP:
                    g.say("·", commands.HELP[args[0]])
                else:
                    self.help_open = True
            elif cmd in ("quit", "q", "exit"):
                self.request_quit()
            else:
                self._err(f"unknown command: {cmd} · try help")
        except ValueError:
            self._err(commands.HELP.get(cmd, "bad argument"))

    def _set(self, args: list) -> None:
        g = self.g
        if len(args) < 2:
            return self._err(commands.HELP["set"])
        key, value = args[0], " ".join(args[1:])
        if key == "sim.silent":
            if not self.sim_link:
                return self._err("sim.silent only exists with --sim")
            (self.sim_link.drop if value in ("on", "1", "true") else self.sim_link.restore)()
            return g.say("·", f"sim: ESP32 {'silent' if self.sim_link.silent else 'talking'}")
        try:
            g.s.set(key, value)
        except KeyError:
            return self._err(f"no setting called {key}")
        except ValueError as e:
            return self._err(str(e))
        g.say("·", f"{key} = {getattr(g.s, key)}" + (" · press u in config > esp32 link to send it" if key.startswith(
            ("mm_per_count", "gain_", "motor_sign", "travel_", "soft_margin", "rail_v", "stall_v")) else ""))
        g.event(f"set {key} {getattr(g.s, key)}")

    # --- what to show -------------------------------------------------
    def state_label(self) -> tuple[str, str]:
        g = self.g
        if self.mode == "boot":
            return "BOOT", "inva"
        if g.estop:
            return "E-STOP", "invr"
        if g.link_lost:
            return "LINK LOST", "invr"
        if g.proc and not g.proc.done:
            return ("HOMING", "invc") if g.proc.kind == "homing" else ("CALIBRATING", "inva")
        j = g.job
        if j and j.state in ("paused", "stopped"):
            return "PAUSED", "inva"
        if j and j.kind == "scan":
            return "SCANNING", "invc"
        if j and j.kind == "weed":
            return "SPRAYING", "inva"
        if self.mode == "review":
            return "REVIEW", "inva"
        if self.mode == "jog":
            return "JOG", "invc"
        if self.mode == "calib":
            return "IDLE · calibration", "invg"
        if self.mode == "config":
            return "IDLE · config", "invg"
        return "IDLE", "invg"

    def now(self) -> float:
        return self.g.wall()

    # --- cached system probes (subprocesses are slow on a Pi Zero) -----
    def probe(self, key: str, fn, ttl: float = 5.0):
        now = self.g.clock()
        hit = self._probes.get(key)
        if hit and now - hit[0] < ttl:
            return hit[1]
        value = fn()
        self._probes[key] = (now, value)
        return value

    def service(self, name: str) -> bool | None:
        return self.probe(f"svc:{name}", lambda: self.g.system.service(name))

    def clients(self):
        s = self.g.s
        return self.probe("clients", lambda: self.g.system.ap_clients(s.dnsmasq_leases, s.ap_iface))

    def ap_clients_count(self) -> int | None:
        cl = self.clients()
        return len(cl) if cl is not None else None

    def net_info(self) -> dict:
        s = self.g.s
        return self.probe("netinfo", lambda: self.g.system.ap_info(s.hostapd_conf, s.ap_iface), ttl=30)
