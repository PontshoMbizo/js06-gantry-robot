"""
UART link to the ESP32-C3 (firmware/gantry_fw): one text line per command, one or more lines back.
The full protocol is at the top of firmware/gantry_fw/gantry_fw.ino and in the README.

Replies: PONG · OK … (done) · PROG … (still running) · ERR CODE … · D k,t (step-test raw edges) ·
BOOT gantry-fw … (the ESP restarted) · anything else is boot text or noise and is logged dim.

SerialLink talks to the real port; SimLink is a fake ESP32 with physical positions behind the
switches, for development off the Pi (--sim) and for the tests.
"""

import math
import os
import queue
import re
import threading
import time


class SerialLink:
    def __init__(self, port: str, baud: int):
        import serial  # pyserial; only needed on the Pi

        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.lines: queue.Queue[tuple[float, str]] = queue.Queue()
        self.garbled = 0
        self._wlock = threading.Lock()      # the switch forwarder thread also writes (SW lines)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    @property
    def device(self) -> str:
        real = os.path.realpath(self.port)
        return self.port if real == self.port else f"{self.port} → {os.path.basename(real)}"

    def _read(self) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                # one byte (waits up to the timeout), then whatever else has arrived
                chunk = self.ser.read(1)
                if chunk and self.ser.in_waiting:
                    chunk += self.ser.read(self.ser.in_waiting)
            except Exception:
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                now = time.monotonic()
                try:
                    self.lines.put((now, raw.decode("ascii").strip()))
                except UnicodeDecodeError:
                    self.garbled += 1
                    self.lines.put((now, raw.decode("ascii", "replace").strip()))

    def write(self, line: str) -> None:
        with self._wlock:
            self.ser.write((line + "\n").encode("ascii"))

    def poll(self) -> list[tuple[float, str]]:
        """Lines received since the last poll, each with its arrival time."""
        out = []
        while True:
            try:
                out.append(self.lines.get_nowait())
            except queue.Empty:
                return out

    def close(self) -> None:
        self._stop.set()
        self.ser.close()


AXES = "XYZ"


class SimLink:
    """A fake ESP32 running gantry_fw 2.x, with plausible timings and physical positions.

    phys[a] is the true position in mm from the home switch trip point (Z: down from the top
    switch); the switches read from it. The firmware's position is phys - zero, so before
    homing the reported position is wherever the ESP started counting. `speed` > 1 runs faster.
    Like the firmware, it homes and runs the span on its own and stops an axis at an end switch.
    """

    MOVE_SPEED = {"X": 30.0, "Y": 30.0, "Z": 8.0}        # mm/s for closed-loop moves
    DRIVE_K = {"X": 7.8, "Y": 7.8, "Z": 2.0}             # mm/s per V above the dead-band
    DEADBAND = 1.0
    HARD = {"X": (-25.0, 645.0), "Y": (-25.0, 1085.0), "Z": (-15.0, 265.0)}
    SPAN = {"X": 620.0, "Y": 1060.0}
    FW = "2.4"

    def __init__(self, clock=time.monotonic, speed: float = 1.0, start=(250.0, 400.0, 60.0)):
        self.clock = clock
        self.speed = speed
        self.port = "/dev/serial0"
        self.baud = 115200
        self.device = "/dev/serial0 → sim"
        self.garbled = 0
        self.phys = dict(zip(AXES, start))
        self.zero = dict(self.phys)                       # firmware counts from where it booted
        self.homed = dict.fromkeys(AXES, False)
        self.mmpc = {"X": 1.0661, "Y": 1.0661, "Z": 0.2667}
        self.soft = {"X": (5.0, 615.0), "Y": (5.0, 1055.0), "Z": (5.0, 245.0)}
        self.rail = 12.0
        self.sw_on = True
        self.home_v = (4.0, 2.5, 5.0)                     # fast, slow, park offset (SET HOME)
        self.estop = False
        self.silent = False
        self.pump = False
        self.out: list[tuple[float, str]] = []
        self.motion = None            # dict describing the running command
        self.last_rx = clock()
        self.watchdog = 1.0
        self.jam: set = set()         # tests: axes that are blocked (no stall check: moves time out)
        self.noise_edges = 0          # tests: extra raw edges per real one in step-test data
        self.log: list[str] = []      # every line the Pi sent except SW (tests)
        self._t = clock()

    # --- helpers -------------------------------------------------------
    def _at(self, delay: float, line: str) -> None:
        self.out.append((self.clock() + delay / self.speed, line))

    def pos(self, a: str) -> float:
        return self.phys[a] - self.zero[a]

    def counts(self, a: str) -> int:
        return int(math.floor(self.phys[a] / self.mmpc[a]))

    def _postext(self) -> str:
        return " ".join(f"{a}{self.pos(a):.1f}" for a in AXES)

    def _homedtext(self) -> str:
        return "".join(a for a in AXES if self.homed[a]) or "-"

    def switch_states(self) -> dict:
        p = self.phys
        return {"x_min": p["X"] <= 0, "x_max": p["X"] >= self.SPAN["X"], "y_min": p["Y"] <= 0,
                "y_max": p["Y"] >= self.SPAN["Y"], "z_top": p["Z"] <= 0}

    def _swbits(self) -> str:
        st = self.switch_states()
        return "".join("1" if st[n] else "0" for n in ("x_min", "x_max", "y_min", "y_max", "z_top"))

    def _end_switch(self, a: str, d: float):
        """The switch an axis reaches moving in direction d, if switches are on and it is pressed."""
        if not self.sw_on:
            return None
        if d < 0 and self.phys[a] <= 0:
            return {"X": "x_min", "Y": "y_min", "Z": "z_top"}[a]
        if d > 0 and a in self.SPAN and self.phys[a] >= self.SPAN[a]:
            return {"X": "x_max", "Y": "y_max"}[a]
        return None

    def _clamp(self, a: str, v: float) -> tuple[float, bool]:
        lo, hi = self.HARD[a]
        if v < lo:
            return lo, True
        if v > hi:
            return hi, True
        return v, False

    def _finish(self, line: str) -> None:
        self.motion = None
        self.pump = False
        self._at(0.0, line)

    def _drive(self, a: str, d: int, v: float, dt: float) -> None:
        sp = self.DRIVE_K[a] * max(0.0, v - self.DEADBAND)
        self.phys[a], _ = self._clamp(a, self.phys[a] + d * sp * dt)

    # --- physics -------------------------------------------------------
    def _advance(self) -> None:
        now = self.clock()
        dt = (now - self._t) * self.speed
        self._t = now
        m = self.motion
        if not m or dt <= 0:
            return
        if m.get("pi") and now - self.last_rx > self.watchdog:      # the Pi's keep-alives arrive in wall time
            if m["kind"] == "STEP":
                m["dur"], m["end"] = m["t"], "stop"
            else:
                self._finish("ERR WATCHDOG")
                return
        m["t"] += dt
        if m["t"] - m["prog"] >= 0.5:
            m["prog"] = m["t"]
            k = m["kind"]
            self._at(0.0, f"PROG STEP n={len([r for r in m['rows'] if r <= m['t'] * 1e6])} {self._postext()}" if k == "STEP"
                     else "PROG SPRAY" if k == "SPRAY" else f"PROG {self._postext()}")
        k = m["kind"]
        if k == "MOVE":
            if m["t"] > m["timeout"]:
                self._finish(f"ERR TIMEOUT move took longer than {m['timeout']:.0f} s {self._postext()}")
                return
            done = True
            for a, tgt in m["targets"].items():
                if a in self.jam:
                    done = False
                    continue
                cur = self.pos(a)
                step = self.MOVE_SPEED[a] * dt
                if abs(tgt - cur) <= step:
                    self.phys[a] = tgt + self.zero[a]
                else:
                    self.phys[a] += math.copysign(step, tgt - cur)
                    done = False
                self.phys[a], _ = self._clamp(a, self.phys[a])
                sw = self._end_switch(a, tgt - cur)
                if sw:
                    self._finish(f"ERR LIMIT {sw}")
                    return
            if done:
                self._finish(f"OK {self._postext()}")
        elif k == "HOME":
            self._home_step(m, dt)
        elif k == "SPAN":
            a = m["axis"]
            if a in self.jam and m["t"] > 180:
                self._finish(f"ERR SPAN {a} no switch within 180 s")
                return
            d = -1 if len(m["c"]) % 2 == 0 else 1
            self._drive(a, d, self.home_v[0], dt)
            if self._end_switch(a, d):
                m["c"].append(self.counts(a))
                if len(m["c"]) >= m["legs"]:
                    self._finish(f"OK SPAN {a} C=" + ",".join(str(c) for c in m["c"]))
        elif k == "STEP":
            if m["t"] >= m["dur"]:
                a, rows = m["axis"], [r for r in m["rows"] if r <= m["dur"] * 1e6]
                raw = []
                for t in rows:
                    raw.append(t)
                    raw += [t + 37 * (j + 1) for j in range(self.noise_edges)]
                for i, t in enumerate(raw, start=1):
                    self._at(0.0, f"D {i},{t}")
                travel = len(rows) * self.mmpc[a]
                self.phys[a], _ = self._clamp(a, self.phys[a] + m["dir"] * travel)
                self._finish(f"OK STEP {a} edges={len(raw)} pulses={len(rows)} end={m['end']} t_end={int(m['dur'] * 1e6)} "
                             f"lost=0 v={m['v']:.2f} dir={'+' if m['dir'] > 0 else '-'}")
        elif k == "SPRAY":
            if m["t"] >= m["ms"] / 1000:
                self._finish(f"OK SPRAY {m['ms']}")

    def _home_step(self, m: dict, dt: float) -> None:
        a = m["axes"][0]
        fast = self.home_v[0]
        if a in self.jam:
            m["jam_t"] = m.get("jam_t", 0.0) + dt
            if m["jam_t"] > 180:
                self._finish(f"ERR HOME {a} no switch within 180 s")
            return
        if self.phys[a] > 0:
            self._drive(a, -1, fast, dt)
        if self.phys[a] <= 0:                       # switch pressed: this is home, the axis stays on it
            self.phys[a] = 0.0
            self.zero[a] = 0.0
            self.homed[a] = True
            m["axes"].pop(0)
            if not m["axes"]:
                self._finish(f"OK HOME {self._postext()} HOMED={self._homedtext()}")

    def _start(self, **m) -> None:
        m.setdefault("t", 0.0)
        m.setdefault("prog", 0.0)
        m["pi"] = True
        self._t = self.clock()
        self.motion = m

    def _step_rows(self, a: str, v: float, d: int, n: int, timeout: float) -> tuple[list, str, float]:
        """Edge times (us from the step) of a first-order speed response (tau 0.1 s, 1 V dead-band)."""
        k = self.DRIVE_K[a] / self.mmpc[a]            # slots/s per V
        wp = 0.0 if a in self.jam else k * max(0.0, v - self.DEADBAND)
        tau = 0.1
        rows, t, slots = [], 0.0, 0.3
        lo, hi = self.soft[a]
        while len(rows) < n and t < timeout:
            t += 0.0005
            x = t - tau * (1 - math.exp(-t / tau))
            if wp * x >= slots:
                rows.append(int(t * 1e6))
                slots += 1
                if self.homed[a] and not lo <= self.pos(a) + d * len(rows) * self.mmpc[a] <= hi:
                    return rows, "limit", t
        if not rows and t >= 2.0:
            return rows, "stall", 2.0
        end = "ticks" if len(rows) >= n else "timeout"
        return rows, end, (t if end == "ticks" else timeout)

    # --- the Pi writes ---------------------------------------------------
    def write(self, line: str) -> None:
        w = line.split()
        cmd = w[0] if w else ""
        if cmd == "SW":                                  # switch states from the Pi's forwarder thread
            self.last_rx = self.clock()
            return
        self._advance()
        self.last_rx = self.clock()
        self.log.append(line)
        if self.silent:
            return
        if cmd == "KA":
            return
        if cmd == "PING":
            return self._at(0.012, "PONG")
        if cmd == "POS?":
            return self._at(0.01, f"OK {self._postext()} HOMED={self._homedtext()} SW={self._swbits() if self.sw_on else 'off'}")
        if cmd == "ESTOP":
            self.motion, self.pump, self.estop = None, False, True
            self.out = [o for o in self.out if not o[1].startswith("D ") and not o[1].startswith("PROG")]
            return self._at(0.02, "OK ESTOP")
        if cmd == "RESET":
            self.estop = False
            return self._at(0.02, "OK RESET")
        if cmd == "ABORT":
            self.motion, self.pump = None, False
            self.out = [o for o in self.out if not o[1].startswith("PROG")]
            return self._at(0.02, "OK ABORT")
        if cmd in ("PAUSE", "RESUME"):
            return self._at(0.02, f"OK {cmd}")
        if cmd == "SET":
            if self.motion:
                return self._at(0.01, "ERR BUSY")
            if len(w) >= 4 and w[1] == "MMPC":
                self.mmpc[w[2]] = float(w[3])
            elif len(w) >= 5 and w[1] == "SOFT":
                self.soft[w[2]] = (float(w[3]), float(w[4]))
            elif len(w) >= 3 and w[1] == "RAIL":
                self.rail = float(w[2])
            elif len(w) >= 3 and w[1] == "SW":
                self.sw_on = w[2] != "0"
            elif len(w) >= 5 and w[1] == "HOME":
                self.home_v = (float(w[2]), float(w[3]), float(w[4]))
            return self._at(0.01, f"OK SET {w[1] if len(w) > 1 else ''}")
        if cmd == "CFG?":
            return self._at(0.01, f"OK CFG FW={self.FW} sim SW={1 if self.sw_on else 0}")
        if cmd == "STOP":
            m = self.motion
            if m and m["kind"] == "STEP":
                m["dur"], m["end"] = m["t"], "stop"
                self._advance()
            elif m:
                self.motion = None
                self._at(0.005, "ERR STOPPED")
            return self._at(0.01, f"OK STOP {self._postext()}")
        if cmd in ("MOVE", "JOG", "HOME", "SPAN", "STEP", "SPRAY"):
            if self.estop:
                return self._at(0.02, "ERR ESTOP latched")
            if self.motion:
                return self._at(0.02, "ERR BUSY")
        if cmd in ("MOVE", "JOG"):
            targets = {}
            for t in w[1:]:
                m = re.fullmatch(r"([XYZ])([+-]?\d+(?:\.\d+)?)", t)
                if not m:
                    return self._at(0.02, f"ERR BAD_ARG {t}")
                a, v = m.group(1), float(m.group(2))
                targets[a] = self.pos(a) + v if cmd == "JOG" else v
                if cmd == "JOG" and not self.homed[a] and abs(v) > 50:
                    return self._at(0.02, f"ERR NOT_HOMED {a}")
            for a, v in targets.items():
                if self.homed[a]:
                    lo, hi = self.soft[a]
                    here = self.pos(a)                       # moving back toward the range is allowed (homed on the switch = 0)
                    if (v < lo - 1e-6 and v < here) or (v > hi + 1e-6 and v > here):
                        return self._at(0.02, f"ERR SOFT_LIMIT {a}")
                elif cmd == "MOVE":
                    return self._at(0.02, f"ERR NOT_HOMED {a}")
                sw = self._end_switch(a, v - self.pos(a))
                if sw and abs(v - self.pos(a)) > 0.5 * self.mmpc[a]:
                    return self._at(0.02, f"ERR LIMIT {sw}")
            far = max((abs(v - self.pos(a)) / (1.0 if a == "Z" else 3.0) for a, v in targets.items()), default=0.0)
            return self._start(kind="MOVE", name=cmd, targets=targets, timeout=15.0 + far)
        if cmd == "HOME":
            axes = [a for a in "ZXY" if a in "".join(w[1:]).upper()] or list("ZXY")
            if not self.sw_on:
                for a in axes:
                    self.zero[a] = self.phys[a]
                    self.homed[a] = True
                return self._at(0.02, f"OK HOME {self._postext()} HOMED={self._homedtext()}")
            for a in axes:
                self.homed[a] = False
            return self._start(kind="HOME", axes=axes, phase="approach")
        if cmd == "SPAN":
            a = w[1].upper() if len(w) > 1 else ""
            passes = int(w[2]) if len(w) > 2 else 2
            if a not in self.SPAN or not self.sw_on:
                return self._at(0.02, "ERR BAD_ARG")
            return self._start(kind="SPAN", axis=a, legs=2 * passes + 1, c=[])
        if cmd == "STEP":
            a, v, n = w[1], float(w[2]), int(w[4])
            d = 1 if w[3] in ("+", "+1") else -1
            timeout = float(w[5]) if len(w) > 5 else 30.0
            rows, end, dur = self._step_rows(a, v, d, n, timeout)
            return self._start(kind="STEP", axis=a, v=v, dir=d, rows=rows, end=end, dur=dur + 0.3)
        if cmd == "SPRAY":
            ms = int(w[1])
            if not 1 <= ms <= 3000:
                return self._at(0.01, "ERR BAD_ARG")
            self.pump = True
            return self._start(kind="SPRAY", ms=ms)
        # unknown lines are ignored, like the firmware

    def poll(self) -> list[tuple[float, str]]:
        self._advance()
        now = self.clock()
        due = sorted((x for x in self.out if x[0] <= now), key=lambda x: x[0])
        self.out = [x for x in self.out if x[0] > now]
        return due if not self.silent else []

    def drop(self) -> None:
        """Go quiet, as if the ESP32 browned out."""
        self.silent = True
        self.out.clear()
        self.motion = None

    def restore(self) -> None:
        """Come back after a reset: boot text, counts and homing lost."""
        self.silent = False
        self.zero = dict(self.phys)
        self.homed = dict.fromkeys(AXES, False)
        self.estop = False
        self._at(0.1, "ESP-ROM:esp32c3-api1-20210207")
        self._at(0.2, f"BOOT gantry-fw {self.FW}")

    def close(self) -> None:
        pass
