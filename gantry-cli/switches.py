"""
The five limit switches: the Pi reads them and forwards them to the ESP32, which does the rest
(stopping an axis at an end switch, homing, the span calibration). gantry_fw 2.x.

Wiring: each switch between its GPIO and GND, with a pull-up.
  switch_nc true  (normally closed, COM + NC): released = LOW, pressed or broken wire = HIGH
  switch_nc false (normally open,   COM + NO): released = HIGH, pressed = LOW. A broken wire reads
                  "released", so press each switch by hand after any wiring change.

    read() -> {"x_min": bool, "x_max": bool, "y_min": bool, "y_max": bool, "z_top": bool}
              True = pressed, None if the switch cannot be read
    bits() -> "01000" (x_min x_max y_min y_max z_top), what the Forwarder sends as "SW 01000"

Backends, tried in this order when switch_backend is "auto":
  lgpio      python3-lgpio (Raspberry Pi OS Bookworm)
  rpigpio    python3-rpi.gpio
  sysfs      /sys/class/gpio (no pull-up control: set the pull-ups in config.txt, see the README)
  file:PATH  a JSON file with the same keys (used by the firmware simulator for end-to-end tests)
  sim        positions from SimLink (gantry.py --sim)
  none       not connected: the ESP is told SET SW 0, and homing sets the datum where the carriage is

Forwarder is the background thread that reads the switches every 3 ms and writes "SW <bits>" to
the ESP on every change and at least every 50 ms (the ESP stops all motion if they stop for 0.5 s).
With no switches it sends "KA" every 0.3 s instead.
"""

import json
import os
import threading
import time

NAMES = ("x_min", "x_max", "y_min", "y_max", "z_top")
HOME_SWITCH = {"X": "x_min", "Y": "y_min", "Z": "z_top"}
FAR_SWITCH = {"X": "x_max", "Y": "y_max"}


class Switches:
    """Common part: the watcher. Subclasses implement _read_raw()."""

    backend = "none"
    poll_s = 0.002

    def __init__(self):
        self._lock = threading.Lock()
        self._watch = None            # (name, want, callback)
        self._thread = None
        self._stop = threading.Event()
        self.last = {n: None for n in NAMES}
        self.error = ""

    @property
    def available(self) -> bool:
        return self.backend != "none"

    def _read_raw(self) -> dict:
        return {n: None for n in NAMES}

    invert = False                    # True for normally-open switches: a LOW pin = pressed

    def bits(self) -> str | None:
        st = self.read()
        if any(st.get(n) is None for n in NAMES):
            return None
        return "".join("1" if st[n] else "0" for n in NAMES)

    def read(self) -> dict:
        try:
            raw = self._read_raw()
            self.last = {n: (None if v is None else (not v if self.invert else v)) for n, v in raw.items()}
        except Exception as e:          # a GPIO read must never take the UI down
            self.error = str(e)
            self.last = {n: None for n in NAMES}
        return self.last

    # --- watcher ------------------------------------------------------
    def watch(self, name: str, want: bool, callback) -> None:
        """Call callback() once, as soon as switch `name` reads `want`."""
        with self._lock:
            self._watch = (name, want, callback)
        if self.threaded and (self._thread is None or not self._thread.is_alive()):
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def unwatch(self) -> None:
        with self._lock:
            self._watch = None

    def check(self) -> None:
        """Fire the watch if its condition holds (also called from the UI tick)."""
        with self._lock:
            w = self._watch
        if not w:
            return
        name, want, cb = w
        state = self.read().get(name)
        if state is want:
            with self._lock:
                if self._watch is not w:
                    return
                self._watch = None
            cb()

    threaded = False

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                active = self._watch is not None
            if not active:
                time.sleep(0.02)
                continue
            self.check()
            time.sleep(self.poll_s)

    def close(self) -> None:
        self._stop.set()


class NoSwitches(Switches):
    backend = "none"


class LgpioSwitches(Switches):
    backend = "lgpio"
    threaded = True

    def __init__(self, pins: dict):
        super().__init__()
        import lgpio
        self.lg = lgpio
        self.h = lgpio.gpiochip_open(0)
        self.pins = pins
        for n, p in pins.items():
            lgpio.gpio_claim_input(self.h, p, lgpio.SET_PULL_UP)

    def _read_raw(self) -> dict:
        return {n: bool(self.lg.gpio_read(self.h, self.pins[n])) if n in self.pins else None for n in NAMES}


class RpiGpioSwitches(Switches):
    backend = "rpigpio"
    threaded = True

    def __init__(self, pins: dict):
        super().__init__()
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.pins = pins
        for p in pins.values():
            GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _read_raw(self) -> dict:
        return {n: bool(self.GPIO.input(self.pins[n])) if n in self.pins else None for n in NAMES}


class SysfsSwitches(Switches):
    """Legacy sysfs. Kernel GPIO numbers are BCM + the base of the pinctrl chip (0, or 512 on newer kernels)."""

    backend = "sysfs"
    threaded = True

    def __init__(self, pins: dict, root: str = "/sys/class/gpio"):
        super().__init__()
        self.root = root
        base = self._base()
        self.paths = {}
        for n, p in pins.items():
            num = base + p
            d = os.path.join(root, f"gpio{num}")
            if not os.path.isdir(d):
                with open(os.path.join(root, "export"), "w") as f:
                    f.write(str(num))
                time.sleep(0.05)
            with open(os.path.join(d, "direction"), "w") as f:
                f.write("in")
            self.paths[n] = os.path.join(d, "value")

    def _base(self) -> int:
        for chip in sorted(os.listdir(self.root)):
            if not chip.startswith("gpiochip"):
                continue
            try:
                with open(os.path.join(self.root, chip, "label")) as f:
                    label = f.read().strip()
                if "bcm2835" in label or "pinctrl" in label:
                    with open(os.path.join(self.root, chip, "base")) as f:
                        return int(f.read().strip())
            except OSError:
                continue
        return 0

    def _read_raw(self) -> dict:
        out = {}
        for n in NAMES:
            p = self.paths.get(n)
            if not p:
                out[n] = None
                continue
            with open(p) as f:
                out[n] = f.read().strip() == "1"
        return out


class FileSwitches(Switches):
    """Switch states from a JSON file (the firmware simulator writes it)."""

    backend = "file"
    threaded = True

    def __init__(self, path: str):
        super().__init__()
        self.path = path

    def _read_raw(self) -> dict:
        try:
            with open(self.path) as f:
                d = json.load(f)
        except (OSError, ValueError):
            return dict(self.last)     # mid-write: keep the previous reading
        return {n: (bool(d[n]) if n in d else None) for n in NAMES}


class SimSwitches(Switches):
    """Switch states from SimLink's physical positions (--sim and the tests)."""

    backend = "sim"
    threaded = False                    # the tests run on a fake clock: checked from the UI tick only

    def __init__(self, link):
        super().__init__()
        self.link = link
        self.forced: dict = {}          # tests: {"y_max": True} simulates a broken wire

    def _read_raw(self) -> dict:
        st = self.link.switch_states()
        st.update(self.forced)
        return st


def open_switches(settings, link=None, sim: bool = False) -> Switches:
    if not getattr(settings, "switches_connected", True):
        return NoSwitches()
    if sim:
        return SimSwitches(link)
    want = settings.switch_backend
    pins = {n: int(p) for n, p in settings.switch_pins.items() if n in NAMES}
    if want.startswith("file:"):
        return FileSwitches(want[5:])
    if want == "none":
        return NoSwitches()
    order = [want] if want != "auto" else ["lgpio", "rpigpio", "sysfs"]
    errors = []
    for name in order:
        try:
            if name == "lgpio":
                sw = LgpioSwitches(pins)
            elif name == "rpigpio":
                sw = RpiGpioSwitches(pins)
            elif name == "sysfs":
                sw = SysfsSwitches(pins)
            else:
                continue
            sw.invert = not getattr(settings, "switch_nc", True)
            return sw
        except Exception as e:
            errors.append(f"{name}: {e}")
    s = NoSwitches()
    s.error = "; ".join(errors) or "no backend"
    return s


class Forwarder:
    """Limit switches -> ESP32, in a thread, so a switch reaches the ESP within a few ms of closing."""

    def __init__(self, link, switches, connected: bool, poll_s: float = 0.003, every_s: float = 0.05, ka_s: float = 0.3):
        self.link, self.switches, self.connected = link, switches, connected
        self.poll_s, self.every_s, self.ka_s = poll_s, every_s, ka_s
        self.bits = None
        self.sent = 0
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self) -> "Forwarder":
        self._t.start()
        return self

    def _run(self) -> None:
        last_sent, last_t = None, 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                if self.connected and self.switches.available:
                    bits = self.switches.bits()
                    self.bits = bits
                    if bits is not None and (bits != last_sent or now - last_t >= self.every_s):
                        self.link.write("SW " + bits)
                        last_sent, last_t = bits, now
                        self.sent += 1
                elif now - last_t >= self.ka_s:
                    self.link.write("KA")
                    last_t = now
            except Exception:
                pass                # a write error shows up as the link going quiet
            time.sleep(self.poll_s)

    def stop(self) -> None:
        self._stop.set()
