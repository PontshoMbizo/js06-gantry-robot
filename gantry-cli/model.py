"""
Data the CLI works on: settings, the bed layout, weeds, axes and the log.

Coordinates are machine coordinates in mm, measured from the homing datum
(the trip points of the X-min, Y-min and Z-top switches, CTRL-06):
  X 0 to 620 (MECH-04: X ±0.31 m), Y 0 to 1060 (Y 0 to 1.06 m), Z 0 to 250
  measured DOWN from the top switch (Z 0 to 0.25 m nozzle descent).
Soft limits sit soft_margin (5 mm) inside each end of travel (CTRL-08).

The bed map is the bed in plan view. With map_view "y-right" (default) the
long Y axis runs left to right and X runs up the screen, so the 1.06 m travel
uses the width of the terminal. Everything here is plain data with no curses,
so it can be tested headless.
"""

import json
import math
import os
from dataclasses import dataclass, field, fields

HERE = os.path.dirname(os.path.abspath(__file__))

MAP_COLS = 50
MAP_ROWS = 12


@dataclass
class Settings:
    """Runtime settings. Override any of them in gantry.json or with :set."""

    port: str = "/dev/serial0"
    baud: int = 115200
    bed_file: str = os.path.join(HERE, "beds", "js06-bed.json")
    log_dir: str = os.path.join(HERE, "logs")
    sysid_dir: str = os.path.join(HERE, "logs", "sysid")
    calib_dir: str = os.path.join(HERE, "logs", "calibration")
    map_view: str = "y-right"           # y-right | x-right

    # --- axes (MECH-04, CTRL-05, CTRL-08) ---
    travel_x: tuple = (0.0, 620.0)      # end of travel = switch trip points [mm]
    travel_y: tuple = (0.0, 1060.0)
    travel_z: tuple = (0.0, 250.0)      # Z down from the top switch
    soft_margin: float = 5.0
    mm_per_count_x: float = 1.0661      # 2*pi*5.09 mm / 30 slots
    mm_per_count_y: float = 1.0661
    mm_per_count_z: float = 0.2667      # 8 mm lead / 30 slots
    motor_sign_x: int = 1               # -1 flips the motor direction
    motor_sign_y: int = 1
    motor_sign_z: int = 1
    gain_x: tuple = (0.30, 0.05, 7.0, 0.0)   # Kp V/mm, Kd V/(mm/s), Vlim V, V0 dead-band offset V
    gain_y: tuple = (0.30, 0.05, 7.0, 0.0)
    gain_z: tuple = (1.20, 0.20, 7.0, 0.0)   # TBC: Z not identified yet
    rail_v: float = 12.0                # motor supply at the L298 boards
    stall_v: float = 2.5                # not used since firmware 2.2 (no stall detection); kept so old gantry.json files load
    check_v: float = 5.0                # sensor check (calibration, v): drive voltage
    check_ticks: int = 10               # pulses wanted from each axis
    check_timeout: float = 6.0          # s per axis
    jog_steps: tuple = (1, 5, 10, 50, 100)
    jog_step: int = 2                   # index into jog_steps (10 mm); len(jog_steps) = your own step (jog_custom)
    jog_custom: float = 0.0             # mm, typed on the jog screen (key 6)
    park_x: float = 5.0
    park_y: float = 5.0

    # --- homing (CTRL-06), coordinated by the Pi ---
    home_order: str = "ZXY"             # Z first, so the nozzle is up before X and Y move (the ESP32 always homes Z, X, Y)
    home_fast_v: float = 4.0            # V-equivalent, approach (sent to the ESP32: SET HOME)
    home_slow_v: float = 2.5            # V-equivalent, sent to the ESP32 but unused by HOME (single approach at home_fast_v)
    home_offset: float = 5.0            # park position after a SPAN (homing leaves the axis on its switch)
    switches_connected: bool = True     # False: the ESP32 is told SET SW 0; h sets the datum where the carriage sits
    switch_nc: bool = True              # False for normally-open switches (pressed = LOW; a broken wire isn't detected)
    switch_backend: str = "auto"        # auto | lgpio | rpigpio | sysfs | file:<path> | none
    switch_pins: dict = field(default_factory=lambda: {"x_min": 17, "x_max": 27, "y_min": 22, "y_max": 23, "z_top": 24})

    # --- spray (ACT-02, ACT-06, ACT-09) ---
    safe_z: float = 5.0                 # travel height (just below the top switch)
    camera_z: float = 5.0               # height for captures
    spray_z: float = 120.0              # TBC (ATP-08): nozzle height giving a 50 mm footprint
    spray_ms: int = 250                 # TBC (ATP-08): dose per spray
    tolerance: float = 20.0             # MECH-05 positioning tolerance
    weed_radius: float = 5.0
    guard: float = 45.0                 # ACT-09 clearance: 2 x tolerance + weed radius

    # --- detection ---
    camera: str = "640×480"             # the simulator's frame size (--sim)
    camera_source: str = "auto"         # auto | rpicam | legacy | usb | none: which camera to look for
    photo_width: int = 1640             # photos (:photo, c on the jog screen) · 0 and 0 = full size
    photo_height: int = 1232
    photo_settle_ms: int = 1000
    photo_dir: str = os.path.join(HERE, "logs", "photos")
    model: str = "detector.tflite"
    thresh: float = 0.60
    scan_cols: int = 2                  # frames across X
    scan_rows: int = 4                  # frames along Y (2 x 4 = 8 frames, DEC-12 note)
    settle: float = 0.3
    merge_mm: float = 15.0              # detections closer than this (overlapping frames) are one weed

    # --- link (EMBED-08) ---
    reply_timeout: float = 1.0
    missed_for_lost: int = 3
    heartbeat: float = 1.0
    ping_every: float = 2.0
    ping_tries: int = 10
    keepalive: float = 0.5              # KA while a long command runs; the ESP stops after 1 s of silence

    # --- calibration ---
    span_x_mm: float = 0.0              # measured distance between the X switch trip points (steel rule); 0 = not measured
    span_y_mm: float = 0.0
    span_passes: int = 3
    step_axis: str = "y"
    step_volts: float = 3.0
    step_dir: int = 1
    step_pulses: int = 100
    step_repeats: int = 1
    step_alternate: bool = True
    step_timeout: float = 15.0

    # --- network ---
    ap_ssid: str = "gantry-net"
    ap_service: str = "gantry-ap"
    ap_iface: str = "wlan0"
    hotspot: str = "gantry-net"
    hostapd_conf: str = "/etc/hostapd/hostapd.conf"
    dnsmasq_leases: str = "/var/lib/misc/dnsmasq.leases"
    net_online_cmd: list = field(default_factory=list)
    net_ap_cmd: list = field(default_factory=list)
    net_restart_cmd: list = field(default_factory=list)
    open_items: list = field(default_factory=list)

    config_path: str = ""               # set by load(): where calibration results are saved
    ignored: list = field(default_factory=list)   # keys in gantry.json this version doesn't use

    def jog_step_mm(self) -> float:
        if self.jog_step >= len(self.jog_steps) and self.jog_custom > 0:
            return float(self.jog_custom)
        return float(self.jog_steps[min(self.jog_step, len(self.jog_steps) - 1)])

    def travel(self, axis: str) -> tuple:
        return {"X": self.travel_x, "Y": self.travel_y, "Z": self.travel_z}[axis]

    def limits(self, axis: str) -> tuple:
        """Soft limits: soft_margin inside each end of travel (CTRL-08)."""
        lo, hi = self.travel(axis)
        return lo + self.soft_margin, hi - self.soft_margin

    def mmpc(self, axis: str) -> float:
        return getattr(self, f"mm_per_count_{axis.lower()}")

    def gain(self, axis: str) -> tuple:
        return getattr(self, f"gain_{axis.lower()}")

    def sign(self, axis: str) -> int:
        return getattr(self, f"motor_sign_{axis.lower()}")

    def span_mm(self, axis: str) -> float:
        return getattr(self, f"span_{axis.lower()}_mm", 0.0)

    @classmethod
    def load(cls, path: str | None) -> "Settings":
        s = cls()
        s.config_path = path or ""
        if path and os.path.exists(path):
            with open(path, encoding="utf-8") as f:
                for k, v in json.load(f).items():
                    try:
                        s.set(k, v)
                    except (KeyError, ValueError, TypeError):
                        s.ignored.append(k)        # an old or unknown key: shown in the config view, not fatal
        return s

    def set(self, key: str, value) -> None:
        """Set one field, converting from text when needed. Raises KeyError/ValueError."""
        names = {f.name: f for f in fields(self)}
        if key not in names or key in ("config_path", "ignored"):
            raise KeyError(key)
        current = getattr(self, key)
        if isinstance(value, str) and not isinstance(current, str):
            if isinstance(current, bool):
                value = value.lower() in ("1", "on", "true", "yes")
            elif isinstance(current, int):
                value = int(value)
            elif isinstance(current, float):
                value = float(value)
            elif isinstance(current, tuple):
                value = tuple(float(v) for v in value.replace(",", " ").split())
                if len(value) != len(current):
                    raise ValueError(f"{key} needs {len(current)} numbers")
            else:
                raise ValueError(f"{key} can only be set in gantry.json")
        if isinstance(current, tuple) and isinstance(value, list):
            value = tuple(value)
        if isinstance(current, float) and isinstance(value, int) and not isinstance(value, bool):
            value = float(value)
        if key in ("bed_file", "log_dir", "sysid_dir", "calib_dir", "photo_dir") and not os.path.isabs(value):
            value = os.path.join(HERE, value)
        setattr(self, key, value)

    def save(self, updates: dict) -> str | None:
        """Write these keys into gantry.json (merged with what is there). Returns an error or None."""
        path = self.config_path or os.path.join(HERE, "gantry.json")
        try:
            data = {}
            if os.path.exists(path):
                with open(path, encoding="utf-8") as f:
                    data = json.load(f)
            for k, v in updates.items():
                data[k] = list(v) if isinstance(v, tuple) else v
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
                f.write("\n")
        except (OSError, ValueError) as e:
            return str(e)
        return None


@dataclass
class Crop:
    kind: str
    x: float
    y: float


@dataclass
class Bed:
    name: str
    width: float            # along X
    height: float           # along Y
    kinds: dict
    crops: list
    crop_rows: list
    sim_weeds: list
    view: str = "y-right"

    @classmethod
    def load(cls, path: str, view: str = "y-right") -> "Bed":
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return cls(
            name=d["name"],
            width=float(d["size_mm"][0]),
            height=float(d["size_mm"][1]),
            kinds=d["kinds"],
            crops=[Crop(c["kind"], float(c["x"]), float(c["y"])) for c in d["crops"]],
            crop_rows=[float(y) for y in d.get("crop_rows_mm", [])],
            sim_weeds=d.get("sim_weeds", []),
            view=view,
        )

    def kind(self, name: str) -> dict:
        return self.kinds.get(name, {"glyph": name[:1], "role": "p", "radius_mm": 0, "short": name[:4]})

    def nearest_crop(self, x: float, y: float) -> tuple[Crop | None, float]:
        """Closest crop and the gap from (x, y) to its canopy edge (centre distance minus radius)."""
        best, gap = None, math.inf
        for c in self.crops:
            d = math.hypot(c.x - x, c.y - y) - self.kind(c.kind).get("radius_mm", 0)
            if d < gap:
                best, gap = c, d
        return best, max(0.0, gap)

    # --- map geometry -------------------------------------------------
    @property
    def h_axis(self) -> str:
        return "Y" if self.view == "y-right" else "X"

    @property
    def v_axis(self) -> str:
        return "X" if self.view == "y-right" else "Y"

    @property
    def h_size(self) -> float:
        return self.height if self.view == "y-right" else self.width

    @property
    def v_size(self) -> float:
        return self.width if self.view == "y-right" else self.height

    def hv(self, x: float, y: float) -> tuple[float, float]:
        return (y, x) if self.view == "y-right" else (x, y)

    def xy(self, h: float, v: float) -> tuple[float, float]:
        return (v, h) if self.view == "y-right" else (h, v)

    def col_of(self, h: float, cols: int = MAP_COLS) -> int:
        return max(0, min(cols - 1, round(h / self.h_size * (cols - 1))))

    def row_of(self, v: float, rows: int = MAP_ROWS) -> int:
        return max(0, min(rows - 1, round((self.v_size - v) / self.v_size * (rows - 1))))

    def cell(self, x: float, y: float, cols: int = MAP_COLS, rows: int = MAP_ROWS) -> tuple[int, int]:
        """(row, col) of a bed point on a cols x rows map."""
        h, v = self.hv(x, y)
        return self.row_of(v, rows), self.col_of(h, cols)

    def point(self, row: int, col: int, cols: int = MAP_COLS, rows: int = MAP_ROWS) -> tuple[float, float]:
        """Bed (x, y) at the centre of a map cell."""
        h = col * self.h_size / (cols - 1)
        v = self.v_size - row * self.v_size / (rows - 1)
        return self.xy(h, v)


@dataclass
class Weed:
    id: int
    x: float
    y: float
    conf: float
    frame: int
    near_kind: str = ""
    near_mm: float = 0.0
    ticked: bool = True
    guarded: bool = False       # ACT-09: centre within the clearance of a crop canopy: never sprayed
    status: str = "found"       # found | queued | spraying | sprayed | skipped | untreated | failed
    spray_s: float = 0.0

    def glyph(self) -> str:
        return str(self.id) if self.id < 10 else chr(ord("a") + self.id - 10)


@dataclass
class Axis:
    name: str
    pos: float = 0.0
    homed: bool = False


@dataclass
class LogLine:
    t: float                # wall clock
    mark: str               # → sent, ← reply, ! error, × timeout, · other, # event
    text: str


@dataclass
class Frame:
    n: int
    x: float
    y: float


def scan_frames(s: Settings, bed: Bed, rows: tuple[int, int] | None = None) -> list[Frame]:
    """Frame stops. Outer loop along Y (the bridge axis), snaking in X inside each
    Y step, so the camera never makes a long return move and the y-right map fills
    left to right. --rows a-b keeps only the frames nearest crop rows a..b (Y values
    listed in the bed file)."""
    fw, fh = bed.width / s.scan_cols, bed.height / s.scan_rows
    ys = [fh * (j + 0.5) for j in range(s.scan_rows)]
    if rows and bed.crop_rows:
        a, b = rows
        wanted = bed.crop_rows[a - 1:b]
        keep = {min(range(len(ys)), key=lambda j: abs(ys[j] - y)) for y in wanted}
        ys = [ys[j] for j in sorted(keep)]
    xs = [fw * (i + 0.5) for i in range(s.scan_cols)]
    out, n = [], 1
    for j, y in enumerate(ys):
        col_xs = xs if j % 2 == 0 else list(reversed(xs))
        for x in col_xs:
            out.append(Frame(n, round(x, 1), round(y, 1)))
            n += 1
    return out


def footprint(s: Settings, bed: Bed) -> tuple[float, float]:
    return bed.width / s.scan_cols, bed.height / s.scan_rows


def in_frames(x: float, y: float, frames: list, fw: float, fh: float) -> bool:
    return any(abs(x - f.x) <= fw / 2 + 1e-6 and abs(y - f.y) <= fh / 2 + 1e-6 for f in frames)


def nearest_neighbour(start: tuple[float, float], weeds: list[Weed]) -> list[Weed]:
    """Spray order: always go to the closest remaining target (SW-03)."""
    left, out, (x, y) = list(weeds), [], start
    while left:
        w = min(left, key=lambda w: math.hypot(w.x - x, w.y - y))
        left.remove(w)
        out.append(w)
        x, y = w.x, w.y
    return out


def vtag(v: float) -> str:
    """1.5 -> 1v5, 3 -> 3v0, 2.25 -> 2v25 (step-test file names)."""
    t = f"{v:.2f}".rstrip("0")
    return (t + "0" if t.endswith(".") else t).replace(".", "v")
