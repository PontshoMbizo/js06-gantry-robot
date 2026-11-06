# Gantry CLI

The terminal UI that runs on the Raspberry Pi Zero W of the gantry robot (JS-06) and drives the
ESP32-C3-Zero over `/dev/serial0`. It follows requirements revision 3.1: spot spraying (ACT),
Pi-read limit switches (EMBED-01), homing (CTRL-06) and closed-loop axes (CTRL) on the ESP32,
acknowledgements, timeouts and a link watchdog (EMBED-08), a spray log (ACT-06), stage logging
(SW-04) and a command line over SSH (SW-05). It also has a calibration screen for step tests
(system ID) and the switch-to-switch mm/count calibration, and takes photos with the Pi camera.

**Who does what (firmware 2.x).** The ESP32 does all the motion: the controllers, the sensor
noise filter, homing, the span calibration, stopping an axis at an end switch.
The Pi reads the five limit switches and forwards them to the ESP32 (`SW 01000` lines from a
background thread), sends one command at a time, and runs the screens, scan/spray jobs, logs,
the step-test CSVs and the camera. The ESP32's BOOT button does nothing; SPACE is the e-stop.

Python 3.11+ (Raspberry Pi OS Bookworm), stdlib `curses`, pyserial (`pip install -r requirements.txt`; `make_bundle.py` vendors it into the offline zip).
The ESP32 firmware is in `firmware/gantry_fw/`.

## Run

```sh
cp gantry.example.json gantry.json   # local settings, not tracked
python3 gantry.py                  # on the Pi, over ssh, in a 100x30 terminal (80x24 also works)
python3 gantry.py --sim            # anywhere: fake ESP32, switches, camera and access point
python3 gantry.py --sim --speed 4  # same, four times faster
python3 -m unittest -v test_gantry # 53 headless tests
firmware/sim/build.sh && firmware/sim/fwsim unit && python3 -m unittest -v test_e2e   # firmware + end to end
```

### Pi without internet

```sh
scp gantry-cli.zip <user>@192.168.4.1:~                 # from the laptop, joined to gantry-net
python3 -m zipfile -e gantry-cli.zip ~ && cd ~/gantry-cli && python3 gantry.py   # on the Pi
```

`python3 make_bundle.py` rebuilds the zip on a machine with internet.

## One-time setup

**ESP32-C3 firmware.** Open `firmware/gantry_fw/gantry_fw.ino` in the Arduino IDE, board
"ESP32C3 Dev Module", Tools > USB CDC On Boot = **Enabled** (the sketch refuses to compile
otherwise, because USB serial would land on GPIO20/21, the Pi link). esp32 core 3.x. Over USB
serial, `CFG?` answers `OK CFG FW=2.2 ...`; the CLI warns at boot if the ESP32 runs older firmware.

**Pump pin pull-down.** GPIO6 floats from reset until the firmware sets it low. With IN4 tied to
GND, a floating IN3 can run the pump and open the valve for a moment at power-up. Fit a 10 kΩ
resistor from GPIO6 to GND (GPIO6 is not a strapping pin, so this is safe).

**Pi serial port.** `sudo raspi-config` > Interface Options > Serial Port: login shell **No**,
serial hardware **Yes**, then reboot. Wiring: Pi pin 8 (GPIO14 TXD) to ESP GPIO20, Pi pin 10
(GPIO15 RXD) to ESP GPIO21, and a GND wire. Both sides are 3.3 V.

**Limit switches (read by the Pi).** For normally-open switches (COM and NO soldered), set
`"switch_nc": false` in `gantry.json`: pressed then reads LOW. The catch is that a broken or
unplugged wire reads "released", so the CLI can't catch it; press each switch by hand after
any wiring change and check its name lights up. Otherwise each switch is normally closed, wired
between its GPIO and GND. At rest the pin reads low; pressed, or with a broken wire, it reads
high, so a broken wire looks like a pressed switch and blocks homing (CTRL-06). The Pi sends the
states to the ESP32 on every change and at least every 50 ms; if they stop for 0.5 s the ESP32
stops all motion (`ERR SW_LOST`).

| switch | BCM | header pin |
|---|---|---|
| x_min (home) | 17 | 11 |
| x_max | 27 | 13 |
| y_min (home) | 22 | 15 |
| y_max | 23 | 16 |
| z_top (home) | 24 | 18 |
| GND | | 9, 14, 20 or 25 |

Change the pins in `gantry.json` (`switch_pins`). Turn on the internal pull-ups at boot so they
do not depend on the Python backend: add this line to `/boot/firmware/config.txt` and reboot:

```
gpio=17,27,22,23,24=ip,pu
```

The CLI reads the pins with `python3-lgpio`, then `python3-rpi.gpio`, then sysfs (whichever
works first; `switch_backend` in `gantry.json` forces one). If none works, the boot check
shows the switches in red and homing is refused. `sudo apt install python3-lgpio` fixes that.

## Running without limit switches

Set `"switches_connected": false` in `gantry.json`. The switches are then not read, the ESP32
is told `SET SW 0`, and homing (`y` at boot, `h`, `:home`) becomes **set datum here**: the
ESP32 zeroes X, Y and Z where the carriage sits (`HOME`). Nothing moves, so an
axis without a motor wired yet can't stall it. So push the carriage to X min, Y min with the nozzle at the top by hand
first. Moves, scan and spray then work. Nothing stops an axis at the frame end except the soft
limits, which are only as good as where you started, so keep a hand near the e-stop.
Set it back to `true` once the switches are wired. Span calibration needs the switches.

## What happens

**Boot.** Checks: UART, ESP32 (PING), config (the axis settings in `gantry.json` are sent to
the ESP32 with SET lines, so the Pi's file is the one source of truth), switches (backend,
any pressed, broken-wire pairs), camera, model, access point, SSH. Then "Home all axes now?".

**Homing (CTRL-06), on the ESP32.** The Pi sends `HOME ZXY`. Z first (nozzle up), then X,
then Y. The gantry is homed when its three home switches are pressed: Z top, X min, Y min. Per axis:

1. If the home switch is already pressed, the axis is already home: the count there is 0.
2. Otherwise drive toward the switch at `home_fast_v` (4 V) until it closes (no switch within
   180 s: `ERR HOME <axis> no switch within 180 s`).
3. Brake; the count where the switch closed becomes 0 and the axis is homed.

There is no back-off and no parking, so each axis stays on its switch (position 0, below the 5 mm
soft limit). A move or jog that goes back toward the soft-limit range is allowed; one that goes
further out is refused. `home_slow_v` and `home_offset` are not used by HOME any more
(`home_offset` is still where SPAN parks). Needs firmware 2.4 (flash `gantry_fw.ino`).

Before the approach the ESP32 brakes and waits until the axis has stopped coasting
(single-channel sensors credit every pulse to the last driven direction). The Pi refuses to
home if both switches of an axis read pressed (a broken wire on NC switches).

**Scan.** Frames 2 x 4 (8 frames, the DEC-12 note), along Y and snaking in X. Detections that
overlap between frames are merged (`merge_mm`). Weed detection is not wired yet (see below), so
on the Pi only `scan --dry` runs.

**Camera.** At boot the CLI looks for a camera: a ribbon-cable Pi camera through `rpicam-still`
(or `libcamera-still`) `--list-cameras`, then the legacy `raspistill`, then a USB webcam through
`fswebcam` or `ffmpeg`. The boot check, the `cam` light on the status bar and the detection
panel show what it found. `:photo`, or `c` on the jog screen, takes a photo in the background and
saves it as `logs/photos/photo_<date>_<time>_X120_Y300_Z5.jpg` (the gantry position in the name).
Settings: `camera_source` (auto, rpicam, legacy, usb, none), `photo_width` / `photo_height`
(1640 x 1232; 0 and 0 = full size), `photo_settle_ms` (1000), `photo_dir`. If it says "not found",
run `rpicam-hello --list-cameras` on the Pi: "No cameras available" means the ribbon cable or
`camera_auto_detect=1` in `/boot/firmware/config.txt`.

**Spray pass (ACT).** Per ticked target: `MOVE X Y` at the safe height, `MOVE Z<spray_z>`,
`SPRAY <spray_ms>`, `MOVE Z<safe_z>`, in nearest-neighbour order (SW-03). A weed whose centre is
within `guard` (45 mm = 2 x 20 mm tolerance + 5 mm weed radius) of a crop canopy edge is never
sprayed and cannot be forced (ACT-09). Every decision goes to `logs/sprays.csv` with time,
coordinates, confidence, nearest crop, gap and outcome (ACT-06). The pump and the normally
closed valve share the L298 output (EMBED-01 note), driven fully on, never PWM.

**Safety.**
- SPACE is the e-stop on every screen: the Pi latches, sends `ESTOP`, and the ESP32 refuses all
  motion and spraying until `:reset`. The BOOT button on the ESP32 is not used.
- The ESP32 stops an axis at the end switch it is moving towards (`ERR LIMIT y_max`) and refuses
  to drive further into a pressed switch.
- The ESP32 refuses moves outside the soft limits once an axis is homed.
- **No stall detection** (firmware 2.2). A move that can't finish ends with `ERR TIMEOUT` after
  15 s plus its distance at 3 mm/s (Z: 1 mm/s); each homing approach and span leg gives up after
  180 s (`ERR HOME`, `ERR SPAN`). The axis keeps its home. To find out whether a sensor or motor
  is the problem, run the sensor check on the calibration screen (`v`).
- While a long command runs the Pi sends `KA` every 0.5 s (and the switch states every 50 ms);
  the ESP32 stops everything after 1 s without a byte from the Pi (EMBED-08). Three silent seconds on the Pi side mean LINK
  LOST: motion is blocked and every axis needs a re-home.
- Lines that are not commands are ignored by the firmware, so Pi or ESP boot text can never
  start a motor.

## Calibration (`k` on the dashboard, or `:calib`)

**Step tests (system ID).** Choose axis, voltage (V-equivalent: duty x `rail_v`), direction,
ticks (the "pulses" field), repeats and alternating direction; `s` starts. The ESP32 waits until
the axis is still, applies the voltage, and timestamps **every raw rising edge** on the sensor
pin (an interrupt, no filter) until its filtered pulse count reaches the ticks (or the timeout,
2 s without a pulse (end=stall), an end switch, or SPACE). Each run that started is saved
unprocessed, even with no edges at all (then the file is just the header), as
`logs/sysid/{axis}_{volts}_{pos|neg}_test.csv`, e.g. `Y_3.0_pos_test.csv`, with two columns:

```
edge,t_us
1,48210
2,79655
```

`t_us` is microseconds from the moment the voltage was applied. Motor noise shows up as extra
edges, by design. Repeats get `_2`, `_3` ...; nothing is ever overwritten. `runs_log.csv` gets
one row per run (raw edges, filtered pulses, how it ended, rail voltage, time). The screen's
speed column skips edges closer than 3 ms, the file doesn't. Step tests also run unhomed; then
only the end switches stop the axis. Command line: `step y 3 + 100 3 alt`.

After a run the bottom line says exactly what was saved (`saved Y_3.0_pos_test.csv in
logs/sysid`), or why not: the axis was already at an end switch in that direction (the test
never started), the switch states were lost, or the folder can't be written.

**Sensor check (`v`, or `:check`).** The only place a stall is detected now. For Z, X and Y in
turn, the ESP32 runs a short step test (`check_ticks` = 10 pulses at `check_v` = 5 V, away from
the nearest end) and the CLI judges the result: *ok* (pulses, clean), *ok but noisy* (more than 3
raw edges per pulse), *only n of 10* (slow or sticking) or *FAIL* (no pulses for 2 s: sensor
unplugged, no 3V3, misaligned with the slots, or the motor isn't turning). Results go to the
STEP RESULTS table and the bottom line; nothing is saved.

**Span: mm per count (X and Y).** Homed axis, `m` starts: the ESP32 drives to the far switch
and back `2 x passes + 1` times (`SPAN X 2`) and reports the raw count at each trip. The spread shows repeatability
(ATP-05: at most 1 count) and belt slip. Measure the distance between the two switch trip points
once with a steel rule and set `span_x_mm` / `span_y_mm`; the screen then shows
`mm/count = span / mean counts`, and `a` (or `:apply`) writes it to `gantry.json` and the
ESP32. Results go to `logs/calibration/span_<axis>_<time>.csv`. Command line: `span x 3`.

## UART protocol (115200 8N1, one line per command)

The full list is at the top of `firmware/gantry_fw/gantry_fw.ino`.

| Pi sends | ESP32 replies |
|---|---|
| `PING` | `PONG` |
| `POS?` | `OK X12.3 Y45.6 Z5.0 HOMED=XYZ SW=01000` |
| `MOVE X300 Y500` · `JOG Y+10` | `PROG X.. Y.. Z..` every 0.5 s, then `OK X.. Y.. Z..` (JOG unhomed: 50 mm at most) |
| `HOME ZXY` | `PROG ...`, then `OK HOME X.. Y.. Z.. HOMED=XYZ` |
| `SPAN X 2` | `PROG ...`, then `OK SPAN X C=<count at each trip>` |
| `STEP Y 3 + 100 15` | `D k,<us>` per raw edge, then `OK STEP Y edges=.. pulses=.. end=.. t_end=.. lost=..` |
| `SPRAY 250` | `OK SPRAY 250` (1 to 3000 ms) |
| `ESTOP` · `RESET` · `ABORT` · `STOP` | `OK ESTOP` · `OK RESET` · `OK ABORT` · `OK STOP X.. Y.. Z..` |
| `PAUSE` · `RESUME` | `OK PAUSE` · `OK RESUME` |
| `SET MMPC X 1.0661` · `SET SOFT X 5 615` · `SET GAIN X kp kd vlim v0` · `SET SIGN X -1` · `SET RAIL 12` · `SET HOME 4 2.5 5` · `SET SW 1` | `OK SET ...` |
| `CFG?` | `OK CFG FW=2.2 ...` |
| `SW 01000` · `KA` | nothing (switch states x_min x_max y_min y_max z_top / keep-alive) |

Errors: `ERR ESTOP latched`, `ERR BUSY`, `ERR BAD_ARG`, `ERR NOT_HOMED <axis>`,
`ERR SOFT_LIMIT <axis>`, `ERR LIMIT <switch>`, `ERR TIMEOUT`, `ERR HOME <axis> ...`,
`ERR SPAN <axis> ...`, `ERR SW_LOST`, `ERR WATCHDOG`, `ERR STOPPED`.
Unprompted: `BOOT gantry-fw 2.2` after a reset (the CLI then un-homes every axis and re-sends
the settings).

## Controller (firmware)

The one-sided saturated PD from the Y-axis controller design, on every axis, every 5 ms:
`u = clamp(V0 + Kp*e - Kd*v + nudge, 0, Vlim)`, slew 50 V/s, never driven against the move, a
2 V/s nudge after 150 ms without a pulse, braking within one count of the target, done after
200 ms with no pulse. Pulses are credited in the direction the axis was last driven. Gains come
from `gain_x/y/z` in `gantry.json` (`Kp, Kd, Vlim, V0`).

`V0` is an optional dead-band offset, 0 by default so the controller is the one you designed
(on the bench, jogs stalled with V0 = 0: try `"gain_y": [0.3, 0.05, 7.0, 2.0]`).
In the firmware simulator with a 1 V dead-band (your assembled gantry showed about 1.5 V), 25
and 50 mm Y moves took 3.7 s with V0 = 0 and 1.9 s with V0 = 1.0, with no overshoot or reversal
either way. Set it from the loaded step tests.

**Sensor noise filter.** The sensor pins are sampled every loop pass rather than by edge
interrupts. A level has to hold for a net 1 ms before it counts, so motor and L298 switching
spikes (microseconds wide) are ignored while real slot pulses (about 10 ms even at full speed)
pass. Counts closer than 3 ms apart are dropped, and an axis that hasn't been driven for
0.6 s ignores its sensor (so X and Z can't count while only Y runs). Every pulse timestamp lags
the real edge by about 1 ms, which is a constant dead time in step-test data, not a change in
the intervals. Wiring still matters: route the sensor wires away from the motor leads, twist
each signal with its GND, and run the motor and logic grounds back to the supply separately.

## Settings to measure (gantry.json)

| key | now | from |
|---|---|---|
| `spray_z`, `spray_ms` | 120 mm, 250 ms | ATP-08: 50 mm footprint, dose |
| `span_x_mm`, `span_y_mm` | 0 (not measured) | steel rule, switch trip to trip |
| `gain_x/y/z` | design values; Z guessed | loaded step tests, ATP-03 |
| `motor_sign_x/y/z` | 1 | first jog: if an axis moves the wrong way, set -1 |
| `rail_v` | 12 V | measure at the L298 boards |
| `beds/js06-bed.json` | sample layout | the real bed (crop positions and canopy radii) |
| `switch_pins` | BCM 17, 27, 22, 23, 24 | your wiring |

Change any of them for one session with `:set <key> <value>`; axis settings are sent to the
ESP32 with `u` in config > esp32 link (or at the next boot).

## Files

| file | what |
|---|---|
| `gantry.py` | entry point, curses loop |
| `app.py` | screens, keys, the `:` command line, prompts, boot checks, calibration fields |
| `controller.py` | UART queue, timeouts, keep-alive, homing, span and step procedures (run on the ESP32), scan/spray jobs, e-stop |
| `switches.py` | limit switch backends and the Forwarder thread that sends them to the ESP32 |
| `camera.py` | finds the camera and takes photos (rpicam-still, raspistill or a USB webcam) |
| `render.py` | every screen, drawn on a Canvas so tests render to text |
| `link.py` | `SerialLink` (pyserial) and `SimLink` (fake ESP32 with physical positions behind the switches) |
| `model.py` | settings, bed geometry (map view), weeds, frame plan, spray order |
| `vision.py` | capture + detection interface; `PiVision` shows the camera, detection is **not wired yet** |
| `system.py` | read-only probes: services, AP clients, ssh sessions, uptime |
| `commands.py` | completions, help text, argument parsing |
| `beds/js06-bed.json` | sample bed: 4 crops at 150 mm spacing, 30 mm canopies, sim weeds |
| `firmware/gantry_fw/` | ESP32-C3 motion firmware |
| `firmware/sim/` | host build of the firmware with a plant model, sensor noise and the switches (`./build.sh && ./fwsim unit`) |
| `test_gantry.py` | headless tests (against `SimLink`) |
| `test_e2e.py` | this CLI against the compiled firmware over a pseudo-terminal |

Logs: `logs/gantry-YYYYMMDD.log` (every UART line and stage, SW-04), `logs/sprays.csv`
(ACT-06), `logs/state.json` (last spray pass), `logs/sysid/`, `logs/calibration/`, `logs/photos/`.

## Not done yet, or needs hardware

- **Weed detection is not wired.** The camera is found and takes photos, but there is no model,
  so a real `scan` is refused and only `scan --dry` runs. It needs a per-frame capture and detect function that
  returns bed coordinates (DEC-06).
- **Firmware 2.x has not run on the real ESP32 yet.** Tested with 53 headless tests, 47
  firmware checks in the simulator (homing, span, end switches, move timeout, watchdog, lost switch
  states, e-stop, abort, spray, step data with sensor noise), and this CLI against the compiled
  firmware over a pseudo-terminal.
  The ESP32 toolchain could not be downloaded here, so the firmware was compiled on the host
  against stub headers with the Arduino IDE's prototype step emulated, and its API calls were
  checked against the esp32 core 3.3 source. Expect small fixes on first flash.
- **Z** gains and motion are not identified; X and Z motor types are still open.
- **E-stop button.** There is no hardware e-stop on the ESP32 (the BOOT button is not used).
  SPACE in the CLI stops everything; a switch in the 12 V supply is the real emergency stop.

## Jog screen steps

Keys `1`-`5` pick the jog step: 1, 5, 10, 50, 100 mm. Key `6` asks for your own step in mm (e.g. `25`);
it stays selected until you press another step key, and `6` again changes it. Shift + arrow is always 1 mm.
Unhomed axes still jog 50 mm at most per move. (The old `t` / `z` nozzle keys are gone.)
