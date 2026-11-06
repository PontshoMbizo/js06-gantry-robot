"""
Gantry CLI: the terminal UI that runs on the Pi Zero W and drives the
ESP32-C3 (firmware/gantry_fw 2.x) over /dev/serial0. The ESP32 does the motion,
homing, end-switch stops and filtering; the Pi reads the five limit switches and
forwards them, and finds the camera.

    python3 gantry.py                 # on the Pi, real UART and switches
    python3 gantry.py --sim           # anywhere: fake ESP32, switches, camera and AP
    python3 gantry.py --sim --speed 4 # same, four times faster

Designed for a 100×30 terminal (ssh 192.168.4.1); works down to 80×24.
SPACE is the e-stop on every screen.
"""

import argparse
import curses
import locale
import os
import sys
import time

# the offline bundle ships pyserial in vendor/ (the Pi has no internet for pip)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "vendor"))

import render
import theme
from app import App
from controller import Gantry
from link import SerialLink, SimLink
from model import Bed, Settings
from camera import Camera
from switches import Forwarder, open_switches
from system import SimSystem, System
from vision import PiVision, SimVision

HERE = os.path.dirname(os.path.abspath(__file__))
TICK_MS = 50


def build(args) -> App:
    settings = Settings.load(args.config)
    if args.port:
        settings.port = args.port
    if args.switches:
        settings.switch_backend = args.switches
    bed = Bed.load(args.bed or settings.bed_file, view=settings.map_view)
    clock = time.monotonic
    camera = None
    if args.sim:
        link = SimLink(clock, speed=args.speed)
        vision = SimVision(bed, settings, clock, speed=args.speed)
        system = SimSystem()
    else:
        link = SerialLink(settings.port, settings.baud)
        camera = Camera(settings).start()
        vision = PiVision(camera)
        system = System()
    switches = open_switches(settings, link, sim=args.sim)
    forwarder = None
    if not args.sim:      # the simulator models its own switches
        forwarder = Forwarder(link, switches, settings.switches_connected).start()
    gantry = Gantry(settings, bed, link, vision, system, switches, clock=clock, camera=camera)
    gantry.forwarder = forwarder
    return App(gantry, sim_link=link if args.sim else None)


def read_keys(scr) -> list:
    """Everything typed since the last tick, so SPACE can be seen first."""
    keys = []
    try:
        k = scr.get_wch()
    except curses.error:
        return keys
    keys.append(k)
    scr.nodelay(True)
    while True:
        try:
            keys.append(scr.get_wch())
        except curses.error:
            break
    scr.timeout(TICK_MS)    # not nodelay(False): that would block until the next key
    return keys


def loop(scr, app: App) -> None:
    curses.curs_set(0)
    scr.keypad(True)
    scr.timeout(TICK_MS)
    theme.init()
    scr.bkgd(" ", theme.attr("t"))
    last_draw = 0.0
    while app.running:
        keys = read_keys(scr)
        app.handle_keys(keys)
        app.tick()
        now = time.monotonic()
        if keys or now - last_draw >= 0.1:
            scr.erase()
            render.draw(app, render.CursesCanvas(scr, theme.attr))
            scr.refresh()
            last_draw = now


def main() -> int:
    p = argparse.ArgumentParser(description="Gantry CLI")
    p.add_argument("--sim", action="store_true", help="fake ESP32, camera and AP (no hardware)")
    p.add_argument("--speed", type=float, default=1.0, help="sim speed-up factor")
    p.add_argument("--port", help="serial port (default /dev/serial0)")
    p.add_argument("--bed", help="bed layout json (default beds/js06-bed.json)")
    p.add_argument("--switches", help="switch backend: auto, lgpio, rpigpio, sysfs, none or file:<path>")
    p.add_argument("--config", default=os.path.join(HERE, "gantry.json"), help="settings json")
    args = p.parse_args()

    locale.setlocale(locale.LC_ALL, "")
    os.environ.setdefault("ESCDELAY", "25")
    try:
        app = build(args)
    except ImportError:
        print("pyserial is missing: pip install -r requirements.txt  (or try --sim)", file=sys.stderr)
        return 1
    except OSError as e:
        print(f"cannot open the UART: {e}\ncheck the port, or try --sim", file=sys.stderr)
        return 1
    try:
        curses.wrapper(loop, app)
    except KeyboardInterrupt:
        pass
    finally:
        if getattr(app.g, "forwarder", None):
            app.g.forwarder.stop()
        app.g.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
