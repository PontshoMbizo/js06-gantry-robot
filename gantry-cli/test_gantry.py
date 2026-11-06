"""
Headless tests: the sim ESP32 (SimLink) and sim switches on a fake clock, the real App and
Gantry, and render.Canvas instead of curses. Run with:  python3 -m unittest -v
"""

import csv
import curses
import json
import os
import subprocess
import tempfile
import time
import unittest

import render
from app import App
from camera import Camera
from controller import Gantry
from link import SimLink
from model import Bed, Settings
from switches import NoSwitches, SimSwitches
from system import SimSystem
from vision import PiVision, SimVision


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


_made = []


def tearDownModule():
    for app in _made:
        app.g.close()


def make(vision=None, switches=None, start=(250.0, 400.0, 60.0), camera=None):
    clk = Clock()
    s = Settings()
    tmp = tempfile.mkdtemp()
    s.log_dir, s.sysid_dir, s.calib_dir = tmp, os.path.join(tmp, "sysid"), os.path.join(tmp, "cal")
    s.photo_dir = os.path.join(tmp, "photos")
    s.config_path = os.path.join(tmp, "gantry.json")
    bed = Bed.load(s.bed_file, view=s.map_view)
    link = SimLink(clk, start=start)
    vis = vision or SimVision(bed, s, clk)
    sw = switches if switches is not None else SimSwitches(link)
    if camera is not None:
        camera.s = s
    g = Gantry(s, bed, link, vis, SimSystem(), sw, clock=clk, wall=lambda: 1.79e9 + clk.t, camera=camera)
    app = App(g, sim_link=link)
    _made.append(app)
    return app, clk


def run(app, clk, secs, until=None, dt=0.05):
    end = clk.t + secs
    while clk.t < end:
        clk.t += dt
        app.tick()
        if until and until():
            return True
    return False


def press(app, clk, *keys):
    for k in keys:
        app.handle_keys([k])
        run(app, clk, 0.1)


def typed(app, clk, text):
    press(app, clk, ":")
    for ch in text:
        app.handle_keys([ch])
    app.handle_keys(["\n"])
    run(app, clk, 0.1)


def booted(app, clk, home=True):
    run(app, clk, 6, lambda: app.boot_phase == "prompt")
    app.handle_keys(["y" if home else "n"])
    run(app, clk, 300, lambda: app.mode == "dash")


def homed(**kw):
    app, clk = make(**kw)
    booted(app, clk)
    assert all(a.homed for a in app.g.axes.values()), app.g.notice
    return app, clk


def parked(**kw):
    """Homed, then 5 mm off the switches (homing leaves each axis standing on its switch)."""
    app, clk = homed(**kw)
    app.g.send("MOVE X5 Y5 Z5")
    run(app, clk, 60, lambda: not app.g.pending and not app.g.queue)
    return app, clk


def scanned(**kw):
    app, clk = homed(**kw)
    press(app, clk, "s")
    run(app, clk, 300, lambda: app.g.job is None)
    return app, clk


def screen(app, w=100, h=30):
    c = render.Canvas(h, w)
    render.draw(app, c)
    return c


class Boot(unittest.TestCase):
    def test_checks_run_in_order_then_prompt(self):
        app, clk = make()
        run(app, clk, 6, lambda: app.boot_phase == "prompt")
        self.assertEqual(list(app.boot_checks), ["uart", "esp32", "config", "switches", "camera", "model", "ap", "ssh"])
        self.assertEqual(app.boot_checks["esp32"][0], "ok")
        self.assertEqual(app.boot_checks["config"][0], "ok")
        self.assertTrue(app.g.config_pushed)
        sent = [l for l in app.g.link.log if l.startswith("SET")]
        self.assertIn("SET MMPC Z 0.2667", sent)
        self.assertIn("SET SOFT Y 5 1055", sent)

    def test_skip_homing_blocks_scan(self):
        app, clk = make()
        booted(app, clk, home=False)
        press(app, clk, "s")
        self.assertIn("not homed", app.g.notice[1])

    def test_switch_fault_shown_at_boot(self):
        app, clk = make()
        app.g.switches.forced = {"y_min": True, "y_max": True}
        run(app, clk, 6, lambda: app.boot_phase == "prompt")
        self.assertEqual(app.boot_checks["switches"], ("fail", "y_min and y_max both pressed"))


class EstopSync(unittest.TestCase):
    def test_esp_latched_without_the_pi_knowing(self):
        app, clk = homed()
        g = app.g
        g.link.estop = True                       # the ESTOP line from the ESP never arrived
        run(app, clk, 5, lambda: not g.pending and not g.queue)
        self.assertIsNone(g.jog("Y", 10))
        run(app, clk, 5, lambda: g.estop)
        self.assertTrue(g.estop, "Pi should latch on ERR ESTOP latched")
        self.assertIsNone(g.reset())
        run(app, clk, 5, lambda: not g.estop)
        self.assertFalse(g.estop)
        self.assertFalse(g.link.estop)


class NormallyOpen(unittest.TestCase):
    def test_no_switches_read_low_as_pressed(self):
        from switches import Switches, NAMES
        class Raw(Switches):
            backend = "test"
            def _read_raw(self):
                return {"x_min": False, "x_max": True, "y_min": True, "y_max": True, "z_top": None}
        sw = Raw()
        sw.invert = True
        st = sw.read()
        self.assertEqual(st, {"x_min": True, "x_max": False, "y_min": False, "y_max": False, "z_top": None})
        sw.invert = False
        self.assertEqual(sw.read()["x_min"], False)


class NoSwitchesDatum(unittest.TestCase):
    def test_datum_here_then_moves_and_jogs_work(self):
        from switches import NoSwitches
        app, clk = make(switches=NoSwitches())
        g = app.g
        g.s.switches_connected = False
        run(app, clk, 6, lambda: app.boot_phase == "prompt")
        self.assertEqual(app.boot_checks["switches"][0], "warn")
        start = {a: g.axes[a].pos for a in "XYZ"}
        app.handle_keys(["y"])
        run(app, clk, 120, lambda: app.mode == "dash")
        self.assertTrue(all(a.homed for a in g.axes.values()), g.notice)
        self.assertTrue(g.pos_trusted)
        self.assertIn("SET SW 0", g.link.log)
        self.assertIn("HOME ZXY", g.link.log)
        for a in "XYZ":
            self.assertAlmostEqual(g.axes[a].pos, start[a] - start[a], delta=1.2)
        self.assertFalse(any(l.startswith("MOVE") for l in g.link.log))
        self.assertIsNone(g.goto(100.0, 200.0))
        run(app, clk, 120, lambda: not g.pending and not g.queue and not g.motion_active())
        self.assertAlmostEqual(g.axes["X"].pos, 100.0, delta=1.2)
        self.assertFalse(g.estop)


class Homing(unittest.TestCase):
    def test_home_runs_on_the_esp(self):
        app, clk = homed()
        g, link = app.g, app.g.link
        self.assertIn("HOME ZXY", link.log)
        self.assertFalse(any(l.startswith(("DRIVE", "STOP")) for l in link.log))
        for a in "XYZ":                                              # homed = standing on the switch, no back-off, no park
            self.assertAlmostEqual(link.phys[a], 0.0, delta=0.5)
            self.assertAlmostEqual(g.axes[a].pos, 0.0, delta=0.5)
        st = g.switches.read()                                       # x_min, y_min and z_top are all pressed
        self.assertTrue(st["x_min"] and st["y_min"] and st["z_top"], st)
        self.assertTrue(g.pos_trusted)
        self.assertIn("SET SW 1", link.log)
        self.assertIn("SET HOME 4 2.5 5", link.log)

    def test_z_switch_stays_pressed_is_not_an_error(self):
        app, clk = homed()
        self.assertNotIn("ERR HOME", app.g.notice[1])
        self.assertTrue(app.g.switches.read()["z_top"])

    def test_jog_off_the_switch_after_homing(self):
        app, clk = homed()
        g = app.g
        self.assertIsNone(g.jog("X", 1.0))                          # 1 mm is below the 5 mm soft limit but moves away from the switch
        run(app, clk, 5, lambda: not g.pending)
        self.assertGreater(g.axes["X"].pos, 0.3)
        self.assertLess(g.axes["X"].pos, 2.2)
        self.assertIn("soft limit", g.jog("X", -3.0) or "")          # but never further out than that

    def test_starting_on_the_switch(self):
        app, clk = make(start=(250.0, -1.0, 60.0))
        booted(app, clk)
        self.assertTrue(app.g.axes["Y"].homed)

    def test_broken_wire_blocks_homing(self):
        app, clk = make()
        app.g.switches.forced = {"x_min": True, "x_max": True}
        run(app, clk, 6, lambda: app.boot_phase == "prompt")
        self.assertEqual(app.boot_checks["switches"][0], "fail")
        app.handle_keys(["y"])
        run(app, clk, 120, lambda: app.mode == "dash")
        self.assertFalse(app.g.axes["X"].homed)
        self.assertIn("broken wire", app.g.notice[1])
        self.assertNotIn("HOME ZXY", app.g.link.log)

    def test_blocked_homing_times_out_no_stall(self):
        app, clk = make()
        app.g.link.jam.add("Z")
        booted(app, clk)
        self.assertIn("ERR HOME Z", app.g.notice[1])
        self.assertFalse(app.g.axes["Z"].homed)

    def test_no_switches_no_homing(self):
        app, clk = make(switches=NoSwitches())
        booted(app, clk)
        self.assertIn("not readable", app.g.notice[1])
        self.assertFalse(any(a.homed for a in app.g.axes.values()))


class Spray(unittest.TestCase):
    def test_scan_refused_without_vision_unless_dry(self):
        app, clk = homed(vision=PiVision())
        press(app, clk, "s")
        self.assertIn("detection not wired", app.g.notice[1])
        typed(app, clk, "scan --dry")
        self.assertIsNotNone(app.g.job)
        run(app, clk, 300, lambda: app.g.job is None)
        self.assertEqual(app.g.scan_summary["frames"], 8)

    def test_scan_merges_overlaps_numbers_along_the_map_and_flags_act09(self):
        app, clk = scanned()
        w = app.g.weeds
        self.assertEqual(len(w), 6)                      # the two boundary weeds are not doubled, 0.41 conf dropped
        self.assertEqual([x.y for x in w], sorted(x.y for x in w))
        close = {x.id for x in w if x.guarded}
        self.assertEqual(len(close), 2)
        self.assertTrue(all(not x.ticked for x in w if x.guarded))

    def test_close_weed_cannot_be_ticked(self):
        app, clk = scanned()
        close = next(w for w in app.g.weeds if w.guarded)
        err = app.g.tick_target(close.id, True)
        self.assertIn("ACT-09", err)
        typed(app, clk, f"weed {close.id}")
        self.assertIn("ACT-09", app.g.notice[1])

    def test_spray_pass_order_moves_and_spray_log(self):
        app, clk = scanned()
        g = app.g
        press(app, clk, "w")
        self.assertEqual(app.mode, "review")
        app.handle_keys(["\n"])
        self.assertIn("2 untreated", app.prompt[0])
        app.handle_keys(["y"])
        run(app, clk, 600, lambda: g.job is None)
        st = {w.status for w in g.weeds}
        self.assertEqual(st, {"sprayed", "untreated"})
        sent = [l for l in g.link.log if l != "KA"]
        first = sent.index(next(l for l in sent if l.startswith("SPRAY")))
        self.assertTrue(sent[first - 2].startswith("MOVE X"))
        self.assertEqual(sent[first - 1], f"MOVE Z{g.s.spray_z:g}")
        self.assertEqual(sent[first + 1], f"MOVE Z{g.s.safe_z:g}")
        with open(os.path.join(g.s.log_dir, "sprays.csv")) as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 6)
        self.assertEqual(sum(r["outcome"] == "sprayed" for r in rows), 4)
        self.assertEqual(sum(r["outcome"].startswith("untreated") for r in rows), 2)
        self.assertEqual(g.weed_summary["untreated"], 2)

    def test_pause_stops_at_next_frame(self):
        app, clk = homed()
        press(app, clk, "s")
        run(app, clk, 30, lambda: app.g.job.frames_done >= 1)
        press(app, clk, "p")
        run(app, clk, 60, lambda: app.g.job.state == "paused")
        done = app.g.job.frames_done
        run(app, clk, 10)
        self.assertEqual(app.g.job.frames_done, done)
        typed(app, clk, "resume")
        run(app, clk, 300, lambda: app.g.job is None)
        self.assertEqual(app.g.scan_summary["frames"], 8)

    def test_abort_keeps_partial_results_and_can_resume(self):
        app, clk = homed()
        press(app, clk, "s")
        run(app, clk, 120, lambda: app.g.job.frames_done >= 3)
        press(app, clk, "a")
        self.assertIsNone(app.g.job)
        self.assertEqual(app.g.resume_scan_from, 3)
        run(app, clk, 60, lambda: not app.g.pending and not app.g.queue)
        typed(app, clk, "scan --resume")
        run(app, clk, 300, lambda: app.g.job is None)
        self.assertEqual(app.g.scan_summary["frames"], 8)

    def test_space_latches_mid_spray_and_resume_raises_first(self):
        app, clk = scanned()
        g = app.g
        press(app, clk, "w")
        app.handle_keys(["\n"])
        app.handle_keys(["y"])
        run(app, clk, 120, lambda: g.pending and g.pending.line.startswith("SPRAY"))
        app.handle_keys([" "])
        self.assertTrue(g.estop)
        run(app, clk, 1)
        self.assertTrue(g.estop_confirmed)
        typed(app, clk, "reset")
        run(app, clk, 2, lambda: not g.estop)
        n = len(g.link.log)
        typed(app, clk, "resume")
        run(app, clk, 30, lambda: any(l.startswith("MOVE") for l in g.link.log[n:]))
        moves = [l for l in g.link.log[n:] if l.startswith("MOVE")]
        self.assertEqual(moves[0], f"MOVE Z{g.s.safe_z:g}")


class Keys(unittest.TestCase):
    def test_space_on_idle_dashboard_latches(self):
        app, clk = homed()
        app.handle_keys([" "])
        self.assertTrue(app.g.estop)

    def test_space_types_in_command_line_when_nothing_moves(self):
        app, clk = homed()
        press(app, clk, ":")
        app.handle_keys(["s", "e", "t", " "])
        self.assertEqual(app.cmd, "set ")
        self.assertFalse(app.g.estop)

    def test_space_in_command_line_is_estop_while_moving(self):
        app, clk = homed()
        typed(app, clk, "jog")
        app.handle_keys([curses.KEY_RIGHT])
        run(app, clk, 0.1)
        press(app, clk, ":")
        app.handle_keys([" "])
        self.assertTrue(app.g.estop)

    def test_jog_arrows_follow_the_map(self):
        app, clk = homed()
        typed(app, clk, "jog")
        app.handle_keys([curses.KEY_RIGHT])
        run(app, clk, 2, lambda: not app.g.pending)
        app.handle_keys([curses.KEY_UP])
        run(app, clk, 2, lambda: not app.g.pending)
        jogs = [l for l in app.g.link.log if l.startswith("JOG")]
        self.assertEqual(jogs, ["JOG Y+10", "JOG X+10"])          # y-right view: right = +Y, up = +X

    def test_jog_refused_past_soft_limit(self):
        app, clk = homed()
        self.assertIsNone(app.g.jog("Y", 3.0))                      # off the switch, still inside the 5 mm soft margin
        run(app, clk, 5)
        self.assertAlmostEqual(app.g.axes["Y"].pos, 3.0, delta=1.2)
        typed(app, clk, "jog")
        app.handle_keys([curses.KEY_LEFT])
        run(app, clk, 1)
        self.assertIn("soft limit", app.g.notice[1])

    def test_test_spray_needs_f_twice_and_respects_clearance(self):
        app, clk = homed()
        typed(app, clk, "jog")
        app.handle_keys(["f"])
        self.assertIn("press f again", app.g.notice[1])
        app.handle_keys(["f"])
        run(app, clk, 1)
        self.assertIn("SPRAY 250", app.g.link.log)
        app.g.goto(255.0, 235.0)                                   # 20 mm from a lettuce canopy edge
        run(app, clk, 60, lambda: not app.g.pending)
        app.handle_keys(["f"])
        app.handle_keys(["f"])
        self.assertIn("ACT-09", app.g.notice[1])


class Safety(unittest.TestCase):
    def test_three_silent_seconds_mid_move_is_link_lost(self):
        app, clk = homed()
        app.g.goto(300, 900)
        run(app, clk, 1)
        app.sim_link.drop()
        run(app, clk, 4, lambda: app.g.link_lost)
        self.assertTrue(app.g.link_lost)
        self.assertFalse(any(a.homed for a in app.g.axes.values()))

    def test_idle_heartbeat_detects_loss(self):
        app, clk = homed()
        app.sim_link.drop()
        run(app, clk, 8, lambda: app.g.link_lost)
        self.assertTrue(app.g.link_lost)

    def test_keepalive_during_long_moves_and_watchdog_without_it(self):
        app, clk = homed()
        g = app.g
        n = len(g.link.log)
        g.goto(300, 1000)
        run(app, clk, 3)
        self.assertGreaterEqual(g.link.log[n:].count("KA"), 4)
        run(app, clk, 60, lambda: not g.pending)
        g.s.keepalive = 99                                        # the Pi "hangs": no KA
        g.goto(300, 100)
        run(app, clk, 4)
        self.assertTrue(any("ERR WATCHDOG" in ln.text for ln in g.log))

    def test_the_esp_stops_at_an_end_switch(self):
        app, clk = make()
        booted(app, clk, home=False)
        typed(app, clk, "jog")
        app.g.link.phys["Y"] = 4.0                                  # carriage 4 mm short of y_min, unhomed
        app.handle_keys([curses.KEY_LEFT])                          # 10 mm towards it
        run(app, clk, 3, lambda: not app.g.pending and not app.g.queue)
        self.assertAlmostEqual(app.g.link.phys["Y"], 0.0, delta=1.0)
        self.assertIn("ERR LIMIT y_min", app.g.notice[1])
        self.assertIn("end switch", app.g.notice[1])
        self.assertFalse(app.g.estop)

    def test_jog_towards_a_pressed_switch_is_refused(self):
        app, clk = make(start=(250.0, -1.0, 60.0))
        booted(app, clk, home=False)
        typed(app, clk, "jog")
        app.handle_keys([curses.KEY_LEFT])
        self.assertIn("y_min is pressed", app.g.notice[1])
        app.handle_keys([curses.KEY_RIGHT])
        run(app, clk, 1)
        self.assertIn("JOG Y+10", app.g.link.log)

    def test_esp_restart_unhomes_and_resends_settings(self):
        app, clk = homed()
        n = len(app.g.link.log)
        app.sim_link.drop()
        app.sim_link.restore()
        run(app, clk, 3)
        self.assertFalse(any(a.homed for a in app.g.axes.values()))
        self.assertTrue(any(l.startswith("SET MMPC") for l in app.g.link.log[n:]))


class Calibration(unittest.TestCase):
    def test_step_test_saves_axis_voltage_csv_with_repeats(self):
        app, clk = parked()
        g = app.g
        typed(app, clk, "step y 3 + 60 3 alt")
        self.assertIn("Step test Y 3 V", app.prompt[0])
        app.handle_keys(["y"])
        run(app, clk, 120, lambda: g.proc is None)
        files = sorted(os.listdir(g.s.sysid_dir))
        self.assertEqual(files, ["Y_3.0_neg_test.csv", "Y_3.0_pos_test.csv", "Y_3.0_pos_test_2.csv", "runs_log.csv"])
        dirs = [l.split()[3] for l in g.link.log if l.startswith("STEP")]
        self.assertEqual(dirs, ["+", "-", "+"])
        with open(os.path.join(g.s.sysid_dir, "Y_3.0_pos_test.csv")) as f:
            lines = f.read().splitlines()
        self.assertEqual(lines[0], "edge,t_us")                      # raw edges only, nothing else in the file
        self.assertEqual(len(lines), 61)
        self.assertEqual(lines[1].split(",")[0], "1")
        self.assertEqual(lines[60].split(",")[0], "60")
        self.assertTrue(all(len(l.split(",")) == 2 for l in lines))
        self.assertGreater(g.step_results[-1]["rpm"], 0)
        with open(os.path.join(g.s.sysid_dir, "runs_log.csv")) as f:
            log = f.read().splitlines()
        self.assertEqual(len(log), 4)
        self.assertTrue(log[1].startswith("Y_3.0_pos_test.csv,Y,3.0,+,60,60,60,ticks"))

    def test_step_csv_keeps_the_noise_edges(self):
        app, clk = parked()
        g = app.g
        g.link.noise_edges = 2
        self.assertIsNone(g.step_test("Z", 6.0, 1, 30))
        run(app, clk, 60, lambda: g.proc is None)
        r = g.step_results[-1]
        self.assertEqual((r["edges"], r["n"]), (90, 30))
        with open(os.path.join(g.s.sysid_dir, "Z_6.0_pos_test.csv")) as f:
            self.assertEqual(len(f.read().splitlines()), 91)
        self.assertGreater(r["rpm"], 0)                               # the screen's speed skips the noise

    def test_step_test_works_unhomed_from_the_calibration_screen(self):
        app, clk = make()
        booted(app, clk, home=False)
        press(app, clk, "k")
        self.assertEqual(app.mode, "calib")
        app.handle_keys(["s"])
        self.assertIn("NOT homed", app.prompt[0])
        app.handle_keys(["y"])
        run(app, clk, 60, lambda: app.g.proc is None)
        self.assertEqual(app.g.step_results[-1]["end"], "ticks")

    def test_span_counts_between_switches_and_apply(self):
        app, clk = parked()
        g = app.g
        typed(app, clk, "span x 2")
        app.handle_keys(["y"])
        run(app, clk, 400, lambda: g.proc is None)
        r = g.span_result
        self.assertEqual(len(r["spans"]), 4)
        self.assertLessEqual(r["spread"], 1)
        self.assertAlmostEqual(r["nominal_mm"], 620, delta=4)
        self.assertIsNone(r["mmpc"])
        typed(app, clk, "apply")
        self.assertIn("span_x_mm", app.g.notice[1])
        typed(app, clk, "set span_x_mm 620")
        g.span_result = None
        typed(app, clk, "span x 1")
        app.handle_keys(["y"])
        run(app, clk, 300, lambda: g.proc is None)
        typed(app, clk, "apply")
        with open(g.s.config_path) as f:
            saved = json.load(f)
        self.assertAlmostEqual(saved["mm_per_count_x"], 620 / g.span_result["mean"], places=4)
        self.assertTrue(any(l.startswith("SET MMPC X") for l in g.link.log[-3:]))

    def test_span_needs_homing_and_two_switches(self):
        app, clk = make()
        booted(app, clk, home=False)
        self.assertIn("home X first", app.g.span_test("x"))
        self.assertIn("one switch", app.g.span_test("z"))

    def test_calibration_fields_adjust(self):
        app, clk = make()
        booted(app, clk, home=False)
        press(app, clk, "k")
        app.handle_keys([curses.KEY_RIGHT])               # axis y -> z
        self.assertEqual(app.g.s.step_axis, "z")
        app.handle_keys([curses.KEY_DOWN, curses.KEY_RIGHT])
        self.assertEqual(app.g.s.step_volts, 3.5)
        app.handle_keys([curses.KEY_DOWN, curses.KEY_RIGHT])
        self.assertEqual(app.g.s.step_dir, -1)


class FakeRun:
    """Stands in for subprocess.run: canned output per tool, and 'takes' the photo by writing the file."""

    def __init__(self, outputs: dict):
        self.outputs, self.calls = outputs, []

    def __call__(self, cmd, **kw):
        self.calls.append(cmd)
        out, rc = self.outputs.get(cmd[0], ("", 0))
        if "-o" in cmd and rc == 0:
            with open(cmd[cmd.index("-o") + 1], "wb") as f:
                f.write(b"\xff\xd8jpeg")
        return subprocess.CompletedProcess(cmd, rc, stdout=out, stderr="")


LISTING = "Available cameras\n-----------------\n0 : imx219 [3280x2464 10-bit RGGB] (/base/soc/i2c0mux/i2c@1/imx219@10)\n"


def rpicam_camera(listing=LISTING, rc=0):
    s = Settings()
    s.photo_dir = tempfile.mkdtemp()
    cam = Camera(s, run=FakeRun({"rpicam-still": (listing, rc)}), which=lambda t: t if t == "rpicam-still" else None,
                 video_glob="/nonexistent/*")
    cam.detect()
    return cam


def wait_cam(cam):
    end = time.monotonic() + 3
    while cam.busy and time.monotonic() < end:
        time.sleep(0.01)


class NoStallSensorCheckAndSaving(unittest.TestCase):
    def test_push_config_has_no_stallv(self):
        app, clk = homed()
        self.assertFalse(any(l.startswith("SET STALLV") for l in app.g.link.log))

    def test_blocked_move_times_out_instead_of_stalling(self):
        app, clk = homed()
        g = app.g
        g.link.jam.add("Y")
        self.assertIsNone(g.goto(100, 60))
        run(app, clk, 10, lambda: not g.pending and not g.queue)
        self.assertTrue(g.pending)                                  # still trying after 10 s: no stall cut-out
        run(app, clk, 60, lambda: not g.pending and not g.queue)
        self.assertIn("timed out", g.notice[1])
        self.assertTrue(g.axes["Y"].homed)

    def test_sensor_check_all_ok(self):
        app, clk = parked()
        g = app.g
        press(app, clk, "k")
        app.handle_keys(["v"])
        self.assertIn("Sensor check", app.prompt[0])
        app.handle_keys(["y"])
        run(app, clk, 60, lambda: g.proc is None)
        self.assertEqual([l.split()[1] for l in g.link.log if l.startswith("STEP")], ["Z", "X", "Y"])
        self.assertEqual({a: r[0] for a, r in g.sensor_check_results.items()}, {"X": "ok", "Y": "ok", "Z": "ok"})
        self.assertIn("sensors: Z ok · X ok · Y ok", g.notice[1])
        self.assertFalse(os.path.exists(g.s.sysid_dir) and any(f.endswith("_test.csv") for f in os.listdir(g.s.sysid_dir)))
        self.assertIn("sensor ok", screen(app).text())

    def test_sensor_check_finds_a_dead_sensor(self):
        app, clk = homed()
        g = app.g
        g.link.jam.add("Y")
        self.assertIsNone(g.sensor_check("Y"))
        run(app, clk, 30, lambda: g.proc is None)
        st, text = g.sensor_check_results["Y"]
        self.assertEqual(st, "fail")
        self.assertIn("no pulses", text)
        self.assertIn("Y FAIL", g.notice[1])

    def test_step_test_with_no_pulses_is_still_saved(self):
        app, clk = homed()
        g = app.g
        g.link.jam.add("X")
        self.assertIsNone(g.step_test("X", 3.0, 1, 50))
        run(app, clk, 60, lambda: g.proc is None)
        r = g.step_results[-1]
        self.assertEqual(r["end"], "stall")
        with open(os.path.join(g.s.sysid_dir, "X_3.0_pos_test.csv")) as f:
            self.assertEqual(f.read(), "edge,t_us\n\n")
        self.assertIn("saved X_3.0_pos_test.csv", g.notice[1])

    def test_a_test_that_never_started_says_why(self):
        from controller import save_step_run
        app, clk = homed()
        r = save_step_run(app.g, "Y", 3.0, -1, 100, [], {"end": "limit", "edges": "0", "pulses": "0", "t_end": "0"})
        self.assertIsNone(r["path"])
        self.assertIn("end switch", r["why"])

    def test_unwritable_folder_is_reported_not_hidden(self):
        app, clk = homed()
        g = app.g
        blocker = os.path.join(g.s.log_dir, "afile")
        with open(blocker, "w") as f:
            f.write("x")
        g.s.sysid_dir = os.path.join(blocker, "sysid")            # a folder inside a file: cannot be created
        self.assertIsNone(g.step_test("Y", 3.0, 1, 20))
        run(app, clk, 60, lambda: g.proc is None)
        self.assertIn("NOT saved", g.notice[1])
        self.assertIn("cannot write", g.notice[1])


class CameraTests(unittest.TestCase):
    def test_rpicam_found_and_photo_named_by_position(self):
        cam = rpicam_camera()
        self.assertEqual((cam.state, cam.name, cam.tool), ("ok", "imx219", "rpicam-still"))
        self.assertIsNone(cam.capture({"X": 120.4, "Y": 300.0, "Z": 5.0}))
        wait_cam(cam)
        self.assertEqual(cam.last_error, "")
        self.assertRegex(os.path.basename(cam.last_photo), r"^photo_\d{8}_\d{6}_X120_Y300_Z5\.jpg$")
        self.assertEqual(cam._run.calls[-1][:3], ["rpicam-still", "-n", "-t"])

    def test_no_camera_says_why(self):
        cam = rpicam_camera(listing="No cameras available!\n")
        self.assertEqual(cam.state, "none")
        self.assertIn("ribbon", cam.error)
        self.assertIn("no camera", cam.capture({"X": 0, "Y": 0, "Z": 0}))

    def test_usb_webcam(self):
        root = tempfile.mkdtemp()
        usbdev = os.path.join(root, "devices", "usb1", "1-1")
        os.makedirs(usbdev)
        v = os.path.join(root, "video0")
        os.makedirs(v)
        os.symlink(usbdev, os.path.join(v, "device"))
        with open(os.path.join(v, "name"), "w") as f:
            f.write("HD Webcam\n")
        codec = os.path.join(root, "video10")                 # the Pi's own codec device: not a camera
        os.makedirs(os.path.join(root, "devices", "platform", "codec"))
        os.makedirs(codec)
        os.symlink(os.path.join(root, "devices", "platform", "codec"), os.path.join(codec, "device"))
        s = Settings()
        cam = Camera(s, run=FakeRun({}), which=lambda t: t if t == "fswebcam" else None, video_glob=os.path.join(root, "video*"))
        cam.detect()
        self.assertEqual((cam.state, cam.name, cam.device, cam.tool), ("ok", "USB: HD Webcam", "/dev/video0", "fswebcam"))

    def test_failed_capture_reports(self):
        cam = rpicam_camera()
        cam._run = FakeRun({"rpicam-still": ("ERROR: *** failed to start camera ***", 1)})
        self.assertIsNone(cam.capture({"X": 0, "Y": 0, "Z": 0}))
        wait_cam(cam)
        self.assertIn("failed to start camera", cam.last_error)

    def test_off(self):
        s = Settings()
        s.camera_source = "none"
        self.assertEqual(Camera(s).start().state, "off")

    def test_boot_check_status_bar_and_photo_from_the_jog_screen(self):
        cam = rpicam_camera()
        app, clk = make(vision=PiVision(cam), camera=cam)
        booted(app, clk, home=False)
        self.assertEqual(app.boot_checks["camera"], ("ok", "imx219"))
        self.assertIn("● cam", screen(app).text())
        typed(app, clk, "jog")
        app.handle_keys(["c"])
        wait_cam(cam)
        run(app, clk, 0.2)
        self.assertIn("photo saved", app.g.notice[1])
        self.assertTrue(os.path.exists(cam.last_photo))
        typed(app, clk, "photo")
        wait_cam(cam)
        run(app, clk, 0.2)
        self.assertIn("photo saved", app.g.notice[1])
        self.assertGreaterEqual(len(os.listdir(app.g.s.photo_dir)), 1)

    def test_no_camera_at_boot(self):
        cam = rpicam_camera(listing="No cameras available!\n")
        app, clk = make(vision=PiVision(cam), camera=cam)
        booted(app, clk, home=False)
        self.assertEqual(app.boot_checks["camera"], ("warn", "not found"))
        typed(app, clk, "photo")
        self.assertIn("no camera", app.g.notice[1])


class Screens(unittest.TestCase):
    def test_dashboard_grid(self):
        app, clk = homed()
        c = screen(app)
        rows = c.text().splitlines()
        self.assertTrue(rows[0].startswith(" GANTRY CLI  ▸ IDLE"))
        self.assertIn("BED MAP", rows[1])
        self.assertIn("AXES", rows[1])
        self.assertIn("JOB", rows[18])
        self.assertIn("UART", rows[18])
        self.assertIn("SPACE", rows[28])

    def test_every_screen_draws_at_both_sizes(self):
        app, clk = scanned()
        for mode in ("dash", "review", "jog", "config", "log", "calib"):
            app.mode = mode
            for i in range(10):
                app.config_sec = i
                for w, h in ((100, 30), (80, 24)):
                    screen(app, w, h)
        app.mode = "dash"
        app.handle_keys([" "])
        for w, h in ((100, 30), (80, 24)):
            self.assertIn("E-STOP LATCHED", screen(app, w, h).text())

    def test_completions_keep_word_group(self):
        import commands
        rows = commands.matches("step")
        self.assertTrue(all(r[0].startswith("step") for r in rows))
        self.assertEqual(commands.parse_dir("-"), -1)


class JogSteps(unittest.TestCase):
    def test_key_5_is_100_mm(self):
        app, clk = homed()
        app.mode = "jog"
        app.handle_keys(["5"])
        self.assertEqual(app.g.s.jog_step_mm(), 100.0)
        for k, mm in zip("1234", (1, 5, 10, 50)):
            app.handle_keys([k])
            self.assertEqual(app.g.s.jog_step_mm(), mm)

    def test_type_your_own_step(self):
        app, clk = homed()
        app.mode = "jog"
        app.handle_keys(["6"])
        self.assertIsNotNone(app.ask)
        for ch in "37.5":
            app.handle_keys([ch])
        app.handle_keys(["\n"])
        self.assertEqual(app.g.s.jog_step_mm(), 37.5)
        app.handle_keys(["5"])                                      # presets still work afterwards
        self.assertEqual(app.g.s.jog_step_mm(), 100.0)

    def test_own_step_rejects_nonsense(self):
        app, clk = homed()
        app.mode = "jog"
        before = app.g.s.jog_step_mm()
        for text in ("abc", "0", "-5", "99999", "nan"):
            app.handle_keys(["6"])
            for ch in text:
                app.handle_keys([ch])
            app.handle_keys(["\n"])
            self.assertEqual(app.g.s.jog_step_mm(), before, text)

    def test_nozzle_keys_are_gone(self):
        app, clk = homed()
        app.mode = "jog"
        log = len(app.g.link.log)
        app.handle_keys(["t"]); app.handle_keys(["z"])
        run(app, clk, 1)
        self.assertEqual([l for l in app.g.link.log[log:] if l.startswith("MOVE")], [])
        self.assertFalse(hasattr(app.g, "nozzle_down") or hasattr(app.g, "nozzle_up"))


if __name__ == "__main__":
    unittest.main()
