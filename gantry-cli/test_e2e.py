"""
End to end: the Gantry CLI (real SerialLink, Forwarder and FileSwitches, real clock) against the
firmware itself (firmware/sim/fwsim: gantry_fw.ino compiled for the PC with a model of the gantry)
over a pseudo-terminal. Build it first:  firmware/sim/build.sh
Then:  python3 -m unittest -v test_e2e
"""

import csv
import os
import subprocess
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "vendor"))

from app import App                                    # noqa: E402
from controller import Gantry                          # noqa: E402
from link import SerialLink                            # noqa: E402
from model import Bed, Settings                        # noqa: E402
from switches import Forwarder, open_switches          # noqa: E402
from system import SimSystem                           # noqa: E402
from vision import PiVision                            # noqa: E402

FWSIM = os.path.join(HERE, "firmware", "sim", "fwsim")
SPEED = 4.0          # the simulated gantry runs this many times faster than the wall clock


@unittest.skipUnless(os.path.exists(FWSIM), "build the firmware simulator first: firmware/sim/build.sh")
class EndToEnd(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        swfile = os.path.join(cls.tmp, "sw.json")
        cls.fw = subprocess.Popen([FWSIM, "pty", "400", swfile, str(SPEED), "0.01"], stdout=subprocess.PIPE, text=True)
        port = cls.fw.stdout.readline().strip()
        time.sleep(0.2)
        s = Settings()
        s.log_dir, s.sysid_dir, s.calib_dir = cls.tmp, os.path.join(cls.tmp, "sysid"), os.path.join(cls.tmp, "cal")
        s.config_path = os.path.join(cls.tmp, "gantry.json")
        s.switches_connected, s.switch_backend = True, "file:" + swfile
        # the simulated gantry runs SPEED x faster: scale the Pi's own timing to match
        s.keepalive, s.reply_timeout, s.heartbeat = 0.5 / SPEED, 1.0, 1.0
        cls.s = s
        bed = Bed.load(s.bed_file, view=s.map_view)
        cls.link = SerialLink(port, 115200)
        cls.sw = open_switches(s)
        cls.fwd = Forwarder(cls.link, cls.sw, True, every_s=0.05 / SPEED / 2).start()
        cls.g = Gantry(s, bed, cls.link, PiVision(), SimSystem(), cls.sw)
        cls.app = App(cls.g)

    @classmethod
    def tearDownClass(cls):
        cls.fwd.stop()
        cls.g.close()
        cls.fw.kill()
        cls.fw.wait()
        cls.fw.stdout.close()

    def run_until(self, secs, until=None):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.app.tick()
            if until and until():
                return True
            time.sleep(0.01)
        return False

    def idle(self, secs=60):
        g = self.g
        return self.run_until(secs, lambda: not g.pending and not g.queue and (g.proc is None or g.proc.done))

    def test_1_boot_and_home(self):
        app, g = self.app, self.g
        self.assertTrue(self.run_until(15, lambda: app.boot_phase == "prompt"), app.boot_checks)
        self.assertEqual(app.boot_checks["esp32"][0], "ok")
        self.assertEqual(app.boot_checks["config"][0], "ok", g.notice)
        self.assertTrue(g.fw.startswith("2."), g.fw)
        self.assertEqual(app.boot_checks["switches"][0], "ok", app.boot_checks["switches"])
        app.handle_keys(["y"])
        self.assertTrue(self.run_until(90, lambda: app.mode == "dash"), g.notice)
        self.assertTrue(all(a.homed for a in g.axes.values()), g.notice)
        for a in "XYZ":
            self.assertAlmostEqual(g.axes[a].pos, 0.0, delta=1.2)

    def test_2_goto_spray(self):
        g = self.g
        self.assertIsNone(g.goto(300, 500))
        self.assertTrue(self.idle(60))
        self.assertAlmostEqual(g.axes["X"].pos, 300, delta=1.2)
        self.assertAlmostEqual(g.axes["Y"].pos, 500, delta=1.2)
        self.assertIsNone(g.spray_here())
        self.assertTrue(self.idle(10))
        self.assertTrue(any(l.text.startswith("OK SPRAY") for l in g.log))

    def test_3_step_test_raw_csv(self):
        g = self.g
        self.assertIsNone(g.step_test("Y", 6.0, -1, 60, 2, True, 15))
        self.assertTrue(self.idle(90))
        r = g.step_results[-2:]
        self.assertEqual([x["file"] for x in r], ["Y_6.0_neg_test.csv", "Y_6.0_pos_test.csv"])
        self.assertEqual([x["n"] for x in r], [60, 60])
        with open(os.path.join(self.s.sysid_dir, "Y_6.0_neg_test.csv")) as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], ["edge", "t_us"])
        self.assertGreaterEqual(len(rows) - 1, 60)               # every raw edge, the noise ones too
        self.assertEqual(len(rows) - 1, r[0]["edges"])

    def test_3b_sensor_check(self):
        g = self.g
        self.assertIsNone(g.sensor_check("ZXY"))
        self.assertTrue(self.idle(60), g.notice)
        res = g.sensor_check_results
        self.assertEqual(sorted(res), ["X", "Y", "Z"])
        for a, (status, text) in res.items():        # the simulator injects sensor noise: "ok" or "ok, but noisy"
            self.assertTrue(status == "ok" or (status == "warn" and "noisy" in text), (a, status, text))

    def test_4_span(self):
        g = self.g
        self.assertIsNone(g.span_test("X", 1))
        self.assertTrue(self.idle(200), g.notice)
        sr = g.span_result
        self.assertIsNotNone(sr, g.notice)
        self.assertEqual(len(sr["spans"]), 2)
        self.assertLessEqual(sr["spread"], 2)
        self.assertAlmostEqual(sr["nominal_mm"], 620, delta=4)
        self.assertAlmostEqual(g.axes["X"].pos, 5.0, delta=1.2)    # parked

    def test_5_estop_and_reset(self):
        g, app = self.g, self.app
        self.assertIsNone(g.goto(300, 800))
        self.run_until(0.5)
        app.handle_keys([" "])
        self.run_until(2, lambda: g.estop_confirmed)
        self.assertTrue(g.estop and g.estop_confirmed)
        self.assertIsNotNone(g.jog("Y", 10))                      # refused on the Pi
        g.send("JOG Y+10")                                         # and on the ESP
        self.run_until(2, lambda: not g.pending and not g.queue)
        self.assertTrue(any("ERR ESTOP latched" in l.text for l in g.log))
        self.assertIsNone(g.reset())
        self.assertTrue(self.run_until(3, lambda: not g.estop))
        self.assertIsNone(g.jog("Y", 10))
        self.assertTrue(self.idle(20))

    def test_6_no_unexpected_errors(self):
        bad = [l.text for l in self.g.log if l.mark in ("!", "×") and "ESTOP latched" not in l.text]
        self.assertEqual(bad, [])


if __name__ == "__main__":
    unittest.main()
