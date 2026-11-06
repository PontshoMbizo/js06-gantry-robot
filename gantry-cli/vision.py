"""
Camera capture and weed detection, behind a small interface so the CLI does
not care where frames come from.

  submit(frame, x, y)  start capture + inference for one scan stop
  poll()               finished frames: [(frame, detections, infer_s, boxes, max_conf)]
  busy()               any frame still being processed

PiVision is the real one. It reports the camera that camera.py found (shown on
the status bar and the boot screen), but detection is NOT wired yet: the code
lives in ../../computer vision/src (segment → classify → pixel_to_gantry) and
still works on a stitched bed image with a placeholder calibration. Until a
per-frame capture + detect function exists, scan only runs with --dry.

SimVision returns the bed file's sim_weeds that fall inside each frame.
"""

import time
from dataclasses import dataclass


@dataclass
class Detection:
    x: float
    y: float
    conf: float


class NotWired(RuntimeError):
    pass


class PiVision:
    model = None        # e.g. "weednet-s.tflite"
    infer_s = None
    detects = False     # scan with detection needs this

    def __init__(self, cam=None):
        self.cam = cam

    @property
    def camera(self):
        """The camera's name once camera.py has found one, else None."""
        c = self.cam
        return c.name if c is not None and c.state == "ok" else None

    def submit(self, frame: int, x: float, y: float, fw: float, fh: float, thresh: float) -> None:
        raise NotWired("vision not wired yet — see vision.py (PiVision)")

    def poll(self) -> list:
        return []

    def busy(self) -> bool:
        return False


class SimVision:
    detects = True

    def __init__(self, bed, settings, clock=time.monotonic, speed: float = 1.0):
        self.bed = bed
        self.camera = settings.camera
        self.model = settings.model
        self.infer_s = 1.8
        self.clock = clock
        self.speed = speed
        self.pending: list[tuple[float, tuple]] = []

    def submit(self, frame: int, x: float, y: float, fw: float, fh: float, thresh: float) -> None:
        found = [
            Detection(float(w["x"]), float(w["y"]), float(w["conf"]))
            for w in self.bed.sim_weeds
            if abs(w["x"] - x) <= fw / 2 and abs(w["y"] - y) <= fh / 2
        ]
        boxes = len(found)
        kept = [d for d in found if d.conf >= thresh]
        max_conf = max((d.conf for d in found), default=0.0)
        self.pending.append((self.clock() + self.infer_s / self.speed,
                             (frame, kept, self.infer_s, boxes, max_conf)))

    def poll(self) -> list:
        now = self.clock()
        done = [r for t, r in self.pending if t <= now]
        self.pending = [(t, r) for t, r in self.pending if t > now]
        return done

    def busy(self) -> bool:
        return bool(self.pending)
