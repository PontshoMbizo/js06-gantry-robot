"""
The Pi camera: find it, show it on the screen, and take a photo with the gantry position in the file name.

Detection, in this order (camera_source = "auto" in gantry.json):
  rpicam-still / libcamera-still --list-cameras   ribbon-cable Pi cameras (Raspberry Pi OS Bullseye and later)
  raspistill + vcgencmd get_camera                the legacy camera stack (older OS images)
  a USB video device + fswebcam or ffmpeg          USB webcams
camera_source = "none" turns it off.

Both detection and capture run in a background thread: on a Pi Zero a still takes a few seconds and
the screen must keep updating.
"""

import glob
import os
import re
import shutil
import subprocess
import threading
import time


class Camera:
    def __init__(self, settings, run=subprocess.run, which=shutil.which, video_glob="/sys/class/video4linux/video*"):
        self.s = settings
        self._run, self._which, self._video_glob = run, which, video_glob
        self.state = "checking"         # checking | ok | none | off
        self.name = ""                  # e.g. "imx219" or "USB: HD Webcam"
        self.kind = ""                  # rpicam | legacy | usb
        self.tool = ""
        self.device = ""
        self.error = ""
        self.busy = False
        self.last_photo = ""
        self.last_error = ""
        self.done_count = 0             # bumps when a capture finishes (the screen shows the result)

    # --- detection ---------------------------------------------------------
    def start(self) -> "Camera":
        if self.s.camera_source == "none":
            self.state, self.error = "off", "camera_source is none in gantry.json"
            return self
        threading.Thread(target=self.detect, daemon=True).start()
        return self

    def detect(self) -> None:
        want = self.s.camera_source
        notes = []
        try:
            if want in ("auto", "rpicam"):
                for tool in ("rpicam-still", "libcamera-still"):
                    if not self._which(tool):
                        continue
                    r = self._run([tool, "--list-cameras"], capture_output=True, text=True, timeout=20)
                    out = (r.stdout or "") + (r.stderr or "")
                    m = re.search(r"^\s*0\s*:\s*(\S+)", out, re.M)
                    if m:
                        return self._found("rpicam", m.group(1), tool)
                    notes.append(f"{tool}: no camera (ribbon cable seated? both ends, contacts the right way?)")
                    break
                else:
                    notes.append("rpicam-still not installed (sudo apt install rpicam-apps)")
            if want in ("auto", "legacy") and self._which("raspistill") and self._which("vcgencmd"):
                r = self._run(["vcgencmd", "get_camera"], capture_output=True, text=True, timeout=10)
                if "detected=1" in (r.stdout or ""):
                    return self._found("legacy", "Pi camera (legacy stack)", "raspistill")
                notes.append("raspistill: no camera detected")
            if want in ("auto", "usb"):
                dev, name = self._usb_video()
                if dev:
                    tool = "fswebcam" if self._which("fswebcam") else ("ffmpeg" if self._which("ffmpeg") else "")
                    if not tool:
                        notes.append(f"USB camera {name} on {dev}, but no capture tool (sudo apt install fswebcam)")
                    else:
                        self.device = dev
                        return self._found("usb", f"USB: {name}", tool)
        except Exception as e:                       # a broken tool must never take the UI down
            notes.append(str(e))
        self.state = "none"
        self.error = "; ".join(notes) or "no camera found"

    def _found(self, kind: str, name: str, tool: str) -> None:
        self.kind, self.name, self.tool, self.state, self.error = kind, name, tool, "ok", ""

    def _usb_video(self) -> tuple:
        """The first video device that sits on USB (the Pi's own codec/ISP devices are not cameras)."""
        for d in sorted(glob.glob(self._video_glob)):
            try:
                if "/usb" not in os.path.realpath(os.path.join(d, "device")):
                    continue
                with open(os.path.join(d, "name")) as f:
                    name = f.read().strip()
                idx_file = os.path.join(d, "index")
                if os.path.exists(idx_file):
                    with open(idx_file) as f:
                        if f.read().strip() not in ("", "0"):
                            continue                 # a webcam's second node is metadata, not frames
                return "/dev/" + os.path.basename(d), name
            except OSError:
                continue
        return "", ""

    # --- capture -------------------------------------------------------------
    def photo_path(self, pos: dict) -> str:
        folder = self.s.photo_dir
        os.makedirs(folder, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(folder, f"photo_{stamp}_X{pos['X']:.0f}_Y{pos['Y']:.0f}_Z{pos['Z']:.0f}.jpg")

    def command(self, path: str) -> list:
        w, h = self.s.photo_width, self.s.photo_height
        if self.kind == "rpicam":
            cmd = [self.tool, "-n", "-t", str(self.s.photo_settle_ms), "-o", path]
            if w and h:
                cmd += ["--width", str(w), "--height", str(h)]
            return cmd
        if self.kind == "legacy":
            cmd = ["raspistill", "-n", "-t", str(self.s.photo_settle_ms), "-o", path]
            if w and h:
                cmd += ["-w", str(w), "-h", str(h)]
            return cmd
        if self.tool == "fswebcam":
            cmd = ["fswebcam", "-q", "-d", self.device, "--no-banner", "-S", "10"]
            if w and h:
                cmd += ["-r", f"{w}x{h}"]
            return cmd + [path]
        cmd = ["ffmpeg", "-loglevel", "error", "-y", "-f", "v4l2"]
        if w and h:
            cmd += ["-video_size", f"{w}x{h}"]
        return cmd + ["-i", self.device, "-frames:v", "1", path]

    def capture(self, pos: dict) -> str | None:
        """Start a photo; returns why not, or None. The result shows up in last_photo / last_error."""
        if self.state != "ok":
            return f"no camera: {self.error or self.state}"
        if self.busy:
            return "already taking a photo"
        path = self.photo_path(pos)
        self.busy = True
        threading.Thread(target=self._capture, args=(path,), daemon=True).start()
        return None

    def _capture(self, path: str) -> None:
        try:
            r = self._run(self.command(path), capture_output=True, text=True, timeout=60)
            if r.returncode == 0 and os.path.exists(path) and os.path.getsize(path) > 0:
                self.last_photo, self.last_error = path, ""
            else:
                msg = ((r.stderr or "") + (r.stdout or "")).strip().splitlines()
                self.last_error = (msg[-1] if msg else f"{self.tool} exited with {r.returncode}")[:120]
        except Exception as e:
            self.last_error = str(e)[:120]
        finally:
            self.busy = False
            self.done_count += 1

    def status_line(self) -> tuple:
        """(text, style) for the screen."""
        if self.state == "checking":
            return "looking for a camera …", "d"
        if self.state == "off":
            return "off (camera_source = none in gantry.json)", "d"
        if self.state == "none":
            return f"not found: {self.error}", "y"
        if self.busy:
            return f"{self.name} · taking a photo …", "c"
        w, h = self.s.photo_width, self.s.photo_height
        size = f"{w}×{h}" if w and h else "full size"
        return f"{self.name} via {self.tool} · {size}", "g"
