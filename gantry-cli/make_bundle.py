"""
Build gantry-cli.zip for a Pi with no internet.

Run on a laptop that has internet:  python3 make_bundle.py
It downloads pyserial (pure Python, so the same wheel works on the Pi Zero's
ARMv6) into vendor/ inside the zip; gantry.py puts vendor/ on sys.path.
Everything else is the Python standard library, which Raspberry Pi OS ships.
"""

import os
import subprocess
import sys
import tempfile
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "gantry-cli.zip")
TOP = "gantry-cli"
PYSERIAL = "pyserial==3.5"
SKIP_DIRS = {"__pycache__", "logs", "vendor", ".venv"}
SKIP_FILES = {"gantry.json"}      # personal settings stay on the machine that made them


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([sys.executable, "-m", "pip", "download", "--quiet", "--no-deps",
                        "--only-binary", ":all:", "--dest", tmp, PYSERIAL], check=True)
        wheel = next(os.path.join(tmp, f) for f in os.listdir(tmp) if f.endswith(".whl"))
        with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
            for root, dirs, files in os.walk(HERE):
                dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS and not d.startswith("."))
                for f in sorted(files):
                    if f in SKIP_FILES or f.endswith(".pyc"):
                        continue
                    path = os.path.join(root, f)
                    z.write(path, os.path.join(TOP, os.path.relpath(path, HERE)))
            with zipfile.ZipFile(wheel) as w:
                for name in w.namelist():
                    z.writestr(os.path.join(TOP, "vendor", name), w.read(name))
    print(f"wrote {OUT} ({os.path.getsize(OUT) // 1024} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
