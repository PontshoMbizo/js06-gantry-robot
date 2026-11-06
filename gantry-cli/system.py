"""
Read-only probes of the Pi: services, AP clients, ssh sessions, uptime.

Everything here is best effort. A probe that cannot answer returns None and
the screen shows "?" rather than guessing. The network actions on the config
screen (go online, restart AP) only run the commands set in gantry.json
(net_online_cmd, net_ap_cmd, net_restart_cmd); nothing is run by default.
"""

import os
import re
import socket
import subprocess
import time
from dataclasses import dataclass


@dataclass
class Client:
    ip: str
    host: str
    signal: str     # "-48 dBm" or "?"
    connected: str  # "00:42:10" or "?"


def _run(cmd: list[str], timeout: float = 2.0) -> str | None:
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        return None
    return r.stdout if r.returncode == 0 else ""


class System:
    def hostname(self) -> str:
        return socket.gethostname()

    def uptime_s(self) -> float | None:
        try:
            with open("/proc/uptime", encoding="ascii") as f:
                return float(f.read().split()[0])
        except OSError:
            return None

    def service(self, name: str) -> bool | None:
        out = _run(["systemctl", "is-active", name])
        if out is None:
            return None
        return out.strip() == "active"

    def ssh_sessions(self) -> int | None:
        out = _run(["who"])
        if out is None:
            return None
        return sum(1 for line in out.splitlines() if re.search(r"\(\d+\.\d+\.\d+\.\d+\)", line))

    def this_ssh_ip(self) -> str | None:
        v = os.environ.get("SSH_CLIENT", "")
        return v.split()[0] if v else None

    def ap_clients(self, leases_path: str, iface: str) -> list[Client] | None:
        try:
            with open(leases_path, encoding="ascii") as f:
                leases = [line.split() for line in f if line.strip()]
        except OSError:
            return None
        stations = self._stations(iface)
        out = []
        for parts in leases:
            if len(parts) < 4:
                continue
            mac, ip, host = parts[1].lower(), parts[2], parts[3]
            st = stations.get(mac, {})
            out.append(Client(ip, host if host != "*" else "?", st.get("signal", "?"), st.get("connected", "?")))
        return out

    def _stations(self, iface: str) -> dict:
        out = _run(["iw", "dev", iface, "station", "dump"])
        stations, mac = {}, None
        for line in (out or "").splitlines():
            m = re.match(r"Station ([0-9a-f:]{17})", line)
            if m:
                mac = m.group(1)
                stations[mac] = {}
                continue
            if mac is None:
                continue
            m = re.search(r"signal:\s+(-?\d+)", line)
            if m:
                stations[mac]["signal"] = f"{m.group(1)} dBm"
            m = re.search(r"connected time:\s+(\d+)", line)
            if m:
                stations[mac]["connected"] = time.strftime("%H:%M:%S", time.gmtime(int(m.group(1))))
        return stations

    def ap_info(self, hostapd_conf: str, iface: str) -> dict:
        info: dict = {}
        try:
            with open(hostapd_conf, encoding="utf-8") as f:
                for line in f:
                    k, _, v = line.strip().partition("=")
                    if k in ("ssid", "channel", "hw_mode", "wpa", "rsn_pairwise", "wpa_key_mgmt"):
                        info[k] = v
        except OSError:
            pass
        out = _run(["ip", "-4", "-o", "addr", "show", iface])
        m = re.search(r"inet (\S+)", out or "")
        if m:
            info["address"] = m.group(1)
        ps = _run(["iw", "dev", iface, "get", "power_save"])
        if ps:
            info["power_save"] = ps.split(":")[-1].strip()
        return info

    def run_action(self, cmd: list[str]) -> tuple[bool, str]:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.SubprocessError) as e:
            return False, str(e)
        text = (r.stdout + r.stderr).strip().splitlines()
        return r.returncode == 0, text[-1] if text else ""


class SimSystem(System):
    """Values drawn in the design, for --sim on a laptop."""

    def __init__(self):
        self.t0 = time.monotonic() - 151

    def hostname(self) -> str:
        return "gantry-main"

    def uptime_s(self) -> float:
        return time.monotonic() - self.t0

    def service(self, name: str) -> bool:
        return True

    def ssh_sessions(self) -> int:
        return 1

    def this_ssh_ip(self) -> str:
        return "192.168.4.12"

    def ap_clients(self, leases_path: str, iface: str) -> list[Client]:
        return [Client("192.168.4.12", "fedora", "-48 dBm", "00:42:10")]

    def ap_info(self, hostapd_conf: str, iface: str) -> dict:
        return {"ssid": "gantry-net", "channel": "6", "hw_mode": "g", "wpa": "2",
                "rsn_pairwise": "CCMP", "wpa_key_mgmt": "WPA-PSK",
                "address": "192.168.4.1/24", "power_save": "off"}

    def run_action(self, cmd: list[str]) -> tuple[bool, str]:
        return True, "(sim) " + " ".join(cmd)
