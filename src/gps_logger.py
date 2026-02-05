from __future__ import annotations

import csv
import json
import logging
import os
import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial
import pynmea2

from .config import GPSConfig


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class GPSFix:
    timestamp_utc: str
    lat: float
    lon: float
    speed_m_s: Optional[float]


class GPSLogger:
    def __init__(self, cfg: GPSConfig, output_path: str = "logs/gps.csv") -> None:
        self._cfg = cfg
        self._output_path = output_path
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._mode: Optional[str] = None
        self._latest_fix: Optional[GPSFix] = None
        self._latest_fix_lock = threading.Lock()

    def get_latest_fix(self) -> Optional[GPSFix]:
        with self._latest_fix_lock:
            return self._latest_fix

    def start(self) -> None:
        if not self._cfg.enabled:
            return
        if self._thread is not None:
            return
        self._stop.clear()
        t = threading.Thread(target=self._run, name="gps-logger", daemon=True)
        self._thread = t
        t.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        self._stop.set()
        t = self._thread
        self._thread = None
        if t is not None:
            t.join(timeout=timeout_s)

    def _ensure_csv(self) -> None:
        out_dir = os.path.dirname(self._output_path) or "."
        os.makedirs(out_dir, exist_ok=True)
        if os.path.exists(self._output_path) and os.path.getsize(self._output_path) > 0:
            return
        with open(self._output_path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "lat", "lon", "speed_m_s"])

    def _append_fix(self, fix: GPSFix) -> None:
        self._ensure_csv()
        with open(self._output_path, "a", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow([fix.timestamp_utc, f"{fix.lat:.7f}", f"{fix.lon:.7f}", "" if fix.speed_m_s is None else f"{fix.speed_m_s:.3f}"])

    def _run(self) -> None:
        next_log = 0.0
        warned = False
        while not self._stop.is_set():
            now = time.monotonic()
            if now < next_log:
                self._stop.wait(timeout=min(0.25, next_log - now))
                continue
            next_log = now + float(self._cfg.log_interval_s)

            fix = self._get_fix()
            if fix is None:
                if not warned:
                    logger.warning("GPS unavailable; continuing without blocking")
                    warned = True
                continue
            warned = False
            with self._latest_fix_lock:
                self._latest_fix = fix
            try:
                self._append_fix(fix)
            except Exception:
                logger.exception("Failed to append GPS fix")

    def _get_fix(self) -> Optional[GPSFix]:
        fix = self._try_gpsd()
        if fix is not None:
            self._mode = "gpsd"
            return fix
        fix = self._try_serial()
        if fix is not None:
            self._mode = "serial"
            return fix
        self._mode = None
        return None

    def _try_gpsd(self) -> Optional[GPSFix]:
        sock: Optional[socket.socket] = None
        try:
            sock = socket.create_connection((self._cfg.gpsd_host, int(self._cfg.gpsd_port)), timeout=0.75)
            sock.settimeout(0.75)
            watch = '?WATCH={"enable":true,"json":true}\n'
            sock.sendall(watch.encode("utf-8"))
            deadline = time.monotonic() + 1.2
            buf = b""
            while time.monotonic() < deadline and not self._stop.is_set():
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode("utf-8", errors="ignore"))
                    except Exception:
                        continue
                    if not isinstance(msg, dict):
                        continue
                    if msg.get("class") != "TPV":
                        continue
                    lat = msg.get("lat")
                    lon = msg.get("lon")
                    t = msg.get("time")
                    if isinstance(lat, (int, float)) and isinstance(lon, (int, float)) and isinstance(t, str):
                        speed = msg.get("speed")
                        speed_m_s = float(speed) if isinstance(speed, (int, float)) else None
                        return GPSFix(timestamp_utc=t, lat=float(lat), lon=float(lon), speed_m_s=speed_m_s)
            return None
        except Exception:
            return None
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

    def _try_serial(self) -> Optional[GPSFix]:
        device = next((d for d in self._cfg.serial_device_candidates if os.path.exists(d)), None)
        if device is None:
            return None
        ser: Optional[serial.Serial] = None
        try:
            ser = serial.Serial(device, baudrate=int(self._cfg.serial_baud), timeout=0.75)
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline and not self._stop.is_set():
                try:
                    line = ser.readline().decode("ascii", errors="ignore").strip()
                except Exception:
                    continue
                if not line.startswith("$"):
                    continue
                try:
                    msg = pynmea2.parse(line)
                except Exception:
                    continue
                fix = self._fix_from_nmea(msg)
                if fix is not None:
                    return fix
            return None
        except Exception:
            return None
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

    def _fix_from_nmea(self, msg: object) -> Optional[GPSFix]:
        if isinstance(msg, pynmea2.types.talker.RMC):
            if msg.status != "A":
                return None
            if msg.datestamp is None or msg.timestamp is None:
                return None
            ts = f"{msg.datestamp.isoformat()}T{msg.timestamp.isoformat()}Z"
            try:
                lat = float(msg.latitude)
                lon = float(msg.longitude)
            except Exception:
                return None
            speed_m_s = None
            try:
                if msg.spd_over_grnd:
                    speed_m_s = float(msg.spd_over_grnd) * 0.514444
            except Exception:
                speed_m_s = None
            return GPSFix(timestamp_utc=ts, lat=lat, lon=lon, speed_m_s=speed_m_s)

        if isinstance(msg, pynmea2.types.talker.GGA):
            if msg.gps_qual is None or str(msg.gps_qual).strip() in ("0", ""):
                return None
            if msg.timestamp is None:
                return None
            ts = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
            try:
                lat = float(msg.latitude)
                lon = float(msg.longitude)
            except Exception:
                return None
            return GPSFix(timestamp_utc=ts, lat=lat, lon=lon, speed_m_s=None)
        return None
