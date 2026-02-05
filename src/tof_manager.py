from __future__ import annotations

import logging
import math
import time
from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Deque, Dict, Optional

from .config import PinsConfig, ToFConfig


logger = logging.getLogger(__name__)


@dataclass
class ToFReading:
    left_cm: Optional[float]
    center_cm: Optional[float]
    right_cm: Optional[float]


class ToFManager:
    def __init__(
        self,
        pins: PinsConfig,
        cfg: ToFConfig,
        simulation_mode: bool,
    ) -> None:
        self._pins = pins
        self._cfg = cfg
        self._simulation_mode = bool(simulation_mode)
        self._sensors: Dict[str, object] = {}
        self._buffers: Dict[str, Deque[float]] = {
            "left": deque(maxlen=int(cfg.median_window)),
            "center": deque(maxlen=int(cfg.median_window)),
            "right": deque(maxlen=int(cfg.median_window)),
        }
        self._last: Dict[str, Optional[float]] = {"left": None, "center": None, "right": None}
        self._start_t = time.monotonic()

    def setup(self) -> None:
        if self._simulation_mode:
            return
        try:
            import RPi.GPIO as GPIO
            import board
            import adafruit_vl53l1x
        except Exception as e:
            raise RuntimeError("VL53L1X requires RPi.GPIO + Adafruit Blinka + adafruit_vl53l1x") from e

        GPIO.setmode(GPIO.BCM)
        xshut_pins = {
            "left": int(self._pins.xshut_left_gpio),
            "center": int(self._pins.xshut_center_gpio),
            "right": int(self._pins.xshut_right_gpio),
        }
        for gpio in xshut_pins.values():
            GPIO.setup(gpio, GPIO.OUT, initial=GPIO.LOW)
        time.sleep(0.05)

        i2c = board.I2C()

        def _configure(sensor: object) -> None:
            mode = 2 if self._cfg.distance_mode == "long" else 1
            setattr(sensor, "distance_mode", mode)
            setattr(sensor, "timing_budget", int(self._cfg.timing_budget_ms))
            try:
                setattr(sensor, "inter_measurement", int(self._cfg.inter_measurement_ms))
            except Exception:
                pass
            sensor.start_ranging()

        sequence = [
            ("left", xshut_pins["left"], self._cfg.addresses["left"]),
            ("center", xshut_pins["center"], self._cfg.addresses["center"]),
            ("right", xshut_pins["right"], self._cfg.addresses["right"]),
        ]

        for name, gpio, addr in sequence:
            GPIO.output(gpio, GPIO.HIGH)
            time.sleep(0.05)
            sensor = adafruit_vl53l1x.VL53L1X(i2c)
            sensor.set_address(int(addr))
            _configure(sensor)
            self._sensors[name] = sensor

        logger.info(
            "VL53L1X sensors initialized: left=0x%02X center=0x%02X right=0x%02X",
            int(self._cfg.addresses["left"]),
            int(self._cfg.addresses["center"]),
            int(self._cfg.addresses["right"]),
        )

    def close(self) -> None:
        if self._simulation_mode:
            return
        try:
            import RPi.GPIO as GPIO
        except Exception:
            return
        for gpio in (self._pins.xshut_left_gpio, self._pins.xshut_center_gpio, self._pins.xshut_right_gpio):
            try:
                GPIO.output(int(gpio), GPIO.LOW)
            except Exception:
                pass

    def read(self) -> ToFReading:
        if self._simulation_mode:
            return self._read_simulated()
        d = {k: self._read_one(k) for k in ("left", "center", "right")}
        return ToFReading(left_cm=d["left"], center_cm=d["center"], right_cm=d["right"])

    def _read_simulated(self) -> ToFReading:
        t = time.monotonic() - self._start_t
        base = 110.0
        amp = 80.0
        left = base + amp * math.sin(t * 0.9 + 0.0)
        center = base + amp * math.sin(t * 0.9 + 1.4)
        right = base + amp * math.sin(t * 0.9 + 2.8)
        return ToFReading(left_cm=max(1.0, left), center_cm=max(1.0, center), right_cm=max(1.0, right))

    def _read_one(self, name: str) -> Optional[float]:
        sensor = self._sensors.get(name)
        if sensor is None:
            return None
        try:
            if not getattr(sensor, "data_ready"):
                return self._last[name]
            dist_cm = float(getattr(sensor, "distance"))
            sensor.clear_interrupt()
        except Exception:
            logger.exception("Failed reading VL53L1X sensor: %s", name)
            return self._last[name]

        buf = self._buffers[name]
        buf.append(dist_cm)
        dist_med = float(median(list(buf))) if buf else dist_cm
        self._last[name] = dist_med
        return dist_med
