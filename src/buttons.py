from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass(frozen=True)
class ButtonEvent:
    name: str
    gpio: int
    pressed: bool
    timestamp: float


class Buttons:
    def __init__(
        self,
        button_gpios: Dict[str, int],
        debounce_s: float = 0.05,
        gpio_module: Optional[object] = None,
    ) -> None:
        self._button_gpios = dict(button_gpios)
        self._debounce_s = float(debounce_s)
        self._gpio = gpio_module
        self._last_raw: Dict[str, int] = {}
        self._last_raw_change: Dict[str, float] = {}
        self._stable: Dict[str, int] = {}

    def setup(self) -> None:
        if self._gpio is None:
            return
        self._gpio.setmode(self._gpio.BCM)
        for name, gpio in self._button_gpios.items():
            self._gpio.setup(gpio, self._gpio.IN, pull_up_down=self._gpio.PUD_UP)
            raw = int(self._gpio.input(gpio))
            now = time.monotonic()
            self._last_raw[name] = raw
            self._stable[name] = raw
            self._last_raw_change[name] = now

    def close(self) -> None:
        return

    def poll(self) -> List[ButtonEvent]:
        if self._gpio is None:
            return []

        events: List[ButtonEvent] = []
        now = time.monotonic()
        for name, gpio in self._button_gpios.items():
            raw = int(self._gpio.input(gpio))
            if name not in self._last_raw:
                self._last_raw[name] = raw
                self._stable[name] = raw
                self._last_raw_change[name] = now
                continue

            if raw != self._last_raw[name]:
                self._last_raw[name] = raw
                self._last_raw_change[name] = now
                continue

            if (now - self._last_raw_change[name]) < self._debounce_s:
                continue

            stable = self._stable[name]
            if raw != stable:
                self._stable[name] = raw
                pressed = raw == 0
                events.append(ButtonEvent(name=name, gpio=gpio, pressed=pressed, timestamp=now))
        return events

    def is_pressed(self, name: str) -> bool:
        stable = self._stable.get(name, 1)
        return stable == 0
