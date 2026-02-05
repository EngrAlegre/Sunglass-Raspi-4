from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional

from .config import HapticsConfig, HapticsPatternConfig


PatternName = str


@dataclass
class _ChannelState:
    pattern: PatternName = "off"
    output_high: bool = False
    next_toggle_at: float = 0.0


class HapticsController:
    def __init__(
        self,
        motor_gpios: Dict[str, int],
        patterns: HapticsConfig,
        gpio_module: Optional[object] = None,
    ) -> None:
        self._motor_gpios = dict(motor_gpios)
        self._patterns_cfg = patterns
        self._gpio = gpio_module
        self._state: Dict[str, _ChannelState] = {name: _ChannelState() for name in motor_gpios.keys()}

    def setup(self) -> None:
        if self._gpio is None:
            return
        self._gpio.setmode(self._gpio.BCM)
        for gpio in self._motor_gpios.values():
            self._gpio.setup(gpio, self._gpio.OUT, initial=self._gpio.LOW)

    def close(self) -> None:
        if self._gpio is None:
            return
        for gpio in self._motor_gpios.values():
            try:
                self._gpio.output(gpio, self._gpio.LOW)
            except Exception:
                pass

    def set_pattern(self, channel: str, pattern: PatternName) -> None:
        if channel not in self._state:
            return
        st = self._state[channel]
        if st.pattern == pattern:
            return
        st.pattern = pattern
        st.output_high = False
        st.next_toggle_at = 0.0
        if self._gpio is not None:
            gpio = self._motor_gpios[channel]
            self._gpio.output(gpio, self._gpio.LOW)

    def all_off(self) -> None:
        for ch in self._state.keys():
            self.set_pattern(ch, "off")

    def _resolve_pattern(self, name: PatternName) -> Optional[HapticsPatternConfig]:
        if name == "continuous":
            return self._patterns_cfg.continuous
        if name == "fast":
            return self._patterns_cfg.fast
        if name == "slow":
            return self._patterns_cfg.slow
        return None

    def update(self, now: Optional[float] = None) -> None:
        if self._gpio is None:
            return
        t = time.monotonic() if now is None else now
        for channel, st in self._state.items():
            gpio = self._motor_gpios[channel]
            if st.pattern == "off":
                if st.output_high:
                    st.output_high = False
                    self._gpio.output(gpio, self._gpio.LOW)
                continue

            cfg = self._resolve_pattern(st.pattern)
            if cfg is None:
                if st.output_high:
                    st.output_high = False
                    self._gpio.output(gpio, self._gpio.LOW)
                continue

            if st.pattern == "continuous":
                if not st.output_high:
                    st.output_high = True
                    self._gpio.output(gpio, self._gpio.HIGH)
                continue

            if st.next_toggle_at == 0.0:
                st.output_high = True
                self._gpio.output(gpio, self._gpio.HIGH)
                st.next_toggle_at = t + cfg.on_s
                continue

            if t < st.next_toggle_at:
                continue

            if st.output_high:
                st.output_high = False
                self._gpio.output(gpio, self._gpio.LOW)
                st.next_toggle_at = t + cfg.off_s
            else:
                st.output_high = True
                self._gpio.output(gpio, self._gpio.HIGH)
                st.next_toggle_at = t + cfg.on_s
