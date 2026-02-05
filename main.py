from __future__ import annotations

import argparse
import logging
import signal
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from src.audio import Audio
from src.buttons import Buttons
from src.config import AppConfig, get_default_sensitivity_name, get_thresholds_for_sensitivity, load_config
from src.gps_logger import GPSLogger
from src.haptics import HapticsController
from src.logging_setup import setup_logging
from src.tof_manager import ToFManager


logger = logging.getLogger(__name__)


@dataclass
class ChannelState:
    zone: int = 0


def _load_gpio(simulation_mode: bool) -> Optional[object]:
    if simulation_mode:
        return None
    try:
        import RPi.GPIO as GPIO

        return GPIO
    except Exception:
        return None


def _zone_from_distance(
    distance_cm: Optional[float],
    thresholds: Tuple[int, int, int],
    hysteresis_cm: int,
    prev_zone: int,
) -> int:
    if distance_cm is None:
        return 0
    d = float(distance_cm)
    t0, t1, t2 = thresholds
    h = float(hysteresis_cm)

    if prev_zone == 3 and d <= (t0 + h):
        return 3
    if prev_zone == 2 and d <= (t1 + h):
        return 2
    if prev_zone == 1 and d <= (t2 + h):
        return 1

    if d <= t0:
        return 3
    if d <= t1:
        return 2
    if d <= t2:
        return 1
    return 0


def _pattern_for_zone(zone: int) -> str:
    if zone == 3:
        return "continuous"
    if zone == 2:
        return "fast"
    if zone == 1:
        return "slow"
    return "off"


def _cycle_sensitivity(current: str, config: AppConfig) -> str:
    preferred = ["near", "normal", "far"]
    names = [n for n in preferred if n in config.sensitivity_presets]
    remaining = sorted([n for n in config.sensitivity_presets.keys() if n not in names])
    order = names + remaining
    if not order:
        return current
    if current not in order:
        return order[0]
    i = order.index(current)
    return order[(i + 1) % len(order)]


def run(config_path: str, once: bool) -> int:
    cfg = load_config(config_path)
    setup_logging(cfg.logging)

    gpio = _load_gpio(cfg.simulation_mode)
    if not cfg.simulation_mode and gpio is None:
        logger.warning("RPi.GPIO unavailable; forcing simulation_mode=true")
        cfg = AppConfig(
            simulation_mode=True,
            loop_hz=cfg.loop_hz,
            pins=cfg.pins,
            tof=cfg.tof,
            sensitivity_presets=cfg.sensitivity_presets,
            haptics=cfg.haptics,
            audio=cfg.audio,
            gps=cfg.gps,
            logging=cfg.logging,
        )

    tof = ToFManager(cfg.pins, cfg.tof, simulation_mode=cfg.simulation_mode)
    audio = Audio(
        enabled=bool(cfg.audio.enabled),
        tone_volume=float(cfg.audio.tone_volume),
        playback_backend_preference=list(cfg.audio.playback_backend_preference),
    )
    haptics = HapticsController(
        motor_gpios={
            "left": cfg.pins.motor_left_gpio,
            "center": cfg.pins.motor_center_gpio,
            "right": cfg.pins.motor_right_gpio,
        },
        patterns=cfg.haptics,
        gpio_module=gpio,
    )
    buttons = Buttons(
        button_gpios={
            "button1": cfg.pins.button1_gpio,
            "button2": cfg.pins.button2_gpio,
            "button3": cfg.pins.button3_gpio,
        },
        debounce_s=0.05,
        gpio_module=gpio,
    )
    gps_logger = GPSLogger(cfg.gps, output_path="logs/gps.csv")

    stop = {"flag": False}

    def _handle_signal(signum: int, _frame: object) -> None:
        logger.info("Signal received: %s", signum)
        stop["flag"] = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    tof.setup()
    haptics.setup()
    buttons.setup()
    gps_logger.start()

    obstacles_enabled = True
    audio_enabled = bool(cfg.audio.enabled)
    sensitivity = get_default_sensitivity_name(cfg)
    thresholds = get_thresholds_for_sensitivity(cfg, sensitivity)

    channel_states: Dict[str, ChannelState] = {"left": ChannelState(), "center": ChannelState(), "right": ChannelState()}

    if once:
        reading = tof.read()
        logger.info("Distances (cm): left=%s center=%s right=%s", reading.left_cm, reading.center_cm, reading.right_cm)
        stop["flag"] = True

    target_dt = 1.0 / float(cfg.loop_hz)
    last_tick = time.monotonic()

    try:
        while not stop["flag"]:
            now = time.monotonic()
            last_tick = now

            for ev in buttons.poll():
                if not ev.pressed:
                    continue
                if ev.name == "button1":
                    obstacles_enabled = not obstacles_enabled
                    audio.play_mode_tone("obstacles_on" if obstacles_enabled else "obstacles_off")
                    if not obstacles_enabled:
                        haptics.all_off()
                elif ev.name == "button2":
                    audio_enabled = not audio_enabled
                    audio.play_mode_tone("audio_on" if audio_enabled else "audio_off")
                    if audio_enabled:
                        if cfg.audio.default_playback_source:
                            audio.start_playback(cfg.audio.default_playback_source)
                    else:
                        audio.stop_playback()
                elif ev.name == "button3":
                    sensitivity = _cycle_sensitivity(sensitivity, cfg)
                    thresholds = get_thresholds_for_sensitivity(cfg, sensitivity)
                    audio.play_mode_tone(f"sensitivity_{sensitivity}")

            reading = tof.read()

            if obstacles_enabled:
                zones = {
                    "left": _zone_from_distance(reading.left_cm, thresholds, cfg.tof.hysteresis_cm, channel_states["left"].zone),
                    "center": _zone_from_distance(
                        reading.center_cm, thresholds, cfg.tof.hysteresis_cm, channel_states["center"].zone
                    ),
                    "right": _zone_from_distance(
                        reading.right_cm, thresholds, cfg.tof.hysteresis_cm, channel_states["right"].zone
                    ),
                }
                for ch, z in zones.items():
                    channel_states[ch].zone = z
                    haptics.set_pattern(ch, _pattern_for_zone(z))
            else:
                haptics.all_off()

            haptics.update(now=now)

            sleep_s = max(0.0, target_dt - (time.monotonic() - now))
            if sleep_s > 0:
                time.sleep(sleep_s)

    finally:
        try:
            haptics.all_off()
            haptics.update()
        except Exception:
            pass
        try:
            audio.close()
        except Exception:
            pass
        try:
            gps_logger.stop()
        except Exception:
            pass
        try:
            tof.close()
        except Exception:
            pass
        if gpio is not None:
            try:
                gpio.cleanup()
            except Exception:
                pass

    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="navigational-sunglasses")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args(argv)
    return run(args.config, args.once)


if __name__ == "__main__":
    raise SystemExit(main())
