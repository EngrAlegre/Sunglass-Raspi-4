from __future__ import annotations

import argparse
import json
import logging
import queue
import signal
import socket
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
from urllib import parse as urlparse
from urllib import request as urlrequest

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
    last_obstacle_tone_at: float = 0.0


def _internet_available(timeout_s: float = 0.75) -> bool:
    try:
        sock = socket.create_connection(("1.1.1.1", 53), timeout=timeout_s)
        try:
            sock.close()
        except Exception:
            pass
        return True
    except Exception:
        return False


def _http_get_json(url: str, headers: Optional[Dict[str, str]] = None, timeout_s: float = 2.5) -> Optional[dict]:
    try:
        req = urlrequest.Request(url, headers=headers or {})
        with urlrequest.urlopen(req, timeout=timeout_s) as resp:
            data = resp.read()
        obj = json.loads(data.decode("utf-8", errors="ignore"))
        return obj if isinstance(obj, dict) else None
    except Exception:
        return None


def _reverse_geocode_place(lat: float, lon: float) -> Optional[str]:
    qs = urlparse.urlencode({"format": "jsonv2", "lat": f"{lat:.7f}", "lon": f"{lon:.7f}"})
    url = f"https://nominatim.openstreetmap.org/reverse?{qs}"
    data = _http_get_json(url, headers={"User-Agent": "navigational-sunglasses/1.0"})
    if not data:
        return None
    address = data.get("address")
    if not isinstance(address, dict):
        address = {}
    road = address.get("road") or address.get("pedestrian") or address.get("footway")
    suburb = address.get("suburb") or address.get("neighbourhood") or address.get("quarter")
    city = address.get("city") or address.get("town") or address.get("village") or address.get("municipality")
    parts = [p for p in (road, suburb, city) if isinstance(p, str) and p.strip()]
    if parts:
        return ", ".join(parts[:3])
    display = data.get("display_name")
    if isinstance(display, str) and display.strip():
        return display.split(",")[0].strip()
    return None


def _weather_phrase(lat: float, lon: float) -> Optional[str]:
    qs = urlparse.urlencode(
        {
            "latitude": f"{lat:.7f}",
            "longitude": f"{lon:.7f}",
            "current": "temperature_2m,precipitation,wind_speed_10m",
            "timezone": "auto",
        }
    )
    url = f"https://api.open-meteo.com/v1/forecast?{qs}"
    data = _http_get_json(url, timeout_s=3.0)
    if not data:
        return None
    current = data.get("current")
    if not isinstance(current, dict):
        return None
    temp = current.get("temperature_2m")
    wind = current.get("wind_speed_10m")
    precip = current.get("precipitation")
    if not isinstance(temp, (int, float)):
        return None
    pieces = [f"Weather: {int(round(float(temp)))} degrees Celsius"]
    if isinstance(wind, (int, float)):
        pieces.append(f"wind {int(round(float(wind)))} kilometers per hour")
    if isinstance(precip, (int, float)) and float(precip) > 0:
        pieces.append(f"precipitation {float(precip):.1f} millimeters")
    return ", ".join(pieces) + "."


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
        speech_enabled=bool(cfg.audio.speech_enabled),
        speech_voice=str(cfg.audio.speech_voice),
        speech_rate_wpm=int(cfg.audio.speech_rate_wpm),
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
    playback_state = {"enabled": bool(cfg.audio.enabled), "source": str(cfg.audio.default_playback_source)}
    playback_lock = threading.Lock()

    announce_stop = threading.Event()
    announce_thread: Optional[threading.Thread] = None

    def _announcer_loop() -> None:
        next_weather = 0.0
        next_location = 0.0
        last_place: str = ""
        while not announce_stop.is_set():
            now = time.monotonic()

            with playback_lock:
                audio_is_enabled = bool(playback_state["enabled"])
                playback_source = str(playback_state["source"])

            if not (audio_is_enabled and cfg.audio.enabled and cfg.audio.speech_enabled):
                announce_stop.wait(timeout=0.5)
                continue
            if not (cfg.audio.announce_weather or cfg.audio.announce_location):
                announce_stop.wait(timeout=1.0)
                continue

            fix = gps_logger.get_latest_fix()
            if fix is None:
                announce_stop.wait(timeout=1.0)
                continue

            if not _internet_available():
                announce_stop.wait(timeout=5.0)
                continue

            phrases: list[str] = []
            if cfg.audio.announce_location and now >= next_location:
                place = _reverse_geocode_place(fix.lat, fix.lon)
                if isinstance(place, str) and place.strip():
                    if place.strip() != last_place:
                        phrases.append(f"Location: near {place.strip()}.")
                        last_place = place.strip()
                next_location = now + float(cfg.audio.location_interval_s)

            if cfg.audio.announce_weather and now >= next_weather:
                w = _weather_phrase(fix.lat, fix.lon)
                if isinstance(w, str) and w.strip():
                    phrases.append(w.strip())
                next_weather = now + float(cfg.audio.weather_interval_s)

            if phrases:
                stopped_playback = False
                if audio.is_playback_running():
                    audio.stop_playback()
                    stopped_playback = True
                audio.speak(" ".join(phrases), blocking=True)
                if stopped_playback and playback_source:
                    with playback_lock:
                        if bool(playback_state["enabled"]):
                            audio.start_playback(playback_source)

            announce_stop.wait(timeout=0.5)

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

    if cfg.audio.enabled and cfg.audio.speech_enabled and (cfg.audio.announce_weather or cfg.audio.announce_location):
        t = threading.Thread(target=_announcer_loop, name="online-announcer", daemon=True)
        announce_thread = t
        t.start()

    obstacles_enabled = True
    audio_enabled = bool(cfg.audio.enabled)
    sensitivity = get_default_sensitivity_name(cfg)
    thresholds = get_thresholds_for_sensitivity(cfg, sensitivity)

    channel_states: Dict[str, ChannelState] = {"left": ChannelState(), "center": ChannelState(), "right": ChannelState()}

    if once:
        deadline = time.monotonic() + 1.5
        reading = tof.read()
        while time.monotonic() < deadline:
            if reading.left_cm is not None and reading.center_cm is not None and reading.right_cm is not None:
                break
            time.sleep(0.05)
            reading = tof.read()
        logger.info(
            "Distances (cm): left=%s center=%s right=%s",
            reading.left_cm,
            reading.center_cm,
            reading.right_cm,
        )
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
                    with playback_lock:
                        playback_state["enabled"] = audio_enabled
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
                audio_alert_candidates: list[str] = []
                for ch, z in zones.items():
                    prev_zone = channel_states[ch].zone
                    channel_states[ch].zone = z
                    haptics.set_pattern(ch, _pattern_for_zone(z))
                    if (
                        audio_enabled
                        and cfg.audio.obstacle_audio_alerts
                        and z == 3
                        and prev_zone != 3
                        and (now - channel_states[ch].last_obstacle_tone_at) >= float(cfg.audio.obstacle_audio_min_interval_s)
                    ):
                        audio_alert_candidates.append(ch)

                if audio_alert_candidates:
                    priority = {"center": 0, "left": 1, "right": 2}
                    ch = sorted(audio_alert_candidates, key=lambda x: priority.get(x, 99))[0]
                    freq_map = {"left": 1200, "center": 1600, "right": 2000}
                    audio.play_tone(freq_map.get(ch, 1500), 0.06)
                    channel_states[ch].last_obstacle_tone_at = now
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
        announce_stop.set()
        if announce_thread is not None:
            try:
                announce_thread.join(timeout=2.0)
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
