from __future__ import annotations

import dataclasses
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Literal, Optional, Tuple

import yaml


@dataclass(frozen=True)
class PinsConfig:
    i2c_sda_gpio: int = 2
    i2c_scl_gpio: int = 3

    xshut_left_gpio: int = 5
    xshut_center_gpio: int = 6
    xshut_right_gpio: int = 13

    motor_left_gpio: int = 17
    motor_center_gpio: int = 27
    motor_right_gpio: int = 22

    button1_gpio: int = 23
    button2_gpio: int = 24
    button3_gpio: int = 25

    gps_uart_rx_gpio: int = 15
    gps_uart_tx_gpio: int = 14


DistanceMode = Literal["short", "long"]


@dataclass(frozen=True)
class ToFConfig:
    i2c_bus: int = 1
    addresses: Dict[str, int] = dataclasses.field(
        default_factory=lambda: {"left": 0x30, "center": 0x31, "right": 0x32}
    )
    distance_mode: DistanceMode = "long"
    timing_budget_ms: int = 50
    inter_measurement_ms: int = 60
    median_window: int = 5
    hysteresis_cm: int = 4


@dataclass(frozen=True)
class HapticsPatternConfig:
    on_s: float
    off_s: float


@dataclass(frozen=True)
class HapticsConfig:
    continuous: HapticsPatternConfig = HapticsPatternConfig(on_s=1.0, off_s=0.0)
    fast: HapticsPatternConfig = HapticsPatternConfig(on_s=0.08, off_s=0.12)
    slow: HapticsPatternConfig = HapticsPatternConfig(on_s=0.20, off_s=0.40)


@dataclass(frozen=True)
class AudioConfig:
    enabled: bool = True
    default_playback_source: str = ""
    playback_backend_preference: List[str] = dataclasses.field(
        default_factory=lambda: ["cvlc", "mpg123"]
    )
    tone_volume: float = 0.35


@dataclass(frozen=True)
class GPSConfig:
    enabled: bool = True
    log_interval_s: int = 5
    gpsd_host: str = "127.0.0.1"
    gpsd_port: int = 2947
    serial_device_candidates: List[str] = dataclasses.field(
        default_factory=lambda: ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyUSB0"]
    )
    serial_baud: int = 9600


@dataclass(frozen=True)
class LoggingConfig:
    level: str = "INFO"
    file: str = "logs/app.log"
    max_bytes: int = 1_000_000
    backup_count: int = 5


@dataclass(frozen=True)
class AppConfig:
    simulation_mode: bool = False
    loop_hz: float = 8.0
    pins: PinsConfig = PinsConfig()
    tof: ToFConfig = ToFConfig()
    sensitivity_presets: Dict[str, Dict[str, List[int]]] = dataclasses.field(
        default_factory=lambda: {
            "near": {"thresholds_cm": [15, 35, 70]},
            "normal": {"thresholds_cm": [20, 50, 100]},
            "far": {"thresholds_cm": [30, 70, 140]},
        }
    )
    haptics: HapticsConfig = HapticsConfig()
    audio: AudioConfig = AudioConfig()
    gps: GPSConfig = GPSConfig()
    logging: LoggingConfig = LoggingConfig()


_HARD_CONSTRAINT_PINS = PinsConfig()


def _ensure_hard_pin_constraints(pins: PinsConfig) -> None:
    if pins != _HARD_CONSTRAINT_PINS:
        expected = dataclasses.asdict(_HARD_CONSTRAINT_PINS)
        got = dataclasses.asdict(pins)
        diffs = {k: (expected[k], got[k]) for k in expected if expected[k] != got[k]}
        raise ValueError(
            "Pin constraints violated; these pins are fixed by the wiring guide: "
            + ", ".join([f"{k} expected {v[0]} got {v[1]}" for k, v in diffs.items()])
        )


def _require_int(value: Any, key_path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{key_path} must be an integer")
    return value


def _require_float(value: Any, key_path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key_path} must be a number")
    return float(value)


def _require_bool(value: Any, key_path: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{key_path} must be a boolean")
    return value


def _require_str(value: Any, key_path: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{key_path} must be a string")
    return value


def _coerce_hex_or_int(value: Any, key_path: str) -> int:
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if isinstance(value, str):
        s = value.strip().lower()
        base = 16 if s.startswith("0x") else 10
        return int(s, base)
    raise ValueError(f"{key_path} must be an int or a hex string like 0x30")


def _deep_get(d: Dict[str, Any], *keys: str) -> Any:
    cur: Any = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def load_config(path: str = "config.yaml") -> AppConfig:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ValueError("config.yaml must be a mapping")

    simulation_mode = _require_bool(raw.get("simulation_mode", False), "simulation_mode")
    loop_hz = _require_float(raw.get("loop_hz", 8), "loop_hz")
    if loop_hz <= 0:
        raise ValueError("loop_hz must be > 0")

    pins_raw = raw.get("pins", {})
    if not isinstance(pins_raw, dict):
        raise ValueError("pins must be a mapping")
    pins = PinsConfig(
        i2c_sda_gpio=_require_int(pins_raw.get("i2c_sda_gpio", 2), "pins.i2c_sda_gpio"),
        i2c_scl_gpio=_require_int(pins_raw.get("i2c_scl_gpio", 3), "pins.i2c_scl_gpio"),
        xshut_left_gpio=_require_int(pins_raw.get("xshut_left_gpio", 5), "pins.xshut_left_gpio"),
        xshut_center_gpio=_require_int(
            pins_raw.get("xshut_center_gpio", 6), "pins.xshut_center_gpio"
        ),
        xshut_right_gpio=_require_int(
            pins_raw.get("xshut_right_gpio", 13), "pins.xshut_right_gpio"
        ),
        motor_left_gpio=_require_int(pins_raw.get("motor_left_gpio", 17), "pins.motor_left_gpio"),
        motor_center_gpio=_require_int(
            pins_raw.get("motor_center_gpio", 27), "pins.motor_center_gpio"
        ),
        motor_right_gpio=_require_int(
            pins_raw.get("motor_right_gpio", 22), "pins.motor_right_gpio"
        ),
        button1_gpio=_require_int(pins_raw.get("button1_gpio", 23), "pins.button1_gpio"),
        button2_gpio=_require_int(pins_raw.get("button2_gpio", 24), "pins.button2_gpio"),
        button3_gpio=_require_int(pins_raw.get("button3_gpio", 25), "pins.button3_gpio"),
        gps_uart_rx_gpio=_require_int(
            pins_raw.get("gps_uart_rx_gpio", 15), "pins.gps_uart_rx_gpio"
        ),
        gps_uart_tx_gpio=_require_int(
            pins_raw.get("gps_uart_tx_gpio", 14), "pins.gps_uart_tx_gpio"
        ),
    )
    _ensure_hard_pin_constraints(pins)

    tof_raw = raw.get("tof", {})
    if not isinstance(tof_raw, dict):
        raise ValueError("tof must be a mapping")
    addresses_raw = tof_raw.get("addresses", {})
    if not isinstance(addresses_raw, dict):
        raise ValueError("tof.addresses must be a mapping")
    addresses = {
        "left": _coerce_hex_or_int(addresses_raw.get("left", 0x30), "tof.addresses.left"),
        "center": _coerce_hex_or_int(addresses_raw.get("center", 0x31), "tof.addresses.center"),
        "right": _coerce_hex_or_int(addresses_raw.get("right", 0x32), "tof.addresses.right"),
    }
    mode_raw = _require_str(tof_raw.get("distance_mode", "long"), "tof.distance_mode").lower()
    if mode_raw not in ("short", "long"):
        raise ValueError("tof.distance_mode must be 'short' or 'long'")
    tof = ToFConfig(
        i2c_bus=_require_int(tof_raw.get("i2c_bus", 1), "tof.i2c_bus"),
        addresses=addresses,
        distance_mode=mode_raw,  # type: ignore[assignment]
        timing_budget_ms=_require_int(tof_raw.get("timing_budget_ms", 50), "tof.timing_budget_ms"),
        inter_measurement_ms=_require_int(
            tof_raw.get("inter_measurement_ms", 60), "tof.inter_measurement_ms"
        ),
        median_window=_require_int(tof_raw.get("median_window", 5), "tof.median_window"),
        hysteresis_cm=_require_int(tof_raw.get("hysteresis_cm", 4), "tof.hysteresis_cm"),
    )
    if tof.median_window < 1:
        raise ValueError("tof.median_window must be >= 1")

    sensitivity_raw = raw.get("sensitivity_presets", {})
    if not isinstance(sensitivity_raw, dict) or not sensitivity_raw:
        raise ValueError("sensitivity_presets must be a non-empty mapping")
    sensitivity_presets: Dict[str, Dict[str, List[int]]] = {}
    for preset_name, preset in sensitivity_raw.items():
        if not isinstance(preset_name, str) or not isinstance(preset, dict):
            raise ValueError("sensitivity_presets must map names to mappings")
        thresholds = _deep_get(preset, "thresholds_cm")
        if not isinstance(thresholds, list) or len(thresholds) != 3:
            raise ValueError(f"sensitivity_presets.{preset_name}.thresholds_cm must be a list of 3")
        t0 = _require_int(thresholds[0], f"sensitivity_presets.{preset_name}.thresholds_cm[0]")
        t1 = _require_int(thresholds[1], f"sensitivity_presets.{preset_name}.thresholds_cm[1]")
        t2 = _require_int(thresholds[2], f"sensitivity_presets.{preset_name}.thresholds_cm[2]")
        if not (0 < t0 < t1 < t2):
            raise ValueError(f"sensitivity_presets.{preset_name}.thresholds_cm must be increasing")
        sensitivity_presets[preset_name] = {"thresholds_cm": [t0, t1, t2]}

    haptics_raw = raw.get("haptics", {})
    if not isinstance(haptics_raw, dict):
        raise ValueError("haptics must be a mapping")

    def _pattern(name: str, default_on: float, default_off: float) -> HapticsPatternConfig:
        pr = haptics_raw.get(name, {})
        if not isinstance(pr, dict):
            raise ValueError(f"haptics.{name} must be a mapping")
        on_s = _require_float(pr.get("on_s", default_on), f"haptics.{name}.on_s")
        off_s = _require_float(pr.get("off_s", default_off), f"haptics.{name}.off_s")
        if on_s < 0 or off_s < 0:
            raise ValueError(f"haptics.{name} on_s/off_s must be >= 0")
        return HapticsPatternConfig(on_s=on_s, off_s=off_s)

    haptics = HapticsConfig(
        continuous=_pattern("continuous", 1.0, 0.0),
        fast=_pattern("fast", 0.08, 0.12),
        slow=_pattern("slow", 0.20, 0.40),
    )

    audio_raw = raw.get("audio", {})
    if not isinstance(audio_raw, dict):
        raise ValueError("audio must be a mapping")
    audio = AudioConfig(
        enabled=_require_bool(audio_raw.get("enabled", True), "audio.enabled"),
        default_playback_source=_require_str(
            audio_raw.get("default_playback_source", ""), "audio.default_playback_source"
        ),
        playback_backend_preference=list(audio_raw.get("playback_backend_preference", ["cvlc", "mpg123"])),
        tone_volume=_require_float(audio_raw.get("tone_volume", 0.35), "audio.tone_volume"),
    )
    if not (0.0 <= audio.tone_volume <= 1.0):
        raise ValueError("audio.tone_volume must be between 0.0 and 1.0")

    gps_raw = raw.get("gps", {})
    if not isinstance(gps_raw, dict):
        raise ValueError("gps must be a mapping")
    serial_candidates = gps_raw.get(
        "serial_device_candidates", ["/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyUSB0"]
    )
    if not isinstance(serial_candidates, list) or not all(isinstance(x, str) for x in serial_candidates):
        raise ValueError("gps.serial_device_candidates must be a list of strings")
    gps = GPSConfig(
        enabled=_require_bool(gps_raw.get("enabled", True), "gps.enabled"),
        log_interval_s=_require_int(gps_raw.get("log_interval_s", 5), "gps.log_interval_s"),
        gpsd_host=_require_str(gps_raw.get("gpsd_host", "127.0.0.1"), "gps.gpsd_host"),
        gpsd_port=_require_int(gps_raw.get("gpsd_port", 2947), "gps.gpsd_port"),
        serial_device_candidates=serial_candidates,
        serial_baud=_require_int(gps_raw.get("serial_baud", 9600), "gps.serial_baud"),
    )
    if gps.log_interval_s < 1:
        raise ValueError("gps.log_interval_s must be >= 1")

    logging_raw = raw.get("logging", {})
    if not isinstance(logging_raw, dict):
        raise ValueError("logging must be a mapping")
    logging_cfg = LoggingConfig(
        level=_require_str(logging_raw.get("level", "INFO"), "logging.level").upper(),
        file=_require_str(logging_raw.get("file", "logs/app.log"), "logging.file"),
        max_bytes=_require_int(logging_raw.get("max_bytes", 1_000_000), "logging.max_bytes"),
        backup_count=_require_int(logging_raw.get("backup_count", 5), "logging.backup_count"),
    )
    if logging_cfg.max_bytes < 1:
        raise ValueError("logging.max_bytes must be >= 1")
    if logging_cfg.backup_count < 0:
        raise ValueError("logging.backup_count must be >= 0")

    return AppConfig(
        simulation_mode=simulation_mode,
        loop_hz=loop_hz,
        pins=pins,
        tof=tof,
        sensitivity_presets=sensitivity_presets,
        haptics=haptics,
        audio=audio,
        gps=gps,
        logging=logging_cfg,
    )


def get_default_sensitivity_name(config: AppConfig) -> str:
    if "normal" in config.sensitivity_presets:
        return "normal"
    return sorted(config.sensitivity_presets.keys())[0]


def get_thresholds_for_sensitivity(config: AppConfig, preset_name: str) -> Tuple[int, int, int]:
    preset = config.sensitivity_presets.get(preset_name)
    if not preset:
        raise KeyError(f"Unknown sensitivity preset: {preset_name}")
    thresholds = preset.get("thresholds_cm")
    if not thresholds or len(thresholds) != 3:
        raise ValueError(f"Invalid thresholds for preset: {preset_name}")
    return int(thresholds[0]), int(thresholds[1]), int(thresholds[2])
