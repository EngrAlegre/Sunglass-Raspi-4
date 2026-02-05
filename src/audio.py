from __future__ import annotations

import math
import os
import shutil
import subprocess
import tempfile
import threading
import wave
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass(frozen=True)
class ToneSpec:
    freq_hz: int
    duration_s: float


class Audio:
    def __init__(
        self,
        enabled: bool,
        tone_volume: float = 0.35,
        playback_backend_preference: Optional[List[str]] = None,
    ) -> None:
        self._enabled = bool(enabled)
        self._tone_volume = float(tone_volume)
        self._backend_preference = playback_backend_preference or ["cvlc", "mpg123"]
        self._tone_cache: Dict[Tuple[int, int, int], str] = {}
        self._tone_process: Optional[subprocess.Popen] = None
        self._playback_process: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()

    def close(self) -> None:
        self.stop_tone()
        self.stop_playback()

    def stop_tone(self) -> None:
        with self._lock:
            p = self._tone_process
            self._tone_process = None
        if p is not None:
            try:
                p.terminate()
            except Exception:
                pass

    def stop_playback(self) -> None:
        with self._lock:
            p = self._playback_process
            self._playback_process = None
        if p is not None:
            try:
                p.terminate()
            except Exception:
                pass

    def _tone_path(self, freq_hz: int, duration_ms: int, volume_milli: int) -> str:
        key = (freq_hz, duration_ms, volume_milli)
        if key in self._tone_cache and os.path.exists(self._tone_cache[key]):
            return self._tone_cache[key]
        directory = tempfile.gettempdir()
        filename = f"ns_tone_{freq_hz}hz_{duration_ms}ms_{volume_milli}.wav"
        path = os.path.join(directory, filename)
        self._write_tone_wav(path, freq_hz=freq_hz, duration_s=duration_ms / 1000.0, volume=volume_milli / 1000.0)
        self._tone_cache[key] = path
        return path

    def _write_tone_wav(self, path: str, freq_hz: int, duration_s: float, volume: float) -> None:
        sample_rate = 22050
        total_samples = max(1, int(sample_rate * duration_s))
        volume = max(0.0, min(1.0, float(volume)))
        amp = int(32767 * volume)
        with wave.open(path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            frames = bytearray()
            for i in range(total_samples):
                v = int(amp * math.sin(2.0 * math.pi * freq_hz * (i / sample_rate)))
                frames += int(v).to_bytes(2, byteorder="little", signed=True)
            wf.writeframes(frames)

    def play_tone(self, freq_hz: int, duration_s: float) -> None:
        if not self._enabled:
            return
        aplay = shutil.which("aplay")
        if aplay is None:
            return
        duration_ms = int(max(10.0, duration_s * 1000.0))
        volume_milli = int(max(0.0, min(1.0, self._tone_volume)) * 1000)
        path = self._tone_path(freq_hz=freq_hz, duration_ms=duration_ms, volume_milli=volume_milli)
        self.stop_tone()
        p = subprocess.Popen([aplay, "-q", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with self._lock:
            self._tone_process = p

    def play_mode_tone(self, mode: str) -> None:
        mapping = {
            "obstacles_on": ToneSpec(freq_hz=880, duration_s=0.10),
            "obstacles_off": ToneSpec(freq_hz=440, duration_s=0.10),
            "audio_on": ToneSpec(freq_hz=660, duration_s=0.12),
            "audio_off": ToneSpec(freq_hz=330, duration_s=0.12),
            "sensitivity_near": ToneSpec(freq_hz=784, duration_s=0.09),
            "sensitivity_normal": ToneSpec(freq_hz=523, duration_s=0.09),
            "sensitivity_far": ToneSpec(freq_hz=392, duration_s=0.09),
        }
        spec = mapping.get(mode)
        if spec is None:
            return
        self.play_tone(spec.freq_hz, spec.duration_s)

    def start_playback(self, source: str) -> bool:
        if not self._enabled:
            return False
        source = (source or "").strip()
        if not source:
            return False
        if self.is_playback_running():
            return True

        cmd = self._build_playback_command(source)
        if cmd is None:
            return False
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            return False
        with self._lock:
            self._playback_process = p
        return True

    def _build_playback_command(self, source: str) -> Optional[List[str]]:
        for backend in self._backend_preference:
            backend = backend.strip().lower()
            if backend == "cvlc":
                cvlc = shutil.which("cvlc") or shutil.which("vlc")
                if cvlc is None:
                    continue
                return [
                    cvlc,
                    "--intf",
                    "dummy",
                    "--no-video",
                    "--loop",
                    "--quiet",
                    source,
                ]
            if backend == "mpg123":
                mpg123 = shutil.which("mpg123")
                if mpg123 is None:
                    continue
                return [mpg123, "-q", "--loop", "-1", source]
        return None

    def is_playback_running(self) -> bool:
        with self._lock:
            p = self._playback_process
        return p is not None and p.poll() is None
