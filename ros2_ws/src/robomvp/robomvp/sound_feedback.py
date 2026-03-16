#!/usr/bin/env python3
"""Moduł sygnalizacji dźwiękowej dla systemu RoboMVP.

Dostarcza funkcje generujące sygnały dźwiękowe informujące
o wyniku operacji: sukces (dwa krótkie, szybkie dźwięki)
lub porażka (jeden długi, niski dźwięk).
"""

import math
import os
import struct
import subprocess
import tempfile
import time
import wave


def _generate_tone(
    frequency: float,
    duration: float,
    amplitude: float = 0.5,
    sample_rate: int = 44100,
) -> bytes:
    """Generuje próbki tonu sinusoidalnego jako dane PCM 16-bit mono.

    Args:
        frequency: Częstotliwość tonu w Hz.
        duration: Czas trwania w sekundach.
        amplitude: Amplituda (0.0–1.0).
        sample_rate: Częstotliwość próbkowania w Hz.

    Returns:
        Bajty z próbkami PCM (16-bit, signed, mono).
    """
    num_samples = int(sample_rate * duration)
    max_value = int(32767 * amplitude)
    samples = bytearray(num_samples * 2)
    two_pi_f_over_sr = 2.0 * math.pi * frequency / sample_rate
    for i in range(num_samples):
        value = int(max_value * math.sin(two_pi_f_over_sr * i))
        struct.pack_into('<h', samples, i * 2, value)
    return bytes(samples)


def _write_wav(filepath: str, pcm_data: bytes, sample_rate: int = 44100):
    """Zapisuje surowe dane PCM do pliku WAV.

    Args:
        filepath: Ścieżka do pliku wyjściowego.
        pcm_data: Dane PCM (16-bit, signed, mono).
        sample_rate: Częstotliwość próbkowania w Hz.
    """
    with wave.open(filepath, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(pcm_data)


def _play_wav(filepath: str):
    """Odtwarza plik WAV za pomocą aplay (Linux).

    Jeśli aplay nie jest dostępny lub wystąpi błąd,
    funkcja kończy się bez zgłaszania wyjątku.

    Args:
        filepath: Ścieżka do pliku WAV.
    """
    try:
        subprocess.run(
            ['aplay', '-q', filepath],
            timeout=10.0,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass


def _play_tone(frequency: float, duration: float, amplitude: float = 0.5):
    """Generuje i odtwarza pojedynczy ton sinusoidalny.

    Args:
        frequency: Częstotliwość tonu w Hz.
        duration: Czas trwania w sekundach.
        amplitude: Amplituda (0.0–1.0).
    """
    pcm_data = _generate_tone(frequency, duration, amplitude)
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            tmp_path = tmp.name
        _write_wav(tmp_path, pcm_data)
        _play_wav(tmp_path)
    finally:
        if tmp_path is not None:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass


def play_success():
    """Odtwarza sygnał sukcesu: dwa krótkie, szybkie tony wysokiej częstotliwości.

    Dwa sygnały o częstotliwości 880 Hz (A5), każdy trwający 0,15 s,
    z przerwą 0,08 s między nimi.
    """
    _play_tone(880.0, 0.15)
    time.sleep(0.08)
    _play_tone(880.0, 0.15)


def play_failure():
    """Odtwarza sygnał porażki: jeden długi, niski ton.

    Jeden sygnał o częstotliwości 220 Hz (A3), trwający 0,8 s.
    """
    _play_tone(220.0, 0.8)
