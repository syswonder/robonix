"""Microphone capture driver using arecord subprocess.

This module provides MicDriver, which wraps the ALSA `arecord` command-line
tool to capture raw PCM audio from a microphone device. It produces audio
chunks (dicts) at a configurable rate for consumption by the gRPC servicer.

Architecture position: Used by node.py to create a mic capture pipeline:
    ALSA device → arecord subprocess → MicDriver.read_chunk() → gRPC AudioChunk

Why subprocess (not PyAudio/sounddevice):
    - arecord is universally available on Linux ALSA systems
    - No Python C-extension compilation needed (no portaudio dependency)
    - Simple and reliable for fixed-format capture

Supported formats:
    - 8-bit:  S8
    - 16-bit: S16_LE (default, standard for ASR)
    - 24-bit: S24_LE
    - 32-bit: S32_LE

Usage:
    driver = MicDriver(device_id="hw:1,0", sample_rate=16000, channels=1)
    driver.start()
    while True:
        chunk = driver.read_chunk()
        if chunk is None:
            break
        # chunk = {"timestamp_ns": int, "data": bytes, "sequence": int, "duration_s": float}
        process(chunk)
    driver.stop()
"""
import subprocess
import time
import logging

log = logging.getLogger(__name__)

# ALSA format name mapping — bits_per_sample → arecord -f argument.
_ALSA_FORMATS = {
    16: "S16_LE",   # 16-bit signed little-endian (standard for ASR)
    24: "S24_LE",   # 24-bit signed little-endian
    32: "S32_LE",   # 32-bit signed little-endian
    8: "S8",        # 8-bit signed
}


class MicDriver:
    """Captures audio from an ALSA device via arecord subprocess.

    Launches `arecord -D <device> -f <format> -r <rate> -c <channels> -t raw`
    and reads fixed-size chunks from its stdout. Each chunk is wrapped in a
    dict with metadata (timestamp, sequence number, duration).

    Thread safety: NOT thread-safe. Designed for single-reader use within
    a dedicated gRPC servicer thread (one client per MicDriver instance).

    Lifecycle:
        1. __init__() — configure parameters, no subprocess yet
        2. start() — launch arecord subprocess
        3. read_chunk() — read one chunk (blocking, returns dict or None)
        4. stop() — terminate subprocess

    Args:
        device_id: ALSA device string (e.g. "hw:1,0", "plughw:0,0").
        sample_rate: Capture sample rate in Hz (default 16000 for ASR).
        channels: Number of channels (default 1 = mono).
        bits_per_sample: Bits per sample, 8/16/24/32 (default 16).
        chunk_duration_s: Duration of each read_chunk() call in seconds
            (default 0.1 = 100ms). Determines chunk size in bytes:
            bytes = sample_rate × channels × (bits/8) × duration_s
    """

    def __init__(
        self,
        device_id: str,
        sample_rate: int = 16000,
        channels: int = 1,
        bits_per_sample: int = 16,
        chunk_duration_s: float = 0.1,
    ):
        self.device_id = device_id
        self.sample_rate = sample_rate
        self.channels = channels
        self.bits_per_sample = bits_per_sample
        self.chunk_duration_s = chunk_duration_s

        # Pre-compute exact chunk size in bytes for read_chunk()
        self._bytes_per_sample = bits_per_sample // 8
        self._chunk_bytes = (
            int(sample_rate * chunk_duration_s)
            * channels * self._bytes_per_sample
        )
        self._process: subprocess.Popen | None = None
        self._sequence = 0

    def start(self) -> None:
        """Launch the arecord subprocess for audio capture.

        Builds and executes the arecord command with the configured parameters.
        The subprocess stdout is read in fixed-size chunks by read_chunk().

        Raises:
            ValueError: If bits_per_sample is not in _ALSA_FORMATS.
            FileNotFoundError: If arecord is not installed on the system.
        """
        fmt = _ALSA_FORMATS.get(self.bits_per_sample)
        if fmt is None:
            raise ValueError(f"Unsupported bits_per_sample={self.bits_per_sample}")

        cmd = [
            "arecord",
            "-D", self.device_id,
            "-f", fmt,
            "-r", str(self.sample_rate),
            "-c", str(self.channels),
            "-t", "raw",
        ]
        log.info("Starting mic capture: %s", " ".join(cmd))
        self._process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self._sequence = 0

    def read_chunk(self) -> dict | None:
        """Read one audio chunk from arecord stdout (blocking).

        Reads exactly _chunk_bytes from the subprocess stdout. This blocks
        until enough audio data is available (real-time capture rate).

        Returns:
            Dict with keys:
                timestamp_ns (int): Nanoseconds since epoch when chunk was read.
                data (bytes): Raw PCM audio bytes (length = _chunk_bytes).
                sequence (int): Monotonically increasing chunk counter.
                duration_s (float): Configured chunk duration in seconds.
            Returns None if the subprocess has ended or read returns less
            data than expected (partial read / EOF).
        """
        if self._process is None:
            return None

        data = self._process.stdout.read(self._chunk_bytes)
        if not data or len(data) < self._chunk_bytes:
            return None

        chunk = {
            "timestamp_ns": int(time.time() * 1e9),
            "data": data,
            "sequence": self._sequence,
            "duration_s": self.chunk_duration_s,
        }
        self._sequence += 1
        return chunk

    def stop(self) -> None:
        """Stop the arecord subprocess gracefully.

        Sends SIGTERM, waits up to 5 seconds, then SIGKILL if necessary.
        Safe to call multiple times.
        """
        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None
            log.info("Mic capture stopped")

    @property
    def is_running(self) -> bool:
        """True if arecord subprocess is alive and capturing."""
        return self._process is not None and self._process.poll() is None
