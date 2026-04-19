"""Speaker playback driver using aplay subprocess.

This module provides SpeakerDriver, which wraps the ALSA `aplay` command-line
tool to play raw PCM audio through a speaker device. It writes audio data to
aplay's stdin and handles automatic restart on pipe errors.

Architecture position: Used by node.py to create a speaker playback pipeline:
    gRPC AudioChunk → SpeakerDriver.play_chunk() → aplay stdin → ALSA speaker

Key design decisions:
    - Lazy subprocess start: aplay is started on first play_chunk() call
    - Thread-safe: uses threading.Lock for concurrent play_chunk() calls
    - Auto-restart: on BrokenPipeError (aplay crashed/ended), re-launches
      automatically and retries the write
    - No buffering: each play_chunk() call flushes immediately to aplay stdin

Usage:
    driver = SpeakerDriver(device_id="hw:1,0", sample_rate=24000)
    driver.play_chunk(pcm_audio_bytes)
    driver.play_chunk(more_audio_bytes)
    driver.stop()
"""
import subprocess
import threading
import logging

log = logging.getLogger(__name__)

# ALSA format name mapping — bits_per_sample → aplay -f argument.
_ALSA_FORMATS = {
    16: "S16_LE",   # 16-bit signed little-endian (standard for TTS output)
    24: "S24_LE",   # 24-bit signed little-endian
    32: "S32_LE",   # 32-bit signed little-endian
    8: "S8",        # 8-bit signed
}


class SpeakerDriver:
    """Plays raw PCM audio through an ALSA device via aplay subprocess.

    Launches `aplay -D <device> -f <format> -r <rate> -c <channels> -t raw`
    on first play_chunk() call. Audio data is written to aplay's stdin.
    If the aplay process dies (BrokenPipeError), it is automatically
    restarted and the audio data is re-sent.

    Thread safety: YES — all writes are protected by a threading.Lock.
    Safe for concurrent use from multiple gRPC handler threads.

    Lifecycle:
        1. __init__() — configure parameters, no subprocess yet
        2. play_chunk(data) — lazy-start aplay, write audio data
        3. play_chunk(data) — write more audio (reuses same process)
        4. stop() — close stdin and terminate aplay

    Args:
        device_id: ALSA device string (e.g. "hw:1,0", "plughw:0,0").
        sample_rate: Playback sample rate in Hz (default 24000 for TTS).
        channels: Number of channels (default 1 = mono).
        bits_per_sample: Bits per sample, 8/16/24/32 (default 16).
    """

    def __init__(
        self,
        device_id: str,
        sample_rate: int = 24000,
        channels: int = 1,
        bits_per_sample: int = 16,
    ):
        self.device_id = device_id
        self.sample_rate = sample_rate
        self.channels = channels
        self.bits_per_sample = bits_per_sample
        self._process: subprocess.Popen | None = None
        self._lock = threading.Lock()

    def _ensure_aplay(self) -> None:
        """Lazy-start the aplay subprocess if not already running.

        Called internally by play_chunk() before each write. If aplay has
        exited (poll() returns non-None), a new process is started.

        Raises:
            ValueError: If bits_per_sample is not supported.
        """
        if self._process and self._process.poll() is None:
            return

        fmt = _ALSA_FORMATS.get(self.bits_per_sample)
        if fmt is None:
            raise ValueError(f"Unsupported bits_per_sample={self.bits_per_sample}")

        cmd = [
            "aplay",
            "-D", self.device_id,
            "-f", fmt,
            "-r", str(self.sample_rate),
            "-c", str(self.channels),
            "-t", "raw",
        ]
        log.info("Starting speaker playback: %s", " ".join(cmd))
        self._process = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    def play_chunk(self, audio_data: bytes) -> None:
        """Write audio data to aplay stdin for immediate playback.

        Thread-safe: acquires a lock so concurrent calls are serialized.
        Auto-restart: if writing fails (BrokenPipeError), terminates the
        old aplay process, starts a new one, and retries the write.

        Args:
            audio_data: Raw PCM audio bytes. Must match the configured
                format (sample_rate, channels, bits_per_sample). Any
                length is acceptable — aplay buffers internally.
        """
        with self._lock:
            self._ensure_aplay()
            try:
                self._process.stdin.write(audio_data)
                self._process.stdin.flush()
            except BrokenPipeError:
                log.warning("aplay pipe broken, restarting")
                self._process.terminate()
                self._process.wait(timeout=3)
                self._process = None
                self._ensure_aplay()
                self._process.stdin.write(audio_data)
                self._process.stdin.flush()

    def stop(self) -> None:
        """Stop the aplay subprocess gracefully.

        Closes stdin (signals aplay to finish), then sends SIGTERM.
        Waits up to 5 seconds, then SIGKILL if necessary.
        Safe to call multiple times.
        """
        with self._lock:
            if self._process:
                try:
                    self._process.stdin.close()
                except Exception:
                    pass
                self._process.terminate()
                try:
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                self._process = None
                log.info("Speaker playback stopped")

    @property
    def is_running(self) -> bool:
        """True if aplay subprocess is alive and ready for audio."""
        return self._process is not None and self._process.poll() is None
