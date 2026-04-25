"""ALSA device auto-detection and driver registry for the audio_driver package.

This module provides two main capabilities:

1. **Device scanning**: Parses `arecord -l` and `aplay -l` output to discover
   all ALSA audio devices on the system, including USB microphones and speakers.

2. **Driver registry**: A pluggable system for hardware-specific drivers.
   The DefaultAlsaDriver handles any standard ALSA device. To support
   specialized hardware (e.g. ReSpeaker multi-channel mic arrays), create
   a subclass of AudioDeviceDriver and call register_driver().

Architecture position: This is a utility module used by node.py during
startup to discover available audio hardware before creating MicDriver
and SpeakerDriver instances.

Usage:
    from audio_driver.alsa_utils import scan_alsa_devices, find_default_mic

    devices = scan_alsa_devices()
    mic = find_default_mic(devices)
    if mic:
        print(f"Using mic: {mic.device_id} ({mic.name})")

    # Register a custom driver:
    from audio_driver.alsa_utils import AudioDeviceDriver, register_driver
    class MyDriver(AudioDeviceDriver):
        def detect(self, devices): ...
        def name(self): return "My Custom Driver"
    register_driver(MyDriver())
"""
import re
import subprocess
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

log = logging.getLogger(__name__)


@dataclass
class AlsaDeviceInfo:
    """Represents a single ALSA audio device discovered on the system.

    Attributes:
        card: ALSA card index (0 = usually built-in, 1+ = often USB).
        device: ALSA device index within the card.
        name: Human-readable device name from arecord/aplay (e.g. "USB Audio Device").
        device_id: ALSA hardware device string (e.g. "hw:1,0") used by arecord/aplay -D.
        is_input: True if this device can capture audio (found in arecord -l).
        is_output: True if this device can play audio (found in aplay -l).
    """
    card: int
    device: int
    name: str          # e.g. "USB Audio Device"
    device_id: str     # e.g. "hw:1,0"
    is_input: bool = False
    is_output: bool = False


# ── ALSA device scanning ─────────────────────────────────────────────────

def _parse_alsa_devices(output: str, is_input: bool) -> list[AlsaDeviceInfo]:
    """Parse the output of `arecord -l` or `aplay -l` into AlsaDeviceInfo list.

    Matches lines like:
        card 1: Device [USB Audio Device], device 0: USB Audio ...

    Args:
        output: Combined stdout+stderr from the ALSA listing command.
        is_input: True for arecord (input devices), False for aplay (output).

    Returns:
        List of AlsaDeviceInfo, one per matched device line.
    """
    devices = []
    # Match lines like: card 1: Device [USB Audio Device], device 0: ...
    pattern = re.compile(
        r"card\s+(\d+):\s+(.+?)\s*\[(.+?)\],\s+device\s+(\d+):"
    )
    for match in pattern.finditer(output):
        card = int(match.group(1))
        name = match.group(3).strip()
        device = int(match.group(4))
        device_id = f"hw:{card},{device}"
        devices.append(AlsaDeviceInfo(
            card=card, device=device, name=name,
            device_id=device_id, is_input=is_input, is_output=not is_input,
        ))
    return devices


def scan_alsa_devices() -> list[AlsaDeviceInfo]:
    """Scan all ALSA input and output devices on the system.

    Runs `arecord -l` (input/mic devices) and `aplay -l` (output/speaker
    devices) via subprocess, parses the output, and merges results by
    (card, device) key. A device that appears in both lists will have
    both is_input=True and is_output=True.

    Returns:
        List of AlsaDeviceInfo. Empty if no ALSA devices found or if
        arecord/aplay are not installed.

    Typical output on a machine with USB mic + built-in speaker:
        [AlsaDeviceInfo(card=0, device=0, name="HDA Intel PCH", ...),
         AlsaDeviceInfo(card=1, device=0, name="USB Audio Device", ...)]
    """
    merged: dict[tuple[int, int], AlsaDeviceInfo] = {}

    for cmd, is_input in [("arecord", True), ("aplay", False)]:
        try:
            result = subprocess.run(
                [cmd, "-l"], capture_output=True, text=True, timeout=5)
            output = result.stdout + result.stderr
        except FileNotFoundError:
            log.warning("%s not found, skipping %s scan",
                        cmd, "input" if is_input else "output")
            continue
        except subprocess.TimeoutExpired:
            log.warning("%s -l timed out", cmd)
            continue

        for dev in _parse_alsa_devices(output, is_input):
            key = (dev.card, dev.device)
            if key in merged:
                existing = merged[key]
                if is_input:
                    existing.is_input = True
                else:
                    existing.is_output = True
            else:
                merged[key] = dev

    devices = list(merged.values())
    for d in devices:
        log.debug("ALSA device: %s (%s) in=%s out=%s",
                  d.device_id, d.name, d.is_input, d.is_output)
    return devices


def find_default_mic(devices: list[AlsaDeviceInfo] | None = None) -> AlsaDeviceInfo | None:
    """Find the best default microphone device.

    Selection priority:
        1. USB input devices (name contains "usb" or card >= 1)
        2. Any input device (first available)

    Args:
        devices: Pre-scanned device list. If None, calls scan_alsa_devices().

    Returns:
        AlsaDeviceInfo for the best mic, or None if no input device exists.
    """
    if devices is None:
        devices = scan_alsa_devices()
    inputs = [d for d in devices if d.is_input]
    if not inputs:
        return None
    # Prefer USB devices (typically have "USB" in name or card >= 1)
    usb = [d for d in inputs if "usb" in d.name.lower() or d.card >= 1]
    return usb[0] if usb else inputs[0]


def find_default_speaker(devices: list[AlsaDeviceInfo] | None = None) -> AlsaDeviceInfo | None:
    """Find the best default speaker device.

    Selection priority:
        1. USB output devices (name contains "usb" or card >= 1)
        2. Any output device (first available)

    Args:
        devices: Pre-scanned device list. If None, calls scan_alsa_devices().

    Returns:
        AlsaDeviceInfo for the best speaker, or None if no output device exists.
    """
    if devices is None:
        devices = scan_alsa_devices()
    outputs = [d for d in devices if d.is_output]
    if not outputs:
        return None
    usb = [d for d in outputs if "usb" in d.name.lower() or d.card >= 1]
    return usb[0] if usb else outputs[0]


# ── Driver Registry ─────────────────────────────────────────────────────
# Pluggable driver system for hardware-specific audio device detection.
# The DefaultAlsaDriver is always registered and accepts all ALSA devices.
# To add support for specialized hardware:
#   1. Subclass AudioDeviceDriver
#   2. Implement detect() to filter supported devices
#   3. Call register_driver(MyDriver()) at module load time
# The node.py startup will iterate through registered drivers to match hardware.

class AudioDeviceDriver(ABC):
    """Abstract base class for audio device drivers.

    A driver's job is to identify which ALSA devices it supports (via detect())
    and provide a human-readable name. Future extensions may add methods for
    device-specific configuration (e.g. ReSpeaker channel mapping).

    Example — custom driver for ReSpeaker mic arrays:
        class RespeakerDriver(AudioDeviceDriver):
            def detect(self, devices):
                return [d for d in devices if "ReSpeaker" in d.name]
            def name(self):
                return "ReSpeaker Driver"
        register_driver(RespeakerDriver())
    """

    @abstractmethod
    def detect(self, devices: list[AlsaDeviceInfo]) -> list[AlsaDeviceInfo]:
        """Return the subset of devices this driver supports.

        Args:
            devices: All ALSA devices found on the system.

        Returns:
            Filtered list of devices this driver can handle.
        """
        ...

    @abstractmethod
    def name(self) -> str:
        """Human-readable driver name for logging."""
        ...


class DefaultAlsaDriver(AudioDeviceDriver):
    """Default ALSA driver — accepts all standard ALSA devices.

    This is the catch-all driver registered by default. It doesn't filter
    any devices, making all detected hardware available for use.
    """

    def detect(self, devices: list[AlsaDeviceInfo]) -> list[AlsaDeviceInfo]:
        return devices  # accepts everything

    def name(self) -> str:
        return "ALSA Default"


# Global driver registry — DefaultAlsaDriver is always available.
# Additional drivers are appended via register_driver().
_DRIVER_REGISTRY: list[AudioDeviceDriver] = [DefaultAlsaDriver()]


def register_driver(driver: AudioDeviceDriver) -> None:
    """Register a hardware-specific audio driver.

    Registered drivers are available for device detection. Call this at
    module import time (top-level) or in your package's __init__.py.

    Args:
        driver: An AudioDeviceDriver subclass instance.
    """
    _DRIVER_REGISTRY.append(driver)
    log.info("Registered audio driver: %s", driver.name())


def get_drivers() -> list[AudioDeviceDriver]:
    """Return a copy of the current driver registry."""
    return list(_DRIVER_REGISTRY)
