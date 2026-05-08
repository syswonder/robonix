#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Audio primitive — Capability-based driver.

Owns `robonix/primitive/audio/*`. Two gRPC streaming interfaces:
  primitive/audio/driver   rpc        gRPC lifecycle (Capability built-in)
  primitive/audio/mic      rpc        server-stream `Stream() returns (stream AudioChunk)`
  primitive/audio/speaker  rpc        client-stream `Stream(stream AudioChunk) returns Empty`

driver-init: start.sh just spawns this; `Driver(CMD_INIT, config_json)`
scans ALSA, instantiates the per-device MicDriver / SpeakerDriver
subprocess wrappers, returns ready. The streaming handlers below pick
the module-level drivers up the first time a client connects.

Env vars:
  AUDIO_MIC_DEVICE             override mic ALSA device id (e.g. "hw:2,0")
  AUDIO_MIC_SAMPLE_RATE        Hz (default 16000)
  AUDIO_MIC_CHANNELS           default 1
  AUDIO_MIC_BITS               default 16
  AUDIO_MIC_CHUNK_MS           default 100
  AUDIO_SPEAKER_DEVICE         override speaker ALSA device id
  AUDIO_SPEAKER_SAMPLE_RATE    Hz (default 24000)
  AUDIO_SPEAKER_CHANNELS       default 1
  AUDIO_SPEAKER_BITS           default 16
"""
from __future__ import annotations

import logging
import os

from robonix_api import Capability

cap = Capability(id="audio_driver", namespace="robonix/primitive/audio")
log = logging.getLogger("audio-driver")

import audio_pb2          # type: ignore  # noqa: E402  (codegen)
import std_msgs_pb2       # type: ignore  # noqa: E402

from audio_driver.alsa_utils import scan_alsa_devices, find_default_mic, find_default_speaker  # noqa: E402
from audio_driver.mic_driver import MicDriver  # noqa: E402
from audio_driver.speaker_driver import SpeakerDriver  # noqa: E402

mic_driver: MicDriver | None = None
speaker_driver: SpeakerDriver | None = None
# Currently-selected device ids (mirror of what mic_driver/speaker_driver
# point at). Empty string = OS / ALSA default. Set by SelectAudioDevice;
# read back by ListAudioDevices.current_*_id.
current_input_id: str = ""
current_output_id: str = ""


# ── streaming handlers ─────────────────────────────────────────────────────
@cap.grpc("robonix/primitive/audio/mic")
def mic_stream(request, context):
    """Server-streaming mic capture. Drives one arecord subprocess per
    open client; ALSA only allows one capture handle so concurrent clients
    share the stream (driver is started by the first call, kept alive
    while any context is active)."""
    if mic_driver is None:
        context.abort(__import__("grpc").StatusCode.UNAVAILABLE,
                      "mic driver not initialized — Driver(CMD_INIT) failed or never ran")
        return
    log.info("mic stream client connected")
    mic_driver.start()
    try:
        while context.is_active():
            chunk = mic_driver.read_chunk()
            if chunk is None:
                break
            yield audio_pb2.AudioChunk(
                timestamp_ns=chunk["timestamp_ns"],
                data=chunk["data"],
                sequence=chunk["sequence"],
                duration_s=chunk["duration_s"],
            )
    finally:
        mic_driver.stop()
        log.info("mic stream client disconnected")


@cap.grpc("robonix/primitive/audio/speaker")
def speaker_stream(request_iterator, context):
    """Client-streaming playback. Pipes received PCM chunks into aplay;
    SpeakerDriver lazy-starts the subprocess and auto-restarts after
    underruns. Returns Empty on stream close."""
    if speaker_driver is None:
        context.abort(__import__("grpc").StatusCode.UNAVAILABLE,
                      "speaker driver not initialized")
        return std_msgs_pb2.Empty()
    log.info("speaker stream client connected")
    try:
        for chunk in request_iterator:
            if chunk.data:
                speaker_driver.play_chunk(bytes(chunk.data))
    finally:
        log.info("speaker stream client disconnected")
    return std_msgs_pb2.Empty()


# ── device list / select ───────────────────────────────────────────────────
#
# Wraps the ALSA scan that init() already runs, so the rbnx chat audio
# settings page can show the same device list and let the user repick
# without restarting the package. SelectAudioDevice rebuilds the
# matching Mic/SpeakerDriver in place; an active stream sees the
# replacement on its next read/write since the streaming handlers
# reference the module globals each iteration.

def _scan_audio_devices_proto():
    """Run the ALSA scan and convert each entry to AudioDevice proto."""
    devs = []
    default_mic = find_default_mic(scan_alsa_devices())
    default_spk = find_default_speaker(scan_alsa_devices())
    default_mic_id = default_mic.device_id if default_mic else ""
    default_spk_id = default_spk.device_id if default_spk else ""
    for d in scan_alsa_devices():
        if not (d.is_input or d.is_output):
            continue
        kind = "duplex" if (d.is_input and d.is_output) else \
               "input" if d.is_input else "output"
        is_default = (d.device_id == default_mic_id and d.is_input) or \
                     (d.device_id == default_spk_id and d.is_output)
        devs.append(audio_pb2.AudioDevice(
            id=d.device_id,
            name=d.name,
            kind=kind,
            is_default=is_default,
            channels=1,           # arecord -l doesn't report; conservative default
            note="",
        ))
    return devs


@cap.grpc("robonix/primitive/audio/list_devices")
def list_devices(request, context):
    return audio_pb2.ListAudioDevices_Response(
        devices=_scan_audio_devices_proto(),
        current_input_id=current_input_id,
        current_output_id=current_output_id,
    )


@cap.grpc("robonix/primitive/audio/select_device")
def select_device(request, context):
    global mic_driver, speaker_driver, current_input_id, current_output_id
    kind = (request.kind or "").lower()
    if kind not in ("input", "output"):
        return audio_pb2.SelectAudioDevice_Response(
            ok=False, error=f"kind must be 'input' or 'output', got '{kind}'")

    requested = request.id
    # "" means revert to default; otherwise ensure the id exists.
    if requested:
        valid = {d.device_id for d in scan_alsa_devices()
                 if (d.is_input if kind == "input" else d.is_output)}
        if requested not in valid:
            return audio_pb2.SelectAudioDevice_Response(
                ok=False, error=f"unknown {kind} id '{requested}'")
        new_id = requested
    else:
        info = find_default_mic(scan_alsa_devices()) if kind == "input" \
               else find_default_speaker(scan_alsa_devices())
        if info is None:
            return audio_pb2.SelectAudioDevice_Response(
                ok=False, error=f"no default {kind} device")
        new_id = info.device_id

    if kind == "input":
        if mic_driver is not None:
            try:
                mic_driver.stop()
            except Exception:  # noqa: BLE001
                pass
        mic_driver = MicDriver(
            device_id=new_id,
            sample_rate=int(os.environ.get("AUDIO_MIC_SAMPLE_RATE", "16000")),
            channels=int(os.environ.get("AUDIO_MIC_CHANNELS", "1")),
            bits_per_sample=int(os.environ.get("AUDIO_MIC_BITS", "16")),
            chunk_duration_s=int(os.environ.get("AUDIO_MIC_CHUNK_MS", "100")) / 1000.0,
        )
        current_input_id = new_id
    else:
        speaker_driver = SpeakerDriver(
            device_id=new_id,
            sample_rate=int(os.environ.get("AUDIO_SPEAKER_SAMPLE_RATE", "24000")),
            channels=int(os.environ.get("AUDIO_SPEAKER_CHANNELS", "1")),
            bits_per_sample=int(os.environ.get("AUDIO_SPEAKER_BITS", "16")),
        )
        current_output_id = new_id
    return audio_pb2.SelectAudioDevice_Response(ok=True, error="")


# ── driver-init lifecycle ──────────────────────────────────────────────────
@cap.on_init
def init(cfg):
    """ALSA scan + device pickup. Honours AUDIO_{MIC,SPEAKER}_DEVICE env
    overrides; falls back to the auto-detector in alsa_utils. Refuses to
    come up if neither a mic nor a speaker is available, so atlas defers
    instead of advertising dead interfaces."""
    global mic_driver, speaker_driver, current_input_id, current_output_id

    devices = scan_alsa_devices()
    for d in devices:
        log.info("ALSA: %s (%s) in=%s out=%s",
                 d.device_id, d.name, d.is_input, d.is_output)

    mic_id = os.environ.get("AUDIO_MIC_DEVICE", "").strip()
    if mic_id:
        log.info("mic override: %s", mic_id)
        mic_dev_id: str | None = mic_id
    else:
        mic_info = find_default_mic(devices)
        if mic_info:
            log.info("mic auto-detected: %s (%s)", mic_info.device_id, mic_info.name)
            mic_dev_id = mic_info.device_id
        else:
            log.warning("no microphone found")
            mic_dev_id = None

    spk_id = os.environ.get("AUDIO_SPEAKER_DEVICE", "").strip()
    if spk_id:
        log.info("speaker override: %s", spk_id)
        spk_dev_id: str | None = spk_id
    else:
        spk_info = find_default_speaker(devices)
        if spk_info:
            log.info("speaker auto-detected: %s (%s)", spk_info.device_id, spk_info.name)
            spk_dev_id = spk_info.device_id
        else:
            log.warning("no speaker found")
            spk_dev_id = None

    if mic_dev_id is None and spk_dev_id is None:
        return cap.error("no ALSA capture or playback device available")

    if mic_dev_id is not None:
        mic_driver = MicDriver(
            device_id=mic_dev_id,
            sample_rate=int(os.environ.get("AUDIO_MIC_SAMPLE_RATE", "16000")),
            channels=int(os.environ.get("AUDIO_MIC_CHANNELS", "1")),
            bits_per_sample=int(os.environ.get("AUDIO_MIC_BITS", "16")),
            chunk_duration_s=int(os.environ.get("AUDIO_MIC_CHUNK_MS", "100")) / 1000.0,
        )
        current_input_id = mic_dev_id

    if spk_dev_id is not None:
        speaker_driver = SpeakerDriver(
            device_id=spk_dev_id,
            sample_rate=int(os.environ.get("AUDIO_SPEAKER_SAMPLE_RATE", "24000")),
            channels=int(os.environ.get("AUDIO_SPEAKER_CHANNELS", "1")),
            bits_per_sample=int(os.environ.get("AUDIO_SPEAKER_BITS", "16")),
        )
        current_output_id = spk_dev_id
    return cap.ready()


def main() -> int:
    cap.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
