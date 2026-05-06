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

from robonix_py import Capability

cap = Capability(id="com.robonix.primitive.audio", namespace="robonix/primitive/audio")
log = logging.getLogger("audio-driver")

import audio_pb2          # type: ignore  # noqa: E402  (codegen)
import std_msgs_pb2       # type: ignore  # noqa: E402

from audio_driver.alsa_utils import scan_alsa_devices, find_default_mic, find_default_speaker  # noqa: E402
from audio_driver.mic_driver import MicDriver  # noqa: E402
from audio_driver.speaker_driver import SpeakerDriver  # noqa: E402

mic_driver: MicDriver | None = None
speaker_driver: SpeakerDriver | None = None


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


# ── driver-init lifecycle ──────────────────────────────────────────────────
@cap.on_init
def init(cfg):
    """ALSA scan + device pickup. Honours AUDIO_{MIC,SPEAKER}_DEVICE env
    overrides; falls back to the auto-detector in alsa_utils. Refuses to
    come up if neither a mic nor a speaker is available, so atlas defers
    instead of advertising dead interfaces."""
    global mic_driver, speaker_driver

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

    if spk_dev_id is not None:
        speaker_driver = SpeakerDriver(
            device_id=spk_dev_id,
            sample_rate=int(os.environ.get("AUDIO_SPEAKER_SAMPLE_RATE", "24000")),
            channels=int(os.environ.get("AUDIO_SPEAKER_CHANNELS", "1")),
            bits_per_sample=int(os.environ.get("AUDIO_SPEAKER_BITS", "16")),
        )
    return cap.ready()


def main() -> int:
    cap.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
