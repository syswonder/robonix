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
import threading

from robonix_api import Primitive, Ok, Err, Deferred

audio_driver = Primitive(id="audio_driver", namespace="robonix/primitive/audio")
log = logging.getLogger("audio-driver")

import audio_pb2          # type: ignore  # noqa: E402  (codegen)
import std_msgs_pb2       # type: ignore  # noqa: E402

from audio_driver.alsa_utils import scan_alsa_devices, find_default_mic, find_default_speaker  # noqa: E402
from audio_driver.mic_driver import MicDriver, probe_mic_sample_rate  # noqa: E402
from audio_driver.speaker_driver import SpeakerDriver  # noqa: E402

mic_driver: MicDriver | None = None
speaker_driver: SpeakerDriver | None = None
# Currently-selected device ids (mirror of what mic_driver/speaker_driver
# point at). Empty string = OS / ALSA default. Set by SelectAudioDevice;
# read back by ListAudioDevices.current_*_id.
current_input_id: str = ""
current_output_id: str = ""
configured_input_id: str = ""
configured_output_id: str = ""
_mic_stream_lock = threading.Lock()


# ── streaming handlers ─────────────────────────────────────────────────────
@audio_driver.grpc("robonix/primitive/audio/mic")
def mic_stream(request, context):
    """Server-streaming mic capture with exclusive ALSA ownership.

    One ``MicDriver`` owns one ``arecord`` process. Reject overlapping clients
    explicitly instead of replacing the shared process handle and leaking the
    previous recorder.
    """
    grpc = __import__("grpc")
    if not _mic_stream_lock.acquire(blocking=False):
        context.abort(
            grpc.StatusCode.RESOURCE_EXHAUSTED,
            "microphone is already in use by another stream",
        )
        return
    driver = None
    try:
        driver = mic_driver
        if driver is None:
            context.abort(
                grpc.StatusCode.UNAVAILABLE,
                "mic driver not initialized — Driver(CMD_INIT) failed or never ran",
            )
            return
        log.info("mic stream client connected")
        driver.start()
        while context.is_active():
            chunk = driver.read_chunk()
            if chunk is None:
                break
            yield audio_pb2.AudioChunk(
                timestamp_ns=chunk["timestamp_ns"],
                data=chunk["data"],
                sequence=chunk["sequence"],
                duration_s=chunk["duration_s"],
            )
    finally:
        try:
            if driver is not None:
                driver.stop()
                log.info("mic stream client disconnected")
        finally:
            _mic_stream_lock.release()


@audio_driver.grpc("robonix/primitive/audio/speaker")
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
    ids = {d.id for d in devs}
    # ALSA plugin devices such as plughw:0,0 are intentionally absent from
    # arecord/aplay -l. Expose the *active* deployment selections as first-class
    # choices so a client refresh does not silently replace them with hw:0,0.
    configured: dict[str, set[str]] = {}
    for device_id, kind in (
        (current_input_id, "input"),
        (current_output_id, "output"),
        (configured_input_id, "input"),
        (configured_output_id, "output"),
        (os.environ.get("AUDIO_MIC_DEVICE", "").strip(), "input"),
        (os.environ.get("AUDIO_SPEAKER_DEVICE", "").strip(), "output"),
    ):
        if device_id:
            configured.setdefault(device_id, set()).add(kind)
    for device_id, kinds in configured.items():
        if device_id in ids:
            continue
        kind = "duplex" if kinds == {"input", "output"} else next(iter(kinds))
        devs.append(audio_pb2.AudioDevice(
            id=device_id,
            name="Configured ALSA device",
            kind=kind,
            is_default=False,
            channels=1,
            note="active deployment selection",
        ))
        ids.add(device_id)
    return devs


@audio_driver.grpc("robonix/primitive/audio/list_devices")
def list_devices(request, context):
    return audio_pb2.ListAudioDevices_Response(
        devices=_scan_audio_devices_proto(),
        current_input_id=current_input_id,
        current_output_id=current_output_id,
    )


@audio_driver.grpc("robonix/primitive/audio/select_device")
def select_device(request, context):
    global mic_driver, speaker_driver, current_input_id, current_output_id
    kind = (request.kind or "").lower()
    if kind not in ("input", "output"):
        return audio_pb2.SelectAudioDevice_Response(
            ok=False, error=f"kind must be 'input' or 'output', got '{kind}'")

    requested = request.id
    # "" means revert to default; otherwise ensure the id exists. ALSA plugin
    # names configured via env (for example "null") are valid even though
    # `arecord -l` / `aplay -l` do not list them as hardware cards.
    if requested:
        valid = {d.device_id for d in scan_alsa_devices()
                 if (d.is_input if kind == "input" else d.is_output)}
        configured = os.environ.get(
            "AUDIO_MIC_DEVICE" if kind == "input" else "AUDIO_SPEAKER_DEVICE",
            "",
        ).strip()
        if configured:
            valid.add(configured)
        current = current_input_id if kind == "input" else current_output_id
        if current:
            valid.add(current)
        deployment_configured = configured_input_id if kind == "input" else configured_output_id
        if deployment_configured:
            valid.add(deployment_configured)
        if requested not in valid:
            return audio_pb2.SelectAudioDevice_Response(
                ok=False, error=f"unknown {kind} id '{requested}'")
        new_id = requested
    else:
        info = (
            find_default_mic(scan_alsa_devices())
            if kind == "input"
            else find_default_speaker(scan_alsa_devices())
        )
        if info is None:
            return audio_pb2.SelectAudioDevice_Response(
                ok=False, error=f"no default {kind} device")
        new_id = info.device_id

    if kind == "input":
        if not _mic_stream_lock.acquire(blocking=False):
            return audio_pb2.SelectAudioDevice_Response(
                ok=False,
                error="cannot change input device while microphone is streaming",
            )
        try:
            previous = mic_driver
            if previous is not None:
                try:
                    previous.stop()
                except Exception:  # noqa: BLE001
                    pass
            # Auto-probe hardware sample rate unless explicitly overridden
            mic_rate = (
                int(os.environ["AUDIO_MIC_SAMPLE_RATE"])
                if "AUDIO_MIC_SAMPLE_RATE" in os.environ
                else previous.sample_rate if previous is not None
                else probe_mic_sample_rate(new_id)
            )
            mic_driver = MicDriver(
                device_id=new_id,
                sample_rate=mic_rate,
                channels=int(os.environ.get(
                    "AUDIO_MIC_CHANNELS",
                    str(previous.channels if previous is not None else 1),
                )),
                bits_per_sample=int(os.environ.get(
                    "AUDIO_MIC_BITS",
                    str(previous.bits_per_sample if previous is not None else 16),
                )),
                chunk_duration_s=int(os.environ.get(
                    "AUDIO_MIC_CHUNK_MS",
                    str(round((previous.chunk_duration_s if previous is not None else 0.1) * 1000)),
                )) / 1000.0,
            )
            current_input_id = new_id
        finally:
            _mic_stream_lock.release()
    else:
        previous = speaker_driver
        speaker_driver = SpeakerDriver(
            device_id=new_id,
            sample_rate=int(os.environ.get(
                "AUDIO_SPEAKER_SAMPLE_RATE",
                str(previous.sample_rate if previous is not None else 24000),
            )),
            channels=int(os.environ.get(
                "AUDIO_SPEAKER_CHANNELS",
                str(previous.channels if previous is not None else 1),
            )),
            bits_per_sample=int(os.environ.get(
                "AUDIO_SPEAKER_BITS",
                str(previous.bits_per_sample if previous is not None else 16),
            )),
        )
        current_output_id = new_id
    return audio_pb2.SelectAudioDevice_Response(ok=True, error="")


# ── driver-init lifecycle ──────────────────────────────────────────────────
@audio_driver.on_init
def init(cfg):
    """Configure ALSA from the primitive config, then env, then auto-detect.

    ``mic_device`` and ``speaker_device`` are deploy-level hardware choices;
    the corresponding ``AUDIO_*`` environment variables remain a backwards
    compatible operator override for older manifests.
    """
    global mic_driver, speaker_driver, current_input_id, current_output_id
    global configured_input_id, configured_output_id

    devices = scan_alsa_devices()
    for d in devices:
        log.info("ALSA: %s (%s) in=%s out=%s",
                 d.device_id, d.name, d.is_input, d.is_output)

    mic_id = str(cfg.get("mic_device") or os.environ.get("AUDIO_MIC_DEVICE", "")).strip()
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

    spk_id = str(cfg.get("speaker_device") or os.environ.get("AUDIO_SPEAKER_DEVICE", "")).strip()
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
        return Err("no ALSA capture or playback device available")

    if mic_dev_id is not None:
        configured_mic_rate = cfg.get("mic_sample_rate") or os.environ.get("AUDIO_MIC_SAMPLE_RATE")
        mic_rate = int(configured_mic_rate) if configured_mic_rate else probe_mic_sample_rate(mic_dev_id)
        mic_driver = MicDriver(
            device_id=mic_dev_id,
            sample_rate=mic_rate,
            channels=int(cfg.get("mic_channels") or os.environ.get("AUDIO_MIC_CHANNELS", "1")),
            bits_per_sample=int(cfg.get("mic_bits") or os.environ.get("AUDIO_MIC_BITS", "16")),
            chunk_duration_s=int(cfg.get("mic_chunk_ms") or os.environ.get("AUDIO_MIC_CHUNK_MS", "100")) / 1000.0,
        )
        current_input_id = mic_dev_id
        configured_input_id = mic_dev_id

    if spk_dev_id is not None:
        speaker_driver = SpeakerDriver(
            device_id=spk_dev_id,
            sample_rate=int(cfg.get("speaker_sample_rate") or os.environ.get("AUDIO_SPEAKER_SAMPLE_RATE", "24000")),
            channels=int(cfg.get("speaker_channels") or os.environ.get("AUDIO_SPEAKER_CHANNELS", "1")),
            bits_per_sample=int(cfg.get("speaker_bits") or os.environ.get("AUDIO_SPEAKER_BITS", "16")),
        )
        current_output_id = spk_dev_id
        configured_output_id = spk_dev_id
    return Ok()


@audio_driver.on_shutdown
def shutdown():
    global mic_driver, speaker_driver
    if mic_driver is not None:
        try:
            mic_driver.stop()
        except Exception:
            pass
        mic_driver = None
    if speaker_driver is not None:
        try:
            speaker_driver.stop()
        except Exception:
            pass
        speaker_driver = None
    return Ok()


def main() -> int:
    audio_driver.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
