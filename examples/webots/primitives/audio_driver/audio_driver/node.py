#!/usr/bin/env python3
"""Audio driver node — the primitive-layer audio primitive for Robonix.

Architecture position: robonix/primitive/audio (primitive / hardware layer)
  - Sits BELOW the srv layer (speech_service) and ABOVE the hardware
  - Auto-discovers ALSA audio devices (microphones and speakers)
  - Provides two gRPC interfaces for audio I/O:
      1. PrimitiveAudioMic.Stream    — server-streaming, outputs mic audio
      2. PrimitiveAudioSpeaker.Stream — client-streaming, plays received audio

This node follows the **tiago_bridge pattern** for primitive driver packages:
  1. Auto-scan ALSA devices (arecord -l / aplay -l)
  2. RegisterNode with Atlas control plane
  3. DeclareInterface for mic + speaker (2 contracts)
  4. Start daemon threads:
     - Heartbeat thread (15s interval to Atlas)
     - Mic gRPC server thread (PrimitiveAudioMicServicer)
     - Speaker gRPC server thread (PrimitiveAudioSpeakerServicer)
  5. Main thread blocks until KeyboardInterrupt

Proto stubs are loaded from codegen-generated files:
  - robonix_contracts_pb2_grpc  (servicer base classes + registration fns)
  - robonix_msg_pb2             (AudioChunk message)

This package is designed as a **REFERENCE TEMPLATE** for all future primitive
driver packages. The same pattern should be replicated for camera,
lidar, IMU, and other hardware drivers.

Environment variables (all optional — sensible defaults provided):
  ROBONIX_ATLAS              Atlas control-plane address (default: localhost:50051)
  ROBONIX_NODE_ID            Node ID for Atlas registration (default: com.robonix.primitive.audio)
  AUDIO_MIC_DEVICE           Override mic ALSA device (default: auto-detect via arecord -l)
  AUDIO_MIC_SAMPLE_RATE      Mic capture sample rate in Hz (default: 16000)
  AUDIO_MIC_CHANNELS         Mic capture channels (default: 1 = mono)
  AUDIO_MIC_BITS             Mic bits per sample (default: 16)
  AUDIO_MIC_CHUNK_MS         Mic chunk duration in milliseconds (default: 100)
  AUDIO_MIC_PORT             Mic gRPC server port (default: 0 = auto-pick)
  AUDIO_SPEAKER_DEVICE       Override speaker ALSA device (default: auto-detect)
  AUDIO_SPEAKER_SAMPLE_RATE  Speaker playback rate in Hz (default: 24000)
  AUDIO_SPEAKER_CHANNELS     Speaker channels (default: 1)
  AUDIO_SPEAKER_BITS         Speaker bits per sample (default: 16)
  AUDIO_SPEAKER_PORT         Speaker gRPC server port (default: 0 = auto-pick)
  AUDIO_DRIVER_STANDALONE    Set to "1" or "true" to skip Atlas registration
"""
import json
import os
import sys
import time
import logging
import threading
from concurrent import futures
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="[audio-driver] %(levelname)s %(message)s")
log = logging.getLogger(__name__)

# ── Proto stub resolution ──────────────────────────────────────────────────
# Walks up from this file's directory looking for proto_gen/ containing the
# codegen-generated robonix_contracts_pb2.py. Adds it to sys.path so the
# proto imports work regardless of the current working directory.

def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_contracts_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent

_ensure_proto_gen()

import grpc
# dev-packaging codegen split the old monolithic robonix_msg_pb2 into
# per-IDL-package modules. AudioChunk lives in audio_pb2 (from
# audio/msg/AudioChunk.msg), Empty in std_msgs_pb2.
import audio_pb2
import std_msgs_pb2
import robonix_contracts_pb2_grpc as contracts_grpc

from audio_driver.alsa_utils import scan_alsa_devices, find_default_mic, find_default_speaker
from audio_driver.mic_driver import MicDriver
from audio_driver.speaker_driver import SpeakerDriver

# ── gRPC Servicers ────────────────────────────────────────────────────────
# Each interface (mic, speaker) gets its own servicer class that wraps
# the corresponding driver. The servicers are registered on separate gRPC
# servers running in daemon threads.

class MicServicer(contracts_grpc.PrimitiveAudioMicServicer):
    """Implements contract robonix/primitive/audio/mic — server-streaming mic audio.

    When a client calls Stream():
      1. Starts the MicDriver (arecord subprocess)
      2. Loops reading chunks from the driver
      3. Wraps each chunk as an AudioChunk proto and yields it
      4. On client disconnect, stops the MicDriver

    Thread safety: One MicDriver per servicer instance. Multiple clients
    connecting simultaneously will share the same mic stream (second client
    gets the already-running stream). For exclusive access, the gRPC layer
    should be configured with one stream at a time.
    """

    def __init__(self, mic_driver: MicDriver):
        self.mic_driver = mic_driver

    def Stream(self, request, context):
        log.info("Mic stream client connected")
        self.mic_driver.start()
        try:
            while context.is_active():
                chunk = self.mic_driver.read_chunk()
                if chunk is None:
                    break
                yield audio_pb2.AudioChunk(
                    timestamp_ns=chunk["timestamp_ns"],
                    data=chunk["data"],
                    sequence=chunk["sequence"],
                    duration_s=chunk["duration_s"],
                )
        finally:
            self.mic_driver.stop()
            log.info("Mic stream client disconnected")


class SpeakerServicer(contracts_grpc.PrimitiveAudioSpeakerServicer):
    """Implements contract robonix/primitive/audio/speaker — client-streaming playback.

    When a client calls Stream() with a stream of AudioChunk messages:
      1. For each chunk with data, calls speaker_driver.play_chunk()
      2. On stream end, returns google.protobuf.Empty

    The SpeakerDriver handles lazy-starting aplay and auto-restart on errors.
    """

    def __init__(self, speaker_driver: SpeakerDriver):
        self.speaker_driver = speaker_driver

    def Stream(self, request_iterator, context):
        log.info("Speaker stream client connected")
        try:
            for chunk in request_iterator:
                if chunk.data:
                    self.speaker_driver.play_chunk(bytes(chunk.data))
        finally:
            log.info("Speaker stream client disconnected")

        return std_msgs_pb2.Empty()


# ── Atlas registration (optional) ─────────────────────────────────────────
# Follows the tiago_bridge pattern exactly:
#   1. RegisterNode — register as a primitive under robonix/primitive/audio
#   2. DeclareInterface x 2 — declare mic and speaker endpoints
#   3. Heartbeat thread — send NodeHeartbeat every 15 seconds

def _register_with_atlas(mic_port: int, speaker_port: int) -> None:
    """Register this node with the Atlas control plane. Non-fatal on failure.

    If robonix_runtime_pb2 is not importable (standalone deployment), or
    if Atlas is unreachable, logs a warning and continues. The driver is
    fully functional without Atlas — it just won't be discoverable.

    Registration details:
      - Node ID: com.robonix.primitive.audio (configurable via ROBONIX_NODE_ID)
      - Namespace: robonix/primitive/audio
      - Kind: primitive (hardware driver)
      - Interface 1: "mic" — contract robonix/primitive/audio/mic, server-stream
      - Interface 2: "speaker" — contract robonix/primitive/audio/speaker, client-stream

    Args:
        mic_port: gRPC port the mic server is listening on.
        speaker_port: gRPC port the speaker server is listening on.
    """
    try:
        import robonix_runtime_pb2 as rpb
        import robonix_runtime_pb2_grpc as rpb_grpc
    except ImportError:
        log.warning("robonix_runtime_pb2 not found, skipping Atlas registration")
        return

    atlas_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    log.info("Connecting to Atlas at %s ...", atlas_addr)

    try:
        channel = grpc.insecure_channel(atlas_addr)
        stub = rpb_grpc.RobonixRuntimeStub(channel)
        node_id = os.environ.get("ROBONIX_NODE_ID", "com.robonix.primitive.audio")

        stub.RegisterNode(rpb.RegisterNodeRequest(
            node_id=node_id,
            namespace="robonix/primitive/audio",
            kind="primitive",
        ))
        log.info("Registered node %s with Atlas", node_id)

        # Mic interface
        stub.DeclareInterface(rpb.DeclareInterfaceRequest(
            node_id=node_id, name="mic",
            supported_transports=["grpc"],
            metadata_json=json.dumps({
                "transport": "grpc",
                "contract": {"idl_type": "protobuf", "mode": "server_stream"},
            }),
            listen_port=mic_port,
            contract_id="robonix/primitive/audio/mic",
        ))
        log.info("Declared robonix/primitive/audio/mic on port %d", mic_port)

        # Speaker interface
        stub.DeclareInterface(rpb.DeclareInterfaceRequest(
            node_id=node_id, name="speaker",
            supported_transports=["grpc"],
            metadata_json=json.dumps({
                "transport": "grpc",
                "contract": {"idl_type": "protobuf", "mode": "client_stream"},
            }),
            listen_port=speaker_port,
            contract_id="robonix/primitive/audio/speaker",
        ))
        log.info("Declared robonix/primitive/audio/speaker on port %d", speaker_port)

        # Heartbeat
        def _heartbeat():
            while True:
                time.sleep(15.0)
                try:
                    stub.NodeHeartbeat(rpb.NodeHeartbeatRequest(node_id=node_id))
                except Exception as e:
                    log.warning("Heartbeat failed: %s", e)

        threading.Thread(target=_heartbeat, daemon=True).start()
        channel.close()

    except grpc.RpcError as e:
        log.warning("Atlas not available (%s), running standalone", e.code())


# ── Main ──────────────────────────────────────────────────────────────────
# Entry point. Called via `python -m audio_driver.node` or via the
# robonix_manifest.yaml start command.

def main() -> None:
    """Start the audio driver node.

    Startup sequence:
        1. Discover ALSA devices via scan_alsa_devices()
        2. Find mic and speaker devices (auto-detect or env override)
        3. Create MicDriver and SpeakerDriver instances
        4. Create a single gRPC server with both servicers
        5. Auto-pick free ports for mic and speaker
        6. Optionally register with Atlas (RegisterNode + DeclareInterface)
        7. Start the gRPC server
        8. Block main thread until KeyboardInterrupt
    """
    log.info("Starting audio driver")

    # 1. Discover ALSA devices
    devices = scan_alsa_devices()
    for d in devices:
        log.info("Found ALSA device: %s (%s) in=%s out=%s",
                 d.device_id, d.name, d.is_input, d.is_output)

    mic_device = None
    mic_device_str = os.environ.get("AUDIO_MIC_DEVICE", "")
    if mic_device_str:
        mic_device = type("Dev", (), {"device_id": mic_device_str})()
        log.info("Mic device overridden: %s", mic_device_str)
    else:
        mic_info = find_default_mic(devices)
        if mic_info:
            mic_device = mic_info
            log.info("Auto-detected mic: %s (%s)", mic_info.device_id, mic_info.name)
        else:
            log.warning("No microphone found")

    speaker_device = None
    speaker_device_str = os.environ.get("AUDIO_SPEAKER_DEVICE", "")
    if speaker_device_str:
        speaker_device = type("Dev", (), {"device_id": speaker_device_str})()
        log.info("Speaker device overridden: %s", speaker_device_str)
    else:
        speaker_info = find_default_speaker(devices)
        if speaker_info:
            speaker_device = speaker_info
            log.info("Auto-detected speaker: %s (%s)", speaker_info.device_id, speaker_info.name)
        else:
            log.warning("No speaker found")

    # 2. Create drivers
    mic_driver = None
    if mic_device:
        mic_driver = MicDriver(
            device_id=mic_device.device_id,
            sample_rate=int(os.environ.get("AUDIO_MIC_SAMPLE_RATE", "16000")),
            channels=int(os.environ.get("AUDIO_MIC_CHANNELS", "1")),
            bits_per_sample=int(os.environ.get("AUDIO_MIC_BITS", "16")),
            chunk_duration_s=int(os.environ.get("AUDIO_MIC_CHUNK_MS", "100")) / 1000.0,
        )

    speaker_driver = None
    if speaker_device:
        speaker_driver = SpeakerDriver(
            device_id=speaker_device.device_id,
            sample_rate=int(os.environ.get("AUDIO_SPEAKER_SAMPLE_RATE", "24000")),
            channels=int(os.environ.get("AUDIO_SPEAKER_CHANNELS", "1")),
            bits_per_sample=int(os.environ.get("AUDIO_SPEAKER_BITS", "16")),
        )

    # 3. Pick ports
    mic_port = int(os.environ.get("AUDIO_MIC_PORT", "0"))
    speaker_port = int(os.environ.get("AUDIO_SPEAKER_PORT", "0"))

    if mic_port == 0:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("0.0.0.0", 0))
        mic_port = s.getsockname()[1]
        s.close()

    if speaker_port == 0:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("0.0.0.0", 0))
        speaker_port = s.getsockname()[1]
        s.close()

    # 4. Atlas registration (optional)
    standalone = os.environ.get("AUDIO_DRIVER_STANDALONE", "").strip() in ("1", "true")
    if not standalone:
        _register_with_atlas(mic_port, speaker_port)

    # 5. Start gRPC server(s)
    # Mic and speaker run on separate servers in daemon threads so they are
    # isolated — a problem with one does not affect the other.
    if mic_driver:
        def _run_mic_server():
            server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
            contracts_grpc.add_PrimitiveAudioMicServicer_to_server(
                MicServicer(mic_driver), server)
            server.add_insecure_port(f"0.0.0.0:{mic_port}")
            server.start()
            log.info("Mic gRPC server on port %d", mic_port)
            server.wait_for_termination()

        threading.Thread(target=_run_mic_server, daemon=True, name="mic-grpc").start()

    if speaker_driver:
        def _run_speaker_server():
            server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
            contracts_grpc.add_PrimitiveAudioSpeakerServicer_to_server(
                SpeakerServicer(speaker_driver), server)
            server.add_insecure_port(f"0.0.0.0:{speaker_port}")
            server.start()
            log.info("Speaker gRPC server on port %d", speaker_port)
            server.wait_for_termination()

        threading.Thread(target=_run_speaker_server, daemon=True, name="speaker-grpc").start()

    log.info("Audio driver ready (mic=%s, speaker=%s)",
             f"port {mic_port}" if mic_driver else "none",
             f"port {speaker_port}" if speaker_driver else "none")

    # 6. Block main thread
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
