#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Enroll a voiceprint through the same audio path used by the demo.

The script discovers the active audio mic primitive and voiceprint enroll
service through Atlas, records 16 kHz mono PCM from the mic, then calls
robonix/service/voiceprint/enroll. It is intentionally small demo tooling:
no new public contract, no bespoke audio path.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import grpc
from google.protobuf.empty_pb2 import Empty


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _add_paths(repo: Path) -> None:
    sys.path.insert(0, str(repo / "pylib" / "robonix-api"))
    sys.path.insert(0, str(repo / "services" / "voiceprint" / "rbnx-build" / "codegen" / "proto_gen"))


def _connect_contract(contract_id: str, provider_id: str = ""):
    from robonix_api import ATLAS
    from robonix_api.atlas_types import Transport

    providers = ATLAS.query(contract_id=contract_id, transport=Transport.GRPC)
    if provider_id:
        providers = [p for p in providers if p.id == provider_id or p.namespace == provider_id]
    if not providers:
        raise RuntimeError(f"no provider for {contract_id!r}")
    provider = providers[0]
    channel = ATLAS.connect_capability(
        consumer_id="voiceprint-enroll-demo",
        provider_id=provider.id,
        contract_id=contract_id,
        transport=Transport.GRPC,
    )
    endpoint = channel.endpoint
    endpoint = endpoint.removeprefix("http://").removeprefix("https://")
    endpoint = endpoint.replace("localhost", "127.0.0.1")
    return channel, endpoint


def _record_pcm(seconds: float, mic_provider: str) -> bytes:
    import robonix_contracts_pb2_grpc as contracts_grpc

    channel, endpoint = _connect_contract("robonix/primitive/audio/mic", mic_provider)
    try:
        stub = contracts_grpc.RobonixPrimitiveAudioMicStub(grpc.insecure_channel(endpoint))
        deadline = time.monotonic() + seconds
        chunks: list[bytes] = []
        for chunk in stub.Mic(Empty()):
            chunks.append(bytes(chunk.data))
            if time.monotonic() >= deadline:
                break
        return b"".join(chunks)
    finally:
        channel.close()


def _enroll(user_id: str, user_name: str, pcm: bytes, voiceprint_provider: str) -> None:
    import robonix_contracts_pb2_grpc as contracts_grpc
    import voiceprint_pb2 as vp

    channel, endpoint = _connect_contract("robonix/service/voiceprint/enroll", voiceprint_provider)
    try:
        stub_cls = getattr(contracts_grpc, "RobonixServiceVoiceprintEnrollStub", None)
        if stub_cls is None:
            stub_cls = getattr(contracts_grpc, "RobonixSystemSpeechVoiceprintEnrollStub", None)
        if stub_cls is None:
            raise RuntimeError("voiceprint enroll stub not found in robonix_contracts_pb2_grpc")
        stub = stub_cls(grpc.insecure_channel(endpoint))
        resp = stub.Enroll(vp.Enroll_Request(
            user_id=user_id,
            user_name=user_name,
            audio_data=pcm,
            encoding="pcm_s16le",
            sample_rate_hz=16000,
        ))
    finally:
        channel.close()

    if not resp.success:
        raise RuntimeError(resp.error or "voiceprint enroll failed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user-id", required=True)
    parser.add_argument("--user-name", default="")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--mic-provider", default="")
    parser.add_argument("--voiceprint-provider", default="")
    args = parser.parse_args()

    repo = _repo_root()
    _add_paths(repo)

    name = args.user_name or args.user_id
    print(f"Recording {args.seconds:.1f}s for user_id={args.user_id!r}. Speak now...")
    pcm = _record_pcm(args.seconds, args.mic_provider)
    if len(pcm) < 16000 * 2:
        raise RuntimeError(f"recorded only {len(pcm)} bytes; need at least ~1s of PCM")
    print(f"Captured {len(pcm)} bytes; enrolling...")
    _enroll(args.user_id, name, pcm, args.voiceprint_provider)
    print("Enroll success.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
