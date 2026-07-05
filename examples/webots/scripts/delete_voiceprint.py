#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Delete an enrolled voiceprint user through Atlas discovery."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import grpc


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
        consumer_id="voiceprint-delete-demo",
        provider_id=provider.id,
        contract_id=contract_id,
        transport=Transport.GRPC,
    )
    endpoint = channel.endpoint
    endpoint = endpoint.removeprefix("http://").removeprefix("https://")
    endpoint = endpoint.replace("localhost", "127.0.0.1")
    return channel, endpoint


def _delete(user_id: str, voiceprint_provider: str) -> None:
    import robonix_contracts_pb2_grpc as contracts_grpc
    import voiceprint_pb2 as vp

    channel, endpoint = _connect_contract("robonix/service/voiceprint/delete", voiceprint_provider)
    try:
        stub = contracts_grpc.RobonixServiceVoiceprintDeleteStub(grpc.insecure_channel(endpoint))
        resp = stub.DeleteEnrolled(vp.DeleteEnrolled_Request(user_id=user_id))
    finally:
        channel.close()

    if not resp.success:
        raise RuntimeError(resp.error or "voiceprint delete failed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user-id", required=True)
    parser.add_argument("--voiceprint-provider", default="")
    args = parser.parse_args()

    repo = _repo_root()
    _add_paths(repo)
    _delete(args.user_id, args.voiceprint_provider)
    print(f"Delete success: user_id={args.user_id!r}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
