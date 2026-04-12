#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""CI smoke test — verifies vlm_service's SrvCognitionReason.Stream contract
end-to-end without needing a real LLM API key.

Run **after** starting:
  1. robonix-atlas on $ROBONIX_ATLAS (default 127.0.0.1:50051)
  2. vlm_service registered to that atlas, with VLM_CI_MODE=1

This script:
  - Queries atlas for the vlm node's data endpoint.
  - Opens the Stream RPC and asserts we get the canned mock reply
    "[vlm-ci-mock] hello from robonix".
"""
import os
import sys
from pathlib import Path


def _ensure_proto_gen() -> None:
    """Find any package's proto_gen (they all share the same contract stubs)."""
    here = Path(__file__).resolve()
    for pkg in ("memsearch_service", "vlm_service"):
        cand = here.parents[1] / "packages" / pkg / "proto_gen"
        if cand.is_dir() and (cand / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(cand))
            return
    print("[ci-smoke] no package proto_gen found — run `rbnx codegen` on a package first", file=sys.stderr)
    sys.exit(2)


_ensure_proto_gen()

import grpc  # noqa: E402
import robonix_runtime_pb2 as pb  # noqa: E402
import robonix_runtime_pb2_grpc as pb_grpc  # noqa: E402
import robonix_contracts_pb2_grpc  # noqa: E402
import vlm_pb2  # noqa: E402


def main() -> int:
    atlas = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    print(f"[ci-smoke] atlas = {atlas}")

    ch = grpc.insecure_channel(atlas)
    stub = pb_grpc.RobonixRuntimeStub(ch)

    resp = stub.QueryNodes(pb.QueryNodesRequest(contract_id="robonix/srv/cognition/reason"))
    if not resp.nodes:
        print("[ci-smoke] FAIL: no node advertises robonix/srv/cognition/reason", file=sys.stderr)
        return 1

    node = resp.nodes[0]
    iface = next((i for i in node.interfaces if i.contract_id == "robonix/srv/cognition/reason"), None)
    if iface is None:
        print("[ci-smoke] FAIL: node has no matching interface", file=sys.stderr)
        return 1

    neg = stub.NegotiateChannel(pb.NegotiateChannelRequest(
        consumer_id="ci-smoke-client",
        provider_node_id=node.node_id,
        interface_name=iface.name,
        transport="grpc",
    ))
    endpoint = neg.endpoint
    print(f"[ci-smoke] vlm endpoint = {endpoint}")

    data_ch = grpc.insecure_channel(endpoint)
    vlm_stub = robonix_contracts_pb2_grpc.SrvCognitionReasonStub(data_ch)

    req = vlm_pb2.ChatStream_Request(
        messages=[
            vlm_pb2.ChatMessage(
                role="user",
                parts=[vlm_pb2.ContentPart(kind="text", text="ping")],
            ),
        ],
        max_tokens=32,
    )

    collected = []
    finish = None
    for event in vlm_stub.Stream(req, timeout=10.0):
        if event.text_delta:
            collected.append(event.text_delta)
        if event.finish_reason:
            finish = event.finish_reason

    full = "".join(collected)
    print(f"[ci-smoke] reply   = {full!r}")
    print(f"[ci-smoke] finish  = {finish!r}")

    expected = "[vlm-ci-mock] hello from robonix"
    if full != expected:
        print(f"[ci-smoke] FAIL: expected {expected!r}, got {full!r}", file=sys.stderr)
        return 1
    if finish != "stop":
        print(f"[ci-smoke] FAIL: expected finish=stop, got {finish!r}", file=sys.stderr)
        return 1

    print("[ci-smoke] OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
