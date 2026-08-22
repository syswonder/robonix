#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Activate and call the standard Robonix Explore skill for Webots tests."""

from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


def _inspect(rbnx: Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(rbnx), "inspect"],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(completed.stdout)


def _explore_endpoints(
    snapshot: dict[str, Any],
) -> tuple[str, str]:
    provider = (snapshot.get("providers") or {}).get("explore") or {}
    driver = ""
    mcp = ""
    for endpoint in provider.get("endpoints") or ():
        contract_id = str(endpoint.get("contract_id") or "")
        if contract_id == "robonix/lifecycle/driver":
            driver = str(endpoint.get("endpoint") or "")
        elif contract_id == "robonix/skill/explore/explore":
            mcp = str(endpoint.get("endpoint") or "")
    if not driver or not mcp:
        raise RuntimeError(
            "Explore is not fully registered: "
            f"driver={driver!r} mcp={mcp!r}"
        )
    return driver, mcp


def _activate(driver_endpoint: str, deploy_dir: Path) -> None:
    proto_root = (
        deploy_dir
        / "rbnx-boot/cache/skill-explore-rbnx"
        / "rbnx-build/codegen/explore_proto_gen"
    )
    if not (proto_root / "lifecycle_pb2.py").is_file():
        raise RuntimeError(f"Explore runtime codegen is missing: {proto_root}")
    sys.path.insert(0, str(proto_root))
    import grpc  # type: ignore[import-not-found]
    import lifecycle_pb2  # type: ignore[import-not-found]
    import robonix_contracts_pb2_grpc  # type: ignore[import-not-found]

    channel = grpc.insecure_channel(driver_endpoint)
    stub = robonix_contracts_pb2_grpc.RobonixLifecycleDriverStub(channel)
    response = stub.Driver(
        lifecycle_pb2.Driver_Request(command=1, config_json=""),
        timeout=60,
    )
    if not bool(response.ok):
        raise RuntimeError(
            "Explore Driver(ACTIVATE) failed: "
            f"state={response.state!r} error={response.error!r}"
        )


def _result_payload(result: Any) -> dict[str, Any]:
    structured = getattr(result, "structured_content", None)
    if isinstance(structured, dict):
        payload = structured.get("result", structured)
        if isinstance(payload, dict):
            return payload
    for item in getattr(result, "content", ()):
        text = getattr(item, "text", "")
        if text:
            payload = json.loads(text)
            if isinstance(payload, dict):
                return payload
    raise RuntimeError("Explore MCP returned no JSON object")


async def _call(endpoint: str, tool: str, arguments: dict[str, Any]) -> dict[str, Any]:
    from fastmcp import Client  # type: ignore[import-not-found]

    async with Client(endpoint) as client:
        return _result_payload(await client.call_tool(tool, arguments))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("ready", "start", "status", "cancel"))
    parser.add_argument("--rbnx", type=Path, required=True)
    parser.add_argument("--deploy-dir", type=Path, required=True)
    parser.add_argument("--run-id", default="")
    parser.add_argument("--area-hint", default="")
    parser.add_argument("--timeout-s", type=float, default=600.0)
    parser.add_argument("--max-speed-m-s", type=float, default=0.12)
    args = parser.parse_args()

    driver_endpoint, mcp_endpoint = _explore_endpoints(_inspect(args.rbnx))
    if args.action == "ready":
        payload = {
            "driver_endpoint": driver_endpoint,
            "mcp_endpoint": mcp_endpoint,
        }
    elif args.action == "start":
        _activate(driver_endpoint, args.deploy_dir)
        payload = asyncio.run(
            _call(
                mcp_endpoint,
                "explore",
                {
                    "area_hint": args.area_hint,
                    "timeout_s": args.timeout_s,
                    "max_speed_m_s": args.max_speed_m_s,
                },
            )
        )
        if not bool(payload.get("accepted")) or not payload.get("run_id"):
            raise RuntimeError(f"Explore rejected the benchmark run: {payload}")
    else:
        payload = asyncio.run(
            _call(mcp_endpoint, args.action, {"run_id": args.run_id})
        )
    print(json.dumps(payload, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
