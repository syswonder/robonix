# SPDX-License-Identifier: MulanPSL-2.0
"""Controllable MCP provider for client and Executor integration tests."""
import argparse
import asyncio
import json
import time
from pathlib import Path
from typing import Any

from mcp.server.fastmcp import FastMCP
from mcp.types import CallToolResult, TextContent
from navigation_mcp import GetNavigationSpeedLimit_Request, GetNavigationSpeedLimit_Response
from robonix_api.tool import mcp_contract

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, required=True)
parser.add_argument("--control", type=Path, required=True)
args = parser.parse_args()
app = FastMCP("channel-test", host="127.0.0.1", port=args.port,
              stateless_http=True)
runs = {}


def config():
    return json.loads(args.control.read_text())


def trace(name, run_id=""):
    with args.control.with_suffix(".trace").open("a") as output:
        output.write(json.dumps({"name": name, "run_id": run_id}) + "\n")


@app.tool()
async def get_mode() -> CallToolResult:
    trace("get_mode")
    settings = config()
    await asyncio.sleep(settings.get("delay", 0))
    if settings.get("tool_error"):
        raise RuntimeError("downstream FAILED_PRECONDITION: chassis busy")
    value = {"ok": True, "mode": "mapping", "detail": ""}
    return CallToolResult(
        content=[TextContent(type="text", text="invalid JSON" if settings.get("malformed") else json.dumps(value))],
        structuredContent=None if settings.get("text_only") or settings.get("malformed") else value,
    )


@app.tool()
async def get_robot_context() -> CallToolResult:
    trace("get_robot_context")
    settings = config()
    await asyncio.sleep(settings.get("delay", 0))
    value = settings.get("scene_response", {
        "pose_known": True, "stale": False, "map_id": "floor-3",
        "x": 1.0, "y": 2.0, "yaw": 0.0, "reason": "current",
    })
    return CallToolResult(
        content=[TextContent(type="text", text=settings.get("scene_text", json.dumps(value)))],
        structuredContent=None if settings.get("text_only") else value,
    )


@app.tool()
async def navigate(goal: dict | None = None) -> Any:
    trace("navigate")
    settings = config()
    await asyncio.sleep(settings.get("start_delay", 0))
    if settings.get("reject"):
        return {"accepted": False, "run_id": "", "detail": "chassis busy"}
    run_id = "" if settings.get("legacy") else f"run-{len(runs) + 1}"
    runs[run_id] = None
    return {"accepted": True, "run_id": run_id, "detail": "started"}


@app.tool()
async def status(run_id: str = "") -> Any:
    trace("status", run_id)
    settings = config()
    await asyncio.sleep(settings.get("status_delay", 0))
    if settings.get("status_error"):
        raise RuntimeError("status unavailable")
    if settings.get("invalid_status"):
        return CallToolResult(content=[TextContent(type="text", text='{"state":"UNKNOWN"}')])
    if settings.get("malformed_status"):
        return CallToolResult(content=[TextContent(type="text", text="invalid JSON")])
    if run_id not in runs:
        raise RuntimeError("unknown run")
    stop_at = runs.get(run_id)
    state = "RUNNING"
    if settings.get("finish"):
        state = "SUCCEEDED"
    elif stop_at is not None and time.monotonic() >= stop_at:
        state = "CANCELED"
    return {"known": True, "state": state, "detail": state.lower()}


@app.tool()
async def cancel(run_id: str = "") -> Any:
    trace("cancel", run_id)
    settings = config()
    if not settings.get("ignore_cancel"):
        runs[run_id] = time.monotonic() + settings.get("cancel_delay", 0)
    return {"accepted": True, "detail": "requested"}


@mcp_contract(app, contract_id="robonix/service/navigation/get_speed_limit")
def get_speed_limit(req: GetNavigationSpeedLimit_Request) -> GetNavigationSpeedLimit_Response:
    return GetNavigationSpeedLimit_Response(available=True, max_linear_speed_mps=0.26)


app.run(transport="streamable-http")
