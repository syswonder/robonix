# SPDX-License-Identifier: MulanPSL-2.0
"""Generic primitive-poller — finds the first cap on atlas that
satisfies a contract, opens an MCP or gRPC channel, and calls the
configured fetch function on a periodic tick. Each instance is one
asyncio task.

We use atlas's QueryCapabilities → ConnectCapability dance so the
poller doesn't need to know endpoints up front; if a primitive
restarts on a different port, the next reconnect picks up the new
endpoint automatically.
"""
from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Optional

import grpc

log = logging.getLogger(__name__)


@dataclass
class _AtlasIface:
    """Resolved atlas channel + transport for one contract."""
    cap_id: str
    contract_id: str
    transport: int
    endpoint: str
    params_json: str


class _AtlasMixin:
    """Shared QueryCapabilities helper. Subclasses inject the atlas
    stub via __init__."""

    def __init__(self, atlas_stub, pb) -> None:
        self.atlas = atlas_stub
        self.pb = pb

    async def _resolve(self, contract_id: str, *, transport: int) -> Optional[_AtlasIface]:
        """First-cap-wins resolution. Returns None when nothing
        matches (Soma adapter will then skip the corresponding poller).
        """
        loop = asyncio.get_running_loop()
        try:
            resp = await loop.run_in_executor(
                None,
                lambda: self.atlas.QueryCapabilities(self.pb.QueryCapabilitiesRequest(
                    contract_id=contract_id,
                    transport=transport,
                )),
            )
        except grpc.RpcError as e:
            log.warning("[scene-ingest] QueryCapabilities(%s) failed: %s", contract_id, e)
            return None
        if not resp.capabilities:
            return None
        cap = resp.capabilities[0]
        # ConnectCapability gives us the actual endpoint + params.
        try:
            cresp = await loop.run_in_executor(
                None,
                lambda: self.atlas.ConnectCapability(self.pb.ConnectCapabilityRequest(
                    consumer_id="com.robonix.system.scene",
                    capability_id=cap.capability_id,
                    contract_id=contract_id,
                    transport=transport,
                )),
            )
        except grpc.RpcError as e:
            log.warning("[scene-ingest] ConnectCapability(%s) failed: %s", contract_id, e)
            return None
        return _AtlasIface(
            cap_id=cap.capability_id,
            contract_id=contract_id,
            transport=transport,
            endpoint=cresp.endpoint,
            params_json=cresp.params.mcp.input_schema_json if hasattr(cresp.params, "mcp") else "",
        )


class PrimitivePoller(_AtlasMixin):
    """Periodic MCP-tool poller. Subclasses provide:

      * `contract_id` — what to look up on atlas
      * `_fetch_once(endpoint)` — coroutine returning the parsed result
      * `_apply(result)` — coroutine that writes into the registry

    The poll loop catches all exceptions; a failed tick is logged and
    the next tick still runs. Cancellation is async-clean."""

    def __init__(
        self,
        *,
        atlas_stub,
        pb,
        contract_id: str,
        period_s: float,
        on_result: Callable[[Any], Awaitable[None]],
        name: str | None = None,
    ) -> None:
        super().__init__(atlas_stub, pb)
        self.contract_id = contract_id
        self.period_s = period_s
        self.on_result = on_result
        self.name = name or contract_id
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()
        self._iface: Optional[_AtlasIface] = None

    async def start(self) -> None:
        # Don't fail boot if a primitive is missing — silently skip.
        self._iface = await self._resolve(self.contract_id, transport=self.pb.TRANSPORT_MCP)
        if self._iface is None:
            log.info("[scene-ingest] no MCP cap for %s; skipping", self.contract_id)
            return
        self._stop.clear()
        self._task = asyncio.create_task(self._run(), name=f"scene-ingest-{self.name}")
        log.info("[scene-ingest] %s polling every %.1fs (endpoint=%s)",
                 self.name, self.period_s, self._iface.endpoint)

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except (asyncio.CancelledError, Exception):  # noqa: BLE001
                pass
            self._task = None

    async def _run(self) -> None:
        while not self._stop.is_set():
            try:
                result = await self._fetch_once()
                if result is not None:
                    await self.on_result(result)
            except Exception as e:  # noqa: BLE001
                log.warning("[scene-ingest] %s tick failed: %s", self.name, e)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=self.period_s)
            except asyncio.TimeoutError:
                pass

    async def _fetch_once(self) -> Any:
        """Default impl: HTTP MCP `tools/call` with empty arguments. Override
        when the tool needs args. Returns the decoded JSON `result.content`."""
        import httpx
        assert self._iface is not None
        url = self._iface.endpoint.rstrip("/")
        # FastMCP serves JSON-RPC at <endpoint>/mcp/. The tools/call
        # method expects a body of {"jsonrpc": "2.0", "id": ...,
        # "method": "tools/call", "params": {"name": <leaf>, "arguments": {}}}.
        leaf = self.contract_id.rsplit("/", 1)[-1]
        body = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {"name": leaf, "arguments": {}},
        }
        async with httpx.AsyncClient(timeout=10.0) as client:
            r = await client.post(url, json=body, headers={"Accept": "application/json, text/event-stream"})
            r.raise_for_status()
            return r.json()


class ChassisStatePoller(PrimitivePoller):
    """Polls chassis/state MCP and feeds robot pose into the registry
    as a self-tracked SceneObject. The robot record never goes
    `missing=True`; if chassis stops responding we just stop updating."""

    def __init__(self, *, atlas_stub, pb, period_s: float, on_pose: Callable[[float, float, float, float], Awaitable[None]]) -> None:
        super().__init__(
            atlas_stub=atlas_stub, pb=pb,
            contract_id="robonix/primitive/chassis/state",
            period_s=period_s,
            on_result=self._on_state,
            name="chassis_state",
        )
        self._on_pose = on_pose

    async def _on_state(self, payload: dict) -> None:
        """`payload` is the raw FastMCP JSON-RPC response. The actual
        chassis state lives in result.content[0].text as JSON."""
        try:
            content = payload.get("result", {}).get("content", [])
            if not content:
                return
            text = content[0].get("text", "{}")
            state = json.loads(text)
            base = state.get("base_pose", {})
            pose = base.get("pose", {}).get("pose", {})
            pos = pose.get("position", {})
            x, y, z = float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), float(pos.get("z", 0.0))
            ori = pose.get("orientation", {})
            yaw = _quat_to_yaw(
                float(ori.get("x", 0.0)), float(ori.get("y", 0.0)),
                float(ori.get("z", 0.0)), float(ori.get("w", 1.0)),
            )
            await self._on_pose(x, y, z, yaw)
        except Exception as e:  # noqa: BLE001
            log.debug("[scene-ingest] chassis_state parse failed: %s", e)


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Quaternion → yaw (rotation about z). Standard ZYX convention."""
    import math
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
