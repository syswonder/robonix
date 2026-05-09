# SPDX-License-Identifier: MulanPSL-2.0
"""Thin wrapper over the generated atlas_pb2 stubs. Lazy-import (the stubs
live in each package's rbnx-build/codegen/proto_gen/, only on sys.path after
ensure_proto_gen() runs).

Returns dataclasses from `robonix_api.atlas_types` so callers never see raw
protobuf messages — keeps the consumer-side API stable across proto edits.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any

from .atlas_types import (
    CapabilityRecord,
    CapabilityState,
    Channel,
    ContractDescriptor,
    GrpcParams,
    McpParams,
    Ros2Params,
    Transport,
    from_pb_contract,
    from_pb_record,
)

log = logging.getLogger("robonix_api.atlas")


def _resolve_transport(t: Transport | str | int) -> Transport:
    if isinstance(t, Transport):
        return t
    if isinstance(t, int):
        return Transport(t)
    name = str(t).strip().lower()
    return {
        "ros2": Transport.ROS2,
        "ros":  Transport.ROS2,
        "grpc": Transport.GRPC,
        "mcp":  Transport.MCP,
        "":     Transport.UNSPECIFIED,
        "unspecified": Transport.UNSPECIFIED,
    }.get(name, Transport.UNSPECIFIED)


def _resolve_state(s: CapabilityState | str | int) -> CapabilityState:
    if isinstance(s, CapabilityState):
        return s
    if isinstance(s, int):
        return CapabilityState(s)
    name = str(s).strip().lower()
    return {
        "registered": CapabilityState.REGISTERED,
        "inactive":   CapabilityState.INACTIVE,
        "active":     CapabilityState.ACTIVE,
        "error":      CapabilityState.ERROR,
        "terminated": CapabilityState.TERMINATED,
    }.get(name, CapabilityState.UNSPECIFIED)


class AtlasClient:
    def __init__(self, endpoint: str = "127.0.0.1:50051") -> None:
        self._endpoint = endpoint
        self._channel: Any = None
        self._stub: Any = None
        self._pb: Any = None  # atlas_pb2 module

    def _ensure_stub(self) -> None:
        if self._stub is not None:
            return
        import grpc
        import atlas_pb2  # type: ignore
        import atlas_pb2_grpc  # type: ignore
        self._pb = atlas_pb2
        self._channel = grpc.insecure_channel(self._endpoint)
        self._stub = atlas_pb2_grpc.AtlasStub(self._channel)

    @property
    def pb(self):
        self._ensure_stub()
        return self._pb

    @property
    def stub(self):
        self._ensure_stub()
        return self._stub

    def transport_enum(self, name: Transport | str | int):
        """Translate a Transport / string / int to the protobuf enum value
        the generated stubs expect."""
        t = _resolve_transport(name)
        attr = {
            Transport.ROS2:        "TRANSPORT_ROS2",
            Transport.GRPC:        "TRANSPORT_GRPC",
            Transport.MCP:         "TRANSPORT_MCP",
            Transport.UNSPECIFIED: "TRANSPORT_UNSPECIFIED",
        }[t]
        return getattr(self.pb, attr)

    # ── registration ─────────────────────────────────────────────────────
    def register_capability(
        self, capability_id: str, namespace: str, capability_md_path: str = "",
    ) -> bool:
        """True if newly registered, False if already exists (idempotent re-deploy)."""
        import grpc
        try:
            self.stub.RegisterCapability(self.pb.RegisterCapabilityRequest(
                capability_id=capability_id,
                namespace=namespace,
                capability_md_path=capability_md_path,
            ))
            return True
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                return False
            raise

    def unregister_capability(self, capability_id: str) -> bool:
        try:
            resp = self.stub.UnregisterCapability(
                self.pb.UnregisterCapabilityRequest(capability_id=capability_id)
            )
            return bool(resp.was_present)
        except Exception as e:  # noqa: BLE001
            log.debug("UnregisterCapability(%s): %s", capability_id, e)
            return False

    # ── interface declares ───────────────────────────────────────────────
    def declare_ros2(
        self, capability_id: str, contract_id: str, topic: str,
        qos_profile: str = "best_effort",
    ) -> str:
        return self._declare(capability_id, contract_id, Transport.ROS2, endpoint=topic,
                             params=self.pb.TransportParams(
                                 ros2=self.pb.Ros2Params(qos_profile=qos_profile),
                             ))

    def declare_grpc(
        self, capability_id: str, contract_id: str, endpoint: str,
        service_name: str, method: str, proto_file: str = "robonix_contracts.proto",
    ) -> str:
        return self._declare(capability_id, contract_id, Transport.GRPC, endpoint=endpoint,
                             params=self.pb.TransportParams(
                                 grpc=self.pb.GrpcParams(
                                     proto_file=proto_file,
                                     service_name=service_name,
                                     method=method,
                                 ),
                             ))

    def declare_mcp(
        self, capability_id: str, contract_id: str, endpoint: str,
        description: str = "", input_schema_json: str = "{}",
    ) -> str:
        return self._declare(capability_id, contract_id, Transport.MCP, endpoint=endpoint,
                             params=self.pb.TransportParams(
                                 mcp=self.pb.McpParams(
                                     description=description,
                                     input_schema_json=input_schema_json,
                                 ),
                             ))

    def _declare(
        self, capability_id: str, contract_id: str, transport: Transport,
        endpoint: str, params,
    ) -> str:
        import grpc
        try:
            resp = self.stub.DeclareInterface(self.pb.DeclareInterfaceRequest(
                capability_id=capability_id,
                contract_id=contract_id,
                transport=self.transport_enum(transport),
                endpoint=endpoint,
                params=params,
            ))
            return resp.endpoint or endpoint
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                log.debug("declare %s/%s/%s already exists; ok",
                          capability_id, contract_id, transport.name)
                return endpoint
            raise

    # ── discovery ────────────────────────────────────────────────────────
    def get(self, capability_id: str) -> CapabilityRecord | None:
        """Look up a capability by its id. Returns None if not registered."""
        recs = self._query(capability_id=capability_id)
        return recs[0] if recs else None

    def find(
        self,
        contract_id: str,
        *,
        transport: Transport | str | int = Transport.UNSPECIFIED,
    ) -> list[CapabilityRecord]:
        """Find capabilities providing `contract_id`. Optional `transport`
        narrows to caps whose implementation of that contract uses the
        given transport (otherwise any transport). Always a list (possibly
        empty). For exactly-one-expected pattern, prefer tuple-unpack:
        `[rec] = atlas.find("X")` raises ValueError on 0 or >1 matches."""
        return self._query(
            contract_id=contract_id,
            transport=transport,
        )

    def _query(
        self,
        *,
        capability_id: str = "",
        contract_id: str = "",
        transport: Transport | str | int = Transport.UNSPECIFIED,
    ) -> list[CapabilityRecord]:
        import grpc
        t = self.transport_enum(transport)
        try:
            resp = self.stub.QueryCapabilities(self.pb.QueryCapabilitiesRequest(
                capability_id=capability_id,
                contract_id=contract_id,
                transport=t,
            ))
        except grpc.RpcError as e:
            log.warning(
                "QueryCapabilities(cap=%r, contract=%r): %s",
                capability_id, contract_id, e,
            )
            return []
        return [from_pb_record(r) for r in resp.records]

    def query_md(self, capability_id: str) -> str:
        try:
            resp = self.stub.QueryCapabilityMd(
                self.pb.QueryCapabilityMdRequest(capability_id=capability_id)
            )
            return resp.capability_md
        except Exception as e:  # noqa: BLE001
            log.debug("QueryCapabilityMd(%s): %s", capability_id, e)
            return ""

    def get_contract(self, contract_id: str) -> ContractDescriptor | None:
        import grpc
        try:
            resp = self.stub.QueryContract(
                self.pb.QueryContractRequest(contract_id=contract_id)
            )
        except grpc.RpcError as e:
            log.debug("QueryContract(%s): %s", contract_id, e)
            return None
        if not resp.found:
            return None
        return from_pb_contract(resp.contract)

    def list_contracts(self, namespace_prefix: str = "") -> list[ContractDescriptor]:
        import grpc
        try:
            resp = self.stub.ListContracts(
                self.pb.ListContractsRequest(namespace_prefix=namespace_prefix)
            )
        except grpc.RpcError as e:
            log.warning("ListContracts(prefix=%r): %s", namespace_prefix, e)
            return []
        return [from_pb_contract(c) for c in resp.contracts]

    def inspect(self) -> dict:
        try:
            resp = self.stub.InspectAtlas(self.pb.InspectAtlasRequest())
            return json.loads(resp.json) if resp.json else {}
        except Exception as e:  # noqa: BLE001
            log.debug("InspectAtlas: %s", e)
            return {}

    # ── channels ─────────────────────────────────────────────────────────
    def connect(
        self,
        *,
        consumer_id: str,
        capability_id: str,
        contract_id: str,
        transport: Transport | str | int,
    ) -> Channel:
        """Open a consumer→provider edge. Returns a `Channel` context
        manager — `with atlas.connect(...) as ch: ...` auto-disconnects."""
        t = self.transport_enum(transport)
        resp = self.stub.ConnectCapability(self.pb.ConnectCapabilityRequest(
            consumer_id=consumer_id,
            capability_id=capability_id,
            contract_id=contract_id,
            transport=t,
        ))
        # Translate pb params → dataclass params (re-use the iface helper).
        from .atlas_types import from_pb_params
        params = from_pb_params(_resolve_transport(transport), resp.params) if resp.HasField("params") else None
        return Channel(
            cap_id=capability_id,
            contract_id=contract_id,
            transport=_resolve_transport(transport),
            endpoint=resp.endpoint,
            channel_id=resp.channel_id,
            params=params,
            _closer=self._disconnect,
        )

    def _disconnect(self, channel_id: str) -> bool:
        try:
            resp = self.stub.DisconnectCapability(
                self.pb.DisconnectCapabilityRequest(channel_id=channel_id)
            )
            return bool(resp.was_open)
        except Exception as e:  # noqa: BLE001
            log.debug("DisconnectCapability(%s): %s", channel_id, e)
            return False

    # ── lifecycle / heartbeat ────────────────────────────────────────────
    def heartbeat(self, capability_id: str) -> None:
        try:
            self.stub.Heartbeat(self.pb.HeartbeatRequest(capability_id=capability_id))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat: %s", e)

    def set_capability_state(
        self,
        capability_id: str,
        state: CapabilityState | str | int,
        detail: str = "",
    ) -> None:
        """Push a lifecycle state transition. `state` accepts the new
        CapabilityState enum, an int, or a lower-case string
        (registered/initialized/running/error/terminated).
        Atlas-side validation is soft in v0.1 — illegal transitions log a
        warn but are still accepted, so this call won't raise."""
        self._ensure_stub()
        cs = _resolve_state(state)
        if cs == CapabilityState.UNSPECIFIED:
            log.warning("set_capability_state: unknown state %r", state)
            return
        try:
            self.stub.SetCapabilityState(self.pb.SetCapabilityStateRequest(
                capability_id=capability_id,
                state=int(cs),
                detail=detail,
            ))
        except Exception as e:  # noqa: BLE001
            log.debug("SetCapabilityState(%s, %s): %s", capability_id, cs.name, e)

    def start_heartbeat(
        self, capability_id: str, period_s: float = 10.0,
    ) -> threading.Thread:
        def _loop():
            while True:
                time.sleep(period_s)
                self.heartbeat(capability_id)
        t = threading.Thread(target=_loop, name=f"robonix-hb-{capability_id}", daemon=True)
        t.start()
        return t


# ── module-level singleton facade ────────────────────────────────────────
# `from robonix_api import atlas` → public discovery API only (`get` / `find`).
# Internal RPCs (declares, heartbeat, set_state, connect) stay on AtlasClient
# instances owned by Capability. The singleton resolves its endpoint lazily
# from $ROBONIX_ATLAS (default 127.0.0.1:50051) on first use.

class _AtlasFacade:
    """Public v1 atlas API. Shared lazy singleton — `atlas.get(...)` and
    `atlas.find(...)`. Capability internals reach into AtlasClient directly,
    so this stays minimal."""
    _client: AtlasClient | None = None

    def _resolve(self) -> AtlasClient:
        if self._client is None:
            import os
            ep = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
            self._client = AtlasClient(ep)
        return self._client

    def get(self, capability_id: str) -> CapabilityRecord | None:
        return self._resolve().get(capability_id)

    def find(
        self,
        contract_id: str,
        *,
        transport: Transport | str | int = Transport.UNSPECIFIED,
    ) -> list[CapabilityRecord]:
        return self._resolve().find(
            contract_id,
            transport=transport,
        )


atlas = _AtlasFacade()
