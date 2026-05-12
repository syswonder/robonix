# SPDX-License-Identifier: MulanPSL-2.0
"""Thin Python wrapper over the generated atlas_pb2 stubs.

Public surface — what `from robonix_api import ATLAS` exposes:
    Registration         register_primitive / register_service / register_skill
                         unregister  /  heartbeat
    Capability binding   declare_capability
    Discovery            query / query_primitives / query_services / query_skills
                         find_capability / find_unique_capability
    Channels             connect_capability / disconnect_capability
    Contracts            query_contract / list_contracts
    Debug                inspect

Privileged operations (`SetLifecycleState`, in particular) are NOT exposed
here — they're framework-internal and live in `_lifecycle_internal.py`.
A regular `from robonix_api import ATLAS` cannot reach them.

Returns dataclasses from `robonix_api.atlas_types`; raw protobuf never
leaves this module.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any

from .atlas_types import (
    Capability,
    CapabilityProvider,
    Channel,
    ContractDescriptor,
    GrpcParams,
    Kind,
    LifecycleState,
    McpParams,
    Ros2Params,
    Transport,
    from_pb_capability,
    from_pb_contract,
    from_pb_params,
    from_pb_provider,
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


def _resolve_kind(k: Kind | str | int) -> Kind:
    if isinstance(k, Kind):
        return k
    if isinstance(k, int):
        return Kind(k)
    name = str(k).strip().lower()
    return {
        "primitive": Kind.PRIMITIVE,
        "service":   Kind.SERVICE,
        "skill":     Kind.SKILL,
        "":          Kind.UNSPECIFIED,
        "unspecified": Kind.UNSPECIFIED,
    }.get(name, Kind.UNSPECIFIED)


class _Atlas:
    """Singleton facade for the atlas gRPC client.

    Exposed as the module-level `ATLAS` constant. Lazy-connects on first
    use, reads `$ROBONIX_ATLAS` (default `127.0.0.1:50051`).
    """

    def __init__(self, endpoint: str | None = None) -> None:
        self._endpoint = endpoint
        self._channel: Any = None
        self._stub: Any = None
        self._pb: Any = None

    # -- lazy stub bootstrap -------------------------------------------------

    def _ensure_stub(self) -> None:
        if self._stub is not None:
            return
        import grpc
        import atlas_pb2          # type: ignore
        import atlas_pb2_grpc     # type: ignore
        ep = self._endpoint or os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
        self._endpoint = ep
        self._pb = atlas_pb2
        self._channel = grpc.insecure_channel(ep)
        self._stub = atlas_pb2_grpc.AtlasStub(self._channel)

    @property
    def _wire_pb(self):
        self._ensure_stub()
        return self._pb

    @property
    def _wire_stub(self):
        self._ensure_stub()
        return self._stub

    def _transport_enum(self, t: Transport | str | int):
        pb = self._wire_pb
        return {
            Transport.GRPC:        pb.TRANSPORT_GRPC,
            Transport.ROS2:        pb.TRANSPORT_ROS2,
            Transport.MCP:         pb.TRANSPORT_MCP,
            Transport.UNSPECIFIED: pb.TRANSPORT_UNSPECIFIED,
        }[_resolve_transport(t)]

    def _kind_enum(self, k: Kind | str | int):
        pb = self._wire_pb
        return {
            Kind.PRIMITIVE:   pb.KIND_PRIMITIVE,
            Kind.SERVICE:     pb.KIND_SERVICE,
            Kind.SKILL:       pb.KIND_SKILL,
            Kind.UNSPECIFIED: pb.KIND_UNSPECIFIED,
        }[_resolve_kind(k)]

    # -- registration -------------------------------------------------------

    def register_primitive(
        self, id: str, namespace: str, capability_md_path: str = ""
    ) -> str:
        return self._register(self._wire_stub.RegisterPrimitive, id, namespace, capability_md_path)

    def register_service(
        self, id: str, namespace: str, capability_md_path: str = ""
    ) -> str:
        return self._register(self._wire_stub.RegisterService, id, namespace, capability_md_path)

    def register_skill(
        self, id: str, namespace: str, capability_md_path: str = ""
    ) -> str:
        return self._register(self._wire_stub.RegisterSkill, id, namespace, capability_md_path)

    def _register(self, rpc, id: str, namespace: str, capability_md_path: str) -> str:
        resp = rpc(self._wire_pb.RegisterRequest(
            id=id,
            namespace=namespace,
            capability_md_path=capability_md_path,
        ))
        return resp.id

    def unregister(self, id: str) -> bool:
        try:
            resp = self._wire_stub.Unregister(self._wire_pb.UnregisterRequest(id=id))
            return bool(resp.was_present)
        except Exception as e:  # noqa: BLE001
            log.debug("Unregister(%s): %s", id, e)
            return False

    def heartbeat(self, id: str) -> None:
        try:
            self._wire_stub.Heartbeat(self._wire_pb.HeartbeatRequest(id=id))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat(%s): %s", id, e)

    def start_heartbeat(self, id: str, period_s: float = 30.0) -> threading.Thread:
        """Background daemon thread that pings Heartbeat every `period_s`
        seconds. Returns the thread for caller bookkeeping (or to ignore)."""
        def _loop():
            while True:
                time.sleep(period_s)
                self.heartbeat(id)
        t = threading.Thread(target=_loop, name=f"robonix-hb-{id}", daemon=True)
        t.start()
        return t

    # -- capability binding -------------------------------------------------

    def declare_capability(
        self,
        owner_id: str,
        contract_id: str,
        transport: Transport | str | int,
        endpoint: str,
        params: GrpcParams | Ros2Params | McpParams | None = None,
        description: str = "",
    ) -> str:
        """Declare one Capability on a registered CapabilityProvider. Returns
        the authoritative endpoint Atlas stored (may differ from `endpoint`
        when Atlas rewrote on collision)."""
        import grpc
        pb_params = self._params_to_pb(transport, params)
        try:
            resp = self._wire_stub.DeclareCapability(self._wire_pb.DeclareCapabilityRequest(
                owner_id=owner_id,
                contract_id=contract_id,
                transport=self._transport_enum(transport),
                endpoint=endpoint,
                params=pb_params,
                description=description,
            ))
            return resp.endpoint or endpoint
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                log.debug("declare %s/%s/%s already exists; ok",
                          owner_id, contract_id, _resolve_transport(transport).name)
                return endpoint
            raise

    def _params_to_pb(
        self,
        transport: Transport | str | int,
        params: GrpcParams | Ros2Params | McpParams | None,
    ):
        pb = self._wire_pb
        t = _resolve_transport(transport)
        if params is None:
            if t == Transport.ROS2:
                return pb.TransportParams(ros2=pb.Ros2Params())
            if t == Transport.GRPC:
                return pb.TransportParams(grpc=pb.GrpcParams(
                    proto_file="robonix_contracts.proto"))
            if t == Transport.MCP:
                return pb.TransportParams(mcp=pb.McpParams(input_schema_json="{}"))
            return pb.TransportParams()
        if isinstance(params, GrpcParams):
            return pb.TransportParams(grpc=pb.GrpcParams(
                proto_file=params.proto_file or "robonix_contracts.proto",
                service_name=params.service_name,
                method=params.method,
            ))
        if isinstance(params, Ros2Params):
            return pb.TransportParams(ros2=pb.Ros2Params(qos_profile=params.qos_profile))
        if isinstance(params, McpParams):
            return pb.TransportParams(mcp=pb.McpParams(
                input_schema_json=params.input_schema_json or "{}"))
        raise TypeError(f"unknown params type: {type(params).__name__}")

    # -- discovery (CapabilityProvider-shaped) -----------------------------

    def query(
        self,
        *,
        kind: Kind | str | int = Kind.UNSPECIFIED,
        id: str = "",
        contract_id: str = "",
        namespace_prefix: str = "",
        transport: Transport | str | int = Transport.UNSPECIFIED,
    ) -> list[CapabilityProvider]:
        """Generic Query. Kind=UNSPECIFIED returns all kinds; each Record's
        `kind` field carries the actual kind so callers can demultiplex."""
        import grpc
        try:
            resp = self._wire_stub.Query(self._wire_pb.QueryRequest(
                kind=self._kind_enum(kind),
                id=id,
                contract_id=contract_id,
                transport=self._transport_enum(transport),
                namespace_prefix=namespace_prefix,
            ))
        except grpc.RpcError as e:
            log.warning("Query(kind=%r, id=%r, contract=%r): %s",
                        kind, id, contract_id, e)
            return []
        return [from_pb_provider(p) for p in resp.providers]

    def query_primitives(
        self,
        *,
        id: str = "",
        contract_id: str = "",
        namespace_prefix: str = "",
        transport: Transport | str | int = Transport.UNSPECIFIED,
    ) -> list[CapabilityProvider]:
        return self.query(kind=Kind.PRIMITIVE, id=id, contract_id=contract_id,
                          namespace_prefix=namespace_prefix, transport=transport)

    def query_services(
        self,
        *,
        id: str = "",
        contract_id: str = "",
        namespace_prefix: str = "",
        transport: Transport | str | int = Transport.UNSPECIFIED,
    ) -> list[CapabilityProvider]:
        return self.query(kind=Kind.SERVICE, id=id, contract_id=contract_id,
                          namespace_prefix=namespace_prefix, transport=transport)

    def query_skills(
        self,
        *,
        id: str = "",
        contract_id: str = "",
        namespace_prefix: str = "",
        transport: Transport | str | int = Transport.UNSPECIFIED,
    ) -> list[CapabilityProvider]:
        return self.query(kind=Kind.SKILL, id=id, contract_id=contract_id,
                          namespace_prefix=namespace_prefix, transport=transport)

    # -- discovery (flat Capability-shaped) --------------------------------

    def find_capability(
        self,
        *,
        contract_id: str = "",
        transport: Transport | str | int = Transport.UNSPECIFIED,
        owner_kind: Kind | str | int = Kind.UNSPECIFIED,
        owner_id: str = "",
        namespace_prefix: str = "",
    ) -> list[Capability]:
        """Flat consumer-facing list of Capabilities matching the filters.
        Walks Query() and flattens each provider's `capabilities[]`. Each
        returned `Capability` already carries owner_id / owner_kind."""
        providers = self.query(
            kind=owner_kind,
            id=owner_id,
            contract_id=contract_id,
            namespace_prefix=namespace_prefix,
            transport=transport,
        )
        out: list[Capability] = []
        for p in providers:
            for c in p.capabilities:
                out.append(c)
        return out

    def find_unique_capability(
        self,
        *,
        contract_id: str,
        transport: Transport | str | int = Transport.UNSPECIFIED,
        owner_kind: Kind | str | int = Kind.UNSPECIFIED,
        owner_id: str = "",
        namespace_prefix: str = "",
    ) -> Capability:
        """Like find_capability but expects exactly one match. Raises
        ValueError on 0 or >1 matches — for "I depend on THE camera/depth
        capability" wiring where ambiguity is a config bug."""
        caps = self.find_capability(
            contract_id=contract_id, transport=transport,
            owner_kind=owner_kind, owner_id=owner_id,
            namespace_prefix=namespace_prefix,
        )
        if not caps:
            raise ValueError(
                f"find_unique_capability(contract_id={contract_id!r}): no matches"
            )
        if len(caps) > 1:
            owners = ", ".join(c.owner_id for c in caps)
            raise ValueError(
                f"find_unique_capability(contract_id={contract_id!r}): "
                f"{len(caps)} matches (owners: {owners}) — pass owner_id to disambiguate"
            )
        return caps[0]

    # -- contracts ----------------------------------------------------------

    def query_contract(self, contract_id: str) -> ContractDescriptor | None:
        import grpc
        try:
            resp = self._wire_stub.QueryContract(
                self._wire_pb.QueryContractRequest(contract_id=contract_id)
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
            resp = self._wire_stub.ListContracts(
                self._wire_pb.ListContractsRequest(namespace_prefix=namespace_prefix)
            )
        except grpc.RpcError as e:
            log.warning("ListContracts(prefix=%r): %s", namespace_prefix, e)
            return []
        return [from_pb_contract(c) for c in resp.contracts]

    # -- channels -----------------------------------------------------------

    def connect_capability(
        self,
        *,
        consumer_id: str,
        owner_id: str,
        contract_id: str,
        transport: Transport | str | int,
    ) -> Channel:
        """Open a consumer->owner edge. Returns a `Channel` context manager
        — `with ATLAS.connect_capability(...) as ch: ...` auto-disconnects."""
        resp = self._wire_stub.ConnectCapability(self._wire_pb.ConnectCapabilityRequest(
            consumer_id=consumer_id,
            owner_id=owner_id,
            contract_id=contract_id,
            transport=self._transport_enum(transport),
        ))
        t = _resolve_transport(transport)
        params = (
            from_pb_params(t, resp.params)
            if resp.HasField("params") else None
        )
        return Channel(
            owner_id=owner_id,
            contract_id=contract_id,
            transport=t,
            endpoint=resp.endpoint,
            channel_id=resp.channel_id,
            params=params,
            _closer=self.disconnect_capability,
        )

    def disconnect_capability(self, channel_id: str) -> bool:
        try:
            resp = self._wire_stub.DisconnectCapability(
                self._wire_pb.DisconnectCapabilityRequest(channel_id=channel_id)
            )
            return bool(resp.was_open)
        except Exception as e:  # noqa: BLE001
            log.debug("DisconnectCapability(%s): %s", channel_id, e)
            return False

    # -- debug --------------------------------------------------------------

    def inspect(self) -> dict:
        try:
            resp = self._wire_stub.InspectAtlas(self._wire_pb.InspectAtlasRequest())
            return json.loads(resp.json) if resp.json else {}
        except Exception as e:  # noqa: BLE001
            log.debug("InspectAtlas: %s", e)
            return {}


# Public singleton. The uppercase name marks it as a globally-shared
# connection (cf. `prometheus_client.REGISTRY`, `os.environ`); it is
# fine to import everywhere — connection is lazy and per-process shared.
ATLAS = _Atlas()


__all__ = [
    "ATLAS",
]
