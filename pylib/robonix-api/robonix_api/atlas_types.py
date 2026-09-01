# SPDX-License-Identifier: MulanPSL-2.0
"""Python dataclass mirrors of atlas.proto types.

Keeps the rest of robonix_api free of raw protobuf access. Frozen +
slotted to keep the runtime cost negligible.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any


class Transport(IntEnum):
    UNSPECIFIED = 0
    GRPC = 1
    ROS2 = 2
    MCP = 3


class Kind(IntEnum):
    """Closed set of CapabilityProvider kinds. Internal: developers
    interact with the concrete `Primitive` / `Service` / `Skill`
    classes, not with this enum directly."""

    UNSPECIFIED = 0
    PRIMITIVE = 1
    SERVICE = 2
    SKILL = 3


class LifecycleState(IntEnum):
    UNSPECIFIED = 0
    REGISTERED = 1
    INACTIVE = 2
    ACTIVE = 3
    ERROR = 4
    TERMINATED = 5


@dataclass(frozen=True, slots=True)
class GrpcParams:
    proto_file: str = "robonix_contracts.proto"
    service_name: str = ""
    method: str = ""


@dataclass(frozen=True, slots=True)
class Ros2Params:
    qos_profile: str = ""


@dataclass(frozen=True, slots=True)
class McpParams:
    input_schema_json: str = "{}"


@dataclass(frozen=True, slots=True)
class FieldSpec:
    name: str
    type_name: str
    is_primitive: bool = False
    is_array: bool = False
    array_size: int = 0  # 0 == unbounded


@dataclass(frozen=True, slots=True)
class Capability:
    """One declared Capability on a CapabilityProvider, mirrored from
    `pb::Capability`. Carries provider_id / provider_kind so consumers can
    flatten without rebuilding the relationship from outer providers.
    `endpoint` is omitted on purpose (see ConnectCapability)."""

    provider_id: str
    provider_kind: Kind
    contract_id: str
    transport: Transport
    description: str = ""
    namespace_mismatch: bool = False
    params: GrpcParams | Ros2Params | McpParams | None = None


@dataclass(frozen=True, slots=True)
class CapabilityProvider:
    """One registered Primitive / Service / Skill and the Capabilities
    it currently offers. Mirrored from `pb::CapabilityProvider`."""

    id: str
    kind: Kind
    namespace: str
    capability_md_path: str = ""
    # CAPABILITY.md content as registered (portable; the path is debug
    # provenance only). Empty when the provider shipped no markdown.
    capability_md: str = ""
    last_heartbeat_ms: int = 0
    state: LifecycleState = LifecycleState.UNSPECIFIED
    state_detail: str = ""
    # Opaque nonce replaced on each successful Atlas Register generation.
    registration_id: str = ""
    capabilities: tuple[Capability, ...] = ()


@dataclass(frozen=True, slots=True)
class ContractDescriptor:
    id: str
    version: str = ""
    kind: Kind = Kind.UNSPECIFIED
    mode: str = ""
    io_msg_type: str = ""
    io_srv_type: str = ""
    source_toml_path: str = ""
    description: str = ""
    cross_namespace: bool = False
    msg_fields: tuple[FieldSpec, ...] = ()
    srv_request_fields: tuple[FieldSpec, ...] = ()
    srv_response_fields: tuple[FieldSpec, ...] = ()


@dataclass
class Channel:
    """Open consumer->provider edge returned by `ATLAS.connect_capability`.
    Context-manager — `__exit__` calls `close()` (which fires
    `DisconnectCapability` on atlas, idempotent)."""

    provider_id: str
    contract_id: str
    transport: Transport
    endpoint: str
    channel_id: str
    params: GrpcParams | Ros2Params | McpParams | None = None
    _closer: Any = field(default=None, repr=False)
    _closed: bool = field(default=False, repr=False)

    def __enter__(self) -> "Channel":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        if self._closed:
            return
        object.__setattr__(self, "_closed", True)
        if self._closer is not None:
            try:
                self._closer(self.channel_id)
            except Exception:  # noqa: BLE001 -- closing is best-effort
                pass


# -- pb -> dataclass converters --------------------------------------------
# Kept out of the dataclass bodies so atlas_types.py has no proto import
# dependency at module-load time.


def from_pb_field_spec(pb_f) -> FieldSpec:
    return FieldSpec(
        name=pb_f.name,
        type_name=pb_f.type_name,
        is_primitive=pb_f.is_primitive,
        is_array=pb_f.is_array,
        array_size=int(pb_f.array_size),
    )


def from_pb_params(
    transport: Transport, pb_params
) -> GrpcParams | Ros2Params | McpParams | None:
    if pb_params is None:
        return None
    if transport == Transport.GRPC and pb_params.HasField("grpc"):
        g = pb_params.grpc
        return GrpcParams(
            proto_file=g.proto_file or "robonix_contracts.proto",
            service_name=g.service_name,
            method=g.method,
        )
    if transport == Transport.ROS2 and pb_params.HasField("ros2"):
        return Ros2Params(qos_profile=pb_params.ros2.qos_profile)
    if transport == Transport.MCP and pb_params.HasField("mcp"):
        return McpParams(input_schema_json=pb_params.mcp.input_schema_json or "{}")
    return None


def from_pb_capability(pb_cap) -> Capability:
    transport = Transport(pb_cap.transport)
    return Capability(
        provider_id=pb_cap.provider_id,
        provider_kind=Kind(pb_cap.provider_kind),
        contract_id=pb_cap.contract_id,
        transport=transport,
        description=pb_cap.description,
        namespace_mismatch=getattr(pb_cap, "namespace_mismatch", False),
        params=from_pb_params(transport, pb_cap.params),
    )


def from_pb_provider(pb_rec) -> CapabilityProvider:
    return CapabilityProvider(
        id=pb_rec.id,
        kind=Kind(pb_rec.kind),
        namespace=pb_rec.namespace,
        capability_md_path=pb_rec.capability_md_path,
        # `getattr` guard: a package built against an older atlas.proto ships
        # a stale atlas_pb2 whose CapabilityProvider has no `capability_md`
        # field. Atlas still sends it on the wire (forward-compatible), but
        # accessing the attribute on the stale class raises AttributeError.
        # Degrade to "" so mixed-version packages can still read atlas.
        capability_md=getattr(pb_rec, "capability_md", ""),
        last_heartbeat_ms=int(pb_rec.last_heartbeat_ms),
        state=LifecycleState(pb_rec.state),
        state_detail=pb_rec.state_detail,
        registration_id=getattr(pb_rec, "registration_id", ""),
        capabilities=tuple(from_pb_capability(c) for c in pb_rec.capabilities),
    )


def from_pb_contract(pb_c) -> ContractDescriptor:
    return ContractDescriptor(
        id=pb_c.id,
        version=pb_c.version,
        kind=Kind(pb_c.kind),
        mode=pb_c.mode,
        io_msg_type=pb_c.io_msg_type,
        io_srv_type=pb_c.io_srv_type,
        source_toml_path=pb_c.source_toml_path,
        description=pb_c.description,
        cross_namespace=getattr(pb_c, "cross_namespace", False),
        msg_fields=tuple(from_pb_field_spec(f) for f in pb_c.msg_fields),
        srv_request_fields=tuple(
            from_pb_field_spec(f) for f in pb_c.srv_request_fields
        ),
        srv_response_fields=tuple(
            from_pb_field_spec(f) for f in pb_c.srv_response_fields
        ),
    )
