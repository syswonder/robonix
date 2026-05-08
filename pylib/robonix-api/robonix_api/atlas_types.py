# SPDX-License-Identifier: MulanPSL-2.0
"""Python dataclass mirrors of atlas.proto types.

`AtlasClient` returns these instead of raw generated protobuf messages so
the rest of robonix_api never has to think about pb<->Python conversion.
Frozen + slotted to keep the runtime cost negligible.

Conversion helpers (`from_pb_*`) live alongside their target types so adding
a new proto field is a one-line edit.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any


class Transport(IntEnum):
    UNSPECIFIED = 0
    GRPC        = 1
    ROS2        = 2
    MCP         = 3


class CapabilityState(IntEnum):
    UNSPECIFIED = 0
    REGISTERED  = 1
    INITIALIZED = 2
    RUNNING     = 3
    ERROR       = 4
    TERMINATED  = 5


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
    description: str = ""
    input_schema_json: str = "{}"


@dataclass(frozen=True, slots=True)
class FieldSpec:
    name: str
    type_name: str
    is_primitive: bool = False
    is_array: bool = False
    array_size: int = 0  # 0 == unbounded


@dataclass(frozen=True, slots=True)
class InterfaceMetadata:
    contract_id: str
    transport: Transport
    params: GrpcParams | Ros2Params | McpParams | None = None


@dataclass(frozen=True, slots=True)
class CapabilityRecord:
    capability_id: str
    namespace: str
    capability_md_path: str = ""
    last_heartbeat_ms: int = 0
    state: CapabilityState = CapabilityState.UNSPECIFIED
    state_detail: str = ""
    interfaces: tuple[InterfaceMetadata, ...] = ()


@dataclass(frozen=True, slots=True)
class ContractDescriptor:
    id: str
    version: str = ""
    kind: str = ""
    mode: str = ""
    io_msg_type: str = ""
    io_srv_type: str = ""
    source_toml_path: str = ""
    msg_fields: tuple[FieldSpec, ...] = ()
    srv_request_fields: tuple[FieldSpec, ...] = ()
    srv_response_fields: tuple[FieldSpec, ...] = ()


@dataclass
class Channel:
    """Open consumer→provider edge. Returned by `AtlasClient.connect()`.
    Use as a context manager — `__exit__` calls `close()` which fires
    `DisconnectCapability` on atlas. Idempotent."""
    cap_id: str
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
            except Exception:  # noqa: BLE001 — closing is best-effort
                pass


# ── pb -> dataclass converters ─────────────────────────────────────────────
# Keep these out of the dataclass bodies so atlas_types.py has no proto
# dependency at import time. Callers in atlas.py wire them up.

def from_pb_field_spec(pb_f) -> FieldSpec:
    return FieldSpec(
        name=pb_f.name,
        type_name=pb_f.type_name,
        is_primitive=pb_f.is_primitive,
        is_array=pb_f.is_array,
        array_size=int(pb_f.array_size),
    )


def from_pb_params(transport: Transport, pb_params) -> GrpcParams | Ros2Params | McpParams | None:
    if pb_params is None:
        return None
    # Each TransportParams oneof variant matches its transport.
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
        m = pb_params.mcp
        return McpParams(
            description=m.description,
            input_schema_json=m.input_schema_json or "{}",
        )
    return None


def from_pb_iface(pb_iface) -> InterfaceMetadata:
    transport = Transport(pb_iface.transport)
    return InterfaceMetadata(
        contract_id=pb_iface.contract_id,
        transport=transport,
        params=from_pb_params(transport, pb_iface.params),
    )


def from_pb_record(pb_rec) -> CapabilityRecord:
    return CapabilityRecord(
        capability_id=pb_rec.capability_id,
        namespace=pb_rec.namespace,
        capability_md_path=pb_rec.capability_md_path,
        last_heartbeat_ms=int(pb_rec.last_heartbeat_ms),
        state=CapabilityState(pb_rec.state),
        state_detail=pb_rec.state_detail,
        interfaces=tuple(from_pb_iface(i) for i in pb_rec.interfaces),
    )


def from_pb_contract(pb_c) -> ContractDescriptor:
    return ContractDescriptor(
        id=pb_c.id,
        version=pb_c.version,
        kind=pb_c.kind,
        mode=pb_c.mode,
        io_msg_type=pb_c.io_msg_type,
        io_srv_type=pb_c.io_srv_type,
        source_toml_path=pb_c.source_toml_path,
        msg_fields=tuple(from_pb_field_spec(f) for f in pb_c.msg_fields),
        srv_request_fields=tuple(from_pb_field_spec(f) for f in pb_c.srv_request_fields),
        srv_response_fields=tuple(from_pb_field_spec(f) for f in pb_c.srv_response_fields),
    )
