# SPDX-License-Identifier: MulanPSL-2.0
"""Thin wrapper over the generated atlas_pb2 stubs. Lazy-import (the stubs
live in each package's rbnx-build/codegen/proto_gen/, only on sys.path after
ensure_proto_gen() runs)."""
from __future__ import annotations

import logging
import threading
import time
from typing import Any

log = logging.getLogger("robonix_api.atlas")


class AtlasClient:
    def __init__(self, endpoint: str = "127.0.0.1:50051") -> None:
        self._endpoint = endpoint
        self._channel: Any = None
        self._stub: Any = None
        self._pb: Any = None  # atlas_pb2 module
        self._pb_grpc: Any = None  # atlas_pb2_grpc

    def _ensure_stub(self) -> None:
        if self._stub is not None:
            return
        import grpc  # noqa: F401
        import atlas_pb2  # type: ignore
        import atlas_pb2_grpc  # type: ignore
        self._pb = atlas_pb2
        self._pb_grpc = atlas_pb2_grpc
        import grpc as _grpc
        self._channel = _grpc.insecure_channel(self._endpoint)
        self._stub = atlas_pb2_grpc.AtlasStub(self._channel)

    @property
    def pb(self):
        self._ensure_stub()
        return self._pb

    @property
    def stub(self):
        self._ensure_stub()
        return self._stub

    # ── transport name ↔ enum ────────────────────────────────────────────
    _TRANSPORT_MAP = {
        "ros2":         "TRANSPORT_ROS2",
        "ros":          "TRANSPORT_ROS2",
        "grpc":         "TRANSPORT_GRPC",
        "mcp":          "TRANSPORT_MCP",
        "unspecified":  "TRANSPORT_UNSPECIFIED",
    }

    def transport_enum(self, name: str):
        key = name.lower()
        attr = self._TRANSPORT_MAP.get(key)
        if attr is None:
            raise ValueError(f"unknown transport {name!r}; expected one of {list(self._TRANSPORT_MAP)}")
        return getattr(self.pb, attr)

    # ── high-level operations ────────────────────────────────────────────
    def register_capability(
        self, capability_id: str, namespace: str, capability_md_path: str = "",
    ) -> bool:
        """True if newly registered, False if already exists (idempotent on re-deploy)."""
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

    def declare_ros2(
        self, capability_id: str, contract_id: str, topic: str,
        qos_profile: str = "best_effort",
    ) -> None:
        self._declare(capability_id, contract_id, "ros2", endpoint=topic,
                      params=self.pb.TransportParams(
                          ros2=self.pb.Ros2Params(qos_profile=qos_profile),
                      ))

    def declare_grpc(
        self, capability_id: str, contract_id: str, endpoint: str,
        service_name: str, method: str, proto_file: str = "robonix_contracts.proto",
    ) -> None:
        self._declare(capability_id, contract_id, "grpc", endpoint=endpoint,
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
    ) -> None:
        self._declare(capability_id, contract_id, "mcp", endpoint=endpoint,
                      params=self.pb.TransportParams(
                          mcp=self.pb.McpParams(
                              description=description,
                              input_schema_json=input_schema_json,
                          ),
                      ))

    def _declare(
        self, capability_id: str, contract_id: str, transport: str,
        endpoint: str, params,
    ) -> None:
        import grpc
        try:
            self.stub.DeclareInterface(self.pb.DeclareInterfaceRequest(
                capability_id=capability_id,
                contract_id=contract_id,
                transport=self.transport_enum(transport),
                endpoint=endpoint,
                params=params,
            ))
        except grpc.RpcError as e:
            if e.code() == grpc.StatusCode.ALREADY_EXISTS:
                log.debug("declare %s/%s/%s already exists; ok",
                          capability_id, contract_id, transport)
                return
            raise

    def query_endpoint(
        self, contract_id: str, transport: str, *, consumer_id: str | None = None,
    ) -> str | None:
        """QueryCapabilities + ConnectCapability for the first matching record.
        Atlas only discloses endpoints after Connect; we send Connect when the
        caller passes consumer_id so the channel is recorded."""
        import grpc
        t = self.transport_enum(transport)
        try:
            resp = self.stub.QueryCapabilities(self.pb.QueryCapabilitiesRequest(
                contract_id=contract_id, transport=t,
            ))
        except grpc.RpcError as e:
            log.warning("QueryCapabilities(%s) failed: %s", contract_id, e)
            return None
        for rec in resp.records:
            for iface in rec.interfaces:
                if iface.contract_id != contract_id or iface.transport != t:
                    continue
                if consumer_id:
                    try:
                        conn = self.stub.ConnectCapability(self.pb.ConnectCapabilityRequest(
                            consumer_id=consumer_id,
                            capability_id=rec.capability_id,
                            contract_id=contract_id,
                            transport=t,
                        ))
                        if conn.endpoint:
                            return conn.endpoint
                    except grpc.RpcError as e:
                        log.warning("ConnectCapability(%s) failed: %s", contract_id, e)
                else:
                    if iface.endpoint:
                        return iface.endpoint
        return None

    def heartbeat(self, capability_id: str) -> None:
        try:
            self.stub.Heartbeat(self.pb.HeartbeatRequest(capability_id=capability_id))
        except Exception as e:  # noqa: BLE001
            log.debug("heartbeat: %s", e)

    def set_capability_state(self, capability_id: str, state: str, detail: str = "") -> None:
        """Push a lifecycle-state transition to atlas. `state` is a string
        from `LIFECYCLE_STATES` (REGISTERED/INITIALIZED/ONLINE/OFFLINE/ERROR).
        Best-effort — atlas downtime should not crash a healthy cap."""
        self._ensure_stub()
        enum_name = "STATE_" + state.upper()
        enum_val = getattr(self.pb.CapabilityState, enum_name, None)
        if enum_val is None:
            log.warning("set_capability_state: unknown state %r", state)
            return
        try:
            self.stub.SetCapabilityState(self.pb.SetCapabilityStateRequest(
                capability_id=capability_id, state=enum_val, detail=detail,
            ))
        except Exception as e:  # noqa: BLE001
            log.debug("SetCapabilityState(%s, %s): %s", capability_id, state, e)

    def start_heartbeat(self, capability_id: str, period_s: float = 15.0) -> threading.Thread:
        def _loop():
            while True:
                time.sleep(period_s)
                self.heartbeat(capability_id)
        t = threading.Thread(target=_loop, name=f"robonix-hb-{capability_id}", daemon=True)
        t.start()
        return t
