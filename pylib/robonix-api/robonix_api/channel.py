# SPDX-License-Identifier: MulanPSL-2.0
"""Client-side communication and the lifetime of one capability execution."""
from __future__ import annotations

import logging
import os
import threading
import time

from .atlas_types import Channel, Transport, from_pb_contract

log = logging.getLogger("robonix_api.channel")

_TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED", "CANCELLED", "TIMEOUT"}
_STATES = _TERMINAL | {"PENDING", "RUNNING", "PAUSED"}
_DEFAULT_CHANNEL_LIMIT = 1024
_COMPANION_SLOTS = 2


def _channel_limit():
    """Allocations per Atlas client; ROBONIX_ATLAS_CHANNEL_LIMIT overrides the default."""
    raw = os.environ.get("ROBONIX_ATLAS_CHANNEL_LIMIT")
    if raw is None:
        return _DEFAULT_CHANNEL_LIMIT
    try:
        limit = int(raw.strip())
    except ValueError:
        limit = 0
    if limit > _COMPANION_SLOTS:
        return limit
    log.warning("ignoring ROBONIX_ATLAS_CHANNEL_LIMIT=%r; expected an integer above %d, using %d",
                raw, _COMPANION_SLOTS, _DEFAULT_CHANNEL_LIMIT)
    return _DEFAULT_CHANNEL_LIMIT


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("Channel operation timed out")
    return remaining


class _ChannelClient:
    """Retain one Atlas client's executions and shared communication resources."""

    def __init__(self, atlas):
        self.atlas = atlas
        self.channels: dict[str, Channel] = {}
        self.lock = threading.RLock()
        self.connecting = 0
        self.limit = _channel_limit()
        self.portal = None
        self.portal_cm = None

    def get(self, channel_id):
        with self.lock:
            return self.channels.get(channel_id)

    def stop_portal(self):
        # Interpreter shutdown freezes the daemon loop thread, so an unstopped portal
        # leaves the exiting main thread waiting on it forever.
        with self.lock:
            portal, portal_cm = self.portal, self.portal_cm
            self.portal = self.portal_cm = None
        if portal is not None:
            try:
                portal.call(portal.stop, True)
            finally:
                portal_cm.__exit__(None, None, None)

    def _descriptor(self, contract_id, transport, deadline):
        atlas = self.atlas
        # atlas.py defines _wire_stub and _wire_pb to use QueryContract defined in atlas.proto
        resp = atlas._wire_stub.QueryContract(
            atlas._wire_pb.QueryContractRequest(contract_id=contract_id), timeout=_remaining(deadline),
        )
        if not resp.found:
            raise ValueError(f"Unknown contract: {contract_id}")
        desc = from_pb_contract(resp.contract)
        modes = {
            Transport.MCP: {"rpc"},
            Transport.GRPC: {"rpc", "rpc_server_stream", "rpc_client_stream",
                             "rpc_bidirectional_stream", "topic_in", "topic_out"},
            Transport.ROS2: {"rpc", "topic_in", "topic_out"},
        }
        if desc.mode not in modes.get(transport, set()):
            raise ValueError(f"{transport.name} channel does not support mode {desc.mode}: {contract_id}")
        return desc

    def connect(self, *, consumer_id, provider_id, contract_id, transport, _deadline=None):
        from .atlas import _resolve_transport
        transport = _resolve_transport(transport)
        if transport not in (Transport.MCP, Transport.GRPC, Transport.ROS2):
            raise ValueError("Specify MCP, GRPC, or ROS2 for connect_channel")
        business = contract_id.rsplit("/", 1)[-1] not in {"status", "cancel"}
        with self.lock:
            limit = self.limit - _COMPANION_SLOTS if business else self.limit
            if len(self.channels) + self.connecting >= limit:
                raise RuntimeError("Atlas client channel capacity exhausted; two slots are reserved for companions; "
                                   "close existing channels or raise ROBONIX_ATLAS_CHANNEL_LIMIT")
            self.connecting += 1
        try:
            return self._connect(consumer_id, provider_id, contract_id, transport,
                                 _deadline if _deadline is not None else time.monotonic() + 30)
        finally:
            with self.lock:
                self.connecting -= 1

    def _connect(self, consumer_id, provider_id, contract_id, transport, deadline):
        atlas = self.atlas
        descriptor = self._descriptor(contract_id, transport, deadline)
        binding = None
        if transport == Transport.MCP:
            from .channel_mcp import _Mcp, binding as mcp_binding
            protocol_type = _Mcp
            binding = mcp_binding(descriptor)
        elif transport == Transport.GRPC:
            from .channel_grpc import _Grpc
            protocol_type = _Grpc
        else:
            from .channel_ros2 import _Ros2
            protocol_type = _Ros2
        companions = {}
        if descriptor.mode == "rpc" and contract_id.rsplit("/", 1)[-1] not in {"status", "cancel"}:
            resp = atlas._wire_stub.Query(atlas._wire_pb.QueryRequest(
                id=provider_id, transport=atlas._transport_enum(Transport.UNSPECIFIED),
            ), timeout=_remaining(deadline))
            capabilities = [c for p in resp.providers for c in p.capabilities]
            available = {c.contract_id for c in capabilities}
            companions = {name: f"{contract_id}/{name}" for name in ("status", "cancel")
                          if f"{contract_id}/{name}" in available}
            if len(companions) == 1:
                raise ValueError(f"Async capability {contract_id} must declare both /status and /cancel")
            # Bind the status and cancel capabilities.
            for cid in companions.values():
                if not any(c.contract_id == cid and c.transport == int(transport) for c in capabilities):
                    raise ValueError(f"Async companion {cid} must support {transport.name}")
                companion = self._descriptor(cid, transport, deadline)
                if companion.mode != "rpc":
                    raise ValueError(f"Async companion must use rpc mode: {cid}")
                if transport == Transport.MCP:
                    mcp_binding(companion)
        ch = atlas._allocate_channel(consumer_id, provider_id, contract_id, transport,
                                     timeout=_remaining(deadline))
        execution = _Execution(self, ch, consumer_id, binding, companions, descriptor)
        ch._execution = execution
        try:
            execution.protocol = protocol_type(execution)
            execution.protocol.prepare(deadline)
        except Exception as exc:
            try:
                execution.close()
            except Exception as cleanup:
                with self.lock:
                    self.channels[ch.channel_id] = ch
                raise RuntimeError(f"{transport.name} setup failed: {exc}; cleanup failed: {cleanup}") from exc
            raise
        with self.lock:
            self.channels[ch.channel_id] = ch
        return ch

    def disconnect(self, ch):
        self.atlas._wire_stub.DisconnectCapability(
            self.atlas._wire_pb.DisconnectCapabilityRequest(channel_id=ch.channel_id), timeout=5,
        )
        with self.lock:
            self.channels.pop(ch.channel_id, None)


class _Execution:
    """Keep a channel allocated until its business operation has certainly stopped."""

    def __init__(self, client, channel, consumer_id, binding, companions, descriptor):
        self.client = client
        self.channel = channel
        self.consumer_id = consumer_id
        self.binding = binding
        self.descriptor = descriptor
        self.protocol = None
        self.companions = companions
        self.run_id = None
        self.raw_response = None
        self.started = False
        self.accepted = False
        self.terminal = False
        self.closing = False
        self.lock = threading.RLock()
        self.close_lock = threading.Lock()

    def _error(self, message):
        return RuntimeError(f"{message}; channel {self.channel.channel_id} retained; use get_channel() to recover")

    def start(self):
        with self.lock:
            if self.started or self.closing or self.channel._closed:
                raise RuntimeError("A channel can execute only one capability call")
            self.started = True

    def publish(self, message):
        if self.channel.transport != Transport.ROS2:
            raise ValueError("publish() requires a ROS 2 channel")
        self.protocol.publish(message)

    def subscribe(self, callback):
        if self.channel.transport != Transport.ROS2:
            raise ValueError("subscribe() requires a ROS 2 channel")
        self.protocol.subscribe(callback)

    def call(self, request, *, timeout=None):
        return self.protocol.call(request, timeout)

    def complete(self, result, raw):
        """Update execution state from a call response and return `result` unchanged.

        `raw` is the response dict used to read `accepted` and `run_id`.
        An accepted async call remains active until its termination is confirmed.
        """
        with self.lock:
            self.raw_response = raw
            if self.companions and raw.get("accepted") is False:
                self.terminal = True
                return result
            run_id = raw.get("run_id")
            if run_id is None:
                run_id = ""
            if not isinstance(run_id, str):
                raise self._error("run_id must be a string")
            self.run_id = run_id
            self.accepted = True
            self.terminal = not self.companions
        return result

    def companion(self, name, *, timeout=30):
        with self.lock:
            if self.channel._closed or not self.accepted or name not in self.companions:
                raise self._error(f"Cannot {name}: no confirmed async execution")
            args = {"run_id": self.run_id} if self.run_id else {}
        # Each companion owns an independent Atlas channel and communication resources.
        deadline = time.monotonic() + timeout
        ch = self.client.connect(consumer_id=self.consumer_id, provider_id=self.channel.provider_id,
                                 contract_id=self.companions[name], transport=self.channel.transport, _deadline=deadline)
        try:
            result = ch.call(args, timeout=_remaining(deadline))
        except Exception as error:
            try:
                ch.close(timeout=max(0, deadline - time.monotonic()))
            except Exception as cleanup:
                raise self._error(f"Companion {name} failed: {error}; cleanup failed: {cleanup}") from error
            raise
        ch.close(timeout=max(0, deadline - time.monotonic()))
        if name == "status":
            state = getattr(result, "state", None)
            if not isinstance(state, str) or state.upper() not in _STATES:
                raise self._error(f"Invalid capability state: {state!r}")
            with self.lock:
                self.terminal = self.terminal or state.upper() in _TERMINAL
        return result

    def close(self, *, timeout=5):
        with self.close_lock:
            if self.channel._closed:
                return
            with self.lock:
                self.closing = True
            try:
                self._close(timeout)
                self.channel._closed = True
            finally:
                with self.lock:
                    self.closing = False

    def _close(self, timeout):
        deadline = time.monotonic() + timeout
        if self.started and not self.terminal and self.protocol is not None and not self.companions:
            self.protocol.finish(max(0, deadline - time.monotonic()))
        if self.started and not self.terminal:
            if not self.accepted or not self.companions:
                raise self._error("Execution termination is unconfirmed")
            self.companion("cancel", timeout=max(0, deadline - time.monotonic()))
            while not self.terminal:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise self._error("Timed out waiting for execution termination")
                self.companion("status", timeout=remaining)
                if not self.terminal:
                    time.sleep(min(2, max(0, deadline - time.monotonic())))
        if self.protocol is not None:
            self.protocol.close()
        try:
            self.client.disconnect(self.channel)
        except Exception as exc:
            raise self._error(f"Atlas disconnect failed: {exc}") from exc
