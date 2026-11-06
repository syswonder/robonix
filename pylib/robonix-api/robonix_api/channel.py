# SPDX-License-Identifier: MulanPSL-2.0
"""Client-side MCP communication and the lifetime of one capability execution."""
from __future__ import annotations

import importlib
import json
import threading
import time
from contextlib import AsyncExitStack, asynccontextmanager
from functools import partial

from .atlas_types import Channel, Transport, from_pb_contract

_TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED", "CANCELLED", "TIMEOUT"}
_STATES = _TERMINAL | {"PENDING", "RUNNING", "PAUSED"}


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TimeoutError("MCP channel operation timed out")
    return remaining


@asynccontextmanager
async def _session(endpoint, timeout):
    from mcp import ClientSession
    from mcp.client.streamable_http import streamable_http_client

    async with AsyncExitStack() as stack:
        import httpx
        uri = endpoint if endpoint.startswith(("http://", "https://")) else f"http://{endpoint}"
        if "/mcp" not in uri:
            uri = uri.rstrip("/") + "/mcp"
        # Provider traffic must not depend on the consumer's shell proxy settings.
        client = await stack.enter_async_context(httpx.AsyncClient(
            trust_env=False, timeout=httpx.Timeout(5, read=None),
        ))
        transport = streamable_http_client(uri, http_client=client)
        streams = await stack.enter_async_context(transport)
        session = await stack.enter_async_context(ClientSession(streams[0], streams[1]))
        import anyio
        with anyio.fail_after(timeout):
            await session.initialize()
        yield session


async def _invoke(session, tool, args, timeout):
    import anyio
    from mcp import types
    with anyio.fail_after(timeout):
        # Receive completion before SDK schema discovery/validation can obscure it.
        return await session.send_request(
            types.ClientRequest(types.CallToolRequest(
                params=types.CallToolRequestParams(name=tool, arguments=args),
            )),
            types.CallToolResult,
        )


def _json_object(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            raise ValueError(f"Duplicate JSON key: {key}")
        value[key] = item
    return value


def _invalid_constant(value):
    raise ValueError(f"Invalid JSON constant: {value}")


def _decode(result):
    if result.isError:
        text = "\n".join(c.text for c in result.content if c.type == "text")
        raise RuntimeError(f"MCP tool error: {text}")
    if result.structuredContent is not None:
        value = result.structuredContent
    else:
        text = "\n".join(c.text for c in result.content if c.type == "text")
        value = json.loads(text, object_pairs_hook=_json_object, parse_constant=_invalid_constant)
    if not isinstance(value, dict):
        raise ValueError("MCP response must be a JSON object")
    return value


class _ChannelClient:
    """Retain live executions and their shared MCP event loop for one Atlas client."""

    def __init__(self, atlas):
        self.atlas = atlas
        self.channels: dict[str, Channel] = {}
        self.lock = threading.RLock()
        self.connecting = 0
        self.portal = None
        self.portal_cm = None

    def get(self, channel_id):
        with self.lock:
            return self.channels.get(channel_id)

    def _binding(self, contract_id, deadline):
        """Return the Python request and response classes for this contract.

        Atlas supplies the contract's service type, such as ``map/srv/GetMode``,
        rather than Python classes. Use that type to import the caller's local
        ``map_mcp`` module and look up ``_SRV_TYPES["GetMode"]``, which returns
        ``(GetMode_Request, GetMode_Response)``. These classes are generated
        from the service's .srv definition by ``rbnx codegen --mcp`` and provide
        the conversion between Python objects and MCP request/response data.

        The return value contains classes, not message instances. This method
        checks the contract and locates its types; it does not invoke the
        capability. ``deadline`` bounds the Atlas metadata query.
        """
        atlas = self.atlas
        resp = atlas._wire_stub.QueryContract(
            atlas._wire_pb.QueryContractRequest(contract_id=contract_id), timeout=_remaining(deadline),
        )
        if not resp.found:
            raise ValueError(f"Unknown contract: {contract_id}")
        desc = from_pb_contract(resp.contract)
        if desc.mode != "rpc":
            raise ValueError(f"MCP channel requires rpc mode: {contract_id}")
        try:
            package, kind, name = desc.io_srv_type.split("/")
            if kind != "srv":
                raise ValueError(desc.io_srv_type)
            module = importlib.import_module(f"{package}_mcp")
            return module._SRV_TYPES[name]
        except (ValueError, ImportError, AttributeError, KeyError) as exc:
            raise RuntimeError(
                f"Missing MCP types for {contract_id}; regenerate with rbnx codegen --mcp"
            ) from exc

    def connect(self, *, consumer_id, provider_id, contract_id, transport, _deadline=None):
        from .atlas import _resolve_transport
        if _resolve_transport(transport) != Transport.MCP:
            raise ValueError("connect_channel currently supports MCP only")
        business = contract_id.rsplit("/", 1)[-1] not in {"status", "cancel"}
        with self.lock:
            limit = 1024 if business else 1024
            if len(self.channels) + self.connecting >= limit:
                raise RuntimeError("Atlas client channel capacity exhausted; two slots are reserved for companions")
            self.connecting += 1
        try:
            return self._connect(consumer_id, provider_id, contract_id,
                                 _deadline if _deadline is not None else time.monotonic() + 30)
        finally:
            with self.lock:
                self.connecting -= 1

    def _connect(self, consumer_id, provider_id, contract_id, deadline):
        atlas = self.atlas
        binding = self._binding(contract_id, deadline)
        companions = {}
        if contract_id.rsplit("/", 1)[-1] not in {"status", "cancel"}:
            resp = atlas._wire_stub.Query(atlas._wire_pb.QueryRequest(
                id=provider_id, transport=atlas._transport_enum(Transport.MCP),
            ), timeout=_remaining(deadline))
            available = {c.contract_id for p in resp.providers for c in p.capabilities}
            companions = {name: f"{contract_id}/{name}" for name in ("status", "cancel")
                          if f"{contract_id}/{name}" in available}
            if len(companions) == 1:
                raise ValueError(f"Async capability {contract_id} must declare both /status and /cancel")
            # Bind the status and cancel capabilities.
            for cid in companions.values():
                self._binding(cid, deadline)
        with self.lock:
            if self.portal is None:
                from anyio.from_thread import start_blocking_portal
                self.portal_cm = start_blocking_portal(name="robonix-mcp-client")
                self.portal = self.portal_cm.__enter__()
        ch = atlas._allocate_channel(consumer_id, provider_id, contract_id, Transport.MCP,
                                     timeout=_remaining(deadline))
        execution = _Execution(self, ch, consumer_id, binding, companions)
        ch._execution = execution
        try:
            execution.session_cm = self.portal.wrap_async_context_manager(_session(ch.endpoint, _remaining(deadline)))
            execution.session = execution.session_cm.__enter__()
        except Exception as exc:
            execution.session_cm = None
            try:
                execution.close()
            except Exception as cleanup:
                with self.lock:
                    self.channels[ch.channel_id] = ch
                raise RuntimeError(f"MCP setup failed: {exc}; cleanup failed: {cleanup}") from exc
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

    def __init__(self, client, channel, consumer_id, binding, companions):
        self.client = client
        self.channel = channel
        self.consumer_id = consumer_id
        self.binding = binding
        self.companions = companions
        self.session_cm = None
        self.session = None
        self.run_id = None
        self.raw_response = None
        self.started = False
        self.accepted = False
        self.terminal = False
        self.closing = False
        self.lock = threading.RLock()
        self.close_lock = threading.Lock()
        self.session_lock = threading.RLock()

    def _error(self, message):
        return RuntimeError(f"{message}; channel {self.channel.channel_id} retained; use get_channel() to recover")

    def _request(self, request, timeout):
        cid = self.channel.contract_id
        output_cls = self.binding[1]
        deadline = None if timeout is None else time.monotonic() + timeout
        acquired = submitted = False
        try:
            args = request if isinstance(request, dict) else request.to_dict()
            if self.session is None:
                raise RuntimeError("MCP session unavailable")
            if deadline is None:
                acquired = self.session_lock.acquire()
            else:
                acquired = self.session_lock.acquire(timeout=_remaining(deadline))
            if not acquired:
                raise TimeoutError("MCP session is busy")
            remaining = None if deadline is None else _remaining(deadline)
            submitted = True
            result = self.client.portal.call(partial(
                _invoke, self.session, cid.rsplit("/", 1)[-1], args, remaining,
            ))
            # A synchronous handler has returned even if its business payload is malformed.
            if not self.companions:
                with self.lock:
                    self.terminal = True
            raw = _decode(result)
            with self.lock:
                self.raw_response = raw
            return output_cls.from_dict(raw), raw
        except Exception as exc:
            if not submitted:
                with self.lock:
                    self.terminal = True
            raise self._error(f"MCP request {cid} failed: {exc}") from exc
        finally:
            if acquired:
                self.session_lock.release()

    def call(self, request, *, timeout=None):
        input_cls = self.binding[0]
        if not isinstance(request, (dict, input_cls)):
            raise TypeError(f"Expected {input_cls.__name__} or dict")
        with self.lock:
            if self.started or self.closing or self.channel._closed:
                raise RuntimeError("A channel can execute only one capability call")
            self.started = True
        result, raw = self._request(request, timeout)
        with self.lock:
            if self.companions and raw.get("accepted") is False:
                self.terminal = True
                return result
            run_id = raw.get("run_id")
            if run_id is None:
                run_id = ""
            if not isinstance(run_id, str):
                raise self._error("MCP run_id must be a string")
            self.run_id = run_id
            self.accepted = True
            self.terminal = not self.companions
        return result

    def companion(self, name, *, timeout=30):
        with self.lock:
            if self.channel._closed or not self.accepted or name not in self.companions:
                raise self._error(f"Cannot {name}: no confirmed async execution")
            args = {"run_id": self.run_id} if self.run_id else {}
        # Companion calls acquire independent Atlas channels, including their own sessions.
        deadline = time.monotonic() + timeout
        ch = self.client.connect(consumer_id=self.consumer_id, provider_id=self.channel.provider_id,
                                 contract_id=self.companions[name], transport=Transport.MCP, _deadline=deadline)
        try:
            result = ch.call(args, timeout=_remaining(deadline))
        finally:
            ch.close()
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
        if self.started and not self.terminal:
            if not self.accepted:
                raise self._error("Execution termination is unconfirmed")
            self.companion("cancel", timeout=max(0, deadline - time.monotonic()))
            while not self.terminal:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise self._error("Timed out waiting for execution termination")
                self.companion("status", timeout=remaining)
                if not self.terminal:
                    time.sleep(min(2, max(0, deadline - time.monotonic())))
        if self.session_cm is not None:
            if not self.session_lock.acquire(timeout=5):
                raise self._error("MCP session still in use during cleanup")
            try:
                self.session_cm.__exit__(None, None, None)
            except Exception as exc:
                raise self._error(f"MCP session cleanup failed: {exc}") from exc
            finally:
                self.session_cm = None
                self.session = None
                self.session_lock.release()
        try:
            self.client.disconnect(self.channel)
        except Exception as exc:
            raise self._error(f"Atlas disconnect failed: {exc}") from exc
