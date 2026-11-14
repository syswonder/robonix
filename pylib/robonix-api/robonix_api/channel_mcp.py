# SPDX-License-Identifier: MulanPSL-2.0
"""MCP sessions and generated message bindings owned by an execution Channel."""
from __future__ import annotations

import importlib
import json
import threading
import time
from contextlib import AsyncExitStack, asynccontextmanager
from functools import partial

from .channel import _remaining


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


def binding(descriptor):
    try:
        package, kind, name = descriptor.io_srv_type.split("/")
        if kind != "srv":
            raise ValueError(descriptor.io_srv_type)
        return importlib.import_module(f"{package}_mcp")._SRV_TYPES[name]
    except (ValueError, ImportError, AttributeError, KeyError) as exc:
        raise RuntimeError(
            f"Missing MCP types for {descriptor.id}; regenerate with rbnx codegen --mcp"
        ) from exc


class _Mcp:
    """Own one execution's MCP session on its Atlas client's shared event loop."""

    def __init__(self, execution):
        self.execution = execution
        self.binding = execution.binding
        self.session_cm = None
        self.session = None
        self.session_lock = threading.RLock()

    def prepare(self, deadline):
        client = self.execution.client
        with client.lock:
            if client.portal is None:
                from anyio.from_thread import start_blocking_portal
                client.portal_cm = start_blocking_portal(name="robonix-mcp-client")
                client.portal = client.portal_cm.__enter__()
        self.session_cm = client.portal.wrap_async_context_manager(
            _session(self.execution.channel.endpoint, _remaining(deadline)),
        )
        try:
            self.session = self.session_cm.__enter__()
        except Exception:
            self.session_cm = None
            raise

    def call(self, request, timeout):
        input_cls = self.binding[0]
        if not isinstance(request, (dict, input_cls)):
            raise TypeError(f"Expected {input_cls.__name__} or dict")
        self.execution.start()
        result, raw = self._request(request, timeout)
        return self.execution.complete(result, raw)

    def _request(self, request, timeout):
        execution = self.execution
        cid = execution.channel.contract_id
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
            result = execution.client.portal.call(partial(
                _invoke, self.session, cid.rsplit("/", 1)[-1], args, remaining,
            ))
            # A synchronous handler has returned even if its business payload is malformed.
            if not execution.companions:
                with execution.lock:
                    execution.terminal = True
            raw = _decode(result)
            with execution.lock:
                execution.raw_response = raw
            return output_cls.from_dict(raw), raw
        except Exception as exc:
            if not submitted:
                with execution.lock:
                    execution.terminal = True
            raise execution._error(f"MCP request {cid} failed: {exc}") from exc
        finally:
            if acquired:
                self.session_lock.release()

    def finish(self, timeout):
        raise self.execution._error("Execution termination is unconfirmed")

    def close(self):
        if self.session_cm is not None:
            if not self.session_lock.acquire(timeout=5):
                raise self.execution._error("MCP session still in use during cleanup")
            try:
                self.session_cm.__exit__(None, None, None)
            except Exception as exc:
                raise self.execution._error(f"MCP session cleanup failed: {exc}") from exc
            finally:
                self.session_cm = None
                self.session = None
                self.session_lock.release()
