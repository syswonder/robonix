# SPDX-License-Identifier: MulanPSL-2.0
"""MCP sessions and generated message bindings owned by an execution Channel."""
from __future__ import annotations

import atexit
import importlib
import json
import math
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
        uri = httpx.URL(endpoint if endpoint.startswith(("http://", "https://")) else f"http://{endpoint}")
        if uri.path in ("", "/"):
            uri = uri.copy_with(path="/mcp")
        resolved = uri
        initializing = True

        async def use_resolved_url(request):
            if request.url == uri:
                request.url = resolved

        async def check_redirect(response):
            nonlocal resolved
            if not response.has_redirect_location:
                return
            url = response.request.url
            target = url.join(response.headers["location"])
            # Old registrations may differ from the MCP route by a trailing slash.
            if (initializing and resolved == uri and response.status_code in (307, 308)
                    and {url.path, target.path} == {"/mcp", "/mcp/"}
                    and target == url.copy_with(path=target.path)):
                resolved = target
            else:
                response.raise_for_status()

        # Provider traffic must not depend on the consumer's shell proxy settings.
        client = await stack.enter_async_context(httpx.AsyncClient(
            trust_env=False, timeout=httpx.Timeout(5, read=None), follow_redirects=True,
            event_hooks={"request": [use_resolved_url], "response": [check_redirect]},
        ))
        transport = streamable_http_client(str(uri), http_client=client)
        streams = await stack.enter_async_context(transport)
        session = await stack.enter_async_context(ClientSession(streams[0], streams[1]))
        import anyio
        with anyio.fail_after(timeout):
            await session.initialize()
        initializing = False
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


def _as_structured(value):
    # FastMCP's structured output carries non-finite floats as null.
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _as_structured(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_as_structured(item) for item in value]
    return value


def _decode(result):
    text = "\n".join(c.text for c in result.content if c.type == "text")
    if result.isError:
        raise RuntimeError(f"MCP tool error: {text}")
    value = result.structuredContent
    if value is None:
        value = json.loads(text, object_pairs_hook=_json_object)
    elif value.keys() == {"result"} and isinstance(value["result"], dict):
        # FastMCP may wrap the structured reply while leaving the text reply flat.
        try:
            flat = json.loads(text, object_pairs_hook=_json_object)
        except ValueError:
            pass
        else:
            if _as_structured(flat) == value["result"]:
                value = flat
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
                client.portal_cm = start_blocking_portal()
                client.portal = client.portal_cm.__enter__()
                atexit.register(client.stop_portal)
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
            # A tool error rejects the start before there is a run to cancel.
            if result.isError or not execution.companions:
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
