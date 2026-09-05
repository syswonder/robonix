# SPDX-License-Identifier: MulanPSL-2.0
"""Contract-enforced MCP tools on top of FastMCP + robonix-codegen ROS dataclasses.

The decorator stashes the codegen IO classes + contract id on the original
handler so Capability.run() can derive atlas DeclareCapability metadata
(JSON Schema, description) without re-reflecting annotations.
"""
from __future__ import annotations

import inspect
import logging
from typing import TYPE_CHECKING, Any, Callable, get_type_hints

if TYPE_CHECKING:
    from mcp.server.fastmcp import FastMCP  # noqa: F401

__all__ = ["mcp_contract"]
log = logging.getLogger(__name__)


def io_types_from_handler(
    user_fn: Callable[..., Any], *, contract_id: str,
) -> tuple[type, type | None]:
    globalns = getattr(user_fn, "__globals__", None) or {}
    hints = get_type_hints(user_fn, globalns=globalns, localns=globalns)
    sig = inspect.signature(user_fn)
    params = [
        p for p in sig.parameters.values()
        if p.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    ]
    if len(params) != 1:
        raise TypeError(
            f"[mcp_contract:{contract_id}] {user_fn.__name__} must have exactly one parameter "
            f"(input message), got {sig}"
        )
    p0 = params[0]
    ann_in = hints.get(p0.name)
    if ann_in is None or not isinstance(ann_in, type):
        raise TypeError(
            f"[mcp_contract:{contract_id}] parameter {p0.name!r} must be annotated with the "
            "codegen input message type"
        )
    ann_out = hints.get("return")
    if ann_out is None:
        raise TypeError(
            f"[mcp_contract:{contract_id}] {user_fn.__name__} must annotate return type"
        )
    if ann_out is type(None):
        return ann_in, None
    if not isinstance(ann_out, type):
        raise TypeError(
            f"[mcp_contract:{contract_id}] return must be a codegen message class"
        )
    return ann_in, ann_out


def json_prop_to_py_type(prop: dict[str, Any]) -> type:
    t = prop.get("type", "string")
    return {"string": str, "integer": int, "number": float, "boolean": bool,
            "array": list, "object": dict}.get(t, Any)  # type: ignore[return-value]


def wire_output_for_json(value: Any) -> Any:
    if value is None or isinstance(value, (dict, list, str, int, float, bool)):
        return value
    td = getattr(value, "to_dict", None)
    return td() if callable(td) else value


def make_shim(user_fn: Callable[..., Any], input_cls: type, out_cls: type | None) -> Callable[..., Any]:
    schema = input_cls.json_schema()
    props: dict[str, Any] = schema.get("properties") or {}
    if not props:
        async def shim() -> Any:
            r = user_fn(input_cls.from_dict({}))
            if inspect.isawaitable(r):
                r = await r
            return wire_output_for_json(r)
    else:
        keys = list(props.keys())
        param_decls: list[str] = []
        ns: dict[str, Any] = {
            "input_cls": input_cls, "user_fn": user_fn,
            "wire_output_for_json": wire_output_for_json, "inspect": inspect,
            "str": str, "int": int, "float": float, "bool": bool,
            "list": list, "dict": dict, "Any": Any,
        }
        for k in keys:
            prop = props[k]
            if prop.get("type") == "object" and prop.get("properties"):
                py_t = dict  # nested ROS sub-messages (Pose, Header, …)
            else:
                py_t = json_prop_to_py_type(prop)
            ann = "Any" if py_t is Any else getattr(py_t, "__name__", "Any")
            param_decls.append(f"{k}: {ann}")
        dict_literal = ", ".join(f"{k!r}: {k}" for k in keys)
        code = (
            f"async def shim({', '.join(param_decls)}) -> Any:\n"
            f"    msg = input_cls.from_dict({{{dict_literal}}})\n"
            f"    r = user_fn(msg)\n"
            f"    if inspect.isawaitable(r):\n"
            f"        r = await r\n"
            f"    return wire_output_for_json(r)\n"
        )
        exec(code, ns, ns)  # noqa: S102 — schema-driven, fixed template
        shim = ns["shim"]
    shim.__name__ = user_fn.__name__
    shim.__doc__ = user_fn.__doc__ or ""
    shim.__module__ = user_fn.__module__
    shim.__qualname__ = user_fn.__qualname__
    if out_cls is not None:
        ann = dict(getattr(shim, "__annotations__", {}))
        # Actual return is JSON wire (dict / primitives) after wire_output_for_json — not
        # output_cls, or Pydantic will validate dict fields against codegen types (Image.data, …).
        ann["return"] = Any
        shim.__annotations__ = ann
    return shim


def mcp_contract(
    mcp: "FastMCP",
    *,
    contract_id: str,
    structured_output: bool | None = None,
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Register an MCP tool bound to a contract.

    Use directly with your own FastMCP app, OR use the provider's
    `@<provider>.mcp(...)` decorator (Primitive / Service / Skill) for a
    one-stop registration that also auto-declares to atlas + manages uvicorn.

    The MCP-server-side tool name is **always** the contract_id's leaf
    segment — executor's dispatch derives the same value from contract_id
    (see executor/dispatch/mcp.rs `mcp_tool_name`). They must match; we
    enforce that by deriving from a single source.

    The decorator stores these attributes on the original handler:

    * ``_robonix_input_cls``
    * ``_robonix_output_cls``
    * ``_robonix_contract_id``

    The provider framework picks these up via attribute reflection during run().
    """
    def decorator(user_fn: Callable[..., Any]) -> Callable[..., Any]:
        input_cls, output_cls = io_types_from_handler(user_fn, contract_id=contract_id)
        shim = make_shim(user_fn, input_cls, output_cls)
        tool_name = contract_id.rsplit("/", 1)[-1]
        mcp.add_tool(
            shim,
            name=tool_name,
            description=(user_fn.__doc__ or "").strip(),
            structured_output=structured_output,
        )
        user_fn._robonix_input_cls = input_cls    # type: ignore[attr-defined]
        user_fn._robonix_output_cls = output_cls  # type: ignore[attr-defined]
        user_fn._robonix_contract_id = contract_id  # type: ignore[attr-defined]
        return user_fn
    return decorator
