# SPDX-License-Identifier: MulanPSL-2.0
"""Contract-enforced MCP tools on top of FastMCP + robonix-codegen ROS dataclasses.

Goals (see repo ``rust/contracts/*.toml`` + ``robonix-codegen --lang mcp``):

- **Declare** ``contract_id`` on ``@mcp_contract``; **input/output codegen types** come from
  the handler's type annotations (single parameter + return). Those types must match the
  contract's ``[io]`` (``robonix-codegen --lang mcp``).
- **MCP wire**: tool ``arguments`` JSON matches ``input_cls.to_dict()`` / ``from_dict()``
  (top-level keys = ROS message fields). FastMCP normally maps Python *parameter names*
  to JSON keys; we generate a **shim** whose parameters match those wire keys, then call
  your handler.

Handlers may be **sync or async**; the shim is always async and awaits only when needed.

Return values that are codegen message instances are passed through ``.to_dict()`` before JSON
serialization so binary fields (e.g. ``sensor_msgs/Image.data``) become base64 strings.

The registered shim’s **return annotation** is ``typing.Any`` (not ``output_cls``): FastMCP/Pydantic
would otherwise coerce the JSON ``dict`` back into a codegen class and reject wire types (e.g.
base64 ``data`` strings vs. buggy ``bytes``/``int`` fields in generated stubs).

Do **not** register the same function with ``@mcp.tool()`` — use only ``@mcp_contract``.
"""

from __future__ import annotations

import inspect
import logging
from typing import Any, Callable, get_type_hints

from mcp.server.fastmcp import FastMCP

logger = logging.getLogger(__name__)


def _io_types_from_handler(
    user_fn: Callable[..., Any], *, contract_id: str
) -> tuple[type, type | None]:
    """Resolve codegen input/output classes from the handler's annotations."""
    globalns = getattr(user_fn, "__globals__", None) or {}
    hints = get_type_hints(user_fn, globalns=globalns, localns=globalns)
    sig = inspect.signature(user_fn)
    params = [
        p
        for p in sig.parameters.values()
        if p.kind in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)
    ]
    if len(params) != 1:
        raise TypeError(
            f"[mcp_contract:{contract_id}] {user_fn.__name__} must have exactly one parameter "
            f"(input message), got {sig}"
        )
    p0 = params[0]
    ann_in = hints.get(p0.name)
    if ann_in is None:
        raise TypeError(
            f"[mcp_contract:{contract_id}] parameter {p0.name!r} must be annotated with the "
            "codegen input message type"
        )
    if not isinstance(ann_in, type):
        raise TypeError(
            f"[mcp_contract:{contract_id}] parameter {p0.name!r} must be a class, got {ann_in!r}"
        )
    ann_out = hints.get("return")
    if ann_out is None:
        raise TypeError(
            f"[mcp_contract:{contract_id}] {user_fn.__name__} must annotate return type "
            "(codegen output message class)"
        )
    if ann_out is type(None):
        return ann_in, None
    if not isinstance(ann_out, type):
        raise TypeError(
            f"[mcp_contract:{contract_id}] return must be a codegen message class, got {ann_out!r}"
        )
    return ann_in, ann_out


def _json_prop_to_py_type(prop: dict[str, Any]) -> type:
    t = prop.get("type", "string")
    if t == "string":
        return str
    if t == "integer":
        return int
    if t == "number":
        return float
    if t == "boolean":
        return bool
    if t == "array":
        return list
    if t == "object":
        return dict
    logger.warning("[robonix_mcp_contract] unknown JSON schema type %r, using Any", t)
    return Any  # type: ignore[return-value]


def _wire_output_for_json(value: Any) -> Any:
    """Turn codegen ROS dataclass results into JSON-safe values.

    FastMCP serializes tool returns with JSON — raw ``bytes`` (e.g. ``sensor_msgs/Image.data``)
    are not UTF-8 and must go through ``to_dict()`` (base64 in wire format).
    """
    if value is None:
        return value
    if isinstance(value, (dict, list, str, int, float, bool)):
        return value
    td = getattr(value, "to_dict", None)
    if callable(td):
        return td()
    return value


def _make_shim(user_fn: Callable[..., Any], input_cls: type, out_cls: type | None) -> Callable[..., Any]:
    """Async shim: MCP ``arguments`` dict → ``input_cls.from_dict`` → ``user_fn(msg)``."""
    schema = input_cls.json_schema()
    props: dict[str, Any] = schema.get("properties") or {}

    if not props:

        async def _noarg_shim() -> Any:
            msg_in = input_cls.from_dict({})
            r = user_fn(msg_in)
            if inspect.isawaitable(r):
                r = await r
            return _wire_output_for_json(r)

        shim = _noarg_shim
    else:
        keys = list(props.keys())
        param_decls: list[str] = []
        ns: dict[str, Any] = {
            "_input_cls": input_cls,
            "_user_fn": user_fn,
            "_wire_output_for_json": _wire_output_for_json,
            "inspect": inspect,
            "str": str,
            "int": int,
            "float": float,
            "bool": bool,
            "list": list,
            "dict": dict,
            "Any": Any,
        }
        for k in keys:
            prop = props[k]
            if prop.get("type") == "object" and prop.get("properties"):
                py_t = dict  # nested ROS sub-messages (Pose, Header, …)
            else:
                py_t = _json_prop_to_py_type(prop)
            ann = "Any" if py_t is Any else getattr(py_t, "__name__", "Any")
            param_decls.append(f"{k}: {ann}")
        dict_literal = ", ".join(f"{k!r}: {k}" for k in keys)
        code = (
            f"async def _shim({', '.join(param_decls)}) -> Any:\n"
            f"    _msg = _input_cls.from_dict({{{dict_literal}}})\n"
            f"    _r = _user_fn(_msg)\n"
            f"    if inspect.isawaitable(_r):\n"
            f"        _r = await _r\n"
            f"    return _wire_output_for_json(_r)\n"
        )
        exec(code, ns, ns)  # noqa: S102 — schema-driven, fixed template
        shim = ns["_shim"]

    shim.__name__ = user_fn.__name__
    shim.__doc__ = user_fn.__doc__ or ""
    shim.__module__ = user_fn.__module__
    shim.__qualname__ = user_fn.__qualname__
    if out_cls is not None:
        ann = dict(getattr(shim, "__annotations__", {}))
        # Actual return is JSON wire (dict / primitives) after _wire_output_for_json — not
        # output_cls, or Pydantic will validate dict fields against codegen types (Image.data, …).
        ann["return"] = Any
        shim.__annotations__ = ann
    return shim


def mcp_contract(
    mcp: FastMCP,
    *,
    contract_id: str,
    name: str | None = None,
    structured_output: bool | None = None,
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Register an MCP tool bound to a contract.

    Parameters
    ----------
    contract_id
        Same string as ``[contract] id`` in the TOML under ``rust/contracts/`` (e.g.
        ``robonix/system/memorysearch``).

    The handler **must** annotate exactly one parameter and the return type with the codegen
    message classes for that contract's ``[io]`` section, e.g.
    ``async def f(msg: std_msgs_mcp.String) -> std_msgs_mcp.String: ...``
    """

    def decorator(user_fn: Callable[..., Any]) -> Callable[..., Any]:
        input_cls, output_cls = _io_types_from_handler(user_fn, contract_id=contract_id)
        shim = _make_shim(user_fn, input_cls, output_cls)
        mcp.add_tool(
            shim,
            name=name or user_fn.__name__,
            description=(user_fn.__doc__ or "").strip(),
            structured_output=structured_output,
        )
        # Stash the codegen IO classes + contract_id on the original handler
        # so the driver's atlas registration can derive the JSON Schema and
        # description without rediscovering them. Keeps atlas DeclareInterface
        # in lockstep with the FastMCP registration above.
        user_fn._robonix_input_cls = input_cls    # type: ignore[attr-defined]
        user_fn._robonix_output_cls = output_cls  # type: ignore[attr-defined]
        user_fn._robonix_contract_id = contract_id  # type: ignore[attr-defined]
        user_fn._robonix_tool_name = name or user_fn.__name__  # type: ignore[attr-defined]
        return user_fn

    return decorator
