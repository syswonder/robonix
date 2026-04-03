# SPDX-License-Identifier: MulanPSL-2.0
"""Contract-enforced MCP tools on top of FastMCP + robonix-codegen ROS dataclasses.

Goals (see repo ``rust/contracts/*.toml`` + ``robonix-codegen --lang mcp``):

- **Input / output types** in your handler **must** be the generated Python message classes
  that match the contract's ``[io]`` (same names as ROS IDL under
  ``rust/crates/robonix-interfaces/lib``).
- You **declare** ``contract_id``, ``input_cls``, and ``output_cls``; we **validate** that
  your function's type annotations match those classes (runtime check at import).
- **MCP wire**: tool ``arguments`` JSON is exactly ``input_cls.to_dict()`` / ``from_dict()``
  (top-level keys = ROS message fields). FastMCP normally maps Python *parameter names*
  to JSON keys; we generate a **shim** whose parameters match those wire keys, then call
  your handler as ``user_fn(msg: input_cls) -> output_cls``.

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
from typing import Any, Callable, TypeVar, get_type_hints

from mcp.server.fastmcp import FastMCP

logger = logging.getLogger(__name__)

T = TypeVar("T")


def _validate_handler_matches_contract(
    user_fn: Callable[..., Any],
    *,
    contract_id: str,
    input_cls: type,
    output_cls: type | None,
) -> None:
    """Ensure annotations match ``input_cls`` / ``output_cls`` (mandatory contract alignment)."""
    hints = get_type_hints(user_fn, globalns=getattr(user_fn, "__globals__", None))
    sig = inspect.signature(user_fn)
    params = list(sig.parameters.values())
    if len(params) != 1:
        raise TypeError(
            f"[mcp_contract:{contract_id}] {user_fn.__name__} must have exactly one parameter "
            f"(input message), got {sig}"
        )
    p0 = params[0]
    if p0.kind not in (inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY):
        raise TypeError(
            f"[mcp_contract:{contract_id}] invalid parameter kind for {user_fn.__name__}"
        )
    ann_in = hints.get(p0.name)
    if ann_in is None:
        raise TypeError(
            f"[mcp_contract:{contract_id}] parameter {p0.name!r} must be annotated "
            f"as {input_cls.__qualname__}"
        )
    if ann_in is not input_cls:
        raise TypeError(
            f"[mcp_contract:{contract_id}] {user_fn.__name__}: parameter {p0.name!r} "
            f"must be annotated as {input_cls!r}, got {ann_in!r}"
        )
    if output_cls is not None:
        ann_out = hints.get("return")
        if ann_out is None:
            raise TypeError(
                f"[mcp_contract:{contract_id}] {user_fn.__name__} must annotate return as "
                f"{output_cls!r}"
            )
        if ann_out is not output_cls:
            raise TypeError(
                f"[mcp_contract:{contract_id}] {user_fn.__name__}: return must be "
                f"{output_cls!r}, got {ann_out!r}"
            )


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
    input_cls: type,
    output_cls: type | None = None,
    name: str | None = None,
    structured_output: bool | None = None,
) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Register an MCP tool with contract-bound, codegen-typed handler.

    Parameters
    ----------
    contract_id
        Same string as ``[contract] id`` in the TOML under ``rust/contracts/`` (e.g.
        ``robonix/sys/memory/search``). Used for error messages and traceability.
    input_cls / output_cls
        Generated module (e.g. ``std_msgs_mcp.String``) for the ROS types listed in that
        contract's ``[io]`` section.

    The user function **must** be annotated as::

        def tool(msg: <input_cls>) -> <output_cls>:
        # or
        async def tool(msg: <input_cls>) -> <output_cls>:
    """

    def decorator(user_fn: Callable[..., Any]) -> Callable[..., Any]:
        _validate_handler_matches_contract(
            user_fn,
            contract_id=contract_id,
            input_cls=input_cls,
            output_cls=output_cls,
        )
        shim = _make_shim(user_fn, input_cls, output_cls)
        mcp.add_tool(
            shim,
            name=name or user_fn.__name__,
            description=(user_fn.__doc__ or "").strip(),
            structured_output=structured_output,
        )
        return user_fn

    return decorator
