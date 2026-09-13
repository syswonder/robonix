"""Fetch and validate robot context from the Scene MCP service."""

import asyncio
import logging

from .core import parse_json_object, parse_robot_context, require_object

SCENE_CONTRACT = "robonix/system/scene/get_robot_context"
log = logging.getLogger("scene_verifier")


def decode_mcp_response(result) -> dict:
    """Decode a Scene response and unwrap its optional result envelope."""
    if getattr(result, "isError", False):
        raise RuntimeError(
            "Scene get_robot_context returned an MCP tool error: "
            f"{getattr(result, 'content', None)!r}"
        )

    structured = getattr(result, "structuredContent", None)

    if structured is not None:
        raw = require_object(
            structured,
            "Scene structuredContent",
        )
    else:
        blocks = getattr(result, "content", None)
        if (
            blocks is None
            or len(blocks) != 1
            or getattr(blocks[0], "type", None) != "text"
        ):
            raise ValueError("Scene must return one JSON object")

        raw = parse_json_object(
            blocks[0].text,
            "Scene response",
        )

    # Accept both a direct Scene object and {"result": SceneObject}.
    # Keep field validation in parse_robot_context().
    if "pose_known" not in raw and "result" in raw:
        raw = require_object(
            raw["result"],
            "Scene response.result",
        )

    return raw


async def fetch_robot_context(
    consumer_id: str,
    scene_provider_id: str,
    timeout_s: float,
):
    """Fetch Scene context and parse it after closing the MCP session.

    Imports remain local so pure tests do not require generated modules.
    The timeout covers MCP observation, but not synchronous Atlas calls.
    """
    from robonix_api import ATLAS, Transport
    from mcp import ClientSession
    from mcp.client.streamable_http import streamablehttp_client

    with ATLAS.connect_capability(
        consumer_id=consumer_id,
        provider_id=scene_provider_id,
        contract_id=SCENE_CONTRACT,
        transport=Transport.MCP,
    ) as channel:

        async def observe():
            """Return the MCP response after closing its session."""
            async with streamablehttp_client(
                channel.endpoint
            ) as (read, write, _):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    result = await session.call_tool(
                        "get_robot_context", {}
                    )

                    log.info(
                        "Scene MCP response: "
                        "isError=%r structuredContent=%r content=%r",
                        getattr(result, "isError", None),
                        getattr(result, "structuredContent", None),
                        getattr(result, "content", None),
                    )

            return result

        result = await asyncio.wait_for(observe(), timeout=timeout_s)

    raw = decode_mcp_response(result)

    log.info(
        "Scene decoded response: type=%s value=%r",
        type(raw).__name__,
        raw,
    )

    log.info(
        "Scene pose_known: present=%s type=%s value=%r",
        "pose_known" in raw,
        type(raw.get("pose_known")).__name__,
        raw.get("pose_known"),
    )

    return parse_robot_context(raw)