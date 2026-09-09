import asyncio

from .core import parse_json_object, parse_robot_context, require_object

SCENE_CONTRACT = "robonix/system/scene/get_robot_context"

def decode_mcp_response(result) -> dict:
    if result.isError:
        raise RuntimeError(
            "Scene get_robot_context returned an MCP tool error"
        )

    if result.structuredContent is not None:
        return require_object(
            result.structuredContent,
            "Scene structuredContent",
        )

    blocks = result.content
    if len(blocks) != 1 or getattr(blocks[0], "type", None) != "text":
        raise ValueError("Scene must return one JSON object")

    return parse_json_object(blocks[0].text, "Scene response")

async def fetch_robot_context(consumer_id: str, scene_provider_id: str, timeout_s: float):
    """Open the exact provider's Atlas channel and close it on every exit.

    Imports are delayed so pure tests need no generated Robonix modules.
    The async timeout bounds MCP observation; Atlas uses the existing sync
    SDK and its calls are not covered by this timeout.
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
            """Use the MCP 1.x SDK and return validated Scene fields."""
            async with streamablehttp_client(channel.endpoint) as (read, write, _):
                async with ClientSession(read, write) as session:
                    await session.initialize()
                    result = await session.call_tool("get_robot_context", {})
                    return parse_robot_context(decode_mcp_response(result))

        return await asyncio.wait_for(observe(), timeout=timeout_s)