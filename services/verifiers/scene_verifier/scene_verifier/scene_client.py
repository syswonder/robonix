"""Fetch and validate robot context from the Scene MCP service."""

import asyncio
import logging

from .core import parse_robot_context, require_object

SCENE_CONTRACT = "robonix/system/scene/get_robot_context"
log = logging.getLogger("scene_verifier")


async def fetch_robot_context(
    consumer_id: str,
    scene_provider_id: str,
    timeout_s: float,
):
    from robonix_api import ATLAS, Transport

    def observe():
        # Atlas setup, MCP calls and cleanup are synchronous client operations.
        with ATLAS.connect_channel(
            consumer_id=consumer_id,
            provider_id=scene_provider_id,
            contract_id=SCENE_CONTRACT,
            transport=Transport.MCP,
        ) as channel:
            channel.call({}, timeout=timeout_s)
            # Generated types supply defaults; verify the fields Scene actually sent.
            raw = channel.raw_response
            if "pose_known" not in raw and "result" in raw:
                raw = require_object(raw["result"], "Scene response.result")
            return parse_robot_context(raw)

    context = await asyncio.to_thread(observe)
    log.info("Scene context: %r", context)
    return context
