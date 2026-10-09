# SPDX-License-Identifier: MulanPSL-2.0
"""A one-sentence VLM caption per settled object, from its best photograph.

Once per object, and never over a person's caption (`set_object_caption`
refuses that); clearing a person's caption hands it back to the model.
"""
from __future__ import annotations

import asyncio
import base64
import logging
import os
import time

log = logging.getLogger(__name__)

_SYSTEM_PROMPT = (
    "You describe one object in a robot's map so a person can tell it apart "
    "from others of the same kind. Reply as JSON: {\"caption\": \"...\"}. "
    "One short sentence: colour, material, shape or anything distinctive "
    "that is visible. Do not guess what is not visible. "
)
# SCENE_CAPTION_LANG picks the caption language; operators read Chinese.
_LANGUAGE = {
    "zh": "Write it in Simplified Chinese, at most 30 characters.",
    "en": "Write it in English, at most 15 words.",
}
# So one unreadable photograph cannot take every call.
_RETRY_AFTER_S = 300.0


async def caption_loop(registry, views, map_binding, llm, *,
                       period_s: float = 5.0) -> None:
    """Caption one object per period; runs until cancelled."""
    tried: dict[str, float] = {}
    while True:
        await asyncio.sleep(period_s)
        if not llm.available:
            continue
        try:
            await caption_one(registry, views, map_binding, llm, tried,
                              time.monotonic())
        except Exception:  # noqa: BLE001
            log.warning("[scene-caption] caption pass failed", exc_info=True)


async def caption_one(registry, views, map_binding, llm, tried: dict,
                      now: float) -> str:
    """Caption the first object that needs one; returns its id or ""."""
    partition = views.partition(map_binding)
    for obj in (await registry.snapshot()).values():
        if obj.caption or not obj.settled:
            continue
        if now - tried.get(obj.object_id, -_RETRY_AFTER_S) < _RETRY_AFTER_S:
            continue
        rows = await asyncio.to_thread(views.views, partition, obj.object_id)
        if not rows:
            continue
        tried[obj.object_id] = now
        jpeg = await asyncio.to_thread(
            views.read, partition, obj.object_id, rows[0]["index"])
        if not jpeg:
            continue
        reply = await llm.chat_json(
            _SYSTEM_PROMPT + _LANGUAGE.get(
                os.environ.get("SCENE_CAPTION_LANG", "zh"), _LANGUAGE["zh"]),
            f"The object is a {obj.label}.",
            images=[base64.b64encode(jpeg).decode("ascii")])
        caption = str((reply or {}).get("caption") or "").strip()
        if not caption:
            continue
        async with registry.lock():
            if registry.get_object(obj.object_id) is None:
                continue
            try:
                registry.set_object_caption(
                    obj.object_id, caption, source="model", now=time.time())
            except ValueError:
                continue
        return obj.object_id
    return ""
