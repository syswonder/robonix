# SPDX-License-Identifier: MulanPSL-2.0
"""Minimal async OpenAI-compatible LLM client for scene graph inference.

Reuses the same VLM_BASE_URL / VLM_API_KEY / VLM_MODEL / VLM_REASONING_EFFORT
environment variables already used by the VLM fallback detector in
perception_vlm.py.
"""
from __future__ import annotations

import json
import logging
import os
import re
from typing import Any

import httpx

log = logging.getLogger(__name__)


class SceneGraphLLMClient:
    """Thin wrapper around an OpenAI-compatible chat-completions endpoint.

    All errors are caught internally — callers always get a dict back
    (empty dict on failure) so the scene graph loop never crashes due
    to LLM issues.
    """

    def __init__(
        self,
        *,
        base_url: str | None = None,
        api_key: str | None = None,
        model: str | None = None,
        timeout: float = 30.0,
        reasoning_effort: str | None = None,
    ) -> None:
        self.base_url = (
            base_url
            or os.environ.get("VLM_BASE_URL")
            or os.environ.get("OPENAI_BASE_URL")
            or ""
        ).rstrip("/")
        self.api_key = (
            api_key
            or os.environ.get("VLM_API_KEY")
            or os.environ.get("OPENAI_API_KEY")
            or ""
        )
        # Relation inference uses the same VLM_MODEL as the rest of scene.
        self.model = (
            model
            or os.environ.get("VLM_MODEL")
            or os.environ.get("OPENAI_MODEL")
            or "gpt-4o-mini"
        )
        self.timeout = timeout
        # Forwarded only when non-empty, so non-reasoning models (and
        # providers that reject the field) are unaffected. Values:
        # minimal | low | medium | high; "minimal" = no thinking, which on
        # a reasoning model (e.g. doubao-seed-2-1-pro) answers a relation
        # prompt in ~2 s instead of blowing past the timeout while it
        # "thinks". Shared VLM-wide knob, opt-in: unset/empty → field omitted
        # (default), so a non-reasoning VLM_MODEL is untouched. An explicit
        # `reasoning_effort` constructor arg overrides the env.
        if reasoning_effort is not None:
            self.reasoning_effort = reasoning_effort.strip()
        else:
            self.reasoning_effort = os.environ.get("VLM_REASONING_EFFORT", "").strip()

        if not self.api_key:
            log.warning(
                "[scene-graph-llm] VLM_API_KEY not set; "
                "relation inference will return 'unknown'"
            )

    @property
    def available(self) -> bool:
        return bool(self.base_url and self.api_key)

    async def chat_json(
        self,
        system_prompt: str,
        user_message: str,
        *,
        timeout: float | None = None,
        images: list[str] | None = None,
    ) -> dict[str, Any]:
        """Send a chat-completions request expecting JSON output.

        Returns the parsed JSON dict, or ``{}`` on any failure.
        Never raises — all errors are logged and swallowed.

        ``images`` are base64-encoded JPEG strings (no data-url prefix). When
        present the user turn is sent as multimodal content
        (``[{text}, {image_url}, ...]``) so a vision model can see the frame;
        otherwise the user turn is the bare ``user_message`` string. The
        image-url shape matches the VLM perception detector
        (``perception_vlm._DETECTION_PROMPT`` request)."""
        if not self.available:
            return {}

        url = f"{self.base_url}/chat/completions"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        if images:
            user_content: Any = [{"type": "text", "text": user_message}]
            for img in images:
                user_content.append({
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{img}"},
                })
        else:
            user_content = user_message
        body: dict[str, Any] = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_content},
            ],
            "temperature": 0.0,
        }
        if self.reasoning_effort:
            body["reasoning_effort"] = self.reasoning_effort

        try:
            async with httpx.AsyncClient(
                timeout=timeout or self.timeout
            ) as client:
                r = await client.post(url, json=body, headers=headers)
                if r.status_code >= 400:
                    log.warning(
                        "[scene-graph-llm] HTTP %d: %s",
                        r.status_code,
                        r.text[:200],
                    )
                    return {}
                data = r.json()
        except (httpx.HTTPError, Exception) as e:  # noqa: BLE001
            # Include the exception type: a read timeout's str() is empty,
            # which historically logged a bare "request failed:" with no
            # clue it was a timeout.
            log.warning(
                "[scene-graph-llm] request failed: %s: %s",
                type(e).__name__,
                e,
            )
            return {}

        try:
            text = data["choices"][0]["message"]["content"]
        except (KeyError, IndexError):
            log.debug("[scene-graph-llm] unexpected response shape")
            return {}

        # Strip markdown fences if the model added them.
        text = re.sub(
            r"^```(?:json)?\s*|\s*```$", "", text.strip(), flags=re.MULTILINE
        )
        try:
            obj = json.loads(text)
        except json.JSONDecodeError:
            log.debug("[scene-graph-llm] non-JSON response: %s", text[:200])
            return {}

        return obj if isinstance(obj, dict) else {}
