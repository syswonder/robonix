# SPDX-License-Identifier: MulanPSL-2.0
"""Minimal async OpenAI-compatible LLM client for scene graph inference.

Reuses the same VLM_BASE_URL / VLM_API_KEY / VLM_MODEL environment
variables already used by the VLM fallback detector in perception_vlm.py.
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
        self.model = (
            model
            or os.environ.get("VLM_MODEL")
            or os.environ.get("OPENAI_MODEL")
            or "gpt-4o-mini"
        )
        self.timeout = timeout

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
    ) -> dict[str, Any]:
        """Send a chat-completions request expecting JSON output.

        Returns the parsed JSON dict, or ``{}`` on any failure.
        Never raises — all errors are logged and swallowed.
        """
        if not self.available:
            return {}

        url = f"{self.base_url}/chat/completions"
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        body: dict[str, Any] = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message},
            ],
            "temperature": 0.0,
        }

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
            log.warning("[scene-graph-llm] request failed: %s", e)
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
