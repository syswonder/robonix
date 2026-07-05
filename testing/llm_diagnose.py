# SPDX-License-Identifier: MulanPSL-2.0
"""Generate an LLM-assisted CI analysis from bounded diagnostic context."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import urllib.error
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _extract_json_object(text: str) -> dict[str, Any]:
    try:
        value = json.loads(text)
        return value if isinstance(value, dict) else {"summary": str(value)}
    except json.JSONDecodeError:
        pass
    match = re.search(r"\{.*\}", text, re.S)
    if match:
        try:
            value = json.loads(match.group(0))
            return value if isinstance(value, dict) else {"summary": str(value)}
        except json.JSONDecodeError:
            pass
    return {"title": "LLM analysis returned non-JSON output", "summary": text[:4000], "confidence": "low"}


def _post_chat(base_url: str, api_key: str, model: str, messages: list[dict[str, str]], timeout: int) -> dict[str, Any]:
    url = base_url.rstrip("/") + "/chat/completions"
    payload = {
        "model": model,
        "messages": messages,
        "temperature": 0.1,
        "response_format": {"type": "json_object"},
    }
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
            "Accept": "application/json",
            "User-Agent": "robonix-ci-diagnostics",
        },
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _build_messages(context: dict[str, Any]) -> list[dict[str, str]]:
    schema = {
        "title": "short status title",
        "stage": "checkout|merge|build|boot|simulator|scenario|report|success|unknown",
        "summary": "2-4 sentence human-readable analysis",
        "pr_changes": ["what changed, using only context.change_set.pr_files or context.change_set.git_diff"],
        "test_result": "what Webots CI exercised and observed, using summary and logs",
        "likely_root_cause": "root cause if failing, otherwise empty or residual concern",
        "evidence": [{"source": "log or diff path", "line": 0, "text": "short evidence"}],
        "suggested_fix": "actionable fix if failing, otherwise follow-up/watch item",
        "risks_or_watchouts": ["possible issue to watch"],
        "confidence": "high|medium|low",
    }
    system = (
        "You are a Robonix CI diagnostic assistant. Analyze only the provided JSON context. "
        "Never claim access to files that are not in the context. Do not decide CI pass/fail; the scripts do that. "
        "The change summary MUST be derived only from context.change_set.pr_files or context.change_set.git_diff. "
        "Do not use logs, capability catalogs, provider names, repository layout, or pre-existing components to infer what changed. "
        "If context.change_set is empty or unavailable, say the change summary is unavailable. "
        "For push events, describe the tested commit only; do not call it a PR unless PR metadata is present. "
        "If tests passed, still summarize what changed, what was tested, and what humans should watch. "
        "Return one JSON object only, matching this schema: " + json.dumps(schema, ensure_ascii=False)
    )
    user = json.dumps(context, ensure_ascii=False)
    return [{"role": "system", "content": system}, {"role": "user", "content": user}]


def main() -> int:
    ap = argparse.ArgumentParser(description="Call DeepSeek for Robonix CI diagnostic analysis")
    ap.add_argument("--context-json", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--base-url", default=os.environ.get("DEEPSEEK_BASE_URL", "https://api.deepseek.com"))
    ap.add_argument("--model", default=os.environ.get("DEEPSEEK_MODEL", "deepseek-chat"))
    ap.add_argument("--api-key-env", default="DEEPSEEK_API_KEY")
    ap.add_argument("--timeout", type=int, default=90)
    args = ap.parse_args()

    api_key = os.environ.get(args.api_key_env, "")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    if not api_key:
        args.out.write_text(
            json.dumps(
                {
                    "generated_on": datetime.now(timezone.utc).isoformat(timespec="seconds"),
                    "available": False,
                    "title": "LLM analysis unavailable",
                    "summary": f"{args.api_key_env} was not configured for this run.",
                    "confidence": "low",
                },
                indent=2,
                ensure_ascii=False,
            )
            + "\n",
            encoding="utf-8",
        )
        return 0

    context = _load_json(args.context_json)
    try:
        response = _post_chat(args.base_url, api_key, args.model, _build_messages(context), args.timeout)
        content = response.get("choices", [{}])[0].get("message", {}).get("content", "")
        analysis = _extract_json_object(str(content))
        analysis["available"] = True
        analysis["provider"] = "deepseek"
        analysis["model"] = args.model
        analysis["generated_on"] = datetime.now(timezone.utc).isoformat(timespec="seconds")
    except (OSError, urllib.error.HTTPError, json.JSONDecodeError, KeyError, IndexError) as exc:
        analysis = {
            "available": False,
            "title": "LLM analysis failed",
            "summary": str(exc),
            "confidence": "low",
            "generated_on": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "provider": "deepseek",
            "model": args.model,
        }
        print(f"warning: LLM analysis failed: {exc}", file=sys.stderr)

    args.out.write_text(json.dumps(analysis, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
