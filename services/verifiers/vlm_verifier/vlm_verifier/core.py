# SPDX-License-Identifier: MulanPSL-2.0
"""Transport-independent verifier request handling."""
import asyncio
import json
import logging

from .vlm import InsufficientEvidence, strict_object

log = logging.getLogger("vlm_verifier")


def camera_provider_ids(args):
    """Accept either the existing single provider or an ordered list."""
    if not isinstance(args, dict) or ("camera_provider_id" in args) == ("camera_provider_ids" in args):
        raise ValueError("provide exactly one of camera_provider_id or camera_provider_ids")
    ids = [args["camera_provider_id"]] if "camera_provider_id" in args else args["camera_provider_ids"]
    if not isinstance(ids, list) or not ids or any(not isinstance(item, str) or not item.strip() for item in ids):
        raise ValueError("camera providers must be nonempty strings")
    ids = [item.strip() for item in ids]
    if len(ids) != len(set(ids)):
        raise ValueError("camera providers must be unique")
    return ids


def parse_envelope(raw):
    """Validate required context while retaining JSON-or-string target values."""
    try:
        envelope = json.loads(raw, object_pairs_hook=strict_object)
    except (ValueError, TypeError):
        raise ValueError("args_json must contain a JSON object") from None
    if not isinstance(envelope, dict):
        raise ValueError("args_json must contain a JSON object")
    for key in ("target_provider_id", "target_contract_id", "target_description"):
        if not isinstance(envelope.get(key), str) or not envelope[key].strip():
            raise ValueError(f"{key} is required")
    if "target_args" not in envelope or "target_output" not in envelope:
        raise ValueError("target_args and target_output are required")
    camera_provider_ids(envelope.get("verifier_args"))
    return envelope


async def verify(call_id, raw, observe, evaluate):
    """Bound all work to 50 seconds and keep each request's evidence isolated."""
    if not isinstance(call_id, str) or not call_id.strip():
        raise ValueError("call_id is required")
    envelope = parse_envelope(raw)

    async def run():
        providers = camera_provider_ids(envelope["verifier_args"])
        captures = [asyncio.create_task(observe(provider)) for provider in providers]
        try:
            frames = await asyncio.wait_for(asyncio.gather(*captures), 5)
        finally:
            for capture in captures:
                capture.cancel()
            await asyncio.gather(*captures, return_exceptions=True)
        return await asyncio.wait_for(evaluate(envelope, dict(zip(providers, frames))), 40)

    try:
        verdict = await asyncio.wait_for(run(), 50)
    except asyncio.CancelledError:
        log.info("call_id=%r verification cancelled", call_id)
        raise
    except InsufficientEvidence:
        log.info("call_id=%r verification unavailable (insufficient visual evidence)", call_id)
        raise RuntimeError("visual verification unavailable (insufficient visual evidence)") from None
    except Exception as exc:
        log.warning("call_id=%r verification unavailable (%s)", call_id, type(exc).__name__)
        # Upstream exceptions may contain URLs, credentials, or response bodies.
        raise RuntimeError(f"visual verification unavailable ({type(exc).__name__})") from None
    log.info("call_id=%r verification passed=%s", call_id, verdict["passed"])
    return verdict
