"""VLM helpers — image description and question answering.

VLM config priority:
  MEM_VLM_BASE_URL/KEY/MODEL  >  VLM_BASE_URL/KEY/MODEL (Pilot)  >  OPENAI_*

Falls back to template-based summary when VLM is unavailable.

Image saving is handled by ``RememberPipeline`` — when ``image_base64``
appears in ``kv``, the remember pipeline saves it to
``data/images/{node_id}/`` via ``ImageStore`` and populates
``MemoryNode.image_refs``.
"""

from __future__ import annotations

import base64
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

from ..storage.image_store import _DEFAULT_IMAGE_ROOT
from .types import ObjectCoord

log = logging.getLogger("scribe_mem")

# ── VLM config ─────────────────────────────────────────────────────────
# Priority: MEM_VLM_*  >  VLM_* (Pilot)  >  OPENAI_*  >  "gpt-4.1"

def _vlm_config() -> dict:
    return {
        "base_url": (os.environ.get("MEM_VLM_BASE_URL")
                     or os.environ.get("VLM_BASE_URL")
                     or os.environ.get("OPENAI_BASE_URL") or ""),
        "api_key": (os.environ.get("MEM_VLM_API_KEY")
                    or os.environ.get("VLM_API_KEY")
                    or os.environ.get("OPENAI_API_KEY") or ""),
        "model": (os.environ.get("MEM_VLM_MODEL")
                  or os.environ.get("VLM_MODEL")
                  or os.environ.get("OPENAI_MODEL") or "gpt-4.1"),
    }


def _vlm_available() -> bool:
    cfg = _vlm_config()
    return bool(cfg["api_key"]) and bool(cfg["base_url"])


# ── VLM image description ──────────────────────────────────────────────────

_VLM_PROMPT = (
    "You are a visual observer for an embodied robot. "
    "Describe the scene in one concise English sentence. "
    "Mention the key objects, their positions relative to each other, "
    "and the type of room or area visible. "
    "Detected objects: {objects}. "
    "Reply with ONLY the description sentence, no extra text."
)


async def _vlm_describe(
    image_bytes: bytes,
    objects: List[ObjectCoord],
) -> Optional[str]:
    """Send image + object list to VLM, return a one-line scene description.

    Returns None when VLM is unavailable or on any failure.
    The caller falls back to template-based summary.
    """
    cfg = _vlm_config()
    if not cfg["api_key"] or not cfg["base_url"]:
        return None

    import httpx

    # Build image content part
    img_b64 = base64.b64encode(image_bytes).decode("ascii")
    data_url = f"data:image/png;base64,{img_b64}"

    obj_labels = ", ".join(o.label for o in objects if o.label) or "unknown"
    prompt = _VLM_PROMPT.format(objects=obj_labels)

    url = f"{cfg['base_url'].rstrip('/')}/chat/completions"
    headers = {
        "Authorization": f"Bearer {cfg['api_key']}",
        "Content-Type": "application/json",
    }
    body: dict = {
        "model": cfg["model"],
        "messages": [{
            "role": "user",
            "content": [
                {"type": "image_url", "image_url": {"url": data_url}},
                {"type": "text", "text": prompt},
            ],
        }],
        "max_tokens": 128,
        "temperature": 0.0,
    }

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            r = await client.post(url, json=body, headers=headers)
            if r.status_code >= 400:
                log.warning("observe: VLM returned %d: %s", r.status_code, r.text[:200])
                return None
            data = r.json()
    except Exception as e:
        log.warning("observe: VLM call failed: %s: %s", type(e).__name__, e)
        return None

    try:
        content = data["choices"][0]["message"]["content"]
        result = content.strip().strip('"').strip("'")
        log.info("vlm_describe: %s", result[:200])
        return result
    except (KeyError, IndexError, TypeError):
        log.warning("observe: unexpected VLM response shape")
        return None


# ── VLM QA over stored images + memory context ─────────────────────────────

_VLM_QA_PROMPT = (
    "You are answering questions about scenes an embodied robot observed "
    "during its patrol. Below are:\n"
    "  1. Memory context — what the robot recorded at each observation "
    "(object labels, spatial positions, scene type, summary).\n"
    "  2. Images — the camera frames captured at those observations.\n"
    "\n"
    "Use BOTH the memory context AND the images to answer the user's "
    "question. Be concise — one or two sentences. "
    "If the answer is not determinable from the provided information, say so."
)


async def vlm_answer_question(
    query: str,
    image_paths: List[str],
    node_contexts: List[str] | None = None,
    image_root: str = "",
) -> Optional[str]:
    """Send user query + node context + stored images to VLM, return answer.

    Called by the search pipeline when ``vlm_qa=True`` and matching
    nodes carry ``image_refs``.  The VLM receives:
      - The user's question
      - Memory node context (summary, tags, objects, spatial)
      - The actual image files loaded from disk

    Returns None when VLM is unavailable or on any failure.
    """
    cfg = _vlm_config()
    if not cfg["api_key"] or not cfg["base_url"]:
        return None

    import httpx

    # Load images from disk
    content_parts: List[Dict[str, Any]] = []
    root = Path(image_root) if image_root else Path(_DEFAULT_IMAGE_ROOT).parent.parent

    for rel_path in image_paths[:3]:  # cap at 3 images to stay within context
        abs_path = root / rel_path
        if not abs_path.exists():
            log.debug("vlm_qa: image not found: %s", abs_path)
            continue
        try:
            img_bytes = abs_path.read_bytes()
            img_b64 = base64.b64encode(img_bytes).decode("ascii")
            # Detect format from extension: .jpg → jpeg, .png → png
            suffix = abs_path.suffix.lower()
            mime = "jpeg" if suffix in (".jpg", ".jpeg") else "png"
            content_parts.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/{mime};base64,{img_b64}"},
            })
        except Exception as e:
            log.warning("vlm_qa: failed to read image %s: %s", abs_path, e)

    # Build text prompt with memory context + user question
    text_prompt = _VLM_QA_PROMPT
    if node_contexts:
        text_prompt += "\n\nMemory context from the knowledge graph:\n"
        for i, ctx in enumerate(node_contexts, 1):
            text_prompt += f"\n  [{i}] {ctx}"
    text_prompt += f"\n\nUser question: {query}"

    # If no images, still try to answer from memory context alone
    if not content_parts:
        if not node_contexts:
            log.debug("vlm_qa: no images and no context")
            return None
        # VLM text-only: answer from memory context
        content_parts.append({"type": "text", "text": text_prompt})
    else:
        content_parts.append({"type": "text", "text": text_prompt})

    url = cfg["base_url"].rstrip("/") + "/chat/completions"
    headers = {
        "Authorization": f"Bearer {cfg['api_key']}",
        "Content-Type": "application/json",
    }
    body = {
        "model": cfg["model"],
        "messages": [{"role": "user", "content": content_parts}],
        "max_tokens": 256,
        "temperature": 0.0,
    }

    try:
        async with httpx.AsyncClient(timeout=30.0) as client:
            r = await client.post(url, json=body, headers=headers)
            if r.status_code >= 400:
                log.warning("vlm_qa: VLM returned %d: %s", r.status_code, r.text[:200])
                return None
            data = r.json()
    except Exception as e:
        log.warning("vlm_qa: VLM call failed: %s: %s", type(e).__name__, e)
        return None

    try:
        answer = data["choices"][0]["message"]["content"].strip()
        log.info("vlm_qa: answer → %s", answer[:300])
        return answer
    except (KeyError, IndexError, TypeError):
        log.warning("vlm_qa: unexpected VLM response shape")
        return None


# ── Summary fallback ───────────────────────────────────────────────────────

def _template_summary(objects: List[ObjectCoord], scene_type: str) -> str:
    """Template-based summary when VLM is unavailable."""
    labels = [o.label for o in objects if o.label]
    obj_str = ", ".join(labels) if labels else "surroundings"
    loc = scene_type.replace("_", " ") if scene_type else "current area"
    return f"observed {obj_str} in the {loc}"


