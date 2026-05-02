"""Default YOLOE checkpoint names/URLs for ultralytics."""

from __future__ import annotations

DEFAULT_YOLOE = str(
    (__import__("pathlib").Path(__file__).resolve().parent.parent / "yoloe-11l-seg.pt")
)


def resolve_open_vocab_weights(spec: str | None) -> str:
    """Return a weight string suitable for ``ultralytics.YOLOE``."""
    s = (spec or DEFAULT_YOLOE).strip() or DEFAULT_YOLOE
    # Convert pasted Hugging Face UI links to raw artifact links.
    return s.replace("/blob/", "/resolve/")


def resolve_yolo_world_weights(spec: str | None) -> str:
    """Backward-compatible alias; runtime now defaults to YOLOE weights."""
    return resolve_open_vocab_weights(spec)
