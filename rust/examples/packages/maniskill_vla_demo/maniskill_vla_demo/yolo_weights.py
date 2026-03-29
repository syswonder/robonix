"""YOLO-World checkpoint names for ultralytics (auto-download on first ``YOLO(...)``)."""

from __future__ import annotations

import sys

# Official YOLO-World v2 weights (see ultralytics assets). There is no nano ("n") v2 build;
# "s" (small) is the lightest published variant and matches typical "demo GPU" expectations.
DEFAULT_YOLO_WORLD_V2 = "yolov8s-worldv2.pt"

_LEGACY_NANO = frozenset({"yolov8n-worldv2.pt", "yolov8n-worldv2"})


def resolve_yolo_world_weights(spec: str | None) -> str:
    """Return a weight string suitable for ``ultralytics.YOLO``.

    Maps the non-existent ``yolov8n-worldv2`` name to :data:`DEFAULT_YOLO_WORLD_V2`.
    Local paths, ``http(s)`` URLs, and other hub names are passed through unchanged.
    """
    s = (spec or DEFAULT_YOLO_WORLD_V2).strip() or DEFAULT_YOLO_WORLD_V2
    if s in _LEGACY_NANO:
        print(
            "[yolo] yolov8n-worldv2 is not published by Ultralytics; "
            f"using {DEFAULT_YOLO_WORLD_V2} (cached download on first run)",
            file=sys.stderr,
        )
        return DEFAULT_YOLO_WORLD_V2
    return s
