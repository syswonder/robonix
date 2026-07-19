"""Image Store — local image persistence under data/images/{node_id}/.

Each MemoryNode's observation frames are stored as PNG files in a
per-node subdirectory.  Paths returned are relative to the service
package root (services/memory/).
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import List

log = logging.getLogger("scribe_mem")

_DEFAULT_IMAGE_ROOT = str(
    Path(__file__).resolve().parent.parent.parent / "data" / "images"
)


class ImageStore:
    """Local file-system store for observation images.

    Layout:
        {image_root}/
          {node_id}/
            frame_0001.png
            frame_0002.png
            ...
    """

    def __init__(self, image_root: str = ""):
        self._root = Path(image_root or _DEFAULT_IMAGE_ROOT)

    @property
    def root(self) -> str:
        return str(self._root)

    # ── Write ──────────────────────────────────────────────────────────

    def save(self, node_id: int, image_bytes: bytes) -> str:
        """Save one image frame for a node.  Returns the relative path."""
        node_dir = self._node_dir(node_id)
        node_dir.mkdir(parents=True, exist_ok=True)

        # Accept both .jpg and .png for backward compat
        existing = (sorted(node_dir.glob("frame_*.jpg"))
                    + sorted(node_dir.glob("frame_*.png")))
        seq = len(existing) + 1
        filename = f"frame_{seq:04d}.jpg"
        filepath = node_dir / filename

        with open(filepath, "wb") as f:
            f.write(image_bytes)

        rel = str(filepath.relative_to(self._root.parent.parent))
        log.info("image_store: wrote %s (%.1f KB)", rel, len(image_bytes) / 1024)
        return rel

    def save_batch(self, node_id: int, images: List[bytes]) -> List[str]:
        """Save multiple frames for one node."""
        paths: List[str] = []
        for img in images:
            paths.append(self.save(node_id, img))
        return paths

    # ── Read ───────────────────────────────────────────────────────────

    def list(self, node_id: int) -> List[str]:
        """Return relative paths of all images for a node, sorted by name."""
        node_dir = self._node_dir(node_id)
        if not node_dir.exists():
            return []
        files = sorted(node_dir.glob("frame_*.jpg")) + sorted(node_dir.glob("frame_*.png"))
        return [str(f.relative_to(self._root.parent.parent)) for f in files]

    def get_node_dir(self, node_id: int) -> str:
        """Return the absolute path to a node's image directory."""
        return str(self._node_dir(node_id))

    def count(self, node_id: int) -> int:
        """Number of images stored for a node."""
        return len(self.list(node_id))

    def remove(self, node_id: int) -> bool:
        """Delete all images for a node.  Returns True if anything was deleted."""
        node_dir = self._node_dir(node_id)
        if not node_dir.exists():
            return False
        import shutil
        shutil.rmtree(str(node_dir), ignore_errors=True)
        log.debug("image_store: removed images for node %d", node_id)
        return True

    # ── Internal ──────────────────────────────────────────────────────

    def _node_dir(self, node_id: int) -> Path:
        return self._root / str(node_id)
