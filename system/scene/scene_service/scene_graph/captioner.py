# SPDX-License-Identifier: MulanPSL-2.0
"""Node captioner — pluggable interface for generating captions.

V1 implementation: caption = label (no VLM call, no crops).
Future versions will accept crop images and call a VLM to produce
richer captions like "a black office chair with wheels near a desk".
"""
from __future__ import annotations

import logging
import time

from .types import SceneGraphNode

log = logging.getLogger(__name__)


class NodeCaptioner:
    """Generate a natural-language caption for a SceneGraphNode.

    The interface is a single async method ``caption_node`` so that
    future implementations can do I/O (VLM calls, crop reads) without
    blocking the event loop.

    V1 simply copies ``node.label`` into ``node.caption``.
    """

    async def caption_node(self, node: SceneGraphNode) -> SceneGraphNode:
        """Set ``node.caption`` and return the mutated node.

        Override this method to plug in a VLM-based captioner.
        """
        node.caption = node.label
        node.caption_updated_at = time.time()
        return node
