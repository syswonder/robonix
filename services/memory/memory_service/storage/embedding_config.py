"""Embedding model configuration and fallback strategy.

Phase1: all-MiniLM-L6-v2 at local path with deterministic-hash fallback
when sentence-transformers or the model is unavailable.

Multimodal interface reserved: modality parameter controls which encoder to use.
Current: "text" only.

Model path precedence:
  1. EMBEDDING_MODEL_PATH env var
  2. sentence-transformers' standard cache for `model_name`
"""

from __future__ import annotations

import hashlib
import os
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class EmbeddingModelConfig:
    """Configuration for the text embedding model. (§2.7.1)"""

    model_name: str = "all-MiniLM-L6-v2"
    model_path: str = ""              # resolved at __post_init__
    dim: int = 384
    batch_size: int = 32
    device: str = "cpu"
    max_seq_length: int = 256

    def __post_init__(self):
        if not self.model_path:
            self.model_path = os.environ.get("EMBEDDING_MODEL_PATH", "")
        self.model_path = os.path.expanduser(self.model_path)


def _hash_embedding(text: str, dim: int = 384) -> List[float]:
    """Deterministic pseudo-embedding from text hash.

    Produces a unit-norm vector of `dim` floats from the SHA-256 of `text`.
    Used as a fallback when the real embedding model is unavailable.

    This is NOT semantically meaningful — it only guarantees:
      - Same text → same vector (deterministic)
      - Different text → (virtually always) different vector
    """
    h = hashlib.sha256(text.encode("utf-8")).digest()
    vec: List[float] = []
    for i in range(dim):
        # Map 4 hash bytes to a float in [-1, 1] via unsigned int division.
        # struct.unpack('>f', ...) can produce NaN/Inf from arbitrary bytes;
        # unsigned-int scaling is always finite and deterministic.
        byte_idx = (i * 4) % len(h)
        u = struct.unpack(">I", h[byte_idx:byte_idx + 4])[0]
        vec.append((u / 0xFFFFFFFF) * 2.0 - 1.0)
    # Normalize to unit vector
    norm = sum(v * v for v in vec) ** 0.5
    if norm > 1e-12:
        vec = [v / norm for v in vec]
    return vec
