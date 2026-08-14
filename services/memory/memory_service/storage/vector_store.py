"""Vector Store — BM25 + Embedding hybrid retrieval.

Phase1: dict-based storage + numpy cosine similarity + BM25 sparse index.
Embedding: all-MiniLM-L6-v2 d=384 (with hash-based fallback).
Multimodal interface reserved via `modality` parameter.

Hybrid score:
    HybridScore = α × BM25_norm(query, summary) + (1-α) × Cosine(q_emb, n_emb)
    Default α = 0.3 (semantic-leaning, keyword-recall-fallback)
"""

from __future__ import annotations

import logging
import math
import os
from collections import defaultdict
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

from .embedding_config import EmbeddingModelConfig, _hash_embedding

log = logging.getLogger("scribe_mem")


# ── Text Embedder ──────────────────────────────────────────────────────

class TextEmbedder:
    """Text → vector encoder.

    Phase1: tries sentence-transformers, falls back to deterministic hash.
    """

    def __init__(self, config: EmbeddingModelConfig):
        self._config = config
        self._model: Any = None
        self._initialized = False
        self._try_load_model()

    def _try_load_model(self) -> None:
        """Attempt to load all-MiniLM-L6-v2 from the local path or via package."""
        model_path = self._config.model_path
        try:
            from sentence_transformers import SentenceTransformer
            if model_path and os.path.isdir(model_path):
                self._model = SentenceTransformer(model_path, device=self._config.device)
                log.info("TextEmbedder: loaded embedding model from %s", model_path)
                self._initialized = True
            else:
                if model_path:
                    log.warning(
                        "TextEmbedder: configured model path not found (%s); "
                        "trying model name %s through the standard cache",
                        model_path,
                        self._config.model_name,
                    )
                self._model = SentenceTransformer(
                    self._config.model_name,
                    device=self._config.device,
                )
                log.info(
                    "TextEmbedder: loaded %s through the standard cache",
                    self._config.model_name,
                )
                self._initialized = True
        except ImportError:
            log.warning(
                "TextEmbedder: sentence-transformers not installed, "
                "falling back to deterministic hash embedding (non-semantic)"
            )
        except Exception as e:
            log.warning(
                "TextEmbedder: failed to load model (%s), "
                "falling back to deterministic hash embedding", e
            )

    @property
    def dim(self) -> int:
        return self._config.dim

    @property
    def is_semantic(self) -> bool:
        """True if using a real embedding model (vs. hash fallback)."""
        return self._initialized

    def encode(self, text: str) -> List[float]:
        """Encode a single text to a vector."""
        if self._initialized and self._model is not None:
            result = self._model.encode([text], batch_size=1, show_progress_bar=False)
            # result is a numpy array of shape (1, dim)
            return result[0].tolist()
        return _hash_embedding(text, dim=self._config.dim)

    def encode_batch(self, texts: List[str]) -> List[List[float]]:
        """Encode multiple texts."""
        if self._initialized and self._model is not None:
            results = self._model.encode(texts, batch_size=self._config.batch_size,
                                         show_progress_bar=False)
            return [r.tolist() for r in results]
        return [_hash_embedding(t, dim=self._config.dim) for t in texts]


# ── BM25 Index ─────────────────────────────────────────────────────────

class BM25Index:
    """Sparse BM25 inverted index for keyword-based scoring.

    Standard BM25 formula:
        score(D, Q) = Σ IDF(qi) × (tf(qi,D) × (k1+1)) / (tf(qi,D) + k1×(1-b+b×|D|/avgdl))

    Phase1: simple tokenization (whitespace + lowercase + punctuation strip).
    """

    def __init__(self, k1: float = 1.5, b: float = 0.75):
        self.k1 = k1
        self.b = b

        # node_id -> [token, ...]
        self._docs: Dict[int, List[str]] = {}
        # token -> {node_id: term_frequency}
        self._inverted: Dict[str, Dict[int, int]] = defaultdict(lambda: defaultdict(int))
        # token -> document frequency
        self._doc_freq: Dict[str, int] = defaultdict(int)
        # node_id -> doc length
        self._doc_len: Dict[int, int] = {}
        self._total_docs: int = 0
        self._avg_doc_len: float = 0.0

    def insert(self, node_id: int, text: str) -> None:
        """Index a document (summary text) for a node."""
        tokens = self._tokenize(text)
        if not tokens:
            return

        # Remove old entry if present
        self.remove(node_id)

        self._docs[node_id] = tokens
        self._doc_len[node_id] = len(tokens)
        self._total_docs += 1
        self._avg_doc_len = (sum(self._doc_len.values()) / self._total_docs
                             if self._total_docs else 0)

        token_counts: Dict[str, int] = defaultdict(int)
        for tok in tokens:
            token_counts[tok] += 1

        for tok, tf in token_counts.items():
            self._inverted[tok][node_id] = tf
            self._doc_freq[tok] += 1

    def remove(self, node_id: int) -> None:
        """Remove a node's document from the index."""
        if node_id not in self._docs:
            return

        tokens = self._docs.pop(node_id)
        old_len = self._doc_len.pop(node_id, 0)
        self._total_docs -= 1
        self._avg_doc_len = (sum(self._doc_len.values()) / self._total_docs
                             if self._total_docs else 0)

        # Decrement doc_freq for each unique token
        seen: Set[str] = set()
        for tok in tokens:
            if tok in seen:
                continue
            seen.add(tok)
            self._inverted[tok].pop(node_id, None)
            self._doc_freq[tok] = max(0, self._doc_freq[tok] - 1)

    def search(self, query: str, candidate_ids: Set[int]) -> List[Tuple[int, float]]:
        """Compute BM25 scores for candidate nodes.

        Returns list of (node_id, bm25_score) sorted descending.
        """
        if not candidate_ids or self._total_docs == 0:
            return []

        query_tokens = self._tokenize(query)
        if not query_tokens:
            return [(nid, 0.0) for nid in candidate_ids]

        scores: Dict[int, float] = defaultdict(float)
        avgdl = max(self._avg_doc_len, 1e-6)
        N = self._total_docs

        for qt in query_tokens:
            posting = self._inverted.get(qt, {})
            df = self._doc_freq.get(qt, 0)
            if df == 0:
                continue
            idf = math.log(1.0 + (N - df + 0.5) / (df + 0.5))

            for nid, tf in posting.items():
                if nid not in candidate_ids:
                    continue
                dl = self._doc_len.get(nid, 1)
                numerator = tf * (self.k1 + 1)
                denominator = tf + self.k1 * (1 - self.b + self.b * dl / avgdl)
                scores[nid] += idf * numerator / denominator

        result = [(nid, scores.get(nid, 0.0)) for nid in candidate_ids]
        result.sort(key=lambda x: x[1], reverse=True)
        return result

    def rebuild(self, nodes: List[Tuple[int, str]]) -> None:
        """Full rebuild from a list of (node_id, text) pairs."""
        self._docs.clear()
        self._inverted.clear()
        self._doc_freq.clear()
        self._doc_len.clear()
        self._total_docs = 0
        self._avg_doc_len = 0.0
        for nid, text in nodes:
            self.insert(nid, text)

    def _tokenize(self, text: str) -> List[str]:
        """Simple tokenization: lowercase, strip punctuation, split whitespace."""
        # Remove common punctuation
        for ch in ".,!?;:()[]{}\"'":
            text = text.replace(ch, " ")
        return [t for t in text.lower().split() if len(t) >= 2]


# ── Vector Store ───────────────────────────────────────────────────────

class VectorStore:
    """Hybrid retrieval: BM25 + Cosine similarity.

    Insert indexes both the embedding vector and the BM25 document.
    Search performs weighted fusion on a candidate set.

    When ``embedding_enabled=False`` (default), the TextEmbedder is
    never loaded — ``encode()`` returns an empty list, ``insert()``
    only indexes BM25, and ``is_semantic`` is ``False``.  The
    retrieve pipeline then falls through to LLM ranking (Path A) or
    chronological fallback (Path C).
    """

    def __init__(self, config: Optional[EmbeddingModelConfig] = None, alpha: float = 0.3,
                 *, embedding_enabled: bool = True):
        self._config = config or EmbeddingModelConfig()
        self._alpha = alpha
        self._embedding_enabled = embedding_enabled
        self._embedder: Optional[TextEmbedder] = None
        if embedding_enabled:
            self._embedder = TextEmbedder(self._config)
        self._bm25 = BM25Index()

        # node_id -> embedding vector
        self._vectors: Dict[int, np.ndarray] = {}

    @property
    def alpha(self) -> float:
        return self._alpha

    @property
    def dim(self) -> int:
        return self._embedder.dim if self._embedder is not None else 0

    @property
    def is_semantic(self) -> bool:
        return self._embedder is not None and self._embedder.is_semantic

    # ── Encoding (modality-reserved interface) ────────────────────────

    def encode(self, text: str, modality: str = "text") -> List[float]:
        """Encode input to vector. Returns empty list when embedding disabled."""
        if self._embedder is None:
            return []
        if modality == "text":
            return self._embedder.encode(text)
        if modality == "visual":
            return self._encode_visual(text)
        if modality == "fused":
            return self._encode_fused(text)
        raise ValueError(f"Unknown modality: {modality}")

    def encode_batch(self, texts: List[str], modality: str = "text") -> List[List[float]]:
        if self._embedder is None:
            return [[] for _ in texts]
        if modality == "text":
            return self._embedder.encode_batch(texts)
        raise NotImplementedError(f"Batch encode not implemented for modality={modality}")

    # ── Multimodal placeholders ───────────────────────────────────────

    def _encode_visual(self, image_path_or_desc: str) -> List[float]:
        """Visual encoder — reserved for Phase2+."""
        raise NotImplementedError(
            "Visual embedding not yet implemented. "
            "Planned: CLIP/ViT visual encoder → d_vis=512"
        )

    def _encode_fused(self, modalities_input: Any) -> List[float]:
        """Multi-modal fusion encoder — reserved for Phase2+."""
        raise NotImplementedError(
            "Multi-modal fusion embedding not yet implemented. "
            "Planned: Concat + Linear(d_text+d_vis+d_act → d_fused=768)"
        )

    def get_dim(self, modality: str = "text") -> int:
        if self._embedder is None:
            return 0
        if modality == "text":
            return self._embedder.dim
        raise ValueError(f"Unknown modality: {modality}")

    # ── Storage ───────────────────────────────────────────────────────

    def insert(self, node_id: int, embedding: List[float], summary: str) -> None:
        """Insert a vector and BM25-index its summary.

        When embedding is an empty list (embedding disabled), only BM25 is
        indexed — the dense-vector store is left untouched.
        """
        if embedding:
            self._vectors[node_id] = np.array(embedding, dtype=np.float32)
        if summary:
            self._bm25.insert(node_id, summary)

    def remove(self, node_id: int) -> None:
        """Remove a node from both indices."""
        self._vectors.pop(node_id, None)
        self._bm25.remove(node_id)

    def count(self) -> int:
        return len(self._vectors)

    # ── Retrieval ─────────────────────────────────────────────────────

    def search(
        self,
        query: str,
        candidate_ids: Set[int],
        top_k: int = 5,
        alpha: Optional[float] = None,
    ) -> List[Tuple[int, float]]:
        """BM25 + Cosine hybrid search on a candidate set.

        Args:
            query: Natural language query string.
            candidate_ids: Node IDs to consider (from TagIndex pre-filter).
            top_k: Number of results to return.
            alpha: BM25 weight (None → use default 0.3).
                   α=1.0 → pure BM25, α=0.0 → pure Cosine.

        Returns:
            List of (node_id, hybrid_score) sorted descending.
        """
        if not candidate_ids:
            return []

        if alpha is None:
            alpha = self._alpha

        # 1. Query embedding
        query_vec = np.array(self.encode(query), dtype=np.float32)

        # 2. BM25 scores (sparse)
        bm25_results = self._bm25.search(query, candidate_ids)
        bm25_scores: Dict[int, float] = dict(bm25_results)
        bm25_max = max(bm25_scores.values()) if bm25_scores else 1.0

        # 3. Cosine scores (dense)
        cosine_scores: Dict[int, float] = {}
        for nid in candidate_ids:
            vec = self._vectors.get(nid)
            if vec is None or vec.shape != query_vec.shape:
                continue
            # Cosine similarity
            dot = np.dot(query_vec, vec)
            q_norm = np.linalg.norm(query_vec)
            v_norm = np.linalg.norm(vec)
            if q_norm < 1e-12 or v_norm < 1e-12:
                cosine_scores[nid] = 0.0
            else:
                cosine_scores[nid] = float(dot / (q_norm * v_norm))

        # Handle case where no vectors match
        if not cosine_scores:
            # Return BM25-only results
            return [(nid, s) for nid, s in bm25_results[:top_k]]

        cosine_max = max(cosine_scores.values()) if cosine_scores else 1.0

        # 4. Hybrid fusion
        hybrid: Dict[int, float] = {}
        all_nids = set(bm25_scores.keys()) | set(cosine_scores.keys())
        for nid in all_nids:
            bm25_norm = (bm25_scores.get(nid, 0.0) / max(bm25_max, 1e-12))
            cos_norm = (cosine_scores.get(nid, 0.0) / max(cosine_max, 1e-12))
            hybrid[nid] = alpha * bm25_norm + (1.0 - alpha) * cos_norm

        # 5. Sort and return top-k
        sorted_items = sorted(hybrid.items(), key=lambda x: x[1], reverse=True)
        return sorted_items[:top_k]

    # ── Rebuild from existing nodes ──────────────────────────────────

    def rebuild(self, nodes: List[Tuple[int, List[float], str]]) -> None:
        """Full rebuild from (node_id, embedding, summary) triples."""
        self._vectors.clear()
        for nid, emb, summary in nodes:
            if emb:
                self._vectors[nid] = np.array(emb, dtype=np.float32)
        self._bm25.rebuild([(nid, summary) for nid, _, summary in nodes])
