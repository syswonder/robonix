"""Tests for VectorStore — embedding, BM25, hybrid search, multimodal interface.

Runs: test_embedding.py + test_bm25.py + test_vector_store.py + test_multimodal_interface.py
(all in one file for simplicity).
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.storage.embedding_config import EmbeddingModelConfig, _hash_embedding
from memory_service.storage.vector_store import TextEmbedder, BM25Index, VectorStore


# ── Embedding tests ─────────────────────────────────────────────────────

class TestEmbeddingConfig:
    def test_defaults(self):
        cfg = EmbeddingModelConfig()
        assert cfg.model_name == "all-MiniLM-L6-v2"
        assert cfg.dim == 384
        assert cfg.device == "cpu"
        # path should be expanded
        assert "~" not in cfg.model_path

    def test_env_override(self):
        import os as _os
        _os.environ["EMBEDDING_MODEL_PATH"] = "/tmp/test_model"
        cfg = EmbeddingModelConfig()
        assert cfg.model_path == "/tmp/test_model"
        del _os.environ["EMBEDDING_MODEL_PATH"]


class TestHashEmbedding:
    def test_deterministic(self):
        v1 = _hash_embedding("hello world")
        v2 = _hash_embedding("hello world")
        assert v1 == v2

    def test_different_texts_different_vectors(self):
        v1 = _hash_embedding("hello")
        v2 = _hash_embedding("world")
        assert v1 != v2

    def test_dimension(self):
        v = _hash_embedding("test", dim=384)
        assert len(v) == 384

    def test_unit_norm(self):
        v = _hash_embedding("some text")
        norm = sum(x * x for x in v) ** 0.5
        assert abs(norm - 1.0) < 1e-6


class TestTextEmbedder:
    def test_hash_fallback_dim(self):
        # sentence-transformers not installed → hash fallback
        cfg = EmbeddingModelConfig()
        emb = TextEmbedder(cfg)
        v = emb.encode("test sentence")
        assert len(v) == 384

    def test_encode_batch(self):
        cfg = EmbeddingModelConfig()
        emb = TextEmbedder(cfg)
        vecs = emb.encode_batch(["a", "b", "c"])
        assert len(vecs) == 3
        assert all(len(v) == 384 for v in vecs)


# ── BM25 tests ──────────────────────────────────────────────────────────

class TestBM25Index:
    def setup_method(self):
        self.bm25 = BM25Index()

    def test_insert_and_search(self):
        self.bm25.insert(1, "robot grasped the red cup in kitchen")
        self.bm25.insert(2, "robot placed the blue cup in living room")
        results = self.bm25.search("grasp red cup", {1, 2})
        assert len(results) == 2
        # node 1 should score higher for "grasp"
        assert results[0][0] == 1

    def test_keyword_match_boosts_score(self):
        self.bm25.insert(1, "navigate to kitchen and grasp cup")
        self.bm25.insert(2, "observe the scenery")
        results = self.bm25.search("grasp", {1, 2})
        assert results[0][0] == 1
        assert results[0][1] > results[1][1]

    def test_candidate_set_filter(self):
        self.bm25.insert(1, "grasp cup in kitchen")
        self.bm25.insert(2, "grasp cup in lab")
        self.bm25.insert(3, "observe landscape")
        # Only consider {1, 3}
        results = self.bm25.search("grasp cup", {1, 3})
        nids = [r[0] for r in results]
        assert 1 in nids
        assert 2 not in nids

    def test_remove(self):
        self.bm25.insert(1, "grasp red cup")
        self.bm25.insert(2, "grasp blue cup")
        self.bm25.remove(1)
        results = self.bm25.search("grasp cup", {1, 2})
        # Node 1 should have score 0 or be excluded
        scores = {r[0]: r[1] for r in results}
        assert scores.get(1, 0.0) == 0.0

    def test_rebuild(self):
        self.bm25.insert(1, "old data")
        self.bm25.rebuild([(2, "grasp cup"), (3, "place block")])
        results = self.bm25.search("grasp", {1, 2, 3})
        nids = {r[0] for r in results if r[1] > 0}
        assert 1 not in nids  # old data gone
        assert 2 in nids


# ── VectorStore hybrid search tests ──────────────────────────────────────

class TestVectorStore:
    def setup_method(self):
        cfg = EmbeddingModelConfig(dim=8)  # small dim for fast tests
        # Override dim for hash fallback
        self.vs = VectorStore(config=cfg, alpha=0.3)

    def _make_emb(self, text: str) -> list:
        return _hash_embedding(text, dim=8)

    def test_insert_and_count(self):
        emb = self._make_emb("test")
        self.vs.insert(1, emb, "test summary")
        assert self.vs.count() == 1

    def test_search_returns_top_k(self):
        # Insert nodes with semantically distinct summaries
        for i, summary in enumerate([
            "grasp red cup in kitchen",
            "place blue cup on table",
            "navigate to living room",
            "grasp green cup in kitchen",
        ]):
            emb = self._make_emb(summary)
            self.vs.insert(i, emb, summary)

        results = self.vs.search("grasp cup in kitchen", {0, 1, 2, 3}, top_k=2)
        assert len(results) == 2
        # Results are (node_id, hybrid_score) descending
        assert results[0][1] >= results[1][1]

    def test_candidate_set_restricts_results(self):
        self.vs.insert(1, self._make_emb("grasp cup kitchen"), "grasp cup kitchen")
        self.vs.insert(2, self._make_emb("grasp cup kitchen"), "grasp cup kitchen")
        self.vs.insert(3, self._make_emb("grasp cup kitchen"), "grasp cup kitchen")
        results = self.vs.search("grasp cup", {1, 2}, top_k=5)
        nids = {r[0] for r in results}
        assert 3 not in nids

    def test_empty_candidates_returns_empty(self):
        self.vs.insert(1, self._make_emb("test"), "test")
        results = self.vs.search("query", set(), top_k=5)
        assert results == []

    def test_pure_bm25_mode(self):
        self.vs.insert(1, self._make_emb("grasp cup kitchen"), "grasp cup kitchen")
        self.vs.insert(2, self._make_emb("observe landscape"), "observe landscape")
        results = self.vs.search("grasp", {1, 2}, top_k=2, alpha=1.0)
        assert len(results) == 2
        assert results[0][0] == 1  # BM25 should match "grasp"

    def test_pure_cosine_mode(self):
        v1 = self._make_emb("grasp cup kitchen")
        v2 = self._make_emb("observe landscape")
        self.vs.insert(1, v1, "grasp cup kitchen")
        self.vs.insert(2, v2, "observe landscape")
        results = self.vs.search("grasp cup", {1, 2}, top_k=2, alpha=0.0)
        assert len(results) == 2
        # With hash embeddings, same input gives same vector → perfect match
        assert results[0][0] == 1

    def test_remove(self):
        self.vs.insert(1, self._make_emb("test"), "test")
        self.vs.remove(1)
        assert self.vs.count() == 0
        results = self.vs.search("test", {1}, top_k=5)
        assert results == [] or all(r[1] == 0.0 for r in results)


# ── Multimodal interface tests ──────────────────────────────────────────

class TestMultimodalInterface:
    def setup_method(self):
        self.vs = VectorStore(alpha=0.3)

    def test_text_modality_works(self):
        v = self.vs.encode("hello", modality="text")
        assert len(v) == 384

    def test_visual_modality_raises_not_implemented(self):
        try:
            self.vs.encode("image.png", modality="visual")
            assert False, "should have raised NotImplementedError"
        except NotImplementedError as e:
            assert "not yet implemented" in str(e)

    def test_fused_modality_raises_not_implemented(self):
        try:
            self.vs.encode("anything", modality="fused")
            assert False, "should have raised NotImplementedError"
        except NotImplementedError as e:
            assert "not yet implemented" in str(e)

    def test_unknown_modality_raises_value_error(self):
        try:
            self.vs.encode("text", modality="audio")
            assert False, "should have raised ValueError"
        except ValueError as e:
            assert "Unknown modality" in str(e)

    def test_get_dim_text(self):
        assert self.vs.get_dim("text") == 384

    def test_get_dim_unknown(self):
        try:
            self.vs.get_dim("audio")
            assert False
        except ValueError:
            pass


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import traceback
    tests = [
        TestEmbeddingConfig(), TestHashEmbedding(), TestTextEmbedder(),
        TestBM25Index(), TestVectorStore(), TestMultimodalInterface(),
    ]
    passed = failed = 0
    for obj in tests:
        cls_name = type(obj).__name__
        for name in dir(obj):
            if name.startswith("test_"):
                try:
                    if hasattr(obj, "setup_method"):
                        obj.setup_method()
                    getattr(obj, name)()
                    print(f"  PASS {cls_name}.{name}")
                    passed += 1
                except Exception:
                    print(f"  FAIL {cls_name}.{name}")
                    traceback.print_exc()
                    failed += 1
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
