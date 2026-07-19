"""Tests for LLM-based memory search — prompt formatting, config, fallback paths."""

import os
import sys
import tempfile
import unittest
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_SVC = _HERE.parent
if str(_SVC) not in sys.path:
    sys.path.insert(0, str(_SVC))

from memory_service.core import llm_search
from memory_service.core.types import MemoryNode, TagSet


class TestFormatNode(unittest.TestCase):
    """Prompt formatting tests — no LLM needed."""

    def test_format_node_full(self):
        node = MemoryNode(
            node_id=42,
            summary="successfully grasp red cup in kitchen",
            tags=TagSet(
                scene_type="kitchen", action_type="grasp",
                task_type="fetch", success=True,
                objects_present=["red cup", "counter"],
                difficulty="easy",
            ),
            timestamp=1_765_432_100_123_456_789,
            weight=0.75,
        )
        line = llm_search._format_node(node)
        assert "Node 42" in line
        assert '"successfully grasp red cup in kitchen"' in line
        assert "kitchen" in line
        assert "grasp" in line
        assert "fetch" in line
        assert "True" in line
        assert "easy" in line
        assert "red cup" in line
        assert "weight: 0.75" in line

    def test_format_node_minimal(self):
        node = MemoryNode(node_id=0, summary="bare node")
        line = llm_search._format_node(node)
        assert "Node 0" in line
        assert '"bare node"' in line

    def test_build_prompt_structure(self):
        nodes = [
            MemoryNode(node_id=1, summary="node one",
                       tags=TagSet(scene_type="kitchen")),
            MemoryNode(node_id=2, summary="node two",
                       tags=TagSet(action_type="grasp")),
        ]
        prompt = llm_search._build_prompt("test query", nodes)
        assert "test query" in prompt
        assert "2 nodes" in prompt
        assert "node one" in prompt
        assert "node two" in prompt
        assert '"nodes"' in prompt  # JSON output instruction


class TestLLMConfig(unittest.TestCase):
    """Env-var config resolution — no LLM needed."""

    def _clear_env(self):
        for k in ("MEMGRAPH_LLM_BASE_URL", "MEMGRAPH_LLM_API_KEY",
                  "MEMGRAPH_LLM_MODEL",
                  "VLM_BASE_URL", "VLM_API_KEY", "VLM_MODEL",
                  "OPENAI_BASE_URL", "OPENAI_API_KEY", "OPENAI_MODEL"):
            os.environ.pop(k, None)

    def test_no_creds_returns_false(self):
        self._clear_env()
        assert llm_search.llm_search_available() is False

    def test_api_key_without_url_returns_false(self):
        self._clear_env()
        os.environ["MEMGRAPH_LLM_API_KEY"] = "sk-test"
        assert llm_search.llm_search_available() is False

    def test_url_without_key_returns_false(self):
        self._clear_env()
        os.environ["MEMGRAPH_LLM_BASE_URL"] = "https://api.openai.com/v1"
        assert llm_search.llm_search_available() is False

    def test_both_set_returns_true(self):
        self._clear_env()
        os.environ["MEMGRAPH_LLM_BASE_URL"] = "https://api.openai.com/v1"
        os.environ["MEMGRAPH_LLM_API_KEY"] = "sk-test"
        assert llm_search.llm_search_available() is True

    def test_vlm_fallback(self):
        self._clear_env()
        os.environ["VLM_BASE_URL"] = "https://vlm.example.com/v1"
        os.environ["VLM_API_KEY"] = "sk-vlm"
        assert llm_search.llm_search_available() is True

    def test_openai_fallback(self):
        self._clear_env()
        os.environ["OPENAI_BASE_URL"] = "https://api.openai.com/v1"
        os.environ["OPENAI_API_KEY"] = "sk-openai"
        assert llm_search.llm_search_available() is True

    def test_model_fallback_chain(self):
        self._clear_env()
        # Only set VLM_MODEL
        os.environ["VLM_BASE_URL"] = "https://x/v1"
        os.environ["VLM_API_KEY"] = "sk"
        os.environ["VLM_MODEL"] = "gpt-5.5"
        cfg = llm_search._llm_config()
        assert cfg["model"] == "gpt-5.5"

    def test_model_default(self):
        self._clear_env()
        os.environ["VLM_BASE_URL"] = "https://x/v1"
        os.environ["VLM_API_KEY"] = "sk"
        cfg = llm_search._llm_config()
        assert cfg["model"] == "gpt-4.1"


class TestLLMRankFallback(unittest.TestCase):
    """Fallback when no LLM is configured — chronological ranking."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        # Small in-memory graph for testing
        self._nodes: dict = {}

    def _get_node(self, nid):
        return self._nodes.get(nid)

    def _add(self, nid, summary, ts):
        self._nodes[nid] = MemoryNode(node_id=nid, summary=summary, timestamp=ts)

    def test_no_llm_falls_back_to_chronological(self):
        # Ensure no LLM creds
        for k in ("MEMGRAPH_LLM_BASE_URL", "MEMGRAPH_LLM_API_KEY",
                  "VLM_BASE_URL", "VLM_API_KEY",
                  "OPENAI_BASE_URL", "OPENAI_API_KEY"):
            os.environ.pop(k, None)
        assert llm_search.llm_search_available() is False

        self._add(10, "oldest", 1000)
        self._add(20, "middle", 2000)
        self._add(30, "newest", 3000)

        import asyncio
        ranked = asyncio.run(llm_search.llm_rank(
            query="anything",
            candidate_ids={10, 20, 30},
            graph_get=self._get_node,
            top_k=3,
        ))
        assert len(ranked) == 3
        # Most recent first
        assert ranked[0][0] == 30, f"expected newest (30), got {ranked[0]}"
        assert ranked[1][0] == 20
        assert ranked[2][0] == 10
        # Scores are descending
        assert ranked[0][1] > ranked[1][1] > ranked[2][1]

    def test_single_node_ranked_first(self):
        for k in ("MEMGRAPH_LLM_BASE_URL", "MEMGRAPH_LLM_API_KEY",
                  "VLM_BASE_URL", "VLM_API_KEY",
                  "OPENAI_BASE_URL", "OPENAI_API_KEY"):
            os.environ.pop(k, None)

        self._add(7, "only", 5000)

        import asyncio
        ranked = asyncio.run(llm_search.llm_rank(
            query="x",
            candidate_ids={7},
            graph_get=self._get_node,
        ))
        assert len(ranked) == 1
        assert ranked[0] == (7, 1.0)


if __name__ == "__main__":
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(__import__(__name__))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
