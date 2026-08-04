# memgraph — Scribe Mem structured memory service

Memgraph is Robonix's **structured CKG memory system** (formerly Scribe Mem):
it consumes `LogRecord` entries from Scribe Log and builds a causal knowledge
graph (CKG) with tag-indexed, vector-searchable memory nodes.

Runs in parallel with **memsearch** under the shared `robonix/service/memory/`
namespace — Pilot discovers both backends and the LLM chooses between them.

## Capability surface

| Contract | Behaviour |
|---|---|
| `robonix/service/memory/remember`       | Write a MemoryNode (tag → vector → graph) |
| `robonix/service/memory/hybrid_search` | Tag filter → BM25+Embedding hybrid → causal filter |
| `robonix/service/memory/promote`       | Promote short-term → long-term nodes |

## Architecture

```
LogRecord ──[remember]──► MemoryNode
                              ├── TagIndex (inverted, O(1) filter)
                              ├── VectorStore (all-MiniLM-L6-v2 d=384, BM25+Cosine)
                              └── GraphStore (JSON persistence, causal edges)
```

## Run standalone

```sh
cd services/memory
PYTHONPATH=. python3 -c "
import asyncio
from memory_service.service import MemoryService
svc = MemoryService()
asyncio.run(svc.init())
print(f'Nodes: {svc.graph.count()}')
"
```

## Configuration

- `EMBEDDING_MODEL_PATH` — path to all-MiniLM-L6-v2 model directory
  (optional; otherwise sentence-transformers uses its standard model cache)
- Spatial records containing objects must provide their actual coordinate
  frame in `origin`; missing origins are rejected rather than labelled
  `world`.
- Falls back to deterministic hash embeddings when model unavailable

## Status

Phase 1 — core remember + search + compact. Ready for Demo 1 (object spatiotemporal
backtracking) and Demo 2 (task history backtracking).
