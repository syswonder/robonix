# memgraph — Scribe Mem structured memory service

Memgraph is Robonix's **structured CKG memory system** (formerly Scribe Mem):
it consumes `LogRecord` entries from Scribe Log and builds a causal knowledge
graph (CKG) with tag-indexed, vector-searchable memory nodes.

Runs in parallel with **memsearch** (`robonix/service/memory/*`) under a
separate namespace (`robonix/service/memgraph/*`) so Pilot can discover both.

## Capability surface

| Contract | Behaviour |
|---|---|
| `robonix/service/memgraph/remember` | Write a MemoryNode (tag → vector → graph) |
| `robonix/service/memgraph/search`   | Tag filter → BM25+Embedding hybrid → causal filter |
| `robonix/service/memgraph/compact`  | Promote short-term → long-term nodes |

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
  (defaults to `~/EmbodyMemory/memory/all-MiniLM-L6-v2`)
- Falls back to deterministic hash embeddings when model unavailable

## Status

Phase 1 — core remember + search + compact. Ready for Demo 1 (object spatiotemporal
backtracking) and Demo 2 (task history backtracking).
