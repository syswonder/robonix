specVersion: 1
description: >-
  The Memgraph memory service ignores its instance config. All settings come
  from environment variables, set in the deployment manifest's env block.

# Environment variables read by memory_service:
#   AGENT_MEMORY_DIR           graph store directory; default <package>/memory
#                              (the start script exports the same path).
#   MEMGRAPH_KEEP_DATA         1/true/yes keeps graph_store.json and
#                              <package>/data/images across starts; by default
#                              both are deleted at every start.
#   MEMGRAPH_ENABLE_EMBEDDING  1/true/yes loads the embedding model and enables
#                              BM25 + cosine ranking; default off (BM25 only).
#   EMBEDDING_MODEL_PATH       local all-MiniLM-L6-v2 directory; default uses
#                              the sentence-transformers cache.
#   MEMGRAPH_LLM_BASE_URL, MEMGRAPH_LLM_API_KEY, MEMGRAPH_LLM_MODEL
#                              LLM search endpoint; falls back to VLM_* and
#                              then OPENAI_*. Model default: gpt-4.1.
#   VLM_BASE_URL, VLM_API_KEY, VLM_MODEL
#                              observation endpoint; falls back to OPENAI_*.
#                              Model default: gpt-4.1.
#   SCENE_HOOK_PORT            Scene Hook HTTP port; default 37798.
#   MEMORY_LOG_LEVEL           log level; default INFO.
#   SCRIBE_LOG_DIR             log directory; default <package>/logs.
properties: {}
