specVersion: 1
description: >-
  Storage locations and ONNX threading for the Memsearch memory service. Each
  key takes priority over its environment fallback; model credentials for
  compact stay in the environment (VLM_* or OPENAI_*).

# Configuration is applied once. A repeated Driver(CMD_INIT) with a different
# resolved configuration fails; an identical one is accepted.
properties:
  memory_dir:
    type: string
    description: >-
      Directory of indexed Markdown notes; save appends to
      <memory_dir>/YYYY-MM-DD_notes.md. Must be non-empty. Relative paths are
      resolved from the package runtime working directory. Use an absolute
      path outside rbnx-build when memories must survive cache or build
      cleanup. Environment fallback: AGENT_MEMORY_DIR.
    default: ./agent_memory

  milvus_uri:
    type: string
    description: >-
      Milvus store. Must be non-empty. A filesystem path is resolved from the
      package runtime working directory and its parent directory is created.
      Values containing "://", and host:port values without "/" that do not
      end in .db, are passed to Milvus unchanged. Environment fallback:
      AGENT_MILVUS_URI.
    default: ./agent_milvus.db

  onnx_threads:
    type: integer
    description: >-
      ONNX Runtime intra-op threads, applied only on aarch64. Other
      architectures keep the ONNX Runtime defaults. Environment fallback:
      MEMSEARCH_ONNX_THREADS.
    default: 1
    minimum: 1
