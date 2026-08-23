# Runtime configuration accepted by the Memory service.
#
# This documents the mapping passed as the service instance's `config:` value.
# It is not loaded as a schema. Explicit instance config takes priority; the
# listed environment variables remain compatibility fallbacks for standalone
# deployments.

config:
  # string directory path, default: ./agent_memory.
  # Relative paths are resolved from the package runtime working directory.
  # Use an absolute path outside rbnx-build when memories must survive cache or
  # build cleanup. Environment fallback: AGENT_MEMORY_DIR.
  memory_dir: ./agent_memory

  # non-empty string, default: ./agent_milvus.db.
  # A filesystem path is resolved from the package runtime working directory;
  # remote endpoints and URIs are preserved. Environment fallback:
  # AGENT_MILVUS_URI.
  milvus_uri: ./agent_milvus.db

  # integer, default: 1; minimum: 1.
  # ONNX Runtime intra-op threads on aarch64. Other architectures retain their
  # native ONNX Runtime defaults. Environment fallback: MEMSEARCH_ONNX_THREADS.
  onnx_threads: 1
