# Runtime configuration accepted by the Voiceprint service.
#
# This documents the mapping passed as the service instance's `config:` value.
# It is not loaded as a schema. Environment variables provide fallbacks for
# standalone use; instance config delivered by Driver(CMD_INIT) takes priority.

config:
  # string directory path, default: rbnx-build/data.
  # Stores the persistent enrolled-speaker database as enrolled.json.
  # Environment fallback: VOICEPRINT_DATA_DIR.
  data_dir: rbnx-build/data

  # finite float cosine similarity in [0, 1], default: 0.25.
  # Minimum score for a known-speaker result and duplicate-voice rejection.
  # Invalid, non-finite, or out-of-range values fail Driver(CMD_INIT).
  # Environment fallback: VOICEPRINT_THRESHOLD.
  threshold: 0.25

  # optional string Torch device. Omit or use null for automatic selection:
  # cuda:0 when CUDA is available, otherwise cpu. Explicit examples are
  # cuda:0 and cpu. Environment fallback: VOICEPRINT_DEVICE.
  device: null
