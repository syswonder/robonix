specVersion: 1
description: >-
  Enrollment storage, match threshold and model device for the Voiceprint
  service. Each key takes priority over its environment fallback.

# Configuration is applied once. A repeated Driver(CMD_INIT) with a different
# resolved configuration fails; an identical one is accepted.
properties:
  data_dir:
    type: string
    description: >-
      Directory holding the enrolled-speaker database, enrolled.json. Created
      if missing. Relative paths are resolved from the service working
      directory, the package directory. Environment fallback:
      VOICEPRINT_DATA_DIR.
    default: rbnx-build/data

  threshold:
    type: number
    description: >-
      Minimum cosine similarity for a known-speaker result and for rejecting
      an enrollment whose voice is already enrolled. A value that is not a
      finite number in [0, 1] fails Driver(CMD_INIT). Environment fallback:
      VOICEPRINT_THRESHOLD.
    default: 0.25
    minimum: 0
    maximum: 1

  device:
    type: [string, "null"]
    description: >-
      Torch device for the ECAPA-TDNN model, such as cuda:0 or cpu, used when
      the provider activates. Omit or set null for automatic selection:
      VOICEPRINT_DEVICE if set, otherwise cuda:0 when CUDA is available and
      cpu when it is not. An empty string fails Driver(CMD_INIT).
    default: null
