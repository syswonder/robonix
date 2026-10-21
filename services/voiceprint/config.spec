specVersion: 1
description: >-
  Enrollment storage and match threshold for the Voiceprint
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
      an enrollment whose voice is already enrolled. Calibrated for the CAM++
      embedding (genuine 0.68-0.90, impostor 0.22-0.44 on the official test
      pairs). A value that is not a finite number in [0, 1] fails
      Driver(CMD_INIT). Environment fallback: VOICEPRINT_THRESHOLD.
    default: 0.55
    minimum: 0
    maximum: 1
