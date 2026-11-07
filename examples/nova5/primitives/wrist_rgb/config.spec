specVersion: 1
description: Nova5 wrist RGB camera using the existing hand-eye calibration.
required: [calibration_root, camera_python]
properties:
  calibration_root:
    type: string
    description: Absolute path to the external calibration project containing handeye/ and config.json.
  camera_python:
    type: string
    description: Absolute path to the camera SDK Python interpreter.
