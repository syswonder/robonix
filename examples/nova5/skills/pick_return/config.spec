specVersion: 1
description: One RGB-guided Nova5 box cycle on the configured hardware.
required: [python, settings]
properties:
  python:
    type: string
    description: Absolute path to the calibrated workflow Python interpreter.
  settings:
    type: string
    description: Absolute path to examples/nova5/settings.json.
