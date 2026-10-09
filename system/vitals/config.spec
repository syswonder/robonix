# The `system.vitals:` block of robonix_manifest.yaml. rbnx boot passes the
# whole block to robonix-vitals as --config-json and also turns listen, atlas,
# provider_id (or id), thresholds_path, soma_endpoint, config and log into CLI
# flags. A key of the wrong type stops Vitals at startup ("parse vitals
# --config-json"). Other keys are ignored.
#
# Precedence: CLI flag or its environment variable, then this block, then the
# `config` file, then the defaults below. Keys rbnx turns into flags therefore
# win over their environment variable; the mock_soma_* keys do not, and their
# ROBONIX_VITALS_MOCK_SOMA_* variables win over this block.

specVersion: 1
description: >-
  Configuration of Vitals, which watches the robot's health (temperatures,
  voltages, joint motors and system modules), checks thresholds and reports
  over gRPC.

properties:
  listen:
    type: string
    default: 127.0.0.1:50093
    description: >-
      gRPC listen address, IP:port. Environment fallback:
      ROBONIX_VITALS_LISTEN. rbnx checks the port is free before spawning.

  atlas:
    type: string
    description: >-
      Atlas endpoint. When unset, rbnx passes system.atlas.listen. Without
      either, Vitals uses ROBONIX_ATLAS_ENDPOINT, then 127.0.0.1:50051.
      `atlas_endpoint` is accepted as an alias, but rbnx does not turn it into
      a flag, so system.atlas.listen wins over it. Setting both spellings
      stops Vitals at startup.

  provider_id:
    type: string
    default: vitals
    description: >-
      Atlas provider id Vitals registers as. Environment fallback:
      ROBONIX_VITALS_PROVIDER_ID.

  id:
    type: string
    description: Older spelling of provider_id, used when provider_id is unset.

  thresholds_path:
    type: string
    description: >-
      YAML file of Soma threshold rules. Default: thresholds/example_thresholds.yaml
      in the Vitals source tree the binary was built from. Environment
      fallback: ROBONIX_VITALS_THRESHOLDS_PATH. A missing file, or one that
      fails to parse, leaves the built-in rules in force. A relative path is
      resolved against the directory rbnx boot runs in.

  soma_endpoint:
    type: string
    description: >-
      Soma gRPC endpoint for the health stream. When unset, rbnx passes
      system.soma.listen, with 0.0.0.0 replaced by 127.0.0.1. Environment
      fallback: ROBONIX_SOMA_ENDPOINT. Without any of them, Vitals finds
      robonix/system/soma/health through Atlas.

  expected_modules:
    type: array
    description: >-
      Modules shown in the module-health view. Setting this replaces the
      default list, which holds vitals (provider_id = this instance), executor
      and pilot, all required. Only executor and pilot are polled for health;
      a module matches them by module_id or by capability.
    items:
      type: object
      required: [module_id]
      properties:
        module_id:
          type: string
          description: Module name. A blank one stops Vitals at startup.
        provider_id:
          type: [string, "null"]
          x-provider: true
          description: Atlas provider id of the module. Defaults to module_id.
        capability:
          type: [string, "null"]
          description: >-
            Health contract to poll: robonix/system/executor/get_health or
            robonix/system/pilot/get_health.
        policy:
          type: string
          enum: [required, optional, disabled]
          default: optional
          description: >-
            required: a module that never reports is marked unavailable with
            error health after ttl_ms. optional: a stale module is a warning.
            disabled: shown as disabled and not polled.
        ttl_ms:
          type: integer
          minimum: 0
          maximum: 4294967295
          default: 5000
          description: >-
            Milliseconds after Vitals starts before a required module that has
            not reported is marked unavailable.

  config:
    type: string
    description: >-
      Optional YAML file with the same keys as this block, except status,
      provider_id, config and log. Its values apply only where this block, a
      CLI flag and the environment give none. Environment fallback:
      ROBONIX_CONFIG_PATH. A relative path is resolved against the directory
      rbnx boot runs in.

  log:
    type: string
    default: robonix_vitals=info
    description: >-
      env_logger filter, for example `info` or `robonix_vitals=debug`. Vitals
      does not log through Scribe, so the level names of other components do
      not apply here. When unset, RUST_LOG applies, then the default.

  # The process runs as a mock Soma instead of Vitals only with --mock-soma or
  # ROBONIX_VITALS_MOCK_SOMA=true, which this block cannot set. The keys below
  # are read but have no effect otherwise.

  mock_soma_id:
    type: string
    default: mock-soma
    x-group: Mock Soma
    description: Provider id the mock Soma registers as.

  mock_soma_listen:
    type: string
    default: 127.0.0.1:50092
    x-group: Mock Soma
    description: gRPC listen address of the mock Soma.

  mock_soma_scenario:
    type: string
    enum: [normal, ramp, fault, toggle, mixed]
    default: normal
    x-group: Mock Soma
    description: >-
      Synthetic health pattern. Matched case-insensitively; an unknown value
      falls back to normal with a warning.

  mock_soma_interval_ms:
    type: integer
    minimum: 0
    default: 10000
    x-group: Mock Soma
    description: Milliseconds between mock health updates.

  mock_soma_arm:
    type: string
    enum: [synthetic, piper, koch]
    default: synthetic
    x-group: Mock Soma
    description: >-
      Arm data source: synthetic data, a Piper arm over CAN, or a Koch arm over
      a serial port, the last two through a Python bridge process. Matched
      case-insensitively; an unknown value falls back to synthetic with a
      warning.

  mock_soma_piper_can:
    type: string
    default: can0
    x-group: Mock Soma
    description: CAN interface of the Piper arm. Used when mock_soma_arm is piper.

  mock_soma_koch_port:
    type: string
    default: /dev/ttyUSB0
    x-group: Mock Soma
    description: Serial port of the Koch arm. Used when mock_soma_arm is koch.

  mock_soma_bridge_python:
    type: string
    default: python3
    x-group: Mock Soma
    description: Python interpreter that runs the arm bridge script.

  mock_soma_piper_script:
    type: string
    x-group: Mock Soma
    description: >-
      Piper bridge script. Default: scripts/piper_bridge.py in the Vitals
      source tree the binary was built from.

  mock_soma_koch_script:
    type: string
    x-group: Mock Soma
    description: >-
      Koch bridge script. Default: scripts/koch_bridge.py in the Vitals source
      tree the binary was built from.
