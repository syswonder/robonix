# Configuration of the `system.executor:` block in robonix_manifest.yaml.
#
# rbnx boot passes the whole block to robonix-executor as `--config-json` and
# derives `--listen`, `--atlas` and `--log` from it. Only string values are
# turned into flags; a key with another type is dropped silently. Every string
# in the manifest has `$VAR` / `${VAR}` expanded first, and an unset variable
# becomes an empty string.
#
# `status: enabled | disabled` is read by rbnx, not by Executor. Executor
# cannot be disabled; `status: disabled` stops the boot.
#
# The provider id (`--id`, ROBONIX_EXECUTOR_PROVIDER_ID, default `executor`)
# cannot be set from this block. ROBONIX_WORKSPACE (default: the working
# directory) is the root the built-in file capabilities are confined to.

specVersion: 1
description: >-
  Executor runs RTDL plans and dispatches each capability call to its
  provider. This block sets its address, the Atlas it registers with, its log
  level, and optional result verification.

properties:
  listen:
    type: string
    default: 127.0.0.1:50061
    description: >-
      Socket address the Executor gRPC services bind to, as IP:port (a host
      name is rejected at startup). An unspecified IPv4 address (0.0.0.0) is
      advertised to Atlas as 127.0.0.1 on the same port. When absent, Executor
      reads ROBONIX_EXECUTOR_LISTEN, then binds 127.0.0.1:50061.

  atlas:
    type: string
    description: >-
      Atlas endpoint to dial, as host:port or http(s)://host:port. When
      absent, rbnx passes `system.atlas.listen` unchanged. When that is absent
      too, Executor reads ROBONIX_ATLAS_ENDPOINT, then ROBONIX_ATLAS, then
      uses 127.0.0.1:50051.

  log:
    type: string
    enum: [debug, info, warn, warning, error]
    default: info
    description: >-
      Lowest level written to Executor's log file. When absent, the
      SCRIBE_FILE_LEVEL environment variable applies, then info. Console
      output is controlled separately by SCRIBE_CONSOLE_LEVEL.
    # Matching ignores case. An unrecognized value is ignored without a
    # warning, and the fallback applies.

  verification:
    type: object
    description: >-
      Routes successful capability results to a verifier provider before the
      node counts as succeeded. A call that matches no rule keeps its result.
      A verifier is called on its `robonix/service/verifier/verify`
      capability, with a fixed 60-second timeout. If it fails, times out or
      returns an invalid response, the node fails.
    # When the block has no `verification` key, Executor falls back to the
    # `verification` key of the YAML file named by --config /
    # ROBONIX_CONFIG_PATH, and with neither, verifies nothing. A wrong type
    # anywhere in this object stops Executor at startup.
    properties:
      overlap:
        type: boolean
        default: false
        description: >-
          When false, a node under a matching rule reports its terminal state
          only after the verifier answers, and the plan waits. When true, the
          node reports VERIFYING and the plan continues while the verifier
          runs; the plan's result still waits for every verifier. Applies to
          all rules.
      rules:
        type: array
        default: []
        description: >-
          Verification rules. A rule naming both the contract and the provider
          takes precedence over a rule naming only the contract. Two rules with
          the same contract and the same provider (or both without one) stop
          Executor at startup.
        items:
          type: object
          required: [target_contract_id, verifier_provider_id]
          properties:
            target_contract_id:
              type: string
              description: >-
                Contract id whose successful results are verified, for example
                robonix/service/navigation/navigate. Surrounding whitespace is
                removed; an empty value stops Executor at startup.
            target_provider_id:
              type: [string, "null"]
              x-provider: true
              description: >-
                Limits the rule to calls served by this provider. Absent, null
                or blank means any provider of the contract.
            verifier_provider_id:
              type: string
              x-provider: robonix/service/verifier/verify
              description: >-
                Provider id of the verifier. Surrounding whitespace is removed;
                an empty value stops Executor at startup.
            verifier_args:
              type: object
              default: {}
              description: >-
                Passed unchanged to the verifier as `verifier_args`, alongside
                the target call's provider, contract, node description,
                arguments and output. Any other JSON type stops Executor at
                startup.
