# Configuration of the `system.pilot:` block in robonix_manifest.yaml.
#
# rbnx boot passes the whole block to robonix-pilot as `--config-json`, from
# which Pilot reads only `log`. Every other key reaches Pilot as a flag rbnx
# derives from the block: `--listen`, `--atlas`, `--log` and the `--vlm-*`
# flags. Only string values become flags, except `vlm.context_window_tokens`,
# which must be an integer; a key of another type is dropped silently. Every
# string in the manifest has `$VAR` / `${VAR}` expanded first, and an unset
# variable becomes an empty string.
#
# `status: enabled | disabled` is read by rbnx, not by Pilot. Pilot may be
# disabled; a disabled block is removed before boot.
#
# The provider id (`--id`, ROBONIX_PILOT_PROVIDER_ID, default `pilot`) cannot
# be set from this block. Environment only:
#   ROBONIX_PILOT_MAX_TOOL_ROUNDS        default 64. RTDL rounds per turn.
#   ROBONIX_PILOT_VLM_IDLE_TIMEOUT_SECS  default 30, clamped to 5..300. How
#                                        long a streaming model response may
#                                        stay silent.
#   ROBONIX_PILOT_SOUL                   path to a personality file read into
#                                        the system prompt; default
#                                        ~/.robonix/SOUL.md when present.
#   ROBONIX_SESSION_DIR                  where session transcripts are kept;
#                                        set by rbnx boot.

specVersion: 1
description: >-
  Pilot turns user requests into RTDL plans using an OpenAI-compatible model.
  This block sets its address, the Atlas it registers with, its log level, and
  the model endpoint.

required: [vlm]

properties:
  listen:
    type: string
    default: 127.0.0.1:50071
    description: >-
      Socket address the Pilot gRPC services bind to, as IP:port (a host name
      is rejected at startup). An unspecified address (0.0.0.0 or [::]) is
      advertised to Atlas as loopback on the same port. When absent, Pilot
      reads ROBONIX_PILOT_LISTEN, then binds 127.0.0.1:50071.

  atlas:
    type: string
    description: >-
      Atlas endpoint to dial, as host:port or http(s)://host:port. When
      absent, rbnx passes `system.atlas.listen` unchanged. When that is absent
      too, Pilot reads ROBONIX_ATLAS_ENDPOINT, then ROBONIX_ATLAS, then uses
      127.0.0.1:50051.

  log:
    type: string
    enum: [debug, info, warn, warning, error]
    default: info
    description: >-
      Lowest level written to Pilot's log file. When absent, the
      SCRIBE_FILE_LEVEL environment variable applies, then info. At debug,
      the log also holds every message sent to the model and every raw reply,
      including user tasks verbatim. Console output is controlled separately
      by SCRIBE_CONSOLE_LEVEL.
    # Matching ignores case. An unrecognized value is ignored without a
    # warning, and the fallback applies.

  vlm:
    type: object
    description: The model endpoint Pilot plans with.
    # rbnx refuses to start Pilot unless upstream, api_key and model are all
    # present and non-empty in this block; the ROBONIX_VLM_* variables Pilot
    # itself reads as fallbacks are not consulted by that check. Write them as
    # ${VLM_BASE_URL}, ${VLM_API_KEY} and ${VLM_MODEL}.
    required: [upstream, api_key, model]
    properties:
      upstream:
        type: string
        description: >-
          Base URL of an OpenAI-compatible API, for example
          https://api.openai.com/v1. Pilot appends /chat/completions and
          /models; a trailing slash is removed. Must not be blank.

      api_key:
        type: string
        x-secret: true
        description: >-
          API key sent to the upstream. Must not be blank.

      model:
        type: string
        description: >-
          Model id sent with each request and used to look up the context
          window. Must not be blank.

      api_format:
        type: string
        enum: [openai]
        default: openai
        description: >-
          Request dialect. Only openai is implemented; any other value stops
          Pilot at startup. When absent, Pilot reads ROBONIX_VLM_FORMAT, then
          uses openai.

      context_window_tokens:
        type: integer
        minimum: 0
        description: >-
          Total context window of the deployed model, in tokens. It overrides
          whatever the upstream reports, which matters when a gateway serves a
          smaller model under a larger model's name. When absent, Pilot reads
          ROBONIX_VLM_CONTEXT_WINDOW_TOKENS. With no size from either, or 0,
          Pilot asks the upstream (GET /models/<model>, then GET /models). If
          that gives no size, Pilot starts without automatic history
          compaction and logs a warning.
        # Must be a literal YAML integer. `${VAR}` expands to a string, which
        # rbnx drops; set ROBONIX_VLM_CONTEXT_WINDOW_TOKENS in the environment
        # instead.
