# Configuration of the `system.atlas:` block in robonix_manifest.yaml.
#
# rbnx boot passes the whole block to robonix-atlas as `--config-json` and
# derives `--listen`, `--log` and `--capabilities` from it. Only string values
# are turned into flags; a key with another type is dropped silently. Every
# string in the manifest has `$VAR` / `${VAR}` expanded first, and an unset
# variable becomes an empty string.
#
# `status: enabled | disabled` is read by rbnx, not by Atlas. Atlas cannot be
# disabled; `status: disabled` stops the boot.
#
# Environment only:
#   ROBONIX_ATLAS_HEARTBEAT_TIMEOUT_MS    default 90000. A provider without a
#                                         heartbeat for this long is marked
#                                         TERMINATED.
#   ROBONIX_ATLAS_GC_AFTER_TERMINATED_MS  default 600000. A TERMINATED record
#                                         is removed after this long.
#   ROBONIX_ATLAS_EVICTION_INTERVAL_MS    default 10000. How often both checks
#                                         run.

specVersion: 1
description: >-
  Atlas is the capability registry and contract catalog. This block sets the
  address it serves on, where it loads contract definitions from, and its log
  level.

properties:
  listen:
    type: string
    default: 0.0.0.0:50051
    description: >-
      Socket address Atlas binds its gRPC service to, as IP:port (a host name
      is rejected at startup). When absent, Atlas reads ROBONIX_ATLAS_LISTEN,
      then binds 0.0.0.0:50051. rbnx also uses this value as the Atlas
      endpoint for every other component: packages dial it with 0.0.0.0
      replaced by 127.0.0.1, while executor, pilot, liaison and vitals receive
      it unchanged unless their own block sets `atlas`. When the key is
      absent, rbnx gives packages 127.0.0.1:50051.

  capabilities:
    type: string
    description: >-
      Comma-separated directories scanned recursively for contract TOML files.
      On a duplicate contract id, the later directory wins. Setting this
      replaces the list rbnx otherwise builds:
      $ROBONIX_SOURCE_PATH/capabilities followed by the `capabilities/`
      directory of every primitive, service and skill package in the
      deployment. If that list is also empty, Atlas reads
      ROBONIX_ATLAS_CAPABILITIES, then $ROBONIX_SOURCE_PATH/capabilities. With
      no directory at all the registry is empty, and every contract lookup
      returns not found.
    # Relative paths resolve against the directory `rbnx boot` runs from,
    # not the manifest directory. A missing directory is skipped with a
    # warning, and any `lib/` subdirectory is skipped. A YAML list is not
    # accepted here; it is ignored and the default list is used.

  log:
    type: string
    enum: [debug, info, warn, warning, error]
    default: info
    description: >-
      Lowest level written to Atlas's log file. When absent, the
      SCRIBE_FILE_LEVEL environment variable applies, then info. Console
      output is controlled separately by SCRIBE_CONSOLE_LEVEL.
    # Matching ignores case. An unrecognized value is ignored without a
    # warning, and the fallback applies.
