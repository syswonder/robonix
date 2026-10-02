# The `system.soma:` block of robonix_manifest.yaml. rbnx boot passes the whole
# block to robonix-soma as --config-json and also turns listen, the Atlas
# endpoint, provider_id, robot_yaml, deployment_manifest, config and log into
# CLI flags. A key of the wrong type stops Soma at startup ("parse Soma
# --config-json"). Other keys are ignored.
#
# A manifest with primitive: or skill: entries but no system.soma block gets an
# empty one from rbnx, so Soma still runs. Soma cannot be disabled; `status:
# disabled` stops the boot.
#
# Each CLI flag also has an environment variable (ROBONIX_ATLAS_ENDPOINT,
# ROBONIX_SOMA_LISTEN, ROBONIX_SOMA_PROVIDER_ID, ROBONIX_SOMA_ROBOT_YAML,
# ROBONIX_SOMA_DEPLOYMENT_MANIFEST, ROBONIX_CONFIG_PATH). It applies when rbnx
# passes no flag for that key; a value in this block wins over it.

specVersion: 1
description: >-
  Configuration of Soma, which loads the robot's soma.yaml and URDF, serves
  them over gRPC, and starts the deployment's primitive and skill packages.

properties:
  robot_yaml:
    type: string
    description: >-
      Path to the robot's soma.yaml. Normally filled by rbnx: when unset or
      empty, rbnx uses soma.yaml next to the manifest if that file exists. A
      relative path is resolved against the manifest's directory. Soma exits
      if no robot YAML is given by this key, ROBONIX_SOMA_ROBOT_YAML or the
      config file.

  deployment_manifest:
    type: string
    description: >-
      Set by rbnx to the manifest selected with `rbnx boot -f`; a value written
      here is replaced. Soma reads its primitive and skill entries from this
      file. Without the flag, Soma uses robonix_manifest.yaml next to
      robot_yaml.

  listen:
    type: string
    default: 127.0.0.1:50091
    description: >-
      gRPC listen address, host:port. rbnx checks the port is free before
      spawning, and gives this address to Vitals as its Soma endpoint (0.0.0.0
      becomes 127.0.0.1).

  atlas_endpoint:
    type: string
    description: >-
      Atlas endpoint. When unset, rbnx uses `atlas`, then system.atlas.listen.
      Without any of them, Soma uses 127.0.0.1:50051.

  atlas:
    type: string
    description: Same as atlas_endpoint, which wins when both are set.

  provider_id:
    type: string
    default: soma
    description: Atlas provider id Soma registers as.

  runtime_reader_command:
    type: array
    items:
      type: string
    default: []
    description: >-
      Command that runs Soma's ROS 2 runtime-state reader, one argument per
      item, without a shell. `{script}` and `{config}` expand to the generated
      reader script and its source list. Use it when ROS 2 runs elsewhere, for
      example inside a simulator container. Empty means the config file's
      value, then `python3 -u {script} {config}`. The first item must name a
      program.

  config:
    type: string
    description: >-
      Optional YAML file with the keys atlas_endpoint, listen, provider_id,
      robot_yaml, deployment_manifest and runtime_reader_command. Its values
      apply only where neither this block nor a CLI flag gives one. rbnx
      resolves a relative path against the manifest's directory; relative
      paths inside the file are resolved against the file's own directory.

  log:
    type: string
    enum: [debug, info, warn, warning, error]
    default: info
    description: >-
      Lowest level written to Soma's log file. Matched case-insensitively; an
      unrecognised value is ignored. When unset, SCRIBE_FILE_LEVEL applies,
      then info.
