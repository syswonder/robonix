# Runtime configuration accepted by the client audio bridge primitive.
#
# This documents the mapping passed as the package instance's `config:` value.
# It is not loaded as a schema. Values below are runtime defaults.

config:
  # string, default: reverse; accepted value: reverse.
  # In reverse mode the robot listens and robonix-client connects to it, so a
  # normal remote-client deployment does not need the client's IP address.
  transport: reverse

  # string IP address or interface name, default: 0.0.0.0.
  # Bind address for the reverse WebSocket endpoint. Keep 0.0.0.0 when the
  # client runs on another host; use a loopback address only for local tests.
  listen_host: 0.0.0.0

  # integer TCP port, default: 60002; valid range: 1..65535.
  # Port exposed by the reverse WebSocket endpoint. The same port must be
  # reachable from robonix-client.
  listen_port: 60002

# Deprecated client-owned-server transport. Existing deployments remain
# compatible: setting `host` selects this mode when `transport` is omitted.
legacy_config:
  # string hostname or IP address, no default in reverse mode.
  # Address of the legacy client_audio_server process.
  host: 127.0.0.1

  # integer TCP port, default: 60000; valid range: 1..65535.
  # Port of the legacy client_audio_server process.
  port: 60000
