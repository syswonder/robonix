# Client audio uses reverse transport by default: this provider listens and
# robonix-client connects to it. A normal deployment needs no config.

config:
  transport: reverse       # Default. The only named transport mode.
  listen_host: 0.0.0.0
  listen_port: 60002

# Backward-compatible client-owned server mode. Setting host selects it;
# transport is omitted in that mode.
legacy_config:
  host: 127.0.0.1
  port: 60000
