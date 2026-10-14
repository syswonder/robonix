# Configuration accepted by robonix-keystone through YAML or --config-json.
# The bootstrap password is environment-only and is intentionally absent.

specVersion: 1
description: >-
  Keystone stores user accounts, sessions, roles, and voiceprint bindings.
  This block configures its Atlas connection, listener, persistent files, and
  first administrator.

properties:
  atlas_endpoint:
    type: string
    default: 127.0.0.1:50051
    description: >-
      Atlas endpoint. Command-line and environment settings take precedence.

  listen:
    type: string
    default: 127.0.0.1:50095
    description: >-
      Local gRPC listen address. Expose Keystone only through a component that
      authenticates external clients.

  provider_id:
    type: string
    default: keystone
    description: Provider id registered with Atlas.

  database:
    type: string
    default: rbnx-boot/data/keystone.db
    description: SQLite database path.

  bootstrap_credentials_file:
    type: string
    default: rbnx-boot/data/keystone-bootstrap-admin.txt
    description: >-
      File that receives generated first-login credentials when the database
      has no users and no bootstrap password is supplied through the
      environment.

  bootstrap_admin_username:
    type: string
    default: admin
    description: Username created when the database has no users.

  bootstrap_admin_display_name:
    type: string
    default: Administrator
    description: Display name created when the database has no users.

  bootstrap_admin_email:
    type: string
    default: ""
    description: Email created when the database has no users.

  log:
    type: string
    enum: [debug, info, warn, warning, error]
    default: info
    description: Lowest level written to Keystone's log file.
