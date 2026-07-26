# Keystone

Keystone is Robonix's persistent user identity and account service. Liaison
uses it to authenticate text, voice, and hands-free interactions before they
reach Pilot.

Keystone stores:

- user accounts, profile fields, enabled state, and `user` / `admin` roles;
- Argon2 password hashes;
- opaque login sessions (only SHA-256 token hashes are persisted);
- per-user voice-guard policy and Voiceprint bindings;
- system account settings such as whether public signup is enabled.

The SQLite database uses foreign keys and WAL mode. Its default path is
`$ROBONIX_DATA_DIR/keystone.db`; `rbnx boot` sets `ROBONIX_DATA_DIR` to the
deployment's `rbnx-boot/data` directory.

## First administrator

Keystone creates an administrator only when the database contains no users.
Set `ROBONIX_KEYSTONE_BOOTSTRAP_ADMIN_PASSWORD` before the first boot to supply
the initial password explicitly. If it is absent, Keystone generates a random
password and writes the one-time credentials to:

```text
rbnx-boot/data/keystone-bootstrap-admin.txt
```

On Unix the file mode is `0600`. The startup log prints its path, never the
password. The administrator must change the password after the first login.

## Deployment configuration

Keystone is a built-in system component:

```yaml
system:
  keystone:
    listen: 127.0.0.1:50095
    log: info
  liaison:
    listen: 0.0.0.0:50081
    keystone_endpoint: 127.0.0.1:50095
```

Keep the Keystone listener private to the robot host. Clients use the account
API proxied by Liaison; they do not connect to port `50095` directly.

Optional configuration can be passed in the `system.keystone` mapping:

| Field | Default | Purpose |
|---|---|---|
| `listen` | `127.0.0.1:50095` | Keystone gRPC listener |
| `database` | `$ROBONIX_DATA_DIR/keystone.db` | SQLite database |
| `bootstrap_credentials_file` | `$ROBONIX_DATA_DIR/keystone-bootstrap-admin.txt` | generated first-login credential file |
| `bootstrap_admin_username` | `admin` | first administrator username |
| `bootstrap_admin_display_name` | `Administrator` | first administrator display name |
| `bootstrap_admin_email` | empty | first administrator email |

The bootstrap password is intentionally environment-only and is not accepted
from the deployment manifest.

## Access behavior

- Text turns require a live session, but do not require a voiceprint.
- When a user's voice guard is off, authenticated voice turns skip Voiceprint.
- When the guard is on, every voice turn must identify as that same logged-in
  user above the configured confidence threshold.
- A disabled user cannot log in, and disabling the account revokes its sessions.
- Account and role management requires `admin`; Keystone prevents deleting,
  disabling, or demoting the last enabled administrator.
- Changing a password keeps the current session and revokes the user's other
  sessions.

Run the focused tests from the repository root:

```bash
cargo test -p robonix-keystone -p robonix-liaison
```
