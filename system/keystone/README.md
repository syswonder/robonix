# Keystone

Keystone is Robonix's core system component for user identity. Liaison, Pilot,
and Executor can each require it to authenticate requests at their own entry
point.

Keystone stores user accounts, roles, voice-guard settings, and login sessions
in SQLite. Passwords are hashed with Argon2; only session token hashes are
persisted. The database defaults to `$ROBONIX_DATA_DIR/keystone.db` (or
`rbnx-boot/data/keystone.db` when unset) and is owner-only on Unix.

## First administrator

Keystone creates an administrator only when the database contains no users.
Set `ROBONIX_KEYSTONE_BOOTSTRAP_ADMIN_PASSWORD` before the first boot to supply
the initial password explicitly. If it is absent, Keystone generates a random
password and writes the one-time credentials to:

```text
rbnx-boot/data/keystone-bootstrap-admin.txt
```

On Unix the file mode is `0600`. The administrator must change the password
after the first login; Keystone then removes the credential file.

## Deployment configuration

Configure Keystone under `system.keystone` in the deployment manifest.
Liaison, Pilot, and Executor each have an independent `require_keystone`
setting. Set `system.liaison.keystone_endpoint` when Liaison should use this
instance as its connection fallback. Keystone gRPC has no TLS; remote
connections need a trusted network or encrypted tunnel.

Configuration fields:

| Field | Default | Purpose |
|---|---|---|
| `listen` | `127.0.0.1:50095` | Keystone gRPC listener |
| `database` | data directory `keystone.db` | SQLite database |
| `bootstrap_credentials_file` | data directory `keystone-bootstrap-admin.txt` | generated first-login credential file |
| `bootstrap_admin_username` | `admin` | first administrator username |
| `bootstrap_admin_display_name` | `Administrator` | first administrator display name |
| `bootstrap_admin_email` | empty | first administrator email |
| `voice_threshold` | `0.25` | minimum Voiceprint confidence when voice guard is enabled |

## Access behavior

When Liaison requires Keystone:

- Text turns require a live session.
- When a user's voice guard is off, authenticated voice turns skip Voiceprint.
- When the guard is on, every voice turn must identify as that same logged-in
  user above Keystone's configured confidence threshold.
- A disabled user cannot log in, and disabling the account revokes its sessions.
- Account and role management requires `admin`; Keystone prevents deleting,
  disabling, or demoting the last enabled administrator.
- Changing a password keeps the current session and revokes the user's other
  sessions.

Run the focused tests from the repository root:

```bash
cargo test -p robonix-keystone -p robonix-liaison
```
