# Keystone — identity, configuration, policy

One of the 12 Robonix system components. Stores the body's identity,
its persistent configuration, and the policy decisions that depend on
who is operating it.

**Status — v0.1 stub.** Not yet implemented.

Today: deployment manifests in YAML, per-package `package_manifest.yaml`,
and ad-hoc `.env` files. User identity / per-user permissions are not
modelled centrally. The currently implemented user gate lives in Liaison:
text/API tasks use `context_json.user_id`, while voice sessions must pass
voiceprint before Pilot/TTS/action when access control is enabled.

When Keystone lands it will:

- own the canonical key/value config store (read by all components on
  boot, hot-reloadable for some keys),
- track identities (operators, deployments, fleets) and the policies
  that bind them to capability allow-lists,
- be the source-of-truth that Liaison and future Sentinel policy checks
  consult for "is this user allowed to call this skill right now".
