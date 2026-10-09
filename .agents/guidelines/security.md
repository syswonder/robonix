# Security

*Can it be misused, or leak what it should not?*

### escape-untrusted-text

Text that comes from a person, a model or the network is escaped before it becomes HTML, SQL, a shell argument or a file name. In web pages, set `textContent` or escape before inserting into `innerHTML`.

### local-by-default

Bind internal servers (gRPC, viewers, debug endpoints) to `127.0.0.1`. Expose them only through the component's own port, which the deployment configures.

### no-secrets

No API keys, tokens, passwords, personal paths or machine names in code, logs, tests or commit messages. Read secrets from the environment.

### validate-paths

An id or name that becomes part of a file path is sanitised first, so it cannot climb out of its directory.
