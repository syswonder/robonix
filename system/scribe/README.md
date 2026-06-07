# Scribe — system log, replay, audit

One of the 12 Robonix system components. Structured, persistent,
replayable system journal.

**Status — v0.1 stub.** Not yet implemented.

Today every component logs to its own file under
`<deploy>/rbnx-boot/logs/<component>.log`. There is no central
structured store, no schema, and no replay tool.

When Scribe lands it will:

- ingest a structured event stream from atlas (state transitions,
  declare / connect / disconnect), executor (plans, dispatches,
  failures), sentinel (rule hits), and per-component lifecycle,
- persist with retention / rotation,
- expose a query / replay capability so a past plan can be re-played
  against the current scene for debugging or audit.
