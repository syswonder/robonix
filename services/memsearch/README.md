# memsearch — reference memory service

`memsearch` is the **default reference implementation** of
`robonix/service/memory/*`. It backs the pilot's memory layer with a
small SQLite-backed `(when, what, who)` event store plus a vector index
for semantic retrieval.

> Memsearch is **a service, not a system component**. It is one
> ready-to-use implementation that lives in this repository so a fresh
> deploy "just works"; integrators are expected to replace it with their
> own service (Redis, Postgres + pgvector, a domain-specific KV store,
> …) by swapping the contract bindings in their deploy manifest.

## Capability surface

| Contract | Behaviour |
| --- | --- |
| `robonix/service/memory/save`    | Insert an `(at, kind, summary, payload)` event   |
| `robonix/service/memory/search`  | Top-k semantic + recency-weighted retrieval      |
| `robonix/service/memory/compact` | Garbage-collect / summarise old events           |

IDL lives under [`capabilities/lib/memory/`](../../capabilities/lib/memory/);
contracts under [`capabilities/service/memory/`](../../capabilities/service/memory/).

## Run

From a deploy that lists this service in `robonix_manifest.yaml`, the
service is started by `rbnx boot` automatically. Manual:

```sh
cd services/memsearch
uv sync
uv run -m memsearch_service.main
```

Configuration is via the deploy manifest under `system:` (or `service:`).
Key fields:

- `backend` — `sqlite` (default) or `chroma`.
- `db_path` — SQLite path; defaults to `~/.robonix/memsearch.sqlite`.
- `embed_model` — name of the embedding model to fetch on first launch.

## Status

Reference quality — fine for demos and small deployments. Production
deployments should swap in a hardened service.
