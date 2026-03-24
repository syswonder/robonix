# Namespace (immutable contract sketch)

> **中文导读（控制平面路径 ↔ 数据面 `.proto`）**：见 mdBook [控制平面与 Proto 契约](../../docs/src/chapter3-developer-guide/namespace-and-proto-contracts.md)（构建后位于《第三章：开发文档》）。

This document defines a stable, tree-shaped naming convention for:

1. Hardware and virtual hardware exposed to Robonix agents (`robonix/prm/...`).
2. System core services (VLM, maps, planning, managers, …) (`robonix/sys/...`).

It is not tied to a Linux distribution or to ROS 2 message names; ROS topics/actions remain implementation details inside a `ros2:*` runtime.

Authority: `RegisterNode.namespace` on the global `robonix-server`. Discovery uses `QueryNodes` with a prefix match on `namespace`. Dot-names such as `robonix.prm.base.move` may be derived from paths for logging only; they are not a second source of truth.

## Goals

- One predictable path per logical capability.
- Immutable path segments for released API levels; new capability means a new leaf, not in-place renames.
- Works for native transports (gRPC, MCP) and for ROS 2 nodes in containers or on the host.

## Root

All registered nodes SHOULD use a namespace under:

```text
robonix/<domain>/...
```

| `domain` | Purpose |
|----------|---------|
| `prm` | Physical robot, simulation, and virtual hardware abstractions (base, cameras, arms, …). |
| `sys` | Framework services: VLM/LLM, maps, planning, skill/task managers, debug helpers. |
| `lab` | Optional: test benches and lab-only rigs (if distinct from `prm`). |

Do not introduce ad-hoc top-level segments under `robonix/` without updating this document.

## System abstract interfaces (`robonix/...`) — target model

The tree alone does not define wire semantics. A system abstract interface is a single stable ID: a path whose last segment is the interface leaf, e.g.:

```text
robonix/sys/model/vlm/chat
robonix/prm/base/move
```

Reference agents and tools (e.g. `robonix-agent`) should bind to that ID via an embedded or file-based contract catalog, not via a hidden coupling to one demo’s `DeclareInterface.name`.

### Catalog and validation (normative intent, server TBD)

For each catalogued abstract interface ID, Robonix defines:

- Which transports are allowed (`grpc`, `ros2`, `mcp`, …).
- For `grpc`: the exact proto service + RPC (or message types) implementations must follow.
- For `ros2`: the exact `.srv` / `.action` / topic type (or generated type identity) implementations must follow.

Registration rule (future): if a provider registers as implementing a catalogued `robonix/...` abstract ID, `robonix-server` SHOULD reject the declaration when the advertised transport and `metadata_json` (or equivalent) do not match the catalog entry. Ad-hoc or experimental providers stay outside the catalog (see Vendor below).

### Instances

Multiple providers of the same abstract capability differ by instance, not by renaming the leaf:

- Path style (recommended): append `instances/<n>`, e.g. `robonix/sys/model/vlm/chat/instances/0`.
- Suffix style (alternative): `abstract_id:instance`, e.g. `robonix/sys/model/vlm/chat:0`, only if the control plane explicitly allows `:` in the stored namespace string and documents parsing rules.

Pick one style in the implementation and keep it consistent.

### Mapping to today’s `RegisterNode` + `DeclareInterface`

Today the path is split across `RegisterNode.namespace` and `DeclareInterface.name`. The abstract interface ID is the conceptual concatenation, e.g. `namespace` = `robonix/sys/model/vlm` and `name` = `chat` yields `robonix/sys/model/vlm/chat` when the server or docs define that join rule. Until the catalog and validation exist, the reference Python/Rust code is transitional and may filter by prefix plus short name.

### Vendor-defined surfaces (`com.*` / custom trees)

Implementations may also register under vendor-owned paths (e.g. `com.acme.services.my_llm` or a documented `com/...` tree). Those are not system-level contracts: no mandatory match to Robonix protos/srv unless the vendor opts in. Generic agents do not discover them unless configured explicitly.

## Node identity (`node_id`)

Every registered process MUST use a reverse-DNS `node_id` that is unique among nodes registered with the same global `robonix-server`:

- Format: `com.<segment>.<segment>[...]` — at least three dot-separated labels; the first label MUST be `com`.
- Segments after `com` are ASCII alphanumeric or `_`.
- Examples: `com.robonix.prm.tiago`, `com.robonix.services.vlm`, `com.robonix.runtime.agent`.
- Omitting `node_id` on register: the server assigns `com.robonix.ephemeral.<uuid>`.

The registry is keyed by `node_id`. Re-registering the same id updates the existing record (reconnect). Two different processes MUST NOT reuse the same `node_id` at the same time.

`consumer_id` and `provider_node_id` in `NegotiateChannel` use the same form.

## Virtual hardware: `robonix/prm/...`

Use one of the following layouts per node (do not mix both for the same logical device without documenting it).

### Pattern A — Platform-first (integrated bridge)

One process exposes many interfaces for a single platform (typical for a robot bridge):

```text
robonix/prm/<platform>/<subsystem>/...
```

| Segment | Examples |
|---------|----------|
| `<platform>` | `tiago`, `generic_diff_drive`, `sim_webots` |
| `<subsystem>` | `base`, `nav`, `sensors`, `manipulation` |
| Deeper leaves | `sensors/camera/rgb`, `nav/goals` |

Example: `robonix/prm/tiago` — default for the Tiago/Webots bridge (`ROBONIX_NAMESPACE` in `tiago_node.py`). Tool names (e.g. MCP) stay short; the node carries the tree path via `RegisterNode.namespace`.

### Pattern B — Capability-first (standalone primitive)

One process implements one abstract capability; multiple providers differ by instance id:

```text
robonix/prm/<category>/<capability>/instances/<id>
```

| Segment | Meaning |
|---------|---------|
| `<category>` | e.g. `base`, `sensors`, `arm`, `gripper` |
| `<capability>` | e.g. `move`, `rgb`, `cartesian_pose` |
| `instances/<id>` | `0`, `1`, … or opaque ids — disambiguates multiple implementations of the same abstract capability |

Example: `robonix/prm/base/move/instances/0`

### Disambiguation

1. Narrow the prefix (e.g. down to `.../instances/1`).
2. If more than one node still matches, use `InterfaceInfo.metadata_json` (e.g. `vendor`, `resolution`, `model`) or `provider_node_id` in `NegotiateChannel`.
3. ROS distro / container are orthogonal: use `RegisterNode` metadata (`distro`, `container_id`), not path segments.

## System services: `robonix/sys/...`

Core services must live under `robonix/sys/...`, not under `prm`, so agents can query `namespace` prefix `robonix/sys` without mixing hardware and software services.

Suggested areas (extend by adding new leaves; do not rename published paths):

| Prefix | Services |
|--------|----------|
| `robonix/sys/model/` | Multimodal / chat / embeddings / vision APIs (e.g. VLM). |
| `robonix/sys/map/` | Semantic map query, map manager, map sync. |
| `robonix/sys/planning/` | Task or motion planning facades exposed as nodes. |
| `robonix/sys/manager/` | Skill library, task manager, model manager, … |
| `robonix/sys/debug/` | Ping, introspection, dev-only helpers. |
| `robonix/sys/runtime/` | Long-lived control-plane peers (optional), e.g. agent. |

Examples (node mount points; full abstract IDs add the interface leaf — see [System abstract interfaces](#system-abstract-interfaces-robonix--target-model)):

- `robonix/sys/model/vlm` — mount for a VLM stack; a catalogued abstract might be `robonix/sys/model/vlm/chat`.
- `robonix/sys/map/semantic` — semantic map query (abstract ID may extend with a leaf when defined).
- `robonix/sys/manager/skill_library` — skill registry (future).

Instances: same as `prm` and as described under [System abstract interfaces](#system-abstract-interfaces-robonix--target-model).

### Legacy: `robonix/system/...`

Older examples and docs used `robonix/system/...`. New code SHOULD use `robonix/sys/...` for the same concepts. Migrate registrations when touching a service.

## Runtime environments (orthogonal to the namespace tree)

| Environment | Typical processes | Notes |
|-------------|-------------------|--------|
| `native` | `robonix-server`, `robonix-agent`, `vlm_service` | No ROS requirement; portable across Linux distros. |
| `ros2:<distro>` | Nav2, Webots bridges, `tiago_node` when it uses `rclpy` | Run in a container or host that sources that distro; Robonix core does not assume Ubuntu. |

Namespace paths do not encode ROS distro; metadata (`distro`, `container_id` on `RegisterNode`) carries that.

## Instruction queue (minimal E2E)

For the smallest loop, user commands are accepted by the agent (interactive stdin). A dedicated task queue service is not required for the reference flow; future work may add a queue with stable IDs without changing these namespace paths.

## VLM-facing data

Images and short JSON summaries returned by tools SHOULD be self-describing (`frame`, `encoding`, error fields) so a VLM can reason without ROS knowledge.

## `robonix-agent` defaults (transitional)

Target: resolve providers by catalogued abstract interface IDs and `NegotiateChannel`.

VLM discovery uses `QueryNodesRequest.abstract_interface_id` (default `robonix/sys/model/vlm/chat`) plus transport `grpc`. Override with `ROBONIX_VLM_ABSTRACT_INTERFACE_ID`. For legacy split discovery (`namespace` prefix + interface leaf), set `ROBONIX_VLM_ABSTRACT_INTERFACE_ID` to empty and configure `ROBONIX_VLM_NAMESPACE_PREFIX` (see `robonix-agent` `vlm.rs`).

| Discovery | Mechanism | Env override |
|-----------|-----------|--------------|
| VLM / LLM | `abstract_interface_id` + `grpc` | `ROBONIX_VLM_ABSTRACT_INTERFACE_ID`, or empty abstract + `ROBONIX_VLM_NAMESPACE_PREFIX` |
| MCP tools | `namespace` prefix + `mcp` | `ROBONIX_MCP_NAMESPACE_PREFIX` (empty = no prefix) |

## Concrete examples and PoC

Abstract capability to `QueryNodes` / `NegotiateChannel` to concrete endpoint is walked through in [POC.md](./POC.md) and `examples/scripts/hal_discovery_poc.py`.

Quick filters: `NAMESPACE_PREFIX=robonix/prm` or `NAMESPACE_PREFIX=robonix/sys`; or `ABSTRACT_INTERFACE_ID=robonix/sys/model/vlm/chat` in `hal_discovery_poc.py`.
