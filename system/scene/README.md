# `system/scene` — live semantic + geometric map

Robonix system service that maintains the **current best estimate** of
what's in the robot's environment. Per-`Object` records (stable id,
fused pose, class, confidence, last_seen), pairwise `Relation`s
(`on` / `inside` / `near` / `reachable_by`), and a planar surface
layer for "things sit on top of things" reasoning. Exposes 5 read-only
MCP tools that Pilot calls each LLM round.

This service is **NOT** a memory store (see `spatial_memory_service`
for episodic recall) and **NOT** a hardware controller. Current-state
only. Reads observations; never writes back to the robot.

## Layout

```
system/scene/
├── README.md                   ← this file
├── IMPLEMENTATION_NOTES.md     deviations / TODOs / known limits
├── package_manifest.yaml       Robonix package: 5 capabilities
├── pyproject.toml              uv-managed deps (numpy / scipy / open3d / mcp / fastmcp / robonix-py / httpx / Pillow)
├── scripts/
│   └── build.sh                uv venv + uv sync + rbnx codegen
└── scene_service/
    ├── service.py              entrypoint: atlas register + asyncio loop + FastMCP
    ├── mcp_tools.py            5 @mcp_contract handlers (thin wrappers)
    ├── state/                  ObjectRegistry, data assoc, relations, snapshot scoping
    ├── geom/                   pointcloud / plane extraction (Open3D, optional)
    └── ingest/                 atlas-mediated MCP pollers + VLM-based perception
```

## How it works (one paragraph)

`scripts/build.sh` materialises a per-package venv and runs `rbnx
codegen --mcp` to generate Python dataclasses from the IDL under
`rust/crates/robonix-interfaces/lib/service/semantic_map/`. At
startup, `service.py` registers `com.robonix.system.scene` with
atlas, declares 5 MCP interfaces, and (per the Soma config block in
`RBNX_CONFIG_FILE`) launches one async task per observation kind.
Each task calls `atlas.QueryCapabilities + ConnectCapability` for the
configured contract and starts polling — missing caps are silently
skipped, so a Webots Tiago (no 3D lidar) just gets the rgb / lidar2d
/ odom subset without per-deploy code edits.

The RGB stream feeds the **VLM-based perception** module: every few
seconds, scene posts the latest camera frame to the same OpenAI-
compatible endpoint Pilot uses (`VLM_BASE_URL` / `VLM_API_KEY` /
`VLM_MODEL`) with a prompt that asks for a JSON list of detections
with image-pixel bboxes + approximate depth. Detections are
back-projected through pinhole intrinsics + the chassis pose to
world coordinates, then handed to `state/data_assoc.py` for Hungarian
spatial-gated assignment against the existing registry. Matched
detections EMA-update existing objects; unmatched detections allocate
new ones. Unmatched objects stick around for a 5 s grace period, then
flip `missing=True` (records are never deleted).

The relation engine (1 Hz tick) walks the registry and computes
`on / inside / near / reachable_by` — purely geometric in v1, no
learning — caching the triple list for fast serving by `query` and
`get_snapshot`.

## Build + run

```bash
cd system/scene
bash scripts/build.sh              # uv sync + rbnx codegen (a few minutes first time)
```

For the canonical Webots demo, `scene` is declared in
`examples/webots/robonix_manifest.yaml` and brought up alongside
chassis / camera / lidar by `rbnx boot`:

```bash
# T1 — sim (Webots + DDS bus, must be up before scene)
bash examples/webots/sim/start.sh

# T2 — full stack (atlas / pilot / executor / scene / chassis / camera / lidar / nav2)
export VLM_BASE_URL=https://api.openai.com/v1
export VLM_API_KEY=sk-...
export VLM_MODEL=gpt-5.4-mini
cd examples/webots
rbnx boot
```

Then in T3:

```bash
rbnx caps                          # com.robonix.system.scene with 5 declared MCP interfaces
rbnx tools | grep scene            # the 5 tools Pilot can call
rbnx chat                          # chat with pilot; ask about objects in front of the robot
```

Drive the robot around the office; after a few seconds the VLM
detector starts populating the registry. Sample direct probe:

```bash
# Fastest way to see what scene knows: pilot's executor will MCP-call
# scene/get_snapshot for you. From rbnx chat:
> what objects are around me
```

Or hit the MCP HTTP endpoint directly (port from atlas `rbnx caps`):

```bash
curl http://127.0.0.1:50106/mcp/ -H "Content-Type: application/json" \
     -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"get_snapshot","arguments":{"spec":{"layers":["object","relation"],"region_frame":"map","region_center_x":0.0,"region_center_y":0.0,"region_center_z":0.0,"region_radius_m":5.0,"freshness_s":30.0,"include_stale":false,"min_confidence":0.0,"max_objects":50}}}}'
```

## Soma config (which observations to subscribe to)

Default — for Webots Tiago — set in `service.py`'s `_DEFAULT_OBSERVATIONS`:

```yaml
observations:
  - kind: rgb       # contract: robonix/primitive/camera/snapshot, period 3s, drives VLM perception
  - kind: lidar2d   # contract: robonix/primitive/lidar/snapshot,  period 2s
  - kind: odom      # contract: robonix/primitive/chassis/state,   period 1s, drives the robot self-object
```

Override in `examples/<your-deploy>/robonix_manifest.yaml`:

```yaml
system:
  scene:
    observations:
      - kind: rgb
        contract: robonix/primitive/camera/snapshot
        period_s: 2.0
      - kind: lidar3d
        contract: robonix/primitive/lidar/lidar3d
        period_s: 1.0
      - kind: depth
        contract: robonix/primitive/camera/depth_snapshot
        period_s: 3.0
      - kind: odom
        contract: robonix/primitive/chassis/state
        period_s: 1.0
```

`kind` is just a label for the log; the contract is what scene
actually queries on atlas. Missing caps log a one-line "skipped" and
don't block startup.

## Capabilities exposed

| Contract                                       | Tool name                | What it does                            |
|------------------------------------------------|--------------------------|------------------------------------------|
| `robonix/system/scene/get_snapshot`            | `get_snapshot`           | Region- and freshness-scoped read; cap'd |
| `robonix/system/scene/query`                   | `query`                  | Class + relation + spatial filter        |
| `robonix/system/scene/get_object`              | `get_object`             | Direct lookup by stable id               |
| `robonix/system/scene/get_semantic_map`        | `get_semantic_map`       | Surfaces + ref to Nav2 occupancy grid    |
| `robonix/system/scene/get_safety_context`      | `get_safety_context`     | Stub (Sentinel placeholder)              |

All 5 are MCP-only (transport=mcp). Schemas auto-derive from the IDL
via `robonix-py`'s `@mcp_contract`.

## Scope (what's NOT here)

- **No write API**. No `IngestObservation`, no `UpdateTaskContext`. Per the
  spec, scene is a sink. If you want to push state into it, that's a
  v2 design decision — open an issue.
- **No subscribe-stream**. `SubscribeUpdates` doesn't fit MCP semantics.
  Pilot polls.
- **No episodic memory**. `record_episode` / time-range queries belong
  to `spatial_memory_service`.
- **No real reachability**. `reachable_by` is a distance stub. TODO
  swap to Soma's kinematics adapter when it lands.
- **No Sentinel**. `get_safety_context` always returns
  `status="not_implemented"` so Pilot can call without exception.

See `IMPLEMENTATION_NOTES.md` for everything else.
