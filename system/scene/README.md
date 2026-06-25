# `system/scene` — live semantic + geometric map

Robonix system service that maintains the **current best estimate** of what's in the robot's environment: per-`Object` records (stable id, fused pose, class, confidence, last_seen, point cloud), pairwise `Relation`s (`on` / `inside` / `near` / `reachable_by`), and a projected 2D occupancy grid republished from the SLAM service. Exposes 5 read-only MCP tools that Pilot calls each LLM round, and a self-contained 2D + 3D web viewer.

This service is **NOT** a memory store (see `spatial_memory_service` for episodic recall) and **NOT** a hardware controller. Current-state only. Reads observations; never writes back to the robot.

## What's inside

```
system/scene/
├── README.md                    ← this file
├── package_manifest.yaml        Robonix package: 5 MCP capabilities
├── docker/
│   ├── Dockerfile               ROS Humble + torch cu124 + CV stack
│   ├── requirements.txt         scene + concept-graphs deps
│   ├── _weights/                pre-fetched YOLO-World + MobileSAM .pt
│   └── no_shm_profile.xml       FastRTPS UDP-only profile
├── scripts/
│   ├── build.sh                 rbnx codegen + docker build (+ weight pre-fetch)
│   └── start.sh                 docker run wrapper used by `rbnx boot`
├── scene_service/
│   ├── service.py               entrypoint: atlas register + asyncio + FastMCP
│   ├── mcp_tools.py             5 @mcp_contract handlers (thin wrappers)
│   ├── web.py                   2D + 3D web viewer (Starlette + three.js)
│   ├── state/                   ObjectRegistry, data assoc, relations, snapshot
│   ├── geom/                    pointcloud / plane extraction (Open3D)
│   ├── ingest/
│   │   ├── ros_subscribers.py   rclpy hub: /tf2 + topic slots (rgb/depth/lidar/...)
│   │   ├── capabilities.py      hardware probe → perception tier (metric/visual/geometric)
│   │   ├── perception_concept_graphs.py  perception pipeline (this file)
│   │   └── perception_vlm.py    VLM fallback (no-depth deploys only)
│   └── static/urdf/meshes/      Tiago STL meshes (kept around for future URDF viz)
```

## What it actually does

**Perception tier.** Which pipeline runs is decided at startup by `ingest/capabilities.py` from the wired hardware, and logged once (`perception plan: tier=… detector=… grounding=… inputs=[…]`):

- **metric** — RGB-D (+ intrinsics + pose) → ConceptGraphs below. Object-level 3D semantics, spatial relations, open-vocab queries.
- **visual** — RGB only → `perception_vlm.py`. Approximate, region-level semantics; positions are coarse (no metric depth back-project).
- **geometric** — no camera (LiDAR / 2D SLAM only) → no detector. The occupancy grid + `goal_near` BFS stay available; object/relation queries return empty.

The metric pipeline is ConceptGraphs-style per-frame perception with 4 stages:

1. **Detect.** YOLO-World v2 (open-vocab via CLIP text encoder) on the live RGB frame. Class list is a 55-entry indoor-office vocabulary; override at runtime via `SCENE_OPEN_VOCAB_CLASSES=cup,chair,...`.
2. **Segment.** MobileSAM, prompted with each YOLO bbox, produces a per-detection mask.
3. **Lift to 3D.** Mask-aware depth backprojection through pinhole intrinsics gives a per-detection point cloud in camera-optical frame. The intrinsics `K` come **only** from the atlas-resolved `primitive/camera/intrinsics` contract — the real per-deployment camera, exactly as extrinsics come from the camera's tf2/URDF rather than a scene-side env var. There is no hardcoded-default fallback: guessing `K` scales every point by `fx/fy` and silently misplaces objects, so when no usable intrinsics are wired the detector *waits* (logs `waiting for camera intrinsics`) instead. The world transform comes from atlas-resolved contracts, not tf2: `T(world ← base_link)` from `service/map/pose` and `T(base_link ← camera_optical)` from `primitive/camera/extrinsics`, composed into a single 4×4. The world frame name is whatever `header.frame_id` the localizer publishes — never a hardcoded `"map"`. tf2 stays only as a last-resort fallback for legacy stacks where the camera primitive hasn't declared `extrinsics` yet (logged once via "no pose contract resolved"). Reading `/odom` directly is **NOT** acceptable — once SLAM corrects `map → odom`, the registry drifts away from rviz.
4. **Match + merge.** Per-detection 512-d OpenCLIP ViT-B-32 image feature + 3D-AABB IoU drives the concept-graphs merge pipeline: `compute_spatial_similarities` + `compute_visual_similarities` + `aggregate_similarities` + `merge_detections_to_objects`. Three hard gates filter the agg_sim matrix:
   * **Distance gate** — centroid > 1.5 m apart → never merge (kills "9 × 5 m bbox spanning the room" failure).
   * **Same-class gate** — different YOLO class names → never merge (kills "potted_plant on cabinet collapses to one record").
   * **Threshold** — agg_sim < 0.55 → spawn new object instead.
5. **Project to registry.** A persistent `MapObjectList.uuid → ObjectRegistry.object_id` cache keeps registry IDs stable across ticks. Bounding boxes are yaw-only (numpy 2D PCA on the XY footprint, no Open3D OBB — `get_oriented_bounding_box(robust=True)` segfaults qhull on near-coplanar pcds), with 5–95 percentile extents to ignore depth-spike outliers.

Periodic cleanup (every 30 ticks) runs concept-graphs's `denoise_objects` + `filter_objects` + `merge_overlap_objects` so duplicates from edge-case detections eventually collapse.

## Build + run

### Canonical Webots demo (sim + full stack)

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix/rust && make install      # rbnx + atlas + pilot + executor + codegen → ~/.cargo/bin

# Build the scene image once. Pulls torch+cu124 wheels and the
# concept-graphs source; pre-fetches YOLO-World + MobileSAM .pt to
# docker/_weights/ so the docker layer stays cache-friendly.
#
# Pick the ROS distro BEFORE this first build (default humble) — see
# "ROS distro" below. e.g.  ROBONIX_SCENE_ROS_DISTRO=jazzy bash scripts/build.sh
cd ../system/scene && bash scripts/build.sh

# T1 — sim. Bring up Webots + chassis driver + camera + lidar in
# their own docker compose stack. The script also auto-launches rviz2
# inside the sim container; if you don't want rviz, edit sim/start.sh.
cd ../../examples/webots && bash sim/start.sh

# T2 — robonix stack. atlas + executor + pilot + 3 primitives +
# scene + mapping (rtabmap) + simple_nav + explore. 11 components up.
export DISPLAY=:0                    # sim's webots/rviz need an X display
rbnx boot
```

When `rbnx boot` finishes (11 components running):

* **Web UI** — http://localhost:50107  (2D occupancy + objects on the left, 3D point clouds + bboxes on the right)
* **rviz2** — already up inside the sim container (map, scan, /tf, /goal_pose). Use the "2D Nav Goal" tool to drive the robot manually via simple_nav.
* **MCP tools** — `rbnx tools | grep scene`

### Drive the robot to populate the map

Use natural language so Pilot runs the explore skill:

```bash
# 3rd terminal, anywhere on the host
rbnx ask "thoroughly explore the entire room, and wait for explore to finish"
```

Pilot calls `skill/explore/explore`, polls `status` every few seconds, and lets rtabmap + scene fill in the map as the robot frontiers across the room. With the default Webots Tiago scene this takes ~3–4 minutes to cover ~6 frontier hops.

For interactive use: `rbnx chat`, type `explore the environment`; Esc cancels current reasoning, `Ctrl+C` exits.

Within ~15 s of the explore goal landing you should see: the robot moving in rviz / Webots; 2D occupancy updating (web UI left panel); objects accumulating in the 3D panel and the scene registry (`get_snapshot` via `rbnx tools`); live RGB + depth in the cam panel (third column).

### Just scene, attached to your own webots/sim

```bash
cd system/scene
bash scripts/build.sh                # one-time
bash scripts/start.sh                # docker run, stays foreground
```

scene's container joins the host DDS bus (`--network host` + FastRTPS UDP-only) and auto-discovers any provider on atlas declaring a ROS2 `topic_out` interface. Required topics:

* RGB image (`sensor_msgs/Image`)
* Depth registered to RGB (`sensor_msgs/Image`, 32FC1 metres or 16UC1 mm)
* `/tf` chain ending at `head_front_camera_rgb_optical_frame` and `base_link` in `map` frame.
* Optional: `nav_msgs/OccupancyGrid` (for the 2D underlay)

If your camera frame isn't `head_front_camera_rgb_optical_frame`, override via env when starting scene:

```bash
SCENE_CAMERA_FRAME=my_camera_optical bash scripts/start.sh
```

## ROS distro

Scene is the **only** Robonix component that consumes ROS topics to get its
data, and it does **not** hard-pin a ROS 2 release. The distro is a
build-time choice; the image base, the `ros-<distro>-*` apt packages, and the
runtime `source /opt/ros/<distro>/setup.bash` all follow one variable.

Supported (these have the `tf2` + `zenoh-bridge-dds` packages scene needs):
**humble** (default, verified), **iron**, **jazzy**, **rolling**.

Pick it with the **`ROBONIX_SCENE_ROS_DISTRO` environment variable**, set
**before the first `rbnx build` / `scripts/build.sh`** — switching distro
means rebuilding the image. `rbnx build` inherits your shell environment, so
the variable reaches `build.sh` either way:

```bash
# via rbnx (the variable propagates through to scene's build.sh):
ROBONIX_SCENE_ROS_DISTRO=jazzy rbnx build -p system/scene

# or building the package directly:
ROBONIX_SCENE_ROS_DISTRO=jazzy bash system/scene/scripts/build.sh
```

The chosen distro is echoed at build time (`[build] ROS distro: …`). At
runtime the container sources that distro's `setup.bash`; nothing else in
scene references a distro. Default builds (variable unset) are unchanged —
plain `humble`.

## Configuration knobs (env vars)

| Env | Default | Notes |
|---|---|---|
| `SCENE_OPEN_VOCAB_CLASSES` | (55-entry default) | comma-separated YOLO-World class list |
| `SCENE_CG_FORCE_CPU` | `` | set to `1` to force CPU mode (~3× slower) |
| `SCENE_PERCEPTION_WAIT_S` | `30` | how long to wait for camera providers before falling back |
| `SCENE_YOLO_WORLD_WEIGHTS` | `/opt/models/yolov8l-world.pt` | path inside container |
| `SCENE_MOBILE_SAM_WEIGHTS` | `/opt/models/mobile_sam.pt` | |
| `SCENE_CLIP_MODEL` / `SCENE_CLIP_PRETRAINED` | `ViT-B-32` / `laion2b_s34b_b79k` | |
| `SCENE_CG_MERGE_THRESHOLD` | `0.55` | per-tick merge threshold |
| `SCENE_CG_MAX_MERGE_DIST_M` | `1.5` | hard distance gate |
| `SCENE_PORT` / `SCENE_WEB_PORT` | `50106` / `50107` | gRPC + web UI ports |
| `SCENE_OBJECT_MEMORY_ENABLED` | `true` | persist stable objects + warm-restore the registry on boot |
| `SCENE_OBJECT_MEMORY_DB` | `/data/robonix/scene_memory/objects.db` | milvus-lite DB path (inside container; host-mounted via `rbnx-build/data/robonix`) |
| `SCENE_MAP_ID` | `default` | SLAM map the persisted objects belong to; restore loads only this map's objects (manifest `map_id` overrides) |

## Object memory (warm restore)

When `SCENE_OBJECT_MEMORY_ENABLED` is on, scene persists its stable objects to a
small embedded milvus-lite DB (`SCENE_OBJECT_MEMORY_DB`) at the scene-graph
builder cadence, and reloads them into the registry at boot. After a restart the
graph is populated immediately instead of re-accumulating every object through
the `min_observations` filter; re-observation re-confirms restored objects in
place (no duplicate ids). Each row carries a caption vector embedded with the
open_clip text encoder already loaded for perception (512-d, shared with the
per-object image features), so a future object-search layer can reuse the table.
The DB is scene-owned — a separate file/process from `system/memory`'s memsearch
DB — and lives under the host-mounted `/data/robonix`, which also makes the
scene-graph JSON caches survive boots. Writes are driven by the scene-graph
builder, so disabling `SCENE_GRAPH_ENABLED` stops new writes (restore still runs).

Persistence is partitioned by `SCENE_MAP_ID` (or the manifest `map_id`): an
object's pose is only meaningful in the `map` frame of the SLAM map it was
observed on, so restore loads exactly the current map's objects and never mixes
two maps. The same `object_id` may exist on different maps without colliding.
`map_id` is a deploy-controlled string today (default `"default"`) and stays
internal to scene — no atlas/MCP contract changes — until mapping emits a real
map identity to wire in here.

## Capabilities exposed

| Contract                                       | Tool name        | What it does                                                        |
|------------------------------------------------|------------------|---------------------------------------------------------------------|
| `robonix/system/scene/list_objects`            | `list_objects`   | Flat list of every currently-tracked object (id, label, x,y,z, last_seen). LLM filters client-side. |
| `robonix/system/scene/goal_near`               | `goal_near`      | Map-frame approach pose near a registered object (id → reachable + x + y + yaw + reason). Pass to `navigation/navigate`. |

Both are MCP-only (transport=mcp). Schemas auto-derive from the IDL via `robonix-api`'s `@mcp_contract`. Example:

```bash
curl -s http://127.0.0.1:50106/mcp/ -H "Content-Type: application/json" \
     -H "Accept: application/json, text/event-stream" \
     -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{
         "name":"get_snapshot","arguments":{"spec":{
             "layers":["object","relation"],"region_frame":"map",
             "region_center_x":0,"region_center_y":0,"region_center_z":0,
             "region_radius_m":5.0,"freshness_s":30.0,
             "include_stale":false,"min_confidence":0.0,
             "max_objects":50}}}}'
```

## Web UI quick tour

* `/` — combined 3-column layout (default): 2D map · 3D scene · cam stack
* `/2d` — only the 2D top-down map
* `/3d` — only the 3D point clouds + bboxes
* `/cam` — only the camera stack: live RGB on top, live depth below
* `/api/state` — JSON: registry + relations + occupancy PNG + robot pose
* `/api/objects3d` — JSON: per-object pcd + 8 bbox corners + CLIP class
* `/api/camera` — JSON: latest RGB + depth as base64 PNGs (5 Hz polling)

The 3D viz draws the OccupancyGrid as a translucent floor plane at z = -0.01 (so you can read room geometry under the point clouds), each detected object as a coloured pcd + yaw-rotated wireframe bbox + class label sprite, and the robot as a composite Tiago-shaped proxy (mobile base + torso + shoulder + head + arm), all parented to a `THREE.Group` that updates from `/api/state`'s `robot` field at 4 Hz.

The cam panel shows the same RGB + depth frames the perception pipeline consumes. If detections look wrong, compare them to this feed. Depth is shown as a per-frame normalised grayscale (near = bright). Each tile shows the encoding + age of the latest sample; the meta line turns red once a stream has been silent for >2 s.

## Troubleshooting

**`/api/state` returns 500 with "Out of range float values are not JSON compliant"** — depth backprojection produced NaN/Inf. Should be caught by the snapshot finite-mask + final-guard; if it still hits, the camera's `K` may be wrong. Check the `primitive/camera/intrinsics` publisher (the `[scene] camera intrinsics from contract: …` startup log shows the K scene actually received).

**Scene container exits with status 139 (SIGSEGV)** — was the Open3D `get_oriented_bounding_box(robust=True)` qhull bug; replaced with numpy PCA. If you still see it, `faulthandler.enable(all_threads=True)` (already on in `service.py`) prints the C trace to docker logs.

**Robot dot in web UI doesn't match rviz** — was the `/odom` vs. `map` frame mismatch; fixed by reading tf2 directly. If still off, `docker exec robonix_tiago_sim ros2 run tf2_ros tf2_echo map base_link` should match the web UI's `robot` field exactly.

**Lots of duplicate objects across the room ("ghosting")** — lower `SCENE_CG_MERGE_THRESHOLD` (default 0.55). Or raise `SCENE_CG_MAX_MERGE_DIST_M` if you have very large objects (e.g. big tables) that span >1.5 m.

**"Desk" detected on the floor** — YOLO-World mask leaked past the object's footprint and the depth points are floor. Floor-noise filter already drops detections of falling-class types if `pcd.z_max < 0.30`; adjust the floor_classes list in `perception_concept_graphs.py` if your robot has a low desk.

**No detections firing** — usually a topic mismatch. Check `docker logs robonix_scene` for `auto-discover 'rgb' / 'depth'` lines; if missing, scene didn't find a cap on atlas advertising `robonix/primitive/camera/rgb` over ROS2. `rbnx caps` should list your camera primitive.

## Scope (what's NOT here)

- **No write API.** No `IngestObservation`, no `UpdateTaskContext`. Per the spec, scene is a sink.
- **No subscribe-stream.** `SubscribeUpdates` doesn't fit MCP semantics. Pilot polls.
- **No episodic memory.** Belongs to `spatial_memory_service`.
- **No real reachability.** `reachable_by` is a distance stub.
- **No Sentinel.** `get_safety_context` always returns `status="not_implemented"`.
- **No real Tiago URDF in the 3D viz.** The composite primitive proxy is good enough; PAL's `tiago_description` xacro chain is too heavy to ship into the browser. STLs are pre-staged under `static/urdf/meshes/` if anyone wants to wire urdf-loader-three.js.
