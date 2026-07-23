# `system/scene` — live semantic + geometric map

Robonix system service that maintains the **current best estimate** of what's in the robot's environment: per-`Object` records (stable id, fused pose, class, confidence, last_seen, point cloud), pairwise `Relation`s (`on` / `inside` / `near` / `reachable_by`), and a projected 2D occupancy grid from the mapping service. Exposes read-only MCP tools that Pilot calls during RTDL planning/execution, and a self-contained 2D + 3D web viewer.

This service is **NOT** a memory store (long-term recall belongs to memory services) and **NOT** a hardware controller. Current-state only. Reads observations; never writes back to the robot.

## What's inside

```
system/scene/
├── README.md                    ← this file
├── package_manifest.yaml        Robonix package: scene MCP capabilities
├── docker/
│   ├── Dockerfile               ROS Humble + torch cu124 + CV stack
│   ├── requirements.txt         scene + concept-graphs deps
│   ├── _weights/                pre-fetched YOLO, MobileSAM + OpenCLIP weights
│   └── entrypoint.sh            container runtime setup, including Zenoh RMW config
├── scripts/
│   ├── build.sh                 rbnx codegen + docker build (+ weight pre-fetch)
│   └── start.sh                 docker run wrapper used by `rbnx boot`
├── scene_service/
│   ├── service.py               entrypoint: atlas register + asyncio + FastMCP
│   ├── mcp_tools.py             5 @mcp_contract handlers (thin wrappers)
│   ├── web.py                   2D + 3D web viewer (Starlette + three.js)
│   ├── state/                   ObjectRegistry, data assoc, snapshot
│   ├── scene_graph/             relations: fast geometric loop (reachable_by) + image-grounded VLM (image_relations.py) + store
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
3. **Lift to 3D.** Mask-aware depth backprojection through pinhole intrinsics gives a per-detection point cloud in camera-optical frame. The intrinsics `K` come **only** from the atlas-resolved `primitive/camera/intrinsics` contract — the real per-deployment camera. There is no hardcoded-default fallback: guessing `K` scales every point by `fx/fy` and silently misplaces objects, so when no usable intrinsics are wired the detector *waits* (logs `waiting for camera intrinsics`) instead. The world transform comes from the robot's TF tree first: Scene asks for `T(world ← selected_camera_optical)` so the full URDF/static chain and SLAM's `map → odom` correction stay intact. Only when that TF lookup is unavailable does the compatibility path compose `T(world ← body)` from `service/map/pose` (or odom) with `T(body ← camera_optical)` from `primitive/camera/extrinsics`. The selected pose source determines `body`; the extrinsics parent must match that same frame. The world frame name is whatever `header.frame_id` the localizer publishes — never a hardcoded `"map"` once a sample exists.
4. **Match + merge.** Per-detection 512-d OpenCLIP ViT-B-32 image feature + 3D-AABB IoU drives the concept-graphs merge pipeline: `compute_spatial_similarities` + `compute_visual_similarities` + `aggregate_similarities` + `merge_detections_to_objects`. Three hard gates filter the agg_sim matrix:
   * **Distance gate** — centroid > 1.5 m apart → never merge (kills "9 × 5 m bbox spanning the room" failure).
   * **Same-class gate** — different YOLO class names → never merge (kills "potted_plant on cabinet collapses to one record").
   * **Threshold** — agg_sim < 0.55 → spawn new object instead.
5. **Project to registry.** A persistent `MapObjectList.uuid → ObjectRegistry.object_id` cache keeps registry IDs stable across ticks. Bounding boxes are yaw-only (numpy 2D PCA on the XY footprint, no Open3D OBB — `get_oriented_bounding_box(robust=True)` segfaults qhull on near-coplanar pcds), with 5–95 percentile extents to ignore depth-spike outliers.

Periodic cleanup (every 30 ticks) runs concept-graphs's `denoise_objects` + `filter_objects` + `merge_overlap_objects` so duplicates from edge-case detections eventually collapse.

## Deployment targets (x86 / Jetson)

Unlike the Rust system binaries (atlas / executor / pilot / liaison — one
static binary, architecture-agnostic), scene ships a heavy Python + CUDA
perception stack, so it is **platform-specific at both build and run time**.
One repo covers three targets, picked by the per-target package manifest
(`rbnx deploy` selects it via the deploy entry's `manifest:` field):

| Target | manifest | torch source | how it runs |
|---|---|---|---|
| **x86-docker** (default) | `package_manifest.yaml` | cu128 x86 wheels baked into `docker/Dockerfile` | `docker run --gpus all` |
| **jetson-docker** | `package_manifest.jetson-docker.yaml` | NVIDIA jetson-ai-lab wheels in `docker/Dockerfile.jetson` | `docker run --runtime nvidia` |
| **jetson-native** | `package_manifest.jetson-native.yaml` | **host JetPack torch** (no image) | host `python3 -m scene_service.service` |

`scripts/build.sh` branches on `RBNX_BUILD_TARGET`; `scripts/start.sh` branches
on `ROBONIX_SCENE_FORCE` (`native`/`docker`, or auto from
`ROBONIX_SCENE_PLATFORM=jetson_orin`). On most Jetsons **jetson-native is the
recommended path** — it reuses the JetPack CUDA stack instead of building a
multi-GB L4T image.

### Jetson native prerequisites (install the JetPack CUDA stack)

`jetson-native` does **not** install torch — it expects a CUDA-capable host
python already has it, and only pip-installs scene's light pure-python deps
(`pip install --user`) on top. On a fresh Jetson (JetPack 6 / L4T r36 /
CUDA 12.6) install the GPU stack first:

```bash
# 1. ROS 2 Humble on the host (apt, from the ROS 2 repo) — scene needs it
#    for tf2 + the message types. (Skip if already installed.)

# 2. JetPack CUDA-enabled torch / torchvision from NVIDIA's Jetson index.
#    The index tag matches your JetPack: jp6/cu126 here; use jp6/cu128 etc.
#    if your JetPack ships a different CUDA. (apt install nvidia-jetpack
#    provides the CUDA toolkit these wheels link against.)
pip install --user --index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 \
    torch torchvision

# 3. perception extras that have native aarch64 wheels
pip install --user ultralytics open3d

# verify CUDA is live before building scene
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
#   → e.g.  2.10.0 True
```

Then build + run scene native (what `rbnx build`/`rbnx boot` do via the
jetson-native manifest):

```bash
RBNX_BUILD_TARGET=jetson-native bash scripts/build.sh   # venv-free; pip --user the light deps
ROBONIX_SCENE_FORCE=native     bash scripts/start.sh    # host python, host RMW
```

> Native scene uses the same ROS 2 RMW configuration as the host process. Set
> `RMW_IMPLEMENTATION` explicitly when debugging transport differences, or use
> the repository default (`rmw_zenoh_cpp`) for normal Robonix deployments. Set
> `ROBONIX_FORCE_CPU=1` to skip CUDA.

## Relations (`scene_graph/`)

Relations are produced **VLM-primary**: an image-grounded VLM owns the relational + semantic edges, geometry keeps only the metric robot-actionable ones. Two layers write disjoint slices of one store (`scene_graph/store.py`), composed on read by `get_snapshot()` — the shape the MCP tools (`get_scene_graph` / `list_relations` / `get_object_context`) and `/api/state` consume:

- **Fast geometric loop** (`geometric_loop.py`, ~3 Hz, always on — even without VLM creds). Emits only **`reachable_by`** (a real 3D gripper→object distance the robot acts on). `near` is not an edge (proximity is served by `get_object_context.nearby_objects`). Contact/containment is **no longer** computed here: the old AABB tests misfired on full-volume boxes (couldn't fire "monitor on desk") and same-surface objects (invented "monitor on_top_of keyboard"). A debounce hysteresis band absorbs EMA pose jitter.
- **Image-grounded VLM** (`builder.py` + `image_relations.py`, ~30 s, gated by `SCENE_GRAPH_ENABLED`). Each rebuild projects the tracked objects into the current RGB keyframe (via the perception detector's `latest_frame_bundle()` — `K` + camera→map transform), draws numbered boxes, and asks the VLM in **one multimodal call** to enumerate the relations among the numbered objects (`on_top_of` / `under` / `inside` / `contains` / `attached_to` / `part_of` / `same_object`); box numbers map back to `object_id`. Reuses the env `VLM_MODEL`/`VLM_REASONING_EFFORT`. Edge hysteresis carries prior edges across a transient empty/failed round.

Why image-grounded: contact/containment is exactly where coordinate-only reasoning fails (perspective, full-volume bboxes), and a VLM reads it holistically from the image. When no camera frame bundle is available (e.g. the visual-tier VLM detector, which has no camera→map transform), the builder **falls back** to the legacy text-only per-pair inference (`SCENE_GRAPH_IMAGE_RELATIONS=false` forces this path). **Object identity must be stable for any of this to be useful** — see the merge/identity knobs below.

## Build + run

### Canonical Webots demo (sim + full stack)

```bash
git clone --recursive https://github.com/syswonder/robonix
cd robonix/rust && make install      # rbnx + atlas + pilot + executor + codegen → ~/.cargo/bin

# Build the scene image once. Pulls torch+cu124 wheels and the
# concept-graphs source; pre-fetches YOLO-World + MobileSAM .pt to
# docker/_weights/ so the docker layer stays cache-friendly. This includes the
# OpenCLIP checkpoint, avoiding Hugging Face metadata calls inside BuildKit.
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

Within ~15 s of the explore goal landing you should see: the robot moving in rviz / Webots; 2D occupancy updating (web UI left panel); objects accumulating in the 3D panel and the scene registry (`list_objects` / `get_scene_graph` via `rbnx tools`); live RGB + depth in the cam panel (third column).

### Just scene, attached to your own webots/sim

```bash
cd system/scene
bash scripts/build.sh                # one-time
bash scripts/start.sh                # docker run, stays foreground
```

Scene auto-discovers data providers from Atlas. At startup it walks the fixed
contract list in `scene_service/service.py` (`_SCENE_CONTRACTS`), asks Atlas for
providers that implement those contracts, connects to their ROS 2 `topic_out`
interfaces, and subscribes only to what the deployment actually exposes.

For a robot with more than one camera, pin Scene to one RGB-D provider in the
deployment manifest:

```yaml
system:
  scene:
    manifest: package_manifest.jetson-native.yaml
    config:
      camera_provider_id: front_rgbd_camera
      web_port: 50107
```

The value is the camera package entry's `name` (and therefore its Atlas
provider id). Scene resolves `rgb`, `depth`, `intrinsics`, and `extrinsics`
from that same provider so streams and calibration from different cameras are
never mixed. The selected provider must expose both `camera/rgb` and
`camera/depth` for RGB-D perception. Omitting `camera_provider_id` preserves
the legacy auto-discovery behaviour for single-camera deployments.

Scene supports lifecycle initialization, activation, and clean shutdown.
`CMD_DEACTIVATE` is not yet a pause operation: Scene rejects it as deferred and
remains `ACTIVE`, because its ROS subscriptions, perception workers, and Web UI
cannot currently be paused atomically. Use `CMD_SHUTDOWN` to stop Scene. A
successful shutdown reply is sent only after perception, ROS, graph/background
tasks, the Web UI serving task/socket, and the object store have all closed, so
an immediate restart can rebind the same web port.

The default RMW is Zenoh (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`). For single-host
deployments the default local router/session is used; advanced deployments can
override it with `ROBONIX_ZENOH_ROUTER`, `ROBONIX_ZENOH_MODE`, and
`ROBONIX_ZENOH_LISTEN`. Fast DDS can still be tested by setting
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, but it is not the default path.

Useful input contracts include:

* `robonix/primitive/camera/rgb` (`sensor_msgs/Image`)
* `robonix/primitive/camera/depth` (`sensor_msgs/Image`, 32FC1 metres or 16UC1 mm)
* `robonix/primitive/camera/intrinsics` (`sensor_msgs/CameraInfo`)
* `robonix/primitive/camera/extrinsics` (`geometry_msgs/TransformStamped`)
* `robonix/service/map/pose` (`geometry_msgs/PoseWithCovarianceStamped`)
* `robonix/service/map/odom` (`nav_msgs/Odometry`, compatibility fallback)
* `robonix/service/map/occupancy_grid` (`nav_msgs/OccupancyGrid`)

Scene's robot marker and self context come primarily from
`robonix/service/map/pose`, not from the camera and not from raw chassis odom.
The mapping/localization provider must publish a globally corrected
`PoseWithCovarianceStamped` for the configured robot body (`base_link` by
default); Scene adopts its
`header.frame_id` as the world frame. `service/map/odom` and TF2 are
not equivalent pose sources: TF is authoritative, corrected pose is the first
contract fallback, and raw odometry is the final contract fallback. Metric
RGB-D projection resolves the selected camera
frame directly through TF first. Only when that lookup is unavailable does
Scene compose the pose with `primitive/camera/extrinsics`. In that fallback
message, `header.frame_id` must be the expected robot body frame (`base_link`
by default), `child_frame_id` must equal the selected camera optical frame, and
`transform` is `T(parent ← child)`: it
converts points from the camera optical frame into the parent frame. Scene
rejects missing or mismatched frame ids instead of composing incompatible
frames. Deployments with a complete URDF and TF tree should not publish a
second transform with the opposite direction.

For robots whose body frame is not `base_link`, set `base_frame` under
`system.scene.config` (or `SCENE_BASE_FRAME`). This is an assertion about the
body represented by the fallback pose and the required parent of camera
extrinsics. If odometry is selected, its non-empty `child_frame_id` must match
the assertion or that sample is rejected. When `base_frame` is omitted, a
preferred `PoseWithCovarianceStamped` represents `base_link`; an odometry-only
fallback uses that selected message's own `child_frame_id` (then `base_link` if
the field is empty). An unselected odometry sample never changes the frame used
for a preferred pose sample.

If your camera frame is not the deployment default, override it when starting scene:

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

### Offline-friendly Docker frontend and model cache

Scene uses the Docker/BuildKit daemon's bundled Dockerfile frontend by
default, so a cached rebuild does not need to query the `docker/dockerfile`
registry tag. Builders that require a pinned or mirrored frontend can opt in:

```bash
ROBONIX_SCENE_BUILDKIT_SYNTAX=registry.example/docker/dockerfile:1.7@sha256:<digest> \
    bash scripts/build.sh
```

Model files are downloaded on the host with retries, staged under the ignored
`docker/_weights/` build context, and reused through
`ROBONIX_MODEL_CACHE_DIR`. OpenCLIP uses the staged
`open_clip_pytorch_model.bin` directly instead of resolving a symbolic model
name through the Hugging Face metadata API. `RBNX_HF_MIRROR` selects the
Hugging Face mirror endpoint (default `https://hf-mirror.com`); the canonical
`https://huggingface.co` file URL is always tried as a fallback. Set
`RBNX_HF_MIRROR=` to skip the mirror.

## Configuration knobs (env vars)

| Env | Default | Notes |
|---|---|---|
| `SCENE_OPEN_VOCAB_CLASSES` | (55-entry default) | comma-separated YOLO-World class list |
| `SCENE_CG_FORCE_CPU` | `` | set to `1` to force CPU mode (~3× slower) |
| `SCENE_PERCEPTION_WAIT_S` | `30` | how long to wait for camera providers before falling back |
| `SCENE_YOLO_WORLD_WEIGHTS` | `/opt/models/yolov8l-world.pt` | path inside container |
| `SCENE_MOBILE_SAM_WEIGHTS` | `/opt/models/mobile_sam.pt` | |
| `SCENE_CLIP_MODEL` / `SCENE_CLIP_PRETRAINED` | `ViT-B-32` / staged `open_clip_pytorch_model.bin` | Local checkpoint; build.sh downloads it before Docker/native build. |
| `SCENE_CG_MERGE_THRESHOLD` | `0.55` | per-tick merge threshold |
| `SCENE_CG_MAX_MERGE_DIST_M` | `1.5` | hard distance gate |
| `SCENE_CG_OBJ_MIN_POINTS` | `20` | periodic-cleanup cull gate; raise to drop sparse/thin objects, lower to keep them (thin objects like keyboards backproject to sparse clouds) |
| `SCENE_CG_CROSS_CLASS_CENTROID_MAX_M` | `0.5` | per-tick class-gate bypass radius: a detection within this of an existing object may merge despite a different class label (handles YOLO label flicker on one fixture) |
| `SCENE_CG_CROSS_CLASS_IOU_THRESH` / `SCENE_CG_CROSS_CLASS_OVERLAP_THRESH` | `0.30` / `0.50` | periodic class-agnostic collapse: fold two records when AABB IoU ≥ first **or** one-inside-other overlap ≥ second, regardless of class/visual sim. Lower to be more aggressive on a flickering desk (`chair` vs `table` split) |
| `SCENE_CG_MERGE_OVERLAP_THRESH` / `SCENE_CG_MERGE_VISUAL_SIM_THRESH` | `0.50` / `0.65` | periodic `merge_overlap` pass: fold pairs with pcd-overlap ≥ first **and** CLIP cosine ≥ second |
| `SCENE_CG_SAME_CLASS_MERGE_DIST_M` | `0.4` | lenient dedup: fold two SAME-class (or same `SCENE_CG_MERGE_CLASS_GROUPS` bucket) records whose centroids are within this distance, regardless of visual sim (kills "one keyboard → three"). `0` disables |
| `SCENE_CG_MERGE_CLASS_GROUPS` | `` | opt-in confusable-class reconciliation, e.g. `chair,table,desk;sofa,couch` — listed classes share one merge bucket so label flicker across the group collapses while distinct, distant objects stay separate. Empty = off (never relabels) |
| `SCENE_OBJECT_TTL_SEC` | `30` | how long a soft-evicted (`missing`) object is kept so a re-detection can re-bind its id + observation_count before it is hard-pruned; decouples object identity from per-tick uuid churn |
| `SCENE_GRAPH_IMAGE_RELATIONS` | `true` | VLM-primary relations: one image-grounded VLM call (projected numbered boxes) owns relational + semantic edges. `false` forces the legacy text-only per-pair inference (also the automatic fallback when no camera frame bundle is available) |
| `SCENE_GRAPH_IMAGE_MAX_DIM` | `960` | longest-side pixel cap for the annotated frame sent to the VLM; bounds image token cost |
| `SCENE_PORT` / `SCENE_WEB_PORT` | `50106` / `50107` | gRPC + web UI ports |
| `SCENE_WEB_HOST` | `0.0.0.0` | Web UI bind host; set `127.0.0.1` on a robot/control workstation to keep the operator surface local-only. An explicit Scene config file's `web_host` takes precedence when that launch path provides one. |
| `SCENE_OBJECT_MEMORY_ENABLED` | `true` | enable the object snapshot DB backing the map UI's Save/Load (boot warm-restore only under `SCENE_RESTORE_ON_START`) |
| `SCENE_OBJECT_MEMORY_DB` | `/data/robonix/scene_memory/objects.db` | milvus-lite DB path (inside container; host-mounted via `rbnx-build/data/robonix`) |
| `SCENE_MAP_ID` | `default` | FALLBACK map binding: mapping's latched `robonix/service/map/lifecycle` broadcast wins when present at startup; this env (below manifest `map_id`) applies when mapping isn't up yet (normal full-boot order) or doesn't broadcast |
| `SCENE_MAP_BINDING_WAIT_S` | `3.0` | how long the startup probe waits for the lifecycle contract to appear on atlas before falling back to static binding; `0` disables the probe |
| `SCENE_ANNOTATIONS_DIR` | `/data/robonix/scene_annotations` | per-map JSON files holding user annotations (rooms / POIs); host-mounted like the object DB |
| `VLM_REASONING_EFFORT` | `` (unset) | opt-in, forwarded to all scene VLM/LLM calls (relation inference + VLM perception): `minimal`\|`low`\|`medium`\|`high`. **Unset → the field is omitted**, so non-reasoning models and strict endpoints are unaffected. Set `minimal` (= no thinking) to keep a reasoning `VLM_MODEL` (e.g. `doubao-seed-2-1-pro`) answering in ~2 s instead of timing out |

## Object memory (Save/Load snapshots)

When `SCENE_OBJECT_MEMORY_ENABLED` is on, scene keeps an embedded milvus-lite
DB (`SCENE_OBJECT_MEMORY_DB`) for its stable objects. By default a boot starts
a **fresh live session** that is never persisted; objects reach the DB only
when the operator **saves a map** in the map UI, which snapshots the live
registry together with the spatial artifact, and come back only when that map
is **loaded** (see "Map library" below for the epoch rules). Each row carries
a caption vector embedded with the open_clip text encoder already loaded for
perception (512-d, shared with the per-object image features), so a future
object-search layer can reuse the table. The DB is scene-owned — a separate
file/process from `system/memory`'s memsearch DB — and lives under the
host-mounted `/data/robonix`, which also makes the scene-graph JSON caches
survive boots.

Persistence is partitioned by the map binding: an object's pose is only
meaningful in the `map` frame of the SLAM map it was observed on, so restore
loads exactly the current map's objects and never mixes two maps. The same
`object_id` may exist on different maps without colliding.

The binding itself (`scene_service/map_binding.py`) is learned at startup with
this precedence: mapping's latched `robonix/service/map/lifecycle` broadcast
(`{map_id, mode, generation}` — the authoritative map identity, probed for
`SCENE_MAP_BINDING_WAIT_S`) → manifest `config.map_id` → `SCENE_MAP_ID` env →
`"default"`. The broadcast needs the generated `map` interface package
(`rbnx codegen --ros2` → colcon overlay, built by `scripts/build.sh`, sourced
by the container entrypoint); without it scene falls back to static binding
with a warning. At runtime scene only WATCHES the broadcast: if mapping's
identity or `generation` (bumped when the map origin may have changed: reset /
mapping-mode session start) drifts from the startup binding, scene logs a
warning and keeps its binding — reacting (flush + re-anchor) is the planned
lifecycle linkage (P3).

## User annotations (rooms / POIs)

Scene also stores **user-authored** semantics: annotations the user draws on
the map canvas — a `room` (named polygon) or a `poi` (named point, optional
heading; model-ready, no UI yet). They live on the same foundation as
perceived objects: coordinates are map-frame meters, storage partitions by
the map binding's `map_id`, and validity follows mapping's `generation`
epoch. Unlike perceived objects they are user assets: a map rebuild (reset /
re-mapping) never deletes them — they are flagged `stale` (with a reason)
for the user to confirm ("still valid" clears the flag) or redraw (new
geometry is authored against the current frame, so redrawing also clears it).

Storage is one JSON file per map under `SCENE_ANNOTATIONS_DIR`
(`<map_id>.json`, atomic writes, no vectors — deliberately not in the milvus
object DB). CRUD goes through the web server's REST API (same trust domain
as the rest of this LAN debug/UI server — no auth):

| Route | Body | Returns |
|---|---|---|
| `GET /api/annotations` | — | `{ok, annotations: [...]}` |
| `POST /api/annotations` | `{kind, name, points, theta?}` | `{ok, annotation}` (with generated id) |
| `PUT /api/annotations/{id}` | any of `{name, points, theta, stale: false}` | `{ok, annotation}` |
| `DELETE /api/annotations/{id}` | — | `{ok}` |

Errors: `400` invalid fields (unknown kind, room polygon < 3 points, poi ≠ 1
point, non-finite numbers, `theta` on a room — headings are poi-only), `404`
unknown id, `503` store unavailable — those carry `{ok: false, detail}`; a
store write failure (e.g. disk full) surfaces as a plain `500` because the
edit was not saved. On `PUT`, `theta: null` (or absent) means "keep the
current heading" — a set heading can be changed but not cleared (deliberate
until the poi UI lands). `points` are `[[x, y], ...]` map-frame
meters. **This shape is the contract any frontend builds on; treat changes
as breaking.** The full annotation list also rides along in `GET /api/state`
(field `annotations`, next to `map_binding`) so map pages get everything in
one poll. There is deliberately no atlas/MCP surface yet — exposing rooms to
Pilot (scene-graph `in_room` edges) is a planned follow-up.

## Capabilities exposed

| Contract                                       | Tool name        | What it does                                                        |
|------------------------------------------------|------------------|---------------------------------------------------------------------|
| `robonix/system/scene/list_objects`            | `list_objects`   | Flat list of every currently-tracked object (id, label, x,y,z, last_seen). LLM filters client-side. |
| `robonix/system/scene/goal_near`               | `goal_near`      | Map-frame approach pose near a registered object (id -> reachable + x + y + yaw + reason). Pass to `navigation/navigate`. |
| `robonix/system/scene/get_scene_graph`         | `get_scene_graph` | Current semantic graph nodes and relation edges. |
| `robonix/system/scene/get_object_context`      | `get_object_context` | One object's graph context plus nearby objects and directly related edges. |
| `robonix/system/scene/list_relations`          | `list_relations` | Relation edges, optionally filtered by relation type. |

These are MCP-only (transport=mcp). Schemas auto-derive from the IDL via `robonix-api`'s `@mcp_contract`. Example:

```bash
curl -s http://127.0.0.1:50106/mcp/ -H "Content-Type: application/json" \
     -H "Accept: application/json, text/event-stream" \
     -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{
         "name":"list_objects","arguments":{}}}'
```

## Web UI quick tour

* `/` — combined 3-column layout (default): 2D map · 3D scene · cam stack
* `/user` — **end-user map page**: SLAM map + room annotations (draw a polygon, name it, rename / delete / confirm-stale) with a lightweight object overlay; the debug pages above are untouched by it
* `/2d` — only the 2D top-down map
* `/3d` — only the 3D point clouds + bboxes
* `/cam` — only the camera stack: live RGB on top, live depth below
* `/api/state` — JSON: registry + relations + occupancy PNG + robot pose + user annotations + map binding
* `/api/objects3d` — JSON: per-object pcd + 8 bbox corners + CLIP class
* `/api/camera` — JSON: latest RGB + depth as base64 PNGs (5 Hz polling)
* `/api/annotations` — user annotation CRUD (see "User annotations" above)

The 3D viz draws the OccupancyGrid as a translucent floor plane at z = -0.01 (so you can read room geometry under the point clouds), each detected object as a coloured pcd + yaw-rotated wireframe bbox + class label sprite, and the robot as a composite Tiago-shaped proxy (mobile base + torso + shoulder + head + arm), all parented to a `THREE.Group` that updates from `/api/state`'s `robot` field at 4 Hz.

The cam panel shows the same RGB + depth frames the perception pipeline consumes. If detections look wrong, compare them to this feed. Depth is shown as a per-frame normalised grayscale (near = bright). Each tile shows the encoding + age of the latest sample; the meta line turns red once a stream has been silent for >2 s.

## Troubleshooting

**`/api/state` returns 500 with "Out of range float values are not JSON compliant"** — depth backprojection produced NaN/Inf. Should be caught by the snapshot finite-mask + final-guard; if it still hits, the camera's `K` may be wrong. Check the `primitive/camera/intrinsics` publisher (the `[scene] camera intrinsics from contract: …` startup log shows the K scene actually received).

**Scene container exits with status 139 (SIGSEGV)** — was the Open3D `get_oriented_bounding_box(robust=True)` qhull bug; replaced with numpy PCA. If you still see it, `faulthandler.enable(all_threads=True)` (already on in `service.py`) prints the C trace to docker logs.

**Robot dot in web UI doesn't match rviz** — was the `/odom` vs. `map` frame mismatch; fixed by reading tf2 directly. If still off, `docker exec robonix_tiago_sim ros2 run tf2_ros tf2_echo map base_link` should match the web UI's `robot` field exactly.

**Lots of duplicate objects across the room ("ghosting")** — lower `SCENE_CG_MERGE_THRESHOLD` (default 0.55). Or raise `SCENE_CG_MAX_MERGE_DIST_M` if you have very large objects (e.g. big tables) that span >1.5 m.

**One physical object shows as several same-class records** (e.g. one keyboard → three) — raise `SCENE_CG_SAME_CLASS_MERGE_DIST_M` so the lenient same-class proximity collapse folds them; `0` disables it.

**One fixture flickers between two class labels and splits into two records** (e.g. a desk as both `chair` and `table`) — lower `SCENE_CG_CROSS_CLASS_IOU_THRESH` / `SCENE_CG_CROSS_CLASS_OVERLAP_THRESH` so the class-agnostic collapse merges them, or set `SCENE_CG_MERGE_CLASS_GROUPS=chair,table,desk` to treat those labels as one merge bucket.

**Object set collapses (e.g. 9 → 1) within minutes** — a transient cleanup cull used to hard-delete records and reset `observation_count`. Records are now soft-evicted (`missing`) and re-bound by class+pose on re-detection within `SCENE_OBJECT_TTL_SEC`; raise it if objects briefly leave view longer than 30 s.

**"Desk" detected on the floor** — YOLO-World mask leaked past the object's footprint and the depth points are floor. Floor-noise filter already drops detections of falling-class types if `pcd.z_max < 0.30`; adjust the floor_classes list in `perception_concept_graphs.py` if your robot has a low desk.

**No detections firing** — usually a topic mismatch. Check `docker logs robonix_scene` for `auto-discover 'rgb' / 'depth'` lines; if missing, scene didn't find a cap on atlas advertising `robonix/primitive/camera/rgb` over ROS2. `rbnx caps` should list your camera primitive.

## Scope (what's NOT here)

- **No write contract.** No `IngestObservation`, no `UpdateTaskContext` on atlas/MCP — perception-wise, scene is a sink. (The web server's annotation REST is an internal UI API, not a capability contract.)
- **No subscribe-stream.** `SubscribeUpdates` doesn't fit MCP semantics. Pilot polls.
- **No episodic memory.** Long-term memory belongs to memory services, not scene.
- **No direct motion control.** `goal_near` returns an approach pose; navigation is performed by `robonix/service/navigation/navigate`.
- **No real Tiago URDF in the 3D viz.** The composite primitive proxy is good enough; PAL's `tiago_description` xacro chain is too heavy to ship into the browser. STLs are pre-staged under `static/urdf/meshes/` if anyone wants to wire urdf-loader-three.js.
