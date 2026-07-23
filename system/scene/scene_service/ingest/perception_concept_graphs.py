# SPDX-License-Identifier: MulanPSL-2.0
"""ConceptGraphs perception and persistent object-map integration.

Each tick runs the following pipeline:

* YOLO-World detects open-vocabulary objects.
* MobileSAM creates bounding-box-prompted masks.
* OpenCLIP produces one visual feature per detection.
* Depth, masks, camera intrinsics, and pose produce map-frame point clouds.
* Spatial and visual similarities associate detections with stored objects.
* Matched detections update objects; unmatched detections create new objects.

This replaces the earlier class-and-distance-only association path, which
could not use visual features and frequently over-segmented objects.

The persistent `MapObjectList` is the source of truth. Every tick we
project it back into scene's `ObjectRegistry` so the existing web UI /
MCP API keep working without changes.

Periodic cleanup runs `denoise_objects` + `merge_overlap_objects` from
concept-graphs to GC duplicates that escape the per-tick merge.
"""
from __future__ import annotations

import asyncio
import copy
import logging
import math
import os
import threading
import time
import uuid
from typing import Any, Awaitable, Callable, Optional

# numpy is unconditionally imported because (a) the detector loop
# uses it on every tick and (b) the per-method lazy-import convention
# elsewhere in this file kept biting us — every helper that touched
# `np.*` had to remember the import or the whole tick silently bailed
# with NameError. One module-level import is cheap and removes the
# class of bug entirely.
import numpy as np

log = logging.getLogger("scene.ingest.cg")


from .perception_vlm import _CamIntrinsics, _canon_class
from ..state.data_assoc import Detection
from ..state.object_registry import BBox3D, ObjectRegistry, Pose3D, SceneObject


# ── Open-vocab class list ────────────────────────────────────────────────
_DEFAULT_OPEN_VOCAB = [
    "chair", "table", "desk", "couch", "sofa", "bookshelf", "shelf",
    "cabinet", "drawer", "whiteboard",
    "monitor", "laptop", "keyboard", "mouse", "computer tower",
    "monitor stand", "headphones", "webcam", "router", "power strip",
    "cup", "mug", "water bottle", "thermos", "paper cup",
    "backpack", "handbag", "book", "notebook", "pen", "pencil",
    "phone", "tablet",
    "box", "cardboard box", "tray", "basket", "trash bin",
    "tool", "screwdriver", "wrench", "tape",
    "plant", "potted plant", "lamp", "clock", "picture frame",
    "snack", "fruit", "apple", "banana",
    "door", "doorway", "fire extinguisher", "person",
]

_BG_CLASSES = frozenset({"floor", "wall", "ceiling", "carpet"})

# Classes we never want to insert (unstable / outdoor / not useful for indoor robot).
_IGNORED_CLASSES = frozenset({"person"})


def _resolved_classes() -> list[str]:
    raw = os.environ.get("SCENE_OPEN_VOCAB_CLASSES", "").strip()
    if not raw:
        return list(_DEFAULT_OPEN_VOCAB)
    return [s.strip() for s in raw.split(",") if s.strip()]


_DEFAULT_YOLO_WORLD_WEIGHTS = "/opt/models/yolov8l-world.pt"
_DEFAULT_MOBILE_SAM_WEIGHTS = "/opt/models/mobile_sam.pt"
_DEFAULT_CLIP_MODEL = "ViT-B-32"
_DEFAULT_CLIP_PRETRAINED = "/opt/models/open_clip_pytorch_model.bin"


# ── Concept-graphs config knobs ──────────────────────────────────────────
# These are the v0 defaults. They map to concept-graphs's own
# realtime_mapping config; documented per-knob so they're tunable
# without grepping ali-dev's hydra cfg.
_CFG_DEFAULTS = {
    # 3D pcd downsampling (Open3D voxel size, metres). 0.025 = 2.5cm
    # voxels — concept-graphs default for indoor tabletop.
    "downsample_voxel_size": 0.025,
    # DBSCAN params for pcd denoise (drop sparse outlier points).
    "dbscan_remove_noise": True,
    "dbscan_eps": 0.10,                 # 10 cm cluster radius
    "dbscan_min_points": 10,
    # Spatial similarity type. ali-dev exposes:
    #   'iou' / 'giou'                  — axis-aligned bbox IoU
    #   'iou_accurate' / 'giou_accurate' — oriented-bbox IoU
    #   'overlap'                       — voxel-grid pcd intersection
    # 'overlap' is concept-graphs's CANONICAL choice — captures dense
    # pcd similarity so cabinet+shelf at the same position merge even
    # when their AABBs differ. Concept-graphs's reference impl routes
    # through pytorch3d.ops.box3d_overlap, which raises ValueError on
    # coplanar bbox vertices (thin objects on floor) and the Open3D
    # qhull fallback segfaults on degenerate input. We don't use that
    # path: see `_voxel_pcd_overlap_matrix` below — pure numpy
    # voxel-set fraction-overlap, no pytorch3d, no qhull. The
    # `spatial_sim_type` value below is still threaded into
    # concept-graphs internals (merge_detections_to_objects,
    # denoise_objects, merge_overlap_objects); they use it only to
    # decide which similarity to recompute when caller didn't provide
    # one — we always provide our overlap matrix, so 'iou' here is a
    # safe sentinel that avoids touching the broken pytorch3d path.
    "spatial_sim_type": "iou",
    # `match_method`: how to combine spatial + visual sims.
    "match_method": "sim_sum",
    # phys_bias > 0 → trust spatial more than visual; <0 → opposite.
    # 0.0 means equal weight.
    "phys_bias": 0.0,
    # A detection is matched to an existing object only when
    # aggregated similarity >= this. Below it, the detection becomes
    # We're back on `iou` spatial sim (axis-aligned bbox IoU) since
    # the `overlap` path segfaulted via Open3D's qhull. With:
    #   spatial AABB-IoU: 0..1 (often 0 for partial views), visual
    #   cos-sim: -1..1 (typically 0.4-0.85 for same instance).
    #   sim_sum with phys_bias=0 sums them. At threshold=1.10 the
    #   same physical object seen from a new viewpoint (e.g. across
    #   the room) usually has IoU≈0 and visual≈0.6 → 0.60 < 1.10 →
    #   spawned as a NEW object. That's the "ghosting" (duplicate) the
    #   user kept catching.
    #   Lowering to 0.55 was needed when spatial sim was AABB IoU
    #   (which is ≈0 for partial-view bboxes). With voxel pcd-overlap
    #   replacing IoU (`_voxel_pcd_overlap_torch`), spatial sim alone
    #   reliably reaches 0.4–0.7 for "same physical object seen from
    #   another angle" — combined with visual sim that's 1.0–1.3 for
    #   true matches and 0.3–0.7 for spurious pairs, threshold 0.85
    #   is the sweet spot. Lower → over-merging, higher → ghosting again.
    "merge_threshold": 0.85,
    # Hard centroid-distance gate on the per-tick merge. With
    # merge_threshold low, visual similarity alone can match; this
    # caps the physical separation so two "potted_plant" detections
    # at opposite corners of the room never merge into one mega-
    # bbox (the 9 × 5 m blob the user kept seeing). 1.5 m fits "same
    # piece of furniture seen from a couple of angles" but rejects
    # "two of the same kind across the room".
    "max_merge_dist_m": 1.5,
    # Minimum 3D points after backprojection to keep a detection.
    # Filters thin/degenerate masks.
    "min_points_threshold": 50,
    # Per-object pcd cap (downsample points if exceeded). Keeps
    # memory bounded as the same object is observed many times.
    "obj_pcd_max_points": 5000,
    # Periodic cleanup interval (in ticks at ~1.6 Hz). 10 ticks = 6 s
    # cadence. Aggressive enough that cross-class duplicates (e.g.
    # cabinet + shelf labelled on the same physical object in the
    # same frame) get merged before they pollute the registry — the
    # per-tick `merge_detections_to_objects` only matches against the
    # existing map, so two BRAND-NEW detections at the same position
    # both get appended; only the periodic merge_overlap pass can
    # collapse them.
    "denoise_interval_ticks": 10,
    "merge_overlap_interval_ticks": 10,
    # Periodic-cleanup thresholds (passed to merge_overlap_objects /
    # denoise_objects in concept-graphs.utils). obj_min_points is the cull
    # gate inside filter_objects: thin/small objects (keyboard, lamp) backproject
    # to sparse clouds, so 50 culled them every cleanup tick → uuid churn →
    # observation_count reset. 20 lets them survive; override via
    # SCENE_CG_OBJ_MIN_POINTS (env table below).
    "obj_min_points": 20,
    "obj_min_detections": 1,
    # merge_overlap_thresh: spatial pcd-overlap ratio above which the
    # pair is a candidate (0..1). 0.5 = "half of A's volume is inside
    # B" — more aggressive than the canonical 0.7 because at this
    # scale and this voxel size, true duplicates often only share half
    # their voxels (noise + sampling differences).
    "merge_overlap_thresh": 0.5,
    # merge_visual_sim_thresh: required CLIP cosine similarity for the
    # pair to be considered "the same object" once spatial passes.
    # 0.65 catches cabinet↔shelf, monitor↔monitor_stand, etc. that
    # share visual features but get different YOLO-World labels.
    "merge_visual_sim_thresh": 0.65,
    "merge_text_sim_thresh": 0.0,        # we don't compute text feats
    # ── Class-agnostic geometric collapse ─────────────────────────────
    # When YOLO-World flickers between e.g. picture_frame / shelf /
    # cabinet for the same physical wall fixture, the same-class gate
    # below keeps them as 3 separate records. CLIP visual sim is also
    # too low because the cropped masks differ. Two safety nets:
    #
    #   (a) **Per-tick centroid proximity bypass.** When a new det's
    #       centroid is within `cross_class_centroid_max_m` of an
    #       existing object's centroid, we drop the same-class gate
    #       for that pair. Visual+spatial sim still has to clear
    #       merge_threshold.
    #   (b) **Periodic class-agnostic AABB-IoU collapse.** Runs on the
    #       cleanup tick AFTER the canonical merge_overlap_objects
    #       pass. Pairs whose AABB IoU >= cross_class_iou_thresh OR
    #       whose pcd-overlap ratio >= cross_class_overlap_thresh get
    #       force-merged regardless of class/visual sim. This is the
    #       only thing strong enough to undo a 3-way YOLO label split
    #       on a wall fixture.
    "cross_class_centroid_max_m": 0.5,
    "cross_class_iou_thresh": 0.30,
    "cross_class_overlap_thresh": 0.50,
    "cross_class_merge_interval_ticks": 10,
    # ── Same-class lenient proximity merge (dedup) ─────────────────────
    # Folds two objects of the SAME class (or same SCENE_CG_MERGE_CLASS_GROUPS
    # bucket) whose centroids are within this distance into one, regardless of
    # visual sim — targets the "one keyboard becomes three" duplication that the
    # visual-gated merge_overlap pass misses (AABB-IoU≈0 across partial views,
    # CLIP crops diverge enough to fail merge_visual_sim_thresh). 0 disables.
    "same_class_merge_dist_m": 0.4,
    "same_class_merge_interval_ticks": 10,
}


def _parse_merge_class_groups(raw: str) -> dict[str, str]:
    """Parse SCENE_CG_MERGE_CLASS_GROUPS into a {class -> bucket-key} map.

    Groups are separated by ``;``, members within a group by ``,`` — e.g.
    ``"chair,table,desk;sofa,couch"`` maps chair/table/desk to one bucket and
    sofa/couch to another. The bucket key is the sorted member list joined by
    ``|`` (order-independent). Empty / malformed input yields an empty map, so
    the feature is off by default and never silently relabels objects."""
    out: dict[str, str] = {}
    for group in raw.split(";"):
        members = [m.strip().lower() for m in group.split(",") if m.strip()]
        if len(members) < 2:
            continue
        key = "|".join(sorted(set(members)))
        for m in members:
            out[m] = key
    return out


# ── Model loading ────────────────────────────────────────────────────────
def _try_load_models(yolo_path, sam_path, clip_model_name, clip_pretrained):
    """Load YOLO-World, MobileSAM, and open_clip in one shot. Returns
    `(yolo, sam, clip_model, clip_preprocess, clip_tokenizer, device)`
    or `(None, ...)` if any piece is unavailable."""
    yw = yolo_path or os.environ.get("SCENE_YOLO_WORLD_WEIGHTS",
                                     _DEFAULT_YOLO_WORLD_WEIGHTS)
    mp = sam_path or os.environ.get("SCENE_MOBILE_SAM_WEIGHTS",
                                    _DEFAULT_MOBILE_SAM_WEIGHTS)
    cm = clip_model_name or os.environ.get("SCENE_CLIP_MODEL", _DEFAULT_CLIP_MODEL)
    cp = clip_pretrained or os.environ.get("SCENE_CLIP_PRETRAINED",
                                           _DEFAULT_CLIP_PRETRAINED)
    if not os.path.isfile(yw):
        log.warning("YOLO-World weights not found at %s — perception disabled", yw)
        return (None,) * 6
    if not os.path.isfile(mp):
        log.warning("MobileSAM weights not found at %s — perception disabled", mp)
        return (None,) * 6
    try:
        import torch
        from ultralytics import YOLO, SAM
        import open_clip
    except Exception as e:  # noqa: BLE001
        log.warning("ultralytics / open_clip / torch not importable (%s) — perception disabled", e)
        return (None,) * 6

    # Default to CUDA when available, but allow forcing CPU via env.
    # The Blackwell sm_120 + Open3D + concept-graphs combination has
    # been segfaulting (SIGSEGV exit 139) under CUDA on this host;
    # SCENE_CG_FORCE_CPU=1 sidesteps it at the cost of ~3-5x slower
    # YOLO/SAM inference (still real-time at the 0.6s tick).
    if os.environ.get("SCENE_CG_FORCE_CPU", "").strip() in ("1", "true", "yes"):
        device = "cpu"
        log.warning("[scene-cg] SCENE_CG_FORCE_CPU set — running on CPU")
    else:
        device = "cuda" if torch.cuda.is_available() else "cpu"
    if device == "cpu":
        log.warning(
            "[scene-cg] CUDA not available — concept-graphs runs on CPU; "
            "expect ~5x slower ticks. Detector will still merge correctly."
        )

    try:
        yolo = YOLO(yw)
        classes = _resolved_classes()
        yolo.set_classes(classes)
        log.info("[scene-cg] YOLO-World loaded: %d open-vocab classes", len(classes))
    except Exception as e:  # noqa: BLE001
        log.warning("YOLO-World load failed: %s", e)
        return (None,) * 6
    try:
        sam = SAM(mp)
        log.info("[scene-cg] MobileSAM loaded")
    except Exception as e:  # noqa: BLE001
        log.warning("MobileSAM load failed: %s", e)
        return (None,) * 6
    try:
        clip_model, _, clip_preprocess = open_clip.create_model_and_transforms(
            cm, pretrained=cp,
        )
        clip_model = clip_model.to(device).eval()
        clip_tokenizer = open_clip.get_tokenizer(cm)
        log.info("[scene-cg] open_clip %s/%s loaded on %s", cm, cp, device)
    except Exception as e:  # noqa: BLE001
        log.warning("open_clip load failed: %s", e)
        return (None,) * 6

    return yolo, sam, clip_model, clip_preprocess, clip_tokenizer, device


def _import_cg():
    """Import concept-graphs's slam modules. Deferred so that scene
    can boot even if /opt/concept-graphs isn't installed (the load_models
    path will already have failed)."""
    from conceptgraph.slam.slam_classes import MapObjectList, DetectionList
    from conceptgraph.slam.utils import detections_to_obj_pcd_and_bbox
    from conceptgraph.slam.mapping import (
        compute_spatial_similarities,
        compute_visual_similarities,
        aggregate_similarities,
        merge_detections_to_objects,
    )
    from conceptgraph.utils.model_utils import compute_clip_features_batched
    from conceptgraph.slam.utils import (
        denoise_objects,
        filter_objects,
        merge_overlap_objects,
        compute_overlap_matrix_general,
    )
    return {
        "MapObjectList": MapObjectList,
        "DetectionList": DetectionList,
        "detections_to_obj_pcd_and_bbox": detections_to_obj_pcd_and_bbox,
        "compute_spatial_similarities": compute_spatial_similarities,
        "compute_visual_similarities": compute_visual_similarities,
        "aggregate_similarities": aggregate_similarities,
        "merge_detections_to_objects": merge_detections_to_objects,
        "compute_clip_features_batched": compute_clip_features_batched,
        "denoise_objects": denoise_objects,
        "filter_objects": filter_objects,
        "merge_overlap_objects": merge_overlap_objects,
        "compute_overlap_matrix_general": compute_overlap_matrix_general,
    }


# ── Detector class ───────────────────────────────────────────────────────
class ConceptGraphsDetector:
    """Per-frame detector that runs the canonical concept-graphs merge
    pipeline. Owns a persistent `MapObjectList` and projects it into
    scene's `ObjectRegistry` once per tick so the web UI/MCP API stay
    consistent.

    `on_detections` is kept for backwards compat but no longer used —
    we write to the registry directly via `_project_to_registry`. The
    caller still passes a registry via this side channel.
    """

    def __init__(
        self,
        *,
        rgb_fetcher_msg: Callable[[], Optional[Any]],
        depth_fetcher_msg: Callable[[], Optional[Any]],
        camera_info_fetcher: Callable[[], Optional[_CamIntrinsics]],
        chassis_pose_fn: Callable[[], Optional[tuple[float, float, float, float]]],
        on_detections: Callable[[list[Detection]], Awaitable[None]],
        registry: ObjectRegistry,
        # Returns the current world frame id (whatever the localizer
        # is publishing in `header.frame_id`). Defaults to "map" when
        # the SelfTracker hasn't seen a pose yet.
        world_frame_fn: Optional[Callable[[], str]] = None,
        period_s: float = 0.6,
        confidence_threshold: float = 0.30,
        camera_height_m: float = 1.1,
        max_detections: int = 30,
        yolo_weights_path: Optional[str] = None,
        sam_weights_path: Optional[str] = None,
        clip_model_name: Optional[str] = None,
        clip_pretrained: Optional[str] = None,
        cfg_overrides: Optional[dict] = None,
        # `hub` exposes tf2 lookups (lookup_transform_4x4). When
        # provided, the camera→map transform is taken from /tf
        # directly instead of composing chassis_pose with a
        # hardcoded camera mount — this matters once SLAM corrects
        # map→odom and the chassis_pose drifts out of map frame.
        hub: Any = None,
        camera_frame: str = "camera_optical_frame",
        # Body frame asserted for the compatibility pose + extrinsics chain.
        # It must match odometry.child_frame_id when odometry is selected.
        # Without it, pose means base_link while odometry uses its own child.
        base_frame: Optional[str] = None,
    ) -> None:
        self._rgb_msg = rgb_fetcher_msg
        self._depth_msg = depth_fetcher_msg
        self._cam_info = camera_info_fetcher
        self._chassis = chassis_pose_fn
        self._on_dets = on_detections
        self._registry = registry
        self._period_s = period_s
        self._conf_thresh = confidence_threshold
        self._cam_z = camera_height_m
        self._max_dets = max_detections
        self._yolo_weights = yolo_weights_path
        self._sam_weights = sam_weights_path
        self._clip_model_name = clip_model_name
        self._clip_pretrained = clip_pretrained
        self._hub = hub
        self._camera_frame = camera_frame
        self._base_frame = (base_frame or "").strip().lstrip("/")
        self._world_frame_fn = world_frame_fn or (lambda: "map")

        self._yolo: Any = None
        self._sam: Any = None
        self._clip_model: Any = None
        self._clip_preprocess: Any = None
        self._clip_tokenizer: Any = None
        self._device: str = "cpu"
        self._cg: Optional[dict] = None
        self._map_objects: Any = None
        self._task: Optional[asyncio.Task[None]] = None
        self._stop = asyncio.Event()
        self._inference_lock = threading.Lock()
        self._classes = _resolved_classes()
        self._tick_idx = 0
        # Set in start() so worker-thread `_project_to_registry` can
        # schedule the actual mutation on the asyncio loop (the registry
        # uses asyncio.Lock, not a sync lock).
        self._asyncio_loop: Optional[asyncio.AbstractEventLoop] = None

        self.cfg = dict(_CFG_DEFAULTS)
        if cfg_overrides:
            self.cfg.update(cfg_overrides)
        # Allow env overrides for the most-tuned knobs. The merge/identity gates
        # below are exposed so the desk chair/table split and object dedup can be
        # tuned live on a running robot without a rebuild.
        for env, key, cast in (
            ("SCENE_CG_MERGE_THRESHOLD", "merge_threshold", float),
            ("SCENE_CG_VOXEL_SIZE", "downsample_voxel_size", float),
            ("SCENE_CG_MIN_POINTS", "min_points_threshold", int),
            ("SCENE_CG_OBJ_MIN_POINTS", "obj_min_points", int),
            ("SCENE_CG_OBJ_MAX_POINTS", "obj_pcd_max_points", int),
            ("SCENE_CG_MAX_MERGE_DIST_M", "max_merge_dist_m", float),
            ("SCENE_CG_CROSS_CLASS_CENTROID_MAX_M", "cross_class_centroid_max_m", float),
            ("SCENE_CG_CROSS_CLASS_IOU_THRESH", "cross_class_iou_thresh", float),
            ("SCENE_CG_CROSS_CLASS_OVERLAP_THRESH", "cross_class_overlap_thresh", float),
            ("SCENE_CG_MERGE_OVERLAP_THRESH", "merge_overlap_thresh", float),
            ("SCENE_CG_MERGE_VISUAL_SIM_THRESH", "merge_visual_sim_thresh", float),
            ("SCENE_CG_SAME_CLASS_MERGE_DIST_M", "same_class_merge_dist_m", float),
        ):
            v = os.environ.get(env, "").strip()
            if v:
                try:
                    self.cfg[key] = cast(v)
                except ValueError:
                    pass

        # uuid → registry object_id binding (see _apply_snapshot). Initialised
        # here (not lazily) so _apply_snapshot is safe to call before the first
        # _project_to_registry tick — including from unit tests.
        self._uuid_to_oid: dict[str, str] = {}
        # How long a soft-evicted (missing) registry record is kept so a
        # re-detection can re-bind it before it is hard-pruned. Decouples
        # observation_count from concept-graphs uuid churn across ticks.
        try:
            self._object_ttl_s = float(
                os.environ.get("SCENE_OBJECT_TTL_SEC", "30.0").strip() or "30.0"
            )
        except ValueError:
            self._object_ttl_s = 30.0
        # Optional confusable-class reconciliation (off by default): maps a
        # group of classes (e.g. chair/table/desk) to one bucket so YOLO-World
        # label flicker across the group merges instead of splitting into
        # separate records. Applied only inside the same-class merge gate and
        # the same-class proximity collapse — both still distance-gated.
        self._merge_class_group = _parse_merge_class_groups(
            os.environ.get("SCENE_CG_MERGE_CLASS_GROUPS", "")
        )

    async def start(self) -> None:
        if self._task is not None:
            return
        loop = asyncio.get_running_loop()
        self._asyncio_loop = loop
        (
            self._yolo, self._sam,
            self._clip_model, self._clip_preprocess, self._clip_tokenizer,
            self._device,
        ) = await loop.run_in_executor(
            None, _try_load_models,
            self._yolo_weights, self._sam_weights,
            self._clip_model_name, self._clip_pretrained,
        )
        if self._yolo is None:
            log.warning("ConceptGraphsDetector skipped — models unavailable")
            return
        try:
            self._cg = _import_cg()
            self._map_objects = self._cg["MapObjectList"]()
        except Exception as e:  # noqa: BLE001
            log.warning("concept-graphs slam modules unavailable (%s) — perception disabled", e)
            return

        self._stop.clear()
        self._task = asyncio.create_task(self._loop(), name="scene-cg-detector")
        log.info(
            "ConceptGraphsDetector started (period=%.1fs, classes=%d, device=%s, "
            "voxel=%s, merge_threshold=%s)",
            self._period_s, len(self._classes), self._device,
            self.cfg["downsample_voxel_size"], self.cfg["merge_threshold"],
        )

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            await self._task
            self._task = None

    # ── Text embedding (shared CLIP) ─────────────────────────────────
    def embed_text(self, texts: list[str]) -> Optional[list[list[float]]]:
        """Encode `texts` with the already-loaded open_clip text encoder,
        L2-normalized to match the per-object image features. Returns one
        512-d vector per input, or None when CLIP isn't loaded (models
        unavailable / detector not started) so the caller can fall back.

        Reuses the model loaded in `start()` — no second model instance.
        Runs synchronously under torch.no_grad on `self._device`."""
        if self._clip_model is None or self._clip_tokenizer is None or not texts:
            return None
        try:
            import torch

            with self._inference_lock, torch.no_grad():
                tokens = self._clip_tokenizer(texts).to(self._device)
                feats = self._clip_model.encode_text(tokens)
                feats = feats / feats.norm(dim=-1, keepdim=True)
                return feats.cpu().tolist()
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-cg] embed_text failed: %s", e)
            return None

    # ── External viz access ──────────────────────────────────────────
    # Web UI's `/3d` page calls into here every frame. Held under the
    # detector's tick lock so we never serialize a half-mutated object.
    # Each object is downsampled to at most `max_points_per_obj` for
    # JSON-friendly transport (Float32Array as base64 would be denser
    # but the JSON form keeps the client trivial — plain `fetch`).
    def export_3d_snapshot(self, max_points_per_obj: int = 256) -> dict:
        if self._map_objects is None:
            return {"objects": [], "stamp_unix": time.time()}
        try:
            import numpy as np
        except Exception:
            return {"objects": [], "stamp_unix": time.time()}
        out = []
        # We don't acquire the inference lock here — that runs on a
        # worker thread and can block for ~100ms during YOLO/SAM. The
        # MapObjectList is appended/replaced atomically per-tick; a
        # stale read between ticks is fine for visualization.
        try:
            map_objs = list(self._map_objects)
        except Exception:
            return {"objects": [], "stamp_unix": time.time()}
        # Filter to only objects that have made it into the registry —
        # `_uuid_to_oid` is the cache `_apply_snapshot` populates, so
        # this is the same set the 2D web UI / MCP API see. Without
        # this filter the 3D view shows every transient MapObjectList
        # entry (including ones that just got created and not yet
        # projected, or ones that periodic cleanup is about to evict),
        # which never matches the 2D view's count.
        live_uuids = getattr(self, "_uuid_to_oid", None)
        for obj_idx, obj in enumerate(map_objs):
            if live_uuids is not None and live_uuids:
                u = str(obj.get("id", ""))
                if u and u not in live_uuids:
                    continue
            try:
                pcd = obj.get("pcd")
                bbox = obj.get("bbox")
                if pcd is None or bbox is None:
                    continue
                pts = np.asarray(pcd.points)
                cols = np.asarray(pcd.colors) if hasattr(pcd, "colors") else None
                if pts.size == 0:
                    continue
                # Drop NaN/Inf rows — depth back-projection produces
                # Inf values when depth==0 (sky/uncovered pixels) and
                # NaN when the camera matrix is singular. JSON refuses
                # them ("Out of range float values are not JSON
                # compliant"). Strip before sampling.
                finite_mask = np.all(np.isfinite(pts), axis=1)
                if not finite_mask.all():
                    pts = pts[finite_mask]
                    if cols is not None and cols.size:
                        cols = cols[finite_mask]
                if pts.shape[0] == 0:
                    continue
                if pts.shape[0] > max_points_per_obj:
                    idx = np.random.choice(pts.shape[0], size=max_points_per_obj, replace=False)
                    pts = pts[idx]
                    if cols is not None and cols.size:
                        cols = cols[idx]
                # Yaw-only OBB via 2D PCA on the XY footprint. Same
                # rationale as in `_project_to_registry`: Open3D's
                # `get_oriented_bounding_box(robust=True)` segfaults
                # in qhull on certain pcds, so we never call it.
                yaw = _yaw_from_pca_xy(pts[:, :2])
                # Rotate the pts into yaw-aligned frame, take axis-
                # aligned extent there, build 8 corners, rotate back.
                cos_y = float(np.cos(yaw)); sin_y = float(np.sin(yaw))
                rot_xy = np.array([[cos_y, sin_y], [-sin_y, cos_y]])
                pts_xy = pts[:, :2] - pts[:, :2].mean(axis=0)
                pts_local = pts_xy @ rot_xy.T  # rotate into yaw frame
                # 5–95 percentile instead of full min/max — robust
                # against the depth-spike outliers that were inflating
                # bbox dimensions to 3 m+ on a single chair.
                lo_x, hi_x = np.percentile(pts_local[:, 0], [5, 95])
                lo_y, hi_y = np.percentile(pts_local[:, 1], [5, 95])
                lo_z, hi_z = np.percentile(pts[:, 2], [5, 95])
                lo_x, hi_x = float(lo_x), float(hi_x)
                lo_y, hi_y = float(lo_y), float(hi_y)
                lo_z, hi_z = float(lo_z), float(hi_z)
                cx, cy = pts[:, :2].mean(axis=0)
                # 8 corners in robot-frame XY (yaw-rotated) + raw Z.
                local_corners = np.array([
                    [lo_x, lo_y, lo_z], [hi_x, lo_y, lo_z],
                    [lo_x, hi_y, lo_z], [lo_x, lo_y, hi_z],
                    [hi_x, hi_y, hi_z], [lo_x, hi_y, hi_z],
                    [hi_x, lo_y, hi_z], [hi_x, hi_y, lo_z],
                ])
                corners = np.zeros_like(local_corners)
                # Rotate xy back into world, then translate.
                world_xy = local_corners[:, :2] @ rot_xy
                corners[:, 0] = world_xy[:, 0] + cx
                corners[:, 1] = world_xy[:, 1] + cy
                corners[:, 2] = local_corners[:, 2]
                bbox_corners_arr = corners
                if not np.all(np.isfinite(bbox_corners_arr)):
                    continue
                bbox_corners = bbox_corners_arr.tolist()  # 8x3
                inst_color = obj.get("inst_color")
                if inst_color is None:
                    inst_color = [0.5, 0.5, 0.5]
                else:
                    inst_color = [float(v) for v in inst_color]
                out.append({
                    "id": str(obj.get("id", f"obj_{obj_idx}")),
                    "cls": obj.get("class_name", "object"),
                    "num_detections": int(obj.get("num_detections", 1)),
                    "n_points": int(obj.get("n_points", pts.shape[0])),
                    "conf_mean": (float(np.mean(obj["conf"])) if obj.get("conf") else 0.0),
                    # Center + size for HUD; clients can also derive from corners.
                    "center": pts.mean(axis=0).tolist(),
                    "bbox_corners": bbox_corners,
                    "inst_color": inst_color,
                    "points": pts.tolist(),
                    "point_colors": cols.tolist() if (cols is not None and cols.size) else None,
                })
            except Exception:  # noqa: BLE001
                continue
        return {"objects": out, "stamp_unix": time.time()}

    # ── frame bundle (for the scene-graph image relation pass) ────────
    def latest_frame_bundle(self):
        """Return ``(rgb_bgr, K, T_cam_map)`` for projecting map-frame points
        into the current camera image, or None when any piece is unavailable.

        Consumed by the scene-graph builder's image-grounded relation pass.
        Reads the latest RGB frame, intrinsics, and camera→map transform
        WITHOUT holding ``_inference_lock`` — same rationale as
        ``export_3d_snapshot``: the lock is held by the worker tick for ~100 ms
        of YOLO/SAM and the asyncio-loop caller must not block on it. A
        one-tick mismatch between frame and transform is immaterial (poses move
        slowly relative to the 30 s rebuild cadence)."""
        rgb_msg = self._rgb_msg()
        if rgb_msg is None:
            return None
        rgb = _image_msg_to_bgr(rgb_msg)
        if rgb is None:
            return None
        K = self._cam_info()
        if K is None or K.fx <= 0 or K.fy <= 0:
            return None
        try:
            T = self._build_camera_to_map_transform()
        except Exception as e:  # noqa: BLE001
            log.debug("[scene-cg] frame bundle: transform unavailable: %s", e)
            return None
        if T is None:
            return None
        return rgb, K, T

    # ── tick loop ─────────────────────────────────────────────────────
    async def _loop(self) -> None:
        loop = asyncio.get_running_loop()
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                await loop.run_in_executor(None, self._tick_once)
            except Exception as e:  # noqa: BLE001
                log.exception("cg tick error: %s", e)
            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.0, self._period_s - elapsed))

    def _tick_once(self) -> None:
        with self._inference_lock:
            self._tick_locked()

    def _tick_locked(self) -> None:
        rgb_msg = self._rgb_msg()
        depth_msg = self._depth_msg()
        if rgb_msg is None or depth_msg is None:
            # One-line diagnostic so "no ticks at all" stops looking
            # like a silent crash. Throttle to once every 25 polls (~15s
            # at the default 0.6s tick) so it doesn't drown the log.
            self._tick_idx += 1
            if self._tick_idx % 25 == 1:
                log.info("[scene-cg] waiting for frames: rgb=%s depth=%s",
                         "ok" if rgb_msg is not None else "none",
                         "ok" if depth_msg is not None else "none")
            return
        K = self._cam_info()
        if K is None or K.fx <= 0 or K.fy <= 0:
            self._tick_idx += 1
            if self._tick_idx % 25 == 1:
                log.info("[scene-cg] waiting for camera intrinsics")
            return
        try:
            import numpy as np
            import torch
        except Exception:
            return
        rgb = _image_msg_to_bgr(rgb_msg)
        depth = _depth_msg_to_metres(depth_msg)
        if rgb is None or depth is None:
            return
        # YOLO-World predict expects RGB in standard channel order.
        # Internal Ultralytics handles BGR-as-input fine but CLIP later
        # needs RGB. Convert once here.
        rgb_for_clip = rgb[:, :, ::-1].copy()  # BGR → RGB

        # ── YOLO-World detect ────────────────────────────────────────
        try:
            # device must be explicit: ultralytics auto-selects CUDA when
            # torch sees a GPU, bypassing SCENE_CG_FORCE_CPU — and crashing
            # on hosts where YOLO-under-CUDA is unstable (the reason that
            # flag exists).
            yolo_results = self._yolo.predict(
                rgb, conf=self._conf_thresh, verbose=False,
                device=self._device,
            )
        except Exception as e:  # noqa: BLE001
            log.warning("yolo-world predict failed: %s", e)
            return
        if not yolo_results:
            return
        r0 = yolo_results[0]
        boxes = getattr(r0, "boxes", None)
        if boxes is None or len(boxes) == 0:
            self._tick_idx += 1
            self._maybe_periodic_cleanup()
            return

        try:
            xyxy = boxes.xyxy.detach().cpu().numpy()
            confs = boxes.conf.detach().cpu().numpy()
            cls_idx = boxes.cls.detach().cpu().numpy().astype(int)
        except Exception:  # noqa: BLE001
            return
        if xyxy.shape[0] == 0:
            return

        # Cap the count so SAM doesn't get a 200-bbox dump.
        if xyxy.shape[0] > self._max_dets:
            top = np.argsort(confs)[-self._max_dets:]
            xyxy = xyxy[top]
            confs = confs[top]
            cls_idx = cls_idx[top]

        names = getattr(r0, "names", None) or {i: c for i, c in enumerate(self._classes)}
        # Per-tick diagnostic: how many bboxes did YOLO-World return,
        # what classes? Throttled to once every 25 ticks (~15s @ 0.6s
        # period). Without this it's impossible to tell whether
        # "1 dets" downstream means YOLO sees one thing vs the rest
        # got dropped by class/floor/SAM/pcd filters.
        if self._tick_idx % 25 == 0:
            cls_names = [str(names.get(int(c), f"c{int(c)}")) for c in cls_idx]
            log.info("[scene-cg] yolo: %d boxes, top conf=%.2f, classes=%s",
                     int(xyxy.shape[0]),
                     float(confs.max()) if confs.size else 0.0,
                     cls_names[:8])

        # Filter ignored classes BEFORE running SAM (saves work).
        keep = np.array([
            str(names.get(int(cidx), f"class_{cidx}")).lower() not in _IGNORED_CLASSES
            for cidx in cls_idx
        ], dtype=bool)
        if not keep.any():
            self._tick_idx += 1
            self._maybe_periodic_cleanup()
            return
        xyxy = xyxy[keep]
        confs = confs[keep]
        cls_idx = cls_idx[keep]

        # ── MobileSAM mask each bbox ─────────────────────────────────
        try:
            sam_results = self._sam.predict(
                rgb, bboxes=xyxy.tolist(), verbose=False,
            )
        except Exception as e:  # noqa: BLE001
            log.warning("mobile-sam predict failed: %s — skipping tick", e)
            return
        if not sam_results or getattr(sam_results[0], "masks", None) is None:
            log.debug("mobile-sam returned no masks")
            return
        masks_t = sam_results[0].masks.data
        masks = masks_t.detach().cpu().numpy().astype(bool)
        if masks.shape[0] != xyxy.shape[0]:
            # SAM dropped some boxes (e.g. degenerate). Realign.
            n = min(masks.shape[0], xyxy.shape[0])
            masks = masks[:n]
            xyxy = xyxy[:n]
            confs = confs[:n]
            cls_idx = cls_idx[:n]
        if masks.shape[0] == 0:
            return

        # Resize masks to depth resolution if needed (MobileSAM masks
        # come back at the input resolution, which matches RGB; depth
        # may differ. Webots both 640x480 so this is a no-op there).
        h_d, w_d = depth.shape[:2]
        h_m, w_m = masks.shape[1], masks.shape[2]
        if (h_m, w_m) != (h_d, w_d):
            try:
                import cv2
                resized = np.zeros((masks.shape[0], h_d, w_d), dtype=bool)
                for i in range(masks.shape[0]):
                    resized[i] = cv2.resize(
                        masks[i].astype(np.uint8), (w_d, h_d),
                        interpolation=cv2.INTER_NEAREST,
                    ).astype(bool)
                masks = resized
            except Exception as e:  # noqa: BLE001
                log.warning("mask resize failed: %s — skipping tick", e)
                return

        # ── CLIP per-detection feature ──────────────────────────────
        try:
            import supervision as sv
            sv_dets = sv.Detections(
                xyxy=xyxy.astype(np.float32),
                class_id=cls_idx.astype(int),
                confidence=confs.astype(np.float32),
            )
            sv_dets.mask = masks
            _, image_feats, _ = self._cg["compute_clip_features_batched"](
                rgb_for_clip, sv_dets,
                self._clip_model, self._clip_preprocess, self._clip_tokenizer,
                self._classes, self._device,
            )
        except Exception as e:  # noqa: BLE001
            # First failure: dump full traceback so we know which list
            # in concept-graphs's batched CLIP path was overrun (cls
            # idx vs class list mismatch is the usual cause). Subsequent
            # failures stay terse.
            if not getattr(self, "_clip_fail_logged", False):
                import traceback
                log.warning(
                    "CLIP feature compute failed (first occurrence — full trace; "
                    "n_dets=%d, n_classes=%d, max_cls_idx=%s):\n%s",
                    int(xyxy.shape[0]) if hasattr(xyxy, "shape") else -1,
                    len(self._classes) if hasattr(self, "_classes") else -1,
                    int(max(cls_idx)) if len(cls_idx) else "n/a",
                    traceback.format_exc(),
                )
                self._clip_fail_logged = True
            else:
                log.warning("CLIP feature compute failed: %s — skipping tick", e)
            return

        # ── Per-detection PCD + bbox in map frame ────────────────────
        cam_K_mat = np.array([
            [K.fx, 0,    K.cx],
            [0,    K.fy, K.cy],
            [0,    0,    1.0],
        ], dtype=np.float32)
        trans_pose = self._build_camera_to_map_transform()
        try:
            obj_pcds_and_bboxes = self._cg["detections_to_obj_pcd_and_bbox"](
                depth_array=depth.astype(np.float32),
                masks=masks,
                cam_K=cam_K_mat,
                image_rgb=rgb_for_clip,
                trans_pose=trans_pose,
                min_points_threshold=self.cfg["min_points_threshold"],
                spatial_sim_type=self.cfg["spatial_sim_type"],
                obj_pcd_max_points=self.cfg["obj_pcd_max_points"],
                downsample_voxel_size=self.cfg["downsample_voxel_size"],
                dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                dbscan_eps=self.cfg["dbscan_eps"],
                dbscan_min_points=self.cfg["dbscan_min_points"],
                run_dbscan=False,
                device=self._device,
            )
        except Exception as e:  # noqa: BLE001
            log.warning("detections_to_obj_pcd_and_bbox failed: %s — skipping tick", e)
            return

        # ── Build DetectionList ──────────────────────────────────────
        DetectionList = self._cg["DetectionList"]
        det_list = DetectionList()
        image_idx = self._tick_idx
        for i in range(xyxy.shape[0]):
            entry = obj_pcds_and_bboxes[i]
            if entry is None:
                continue
            cls_id_i = int(cls_idx[i])
            cls_name = str(names.get(cls_id_i, f"class_{cls_id_i}")).lower()
            # Drop background classes (floor/wall/ceiling/carpet) — these
            # exist in `_resolved_classes` so YOLO-World *can* label
            # them when it gets confused on a flat surface, but they're
            # not "objects" in the scene-graph sense. The previous code
            # set an `is_background` flag on the detection record but
            # never read it, so floor/wall ended up in the registry.
            if cls_name in _BG_CLASSES:
                continue
            # Floor-only filter — when YOLO's mask leaks past an
            # object's footprint or there's a thin object on the floor
            # the depth-backprojected pcd ends up almost entirely
            # below z=0.10 m. Such pcds get labelled "desk" / "shelf"
            # / "trash bin" but the points are obviously the floor.
            # Drop them before they reach the merge pipeline.
            try:
                pts_chk = np.asarray(entry["pcd"].points)
                if pts_chk.shape[0] >= 4:
                    z_max = float(pts_chk[:, 2].max())
                    z_p90 = float(np.percentile(pts_chk[:, 2], 90))
                    # Class-specific minimum height: a desk top sits
                    # ≥ 0.55 m above the floor, a chair seat ≥ 0.30 m,
                    # a cup on a table ≥ 0.50 m. If none of those hold
                    # it's almost certainly floor noise.
                    floor_classes = {
                        "table", "desk", "couch", "sofa", "shelf",
                        "bookshelf", "cabinet", "drawer", "monitor",
                        "monitor stand", "computer tower",
                    }
                    if cls_name in floor_classes and z_max < 0.30:
                        log.debug(
                            "[scene-cg] drop %s det at z_max=%.2f (floor noise)",
                            cls_name, z_max,
                        )
                        continue
                    if z_p90 < 0.05:
                        # Anything whose 90th percentile is on the
                        # floor cell, drop regardless of class.
                        continue
            except Exception:
                pass
            d = {
                "id": uuid.uuid4(),
                "image_idx": [image_idx],
                "mask_idx": [i],
                "color_path": [""],
                "class_name": cls_name,
                "class_id": [cls_id_i],
                "captions": [""],
                "num_detections": 1,
                "mask": [masks[i]],
                "xyxy": [xyxy[i].tolist()],
                "conf": [float(confs[i])],
                "n_points": len(entry["pcd"].points),
                "contain_number": [None],
                "inst_color": np.random.rand(3),
                "is_background": cls_name in _BG_CLASSES,
                "pcd": entry["pcd"],
                "bbox": entry["bbox"],
                "clip_ft": torch.from_numpy(image_feats[i]).float(),
                "num_obj_in_class": 0,
                "curr_obj_num": 0,
                "new_counter": 0,
            }
            det_list.append(d)

        if len(det_list) == 0:
            self._tick_idx += 1
            self._maybe_periodic_cleanup()
            return

        # ── Concept-graphs merge ─────────────────────────────────────
        if len(self._map_objects) == 0:
            for d in det_list:
                self._map_objects.append(d)
            log.info("[scene-cg] init map with %d objects", len(self._map_objects))
        else:
            try:
                # Voxel pcd-overlap (our impl) instead of concept-graphs's
                # compute_spatial_similarities('overlap', ...) which crashes
                # on coplanar bbox vertices via pytorch3d.box3d_overlap. AABB-IoU
                # was the previous fallback but was too weak for the cross-view
                # merge case. See `_voxel_pcd_overlap_matrix` for the rationale.
                #
                # Visual sim comes back on whatever device CLIP runs on
                # (cuda when available); pin our spatial_sim to that same
                # device before they're added together by aggregate_similarities.
                visual_sim = self._cg["compute_visual_similarities"](
                    det_list, self._map_objects,
                )
                spatial_sim = self._voxel_pcd_overlap_torch(det_list, self._map_objects)
                spatial_sim = spatial_sim.to(visual_sim.device)
                agg_sim = self._cg["aggregate_similarities"](
                    self.cfg["match_method"], self.cfg["phys_bias"],
                    spatial_sim, visual_sim,
                )

                # ── Hard centroid-distance + same-class gates ───────
                # Two reasons to short-circuit a "high agg_sim" pair:
                #
                # (1) **Distance gate.** With merge_threshold low,
                #     visual sim alone can carry a match — but two
                #     physically distant potted_plants ALSO have visual
                #     sim ≈ 0.6. Result: one bloated bbox spanning the
                #     room (the 9 × 5 m blob).
                #
                # (2) **Same-class gate.** A potted plant sitting on
                #     top of a cabinet has near-zero centroid distance
                #     and reasonable visual sim → without this gate
                #     the two get merged into a single record (the
                #     "potted_plant and cabinet can no longer be told apart" complaint).
                #     CLIP's discrimination breaks down on co-located
                #     objects; the YOLO-World class label is more
                #     reliable. Only merge when the YOLO classes match
                #     OR when one of the two is the unconfigured
                #     fallback class.
                try:
                    import torch
                    det_centroids = []
                    det_classes = []
                    for d in det_list:
                        pp = np.asarray(d["pcd"].points)
                        det_centroids.append(pp.mean(axis=0) if pp.size else np.zeros(3))
                        det_classes.append(str(d.get("class_name", "")).lower())
                    obj_centroids = []
                    obj_classes = []
                    for o in self._map_objects:
                        pp = np.asarray(o["pcd"].points)
                        obj_centroids.append(pp.mean(axis=0) if pp.size else np.zeros(3))
                        obj_classes.append(str(o.get("class_name", "")).lower())
                    if det_centroids and obj_centroids:
                        dC = np.asarray(det_centroids)             # (M, 3)
                        oC = np.asarray(obj_centroids)             # (N, 3)
                        diffs = dC[:, None, :] - oC[None, :, :]    # (M, N, 3)
                        dist = np.linalg.norm(diffs, axis=2)       # (M, N)
                        max_d = float(self.cfg["max_merge_dist_m"])
                        dist_mask = torch.from_numpy(dist > max_d).to(agg_sim.device)
                        agg_sim[dist_mask] = float("-inf")

                        # Same-class gate. Only enforced when both
                        # sides have a real class label — empty
                        # strings (rare) get a free pass.
                        # **Centroid-proximity bypass**: when the two
                        # are physically co-located (< cross_class_
                        # centroid_max_m) we drop the class gate, so
                        # YOLO label flicker (picture_frame /
                        # shelf / cabinet on the same wall fixture)
                        # collapses into one record instead of three
                        # overlapping bboxes.
                        prox_m = float(self.cfg["cross_class_centroid_max_m"])
                        proximate = dist <= prox_m                  # (M, N) bool
                        # Confusable-class buckets (SCENE_CG_MERGE_CLASS_GROUPS)
                        # collapse e.g. chair/table/desk to one key so a desk's
                        # YOLO label flicker is NOT treated as a class mismatch.
                        # Empty map → identity, i.e. plain class comparison.
                        grp = self._merge_class_group
                        cls_mismatch = np.zeros((len(det_classes), len(obj_classes)), dtype=bool)
                        for i, dc in enumerate(det_classes):
                            dcg = grp.get(dc, dc)
                            for j, oc in enumerate(obj_classes):
                                ocg = grp.get(oc, oc)
                                if dcg and ocg and dcg != ocg and not proximate[i, j]:
                                    cls_mismatch[i, j] = True
                        cls_mask = torch.from_numpy(cls_mismatch).to(agg_sim.device)
                        agg_sim[cls_mask] = float("-inf")
                except Exception as _e:  # noqa: BLE001
                    log.debug("merge-gate skipped: %s", _e)

                # Below threshold → unmatched → new object.
                agg_sim[agg_sim < self.cfg["merge_threshold"]] = float("-inf")
                pre_n = len(self._map_objects)
                self._map_objects = self._cg["merge_detections_to_objects"](
                    downsample_voxel_size=self.cfg["downsample_voxel_size"],
                    dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg["dbscan_eps"],
                    dbscan_min_points=self.cfg["dbscan_min_points"],
                    spatial_sim_type=self.cfg["spatial_sim_type"],
                    device=self._device,
                    match_method=self.cfg["match_method"],
                    phys_bias=self.cfg["phys_bias"],
                    detection_list=det_list,
                    objects=self._map_objects,
                    agg_sim=agg_sim,
                )
                added = len(self._map_objects) - pre_n
                # Per-frame log is too noisy. Only log when (a) something
                # changed (new/total delta) or (b) every Nth tick as a
                # heartbeat. Otherwise INFO once for the first tick is
                # enough so the dev sees ingest is alive.
                self._tick_seq = getattr(self, "_tick_seq", 0) + 1
                prev_total = getattr(self, "_last_logged_total", -1)
                changed = added != 0 or len(self._map_objects) != prev_total
                if changed or self._tick_seq == 1 or self._tick_seq % 100 == 0:
                    log.info(
                        "[scene-cg] tick %d: %d dets, %d new, %d total objects",
                        self._tick_seq, len(det_list), added, len(self._map_objects),
                    )
                    self._last_logged_total = len(self._map_objects)
            except Exception as e:  # noqa: BLE001
                # First few failures: dump the full traceback so we can
                # tell whether it was compute_spatial_similarities,
                # compute_visual_similarities, aggregate, or the merge
                # itself. Subsequent failures stay terse.
                if not getattr(self, "_merge_fail_logged", False):
                    import traceback
                    log.warning(
                        "concept-graphs merge pipeline failed (first occurrence — full trace):\n%s",
                        traceback.format_exc(),
                    )
                    self._merge_fail_logged = True
                else:
                    log.warning("concept-graphs merge pipeline failed: %s", e)
                return

        self._tick_idx += 1
        self._maybe_periodic_cleanup()
        self._project_to_registry()

    # ── Voxel pcd-overlap (replaces concept-graphs `overlap` mode) ──
    @staticmethod
    def _voxel_pcd_overlap_matrix(
        objects_a, objects_b=None, *, voxel_size: float = 0.025,
    ):
        """Pairwise voxel-set fraction-overlap.

        For each pair (i, j) returns
            overlap(i, j) = |A_i ∩ B_j| / min(|A_i|, |B_j|)
        where A_i / B_j are the voxel-grid hashes of the i-th / j-th
        point clouds at `voxel_size` resolution. Pure numpy + Python
        set ops — no pytorch3d, no qhull. Robust to degenerate
        (coplanar / sparse) point clouds, which crash concept-graphs's
        canonical `compute_overlap_matrix_general` via box3d_overlap.

        Returns an (M, N) float32 numpy array, where M = len(objects_a)
        and N = len(objects_b) (or M when `objects_b is None`, with a
        symmetric matrix and zero diagonal).
        """
        # Lazy numpy import — matches the per-method pattern the rest of
        # this file uses (every other method that calls np.* imports it
        # locally). Without this, `np` is unbound here at run time.
        import numpy as np

        def voxel_set(pcd) -> frozenset:
            pts = np.asarray(pcd.points)
            if pts.size == 0:
                return frozenset()
            ix = np.floor(pts / voxel_size).astype(np.int64)
            return frozenset(map(tuple, ix.tolist()))

        a_sets = [voxel_set(o["pcd"]) for o in objects_a]
        symmetric = objects_b is None
        b_sets = a_sets if symmetric else [voxel_set(o["pcd"]) for o in objects_b]

        m, n = len(a_sets), len(b_sets)
        out = np.zeros((m, n), dtype=np.float32)
        for i, a in enumerate(a_sets):
            if not a:
                continue
            j_start = i + 1 if symmetric else 0
            for j in range(j_start, n):
                b = b_sets[j]
                if not b:
                    continue
                inter = len(a & b)
                if inter == 0:
                    continue
                v = inter / min(len(a), len(b))
                out[i, j] = v
                if symmetric:
                    out[j, i] = v
        return out

    def _voxel_pcd_overlap_torch(self, det_list, map_objects):
        """Wrap `_voxel_pcd_overlap_matrix` for the per-tick merge —
        returns an (M, N) torch tensor on `self._device` so it can be
        passed directly to `aggregate_similarities` alongside the CLIP
        visual similarity matrix."""
        import torch
        arr = self._voxel_pcd_overlap_matrix(
            det_list, map_objects, voxel_size=self.cfg["downsample_voxel_size"],
        )
        return torch.from_numpy(arr).to(self._device)

    # ── Periodic cleanup ─────────────────────────────────────────────
    def _maybe_periodic_cleanup(self) -> None:
        if self._map_objects is None or len(self._map_objects) == 0:
            return
        ran_any = False
        if self._tick_idx % self.cfg["denoise_interval_ticks"] == 0:
            try:
                pre = len(self._map_objects)
                self._map_objects = self._cg["denoise_objects"](
                    downsample_voxel_size=self.cfg["downsample_voxel_size"],
                    dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg["dbscan_eps"],
                    dbscan_min_points=self.cfg["dbscan_min_points"],
                    spatial_sim_type=self.cfg["spatial_sim_type"],
                    device=self._device,
                    objects=self._map_objects,
                )
                # filter_objects drops objects with too-few points / detections
                self._map_objects = self._cg["filter_objects"](
                    obj_min_points=self.cfg["obj_min_points"],
                    obj_min_detections=self.cfg["obj_min_detections"],
                    objects=self._map_objects,
                )
                if len(self._map_objects) != pre:
                    log.info("[scene-cg] cleanup denoise+filter: %d → %d", pre, len(self._map_objects))
                ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("denoise/filter failed: %s", e)
        if self._tick_idx % self.cfg["merge_overlap_interval_ticks"] == 0:
            try:
                pre = len(self._map_objects)
                # Canonical concept-graphs cleanup pass. Computes the
                # full N×N pcd-overlap matrix (uses pytorch3d under
                # the hood for oriented-bbox IoU prefilter), then
                # merges every pair where overlap > merge_overlap_thresh
                # AND CLIP cosine > merge_visual_sim_thresh. This is
                # what catches cabinet+shelf at the same physical
                # position labelled separately by YOLO-World — per-tick
                # `merge_detections_to_objects` only matches new dets
                # against the existing map, so two BRAND NEW dets in
                # one tick both get appended; this pass collapses them.
                # Same swap as the per-tick merge: our voxel pcd-overlap
                # impl avoids concept-graphs's pytorch3d crash path. The
                # `merge_overlap_objects` callee accepts the matrix as a
                # plain numpy array of shape (N, N).
                overlap_mat = self._voxel_pcd_overlap_matrix(
                    self._map_objects,
                    objects_b=None,
                    voxel_size=self.cfg["downsample_voxel_size"],
                )
                # concept-graphs `merge_overlap_objects` returns
                # `(MapObjectList, index_updates)` — assigning the
                # tuple directly into self._map_objects silently broke
                # every subsequent tick (compute_spatial_similarities
                # then called .get_stacked_values_torch on a tuple).
                # Unpack and discard index_updates (we don't carry
                # MapEdgeMapping anyway).
                merged_objs, _index_updates = self._cg["merge_overlap_objects"](
                    merge_overlap_thresh=self.cfg["merge_overlap_thresh"],
                    merge_visual_sim_thresh=self.cfg["merge_visual_sim_thresh"],
                    merge_text_sim_thresh=self.cfg["merge_text_sim_thresh"],
                    objects=self._map_objects,
                    overlap_matrix=overlap_mat,
                    downsample_voxel_size=self.cfg["downsample_voxel_size"],
                    dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                    dbscan_eps=self.cfg["dbscan_eps"],
                    dbscan_min_points=self.cfg["dbscan_min_points"],
                    spatial_sim_type=self.cfg["spatial_sim_type"],
                    device=self._device,
                )
                self._map_objects = merged_objs
                if len(self._map_objects) != pre:
                    log.info("[scene-cg] cleanup merge_overlap: %d → %d", pre, len(self._map_objects))
                ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("merge_overlap_objects failed: %s", e)
        if self._tick_idx % self.cfg["cross_class_merge_interval_ticks"] == 0:
            try:
                pre = len(self._map_objects)
                self._map_objects = self._cross_class_geometric_collapse(self._map_objects)
                if len(self._map_objects) != pre:
                    log.info("[scene-cg] cleanup cross-class geom: %d → %d", pre, len(self._map_objects))
                    ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("cross-class geometric collapse failed: %s", e)
        if self._tick_idx % self.cfg["same_class_merge_interval_ticks"] == 0:
            try:
                pre = len(self._map_objects)
                self._map_objects = self._same_class_proximity_collapse(self._map_objects)
                if len(self._map_objects) != pre:
                    log.info("[scene-cg] cleanup same-class proximity: %d → %d", pre, len(self._map_objects))
                    ran_any = True
            except Exception as e:  # noqa: BLE001
                log.warning("same-class proximity collapse failed: %s", e)
        if ran_any:
            self._project_to_registry()

    def _cross_class_geometric_collapse(self, objects):
        """Class-and-visual-agnostic merge pass. Folds together objects
        whose AABBs overlap heavily (IoU or one-inside-other ratio),
        regardless of class label or CLIP feature distance.

        Defends against the YOLO-World label flicker case where the
        same physical wall fixture gets registered three times as
        picture_frame + shelf + cabinet — `merge_overlap_objects`
        skips them because their CLIP features differ enough to fail
        merge_visual_sim_thresh, even though their point clouds are
        on top of each other in 3D.
        """
        # Same per-method lazy numpy import as the rest of this file —
        # without it, `np.*` references below raise NameError and the
        # whole class-agnostic merge pass silently bails every cleanup
        # tick (visible in scene log as "cross-class geometric
        # collapse failed: name 'np' is not defined").
        import numpy as np

        n = len(objects)
        if n < 2:
            return objects
        merge_obj2_into_obj1 = self._cg.get("merge_obj2_into_obj1")
        if merge_obj2_into_obj1 is None:
            try:
                from conceptgraph.slam.utils import merge_obj2_into_obj1 as _m
                merge_obj2_into_obj1 = _m
            except Exception:
                return objects

        bboxes: list[tuple["np.ndarray", "np.ndarray"]] = []
        for o in objects:
            pts = np.asarray(o["pcd"].points)
            pts = pts[np.all(np.isfinite(pts), axis=1)] if pts.size else pts
            if pts.size == 0:
                bboxes.append((np.zeros(3), np.zeros(3)))
            else:
                bboxes.append((pts.min(axis=0), pts.max(axis=0)))

        iou_thr = float(self.cfg["cross_class_iou_thresh"])
        ratio_thr = float(self.cfg["cross_class_overlap_thresh"])
        keep = [True] * n
        for i in range(n):
            if not keep[i]:
                continue
            for j in range(i + 1, n):
                if not keep[j]:
                    continue
                a_min, a_max = bboxes[i]
                b_min, b_max = bboxes[j]
                inter = np.maximum(np.minimum(a_max, b_max) - np.maximum(a_min, b_min), 0.0)
                inter_vol = float(np.prod(inter))
                a_vol = float(np.prod(np.maximum(a_max - a_min, 0.0)))
                b_vol = float(np.prod(np.maximum(b_max - b_min, 0.0)))
                union = a_vol + b_vol - inter_vol
                if union <= 1e-9 or a_vol <= 1e-9 or b_vol <= 1e-9:
                    continue
                iou = inter_vol / union
                ratio = max(inter_vol / a_vol, inter_vol / b_vol)
                if iou < iou_thr and ratio < ratio_thr:
                    continue
                try:
                    objects[i] = merge_obj2_into_obj1(
                        obj1=objects[i], obj2=objects[j],
                        downsample_voxel_size=self.cfg["downsample_voxel_size"],
                        dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                        dbscan_eps=self.cfg["dbscan_eps"],
                        dbscan_min_points=self.cfg["dbscan_min_points"],
                        spatial_sim_type=self.cfg["spatial_sim_type"],
                        device=self._device,
                        run_dbscan=False,
                    )
                except Exception as e:  # noqa: BLE001
                    log.debug("cross-class merge i=%d j=%d failed: %s", i, j, e)
                    continue
                keep[j] = False
                pts = np.asarray(objects[i]["pcd"].points)
                pts = pts[np.all(np.isfinite(pts), axis=1)] if pts.size else pts
                if pts.size:
                    bboxes[i] = (pts.min(axis=0), pts.max(axis=0))

        MapObjectList = self._cg["MapObjectList"]
        out = MapObjectList()
        for k, alive in enumerate(keep):
            if alive:
                out.append(objects[k])
        return out

    def _same_class_proximity_collapse(self, objects):
        """Fold same-class (or same merge-group) objects whose centroids are
        within ``same_class_merge_dist_m`` into one, regardless of visual sim.

        Targets the duplication the visual-gated ``merge_overlap_objects`` pass
        misses: one physical object (e.g. a keyboard) seen across frames at
        AABB-IoU≈0 spawns several same-class records whose CLIP crops diverge
        enough to fail ``merge_visual_sim_thresh`` but which are obviously the
        same thing by proximity + class. Distance 0 disables the pass. Class
        grouping uses the optional SCENE_CG_MERGE_CLASS_GROUPS bucket map so a
        flickering desk's chair/table records also collapse here. Returns a new
        MapObjectList; reuses concept-graphs ``merge_obj2_into_obj1``."""
        import numpy as np

        max_d = float(self.cfg.get("same_class_merge_dist_m", 0.0))
        if max_d <= 0.0:
            return objects
        n = len(objects)
        if n < 2:
            return objects
        merge_obj2_into_obj1 = self._cg.get("merge_obj2_into_obj1")
        if merge_obj2_into_obj1 is None:
            try:
                from conceptgraph.slam.utils import merge_obj2_into_obj1 as _m
                merge_obj2_into_obj1 = _m
            except Exception:
                return objects

        def _centroid(o):
            pts = np.asarray(o["pcd"].points)
            pts = pts[np.all(np.isfinite(pts), axis=1)] if pts.size else pts
            return pts.mean(axis=0) if pts.size else np.zeros(3)

        def _grp(o):
            c = str(o.get("class_name", "")).lower()
            return self._merge_class_group.get(c, c)

        centroids = [_centroid(o) for o in objects]
        groups = [_grp(o) for o in objects]
        keep = [True] * n
        for i in range(n):
            if not keep[i]:
                continue
            for j in range(i + 1, n):
                if not keep[j]:
                    continue
                if not groups[i] or groups[i] != groups[j]:
                    continue
                if float(np.linalg.norm(centroids[i] - centroids[j])) > max_d:
                    continue
                try:
                    objects[i] = merge_obj2_into_obj1(
                        obj1=objects[i], obj2=objects[j],
                        downsample_voxel_size=self.cfg["downsample_voxel_size"],
                        dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                        dbscan_eps=self.cfg["dbscan_eps"],
                        dbscan_min_points=self.cfg["dbscan_min_points"],
                        spatial_sim_type=self.cfg["spatial_sim_type"],
                        device=self._device,
                        run_dbscan=False,
                    )
                except Exception as e:  # noqa: BLE001
                    log.debug("same-class merge i=%d j=%d failed: %s", i, j, e)
                    continue
                keep[j] = False
                centroids[i] = _centroid(objects[i])

        MapObjectList = self._cg["MapObjectList"]
        out = MapObjectList()
        for k, alive in enumerate(keep):
            if alive:
                out.append(objects[k])
        return out

    # ── Cross-class merge (UNUSED) ────────────────────────────────────
    # Kept as a fallback for environments where pytorch3d cannot be
    # installed (the canonical path uses
    # `compute_overlap_matrix_general` which calls into pytorch3d's
    # 3D IoU). No callsite hits this today since pytorch3d ships in
    # the docker image; can be deleted once the ali-dev path is
    # confirmed stable across deployments.
    def _merge_overlap_no_p3d(self, objects):
        try:
            import numpy as np
            import torch
            import torch.nn.functional as F
        except Exception:
            return objects
        n = len(objects)
        if n < 2:
            return objects

        # Stack AABB extents (8 corners → min/max).
        bboxes = []
        for o in objects:
            bb = o["bbox"]
            try:
                pts = np.asarray(bb.get_box_points())
            except Exception:
                pts = np.asarray(o["pcd"].points)
            if pts.size == 0:
                bboxes.append((np.zeros(3), np.zeros(3)))
                continue
            mn = pts.min(axis=0); mx = pts.max(axis=0)
            bboxes.append((mn, mx))

        # CLIP feats stacked.
        clip_fts = []
        for o in objects:
            ft = o["clip_ft"]
            if isinstance(ft, np.ndarray):
                ft = torch.from_numpy(ft).float()
            clip_fts.append(F.normalize(ft.float().flatten(), dim=0))
        clip_mat = torch.stack(clip_fts, dim=0)  # (N, D)

        from conceptgraph.slam.utils import merge_obj2_into_obj1

        keep = [True] * n
        merged_into = list(range(n))  # union-find lite
        for i in range(n):
            if not keep[i]:
                continue
            for j in range(i + 1, n):
                if not keep[j]:
                    continue
                # AABB IoU
                a_min, a_max = bboxes[i]
                b_min, b_max = bboxes[j]
                inter_min = np.maximum(a_min, b_min)
                inter_max = np.minimum(a_max, b_max)
                inter_size = np.maximum(inter_max - inter_min, 0.0)
                inter_vol = float(np.prod(inter_size))
                a_vol = float(np.prod(a_max - a_min))
                b_vol = float(np.prod(b_max - b_min))
                union = a_vol + b_vol - inter_vol
                iou = inter_vol / union if union > 1e-9 else 0.0
                # Visual sim (cosine).
                vis = float(F.cosine_similarity(
                    clip_mat[i].unsqueeze(0), clip_mat[j].unsqueeze(0)
                ).item())
                # Spatial overlap proxy: max of (inter/a_vol, inter/b_vol).
                # Same intent as upstream's pcd-based ratio: "how much
                # of one volume is inside the other".
                ratio = 0.0
                if a_vol > 1e-9 and b_vol > 1e-9:
                    ratio = max(inter_vol / a_vol, inter_vol / b_vol)
                merge_thr = self.cfg["merge_overlap_thresh"]
                vis_thr = self.cfg["merge_visual_sim_thresh"]
                if (ratio >= merge_thr and vis >= vis_thr) or (iou > 0.3 and vis >= vis_thr):
                    # j is folded into i.
                    try:
                        objects[i] = merge_obj2_into_obj1(
                            obj1=objects[i], obj2=objects[j],
                            downsample_voxel_size=self.cfg["downsample_voxel_size"],
                            dbscan_remove_noise=self.cfg["dbscan_remove_noise"],
                            dbscan_eps=self.cfg["dbscan_eps"],
                            dbscan_min_points=self.cfg["dbscan_min_points"],
                            spatial_sim_type=self.cfg["spatial_sim_type"],
                            device=self._device,
                            run_dbscan=False,
                        )
                    except Exception as e:  # noqa: BLE001
                        log.debug("merge_obj2_into_obj1 failed (i=%d,j=%d): %s", i, j, e)
                        continue
                    keep[j] = False
                    # Update bbox/clip cache for i.
                    pts = np.asarray(objects[i]["pcd"].points)
                    if pts.size:
                        bboxes[i] = (pts.min(axis=0), pts.max(axis=0))
                    ft = objects[i]["clip_ft"]
                    if isinstance(ft, np.ndarray):
                        ft = torch.from_numpy(ft).float()
                    clip_mat[i] = F.normalize(ft.float().flatten(), dim=0)

        # Build new MapObjectList preserving order.
        MapObjectList = self._cg["MapObjectList"]
        out = MapObjectList()
        for k, alive in enumerate(keep):
            if alive:
                out.append(objects[k])
        return out

    # ── Camera-to-world transform ───────────────────────────────────
    def _build_camera_to_map_transform(self):
        """4x4 homogeneous transform mapping camera-optical points
        into the world frame published by the active localizer.

        Three paths, in order of preference:

          1. **tf2 (authoritative).** Look up the selected camera frame
             directly against the world frame advertised by the localizer.
             This preserves the complete URDF/static-transform chain and
             avoids choosing between duplicate transform sources.

          2. **Atlas-resolved compatibility contracts.** Only when tf2 is
             unavailable, compose
             `T(world ← base) ⊕ T(base ← camera_optical)` from the
             two slots:
               - `service/map/pose` (or `service/map/odom`) — robot
                 pose in world frame, supplied by mapping/AMCL/mocap.
               - `primitive/camera/extrinsics` — static camera mount
                 transform supplied by the camera primitive itself.
             The extrinsics message is accepted only when its parent is the
             body frame returned by that selected pose source and its child
             is the selected camera frame.

          3. **Naive fallback.** No pose stream and no tf either —
             compose chassis_pose with a yaw-only rotation and a
             configured camera height. Useful only for bring-up before
             SLAM is wired; emits a debug-level warning.
        """
        import numpy as np

        # Path 1: the TF tree is authoritative. Prefer the world frame the
        # localizer is publishing; use "map" only when no pose sample has
        # identified it yet.
        if self._hub is not None:
            world_frame = self._world_frame_fn() or "map"
            T = self._hub.lookup_transform_4x4(self._camera_frame, world_frame)
            if T is not None:
                return T.astype(np.float32)

        # Path 2: compatibility contracts, used only when TF lookup is not
        # available. Carry the selected pose source's body frame into the
        # extrinsics validator; an unselected odometry sample must never change
        # the coordinate chain chosen for a preferred pose sample.
        pose_transform = self._slot_pose_transform()
        if pose_transform is not None:
            T_pose, pose_body_frame = pose_transform
            T_ext = self._slot_extrinsics_4x4(pose_body_frame)
        else:
            T_pose, T_ext = None, None
        if T_pose is not None and T_ext is not None:
            return (T_pose @ T_ext).astype(np.float32)

        # Path 3: naive composition (bring-up only).
        chassis = self._chassis()
        rx, ry, _, ryaw = chassis if chassis is not None else (0.0, 0.0, 0.0, 0.0)
        R_cb = np.array([
            [0,  0,  1],
            [-1, 0,  0],
            [0, -1,  0],
        ], dtype=np.float32)
        t_cb = np.array([0.0, 0.0, self._cam_z], dtype=np.float32)
        T_cb = np.eye(4, dtype=np.float32)
        T_cb[:3, :3] = R_cb
        T_cb[:3, 3] = t_cb
        c, s = math.cos(ryaw), math.sin(ryaw)
        T_bm = np.array([
            [c, -s, 0, rx],
            [s,  c, 0, ry],
            [0,  0, 1, 0.0],
            [0,  0, 0, 1.0],
        ], dtype=np.float32)
        return T_bm @ T_cb

    def _slot_pose_transform(self):
        """Return ``(T(world ← body), body_frame)`` for the chosen sample.

        ``pose`` is preferred and represents configured ``base_frame`` or
        ``base_link`` because PoseWithCovarianceStamped has no child-frame
        field. ``odom`` instead derives the body from its own child_frame_id;
        an explicit base_frame is an assertion and rejects a conflicting
        odometry sample. The returned frame must be passed to extrinsics
        validation so both matrices describe one coordinate chain.
        """
        if self._hub is None:
            return None
        for kind in ("pose", "odom"):
            if not self._hub.has(kind):
                continue
            msg, stamp_unix, _count = self._hub.latest(kind)
            if msg is None or stamp_unix <= 0:
                continue
            body_frame = self._body_frame_for_pose_source(kind, msg)
            if body_frame is None:
                continue
            p = msg.pose.pose if hasattr(msg, "pose") and hasattr(msg.pose, "pose") else msg.pose
            q = p.orientation
            return (
                _quat_xyz_to_matrix(
                    float(q.x), float(q.y), float(q.z), float(q.w),
                    float(p.position.x), float(p.position.y), float(p.position.z),
                ),
                body_frame,
            )
        return None

    def _body_frame_for_pose_source(self, kind: str, msg):
        """Resolve and validate the body frame for one selected pose source."""
        configured = getattr(self, "_base_frame", "").strip().lstrip("/")
        if kind != "odom":
            return configured or "base_link"
        odom_child = str(getattr(msg, "child_frame_id", "")).strip().lstrip("/")
        if configured and odom_child and odom_child != configured:
            signature = (odom_child, configured)
            if getattr(self, "_invalid_odom_body_signature", None) != signature:
                log.warning(
                    "ignoring odometry with child frame %r; configured "
                    "base_frame asserts %r",
                    odom_child,
                    configured,
                )
                self._invalid_odom_body_signature = signature
            return None
        return configured or odom_child or "base_link"

    def _slot_extrinsics_4x4(self, expected_parent: str | None = None):
        """Read the static camera-mount transform (TransformStamped
        from `primitive/camera/extrinsics`) and return a 4×4
        `T(body ← selected_camera_frame)` matrix. Reject missing or mismatched
        parent/child frame ids rather than composing transforms from
        incompatible coordinate frames. Production callers pass the body
        frame returned by `_slot_pose_transform`; the optional default exists
        only for direct compatibility probes and never inspects odometry."""
        import numpy as np
        if self._hub is None or not self._hub.has("camera_extrinsics"):
            return None
        msg, stamp_unix, _count = self._hub.latest("camera_extrinsics")
        if msg is None or stamp_unix <= 0:
            return None
        parent_frame = str(getattr(getattr(msg, "header", None), "frame_id", ""))
        child_frame = str(getattr(msg, "child_frame_id", ""))
        parent_frame = parent_frame.strip().lstrip("/")
        child_frame = child_frame.strip().lstrip("/")
        expected_parent = str(
            expected_parent
            or getattr(self, "_base_frame", "")
            or "base_link"
        ).strip().lstrip("/")
        expected_child = self._camera_frame.strip().lstrip("/")
        if parent_frame != expected_parent or child_frame != expected_child:
            signature = (parent_frame, child_frame, expected_parent, expected_child)
            if getattr(self, "_invalid_extrinsics_signature", None) != signature:
                log.warning(
                    "ignoring camera extrinsics with frames %r ← %r; expected "
                    "%r ← %r",
                    parent_frame,
                    child_frame,
                    expected_parent,
                    expected_child,
                )
                self._invalid_extrinsics_signature = signature
            return None
        t = msg.transform.translation
        q = msg.transform.rotation
        return _quat_xyz_to_matrix(
            float(q.x), float(q.y), float(q.z), float(q.w),
            float(t.x), float(t.y), float(t.z),
        )

    # ── Project MapObjectList → ObjectRegistry ──────────────────────
    def _project_to_registry(self) -> None:
        """Replace registry's perception-source objects with a snapshot
        of the current MapObjectList. Robot self-record is preserved.

        Runs from the worker thread; the registry uses an asyncio.Lock,
        so we schedule the actual mutation back onto the asyncio loop
        via run_coroutine_threadsafe."""
        if self._map_objects is None or self._asyncio_loop is None:
            return
        try:
            import numpy as np
        except Exception:
            return
        # Cache that maps each MapObjectList entry's stable uuid →
        # registry object_id. Without this, every tick we'd doom +
        # re-insert every object, which (a) churns the user-visible
        # `<cls>_NNN` suffix into the thousands, (b) resets
        # observation_count, (c) breaks consumers that hold an oid
        # across ticks. Initialised once in __init__/start.
        if not hasattr(self, "_uuid_to_oid"):
            self._uuid_to_oid = {}
        snapshots = []
        for obj in self._map_objects:
            try:
                pcd = obj["pcd"]
                pts = np.asarray(pcd.points)
                if pts.size == 0:
                    continue
                # Yaw-only bbox via 2D PCA on the XY footprint. We
                # used to call `pcd.get_oriented_bounding_box(robust=True)`
                # but Open3D's qhull SEGFAULTS on certain near-coplanar
                # / sparse pcds (caught by faulthandler — exit 139,
                # `_project_to_registry` line where we called the OBB).
                # Pure-numpy PCA on the XY footprint dodges qhull
                # entirely and gives the same result for our purposes
                # (the world is upright, all we want is yaw).
                # Drop NaN/Inf rows BEFORE computing the bbox — depth
                # back-projection produces them when depth is 0
                # (sky/holes), and a single Inf in a coordinate
                # poisons mean()/max()/min() into NaN/Inf, which then
                # breaks JSON serialisation downstream
                # ("Out of range float values are not JSON compliant").
                finite_pts = pts[np.all(np.isfinite(pts), axis=1)]
                if finite_pts.shape[0] < 4:
                    continue
                pts = finite_pts
                obb_yaw = _yaw_from_pca_xy(pts[:, :2])
                cy_, sy_ = float(np.cos(obb_yaw)), float(np.sin(obb_yaw))
                pts_local_xy = (pts[:, :2] - pts[:, :2].mean(axis=0)) @ np.array([[cy_, sy_], [-sy_, cy_]]).T
                obb_center = np.array([
                    float(pts[:, 0].mean()), float(pts[:, 1].mean()), float(pts[:, 2].mean()),
                ])
                # Use 5–95 percentile range instead of full min/max:
                # a single outlier point (depth backprojection
                # spike, mask leak) along the principal axis was
                # blowing chair / table extents to 3 m+. Percentile
                # is robust to a few stragglers and matches what
                # the user actually expects to see.
                lo, hi = np.percentile(pts_local_xy[:, 0], [5, 95])
                ex_x = float(hi - lo)
                lo, hi = np.percentile(pts_local_xy[:, 1], [5, 95])
                ex_y = float(hi - lo)
                lo, hi = np.percentile(pts[:, 2], [5, 95])
                ex_z = float(hi - lo)
                obb_extent = np.array([ex_x, ex_y, ex_z])
                # Final NaN/Inf guard. If any value made it through
                # (e.g. zero-variance pcd → PCA returns NaN yaw), skip
                # this object rather than letting JSON reject the
                # whole snapshot.
                if not (np.all(np.isfinite(obb_center)) and np.all(np.isfinite(obb_extent))
                        and math.isfinite(obb_yaw)):
                    continue

                cls = _canon_class(obj.get("class_name", "object"))
                conf_list = obj.get("conf", [])
                conf = float(np.mean(conf_list)) if conf_list else 0.5
                snapshots.append({
                    # The MapObjectList entry's stable uuid — this is
                    # the key the registry uses to decide insert vs.
                    # update vs. evict, so identifiers stop churning.
                    "uuid": str(obj.get("id", "")),
                    "cls": cls,
                    "x": float(obb_center[0]),
                    "y": float(obb_center[1]),
                    "z": float(obb_center[2]),
                    "yaw": float(obb_yaw),
                    "size_x": float(max(0.05, obb_extent[0])),
                    "size_y": float(max(0.05, obb_extent[1])),
                    "size_z": float(max(0.05, obb_extent[2])),
                    "confidence": max(0.0, min(1.0, conf)),
                })
            except Exception:  # noqa: BLE001
                continue

        # Schedule the registry mutation on the asyncio loop so it
        # serializes with the snapshot reader (same asyncio.Lock).
        try:
            asyncio.run_coroutine_threadsafe(
                self._apply_snapshot(snapshots), self._asyncio_loop,
            )
        except Exception as e:  # noqa: BLE001
            log.debug("schedule registry update failed: %s", e)

    async def _apply_snapshot(self, snapshots: list[dict]) -> None:
        """Reconcile concept-graphs MapObjectList state into the
        registry **incrementally**. Robot self-record and non-
        perception objects are preserved; for our own objects we:

          - REUSE the existing registry record when the
            MapObjectList uuid is already mapped (just refresh
            pose / bbox / confidence and bump observation_count)
          - RE-BIND a warm-restored object of the same class within
            the merge-distance gate to a fresh detection before minting
            a new id, so a remembered object keeps its stable id on
            re-observation instead of churning a new suffix
          - ADOPT an orphaned live record (its cg_uuid vacated this tick
            by merge-dedup or filter-cull) of the same class within the
            merge gate, so the object keeps its id + accumulated count
            across a uuid swap instead of resetting to obs=1
          - INSERT a new record only when nothing above matches
          - EVICT registry records whose uuid is no longer present in
            the latest MapObjectList (concept-graphs merged or
            filtered them out) — warm-restored records that have not yet
            been re-bound are exempt (they were never in this process's
            MapObjectList); if never re-seen, mark_stale keeps them
            missing rather than deleting them

        Order matters: bind/adopt/insert runs FIRST, eviction AFTER, so
        the new winning uuid of a merged/regenerated object can adopt the
        record the old uuid is about to vacate (the orphan must still
        exist when the snapshot loop runs). Records touched this tick are
        spared from eviction via `adopted_oids`; observation_count is
        registry-owned (one per tick seen), so it survives a uuid swap.

        The previous "drop everything, re-insert everything" approach
        churned object_ids on every tick (the visible `<cls>_NNN`
        suffix climbed into the thousands), reset observation_count,
        and broke any consumer that held an oid across ticks.
        """
        now = time.time()
        live_uuids = {s["uuid"] for s in snapshots if s.get("uuid")}
        async with self._registry.lock():
            wf = self._world_frame_fn()
            # oids touched this tick (bound / adopted / inserted). Used to (a)
            # stop two detections claiming the same orphan and (b) spare a
            # touched record from the eviction sweep below.
            adopted_oids: set[str] = set()
            # Bind / adopt / insert FIRST; eviction runs after. The new winning
            # uuid of a merged or regenerated object must be able to adopt the
            # record the old uuid is about to vacate, so the orphan has to still
            # exist while this loop runs.
            for s in snapshots:
                u = s.get("uuid", "")
                pose = Pose3D(
                    x=s["x"], y=s["y"], z=s["z"],
                    yaw=s.get("yaw", 0.0), frame_id=wf,
                )
                bbox = BBox3D(
                    size_x=s["size_x"], size_y=s["size_y"], size_z=s["size_z"],
                    yaw=s.get("yaw", 0.0), frame_id=wf,
                )
                oid = self._uuid_to_oid.get(u) if u else None
                existing = self._registry._objects.get(oid) if oid else None
                if existing is None:
                    # No uuid binding yet (first sighting of this uuid). Before
                    # minting a new id, try to re-adopt a stable record so the
                    # object keeps its id (and accumulated count) across a uuid
                    # swap: first a warm-restored object of the same class, then
                    # a live record whose cg_uuid vacated this tick (merge-dedup
                    # / filter-cull). Both use the same class + merge-distance
                    # gate, so re-binding is no looser than same-tick merging.
                    existing = self._rebind_restored(s["cls"], pose)
                    # Cross-tick re-bind: a record soft-evicted (`missing`) by a
                    # previous cull / uuid-churn tick is reclaimed by this
                    # detection, so the object keeps its id + accumulated
                    # observation_count instead of minting a fresh id at obs=1.
                    # Same class + merge-distance gate as restored-rebind. This
                    # is the cross-tick complement to _adopt_orphan (which only
                    # rescues a uuid swap within the SAME tick).
                    if existing is None:
                        existing = self._registry.find_rebindable(
                            s["cls"], pose,
                            float(self.cfg.get("max_merge_dist_m", 1.5)),
                            only_missing=True,
                            exclude_oids=adopted_oids,
                        )
                    # Orphan-adoption needs a uuid to rebind to; a uuid-less
                    # detection can't bind durably, so it must not consume an
                    # orphan another (uuid-bearing) detection could re-adopt
                    # this same tick.
                    if existing is None and u:
                        existing = self._adopt_orphan(
                            s["cls"], pose, live_uuids, adopted_oids,
                        )
                    if existing is not None:
                        existing.attributes.pop("restored", None)
                        # Rebind to the new uuid only when we have one. A
                        # uuid-less detection (CG object missing `id`) can't
                        # establish a durable binding, so leave the adopted
                        # record's old cg_uuid mapping intact rather than
                        # popping it and orphaning the record (it is spared
                        # this tick via adopted_oids regardless).
                        if u:
                            old_uuid = existing.attributes.get("cg_uuid")
                            if old_uuid and old_uuid != u:
                                self._uuid_to_oid.pop(old_uuid, None)
                            existing.attributes["cg_uuid"] = u
                            self._uuid_to_oid[u] = existing.object_id
                if existing is not None:
                    # Update in place. Preserve oid + first_seen. The count is
                    # registry-owned (one per tick seen) so it survives a uuid
                    # swap instead of resetting to the CG object's num_detections.
                    existing.pose = pose
                    existing.bbox = bbox
                    existing.confidence = max(0.0, min(1.0, s["confidence"]))
                    existing.last_seen = now
                    existing.missing = False
                    existing.observation_count += 1
                    adopted_oids.add(existing.object_id)
                else:
                    obj = self._registry.insert_object(
                        cls=s["cls"],
                        pose=pose,
                        bbox=bbox,
                        confidence=s["confidence"],
                        now=now,
                        source="concept_graphs",
                    )
                    # insert_object seeds observation_count=1; it is
                    # registry-owned from here (see the += 1 on re-sighting).
                    if u:
                        obj.attributes["cg_uuid"] = u
                        self._uuid_to_oid[u] = obj.object_id
                    adopted_oids.add(obj.object_id)

            # Evict registry records whose source uuid is gone. Runs AFTER
            # bind/adopt so a record adopted above (cg_uuid rebound to a live
            # uuid, and in adopted_oids) is spared automatically; only
            # genuinely-vanished records remain doomed. We only touch our own
            # records — robot self / surfaces / other-source objects untouched.
            doomed = []
            for oid, obj in list(self._registry._objects.items()):
                if oid in adopted_oids:
                    continue
                if obj.attributes.get("is_robot"):
                    continue
                src = obj.attributes.get("source")
                if src not in ("concept_graphs", None):
                    continue
                # Warm-restored objects were never in this process's
                # MapObjectList, so the uuid-membership rule below can't apply.
                # The loop above re-binds them by class+pose; if never re-seen,
                # mark_stale keeps them missing instead of deleting them here.
                if obj.attributes.get("restored"):
                    continue
                cg_uuid = obj.attributes.get("cg_uuid")
                if cg_uuid is None or cg_uuid not in live_uuids:
                    doomed.append(obj)
                    if cg_uuid is not None:
                        self._uuid_to_oid.pop(cg_uuid, None)
            # Soft-evict (mark `missing`, release the uuid) rather than delete,
            # so a re-detection within SCENE_OBJECT_TTL_SEC can re-bind the same
            # id + observation_count via find_rebindable — fixing the cull→
            # re-detect obs reset. A record concept-graphs deduped lingers
            # `missing` until the TTL prune; a future detection at that spot
            # re-binds the nearest *live* record (the dedup survivor), not the
            # stale one, so soft eviction does not resurrect duplicates.
            for obj in doomed:
                self._registry.soft_evict(obj)
            self._registry.prune_expired(now, self._object_ttl_s)

    def _rebind_restored(self, cls: str, pose: Pose3D) -> Optional[SceneObject]:
        """Nearest warm-restored, not-yet-rebound object of class `cls` whose
        centroid is within the merge-distance gate of `pose`, or None.

        Lets a live detection re-adopt a remembered object's stable id instead
        of spawning a duplicate. Uses the same class + 3D-distance gate as the
        concept-graphs merge (`max_merge_dist_m`), so re-binding is no looser
        than same-tick merging. Caller must hold the registry lock; the caller
        clears the matched object's `restored` flag so each restored object
        binds at most one detection per tick."""
        max_d = float(self.cfg.get("max_merge_dist_m", 1.5))
        best: Optional[SceneObject] = None
        best_d = max_d
        for obj in self._registry._objects.values():
            if not obj.attributes.get("restored") or obj.cls != cls:
                continue
            d = math.sqrt(
                (obj.pose.x - pose.x) ** 2
                + (obj.pose.y - pose.y) ** 2
                + (obj.pose.z - pose.z) ** 2
            )
            if d <= best_d:
                best_d = d
                best = obj
        return best

    def _adopt_orphan(
        self,
        cls: str,
        pose: Pose3D,
        live_uuids: set[str],
        adopted_oids: set[str],
    ) -> Optional[SceneObject]:
        """Nearest same-class concept-graphs record whose cg_uuid vacated this
        tick (merge-dedup or filter-cull), within the merge-distance gate and
        not already adopted, or None.

        Lets the new uuid of a merged/regenerated object inherit the stable id
        (and accumulated observation_count) of the record the old uuid is about
        to vacate, instead of churning a fresh id at obs=1. Considers only
        records whose cg_uuid is set but absent from this tick's `live_uuids`
        (the eviction sweep removed earlier-tick orphans, so a surviving orphan
        was vacated this tick). Same class + 3D-distance gate as
        `_rebind_restored` / the concept-graphs merge, so re-binding is no
        looser than same-tick merging. Caller must hold the registry lock and
        rebinds the matched record's cg_uuid + records its oid in adopted_oids."""
        max_d = float(self.cfg.get("max_merge_dist_m", 1.5))
        best: Optional[SceneObject] = None
        best_d = max_d
        for obj in self._registry._objects.values():
            if obj.object_id in adopted_oids or obj.cls != cls:
                continue
            if obj.attributes.get("is_robot") or obj.attributes.get("restored"):
                continue
            if obj.attributes.get("source") not in ("concept_graphs", None):
                continue
            cg_uuid = obj.attributes.get("cg_uuid")
            if cg_uuid is None or cg_uuid in live_uuids:
                continue
            d = math.sqrt(
                (obj.pose.x - pose.x) ** 2
                + (obj.pose.y - pose.y) ** 2
                + (obj.pose.z - pose.z) ** 2
            )
            if d <= best_d:
                best_d = d
                best = obj
        return best


# ── Geometry helpers ──────────────────────────────────────────────────────
def _quat_xyz_to_matrix(qx: float, qy: float, qz: float, qw: float,
                        tx: float, ty: float, tz: float):
    """Quaternion + translation → 4×4 homogeneous transform.

    Used by the camera-to-world compatibility path when TF is unavailable
    and Scene composes the pose and validated extrinsics contract slots.
    """
    import numpy as np
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return None
    s = 2.0 / n
    wx, wy, wz = s * qw * qx, s * qw * qy, s * qw * qz
    xx, xy, xz = s * qx * qx, s * qx * qy, s * qx * qz
    yy, yz, zz = s * qy * qy, s * qy * qz, s * qz * qz
    R = np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def _yaw_from_pca_xy(xy) -> float:
    """Yaw of the longest in-plane axis via 2D PCA. Pure numpy; replaces
    the Open3D `get_oriented_bounding_box(robust=True)` path that was
    SIGSEGV'ing in qhull on near-coplanar pcds.

    Returns yaw in [-pi/2, pi/2] (the principal axis is direction-
    symmetric, so we wrap into one quadrant half so neighbouring frames
    don't flip).
    """
    try:
        import numpy as np
    except Exception:
        return 0.0
    if xy is None:
        return 0.0
    try:
        a = np.asarray(xy, dtype=np.float64)
    except Exception:
        return 0.0
    if a.ndim != 2 or a.shape[0] < 3 or a.shape[1] < 2:
        return 0.0
    centred = a - a.mean(axis=0)
    cov = centred.T @ centred / max(1, a.shape[0] - 1)
    if not np.all(np.isfinite(cov)):
        return 0.0
    try:
        w, V = np.linalg.eigh(cov)
    except Exception:
        return 0.0
    if w.size == 0:
        return 0.0
    # eigh returns ascending eigenvalues; the largest is at index -1.
    principal = V[:, int(np.argmax(w))]
    yaw = float(np.arctan2(principal[1], principal[0]))
    while yaw > np.pi / 2: yaw -= np.pi
    while yaw < -np.pi / 2: yaw += np.pi
    return yaw


# ── Image / depth helpers ────────────────────────────────────────────────
def _image_msg_to_bgr(msg: Any) -> Optional[Any]:
    """sensor_msgs/Image (rgb8 / bgr8 / rgba8 / bgra8 / mono8 / jpeg) → numpy BGR.

    Webots emits bgra8 (4-channel); leaving that out of the supported
    set silently dropped every frame. Both rgba8 and bgra8 paths here.
    """
    try:
        import numpy as np
    except Exception:
        return None
    encoding = (getattr(msg, "encoding", "") or "").lower()
    data = bytes(msg.data) if not isinstance(msg.data, bytes) else msg.data
    h = int(getattr(msg, "height", 0))
    w = int(getattr(msg, "width", 0))
    if "jpeg" in encoding:
        try:
            import cv2  # type: ignore
            arr = np.frombuffer(data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception:
            return None
    if h == 0 or w == 0:
        return None
    if encoding in ("rgb8", "bgr8"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        if encoding == "rgb8":
            arr = arr[:, :, ::-1]
        return arr.copy()
    if encoding in ("rgba8", "bgra8"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 4)
        if encoding == "rgba8":
            arr = arr[:, :, [2, 1, 0]]
        else:  # bgra8
            arr = arr[:, :, :3]
        return arr.copy()
    if encoding in ("mono8", "8uc1"):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
        return np.stack([arr, arr, arr], axis=2)
    return None


def _depth_msg_to_metres(msg: Any) -> Optional[Any]:
    """sensor_msgs/Image (16UC1 mm or 32FC1 m) → numpy float32 metres."""
    try:
        import numpy as np
    except Exception:
        return None
    encoding = (getattr(msg, "encoding", "") or "").lower()
    h = int(getattr(msg, "height", 0))
    w = int(getattr(msg, "width", 0))
    if h == 0 or w == 0:
        return None
    data = bytes(msg.data) if not isinstance(msg.data, bytes) else msg.data
    if encoding in ("32fc1", "32fc"):
        return np.frombuffer(data, dtype=np.float32).reshape(h, w).copy()
    if encoding in ("16uc1", "mono16", "16uc"):
        mm = np.frombuffer(data, dtype=np.uint16).reshape(h, w).astype(np.float32)
        return mm / 1000.0
    return None
