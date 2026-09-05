# SPDX-License-Identifier: MulanPSL-2.0
"""Scene perception backed by DualMap (Eku127/DualMap, RA-L 2025, Apache-2.0).

DualMap is an online open-vocabulary mapper built from YOLO-World + FastSAM /
MobileSAM + MobileCLIP with a Bayesian class filter and a local/global map.
Scene runs its detector and *local* map on the live RGB-D stream and projects
the resulting objects into ``ObjectRegistry`` exactly the way the
ConceptGraphs backend does, so every consumer (MCP tools, web UI, scene graph,
persistence, the Replica export) stays unchanged.

Reuse strategy: this class subclasses ``ConceptGraphsDetector`` for the parts
that are backend-neutral — frame fetching, camera→map transform, registry
reconciliation (``_project_to_registry`` / ``_apply_snapshot``), the 3D
snapshot for the web UI and ``latest_frame_bundle`` — and replaces model
loading, the per-tick mapping step, text embedding and the export. The map
state it keeps in ``self._map_objects`` is a plain list of dicts shaped like
ConceptGraphs' ``MapObjectList`` entries (``id``, ``class_name``, ``pcd``,
``bbox``, ``conf``, ``num_detections``, ``n_points``, ``inst_color``,
``clip_ft``), rebuilt from DualMap's local map after every frame.

DualMap's own configuration is composed with Hydra from its checked-in YAML
files (``/opt/dualmap/config``) with Scene's overrides applied on top, so the
upstream defaults stay authoritative.

Runtime requirements: the DualMap source at ``SCENE_DUALMAP_ROOT`` (default
``/opt/dualmap``) on ``sys.path``, ``supervision>=0.25``, ``mobileclip``, and
the weights ``yolov8l-world.pt``, ``mobile_sam.pt``, ``FastSAM-s.pt`` and the
MobileCLIP-S2 OpenCLIP checkpoint (see ``_DEFAULT_WEIGHTS``).
"""
from __future__ import annotations

import asyncio
import logging
import os
import sys
import tempfile
import time
from typing import Any, Optional

from .capabilities import DUALMAP_KEYS
from .cg_export import export_map_objects
from .perception_concept_graphs import (
    ConceptGraphsDetector,
    _depth_msg_to_metres,
    _image_msg_to_bgr,
)

log = logging.getLogger("scene.dualmap")

_DEFAULT_ROOT = "/opt/dualmap"
_DEFAULT_WEIGHTS = {
    "yolo": "/opt/models/yolov8l-world.pt",
    "sam": "/opt/models/mobile_sam.pt",
    "fastsam": "/opt/models/dualmap/FastSAM-s.pt",
    "mobileclip": "/opt/models/dualmap/mobileclip_s2_datacompdr.bin",
}
# Text encoder used to stamp labels into the Replica export. The scorer matches
# objects to class prompts through CLIP text features, so the export carries the
# ViT-B-32 text embedding of each object's label (same weights Scene lite uses).
_EXPORT_CLIP = ("ViT-B-32", "/opt/models/open_clip_pytorch_model.bin")

# Export cadence: the map is written every this many ticks (and at stop) so a
# killed container still leaves a recent export behind.
_EXPORT_EVERY_TICKS = 100
# Consecutive CUDA failures after which the tick loop gives up (see _tick_locked).
_CUDA_FAILURE_LIMIT = 20


def _inst_color(uid: str) -> list[float]:
    """Stable pastel colour per object id for the 3D view."""
    h = hash(uid) & 0xFFFFFF
    return [0.35 + 0.65 * ((h >> 16) & 0xFF) / 255.0,
            0.35 + 0.65 * ((h >> 8) & 0xFF) / 255.0,
            0.35 + 0.65 * (h & 0xFF) / 255.0]


class DualMapDetector(ConceptGraphsDetector):
    """Per-frame DualMap mapper that feeds Scene's ``ObjectRegistry``."""

    def __init__(self, *args: Any, dualmap_cfg: Optional[dict] = None, **kwargs: Any) -> None:
        """Same constructor as ``ConceptGraphsDetector`` plus ``dualmap_cfg``, the
        manifest's ``perception.dualmap`` mapping (see ``DUALMAP_KEYS``). Nothing
        heavy happens here; models load in ``start``."""
        super().__init__(*args, **kwargs)
        self._dualmap_cfg = dict(dualmap_cfg or {})
        self._dm_root = os.environ.get("SCENE_DUALMAP_ROOT", "").strip() or _DEFAULT_ROOT
        self._dm: Any = None      # DualMap Detector
        self._lm: Any = None      # DualMap LocalMapManager
        self._gm: Any = None      # DualMap GlobalMapManager
        self._dm_data_input: Any = None
        self._dm_names: list[str] = []
        self._map_objects = []
        self._keep_unknown = bool(self._dualmap_cfg.get("keep_unknown", False))
        # FastSAM only adds class-agnostic segments, which become "unknown" objects.
        # When those are dropped anyway it is pure cost (and a second CUDA thread), so
        # it follows keep_unknown unless the manifest says otherwise.
        self._use_fastsam = bool(self._dualmap_cfg.get("use_fastsam", self._keep_unknown))
        self._consecutive_failures = 0
        # Keyframe gate (see DualMap core.check_keyframe). Feeding every tick
        # gave ~10x the observations of DualMap's stride-10 evaluation and
        # fragmented objects; on a robot a frame without motion adds nothing.
        self._kf_translation_m = float(self._dualmap_cfg.get("keyframe_translation_m", 0.1))
        self._kf_rotation_deg = float(self._dualmap_cfg.get("keyframe_rotation_deg", 3.0))
        self._kf_time_s = float(self._dualmap_cfg.get("keyframe_time_s", 5.0))
        self._last_kf_pose: Any = None
        self._last_kf_time = 0.0
        self._last_frame_key: Any = None
        self._skipped_frames = 0
        # DualMap only merges its local map against itself in end_process(); the
        # same call is made here every N keyframes so partial views of one object
        # (a rug seen from two sides) collapse while the map is live.
        self._merge_every = int(self._dualmap_cfg.get("merge_every_keyframes", 20))
        self._keyframes = 0
        # Only tracks DualMap itself marks stable (its Bayesian class filter has
        # converged) reach the registry; transient tracks that appear for a few
        # frames otherwise pile up as "missing" ghosts in the map views.
        self._stable_only = bool(self._dualmap_cfg.get("stable_only", False))
        # Tracks seen in fewer keyframes than this stay out of the registry (a
        # milder filter than stable_only: one-frame flickers never show up).
        self._min_observations = int(self._dualmap_cfg.get("min_observations", 1))
        # Height of the floor in the world frame, and the gate measured from it:
        # a track whose points all lie on (or under) the floor is depth noise,
        # not an object. The same rule guards the concept_graphs backend, and
        # the value comes from the same place — perception.floor_z_m or
        # SCENE_CG_FLOOR_Z_M, which the Replica replay sets from the dataset's
        # own floor (-1.51 m). A backend-private default of 0 here silently
        # dropped every Replica object below the world origin.
        self._floor_z_override = self._dualmap_cfg.get("floor_z_m")
        self._promoted = 0        # local tracks handed to the global map
        # DualMap's global ("abstract") map is a navigation memory: only tracks
        # judged low-mobility (furniture) are promoted into it, and every other
        # stable track is DROPPED from the local map once it leaves the active
        # window. That is the right memory for "go to the table", and the wrong
        # one for an inventory — on Replica it halved the object count and
        # the segmentation score. DualMap's own Replica evaluation runs
        # local-only (runner_dataset.yaml), which keeps every stable track; so
        # does this backend unless a deployment asks for the global map.
        self._global_map = bool(self._dualmap_cfg.get("global_map", False))
        # DualMap object-lifecycle overrides, passed straight through to its
        # config; only the keys a deployment has a reason to change are exposed.
        self._lifecycle_cfg = {k: int(self._dualmap_cfg[k]) for k in
                               ("stable_num", "active_window_size", "max_pending_count")
                               if self._dualmap_cfg.get(k) is not None}
        # Association knobs, also DualMap's own. Its dataset default gates
        # point overlap at 2 cm, which assumes ground-truth poses; a robot whose
        # SLAM pose is off by 3-10 cm between keyframes never meets it.
        self._lifecycle_cfg.update({k: float(self._dualmap_cfg[k]) for k in
                                    ("downsample_voxel_size", "sim_threshold")
                                    if self._dualmap_cfg.get(k) is not None})
        self._all_objects: list = []  # every track, for the Replica export
        self._classes_file: Optional[str] = None
        self._classes_file_is_temp = False
        self._last_lm_size = -1
        self._undecodable = 0
        self._embed_warned = False
        self._export_encoder: Any = None  # (model, tokenizer) once loaded
        self._export_cache: dict[str, Any] = {}
        unknown = sorted(k for k in self._dualmap_cfg if k not in DUALMAP_KEYS)
        if unknown:
            raise ValueError(f"perception.dualmap has unknown keys {unknown}; accepted: {sorted(DUALMAP_KEYS)}")

    # ── lifecycle ─────────────────────────────────────────────────────
    async def start(self) -> None:
        """Load DualMap in an executor thread and start the tick loop. When
        DualMap cannot be loaded the detector stays idle (no task) and Scene
        runs without object recognition, mirroring the ConceptGraphs backend."""
        if self._task is not None:
            return
        loop = asyncio.get_running_loop()
        self._asyncio_loop = loop
        ok = await loop.run_in_executor(None, self._load_dualmap)
        if not ok:
            log.warning("DualMapDetector skipped — DualMap unavailable (see log above)")
            return
        self._stop.clear()
        self._task = asyncio.create_task(self._loop(), name="scene-dualmap-detector")
        log.info(
            "DualMapDetector started (period=%.1fs, classes=%d, device=%s, fastsam=%s, keep_unknown=%s)",
            self._period_s, len(self._dm_names), self._device, self._use_fastsam, self._keep_unknown,
        )

    async def stop(self) -> None:
        self._stop.set()
        if self._task is not None:
            await self._task
            self._task = None
        if self._dm is not None:
            self._export()
        if self._classes_file_is_temp and self._classes_file:
            try:
                os.remove(self._classes_file)
            except OSError:
                pass

    def _write_classes_file(self) -> Optional[str]:
        """Materialise the manifest vocabulary (``perception.dualmap.classes``)
        as the one-name-per-line file DualMap's YOLO-World wrapper reads.
        ``SCENE_DUALMAP_CLASSES`` may point at an existing file instead."""
        env_path = os.environ.get("SCENE_DUALMAP_CLASSES", "").strip()
        classes = self._dualmap_cfg.get("classes")
        if classes:
            if not isinstance(classes, (list, tuple)) or not all(isinstance(c, str) for c in classes):
                raise ValueError("scene.config.perception.dualmap.classes must be a list of names")
            fd, path = tempfile.mkstemp(prefix="scene-dualmap-classes-", suffix=".txt")
            with os.fdopen(fd, "w") as fh:
                fh.write("\n".join(c.strip() for c in classes if c.strip()) + "\n")
            self._classes_file_is_temp = True
            return path
        if env_path:
            if not os.path.isfile(env_path):
                raise FileNotFoundError(f"SCENE_DUALMAP_CLASSES={env_path} is not a file")
            return env_path
        return None

    def _load_dualmap(self) -> bool:
        """Compose DualMap's Hydra config with Scene's overrides and build the
        detector and local map manager. Runs in an executor thread; returns
        False (after logging why) instead of raising so perception degrades to
        'no detector' the same way the ConceptGraphs backend does."""
        root = self._dm_root
        if not os.path.isdir(os.path.join(root, "config")):
            log.warning("[scene-dualmap] DualMap root %s has no config/ directory", root)
            return False
        if root not in sys.path:
            sys.path.insert(0, root)
        try:
            import torch
            from hydra import compose, initialize_config_dir
            from hydra.core.global_hydra import GlobalHydra
            from utils.global_map_manager import GlobalMapManager  # DualMap
            from utils.local_map_manager import LocalMapManager  # DualMap
            from utils.object_detector import Detector  # DualMap
            from utils.types import DataInput  # DualMap
            from utils.visualizer import ReRunVisualizer  # DualMap
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-dualmap] import failed: %s", e)
            return False

        force_cpu = os.environ.get("SCENE_CG_FORCE_CPU", "").strip().lower() in ("1", "true", "yes")
        device = str(self._dualmap_cfg.get("device") or ("cpu" if force_cpu or not torch.cuda.is_available() else "cuda"))
        out_dir = os.path.join(tempfile.gettempdir(), "scene-dualmap")
        os.makedirs(out_dir, exist_ok=True)
        weights = {k: os.environ.get(f"SCENE_DUALMAP_{k.upper()}_WEIGHTS", "").strip() or v
                   for k, v in _DEFAULT_WEIGHTS.items()}
        for name, path in weights.items():
            if not os.path.isfile(path):
                log.warning("[scene-dualmap] %s weights missing: %s", name, path)
                return False
        try:
            self._classes_file = self._write_classes_file()
        except (ValueError, FileNotFoundError) as e:
            log.warning("[scene-dualmap] %s", e)
            return False

        overrides = [
            "use_rerun=false", "use_parallel=false",
            f"run_local_mapping_only={str(not self._global_map).lower()}",
            "save_local_map=false", "save_global_map=false",
            "save_detection=false", "visualize_detection=false",
            "run_detection=true", f"output_path={out_dir}", f"device={device}",
            f"yolo.model_path={weights['yolo']}", f"sam.model_path={weights['sam']}",
            f"fastsam.model_path={weights['fastsam']}", f"clip.pretrained={weights['mobileclip']}",
            f"use_fastsam={str(self._use_fastsam).lower()}",
        ]
        # DualMap's object lifecycle is sized for a dataset replay that maps
        # every frame: an object needs `stable_num` observations before it counts
        # as stable, and one that leaves the `active_window_size` most recent
        # frames without getting there is deleted after `max_pending_count`
        # rounds. A robot mapping keyframes at walking pace sees each object far
        # fewer times, so on a deployment these three have to be scaled to the
        # observation rate or the map empties out behind the robot.
        overrides += [f"{k}={self._lifecycle_cfg[k]}" for k in sorted(self._lifecycle_cfg)]
        # DualMap's YAML uses paths relative to its checkout; Scene's cwd is not
        # that checkout, so every such path is pinned to the root here.
        classes = self._classes_file or os.path.join(root, "config", "class_list", "gpt_indoor_general.txt")
        overrides += [
            "yolo.use_given_classes=true", f"yolo.given_classes_path={classes}",
            f"logging_config={os.path.join(root, 'config', 'support_config', 'logging_config.yaml')}",
            f"config_file_path={os.path.join(root, 'config', 'actions.yaml')}",
        ]
        try:
            GlobalHydra.instance().clear()
            with initialize_config_dir(version_base=None, config_dir=os.path.join(root, "config")):
                cfg = compose(config_name="runner_dataset", overrides=overrides)
            # ReRunVisualizer is a process-wide singleton that the detector and
            # map manager fetch by calling it with no arguments; it has to be
            # configured once first. Rerun itself stays off.
            vis = ReRunVisualizer(cfg)
            vis.set_use_rerun(False)
            self._dm = Detector(cfg)
            self._lm = LocalMapManager(cfg)
            self._gm = GlobalMapManager(cfg) if self._global_map else None
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-dualmap] DualMap init failed: %s", e, exc_info=True)
            return False
        self._dm_data_input = DataInput
        self._device = device
        try:
            self._dm_names = [str(n) for n in self._dm.obj_classes.get_classes_arr()]
        except Exception as e:  # noqa: BLE001
            log.warning("[scene-dualmap] could not read DualMap's class list: %s", e)
            self._dm_names = []
        if not self._dm_names:
            log.warning("[scene-dualmap] empty vocabulary — every object would be dropped as unknown; not starting")
            return False
        log.info("[scene-dualmap] root=%s vocabulary=%d names (%s) device=%s", root, len(self._dm_names),
                 self._classes_file or "DualMap default list", device)
        return True

    # ── per-tick mapping ──────────────────────────────────────────────
    def _tick_locked(self) -> None:
        """Feed one RGB-D frame through DualMap's detector and local map, then
        mirror the local map into ``self._map_objects`` and the registry.
        Side effects: advances ``self._tick_idx`` and replaces ``self._map_objects``."""
        rgb_msg = self._rgb_msg()
        depth_msg = self._depth_msg()
        if rgb_msg is None or depth_msg is None:
            self._tick_idx += 1
            if self._tick_idx % 25 == 1:
                log.info("[scene-dualmap] waiting for frames: rgb=%s depth=%s",
                         "ok" if rgb_msg is not None else "none",
                         "ok" if depth_msg is not None else "none")
            return
        K = self._cam_info()
        if K is None or K.fx <= 0 or K.fy <= 0:
            self._tick_idx += 1
            if self._tick_idx % 25 == 1:
                log.info("[scene-dualmap] waiting for camera intrinsics")
            return
        import numpy as np

        bgr = _image_msg_to_bgr(rgb_msg)
        depth = _depth_msg_to_metres(depth_msg)
        if bgr is None or depth is None:
            self._tick_idx += 1
            self._undecodable += 1
            if self._undecodable % 25 == 1:
                log.warning("[scene-dualmap] undecodable frame (rgb=%s depth=%s), %d so far",
                            getattr(rgb_msg, "encoding", "?"), getattr(depth_msg, "encoding", "?"), self._undecodable)
            return
        try:
            pose = self._build_camera_to_map_transform()
        except Exception as e:  # noqa: BLE001
            log.debug("[scene-dualmap] transform unavailable: %s", e)
            pose = None
        if pose is None:
            self._tick_idx += 1
            if self._tick_idx % 25 == 1:
                log.info("[scene-dualmap] waiting for camera→map transform")
            return
        if not self._is_keyframe(rgb_msg, pose):
            self._tick_idx += 1
            return
        rgb = np.ascontiguousarray(bgr[:, :, ::-1], dtype=np.uint8)
        depth_m = np.ascontiguousarray(depth, dtype=np.float32)
        depth_m[~np.isfinite(depth_m)] = 0.0
        if depth_m.ndim == 2:
            depth_m = depth_m[:, :, None]  # DualMap's DataInput carries depth as H x W x 1
        K33 = np.array([[K.fx, 0.0, K.cx], [0.0, K.fy, K.cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        data = self._dm_data_input(
            idx=self._tick_idx, time_stamp=time.time(), color=rgb, depth=depth_m,
            color_name=f"tick{self._tick_idx:06d}", intrinsics=K33,
            pose=np.asarray(pose, dtype=np.float64),
        )
        t0 = time.monotonic()
        try:
            self._dm.set_data_input(data)
            self._dm.process_detections()
            self._dm.calculate_observations()
            obs = self._dm.get_curr_observations()
            self._dm.update_state()
            self._dm.update_data()
            self._lm.set_curr_idx(self._tick_idx)
            self._lm.process_observations(obs)
            # Both halves of DualMap. The local map associates observations
            # within a class, using CLIP similarity as well as geometry; a track
            # that becomes stable is promoted to the global map, which merges by
            # top-down 2D box overlap ACROSS classes — that is what collapses one
            # workstation reported as tv + speaker + desk into one object. Running
            # only the local half (as this adapter first did) throws that away and
            # loses every promoted object with it.
            promoted = self._lm.get_global_observations()
            self._lm.clear_global_observations()
            if promoted and self._gm is not None:
                self._gm.process_observations(promoted)
                self._promoted += len(promoted)
            self._keyframes += 1
            if self._merge_every > 0 and self._keyframes % self._merge_every == 0:
                before = list(self._lm.local_map)
                self._lm.merge_local_map()
                self._keep_merged_uids(before)
                log.info("[scene-dualmap] local-map merge after %d keyframes: %d -> %d objects",
                         self._keyframes, len(before), len(self._lm.local_map))
        except Exception as e:  # noqa: BLE001
            self._consecutive_failures += 1
            self._tick_idx += 1
            if self._consecutive_failures <= 3 or self._consecutive_failures % 50 == 0:
                log.warning("[scene-dualmap] frame %d failed (%d in a row): %s",
                            self._tick_idx - 1, self._consecutive_failures, e,
                            exc_info=self._consecutive_failures == 1)
            if "CUDA error" in str(e) and self._consecutive_failures >= _CUDA_FAILURE_LIMIT:
                # A CUDA error is sticky for the process: every later kernel fails too.
                # Stop ticking so the log says so once instead of forever, and keep the
                # last good map in the registry until the service is restarted.
                log.error("[scene-dualmap] %d consecutive CUDA failures — GPU context is lost; "
                          "stopping perception until Scene is restarted", self._consecutive_failures)
                self._stop.set()
            return
        self._consecutive_failures = 0
        objects, all_objects, dropped = [], [], {"points": 0, "unknown": 0, "observations": 0,
                                                 "unstable": 0, "floor": 0}
        # A promoted object leaves the local map, so the scene is the union of
        # both: what the robot is looking at now, and what it has already
        # committed to memory. Promotion keeps the uid (GlobalObject inherits it
        # from the observation), so an object does not change identity when it
        # crosses over.
        tracks = list(getattr(self._lm, "local_map", []) or [])
        if self._gm is not None:
            tracks += list(getattr(self._gm, "global_map", []) or [])
        for o in tracks:
            d = self._to_map_object(o, dropped)
            if d is None:
                continue
            all_objects.append(d)
            if d["num_detections"] < self._min_observations:
                dropped["observations"] += 1
                continue
            if self._stable_only and not d["stable"]:
                dropped["unstable"] += 1
                continue
            objects.append(d)
        self._map_objects = objects
        self._all_objects = all_objects
        n = len(objects)
        if n != self._last_lm_size or self._tick_idx % 25 == 0:
            log.info("[scene-dualmap] tick %d: %d observations, %d objects of %d tracks (%.2fs, %d frames skipped, "
                     "dropped: %d few-points %d unknown %d few-observations %d unstable "
                     "%d on-floor, %d promoted)",
                     self._tick_idx, len(obs) if obs is not None else 0, n, len(self._lm.local_map),
                     time.monotonic() - t0, self._skipped_frames,
                     dropped["points"], dropped["unknown"], dropped["observations"], dropped["unstable"],
                     dropped["floor"], self._promoted)
            self._last_lm_size = n
        self._tick_idx += 1
        self._project_to_registry()
        if self._tick_idx % _EXPORT_EVERY_TICKS == 0:
            self._export()

    # ── operator hooks (delete / flush) ───────────────────────────────
    # The base class edits `_map_objects` only; here that list is rebuilt from
    # DualMap's local map every tick, so the removal has to reach the local map.
    async def delete_object(self, object_id: str) -> None:
        """Drop the DualMap local-map objects bound to ``object_id`` so they do
        not come back on the next tick; the registry record is handled by the
        caller (``ObjectMutationCoordinator``)."""
        uuids = {u for u, oid in getattr(self, "_uuid_to_oid", {}).items() if oid == object_id}
        for u in uuids:
            self._uuid_to_oid.pop(u, None)
        if not uuids or self._lm is None:
            return

        def _drop() -> None:
            with self._inference_lock:
                self._lm.local_map = [o for o in self._lm.local_map if str(getattr(o, "uid", "")) not in uuids]
                self._map_objects = [o for o in self._map_objects if o["id"] not in uuids]

        await asyncio.get_running_loop().run_in_executor(None, _drop)

    async def reset_derived_state(self) -> None:
        """Empty DualMap's local map and the uuid bindings (flush)."""
        if hasattr(self, "_uuid_to_oid"):
            self._uuid_to_oid.clear()
        if self._lm is None:
            return

        def _reset() -> None:
            with self._inference_lock:
                self._lm.local_map = []
                self._map_objects = []

        await asyncio.get_running_loop().run_in_executor(None, _reset)

    def _is_keyframe(self, rgb_msg: Any, pose: Any) -> bool:
        """DualMap's keyframe rule on the live stream: skip a frame that is the
        same message as the last mapped one, and otherwise map it only when the
        camera translated ``keyframe_translation_m``, rotated
        ``keyframe_rotation_deg`` or ``keyframe_time_s`` elapsed. Updates the
        last-keyframe state when it returns True."""
        import numpy as np
        hdr = getattr(rgb_msg, "header", None)
        stamp = getattr(hdr, "stamp", None)
        key = (getattr(stamp, "sec", None), getattr(stamp, "nanosec", None)) if stamp is not None else None
        if key is not None and key == self._last_frame_key:
            return False
        now = time.monotonic()
        pose = np.asarray(pose, dtype=np.float64)
        keyframe = self._last_kf_pose is None or (now - self._last_kf_time) >= self._kf_time_s
        if not keyframe:
            translation = float(np.linalg.norm(pose[:3, 3] - self._last_kf_pose[:3, 3]))
            rel = self._last_kf_pose[:3, :3].T @ pose[:3, :3]
            cos_angle = max(-1.0, min(1.0, (float(np.trace(rel)) - 1.0) / 2.0))
            rotation_deg = float(np.degrees(np.arccos(cos_angle)))
            keyframe = translation >= self._kf_translation_m or rotation_deg >= self._kf_rotation_deg
        if not keyframe:
            self._skipped_frames += 1
            return False
        self._last_kf_pose, self._last_kf_time, self._last_frame_key = pose, now, key
        return True

    def _keep_merged_uids(self, before: list) -> None:
        """DualMap's merge builds a fresh LocalObject (new uid) from the observations
        of the tracks it fuses. Give each merged object the uid of the constituent
        that contributed most observations so the registry record (and its
        ``scene.object.<cls>_NNN`` id) survives the merge instead of churning."""
        obs_owner: dict[int, Any] = {}
        for obj in before:
            for ob in getattr(obj, "observations", []) or []:
                obs_owner[id(ob)] = obj
        for obj in self._lm.local_map:
            if not getattr(obj, "is_merged", False):
                continue
            votes: dict[Any, int] = {}
            for ob in getattr(obj, "observations", []) or []:
                owner = obs_owner.get(id(ob))
                if owner is not None:
                    votes[owner.uid] = votes.get(owner.uid, 0) + 1
            if votes:
                obj.uid = max(votes, key=votes.get)
            obj.is_merged = False  # consumed: the next merge round votes again

    @property
    def _floor_z_m(self) -> float:
        """Floor height: the backend's own setting when given, else the shared
        perception setting (env overrides are applied to that at start)."""
        if self._floor_z_override is not None:
            return float(self._floor_z_override)
        return float((getattr(self, "cfg", None) or {}).get("floor_z_m", 0.0))

    def _to_map_object(self, o: Any, dropped: Optional[dict] = None) -> Optional[dict]:
        """Shape one DualMap ``LocalObject`` like a ConceptGraphs map entry, or
        None when it has too few points or no usable label; ``dropped`` counts
        the reasons for the periodic diagnostic."""
        import numpy as np
        pcd = getattr(o, "pcd", None)
        if pcd is None:
            return None
        try:
            n_points = len(pcd.points)
        except Exception:  # noqa: BLE001
            return None
        if n_points < 4:
            if dropped is not None:
                dropped["points"] += 1
            return None
        # Floor gate: depth noise on the floor plane gets segmented and labelled
        # like an object ("bed", "desk") and then sits under the map at z<0. A
        # real object has some height above the floor; measure it at the 90th
        # percentile so a few stray points below the plane do not save a track.
        try:
            z = np.asarray(pcd.points, dtype=float)[:, 2]
            if float(np.percentile(z, 90)) - self._floor_z_m < 0.05:
                if dropped is not None:
                    dropped["floor"] += 1
                return None
        except Exception:  # noqa: BLE001
            pass
        cid = getattr(o, "class_id", None)
        name = "unknown"
        if cid is not None and 0 <= int(cid) < len(self._dm_names):
            name = self._dm_names[int(cid)]
        if name == "unknown" and not self._keep_unknown:
            if dropped is not None:
                dropped["unknown"] += 1
            return None
        bbox = getattr(o, "bbox", None)
        if bbox is None:
            try:
                bbox = pcd.get_axis_aligned_bounding_box()
            except Exception:  # noqa: BLE001
                return None
        conf = float(getattr(o, "max_prob", 0.0) or 0.0)
        uid = str(getattr(o, "uid", ""))
        clip_ft = getattr(o, "clip_ft", None)
        return {
            "id": uid,
            "class_name": name,
            "pcd": pcd,
            "bbox": bbox,
            "conf": [conf if conf > 0 else 0.5],
            "num_detections": int(getattr(o, "observed_num", 1) or 1),
            "n_points": int(n_points),
            "inst_color": _inst_color(uid),
            "clip_ft": np.asarray(clip_ft, dtype=np.float32) if clip_ft is not None else None,
            "stable": bool(getattr(o, "is_stable", True)),
        }

    # ── text embedding (MobileCLIP shared with the detector) ──────────
    def embed_text(self, texts: list[str]) -> Optional[list[list[float]]]:
        if self._dm is None or getattr(self._dm, "clip_model", None) is None:
            return None
        with self._inference_lock:
            return self._encode_text_nolock(texts)

    def _encode_text_nolock(self, texts: list[str]) -> Optional[list[list[float]]]:
        """Encode ``texts`` with the detector's MobileCLIP text tower (L2-normalised,
        one 512-d vector per text). Caller holds ``_inference_lock``; returns None
        when the encoder is unavailable or fails."""
        try:
            import torch
            tok = self._dm.clip_tokenizer(list(texts)).to(self._device)
            with torch.no_grad():
                feats = self._dm.clip_model.encode_text(tok)
                feats = feats / feats.norm(dim=-1, keepdim=True).clamp_min(1e-6)
            return feats.detach().cpu().tolist()
        except Exception as e:  # noqa: BLE001
            if not self._embed_warned:
                log.warning("[scene-dualmap] embed_text failed (text queries/persistence get no vectors): %s", e)
                self._embed_warned = True
            return None

    # ── Replica export ────────────────────────────────────────────────
    def _export(self) -> None:
        """Write the local map as a ConceptGraphs-style export when
        ``SCENE_EXPORT_CG_PICKLE`` is set; an empty map still writes a file so a
        scorer never reads a stale run. Labels are embedded with the ViT-B-32 text
        encoder (loaded once, cached) so the upstream scorer's CLIP matching
        reduces to label matching; when that encoder is unavailable the geometry
        is still written with zero features and an error is logged."""
        out_dir = os.environ.get("SCENE_EXPORT_CG_PICKLE", "").strip()
        if not out_dir:
            return
        exp = os.environ.get("SCENE_EXPORT_CG_EXP", "scene").strip() or "scene"
        model_name, pretrained = _EXPORT_CLIP
        pretrained = os.environ.get("SCENE_DUALMAP_EXPORT_CLIP", "").strip() or pretrained
        import numpy as np
        encoder = self._text_encoder(model_name, pretrained)
        objs = []
        for o in list(self._all_objects or self._map_objects):
            label = str(o["class_name"])
            feat = self._label_feature(encoder, label)
            pts = np.asarray(o["pcd"].points, dtype=np.float64)
            if pts.shape[0] == 0:
                continue
            cols = np.asarray(o["pcd"].colors) if o["pcd"].has_colors() else np.zeros_like(pts)
            lo, hi = pts.min(0), pts.max(0)
            corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
            objs.append({
                "pcd_np": pts, "pcd_color_np": cols, "bbox_np": corners, "clip_ft": feat,
                "class_name": label, "num_detections": o["num_detections"], "dualmap_uid": o["id"],
                "conf": list(o["conf"]),
            })

        class _Serializable(list):
            def to_serializable(self):
                return list(self)

        try:
            path = export_map_objects(_Serializable(objs), exp, out_dir, model_name, pretrained)
            log.info("[scene-dualmap] exported %d objects to %s", len(objs), path)
        except Exception as e:  # noqa: BLE001
            log.error("[scene-dualmap] export to %s failed: %s", out_dir, e)

    def _text_encoder(self, model_name: str, pretrained: str):
        """Load the export text encoder once; None (after one error log) when unavailable."""
        if self._export_encoder is not None:
            return self._export_encoder or None
        try:
            import open_clip
            model, _, _ = open_clip.create_model_and_transforms(model_name, pretrained=pretrained)
            model.eval()
            self._export_encoder = (model, open_clip.get_tokenizer(model_name))
        except Exception as e:  # noqa: BLE001
            log.error("[scene-dualmap] export text encoder %s (%s) unavailable: %s — writing zero features",
                      model_name, pretrained, e)
            self._export_encoder = ()  # remembered failure
            return None
        return self._export_encoder

    def _label_feature(self, encoder, label: str):
        """ViT-B-32 text feature of ``label`` (cached), or a zero vector without an encoder."""
        import numpy as np
        if label in self._export_cache:
            return self._export_cache[label]
        if encoder is None:
            feat = np.zeros(512, dtype=np.float32)
        else:
            import torch
            model, tok = encoder
            with torch.no_grad():
                tf = model.encode_text(tok([label]))
                feat = (tf / tf.norm(dim=-1, keepdim=True))[0].cpu().numpy().astype(np.float32)
        self._export_cache[label] = feat
        return feat
