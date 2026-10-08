# SPDX-License-Identifier: MulanPSL-2.0
"""Scene perception backed by DualMap (Eku127/DualMap, Apache-2.0).

Subclasses ``ConceptGraphsDetector`` for the backend-neutral parts (frames,
camera-to-map transform, registry reconciliation, 3D snapshot) and replaces
model loading, the per-tick mapping step, text embedding and the export.
``self._map_objects`` holds dicts shaped like ConceptGraphs map entries,
rebuilt from DualMap's maps every keyframe. DualMap's config is composed with
Hydra from its own YAML under ``SCENE_DUALMAP_ROOT`` with Scene's overrides.
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
    _header_stamp,
    _image_msg_to_bgr,
)

from .geometry_gates import KnownGround, fraction_on_known_ground  # noqa: E402

log = logging.getLogger("scene.dualmap")

_DEFAULT_ROOT = "/opt/dualmap"
_DEFAULT_WEIGHTS = {
    "yolo": "/opt/models/yolov8l-world.pt",
    "sam": "/opt/models/mobile_sam.pt",
    "fastsam": "/opt/models/dualmap/FastSAM-s.pt",
    "mobileclip": "/opt/models/dualmap/mobileclip_s2_datacompdr.bin",
}
# Text encoder for the Replica export; its scorer matches labels via CLIP text features.
_EXPORT_CLIP = ("ViT-B-32", "/opt/models/open_clip_pytorch_model.bin")

# Also exported at stop; periodic so a killed container leaves a recent export.
_EXPORT_EVERY_TICKS = 100
_CUDA_FAILURE_LIMIT = 20
_MERGE_OVERLAP = 0.5
# Thinner than any real object (keyboards, rugs): a slice of the floor.
_SLAB_THICKNESS_M = 0.005
_SLAB_OF_FLOOR_M = 0.20
_BELOW_FLOOR_M = 0.02
_OUTLIER_SIZE_FRACTION = 0.25
_OUTLIER_POINTS_FRACTION = 0.10


def _inst_color(uid: str) -> list[float]:
    """Stable pastel colour per object id for the 3D view."""
    h = hash(uid) & 0xFFFFFF
    return [0.35 + 0.65 * ((h >> 16) & 0xFF) / 255.0,
            0.35 + 0.65 * ((h >> 8) & 0xFF) / 255.0,
            0.35 + 0.65 * (h & 0xFF) / 255.0]


def _overlap_fraction(a: tuple, b: tuple) -> float:
    """Share of the smaller of two (min_xyz, max_xyz) boxes that their
    intersection covers. Extents are floored at 2 cm so flat tracks compare by area."""
    import numpy as np

    lo = np.maximum(a[0], b[0])
    hi = np.minimum(a[1], b[1])
    span = hi - lo
    if bool(np.any(span <= 0)):
        return 0.0
    inter = float(np.prod(span))
    va = float(np.prod(np.maximum(a[1] - a[0], 0.02)))
    vb = float(np.prod(np.maximum(b[1] - b[0], 0.02)))
    return inter / max(1e-9, min(va, vb))


class DualMapDetector(ConceptGraphsDetector):
    """Per-frame DualMap mapper that feeds Scene's ``ObjectRegistry``."""

    def __init__(self, *args: Any, dualmap_cfg: Optional[dict] = None, **kwargs: Any) -> None:
        """``dualmap_cfg`` is the manifest's ``perception.dualmap`` (see ``DUALMAP_KEYS``)."""
        super().__init__(*args, **kwargs)
        self._dualmap_cfg = dict(dualmap_cfg or {})
        self._dm_root = os.environ.get("SCENE_DUALMAP_ROOT", "").strip() or _DEFAULT_ROOT
        self._dm: Any = None      # DualMap Detector
        self._lm: Any = None      # DualMap LocalMapManager
        self._gm: Any = None      # DualMap GlobalMapManager
        self._dm_data_input: Any = None
        self._dm_names: list[str] = []
        self._map_objects = []
        self._known_gate = None
        self._min_mapped_fraction = float(self._dualmap_cfg.get("min_mapped_fraction", 0.5))
        self._keep_unknown = bool(self._dualmap_cfg.get("keep_unknown", False))
        # FastSAM only yields "unknown" objects, so it follows keep_unknown by default.
        self._use_fastsam = bool(self._dualmap_cfg.get("use_fastsam", self._keep_unknown))
        self._consecutive_failures = 0
        self.backend_name = "dualmap"
        self.last_tick_s = 0.0
        # Keyframe gate (DualMap core.check_keyframe): mapping every frame fragments objects.
        self._kf_translation_m = float(self._dualmap_cfg.get("keyframe_translation_m", 0.1))
        self._kf_rotation_deg = float(self._dualmap_cfg.get("keyframe_rotation_deg", 3.0))
        self._kf_time_s = float(self._dualmap_cfg.get("keyframe_time_s", 5.0))
        self._last_kf_pose: Any = None
        self._last_kf_time = 0.0
        self._last_frame_key: Any = None
        self._skipped_frames = 0
        # DualMap merges its local map only at end_process(); do it periodically instead.
        self._merge_every = int(self._dualmap_cfg.get("merge_every_keyframes", 20))
        self._keyframes = 0
        self._stable_only = bool(self._dualmap_cfg.get("stable_only", False))
        self._min_observations = int(self._dualmap_cfg.get("min_observations", 1))
        # Falls back to the shared perception floor_z_m (see _floor_z_m).
        self._floor_z_override = self._dualmap_cfg.get("floor_z_m")
        # Off by default: it also drops rugs and carpets.
        self._floor_gate = bool(self._dualmap_cfg.get("floor_gate", False))
        self._promoted = 0        # local tracks handed to the global map
        # DualMap's global map keeps only low-mobility tracks and drops the rest
        # from the local map, which suits navigation but not an inventory; off by default.
        self._global_map = bool(self._dualmap_cfg.get("global_map", False))
        # DualMap config overrides passed straight through.
        self._lifecycle_cfg = {k: int(self._dualmap_cfg[k]) for k in
                               ("stable_num", "active_window_size", "max_pending_count")
                               if self._dualmap_cfg.get(k) is not None}
        self._lifecycle_cfg.update({k: float(self._dualmap_cfg[k]) for k in
                                    ("downsample_voxel_size", "sim_threshold", "merge_sim_threshold")
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
        """Load DualMap and start the tick loop. A failed load raises: running
        without the requested backend would look like an empty room."""
        if self._task is not None:
            return
        loop = asyncio.get_running_loop()
        self._asyncio_loop = loop
        ok = await loop.run_in_executor(None, self._load_dualmap)
        if not ok:
            raise RuntimeError(
                "perception.backend is 'dualmap' but DualMap could not be loaded "
                "(see the log above). Build the image for this backend: "
                "SCENE_PERCEPTION_BACKEND=dualmap bash scripts/build.sh")
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
        """The vocabulary file DualMap reads: ``perception.dualmap.classes`` written
        to a temp file, or the file named by ``SCENE_DUALMAP_CLASSES``."""
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
        """Compose DualMap's Hydra config and build its detector and map managers.
        Runs in an executor thread; returns False after logging why."""
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
        overrides += [f"{k}={self._lifecycle_cfg[k]}" for k in sorted(self._lifecycle_cfg)]
        # DualMap's YAML paths are relative to its checkout, not Scene's cwd.
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
            # A singleton the detector and map managers fetch; configure it once.
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
        """Feed one keyframe through DualMap, then mirror its maps into
        ``self._map_objects`` and the registry."""
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
            pose = self._build_camera_to_map_transform(stamp=_header_stamp(depth_msg))
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
            # Stable local tracks are promoted to the global map, which merges across classes.
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
                # A CUDA error is sticky for the process; stop and keep the last good map.
                log.error("[scene-dualmap] %d consecutive CUDA failures — GPU context is lost; "
                          "stopping perception until Scene is restarted", self._consecutive_failures)
                self._stop.set()
            return
        self._consecutive_failures = 0
        objects, all_objects, dropped = [], [], {"points": 0, "unknown": 0, "observations": 0,
                                                 "unstable": 0, "floor": 0, "unmapped": 0,
                                                 "ground_slab": 0, "class_outlier": 0,
                                                 "overlapping": 0}
        known = self._known_ground()
        # Promotion moves a track to the global map with the same uid; show both maps.
        tracks = list(getattr(self._lm, "local_map", []) or [])
        if self._gm is not None:
            tracks += list(getattr(self._gm, "global_map", []) or [])
        for o in tracks:
            d = self._to_map_object(o, dropped)
            if d is None:
                continue
            all_objects.append(d)
            if known is not None and not self._on_known_ground(d, known):
                dropped["unmapped"] += 1
                continue
            if d["num_detections"] < self._min_observations:
                dropped["observations"] += 1
                continue
            if self._stable_only and not d["stable"]:
                dropped["unstable"] += 1
                continue
            objects.append(d)
        # Floor slabs go first: they carry the most points and would win any ranking.
        objects = self._drop_ground_slabs(objects, dropped)
        objects = self._drop_class_outliers(objects, dropped)
        objects = self._absorb_overlapping(objects, dropped)
        self._map_objects = objects
        self._all_objects = all_objects
        n = len(objects)
        self.last_tick_s = time.monotonic() - t0
        if n != self._last_lm_size or self._tick_idx % 25 == 0:
            log.info("[scene-dualmap] tick %d: %d observations, %d objects of %d tracks (%.2fs, %d frames skipped, "
                     "dropped: %d few-points %d unknown %d few-observations %d unstable "
                     "%d on-floor %d unmapped %d ground-slab %d class-outlier "
                     "%d overlapping, %d promoted)",
                     self._tick_idx, len(obs) if obs is not None else 0, n, len(self._lm.local_map),
                     time.monotonic() - t0, self._skipped_frames,
                     dropped["points"], dropped["unknown"], dropped["observations"], dropped["unstable"],
                     dropped["floor"], dropped["unmapped"], dropped["ground_slab"],
                     dropped["class_outlier"], dropped["overlapping"], self._promoted)
            self._last_lm_size = n
        self._tick_idx += 1
        self._project_to_registry()
        if self._tick_idx % _EXPORT_EVERY_TICKS == 0:
            self._export()

    # ── operator hooks (delete / flush) ───────────────────────────────
    # `_map_objects` is rebuilt from DualMap's maps each tick, so edits go there.
    async def delete_object(self, object_id: str) -> None:
        """Drop the DualMap tracks bound to ``object_id``; the caller removes the registry record."""
        uuids = {u for u, oid in getattr(self, "_uuid_to_oid", {}).items() if oid == object_id}
        for u in uuids:
            self._uuid_to_oid.pop(u, None)
        if not uuids or self._lm is None:
            return

        def _drop() -> None:
            with self._inference_lock:
                self._lm.local_map = [o for o in self._lm.local_map
                                      if str(getattr(o, "uid", "")) not in uuids]
                if self._gm is not None:
                    self._gm.global_map = [o for o in self._gm.global_map
                                           if str(getattr(o, "uid", "")) not in uuids]
                self._map_objects = [o for o in self._map_objects if o["id"] not in uuids]

        await asyncio.get_running_loop().run_in_executor(None, _drop)

    async def reset_derived_state(self) -> None:
        """Empty both DualMap maps and the uuid bindings (flush)."""
        if hasattr(self, "_uuid_to_oid"):
            self._uuid_to_oid.clear()
        if self._lm is None:
            return

        def _reset() -> None:
            with self._inference_lock:
                self._lm.local_map = []
                if self._gm is not None:
                    self._gm.global_map = []
                self._map_objects = []

        await asyncio.get_running_loop().run_in_executor(None, _reset)

    def _is_keyframe(self, rgb_msg: Any, pose: Any) -> bool:
        """True for a new frame after enough translation, rotation or time; records it."""
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
        """Give each merged object the uid of the track that contributed most
        observations, so its registry id survives DualMap's merge."""
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
        """Backend floor_z_m if set, else the shared perception floor_z_m."""
        if self._floor_z_override is not None:
            return float(self._floor_z_override)
        return float((getattr(self, "cfg", None) or {}).get("floor_z_m", 0.0))

    def _to_map_object(self, o: Any, dropped: Optional[dict] = None) -> Optional[dict]:
        """One DualMap track as a map entry, or None; ``dropped`` counts why."""
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
        # Floor gate: floor noise has no height; 90th percentile ignores stray points.
        try:
            z = np.asarray(pcd.points, dtype=float)[:, 2] if self._floor_gate else None
            if z is not None and float(np.percentile(z, 90)) - self._floor_z_m < 0.05:
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
        conf = _track_confidence(o)
        uid = str(getattr(o, "uid", ""))
        clip_ft = getattr(o, "clip_ft", None)
        return {
            "id": uid,
            "class_name": name,
            "pcd": pcd,
            "bbox": bbox,
            "conf": [conf],
            "num_detections": int(getattr(o, "observed_num", 1) or 1),
            "n_points": int(n_points),
            "inst_color": _inst_color(uid),
            "clip_ft": np.asarray(clip_ft, dtype=np.float32) if clip_ft is not None else None,
            "stable": bool(getattr(o, "is_stable", True)),
        }

    # ── nested copies of one object ───────────────────────────────────
    def _drop_ground_slabs(self, objects: list, dropped: dict) -> list:
        """Drop tracks that are thin and on the floor, or mostly below it."""
        import numpy as np

        kept = []
        for o in objects:
            pts = np.asarray(o["pcd"].points, dtype=np.float64)
            if pts.shape[0]:
                z0, z1 = float(pts[:, 2].min()), float(pts[:, 2].max())
                zc = float(np.median(pts[:, 2]))
                thin_on_floor = ((z1 - z0) < _SLAB_THICKNESS_M
                                 and abs(z0 - self._floor_z_m) < _SLAB_OF_FLOOR_M)
                mostly_below_floor = zc < self._floor_z_m - _BELOW_FLOOR_M
                if thin_on_floor or mostly_below_floor:
                    dropped["ground_slab"] += 1
                    continue
            kept.append(o)
        return kept

    def _drop_class_outliers(self, objects: list, dropped: dict) -> list:
        """Drop a track far smaller, on far less evidence, than the largest of its
        class (classes with at least three members only)."""
        import numpy as np

        by_class: dict[str, list] = {}
        boxes: dict[int, tuple] = {}
        for o in objects:
            pts = np.asarray(o["pcd"].points, dtype=np.float64)
            if not pts.shape[0]:
                continue
            boxes[id(o)] = (float(np.max(pts.max(0) - pts.min(0))), int(o["n_points"]))
            by_class.setdefault(o["class_name"], []).append(o)
        drop: set[int] = set()
        for group in by_class.values():
            if len(group) < 3:
                continue
            big_d = max(boxes[id(o)][0] for o in group)
            big_p = max(boxes[id(o)][1] for o in group)
            for o in group:
                d, n = boxes[id(o)]
                if d < _OUTLIER_SIZE_FRACTION * big_d and n < _OUTLIER_POINTS_FRACTION * big_p:
                    drop.add(id(o))
        if not drop:
            return objects
        dropped["class_outlier"] += len(drop)
        return [o for o in objects if id(o) not in drop]

    def _absorb_overlapping(self, objects: list, dropped: dict) -> list:
        """Absorb a same-class track that claims a bigger one's space: its centre
        is inside, or the boxes overlap by more than half of the smaller."""
        import numpy as np

        by_class: dict[str, list] = {}
        for o in objects:
            by_class.setdefault(o["class_name"], []).append(o)
        absorbed: set[int] = set()
        for group in by_class.values():
            if len(group) < 2:
                continue
            # Bigger first: a copy is absorbed into the fullest version of itself.
            order = sorted(group, key=lambda o: -int(o["n_points"]))
            boxes = []
            for o in order:
                pts = np.asarray(o["pcd"].points, dtype=np.float64)
                boxes.append((pts.min(0), pts.max(0)) if pts.shape[0] else None)
            centres = [None if b is None else (b[0] + b[1]) / 2.0 for b in boxes]
            for i, keeper in enumerate(order):
                if id(keeper) in absorbed or boxes[i] is None:
                    continue
                lo, hi = boxes[i]
                for j in range(i + 1, len(order)):
                    if id(order[j]) in absorbed or boxes[j] is None:
                        continue
                    inside = bool(np.all(centres[j] >= lo) and np.all(centres[j] <= hi))
                    if inside or _overlap_fraction(boxes[i], boxes[j]) > _MERGE_OVERLAP:
                        absorbed.add(id(order[j]))
        if not absorbed:
            return objects
        dropped["overlapping"] += len(absorbed)
        return [o for o in objects if id(o) not in absorbed]

    # ── occupancy consistency ─────────────────────────────────────────
    def _known_ground(self):
        """The occupancy grid as a looked-here mask; see geometry_gates."""
        if self._known_gate is None:
            self._known_gate = KnownGround(self._hub)
        return self._known_gate.current()

    def _on_known_ground(self, obj: dict, known) -> bool:
        """Whether enough of the object lies on ground the map has observed
        (kept when the map has no opinion)."""
        import numpy as np

        pts = np.asarray(obj["pcd"].points, dtype=np.float64)
        frac = fraction_on_known_ground(pts[:, :2] if pts.ndim == 2 and pts.shape[0] else pts, known)
        return True if frac is None else frac >= self._min_mapped_fraction

    # ── text embedding (MobileCLIP shared with the detector) ──────────
    def embed_text(self, texts: list[str]) -> Optional[list[list[float]]]:
        if self._dm is None or getattr(self._dm, "clip_model", None) is None:
            return None
        with self._inference_lock:
            return self._encode_text_nolock(texts)

    def _encode_text_nolock(self, texts: list[str]) -> Optional[list[list[float]]]:
        """L2-normalised MobileCLIP text features; caller holds ``_inference_lock``."""
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
        """Write a ConceptGraphs-style export when ``SCENE_EXPORT_CG_PICKLE`` is set
        (also for an empty map, so a scorer never reads a stale run)."""
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


def _track_confidence(track: Any) -> float:
    """Label confidence: the class filter's max_prob, else class_probs above the
    uniform prior, else the latest detection score, else 0.0 (no evidence)."""
    prob = float(getattr(track, "max_prob", 0.0) or 0.0)
    if prob > 0.0:
        return min(1.0, prob)

    probs = getattr(track, "class_probs", None)
    if probs is not None:
        try:
            import numpy as _np

            arr = _np.asarray(probs, dtype=float)
            if arr.size:
                top = float(arr.max())
                if top > (1.0 / arr.size) + 1e-6:
                    return min(1.0, top)
        except Exception:  # noqa: BLE001
            pass

    try:
        latest = track.get_latest_observation()
        score = float(getattr(latest, "conf", 0.0) or 0.0)
        if score > 0.0:
            return min(1.0, score)
    except Exception:  # noqa: BLE001
        pass

    return 0.0
