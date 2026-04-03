#!/usr/bin/env python3
"""Visualization node: streams env camera + real-time detection overlays to Rerun.

Connects to env_node via gRPC, polls observations at target FPS, and logs to a
live Rerun viewer.  A background thread runs YOLO-World (default) or GroundingDINO
on the latest camera frame for detection overlays.

Usage:
  python3 -m maniskill_vla_demo.viz_node
  python3 -m maniskill_vla_demo.viz_node --detect-query "cube . red cup . box"

Env:
  ROBONIX_ATLAS     gRPC address of robonix-atlas  (default: localhost:50051)
  RERUN_GRPC_PORT    port for the Rerun gRPC data server  (default: 9877)
  VIZ_DETECT_QUERY   detection query (default: "object . cup . box")
  VIZ_DETECT_BACKEND yolo_world | grounding_dino  (default: yolo_world)
  VIZ_YOLO_WEIGHTS   ultralytics YOLO-World weights (default: yolov8s-worldv2.pt)
  VIZ_DETECT_DEVICE  detector device: auto|cuda|cpu  (default: auto)
  VIZ_DETECT_FP16    use FP16 on CUDA (default: 1)
"""
import argparse
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np


def _ensure_proto_paths() -> None:
    pkg = Path(__file__).resolve().parent
    sys.path.insert(0, str(pkg))
    d = pkg
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent


_ensure_proto_paths()

import grpc  # noqa: E402
import robonix_runtime_pb2 as pb  # noqa: E402
import robonix_runtime_pb2_grpc as pb_grpc  # noqa: E402
import maniskill_env_pb2 as env_pb  # noqa: E402
import maniskill_env_pb2_grpc as env_pb_grpc  # noqa: E402
from maniskill_vla_demo.yolo_weights import resolve_yolo_world_weights  # noqa: E402

# ── Transport selection ───────────────────────────────────────────────────────

_IOX2_TRANSPORT = os.environ.get("OBS_TRANSPORT", "grpc").strip().lower() == "iceoryx2"


def _discover_env_grpc(robonix_stub) -> str:
    for attempt in range(60):
        try:
            resp = robonix_stub.NegotiateChannel(pb.NegotiateChannelRequest(
                consumer_id="com.robonix.demo.viz",
                provider_node_id="com.robonix.demo.maniskill",
                interface_name="env_data",
                transport="grpc",
            ))
            return resp.endpoint
        except grpc.RpcError:
            if attempt < 59:
                print(f"[viz] waiting for env_node… ({attempt+1}/60)", file=sys.stderr)
                time.sleep(2)
            else:
                raise


# ── Depth-based 3-D point cloud accumulator ──────────────────────────────────
# Back-projects depth images with ground-truth camera poses to build an
# incremental voxelised world-frame point cloud — no external SLAM required.


class DepthCloudAccumulator:
    """Accumulate a coloured 3-D point cloud from successive depth frames.

    Each call to ``add_frame`` back-projects the depth image into camera space,
    transforms to world space using the ground-truth camera pose, and inserts
    the resulting points into a voxel hash map.  The newest colour wins when
    two samples fall in the same voxel.
    """

    VOXEL_SIZE = 0.03   # metres — 3 cm resolution
    DEPTH_STEP = 4      # process every Nth pixel in u and v (4 → 160×120 @ 640×480)
    MIN_DEPTH  = 0.15   # metres — ignore artefacts very close to the lens
    MAX_DEPTH  = 8.0    # metres — clip far background

    def __init__(self) -> None:
        self._voxels: dict = {}   # (xi, yi, zi) → np.ndarray shape (3,) uint8 RGB
        self._lock = threading.Lock()
        self._n_frames = 0

    def add_frame(self, obs) -> None:
        """Insert one observation into the accumulator (fast, vectorised)."""
        if not obs.depth or obs.width == 0 or obs.height == 0:
            return
        if len(obs.camera_pose) < 16 or obs.fx <= 0:
            return

        w, h = obs.width, obs.height
        fx, fy, cx, cy = obs.fx, obs.fy, obs.cx, obs.cy
        step = self.DEPTH_STEP

        depth = np.frombuffer(obs.depth, dtype=np.float32).reshape(h, w)

        has_rgb = len(obs.rgb) > 0
        if has_rgb:
            rgb_img = np.frombuffer(obs.rgb, dtype=np.uint8).reshape(h, w, 3)

        # Camera-to-world transform (row-major 4×4)
        T = np.array(obs.camera_pose, dtype=np.float64).reshape(4, 4)
        R, t = T[:3, :3], T[:3, 3]

        # Build downsampled pixel grid
        u = np.arange(0, w, step)
        v = np.arange(0, h, step)
        uu, vv = np.meshgrid(u, v)

        z = depth[vv, uu]
        valid = np.isfinite(z) & (z > self.MIN_DEPTH) & (z < self.MAX_DEPTH)
        if not valid.any():
            return

        z  = z[valid].astype(np.float64)
        uu_v = uu[valid]
        vv_v = vv[valid]

        # Back-project to camera space (standard OpenCV pinhole: +Z forward)
        x = (uu_v - cx) * z / fx
        y = (vv_v - cy) * z / fy
        pts_cam = np.stack([x, y, z], axis=1)          # (N, 3)

        # Transform to world space
        pts_world = (R @ pts_cam.T).T + t              # (N, 3)

        # Voxel indices
        vi = np.floor(pts_world / self.VOXEL_SIZE).astype(np.int32)

        colors = (rgb_img[vv_v, uu_v] if has_rgb
                  else np.full((len(z), 3), 180, dtype=np.uint8))

        with self._lock:
            for i in range(len(vi)):
                key = (vi[i, 0], vi[i, 1], vi[i, 2])
                self._voxels[key] = colors[i]
        self._n_frames += 1

    def get_cloud(self) -> "tuple[np.ndarray | None, np.ndarray | None]":
        """Return (xyz, rgb) numpy arrays for the current accumulated cloud."""
        with self._lock:
            if not self._voxels:
                return None, None
            keys = np.array(list(self._voxels.keys()), dtype=np.float32)
            cols = np.array(list(self._voxels.values()), dtype=np.uint8)
        # Centre of each voxel in world space
        pts = (keys + 0.5) * self.VOXEL_SIZE
        return pts, cols


# ── Detector (runs in background thread) ─────────────────────────────────────


def _viz_classes_from_query(q: str) -> list[str]:
    parts = [p.strip() for p in q.replace(",", ".").split(".")]
    return [p for p in parts if p] or ["object"]


class RealtimeDetector:
    """YOLO-World (default) or GroundingDINO; runs on frames fed by the main loop."""

    def __init__(self, query: str, threshold: float, device: str, use_fp16: bool):
        self._query = query
        self._threshold = threshold
        self._device_pref = device
        self._use_fp16 = use_fp16

        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None       # latest frame (set by main loop)
        self._frame_seq: int = -1                    # sequence id of latest frame
        self._processed_seq: int = -1                # last sequence id we processed
        self._results: list[dict] = []               # latest detection results
        self._results_rgb: np.ndarray | None = None  # the frame these results match

        self._processor = None
        self._model = None
        self._yolo_model = None
        self._yolo_device: str | int = "cpu"
        self._yolo_half = False
        self._backend: str | None = None  # "yolo_world" | "grounding_dino"
        self._ready = threading.Event()
        self._stop = threading.Event()

    def start(self) -> None:
        t = threading.Thread(target=self._loop, daemon=True, name="viz-detector")
        t.start()

    def stop(self) -> None:
        self._stop.set()

    def feed_frame(self, rgb: np.ndarray, seq: int) -> None:
        """Called by main loop to provide the latest camera frame."""
        with self._lock:
            self._frame = rgb
            self._frame_seq = seq

    def get_results(self) -> tuple[list[dict], np.ndarray | None]:
        """Returns (detections, rgb_at_detection_time). Non-blocking."""
        with self._lock:
            return list(self._results), self._results_rgb

    def _loop(self) -> None:
        self._load_model()
        self._ready.set()
        print("[viz-detect] detector thread running", file=sys.stderr)

        while not self._stop.is_set():
            with self._lock:
                if self._frame is None or self._frame_seq == self._processed_seq:
                    frame, seq = None, -1
                else:
                    frame = self._frame.copy()
                    seq = self._frame_seq

            if frame is None:
                time.sleep(0.01)
                continue

            detections = self._run(frame)

            with self._lock:
                self._results = detections
                self._results_rgb = frame
                self._processed_seq = seq

    # Minimum free GPU memory (MB) required before loading detector on CUDA.
    _MIN_GPU_MB = int(os.environ.get("VIZ_DETECT_MIN_GPU_MB", "400"))

    def _pick_device(self) -> str:
        import torch

        d = self._device_pref
        if d == "cpu":
            return "cpu"
        if d == "cuda":
            return "cuda" if torch.cuda.is_available() else "cpu"
        if torch.cuda.is_available():
            free_mb = torch.cuda.mem_get_info()[0] / 1024 / 1024
            if free_mb < self._MIN_GPU_MB:
                print(f"[viz-detect] GPU free={free_mb:.0f} MB < {self._MIN_GPU_MB} MB "
                      f"→ using CPU", file=sys.stderr)
                return "cpu"
            return "cuda"
        return "cpu"

    def _load_model(self) -> None:
        be = os.environ.get("VIZ_DETECT_BACKEND", "yolo_world").strip().lower()
        if be in ("grounding_dino", "grounding-dino", "gdino", "dino"):
            self._load_grounding_dino()
        else:
            self._load_yolo_world()
            if self._yolo_model is None:
                print("[viz-detect] YOLO-World failed → trying GroundingDINO", file=sys.stderr)
                self._load_grounding_dino()

    def _load_yolo_world(self) -> None:
        try:
            from ultralytics import YOLO
            import torch
            from PIL import Image as PILImage
        except ImportError:
            print("[viz-detect] ultralytics not installed — skipping YOLO-World", file=sys.stderr)
            return

        device = self._pick_device()
        dev_arg: str | int = 0 if device == "cuda" else "cpu"
        self._yolo_half = bool(device == "cuda" and self._use_fp16)
        weights = resolve_yolo_world_weights(os.environ.get("VIZ_YOLO_WEIGHTS"))
        print(f"[viz-detect] loading YOLO-World {weights} on {device}…", file=sys.stderr)
        try:
            self._yolo_model = YOLO(weights)
        except OSError as e:
            print(f"[viz-detect] YOLO load error: {e}", file=sys.stderr)
            return

        dummy = PILImage.new("RGB", (320, 240))
        actual_classes = _viz_classes_from_query(self._query)
        try:
            # set_classes MUST be called before predict().
            # predict() moves the YOLO/CLIP model to CUDA; calling set_classes
            # afterwards produces a device mismatch because the tokenizer still
            # emits CPU tensors while token_embedding.weight is already on cuda:0.
            self._yolo_model.set_classes(actual_classes)
            self._yolo_model.predict(
                dummy, conf=0.5, verbose=False, device=dev_arg, half=self._yolo_half)
        except (RuntimeError, torch.OutOfMemoryError) as exc:
            if device == "cuda" and "out of memory" in str(exc).lower():
                print("[viz-detect] YOLO CUDA OOM → CPU", file=sys.stderr)
                torch.cuda.empty_cache()
                device = "cpu"
                dev_arg = "cpu"
                self._yolo_half = False
                self._yolo_model.set_classes(actual_classes)
                self._yolo_model.predict(
                    dummy, conf=0.5, verbose=False, device="cpu", half=False)
            else:
                raise

        self._yolo_device = dev_arg
        self._yolo_classes = actual_classes  # frozen; never re-encoded at runtime
        self._backend = "yolo_world"
        print(f"[viz-detect] YOLO-World ready (classes={actual_classes})", file=sys.stderr)

    def _load_grounding_dino(self) -> None:
        try:
            from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
            import torch
        except ImportError:
            print("[viz-detect] transformers/torch not installed — detection disabled",
                  file=sys.stderr)
            return

        device_str = self._pick_device()
        model_id = os.environ.get("VIZ_DETECT_MODEL", "IDEA-Research/grounding-dino-tiny")
        print(f"[viz-detect] loading {model_id} on {device_str}…", file=sys.stderr)

        self._processor = AutoProcessor.from_pretrained(model_id)
        model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id)

        if device_str == "cuda" and self._use_fp16:
            model = model.half()

        try:
            model = model.to(device_str)
        except (RuntimeError, torch.OutOfMemoryError) as exc:
            if device_str != "cpu" and "out of memory" in str(exc).lower():
                print("[viz-detect] CUDA OOM → falling back to CPU", file=sys.stderr)
                torch.cuda.empty_cache()
                device_str = "cpu"
                model = model.float().to("cpu")
            else:
                raise

        model.eval()
        self._model = model
        self._device = device_str
        self._backend = "grounding_dino"
        print(f"[viz-detect] GroundingDINO ready (device={device_str})", file=sys.stderr)

    def _run(self, rgb: np.ndarray) -> list[dict]:
        if self._backend == "yolo_world" and self._yolo_model is not None:
            return self._run_yolo(rgb)
        if self._processor is None or self._model is None:
            return []
        return self._run_grounding_dino(rgb)

    def _run_yolo(self, rgb: np.ndarray) -> list[dict]:
        # Classes are set once during _load_yolo_world; never re-call set_classes
        # here — repeated CLIP text encoding from a background thread causes a
        # CUDA F.embedding crash.
        classes = self._yolo_classes
        results = self._yolo_model.predict(
            rgb,
            conf=self._threshold,
            verbose=False,
            device=self._yolo_device,
            half=self._yolo_half,
        )
        r = results[0]
        if r.boxes is None or len(r.boxes) == 0:
            return []
        det = []
        boxes = r.boxes
        for i in range(len(boxes)):
            ci = int(boxes.cls[i].cpu())
            label = classes[ci] if 0 <= ci < len(classes) else str(ci)
            det.append({
                "bbox": boxes.xyxy[i].cpu().tolist(),
                "label": label,
                "score": float(boxes.conf[i].cpu()),
            })
        return det

    def _run_grounding_dino(self, rgb: np.ndarray) -> list[dict]:
        from PIL import Image as PILImage
        import torch

        pil = PILImage.fromarray(rgb)
        inputs = self._processor(images=pil, text=self._query, return_tensors="pt")
        device = next(self._model.parameters()).device
        inputs = {k: v.to(device) if isinstance(v, torch.Tensor) else v
                  for k, v in inputs.items()}

        if next(self._model.parameters()).dtype == torch.float16:
            inputs = {k: v.half() if isinstance(v, torch.Tensor) and v.is_floating_point() else v
                      for k, v in inputs.items()}

        try:
            with torch.no_grad():
                outputs = self._model(**inputs)
        except RuntimeError as e:
            if "Float" in str(e) and "Half" in str(e):
                print("[viz-detect] dtype mismatch, retrying FP32", file=sys.stderr)
                self._model = self._model.float()
                inputs = {k: v.float() if isinstance(v, torch.Tensor) and v.is_floating_point() else v
                          for k, v in inputs.items()}
                with torch.no_grad():
                    outputs = self._model(**inputs)
            else:
                raise

        results = self._processor.post_process_grounded_object_detection(
            outputs, inputs["input_ids"],
            threshold=self._threshold,
            target_sizes=[pil.size[::-1]],
        )[0]

        boxes = results["boxes"].cpu().numpy()
        labels = [str(l) for l in results["labels"]]
        scores = results["scores"].cpu().numpy()

        detections = []
        for i in range(len(boxes)):
            detections.append({
                "bbox": boxes[i].tolist(),
                "label": labels[i],
                "score": float(scores[i]),
            })
        return detections


# ── Main ─────────────────────────────────────────────────────────────────────


def main() -> None:
    global _IOX2_TRANSPORT
    parser = argparse.ArgumentParser(description="Rerun visualizer for maniskill_vla_demo")
    parser.add_argument("--detect-query",
                        default=os.environ.get("VIZ_DETECT_QUERY", "object . cup . box"),
                        help="Detection classes (dot-separated; YOLO-World / GroundingDINO)")
    parser.add_argument("--fps", type=float, default=10.0, help="target polling rate (Hz)")
    parser.add_argument("--threshold", type=float, default=0.25,
                        help="detection confidence threshold")
    parser.add_argument("--no-detect", action="store_true",
                        help="disable detection overlay entirely")
    args = parser.parse_args()

    try:
        import rerun as rr
    except ImportError:
        print("[viz] rerun-sdk not installed.  pip install rerun-sdk>=0.22", file=sys.stderr)
        sys.exit(1)

    rr.init("maniskill_vla_demo")

    grpc_port = int(os.environ.get("RERUN_GRPC_PORT", "9877"))
    grpc_addr = rr.serve_grpc(grpc_port=grpc_port)
    print(f"[viz] gRPC data server listening at {grpc_addr}", file=sys.stderr)
    print(f"[viz] connect with:  rerun \"{grpc_addr.replace('0.0.0.0', 'localhost')}\"",
          file=sys.stderr)

    try:
        from rerun.blueprint import (
            Blueprint, Horizontal, Vertical,
            Spatial2DView, Spatial3DView, TimeSeriesView,
        )
        # Layout:
        #  Left column  – live camera feeds (RGB, depth)
        #  Middle column – object detections + joint telemetry
        #  Right column  – incremental 3-D point cloud (depth back-projection)

        # Build 3-D view; try to set white background (Rerun ≥ 0.18).
        try:
            cloud_view = Spatial3DView(
                name="3-D Point Cloud",
                origin="world",
                background=[255, 255, 255],
            )
        except TypeError:
            cloud_view = Spatial3DView(name="3-D Point Cloud", origin="world")

        blueprint = Blueprint(
            Horizontal(
                Vertical(
                    Spatial2DView(name="Robot Camera", origin="camera/rgb"),
                    Spatial2DView(name="Depth (m)",    origin="camera/depth"),
                ),
                Vertical(
                    TimeSeriesView(name="Joint Positions", origin="robot/joint"),
                    Spatial2DView(name="Detections",
                                  origin="camera/detections"),
                ),
                Vertical(cloud_view),
            ),
            collapse_panels=True,
        )
        rr.send_blueprint(blueprint)
    except Exception:
        pass

    # ── Connect to robonix-atlas ─────────────────────────────────────────────
    server_addr = os.environ.get("ROBONIX_ATLAS", "localhost:50051")
    robonix_channel = grpc.insecure_channel(server_addr)
    robonix_stub = pb_grpc.RobonixRuntimeStub(robonix_channel)

    try:
        robonix_stub.RegisterNode(pb.RegisterNodeRequest(
            node_id="com.robonix.demo.viz",
            namespace="robonix/viz",
            kind="primitive",
        ))
    except grpc.RpcError as e:
        print(f"[viz] could not register with server (non-fatal): {e.details()}", file=sys.stderr)

    # ── Transport: gRPC or iceoryx2 ───────────────────────────────────────────
    env_stub = None
    _iox2_rgb_sub = None
    _iox2_dep_sub = None
    _iox2_node = None

    if _IOX2_TRANSPORT:
        try:
            import iceoryx2 as iox2
            from maniskill_vla_demo.iox2_types import (
                RgbCameraFrame, DepthCameraFrame,
                IOX2_SERVICE_RGB, IOX2_SERVICE_DEPTH,
            )
            _iox2_node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
            _iox2_rgb_sub = (
                _iox2_node.service_builder(iox2.ServiceName.new(IOX2_SERVICE_RGB))
                          .publish_subscribe(RgbCameraFrame)
                          .open_or_create()
                          .subscriber_builder().create()
            )
            _iox2_dep_sub = (
                _iox2_node.service_builder(iox2.ServiceName.new(IOX2_SERVICE_DEPTH))
                          .publish_subscribe(DepthCameraFrame)
                          .open_or_create()
                          .subscriber_builder().create()
            )
            print("[viz] iceoryx2 subscriber ready (robonix/camera/{rgb,depth})", file=sys.stderr)
        except Exception as exc:
            print(f"[viz] iceoryx2 init failed, falling back to gRPC: {exc}", file=sys.stderr)
            _IOX2_TRANSPORT = False  # type: ignore[assignment]

    if not _IOX2_TRANSPORT:
        print("[viz] waiting for env_node gRPC…", file=sys.stderr)
        endpoint = _discover_env_grpc(robonix_stub)
        env_channel = grpc.insecure_channel(endpoint)
        env_stub = env_pb_grpc.EnvDataServiceStub(env_channel)
        print(f"[viz] streaming from env at {endpoint}", file=sys.stderr)

    # ── 3-D point cloud accumulator (depth back-projection) ──────────────────
    cloud_acc = DepthCloudAccumulator()
    _CLOUD_LOG_EVERY = 15   # log to Rerun every N frames (≈0.5 s at 30 fps)

    # ── Start real-time detector ──────────────────────────────────────────────
    detector: RealtimeDetector | None = None
    if not args.no_detect:
        device = os.environ.get("VIZ_DETECT_DEVICE", "auto")
        fp16 = os.environ.get("VIZ_DETECT_FP16", "1") in ("1", "true", "True")
        detector = RealtimeDetector(args.detect_query, args.threshold, device, fp16)
        detector.start()

    # ── Streaming loop ────────────────────────────────────────────────────────
    interval = 1.0 / max(1.0, args.fps)
    frame = 0
    env_id = os.environ.get("MANISKILL_ENV_ID", "?")
    last_det_seq = -1
    print(f"[viz] streaming at {args.fps:.0f} fps  env={env_id}  transport={'iceoryx2' if _IOX2_TRANSPORT else 'grpc'}", file=sys.stderr)

    while True:
        t0 = time.monotonic()

        if _IOX2_TRANSPORT:
            # ── iceoryx2 receive ─────────────────────────────────────────────
            rgb_sample = _iox2_rgb_sub.receive() if _iox2_rgb_sub else None
            if rgb_sample is None:
                time.sleep(0.005)
                continue
            rf = rgb_sample.payload().contents          # RgbCameraFrame

            # Build a simple namespace that downstream code reads like a proto obs
            class _Obs:  # noqa: N801
                pass
            obs = _Obs()
            obs.width    = rf.width
            obs.height   = rf.height
            obs.rgb      = bytes(rf.pixels[: rf.width * rf.height * rf.channels])
            obs.depth    = b""
            obs.fx = obs.fy = obs.cx = obs.cy = 0.0
            obs.proprio  = []
            obs.goal_pos = []
            obs.camera_pose = []

            dep_sample = _iox2_dep_sub.receive() if _iox2_dep_sub else None
            if dep_sample is not None:
                df = dep_sample.payload().contents      # DepthCameraFrame
                n_dep = df.width * df.height * 4
                obs.depth = bytes(df.pixels[:n_dep])
        else:
            # ── gRPC receive ─────────────────────────────────────────────────
            try:
                obs = env_stub.GetObs(env_pb.Empty())
            except grpc.RpcError as e:
                print(f"[viz] GetObs error: {e.details()}", file=sys.stderr)
                time.sleep(1.0)
                continue

        rgb = np.frombuffer(obs.rgb, dtype=np.uint8).reshape(obs.height, obs.width, 3)

        rr.set_time("frame", sequence=frame)

        if frame == 0:
            rr.log("info/env", rr.TextLog(
                f"env={env_id}  {obs.width}×{obs.height}  "
                f"proprio_dim={len(obs.proprio)}",
                level=rr.TextLogLevel.INFO,
            ))

        # ── RGB camera ────────────────────────────────────────────────────────
        rr.log("camera/rgb", rr.Image(rgb, color_model="RGB"))

        # Always log RGB as detection background so the panel is never black
        rr.log("camera/detections/image", rr.Image(rgb, color_model="RGB"))

        # ── Depth map ─────────────────────────────────────────────────────────
        if obs.depth:
            depth = np.frombuffer(obs.depth, dtype=np.float32).reshape(obs.height, obs.width)
            rr.log("camera/depth", rr.DepthImage(depth, meter=1.0))

        # ── Camera intrinsics ─────────────────────────────────────────────────
        if obs.fx > 0 and obs.fy > 0:
            rr.log("camera/rgb",
                   rr.Pinhole(
                       focal_length=[obs.fx, obs.fy],
                       principal_point=[obs.cx, obs.cy],
                       width=obs.width,
                       height=obs.height,
                   ))

        # ── Joint positions ───────────────────────────────────────────────────
        for i, v in enumerate(obs.proprio):
            rr.log(f"robot/joint/{i:02d}", rr.Scalars(float(v)))

        # ── Goal position (shared world space with SLAM map from bridge) ──────
        if len(obs.goal_pos) >= 3:
            rr.log("world/goal",
                   rr.Points3D(
                       [[obs.goal_pos[0], obs.goal_pos[1], obs.goal_pos[2]]],
                       radii=0.025,
                       colors=[[255, 128, 0]],
                       labels=["goal"],
                   ))

        # ── 3-D point cloud (accumulated from depth back-projection) ─────────
        cloud_acc.add_frame(obs)
        if frame % _CLOUD_LOG_EVERY == 0:
            pts, cols = cloud_acc.get_cloud()
            if pts is not None:
                rr.log("world/cloud",
                       rr.Points3D(pts, colors=cols, radii=0.015))
                if frame % (_CLOUD_LOG_EVERY * 10) == 0:
                    print(f"[viz] cloud: {len(pts)} voxels → Rerun",
                          file=sys.stderr)

        # ── Feed frame to detector & read latest results ─────────────────────
        if detector is not None:
            detector.feed_frame(rgb, frame)

            detections, det_rgb = detector.get_results()
            if detections and det_rgb is not None:
                boxes_min = []
                boxes_size = []
                labels = []
                colors = []
                pts3d = []
                for det in detections:
                    bbox = det.get("bbox", [])
                    if len(bbox) >= 4:
                        x1, y1, x2, y2 = bbox[:4]
                        boxes_min.append([x1, y1])
                        boxes_size.append([x2 - x1, y2 - y1])
                        lbl = det.get("label", "?")
                        score = det.get("score", 0.0)
                        labels.append(f"{lbl} {score:.2f}")
                        colors.append([50, 220, 50])

                    xyz = det.get("center_xyz")
                    if xyz and len(xyz) >= 3 and all(
                        isinstance(c, (int, float)) for c in xyz
                    ):
                        pts3d.append(xyz[:3])

                if boxes_min:
                    rr.log("camera/detections/boxes",
                           rr.Boxes2D(mins=np.array(boxes_min),
                                      sizes=np.array(boxes_size),
                                      labels=labels, colors=colors))
                else:
                    rr.log("camera/detections/boxes", rr.Clear(recursive=False))

                if pts3d:
                    rr.log("world/detections",
                           rr.Points3D(pts3d, radii=0.02,
                                       colors=[[50, 220, 50]] * len(pts3d),
                                       labels=labels[:len(pts3d)]))

        frame += 1
        elapsed = time.monotonic() - t0
        wait = interval - elapsed
        if wait > 0:
            time.sleep(wait)


if __name__ == "__main__":
    main()
