#!/usr/bin/env python3
"""Perception node: open-vocabulary detection (YOLO-World or Grounding DINO).

Default backend is YOLO-World via ultralytics (much lower VRAM than GroundingDINO).
Set PERCEPTION_BACKEND=grounding_dino to use the previous HuggingFace model.

Fetches RGB+depth from env_node via gRPC, runs detection, exposes MCP tool
for robonix-agent.  Agent never needs to transfer images.
"""
import asyncio
import json
import logging
import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

for _n in (
    "mcp", "mcp.server", "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
    "httpx", "httpcore", "uvicorn", "uvicorn.access",
):
    logging.getLogger(_n).setLevel(logging.WARNING)

from mcp.server.fastmcp import FastMCP


def _ensure_proto_paths() -> None:
    pkg = Path(__file__).resolve().parent
    sys.path.insert(0, str(pkg))
    d = pkg
    while d.parent != d:
        pc = d / "proto_stubs"
        pg = d / "proto_gen"
        if pc.is_dir() and (pc / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pc))
            if pg.is_dir():
                sys.path.append(str(pg))
            return
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

mcp = FastMCP("perception")

_IOX2_TRANSPORT = os.environ.get("OBS_TRANSPORT", "grpc").strip().lower() == "iceoryx2"

_env_stub: env_pb_grpc.EnvDataServiceStub | None = None
_iox2_rgb_sub = None
_iox2_dep_sub = None
_detector = None
_processor = None
_detector_use_fp16 = False
_yolo_model = None
_yolo_device: str | int = "cpu"
_yolo_half = False
_detector_kind: str | None = None  # "yolo_world" | "grounding_dino"

# Minimum free GPU memory (MB) before preferring CPU for heavy models.
_MIN_GPU_MB = int(os.environ.get("PERCEPTION_MIN_GPU_MB", "400"))


def _pick_torch_device(device_req: str) -> str:
    import torch

    if device_req == "cpu":
        return "cpu"
    if device_req == "cuda":
        return "cuda" if torch.cuda.is_available() else "cpu"
    if torch.cuda.is_available():
        free_mb = torch.cuda.mem_get_info()[0] / 1024 / 1024
        if free_mb < _MIN_GPU_MB:
            print(f"[perception] GPU free={free_mb:.0f} MB < {_MIN_GPU_MB} MB "
                  f"→ falling back to CPU", file=sys.stderr)
            return "cpu"
        return "cuda"
    return "cpu"


def _class_list_from_prompt(text_prompt: str) -> list[str]:
    parts = [p.strip() for p in text_prompt.replace(",", ".").split(".")]
    return [p for p in parts if p] or ["object"]


def _load_yolo_world() -> None:
    global _yolo_model, _yolo_device, _yolo_half, _detector_kind
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[perception] ultralytics not installed — cannot load YOLO-World", file=sys.stderr)
        return

    import torch

    weights = resolve_yolo_world_weights(os.environ.get("PERCEPTION_YOLO_WEIGHTS"))
    device_req = os.environ.get("PERCEPTION_DEVICE", "auto").strip().lower()
    fp16_enabled = os.environ.get("PERCEPTION_FP16", "1") == "1"
    device = _pick_torch_device(device_req)
    _yolo_half = bool(device == "cuda" and fp16_enabled)
    dev_arg: str | int = 0 if device == "cuda" else "cpu"

    print(f"[perception] loading YOLO-World {weights}…", file=sys.stderr)
    try:
        _yolo_model = YOLO(weights)
    except (RuntimeError, OSError) as e:
        print(f"[perception] YOLO load failed: {e}", file=sys.stderr)
        _yolo_model = None
        return

    # Warm-up on target device so OOM happens here, not on first tool call.
    from PIL import Image as PILImage

    dummy = PILImage.new("RGB", (320, 240))
    try:
        _yolo_model.set_classes(["object"])
        _yolo_model.predict(
            dummy,
            conf=0.5,
            verbose=False,
            device=dev_arg,
            half=_yolo_half,
        )
    except (RuntimeError, torch.OutOfMemoryError) as exc:
        if device == "cuda" and "out of memory" in str(exc).lower():
            print("[perception] YOLO-World CUDA OOM → retrying on CPU", file=sys.stderr)
            torch.cuda.empty_cache()
            device = "cpu"
            dev_arg = "cpu"
            _yolo_half = False
            _yolo_model.predict(
                dummy,
                conf=0.5,
                verbose=False,
                device="cpu",
                half=False,
            )
        else:
            raise

    _yolo_device = dev_arg
    _detector_kind = "yolo_world"
    print(f"[perception] YOLO-World ready (device={device}, half={_yolo_half})", file=sys.stderr)


def _load_grounding_dino() -> None:
    global _detector, _processor, _detector_use_fp16, _detector_kind
    try:
        from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
    except ImportError:
        print("[perception] WARNING: transformers not installed, using dummy detector", file=sys.stderr)
        return

    import torch

    model_id = os.environ.get("PERCEPTION_MODEL_ID", "IDEA-Research/grounding-dino-tiny")
    device_req = os.environ.get("PERCEPTION_DEVICE", "auto").strip().lower()
    fp16_enabled = os.environ.get("PERCEPTION_FP16", "1") == "1"
    print(f"[perception] loading {model_id}…", file=sys.stderr)
    _processor = AutoProcessor.from_pretrained(model_id)

    device = _pick_torch_device(device_req)
    use_fp16 = (device == "cuda" and fp16_enabled)
    _detector_use_fp16 = use_fp16
    dtype = torch.float16 if use_fp16 else torch.float32
    _detector = AutoModelForZeroShotObjectDetection.from_pretrained(model_id, torch_dtype=dtype)

    try:
        _detector = _detector.to(device)
    except (RuntimeError, torch.OutOfMemoryError) as exc:
        if device != "cpu" and "out of memory" in str(exc).lower():
            print("[perception] CUDA OOM moving model to GPU → falling back to CPU",
                  file=sys.stderr)
            torch.cuda.empty_cache()
            device = "cpu"
            _detector_use_fp16 = False
            _detector = _detector.float().to("cpu")
        else:
            raise

    _detector.eval()
    _detector_kind = "grounding_dino"
    print(f"[perception] GroundingDINO ready (device={device}, fp16={use_fp16})", file=sys.stderr)


def _load_detector() -> None:
    backend = os.environ.get("PERCEPTION_BACKEND", "yolo_world").strip().lower()
    if backend in ("grounding_dino", "grounding-dino", "gdino", "dino"):
        _load_grounding_dino()
    else:
        _load_yolo_world()
        if _yolo_model is None:
            print("[perception] YOLO-World unavailable → trying GroundingDINO", file=sys.stderr)
            _load_grounding_dino()


# ── Detection logic ──────────────────────────────────────────────────────────


def _detect_yolo(
    rgb: np.ndarray,
    text_prompt: str,
    depth: np.ndarray | None,
    threshold: float,
    intrinsics: tuple[float, float, float, float] | None,
) -> list[dict]:
    """YOLO-World open-vocabulary detection (ultralytics)."""
    if _yolo_model is None:
        return []

    classes = _class_list_from_prompt(text_prompt)
    _yolo_model.set_classes(classes)

    results = _yolo_model.predict(
        rgb,
        conf=threshold,
        verbose=False,
        device=_yolo_device,
        half=_yolo_half,
    )
    r = results[0]
    if r.boxes is None or len(r.boxes) == 0:
        return []

    h, w = rgb.shape[:2]
    fx, fy, cx, cy = intrinsics if intrinsics else (w / 2.0, h / 2.0, w / 2.0, h / 2.0)

    out: list[dict] = []
    boxes = r.boxes
    for i in range(len(boxes)):
        box = boxes.xyxy[i].cpu().tolist()
        score = float(boxes.conf[i].cpu())
        ci = int(boxes.cls[i].cpu())
        label = classes[ci] if 0 <= ci < len(classes) else str(ci)

        cx_px = (box[0] + box[2]) / 2.0
        cy_px = (box[1] + box[3]) / 2.0
        center_xyz = None
        if depth is not None:
            u, v = int(cx_px), int(cy_px)
            u, v = max(0, min(u, w - 1)), max(0, min(v, h - 1))
            z = float(depth[v, u])
            if 0.01 < z < 10.0:
                x = (u - cx) / fx * z
                y = (v - cy) / fy * z
                center_xyz = [round(x, 4), round(y, 4), round(z, 4)]

        out.append({
            "label": label,
            "bbox": [round(c, 1) for c in box],
            "score": round(score, 3),
            "center_xyz": center_xyz,
        })
    return out


def _detect(rgb: np.ndarray, text_prompt: str, depth: np.ndarray | None = None,
            threshold: float = 0.25,
            intrinsics: tuple[float, float, float, float] | None = None):
    """Run open-vocab detector on rgb (HWC uint8). Returns list of detections."""
    if _detector_kind == "yolo_world":
        if _yolo_model is not None:
            return _detect_yolo(rgb, text_prompt, depth, threshold, intrinsics)
        return [{"label": text_prompt.split(".")[0].strip(),
                 "bbox": [0.3, 0.3, 0.7, 0.7], "score": 0.5,
                 "center_xyz": [0.3, 0.0, 0.5]}]

    from PIL import Image as PILImage
    import torch

    pil_img = PILImage.fromarray(rgb)

    if _detector is None or _processor is None:
        return [{"label": text_prompt.split(".")[0].strip(),
                 "bbox": [0.3, 0.3, 0.7, 0.7], "score": 0.5,
                 "center_xyz": [0.3, 0.0, 0.5]}]

    inputs = _processor(images=pil_img, text=text_prompt, return_tensors="pt")
    model_param = next(_detector.parameters())
    device = model_param.device
    model_dtype = model_param.dtype
    converted = {}
    for k, v in inputs.items():
        if isinstance(v, torch.Tensor):
            v = v.to(device)
            # Keep integer tensors (e.g., input_ids) unchanged; cast floating tensors
            # to the model's dtype so fp16 models don't hit dtype mismatch errors.
            if torch.is_floating_point(v):
                v = v.to(dtype=model_dtype)
        converted[k] = v
    inputs = converted

    def _run_forward(run_inputs):
        with torch.no_grad():
            return _detector(**run_inputs)

    try:
        outputs = _run_forward(inputs)
    except RuntimeError as e:
        err = str(e)
        # Some mixed-precision combinations in GroundingDINO can still produce
        # matmul dtype mismatches on specific kernels. Fallback to FP32 once.
        if "same dtype" in err and "Float and Half" in err and _detector is not None:
            print("[perception] dtype mismatch in fp16 path; retrying in fp32", file=sys.stderr)
            _detector.float()
            for k, v in list(inputs.items()):
                if isinstance(v, torch.Tensor) and torch.is_floating_point(v):
                    inputs[k] = v.float()
            outputs = _run_forward(inputs)
        else:
            raise

    results = _processor.post_process_grounded_object_detection(
        outputs,
        inputs["input_ids"],
        threshold=threshold,
        target_sizes=[pil_img.size[::-1]],
    )[0]

    h, w = rgb.shape[:2]
    fx, fy, cx, cy = intrinsics if intrinsics else (w / 2.0, h / 2.0, w / 2.0, h / 2.0)

    detections = []
    for box, score, label in zip(results["boxes"], results["scores"], results["labels"]):
        box = box.cpu().tolist()
        cx_px = (box[0] + box[2]) / 2.0
        cy_px = (box[1] + box[3]) / 2.0

        center_xyz = None
        if depth is not None:
            u, v = int(cx_px), int(cy_px)
            u, v = max(0, min(u, w - 1)), max(0, min(v, h - 1))
            z = float(depth[v, u])
            if 0.01 < z < 10.0:
                x = (u - cx) / fx * z
                y = (v - cy) / fy * z
                center_xyz = [round(x, 4), round(y, 4), round(z, 4)]

        detections.append({
            "label": label,
            "bbox": [round(c, 1) for c in box],
            "score": round(float(score.cpu()), 3),
            "center_xyz": center_xyz,
        })

    return detections


# ── MCP tool ─────────────────────────────────────────────────────────────────


def _get_obs_snapshot():
    """Return (rgb_hwc_uint8, depth_hw_f32_or_None, intrinsics_or_None)."""
    if _IOX2_TRANSPORT and _iox2_rgb_sub is not None:
        rgb_sample = _iox2_rgb_sub.receive()
        if rgb_sample is None:
            return None, None, None
        rf = rgb_sample.payload().contents
        n_rgb = rf.width * rf.height * rf.channels
        rgb = np.frombuffer(bytes(rf.pixels[:n_rgb]), dtype=np.uint8).reshape(
            rf.height, rf.width, rf.channels
        )
        depth = None
        if _iox2_dep_sub is not None:
            dep_sample = _iox2_dep_sub.receive()
            if dep_sample is not None:
                df = dep_sample.payload().contents
                n_dep = df.width * df.height * 4
                depth = np.frombuffer(bytes(df.pixels[:n_dep]), dtype=np.float32).reshape(
                    df.height, df.width
                )
        return rgb, depth, None  # no intrinsics in iceoryx2 path

    if _env_stub is None:
        return None, None, None
    try:
        obs = _env_stub.GetObs(env_pb.Empty())
    except grpc.RpcError as e:
        print(f"[perception] GetObs error: {e.details()}", file=sys.stderr)
        return None, None, None
    rgb = np.frombuffer(obs.rgb, dtype=np.uint8).reshape(obs.height, obs.width, 3)
    depth = None
    if obs.depth:
        depth = np.frombuffer(obs.depth, dtype=np.float32).reshape(obs.height, obs.width)
    intrinsics = None
    if obs.fx > 0 and obs.fy > 0:
        intrinsics = (obs.fx, obs.fy, obs.cx, obs.cy)
    return rgb, depth, intrinsics


@mcp.tool()
def detect_objects(text_prompt: str, threshold: float = 0.25) -> str:
    """Detect objects matching text_prompt in current env camera view.
    text_prompt: dot-separated labels, e.g. 'red apple . table . cup'.
    Returns JSON list of {label, bbox, score, center_xyz}."""
    rgb, depth, intrinsics = _get_obs_snapshot()
    if rgb is None:
        return json.dumps({"error": "no observation available (transport not ready)"})
    dets = _detect(rgb, text_prompt, depth, threshold, intrinsics)
    return json.dumps(dets)


# ── Boilerplate ──────────────────────────────────────────────────────────────


def _pick_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _mcp_tools_list() -> list[dict]:
    async def _list():
        return await mcp.list_tools()
    tools = asyncio.run(_list())
    return [{"name": t.name, "description": t.description or "",
             "input_schema": dict(t.inputSchema)} for t in tools]


def _iface_meta_mcp() -> str:
    return json.dumps({"tools": _mcp_tools_list()})


def _start_mcp_http(port: int) -> None:
    import uvicorn
    app = mcp.streamable_http_app()
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")


def _heartbeat_loop(stub, node_id: str) -> None:
    while True:
        time.sleep(15.0)
        try:
            stub.NodeHeartbeat(pb.NodeHeartbeatRequest(node_id=node_id))
        except Exception as e:
            print(f"[perception] heartbeat failed: {e}", file=sys.stderr)


def _discover_env_grpc(stub, node_id: str) -> str:
    """Use NegotiateChannel to discover env_node's gRPC data endpoint."""
    for attempt in range(30):
        try:
            resp = stub.NegotiateChannel(pb.NegotiateChannelRequest(
                consumer_id=node_id,
                provider_node_id="com.robonix.demo.maniskill",
                interface_name="env_data",
                transport="grpc",
            ))
            return resp.endpoint
        except grpc.RpcError:
            if attempt < 29:
                time.sleep(2)
            else:
                raise
    raise RuntimeError("could not discover env gRPC endpoint")


# ── Main ─────────────────────────────────────────────────────────────────────


def main() -> None:
    global _env_stub, _iox2_rgb_sub, _iox2_dep_sub

    _load_detector()

    server_addr = os.environ.get("ROBONIX_SERVER", "localhost:50051")
    channel = grpc.insecure_channel(server_addr)
    stub = pb_grpc.RobonixRuntimeStub(channel)

    node_id = "com.robonix.demo.perception"
    stub.RegisterNode(pb.RegisterNodeRequest(
        node_id=node_id,
        namespace="robonix/prm/perception",
        kind="primitive",
    ))

    mcp_port = _pick_port()
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        node_id=node_id,
        name="mcp_tools",
        supported_transports=["mcp"],
        metadata_json=_iface_meta_mcp(),
        listen_port=mcp_port,
    ))

    threading.Thread(target=_heartbeat_loop, args=(stub, node_id), daemon=True).start()
    threading.Thread(target=_start_mcp_http, args=(mcp_port,), daemon=True).start()
    print(f"[perception] MCP :{mcp_port} (serving)", file=sys.stderr)

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
            print("[perception] iceoryx2 subscriber ready", file=sys.stderr)
        except Exception as exc:
            print(f"[perception] iceoryx2 init failed, falling back to gRPC: {exc}", file=sys.stderr)
    else:
        print("[perception] discovering env gRPC endpoint...", file=sys.stderr)
        env_endpoint = _discover_env_grpc(stub, node_id)
        env_channel = grpc.insecure_channel(env_endpoint)
        _env_stub = env_pb_grpc.EnvDataServiceStub(env_channel)
        print(f"[perception] connected to env gRPC at {env_endpoint}", file=sys.stderr)

    print("[perception] ready", file=sys.stderr)

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
