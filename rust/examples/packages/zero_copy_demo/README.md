# Robonix Zero-Copy Multi-Process Demo

Three independent OS processes share high-bandwidth data without unnecessary
memory copies, coordinated by `robonix-atlas`.

```
Camera → YOLO → Sobel   (3 processes, 1 unavoidable copy)
```

The buffer system is data-type agnostic — images, point clouds, tensors,
embeddings, or any contiguous data can flow through the same zero-copy path.

---

## Architecture

```
                        robonix-atlas
                     (gRPC control plane)
                    ┌─────────────────────┐
                    │ RegisterNode        │
                    │ DeclareInterface    │
                    │ NegotiateChannel    │
                    │ metadata_json pass  │
                    └──┬───────┬───────┬──┘
                       │       │       │
         ┌─────────────┘       │       └─────────────┐
         ▼                     ▼                     ▼
  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐
  │ Camera  (P1) │     │  YOLO   (P2) │     │ Sobel   (P3) │
  │              │ SHM │              │CUDA │              │
  │  DMA → SHM   ├────▶│  mmap 0-copy │ IPC │  IPC import  │
  │  shm_open    │     │  pin + H2D   ├────▶│  D2D ~50 µs  │
  │  +mmap       │     │  YOLOv8 GPU  │     │  Sobel GPU   │
  │              │     │  IPC export  │     │              │
  └──────────────┘     └──────────────┘     └──────────────┘

  Data copies: 1 (pinned H2D DMA — unavoidable on discrete GPU)
```

| Stage | Mechanism | Copy? |
|-------|-----------|-------|
| Camera → YOLO | POSIX SHM (`/dev/shm`), mmap | **0** (same physical pages) |
| SHM → GPU | `cudaHostRegister` + async DMA | **1** (H2D, unavoidable) |
| YOLO → Sobel | CUDA IPC (`cudaIpcGetMemHandle`) | **0** (same VRAM) |

vs. Traditional ROS 2 / FastDDS: **5+ copies** per frame for the same topology.

---

## Quick Start

### Prerequisites

- Linux, NVIDIA GPU + CUDA ≥ 12.0, Rust, Python ≥ 3.10

### Build

```bash
cd rust/examples/packages/zero_copy_demo
./run.sh setup          # builds Rust lib + installs Python deps
```

### Run (auto-starts robonix-atlas)

```bash
./run.sh start 200                          # launch 3-process pipeline
./run.sh benchmark --frames 200             # multi-process benchmark with timing
./run.sh compare 1920 1080 200              # Robonix vs ROS 2/FastDDS comparison
```

---

## Data Flow

```
1. Camera registers with robonix-atlas, declares "rgb" interface (shared_memory).
   Server assigns SHM name: /rbnx_shm_<uuid>

2. Camera allocates SHM via RobonixBufferManager.allocate().
   Writes frames directly into SHM (DMA simulation).

3. YOLO registers, calls NegotiateChannel(camera→yolo).
   Server returns SHM endpoint + metadata_json {width, height, format, ...}.

4. YOLO opens camera's SHM via RobonixBufferManager.open(shm_name, pin_for_gpu=True).
   Rust side: shm_open → mmap → cudaHostRegister (page-lock for DMA).

5. YOLO reads SHM via mmap (zero-copy), does pinned async H2D,
   GPU preprocessing (HWC uint8 → NCHW float32), and YOLOv8 inference.

6. YOLO allocates a GPU buffer (cudaMalloc), exports CUDA IPC handle,
   base64-encodes it into metadata_json of its "gpu_tensor" interface.

7. Sobel registers, calls NegotiateChannel(yolo→sobel).
   Server returns metadata_json containing cuda_ipc_handle_b64.

8. Sobel imports the handle (cudaIpcOpenMemHandle), gets a device pointer
   to the same physical VRAM. Reads via D2D copy (~50 µs), runs Sobel on GPU.
```

---

## Key Concepts

| Term | Meaning |
|------|---------|
| **SHM** | POSIX shared memory (`/dev/shm`). Multiple processes mmap the same physical pages. |
| **Pinned memory** | `cudaHostRegister` locks pages in RAM, enabling GPU DMA at full PCIe bandwidth. |
| **H2D** | Host-to-Device transfer. Pinned → async DMA; pageable → staged copy (2× slower). |
| **CUDA IPC** | `cudaIpcGetMemHandle` exports a 64-byte token; another process imports it with `cudaIpcOpenMemHandle` to access the same VRAM. |
| **metadata_json** | JSON in `DeclareInterface` / `NegotiateChannel` carrying buffer geometry, format, IPC handles, etc. |
| **HWC / NCHW** | Image layouts. HWC = camera/OpenCV; NCHW = PyTorch/cuDNN. Conversion is GPU-only (no CPU). |

---

## metadata_json Schema

```json
{
  "width": 1920, "height": 1080, "channels": 3,
  "format": 0,
  "memory_domain": "cpu",
  "cuda_ipc_handle_b64": "<base64 of 64-byte handle>",
  "gpu_buf_size": 6220800,
  "msg_type": "robonix_msg/ZeroCopyFrame"
}
```

For non-image data (tensors, point clouds), use `shape` + `data_bytes` instead of `width`/`height`/`channels`.

---

## File Structure

```
zero_copy_demo/
├── run.sh                       # setup / start / benchmark / compare
├── robonix_manifest.yaml        # rbnx manifest (3 nodes)
├── pyproject.toml
├── zero_copy_demo/
│   ├── common.py                # GPU kernels, gRPC helpers, fake camera
│   ├── rbnx_buffer.py           # Python ctypes wrapper for librobonix_buffer.so
│   ├── benchmark.py             # Multi-process benchmark launcher + collector
│   └── nodes/
│       ├── camera_node.py       # Process 1: SHM producer
│       ├── yolo_node.py         # Process 2: SHM consumer → CUDA IPC producer
│       └── edge_node.py         # Process 3: CUDA IPC consumer → Sobel
└── docker/
    ├── ros2_bench.py            # ROS 2/FastDDS traditional benchmark
    ├── run_comparison.sh        # Robonix vs ROS 2 orchestration
    ├── Dockerfile.ros2_bench    # ROS 2 + CUDA container
    └── verify_results.py        # Numerical result comparison
```

---

## TODO

- [ ] Jetson UMA: `cudaHostGetDevicePointer()` for true zero-copy (no H2D)
- [ ] Multi-consumer ring buffer with ref-counted slots
- [ ] Cross-process frame signaling (eventfd / CUDA events)
- [ ] MCAP recording via shadow consumer
