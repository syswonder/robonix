# Examples (minimal E2E)

Robonix targets Linux broadly. The control plane (`robonix-server`), agent (`robonix-agent`), and VLM run as native processes when possible. ROS 2 workloads use **RFC002 packages** under `examples/packages/` and/or Docker (`tiago_sim_stack`).

Step-by-step checklist: [MINIMAL_PLATFORM.md](MINIMAL_PLATFORM.md)

Abstract to concrete interface: [../docs/POC.md](../docs/POC.md) · script: `scripts/hal_discovery_poc.py`

## RFC002 packages (`examples/packages/`)

| Package | Role |
|---------|------|
| `packages/tiago_bridge/` | ROS2 ↔ MCP bridge (`rbnx` colcon path or run `python3 -m tiago_bridge.node` with `PYTHONPATH`). |
| `packages/vlm_service/` | VLM gRPC service; manifest **`build.native_python: true`** → `rbnx build` skips RIDL/ridlc/colcon and sets `PYTHONPATH` to package + `examples/proto_gen`. |
| `packages/tiago_sim_stack/` | Docker Compose: Webots (GUI) + Nav2 + `tiago_bridge` (`rbnx` build/start). |

Validate:

```bash
cd rust
./examples/scripts/test_examples_packages.sh
```

**Compose stack (no ROS on host):**

```bash
cd rust
cargo run -p robonix-cli -- validate examples/packages/tiago_sim_stack
cargo run -p robonix-cli -- build -p examples/packages/tiago_sim_stack
# with robonix-server running:
cargo run -p robonix-cli -- start -p examples/packages/tiago_sim_stack -n com.robonix.prm.tiago
```

**Python path** for `-m` modules:

```bash
export PYTHONPATH=rust/examples/packages/vlm_service:rust/examples/packages/tiago_bridge
```

## Contents

| Path | Role |
|------|------|
| `packages/` | RFC002 manifests + code (`tiago_bridge`, `vlm_service`, `tiago_sim_stack`). |
| `proto_gen/` | Generated Python `*_pb2.py` (see script below). |
| `scripts/gen_proto_python.sh` | Regenerate `proto_gen/` from `rust/proto` + `robonix-interfaces/robonix_proto`. |
| `scripts/smoke_minimal.sh` | Start server + run `smoke_control_plane.py` (no VLM). |
| `scripts/poc_container_bridge.sh` | Host server + Docker `ros2-bridge` (uses `packages/tiago_sim_stack/compose.yaml`). |
| `run.sh` | Server + **rbnx** validate/build/start per package + agent. By default starts **`tiago_sim_stack`** (Docker sim). `START_SIM_STACK=0` to skip; `START_TIAGO_NODE=1` only with `START_SIM_STACK=0` (host bridge). |
| `requirements.txt` | Pinned Python deps; `protobuf<7` so `grpcio-tools` and `openai` coexist. |

## Regenerating `proto_gen/`

After changing any `.proto` used by the Python nodes:

```bash
cd rust
pip install -r examples/requirements.txt
./examples/scripts/gen_proto_python.sh
```

On PEP 668–restricted systems use a venv, or `pip install -r examples/requirements.txt --break-system-packages` where appropriate.

## Namespaces

See `../docs/NAMESPACE.md`.

## End-to-end (recommended)

1. `cd rust && cargo build`
2. `pip install -r examples/requirements.txt` and `cp examples/.env.example examples/.env` (set `VLM_*`).
3. **E2E (rbnx packages):** from `rust/`, `./examples/run.sh` — by default **VLM + `tiago_sim_stack`** (Docker + Webots; needs Docker and X11). **`START_SIM_STACK=0`** for VLM-only (no sim). **`START_TIAGO_NODE=1`** only with **`START_SIM_STACK=0`** for host `tiago_bridge` (source ROS 2 first).
4. **Manual rbnx:** `cargo run -p robonix-cli -- validate|build|start -p examples/packages/...` with `robonix-server` running.

**Webots + Nav2** run inside the `tiago_sim_stack` image (see `packages/tiago_sim_stack/README.md` for GUI / X11).

Instruction input: the agent reads stdin (interactive). A dedicated job queue is optional for later.

## Python dependencies

Prefer the pinned set (avoids `protobuf 7.x` vs `grpcio-tools` conflicts):

```bash
cd rust
pip install -r examples/requirements.txt
```

ROS bridge on the **host** additionally requires a sourced ROS 2 distro and `rclpy` / message packages.
