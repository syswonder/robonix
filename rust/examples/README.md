# Examples (minimal E2E)

Robonix targets Linux broadly. The control plane (`robonix-server`), agent (`robonix-agent`), and VLM run as native processes. ROS 2 workloads use packages under `examples/packages/` and Docker (`tiago_sim_stack`).

Abstract to concrete interface: [../docs/POC.md](../docs/POC.md) · script: `scripts/hal_discovery_poc.py`

## Packages (`examples/packages/`)

| Package | Role |
|---------|------|
| `packages/vlm_service/` | VLM gRPC service (`rbnx build` sets `PYTHONPATH` to package + `examples/proto_gen`). |
| `packages/tiago_sim_stack/` | Docker Compose: Webots (GUI) + Nav2 + rviz2 + ROS2-MCP bridge (`tiago_bridge` module inside the image). |

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

## Contents

| Path | Role |
|------|------|
| `packages/` | Manifests + code (`vlm_service`, `tiago_sim_stack`). |
| `proto_gen/` | Generated Python `*_pb2.py` (see script below). |
| `scripts/gen_proto_python.sh` | Regenerate `proto_gen/` from `rust/proto` + `robonix-interfaces/robonix_proto`. |
| `scripts/smoke_minimal.sh` | Start server + run `smoke_control_plane.py` (no VLM). |
| `scripts/poc_container_bridge.sh` | Host server + Docker `ros2-bridge` (uses `packages/tiago_sim_stack/compose.yaml`). |
| `run.sh` | Server + **rbnx** validate/build/start per package + agent. By default starts **`tiago_sim_stack`** (Docker sim). `START_SIM_STACK=0` to skip. |
| `requirements.txt` | Pinned Python deps; `protobuf<7` so `grpcio-tools` and `openai` coexist. |

## Regenerating `proto_gen/`

After changing any `.proto` used by the Python nodes:

```bash
cd rust
pip install -r examples/requirements.txt
./examples/scripts/gen_proto_python.sh
```

## Namespaces

See `../docs/NAMESPACE.md`.

## End-to-end (recommended)

1. `cd rust && cargo build --workspace`
2. `pip install -r examples/requirements.txt` and `cp examples/.env.example examples/.env` (set `VLM_*`).
3. **E2E:** from `rust/`, `./examples/run.sh` — by default **VLM + `tiago_sim_stack`** (Docker + Webots; needs Docker and X11). **`START_SIM_STACK=0`** for VLM-only (no sim).
4. **Manual rbnx:** `cargo run -p robonix-cli -- validate|build|start -p examples/packages/...` with `robonix-server` running.

**Webots + Nav2 + rviz2** run inside the `tiago_sim_stack` image (see `packages/tiago_sim_stack/README.md` for GPU / X11).

Instruction input: the agent reads stdin (interactive).
