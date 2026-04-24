# Examples (E2E demo)

Robonix targets Linux broadly. The control plane (`robonix-atlas`), runtime services (`robonix-executor`, `robonix-pilot`), and VLM run as native processes. ROS 2 workloads use packages under `examples/packages/` and Docker (`tiago_sim_stack`).

## Packages (`examples/packages/`)

| Package | Role |
|---------|------|
| `packages/vlm_service/` | VLM data plane: implements contract `SysModelVlmChat` from `robonix_contracts.proto` (wire types in `vlm.proto`). `rbnx build` refreshes **package-local** `proto_gen/`. |
| `packages/tiago_sim_stack/` | Docker Compose: Webots (GUI) + Nav2 + rviz2 + `tiago_bridge` (MCP tools + `PrmCameraRgb` gRPC stream from `robonix_contracts.proto`). |
| `packages/maniskill_vla_demo/` | ManiSkill3 VLA demo: MCP contracts + **demo-local** `EnvDataService` gRPC (`maniskill_env.proto`, not merged contract services). |
| `packages/memsearch_service/` | Semantic memory over MCP (`RobonixRuntime` registration only). |

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
# with robonix-atlas running:
cargo run -p robonix-cli -- start -p examples/packages/tiago_sim_stack -n com.robonix.prm.tiago
```

## Contents

| Path | Role |
|------|------|
| `packages/` | Manifests + code per demo. |
| `scripts/smoke_minimal.sh` | Start atlas + run `smoke_control_plane.py` (no VLM). |
| `run.sh` | Full E2E: atlas → executor → pilot → packages → liaison (foreground). |
| `requirements.txt` | Pinned Python deps; `protobuf<7` so `grpcio-tools` and `openai` coexist. |

## Regenerating `proto_gen/`

After changing any `.proto` used by the Python nodes:

```bash
cd rust
pip install -r examples/requirements.txt
./examples/scripts/gen_proto_python.sh
```

## End-to-end (recommended)

1. `cd rust && cargo build --workspace`
2. `pip install -r examples/requirements.txt` and `cp examples/.env.example examples/.env` (set `VLM_*`).
3. **E2E:** from `rust/`, `./examples/run.sh` — by default **VLM + `tiago_sim_stack`** (Docker + Webots; needs Docker and X11). **`START_SIM_STACK=0`** for VLM-only.
4. **Manual rbnx:** `cargo run -p robonix-cli -- validate|build|start -p examples/packages/...` with `robonix-atlas` running.

**Webots + Nav2 + rviz2** run inside the `tiago_sim_stack` image (see `packages/tiago_sim_stack/README.md` for GPU / X11).

Instruction input: robonix-liaison reads stdin (interactive).
