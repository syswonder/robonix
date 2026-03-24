# Minimal platform checklist

Goal: one `robonix-server` on the host, native agent + VLM, optional ROS bridge in Docker, Webots/Nav2 from `rust/_deprecated/provider/tiago_demo_package`.

## 1. Build

```bash
cd rust && cargo build --workspace
```

## 2. Python stubs (when `.proto` changes)

```bash
cd rust
pip install -r examples/requirements.txt
./examples/scripts/gen_proto_python.sh
```

(`grpcio-tools` needs `protobuf>=6.30,<7`. Installing `openai` without that cap can pull `protobuf 7.x` and break `grpcio-tools`—use `requirements.txt`.)

## 3. Python deps (agent + nodes)

```bash
cd rust
pip install -r examples/requirements.txt
```

Or a venv: `python3 -m venv .venv && source .venv/bin/activate && pip install -r examples/requirements.txt`

## 4. Smoke (control plane only, no VLM)

```bash
cd rust
./examples/scripts/smoke_minimal.sh
```

Or with a server you already started: `SMOKE_USE_EXISTING_SERVER=1 ./examples/scripts/smoke_minimal.sh`

Namespace discovery PoC (print registered nodes, namespaces, interfaces; run after `vlm_service` / `tiago_node` to see real entries):

```bash
cd rust
python3 examples/scripts/hal_discovery_poc.py
```

See `rust/docs/POC.md` for the abstract-to-concrete flow and examples.

## 5. Full interactive E2E (VLM + sim + agent)

From `rust/` (starts **VLM**, **`tiago_sim_stack`** in Docker with Webots GUI, **server**, **agent** via `rbnx` — needs Docker + X11):

```bash
cd rust
cp examples/.env.example examples/.env   # set VLM_*
./examples/run.sh
```

VLM-only (no sim): `START_SIM_STACK=0 ./examples/run.sh`.

## 6. Tiago sim stack details

See `examples/packages/tiago_sim_stack/README.md` (compose, `rbnx`, X11). Legacy host Webots flow: `rust/_deprecated/provider/tiago_demo_package/README.md`.

## Namespaces

See `rust/docs/NAMESPACE.md`.
