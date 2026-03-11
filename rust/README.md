# Robonix

This `rust/` tree currently contains two interface paths:

- the new RIDL-first path centered on `ridlc`, `robonix-interfaces`, and the gRPC runtime registry inside `robonix-server`
- the older provider / CLI / `/rbnx/*` ROS service path, which still contains many legacy `robonix_sdk` references

For new interface work, use the RIDL-first path. `robonix-sdk` is not part of the new architecture target.

## Active components

- `ridlc`
  - compiles `.ridl` into a directly buildable ROS 2 workspace
  - generates `src/generated`, `src/vendor`, and `src/app`
- `robonix-interfaces`
  - source of truth for handwritten RIDL and reusable ROS message packages
  - `lib/` contains reusable `msg` packages only
  - `ridl/` contains Robonix `query`, `command`, `stream`, and `event` definitions
- `robonix-server`
  - hosts the runtime gRPC control plane used by RIDL-generated code
  - still also hosts legacy `/rbnx/*` ROS APIs while migration is incomplete

## RIDL-first quick start

Prerequisites:

- ROS 2 Humble
- Rust toolchain
- Python 3.10+
- `python3-grpcio`
- `ros-humble-rmw-zenoh-cpp` for the Zenoh end-to-end test

Start `robonix-server`:

```bash
cd rust
ROBONIX_WEB_ASSETS_DIR="$(pwd)/robonix-server/web" \
ROBONIX_WEB_PORT=8000 \
ROBONIX_META_GRPC_ADDR=127.0.0.1:50051 \
ROBONIX_META_GRPC_ENDPOINT=127.0.0.1:50051 \
RUST_LOG=robonix_server=info \
cargo run --manifest-path robonix-server/Cargo.toml
```

Generate and validate interfaces:

```bash
cd rust/ridlc
./tests/test_codegen.sh
./tests/run_zenoh_rmw_e2e.sh
```

Those tests cover the current supported flow:

- RIDL compilation
- vendored ROS dependency export
- generated `app` layer creation
- combined runtime execution in one process
- end-to-end `query`, `stream`, and `command` transport over ROS 2 + Zenoh

## Repository status

What is current:

- `ridlc` + `robonix-interfaces` + `robonix-server` runtime gRPC path
- canonical RIDL namespaces under `robonix/hal/*` and `robonix/system/*`
- generated app skeletons intended for vendor customization
- new package-manifest target documented in `ROBONIX_PACKAGE_MANIFEST.md`
- generated RIDL stubs are the target place for automatic runtime registration with `robonix-server`

What is still legacy:

- many `robonix-cli` commands and configs
- several provider packages under `provider/`
- the older `/rbnx/*` ROS service registration stack in `robonix-server`
- the checked-in `robonix-sdk` package and code that still imports `robonix_sdk/*`

## Important note about `robonix-sdk`

`robonix-sdk` can no longer be treated as the architecture baseline.
However, it cannot be deleted safely yet because multiple runtime components in this repository still compile against or import it.

If the goal is full removal, the remaining `robonix_sdk` references in:

- `robonix-server`
- `robonix-cli`
- `provider/*`
- `tools/*`

must be migrated to the new RIDL / `robonix-interfaces` model first.

## Packaging docs

- New package manifest spec: `ROBONIX_PACKAGE_MANIFEST.md`
- New deployment composition spec: `ROBONIX_DEPLOYMENT.md`
- Legacy package spec: `PACKAGE_SPEC.md`

## vNext CLI helpers

From `rust/robonix-cli`:

```bash
cargo run -- package validate ../examples/demo_service_provider_vnext
cargo run -- package validate-deployment ../examples/demo_service_provider_vnext
```
