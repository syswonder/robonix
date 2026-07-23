# rbnx — the Robonix CLI

`rbnx` is the command-line entry point to Robonix. It wraps build,
deploy, boot, inspect, codegen, and chat operations into one binary so a
user / integrator never has to remember which Rust crate or Python service
owns which subcommand.

## Install

From the repo root:

```sh
make install
```

This builds the Cargo workspace and copies `rbnx` (plus the other system
binaries) into `~/.cargo/bin`, then runs `rbnx setup` to register this
clone as the Robonix source tree (`~/.robonix/config.yaml`).

You can also install just `rbnx`:

```sh
cargo install --force --path tools/rbnx --bin rbnx
```

## Subcommands

| Command | What it does |
| --- | --- |
| `rbnx build [PKG]`        | Build a deploy or a single package (incl. ROS workspaces) |
| `rbnx boot [-f MANIFEST]` | Bring up the full stack from a deploy manifest         |
| `rbnx shutdown`           | Tear down what `rbnx boot` brought up                  |
| `rbnx start PKG`          | Launch one package directly (used internally by boot)  |
| `rbnx caps [-v]`          | List capabilities currently registered with atlas      |
| `rbnx contracts`          | List contracts in the loaded registry                  |
| `rbnx channels`           | List open consumer→provider channels                   |
| `rbnx inspect`            | Dump atlas state for debugging                         |
| `rbnx codegen [--mcp]`    | Generate proto + MCP type stubs for a package          |
| `rbnx chat`               | Interactive TUI chat with the pilot                    |
| `rbnx ask "..."`          | Single-turn task submission                            |
| `rbnx path KEY`           | Resolve a registered source path (root / capabilities / interfaces-lib / runtime-proto / robonix-api) |
| `rbnx setup [PATH]`       | Register a robonix source tree at `PATH`               |
| `rbnx clean`              | Remove per-package build artefacts                     |

Run `rbnx <cmd> --help` for full flags.

## Environment

- `ROBONIX_ATLAS` — atlas endpoint, default `127.0.0.1:50051`.
- `VLM_API_KEY` / `VLM_BASE_URL` / `VLM_MODEL` — VLM credentials forwarded
  to pilot at boot.

Package instance settings belong in the deployment manifest's nested
`config:` mapping. `rbnx boot` serializes that mapping and sends it to the
provider through `Driver(CMD_INIT)`; it does not create a second package
configuration file.

Each primitive, service, or skill entry's `name` is its Atlas provider id and
must be non-empty, whitespace-normalized, and unique across those package
sections. `rbnx boot` passes that
identity to the package as `RBNX_INSTANCE_NAME`; the Python SDK uses it for
registration, capability declaration, heartbeat, and lifecycle state while
preserving the package's source-level id for standalone `rbnx start`. Startup
waits only for a fresh registration of that exact id, so unrelated providers
that register concurrently cannot receive the instance's lifecycle config. If
that id is already live in Atlas before spawn, startup fails rather than taking
over the existing provider.

Package authors normally omit Driver from `capabilities`; rbnx and current
codegen automatically select and register `robonix/lifecycle/driver`. Explicit
shared selection is accepted, and one explicitly selected namespace Driver
remains compatible when it exactly matches `<provider namespace>/driver`. A
legacy manifest may use a current shared runtime Driver while it is migrated;
the reverse shared-to-legacy substitution is rejected. At runtime every
provider must expose exactly one lifecycle Driver; missing, mismatched,
unrelated, and dual registrations all fail startup before config is delivered.

The compatibility handshake is fail-closed. For an omitted Driver, rbnx
exports `ROBONIX_DRIVER_CONTRACT_ID=robonix/lifecycle/driver` and clears the
compatibility marker. For an explicit legacy Driver, rbnx exports that exact ID
plus `ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK=1`; despite its historical
name, the marker permits only legacy-manifest to shared-runtime migration. The
SDK uses a complete legacy service pair when present, or a complete shared pair
when the legacy pair is wholly absent. Partial stubs, zero or multiple runtime
Drivers, unrelated IDs, and shared-to-legacy fallback all fail startup, so
config is never silently lost and no provider is promoted without a Driver.

Each primitive, service, and skill entry's `name` is its deploy-time Atlas
provider id. `rbnx boot` passes it as `RBNX_INSTANCE_NAME`; the SDK uses that
identity for registration, capability declaration, heartbeat, and lifecycle
state. Package source keeps its own default id for standalone `rbnx start`.
Startup waits only for the exact expected id, so an unrelated concurrent
registration cannot receive another instance's lifecycle config.

## How it finds the source tree

Many subcommands (codegen, build, boot) need to resolve paths inside
the Robonix source repo (the IDL tree under `capabilities/lib`, the
atlas proto under `system/atlas/proto`, the Python SDK under
`pylib/robonix-api`). `rbnx setup` writes the absolute path of the
current clone into `~/.robonix/config.yaml`; later calls anywhere on
disk resolve those keys via `rbnx path`.

The repo root is identified by the presence of `Cargo.toml`,
`capabilities/`, and `capabilities/lib/` (the IDL tree).
