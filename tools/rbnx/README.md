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
- `RBNX_CONFIG_FILE` — explicit path to a config file (overrides
  `~/.robonix/config.yaml`).

## How it finds the source tree

Many subcommands (codegen, build, boot) need to resolve paths inside
the Robonix source repo (the IDL tree under `capabilities/lib`, the
atlas proto under `system/atlas/proto`, the Python SDK under
`pylib/robonix-api`). `rbnx setup` writes the absolute path of the
current clone into `~/.robonix/config.yaml`; later calls anywhere on
disk resolve those keys via `rbnx path`.

The repo root is identified by the presence of `Cargo.toml`,
`capabilities/`, and `capabilities/lib/` (the IDL tree).
