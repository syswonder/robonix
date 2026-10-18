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
| `rbnx build [-f MANIFEST \| -p PKG \| -g NAME]` | Build a deploy or a single package (incl. ROS workspaces) |
| `rbnx boot [-f MANIFEST]` | Bring up the full stack from a deploy manifest         |
| `rbnx shutdown`           | Tear down what `rbnx boot` brought up                  |
| `rbnx start [-p PKG]`     | Launch one package directly (used internally by boot)  |
| `rbnx caps [-v]`          | List capabilities currently registered with atlas      |
| `rbnx contracts`          | List contracts in the loaded registry                  |
| `rbnx tools`              | List the MCP capabilities visible to Pilot's model     |
| `rbnx channels`           | List open consumer→provider channels                   |
| `rbnx inspect`            | Dump atlas state for debugging                         |
| `rbnx codegen [--mcp]`    | Generate proto + MCP type stubs for a package          |
| `rbnx chat`               | Interactive TUI chat with the pilot                    |
| `rbnx ask "..."`          | Single-turn task submission; `--session ID` continues or steers a session |
| `rbnx path KEY`           | Resolve a registered source path (root / capabilities / interfaces-lib / runtime-proto / robonix-api) |
| `rbnx setup [PATH]`       | Register a robonix source tree at `PATH`               |
| `rbnx clean`              | Remove per-package build artefacts                     |
| `rbnx validate [PATH]`    | Validate a package manifest without building           |
| `rbnx config [-s \| -p PATH]` | Show the CLI configuration, or set the package storage path |
| `rbnx docs [--out-dir DIR]` | Regenerate the contract and ROS IDL reference pages from `capabilities/` |
| `rbnx describe [--provider ID] [--json]` | Show `CAPABILITY.md` for registered providers |
| `rbnx init NAME [--path DIR]` | Create a robot deployment directory with a `robonix_manifest.yaml` |
| `rbnx package-new NAME [-t primitive\|service\|skill] [--path DIR]` | Create a new package |
| `rbnx logs [-d DIR] [-t TAG] [-l LEVEL] [-f] [--json] [--list-tags]` | Read Scribe log files, by default from `./rbnx-boot/logs` |
| `rbnx update [-y]`        | Fast-forward cloned `url:` packages; asks first unless `-y`, and refuses without a terminal |
| `rbnx update --check [--json]` | Report which `url:` packages have upstream commits; pulls nothing |
| `rbnx version [--json]`   | Version, git commit, build date, rustc, target; `--json` adds the source root |

Run `rbnx <cmd> --help` for full flags.

`rbnx install [--github REPO \| --path DIR]`, `rbnx list`, and `rbnx info NAME`
manage system-installed packages. They are legacy commands, hidden from
`rbnx --help`; deployment manifests with `rbnx build` and `rbnx boot` replace
them.

`rbnx tools` follows contract metadata and omits entries marked
`llm_callable = false`; `rbnx contracts` still lists those contracts and marks
them `pilot-hidden`, because they remain registered and callable by system
consumers.

### What a boot leaves behind

`rbnx boot` writes `<manifest-dir>/rbnx-boot/deployment.lock` before it spawns
anything. Each package records what the manifest asked for and what that
resolved to, kept apart the way `flake.lock` keeps them:

```yaml
version: 1
packages:
  - name: mapping
    original:
      url: https://github.com/syswonder/service-map-rbnx
      branch: feat/map-localizer
    locked:
      commit: df2ba718c0de1a5f...
      dirty: true
      dirty_digest: sha256:9c5e5cd...
```

Entries are sorted and nothing is timestamped, so the file changes when the
deployment changes rather than every time it runs — a lock that rewrites itself
on every boot is one nobody reads a diff of. A dirty checkout gets a digest of
its uncommitted changes, not just a flag: a flag cannot tell two different
local edits apart.

A manifest pins packages by branch, and a cache is cloned once and then reused
untouched, so "the same manifest" is not the same code twice — a deployment was
found running three commits behind its branch with hand-applied edits on top,
and nothing about the run recorded it. The lockfile is that record. It is
written, not enforced: boot still runs whatever the cache holds, and a dirty
checkout is warned about rather than refused.

Next to the lock, `rbnx-boot/logs/` holds each component's log and
`rbnx-boot/sessions/` holds session state: boot passes it to system
components as `ROBONIX_SESSION_DIR` (an existing value wins), and Pilot keeps
each session's transcript there so the session survives a restart. Boot clears
old log files but never touches `sessions/`. `rbnx clean -f` removes
everything under `rbnx-boot/` except `cache/` and `sessions/`; pass `--cache`
or `--sessions` to remove those too.

## Built-in system components

`rbnx boot` launches each declared built-in component in dependency order:
Atlas, Executor, Soma, Vitals, Pilot, then Liaison. Their `system.<name>`
manifest blocks are translated into CLI flags and also passed intact through
`--config-json`. Components not present in the manifest are not started, except
that Soma is added automatically when the deployment contains primitives or
skills.

For example, this enables Vitals without a separate terminal or global
`RUST_LOG` setting:

```yaml
system:
  vitals:
    listen: 127.0.0.1:50093
    log: info
```

## Environment

- `ROBONIX_ATLAS` — atlas endpoint, default `localhost:50051`.
- `VLM_API_KEY` / `VLM_BASE_URL` / `VLM_MODEL` — VLM credentials forwarded
  to pilot at boot.
- `ROBONIX_DRIVER_INIT_TIMEOUT_S` — seconds `rbnx boot` waits for a package's
  `Driver(CMD_INIT)` reply. Default 90.
- `RBNX_GH_MIRROR` — prefix tried before `https://github.com/` URLs when
  cloning `url:` packages. Default `https://ghfast.top/`; set it empty to clone
  from GitHub directly.
- `ROBONIX_CODEGEN_BIN` — path to the `robonix-codegen` binary that
  `rbnx codegen` runs. Without it, rbnx looks in `~/.cargo/bin`, the
  workspace `target/`, and `PATH`, then falls back to `cargo run`.

`rbnx chat` also reads:

- `ROBONIX_LIAISON_ENDPOINT` — Liaison address. Without it, chat discovers
  Liaison through Atlas and falls back to `127.0.0.1:50081`.
- `ROBONIX_CHAT_MIC_NODE` / `ROBONIX_CHAT_SPEAKER_NODE` — provider ids of the
  microphone and speaker to use. They override `~/.robonix/chat.yaml`, which
  stores the choices made in the chat device picker (Ctrl+A reopens it).
- `ROBONIX_CHAT_ASR_NODE` / `ROBONIX_CHAT_TTS_NODE` — provider ids of the ASR
  and TTS services. Unset lets Liaison pick from Atlas.
- `ROBONIX_CHAT_VOICE_SECONDS` — recording length per voice turn. Unset or 0
  uses Liaison's default.
- `ROBONIX_CHAT_VOICE_LANG` — language hint passed to ASR.
- `ROBONIX_CHAT_VOICE_TTS` — set to `0`, `false`, `no`, or `off` to skip
  spoken replies.

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

## How it finds the source tree

Many subcommands (codegen, build, boot) need to resolve paths inside
the Robonix source repo (the IDL tree under `capabilities/lib`, the
atlas proto under `system/atlas/proto`, the Python SDK under
`pylib/robonix-api`). `rbnx setup` writes the absolute path of the
current clone into `~/.robonix/config.yaml`; later calls anywhere on
disk resolve those keys via `rbnx path`.

The repo root is identified by the presence of `Cargo.toml`,
`capabilities/`, and `capabilities/lib/` (the IDL tree).
