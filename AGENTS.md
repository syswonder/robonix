# Repository Guidelines

## Project Structure & Module Organization

Robonix is an **embodied AI operating system**—a monorepo with Rust binaries, Python service packages, capability contracts, and documentation. Rust code lives in `rust/crates/*`; shared protobuf sources are in `rust/proto/`. Python workspace packages are under `pylib/*` and `system/*`, managed by the root `pyproject.toml` and `uv.lock`. Capability schemas are TOML files under `capabilities/{primitive,service,system}/`. End-to-end examples live in `examples/`, Docker packaging in `docker/`, and the mdBook manual in `docs/src/`.

### Crate README maintenance

When you add or change functionality in a crate under `rust/crates/*`, update that crate’s `README.md` in the same contribution. If the crate has no `README.md`, add one. Each crate README should be concise and informative: what the package is, what it is for, and how to use it (for CLIs and similar tools, document subcommands, flags, and important environment variables). Cover the usual README expectations—installation or build notes, quick start, and links to deeper docs when useful—without unnecessary verbosity.

## Build, Test, and Development Commands

Run Rust commands from `rust/` unless noted:

- `make build`: build all Rust workspace crates in debug mode.
- `make release`: build all Rust crates with `--release`.
- `make install`: install `rbnx`, Atlas, Pilot, Executor, Liaison, and Codegen to `~/.cargo/bin`.
- `make check`: run `cargo fmt --all -- --check` and clippy with warnings denied.
- `cargo test --workspace --all-targets`: run unit and integration tests.
- `uv sync`: resolve Python workspace dependencies from the repo root; service `scripts/build.sh` files usually perform package-local setup.
- `cd docs && mdbook build`: build the documentation book when mdBook and preprocessors are installed.

## Coding Style & Naming Conventions

Rust uses edition 2024 and standard `rustfmt`; keep clippy clean with `-D warnings`. Use `snake_case` for Rust modules/functions, `CamelCase` for types, and crate names matching `robonix-*`. Python targets 3.10+, uses 4-space indentation, and keeps package modules in `snake_case`. Capability files follow `<interface>.v1.toml` naming, grouped by domain, for example `capabilities/primitive/camera/rgb.v1.toml`.

## Testing Guidelines

Place Rust integration tests in each crate’s `tests/` directory and keep unit tests near the code behind `#[cfg(test)]`. CI currently gates Rust format, clippy, build, and `cargo test --workspace --all-targets`. For Python services, add package-local smoke tests or scripts when changing runtime behavior, and document required environment variables in the service README.

## Commit & Pull Request Guidelines

Recent commits use short, imperative, scope-prefixed subjects such as `audio: list_devices + select_device contracts` or `rbnx chat: rename agent reply prefix`. Follow that style: `<scope>: <concise change>`. Pull requests should describe behavior changes, list validation commands run, link related issues, and include screenshots or terminal output for CLI/TUI changes. Note any new capability contracts, generated code impacts, or configuration requirements.
