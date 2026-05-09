# Robonix Agent Guidelines

This file defines repository-specific rules for AI coding agents working on
Robonix. Read once at session start; obey throughout.

## Concept and naming stability (v0.1 lock)

- `docs/src/developer-guide.md` is the source of truth for concepts and
  terminology. When code and the dev guide disagree, fix the code, not
  the guide.
- The formal concepts are **capability** (the interface) + **contract**
  (its shape) + three provider kinds **primitive / service / skill**.
  Do not introduce additional umbrella terms in user-facing prose
  (dev guide, README, quickstart, CLI output, error messages, commit
  messages). The internal Rust type `CapabilityProvider` / Python
  `_ProviderBase` / fields `provider_id`, `provider_kind` are
  implementation labels and stay as-is.
- No new concept / rename / RPC / state name without explicit user
  confirmation in the conversation. If you find yourself about to
  introduce a noun or verb that is not already in the dev guide, **stop
  and ask** before writing code.
- Wire format (`atlas.proto`, `Driver.srv`, contract TOML schema) and
  `pylib/robonix-api/` public surface are frozen for v0.1. Changes
  require an explicit discussion thread, not a patch.

## Project Structure & Module Organization

Robonix is an **embodied AI operating system**: a monorepo with Rust binaries,
Python service packages, capability contracts, and documentation. Rust code
lives in `rust/crates/*`; shared protobuf sources are in `rust/proto/`. Python
workspace packages are under `pylib/*` and `system/*`, managed by the root
`pyproject.toml` and `uv.lock`. Capability schemas are TOML files under
`capabilities/{primitive,service,system}/`. End-to-end examples live in
`examples/`, Docker packaging in `docker/`, and the mdBook manual in
`docs/src/`.

### Crate README maintenance

When you add or change functionality in a crate under `rust/crates/*`, update
that crate's `README.md` in the same contribution. If the crate has no
`README.md`, add one. Each crate README should be concise and informative:
what the package is, what it is for, and how to use it. For CLIs and similar
tools, document subcommands, flags, and important environment variables.

## Build, Test, and Development Commands

Run Rust commands from `rust/` unless noted:

- `make build`: build all Rust workspace crates in debug mode.
- `make release`: build all Rust crates with `--release`.
- `make install`: install `rbnx`, Atlas, Pilot, Executor, Liaison, and Codegen
  to `~/.cargo/bin`.
- `make check`: run `cargo fmt --all -- --check` and clippy with warnings
  denied.
- `cargo test --workspace --all-targets`: run unit and integration tests.
- `uv sync`: resolve Python workspace dependencies from the repo root; service
  `scripts/build.sh` files usually perform package-local setup.
- `cd docs && mdbook build`: build the documentation book when mdBook and
  preprocessors are installed.

## Testing Guidelines

Place Rust integration tests in each crate's `tests/` directory and keep unit
tests near the code behind `#[cfg(test)]`. CI currently gates Rust format,
clippy, build, and `cargo test --workspace --all-targets`. For Python services,
add package-local smoke tests or scripts when changing runtime behavior, and
document required environment variables in the service README.

## Editing discipline

- Never run `sed` (or equivalent multi-file replace) across more than one file
  at a time during a rename. Read each file before editing. Pattern variables
  that look identical at the string level can have different types; blanket
  replace breaks tuple destructures and loop variables.
- `cargo fmt --all && cargo clippy --workspace --all-targets -- -D warnings`
  must be clean before any commit that touches Rust.
- Don't commit without an end-to-end run when the change crosses process
  boundaries (atlas wire, Driver lifecycle, pylib singleton). "It compiles +
  clippy clean" is not enough.

## Review before commit

For non-trivial diffs, run the pr-review-toolkit agents in parallel before
staging:

- `pr-review-toolkit:code-reviewer` — AGENTS.md adherence + style.
- `pr-review-toolkit:silent-failure-hunter` — fallback / swallowed error.
- `pr-review-toolkit:type-design-analyzer` — type discipline, when introducing
  new types.
- `pr-review-toolkit:pr-test-analyzer` — test coverage of the diff.

A repo-local Claude skill `.claude/skills/pre-commit-review/` orchestrates
this; invoke with `/pre-commit-review` after a logical chunk is done when using
Claude Code. Other agents should follow the same review intent with the tools
available to them.

## Commit & Pull Request Guidelines

- Don't commit unless the user explicitly says to. Multiple smaller reviewable
  commits beat one mega-commit.
- Don't force-push to `main` ever. Force-push to `dev` / `dev-wheatfox` only
  when the user explicitly authorizes it.
- Always create new commits rather than amending pushed commits, unless the
  user asks for an amend.
- Use short, imperative, scope-prefixed commit subjects such as
  `audio: list_devices + select_device contracts` or
  `rbnx chat: rename agent reply prefix`.
- Pull requests should describe behavior changes, list validation commands run,
  link related issues, and include screenshots or terminal output for CLI/TUI
  changes. Note any new capability contracts, generated code impacts, or
  configuration requirements.

## Output discipline

- Don't generate large blocks of new prose / code unless asked. The user
  reviews everything; long outputs are expensive to read.
- When proposing a design or a multi-step plan, present the headline +
  trade-off in 2-3 sentences first. Expand only on request.
- After completing a task, summarise in one or two sentences. No trailing recap
  sections.

## Upstream packages

`mapping_rbnx` and `explore_rbnx` are separate repositories cloned into
`examples/webots/rbnx-boot/cache/` at boot. `template_rbnx` is a deployment
scaffold users clone separately; it does **not** live in the webots cache. When
you rename / break something that affects any of them, push the upstream fix
first, then pull cache. Don't rely on local cache edits surviving:
`rbnx clean --cache` will wipe them.
