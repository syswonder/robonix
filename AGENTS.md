# Robonix Agent Guidelines

This file defines repository-specific rules for AI coding agents working on
Robonix. Read once at session start; obey throughout.

## Concept and naming stability

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

## Project Structure & Module Organization

Robonix is an **embodied AI operating system**: a monorepo with Rust system
components, Python packages, capability contracts, IDL libraries, examples, and
documentation. The Rust workspace lives at the repository root in `Cargo.toml`;
current Rust members are `system/{atlas,executor,liaison,pilot}` and
`tools/{rbnx,codegen}`. The twelve system component directories live under
`system/`; several are documentation/planning components today, while
`system/scene` is a Python package managed by uv. Python workspace packages are
listed in the root `pyproject.toml`: shared libraries in `pylib/*`, reference
services in `services/*`, and `system/scene`. Capability contracts are TOML
files under `capabilities/{primitive,service,system}/`; reusable message and
service IDL libraries live under `capabilities/lib/`. End-to-end and Webots
examples live in `examples/`, Docker packaging in `docker/`, repo scripts in
`scripts/`, image assets in
`images/`, and the mdBook manual in `docs/src/`.

### Crate README maintenance

When you add or change functionality in a Rust workspace member under
`system/*` or `tools/*`, update that member's `README.md` in the same
contribution. If the member has no `README.md`, add one. Each README should be
concise and informative: what the package is, what it is for, and how to use
it. For CLIs and similar tools, document subcommands, flags, and important
environment variables.

## Build, Test, and Development Commands

Run Rust commands from the repository root unless noted:

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

## Coding Style & Naming Conventions

Rust uses edition 2024 and standard `rustfmt`; keep clippy clean with
`-D warnings`. Use `snake_case` for Rust modules/functions, `CamelCase` for
types, and crate names matching `robonix-*`. Python targets 3.10+, uses
4-space indentation, and keeps package modules in `snake_case`. Capability
files follow `<interface>.v1.toml` naming, grouped by domain, for example
`capabilities/primitive/camera/rgb.v1.toml`.

If one function/method body exceeds five lines, treat documentation as mandatory: The comment block must state behavior at minimum; add side effects, or constraints when they are not obvious from the signature.

## Testing Guidelines

Place Rust integration tests in each crate's `tests/` directory and keep unit
tests near the code behind `#[cfg(test)]`. CI currently gates Rust format,
clippy, build, and `cargo test --workspace --all-targets`. For Python services,
add package-local smoke tests or scripts when changing runtime behavior, and
document required environment variables in the service README.

Do not add production-code solely to make tests pass; if tests require
production changes, those changes must improve real behavior, design clarity,
diagnostics, or maintainability outside the test.

## Editing discipline

- Never run `sed` (or equivalent multi-file replace) across more than one file
  at a time during a rename. Read each file before editing. Pattern variables
  that look identical at the string level can have different types; blanket
  replace breaks tuple destructures and loop variables.
- Do not hand-edit generated documentation under `docs/src/reference/`
  (including `idl.md` and `contracts.md`). Update the source contracts / IDL /
  codegen inputs, then regenerate docs with the project tooling when generated
  reference pages need to change.
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
- A commit's Git author must be a human contributor. Never use an AI agent in
  the author or committer fields, or in authorship, sign-off, review, or test
  trailers. When AI materially assists a change, the human author may disclose
  it only as `Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL ...]` and remains
  fully responsible for the contribution. See `CONTRIBUTING.md`.
- Don't force-push to `main` ever. Force-push to `dev` / `dev-wheatfox` only
  when the user explicitly authorizes it.
- Always create new commits rather than amending pushed commits, unless the
  user asks for an amend.
- Write commit messages using the
  [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
  format: `<type>(optional scope): <description>`. Use imperative mood in the
  description. Prefer `feat` and `fix` when they apply; otherwise use types
  such as `docs`, `chore`, `refactor`, `test`, `perf`, `ci`, or `build`.
- For breaking changes, use `!` before `:` or a `BREAKING CHANGE:` footer.
- Pull requests should describe behavior changes, list validation commands you
  ran, link related issues, and include screenshots or terminal output for CLI
  or TUI changes. Note any new capability contracts, generated code impacts, or
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
