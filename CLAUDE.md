# Robonix — Claude Code rules

This file overrides default Claude behavior for anyone working on this
repository with Claude Code. Read once at session start; obey throughout.

## Concept and naming stability (v0.1 lock)

- `docs/src/developer-guide.md` is the source of truth for concepts and
  terminology. When code and the dev guide disagree, fix the code, not
  the guide.
- The four formal concepts are **capability / contract / primitive /
  service / skill**. Do not introduce additional umbrella terms in
  user-facing prose (dev guide, README, quickstart, CLI output, error
  messages, commit messages). The internal Rust type `CapabilityProvider`
  / Python `_ProviderBase` / fields `provider_id`, `provider_kind` are
  implementation labels and stay as-is.
- No new concept / rename / RPC / state name without explicit user
  confirmation in the conversation. If you find yourself about to
  introduce a noun or verb that is not already in the dev guide, **stop
  and ask** before writing code.
- Wire format (`atlas.proto`, `Driver.srv`, contract TOML schema) and
  `pylib/robonix-api/` public surface are frozen for v0.1. Changes
  require an explicit discussion thread, not a patch.

## Editing discipline

- Never run `sed` (or equivalent multi-file replace) across more than
  one file at a time during a rename. Read each file before editing.
  Pattern variables that look identical at the string level can have
  different types — blanket replace breaks tuple destructures and
  loop variables. This rule exists because we already hit that bug
  twice.
- `cargo fmt --all && cargo clippy --workspace --all-targets -- -D warnings`
  must be clean before any commit that touches Rust.
- Don't commit without an end-to-end run when the change crosses
  process boundaries (atlas wire, Driver lifecycle, pylib singleton).
  "It compiles + clippy clean" is not enough — those passes missed the
  `wait_for_registration` race and a tuple-shadow bug in this repo.

## Review before commit

For non-trivial diffs, run the pr-review-toolkit agents in parallel
before staging:

- `pr-review-toolkit:code-reviewer` — CLAUDE.md adherence + style
- `pr-review-toolkit:silent-failure-hunter` — fallback / swallowed error
- `pr-review-toolkit:type-design-analyzer` — type discipline (when
  introducing new types)
- `pr-review-toolkit:pr-test-analyzer` — test coverage of the diff

A repo-local skill `.claude/skills/pre-commit-review/` orchestrates
this; invoke with `/pre-commit-review` after a logical chunk is done.

## Commit policy

- Don't commit unless the user explicitly says to. Multiple smaller
  reviewable commits beat one mega-commit.
- Don't force-push to `main` ever. Force-push to `dev` / `dev-wheatfox`
  only when the user explicitly authorizes it (e.g. to fix an amend
  that's already been pushed).
- Always create new commits rather than amending pushed commits,
  unless the user asks for an amend.

## Output discipline

- Don't generate large blocks of new prose / code unless asked. The
  user reviews everything; long outputs are expensive to read.
- When proposing a design or a multi-step plan, present the headline +
  trade-off in 2–3 sentences first. Expand only on request.
- After completing a task, summarise in one or two sentences. No
  trailing recap sections.

## Upstream packages

`mapping_rbnx`, `explore_rbnx`, `template_rbnx` are separate
repositories cloned into `examples/webots/rbnx-boot/cache/` at boot.
When you rename / break something that affects them, push the
upstream fix first, then pull cache. Don't rely on local cache edits
surviving — `rbnx clean --cache` will wipe them.
