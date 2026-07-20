# Contributing to Robonix

Robonix accepts contributions from people who can review, explain, license,
validate, and maintain the changes they submit. This guide applies to code,
capability contracts, system components, examples, and repository
documentation in `syswonder/robonix`.

## Prepare a focused change

Base normal development work on the latest `dev` branch and create a focused
topic branch:

```bash
git switch dev
git pull --ff-only origin dev
git switch -c fix/short-description
```

Read the repository-root `AGENTS.md` and the README for every component you
change. Keep each commit limited to one independently reviewable problem. Do
not rename public concepts, change capability contracts, or reorganize
unrelated code as part of an incidental fix.

## License and SPDX identifiers

Original Robonix source code is licensed under MulanPSL-2.0. Add the matching
SPDX identifier at the beginning of every new original source file:

```rust
// SPDX-License-Identifier: MulanPSL-2.0
```

```python
# SPDX-License-Identifier: MulanPSL-2.0
```

Keep a shell script's shebang on the first line and put the SPDX identifier on
the second line:

```bash
#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
```

Preserve the existing license, copyright notice, and SPDX identifier of code
imported from another project. Do not relabel third-party code as
MulanPSL-2.0. Contributors are responsible for confirming provenance and
license compatibility before submission.

## Code style and repository layout

- Rust uses edition 2024 and standard `rustfmt`. Use `snake_case` for modules
  and functions, `CamelCase` for types, and `robonix-*` crate names.
- Python targets Python 3.10 or newer, uses four-space indentation, and uses
  `snake_case` for modules and functions. Run the checks supplied by the
  package being changed; the repository does not define one global Python
  formatter command.
- Capability contracts live under the appropriate `capabilities/` category
  and use `<interface>.v1.toml` filenames. Shared IDL lives under
  `capabilities/lib/`.
- Put Rust unit tests next to the code behind `#[cfg(test)]` and integration
  tests in the crate's `tests/` directory. Add package-local tests or smoke
  tests for Python runtime changes.
- When functionality changes in a Rust component under `system/*` or
  `tools/*`, update that component's README in the same contribution.
- Do not hand-edit generated documentation under `docs/src/reference/`.
  Change the contracts, IDL, or generator inputs and regenerate it.

The repository requires a comment for a function or method body longer than
five lines. Describe at least its behavior and document side effects or
constraints that are not evident from the signature.

## Build and validation

Run Rust commands from the repository root. Before submitting a Rust change,
run:

```bash
make fmt
make check
cargo build --workspace
cargo test --workspace --all-targets
```

`make fmt` runs `cargo fmt --all`. `make check` verifies formatting and runs
workspace clippy with warnings denied. GitHub Actions additionally builds the
workspace and runs all workspace targets.

Run the package-specific tests, build scripts, and smoke tests for Python,
contracts, code generation, Webots, or hardware-facing changes. Changes that
cross an Atlas wire protocol, Driver lifecycle, or Python API singleton
boundary require an end-to-end run; compilation alone is not sufficient.

## Commit messages

Use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/):

```text
<type>(optional scope): <imperative description>
```

Recommended types are `feat`, `fix`, `docs`, `refactor`, `test`, `perf`, `ci`,
`build`, and `chore`. Examples:

```text
feat(atlas): expose provider health query
fix(pilot): preserve active plan on steer
docs: explain package configuration
ci: check commit authorship
```

Use `!` before the colon and a `BREAKING CHANGE:` footer for an incompatible
change. Avoid vague subjects such as `update`, `changes`, or `fix stuff`.

## Human authorship and AI assistance

This policy applies to commits submitted on or after July 20, 2026.

- A commit's Git author and committer must be human contributors. An AI coding
  agent must not be named in either field.
- Do not attribute authorship or responsibility to an AI agent through
  `Co-authored-by`, `Co-developed-by`, `Signed-off-by`, `Reviewed-by`,
  `Tested-by`, `Acked-by`, or `Suggested-by` trailers.
- The human author must review and understand the complete change, confirm its
  provenance and license compatibility, run appropriate validation, and accept
  full responsibility for its correctness, security, and maintenance.
- AI assistance is permitted. When an AI tool materially assists a change,
  disclose it in the commit message with the Robonix `Assisted-by` format:

  ```text
  Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL ...]
  ```

  For example:

  ```text
  Assisted-by: Codex:gpt-5.6 clang-tidy
  ```

  `AGENT_NAME` identifies the tool or agent, `MODEL_VERSION` identifies the
  model, and optional trailing names identify specialized analysis tools. Do
  not put an email address in `Assisted-by`, and do not list ordinary tools
  such as Git, compilers, build systems, editors, or shells.

Robonix defines this as its own contribution policy. Its `Assisted-by` syntax
adopts a documented convention from the Linux kernel community, whose public
guidance also distinguishes human DCO certification and responsibility from
tool attribution. These documents are references for the trailer format, not
the governance policy of this project:

- <https://docs.kernel.org/process/coding-assistants.html>
- <https://docs.kernel.org/process/submitting-patches.html#using-assisted-by>

CI checks every commit introduced by a pull request or protected-branch push.
It rejects known AI identities in Git authorship fields and responsibility or
authorship trailers, while allowing a correctly formatted `Assisted-by`
trailer. The check cannot determine whether undisclosed AI assistance occurred;
accurate disclosure remains the human contributor's responsibility.

Run the same check locally against a proposed range:

```bash
python3 scripts/check_commit_authorship.py --base origin/dev --head HEAD
```

## Commit trailers

Trailers record specific human actions and must not be added without the named
person's knowledge:

- `Signed-off-by: Name <email>` certifies the
  [Developer Certificate of Origin 1.1](https://developercertificate.org/).
  Robonix does not currently require a sign-off on every commit. When a human
  can make that certification and chooses or is asked to sign, use
  `git commit -s`.
- `Co-authored-by: Name <email>` identifies another human who wrote the
  change.
- `Co-developed-by: Name <email>` identifies a human co-developer and is
  immediately followed by that person's own `Signed-off-by`.
- `Reviewed-by`, `Tested-by`, and `Acked-by` record actions explicitly
  performed and approved for attribution by the named human.
- `Fixes: <commit> ("<subject>")` identifies the commit that introduced a
  regression. Use at least 12 hexadecimal characters and include its subject.
- `Assisted-by` is the only trailer for AI assistance and does not assign
  authorship, review, testing, or legal certification.

## Pull requests

Push the topic branch and open a pull request against `dev`. The pull request
must:

1. explain the problem, user-visible impact, and implementation;
2. list the validation commands actually run and their results;
3. identify capability-contract, generated-code, configuration, or migration
   effects;
4. link related issues, for example with `Closes #123`; and
5. include screenshots or terminal output for CLI, TUI, or web UI changes.

Do not commit credentials, model keys, generated secrets, build artifacts, or
unrelated local changes. All required checks must pass before merge.
