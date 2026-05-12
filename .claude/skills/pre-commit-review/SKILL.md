---
name: pre-commit-review
description: Run pr-review-toolkit agents over the current `git diff` before staging a commit. Catches CLAUDE.md violations, silent failures, comment rot, and test-coverage gaps in one shot.
---

# pre-commit-review

Run this **after** finishing a logical chunk of work and **before** staging
to git. It launches several specialised review agents in parallel on the
current unstaged diff, surfaces their findings to the user, and stops
short of committing — the human still decides whether to fix and commit.

## When to use

- A function / file / feature is done and you're about to `git add`.
- After resolving a bug fix, before pushing.
- After any rename / refactor that touched more than a couple of files.

Skip for trivial edits (typo fixes, comment-only changes) — the agent
fan-out cost isn't worth it.

## What it does

1. Read `git diff HEAD` (staged + unstaged) so the agents see the full set about to be committed.
2. Launch in parallel:
   - `pr-review-toolkit:code-reviewer` — CLAUDE.md / style adherence
   - `pr-review-toolkit:silent-failure-hunter` — catch-all blocks,
     fallbacks, swallowed errors introduced by the diff
   - `pr-review-toolkit:comment-analyzer` — comments that drifted from
     the code they describe (only if the diff added / changed comments)
3. If the diff introduces new types or trait/struct definitions, also
   run `pr-review-toolkit:type-design-analyzer`.
4. If the diff is large (≥ 300 LOC), also run `pr-review-toolkit:pr-test-analyzer`
   to check whether test coverage tracked the change.
5. Collate findings. Present to the user as a punch list. Do not
   auto-fix; let the user decide which findings to act on.

## Prereqs

The `pr-review-toolkit` plugin must be enabled in the user's Claude
Code config. Without it the named agents won't be available — print a
clear error and fall back to a manual checklist:

```
[ ] CLAUDE.md hard rules respected (concept stability, no blanket sed)
[ ] no catch-all that swallows an unexpected error
[ ] comments still match code
[ ] every new public type / function has a doc comment
[ ] tests added or updated for behaviour changes
```

## Output

After all agents return, print:

```
## pre-commit-review summary

Findings (N total):
  - [code-reviewer]    <issue>
  - [silent-failure]   <issue>
  - ...

Pass / discuss / fix? (no auto-commit)
```

Wait for the user's instruction. Never `git add` / `git commit` from
inside this skill.
