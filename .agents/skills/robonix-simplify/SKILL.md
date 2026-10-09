---
name: robonix-simplify
description: Shrink a Robonix change before review without changing behaviour. Use when a feature works and tests pass, before opening or updating a PR, and when a review asks for a smaller change.
---

# robonix-simplify

A working change is usually larger than it needs to be. This pass makes it smaller and plainer before anyone reviews it. The rules are the maintainability page of `.agents/guidelines/`.

## Steps

1. **Measure.** `git diff --stat <base>...HEAD`, excluding generated files. Note the size.
2. **Find the fat**, in this order, because the first items are the cheapest and safest:
   - comments and docstrings that restate the code, tell the history of a bug, or quote measurements (`comments-explain-why`, `no-history-in-code`);
   - code nothing calls, reads or configures (`delete-dead-code`) — search the whole repository, including `testing/` and `examples/`, before deleting;
   - options, parameters and environment variables nobody sets (`minimal-change`);
   - the same logic in two places (`dry`), and new helpers that duplicate existing ones (`reuse-first`);
   - wrappers, layers and classes with one caller (`no-single-use-abstraction`);
   - long functions and deep nesting (`small-functions`, `flat-control-flow`).
3. **Understand before deleting.** For each candidate, find why it exists (callers, tests, `git log -S`). If the reason still holds, keep it.
4. **Change in small steps**, running the relevant tests after each. Behaviour, public names, contracts and stored formats stay the same; renames follow `deprecate-dont-break`.
5. **Measure again** and report before/after line counts with the categories removed.

Stop when the remaining code is what a careful engineer would write by hand. Fewer lines is not the goal on its own: don't merge unrelated logic or inline a helper whose name explains a step.
