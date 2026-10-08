# Maintainability

*Is this the smallest clear change, and will the next reader follow it?*

Agents tend to solve a problem the most elaborate way available: extra layers, options nobody asked for, and a paragraph of prose above every function. Review for the opposite. A diff that could be half its size should be.

### minimal-change

Write the least code that solves the request. No features, parameters, environment variables or fallbacks beyond what was asked. If 200 lines could be 50, rewrite them before review.

### no-single-use-abstraction

Don't add a class, base class, registry, factory, strategy or hook that has one implementation or one caller. Inline it. Add the abstraction when the second caller arrives.

### reuse-first

Before writing a helper, search for one. Robonix already has shared code for geometry (`system/scene/scene_service/geometry.py`), capability calls (`robonix_api`), codegen (`rbnx codegen`) and configuration (`config.spec` per package). A second implementation of the same thing is a defect even when both work.

### one-concept-one-name

A concept has one name everywhere: code, contracts, UI strings, docs. `docs/src/developer-guide.md` is the source of truth for terms. Don't introduce a synonym for an existing concept; don't reuse a name for a different one.

### dry

When the same logic appears a second time, move it to one place and call it from both. Request-handling boilerplate, validation and formatting are the usual repeats.

### small-functions

A function does one thing. If it needs comments to mark its sections, split it at those comments. Keep a request handler to parsing, one call, and a response.

### flat-control-flow

Use guard clauses and early returns. Past three levels of nesting, restructure.

### delete-dead-code

Remove functions nothing calls, fields nothing reads, settings nothing sets and branches nothing reaches. Check with a search across the repository (including `testing/` and `examples/`) before deleting, and delete in the same PR that made it dead.

### comments-explain-why

A comment says why the code is the way it is when the code cannot say it: an external constraint, a non-obvious invariant, a workaround with its cause. One or two lines. Don't restate what the code does, and don't write a docstring for a function whose name and signature already say it.

### no-history-in-code

"This used to crash when…", "measured 13.9 KB on…", "an earlier version did…" go in the commit message or the PR, where they stay attached to the change. The source describes the code as it is.

### focused-pr

One topic per PR. Keep a PR reviewable in one sitting, roughly under 1,000 changed lines excluding generated files. Put a refactor in its own commit before the feature that needs it.

### deprecate-dont-break

When renaming something other packages or saved data depend on (a capability, a route, a payload field, a stored value), keep the old name as an alias marked deprecated in code and docs, and read old stored data. Remove the alias in a later release.
