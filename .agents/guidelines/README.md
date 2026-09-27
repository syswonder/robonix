# Robonix coding guidelines

These pages are the standard for writing and for reviewing code in this repository, for people and agents alike. Each page is one reviewer's point of view, and its index is that reviewer's checklist: a stable `short-name` and a one-line rule. A review comment cites the short-name it relies on.

| Page | The question it asks |
|---|---|
| [Maintainability](maintainability.md) | Is this the smallest clear change, and will the next reader follow it? |
| [Correctness](correctness.md) | Does it do what it claims, including when things fail? |
| [Security](security.md) | Can it be misused, or leak what it should not? |
| [Writing](writing.md) | Do the commit, the PR and the comments say what matters, plainly? |

## Index

**Maintainability**
- [`minimal-change`](maintainability.md#minimal-change): Write the least code that solves the request; nothing speculative.
- [`no-single-use-abstraction`](maintainability.md#no-single-use-abstraction): Don't add a class, layer, option or hook that has one caller.
- [`reuse-first`](maintainability.md#reuse-first): Use the helper, module or contract that already exists before writing a new one.
- [`one-concept-one-name`](maintainability.md#one-concept-one-name): Every concept has one name across code, contracts, UI and docs.
- [`dry`](maintainability.md#dry): Once the same logic appears twice, give it one home.
- [`small-functions`](maintainability.md#small-functions): A function does one thing; split one that needs section comments.
- [`flat-control-flow`](maintainability.md#flat-control-flow): Return early; don't nest past three levels.
- [`delete-dead-code`](maintainability.md#delete-dead-code): Remove what nothing calls, reads or configures.
- [`comments-explain-why`](maintainability.md#comments-explain-why): A comment states a reason the code cannot; keep it to a line or two.
- [`no-history-in-code`](maintainability.md#no-history-in-code): Bug stories, measurements and "this used to" belong in the commit, not the source.
- [`focused-pr`](maintainability.md#focused-pr): One topic per PR; refactoring in its own commit, before the feature.
- [`deprecate-dont-break`](maintainability.md#deprecate-dont-break): Keep a published name as a marked deprecated alias when you rename it.

**Correctness**
- [`no-silent-failure`](correctness.md#no-silent-failure): Don't swallow an error; handle it, report it, or let it propagate.
- [`no-impossible-handling`](correctness.md#no-impossible-handling): Don't guard against states the code cannot reach.
- [`contract-first`](correctness.md#contract-first): Read the capability contract before calling it; async contracts are not blocking calls.
- [`bounded-resources`](correctness.md#bounded-resources): Anything that grows with runtime (caches, logs, recordings, files) has a bound.
- [`dont-block-the-loop`](correctness.md#dont-block-the-loop): Keep slow or GIL-holding work off asyncio loops and request threads.
- [`test-the-behaviour`](correctness.md#test-the-behaviour): Few tests, of behaviour that can break; one test file per module; never assert source or markup text.
- [`end-to-end-for-boundaries`](correctness.md#end-to-end-for-boundaries): A change across processes is verified by running them, not by compiling.

**Security**
- [`escape-untrusted-text`](security.md#escape-untrusted-text): Text from users, models or the network is escaped before it becomes markup, SQL or a shell argument.
- [`local-by-default`](security.md#local-by-default): Bind internal servers to localhost; expose only through the component's own port.
- [`no-secrets`](security.md#no-secrets): No keys, tokens or personal paths in code, logs or commits.
- [`validate-paths`](security.md#validate-paths): Ids that become file paths are sanitised before use.

**Writing**
- [`plain-statements`](writing.md#plain-statements): Say the fact. No staged contrasts, closing one-liners, forced triads or inflated words.
- [`why-in-commit`](writing.md#why-in-commit): The commit body says why the change is needed and what it changes, in prose.
- [`pr-says-why`](writing.md#pr-says-why): A PR description starts with the problem, then the change, then how it was verified.
- [`english-in-repo`](writing.md#english-in-repo): Code, comments, commits, issues and PRs are in English.
- [`no-hard-wrap-in-descriptions`](writing.md#no-hard-wrap-in-descriptions): In PR and issue bodies, one paragraph is one line.
- [`human-authorship`](writing.md#human-authorship): A human is the author; AI help is disclosed only with `Assisted-by`.
