# Writing

*Do the commit, the PR and the comments say what matters, plainly?*

Reviewers read every line. Text that sounds generated costs them time and hides the one fact they need.

### plain-statements

State the fact and stop. Avoid the habits that mark generated text: "not X but Y" contrasts, a one-line closer that repeats the point, a staged run-up ("Here's the thing:"), lists of exactly three, a dash in every sentence, bold labels on every item, and words like *crucial, robust, seamless, leverage, delve, pivotal*. Don't claim significance; show the change.

### why-in-commit

Use Conventional Commits (`type(scope): subject`, imperative, under 72 characters). The body is prose: why the change is needed, what it changes, and anything a reviewer can't see in the diff. Wrap commit bodies at 72 columns.

### pr-says-why

A PR description starts with the problem, then what changes, then how it was verified (commands run, results). List breaking changes and deprecations explicitly. Keep it as short as the change allows.

### english-in-repo

Code, comments, commit messages, issues and PRs are in English. Translations live in data files (for example `strings_<lang>.json`).

### no-hard-wrap-in-descriptions

In PR and issue bodies, write each paragraph on one line; GitHub reflows it. Source comments and commit messages are wrapped as usual.

### human-authorship

A commit's author and committer are human. Don't add `Co-Authored-By` or similar trailers naming an agent. Disclose material AI help only as `Assisted-by: AGENT_NAME:MODEL_VERSION`, as `CONTRIBUTING.md` describes; CI enforces this.
