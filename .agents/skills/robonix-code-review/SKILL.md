---
name: robonix-code-review
description: Review a Robonix change against the repository's coding guidelines and write a Markdown review. Use when asked to review a branch, a PR, or files, and before asking a human to review your own change.
---

# robonix-code-review

Review a change against `.agents/guidelines/` and write one Markdown file. Every comment cites the guideline short-name it rests on, so the author can look the rule up and the reviewer can check the call.

## Input

One argument string, in one of two forms:

```
diff <base> <output>          # the commits on HEAD since merge-base(<base>, HEAD)
files <path>... <output>      # the current contents of these files
```

Produce the review input with the script, from the repository root:

```sh
.agents/skills/robonix-code-review/scripts/review_input.sh diff origin/dev > /tmp/review-input.txt
```

In `diff` mode the input is each commit's message and patch, so commit hygiene is reviewed along with the code.

## Steps

1. **Read the guidelines.** Read `.agents/guidelines/README.md`. Open a page only for the rules you need.
2. **One pass per page.** Review the input once for each guideline page (maintainability, correctness, security, writing), each in a fresh sub-agent that gets only that page and the input. Separate passes find more than one pass looking for everything. Skip a page only when the input provably has nothing in its scope (no prose → skip writing).
3. **Comment format.** Each pass returns comments as `file`, `line`, `short-name`, `severity` (blocker / major / minor), `problem` (one sentence), `fix` (what to change). No comment without a short-name.
4. **Try to refute each comment.** Re-read the cited code. For a claim about a library, contract or protocol, check the source or the contract file. Keep it, mark it `(unverified)`, or drop it; list dropped comments at the end with a one-line reason.
5. **Group by cause.** When several comments share one fix (for example the same boilerplate in five handlers), give them one fix and point each comment at it. Keep every comment at its own location.
6. **Size check.** Report the diff size. If the maintainability pass finds more than a few `minimal-change`, `dry` or `delete-dead-code` comments, say plainly that the change should shrink before a human reviews it.
7. **Write the file.** Front matter (`date`, `mode`, `base`, `head`), a short summary (what is good, the top problems by severity), then one section per page with its comments sorted by file and line.

Don't fix the code during a review; the review is the output.
