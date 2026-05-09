#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Claude Code PreToolUse hook — if the about-to-run Bash command is a
# `git push` from this repo, run the same Rust CI steps GitHub Actions
# runs (.github/workflows/ci.yml) and block the push if any step fails.
#
# Wire-up in .claude/settings(.local).json:
#
#   { "hooks": { "PreToolUse": [ { "matcher": "Bash", "hooks": [
#       { "type": "command",
#         "command": "<repo>/scripts/check-rust-ci.sh" } ] } ] } }
#
# The script reads the tool-call JSON on stdin, peeks at tool_input.command,
# and only runs the (slow-ish) cargo gauntlet when that command is a
# `git push` issued from inside this repo. Anything else early-exits
# 0 in milliseconds so the rest of the session is unaffected.
#
# Block convention: exit code 2 + stderr message → Claude Code suppresses
# the tool call and feeds stderr back to the model so the error is
# actionable on the next turn.

set -uo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUST_DIR="$REPO_ROOT/rust"

stdin_json="$(cat)"

# Tool name + command. jq -r returns "null" if missing — be defensive.
tool_name="$(printf '%s' "$stdin_json" | jq -r '.tool_name // ""')"
cmd="$(printf '%s' "$stdin_json" | jq -r '.tool_input.command // ""')"

if [[ "$tool_name" != "Bash" ]]; then
    exit 0
fi

# Catch `git push`, `git push origin foo`, leading env-var prepends like
# `FOO=1 git push`, but skip `git pushed` typos and unrelated `push` words.
if ! printf '%s\n' "$cmd" | grep -qE '(^|[[:space:];&|])git[[:space:]]+push([[:space:];&|]|$)'; then
    exit 0
fi

# Only enforce when the push originates from this repo. The hook's CWD
# at call time is the agent's working directory; if a coordinator agent
# happens to invoke from outside this tree, just skip silently.
agent_cwd="$(pwd -P 2>/dev/null || echo "")"
case "$agent_cwd/" in
    "$REPO_ROOT"/*) ;;
    *)
        exit 0 ;;
esac

if [[ ! -d "$RUST_DIR" ]]; then
    exit 0
fi

cd "$RUST_DIR"

# Match ci.yml env exactly so local results == CI.
export CARGO_TERM_COLOR=always
export CARGO_INCREMENTAL=0

step() {
    local name="$1"
    shift
    if ! "$@" >&2; then
        echo "" >&2
        echo "[claude-hook] BLOCKED: \`$name\` failed locally — fix before pushing." >&2
        echo "[claude-hook] To bypass once, retry the push in a tool call that doesn't" >&2
        echo "[claude-hook] match \`git push\` (or invoke after fixing). See rust/ output above." >&2
        exit 2
    fi
}

echo "[claude-hook] git push detected — running CI gauntlet from $RUST_DIR" >&2
step "cargo fmt --all -- --check"             cargo fmt --all -- --check
step "cargo clippy --workspace --tests"       cargo clippy --workspace --tests -- -D warnings
step "cargo build --workspace"                cargo build --workspace
step "cargo test --workspace --all-targets"   cargo test --workspace --all-targets
echo "[claude-hook] all CI checks passed locally; allowing push" >&2
exit 0
