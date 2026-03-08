#!/usr/bin/env bash
# Fetch each repo per distro branch into repos/<repo>/<distro>
# Usage: ./scripts/fetch_repos.sh [--refresh] [--dry-run]

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR"
exec python3 "$SCRIPT_DIR/fetch_repos.py" "$@"
