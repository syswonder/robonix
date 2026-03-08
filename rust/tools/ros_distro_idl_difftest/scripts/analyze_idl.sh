#!/usr/bin/env bash
# Analyze IDL differences across distros under repos/, write reports to reports/
# Usage: ./scripts/analyze_idl.sh [--repos-dir repos] [--out-dir reports] [--distros humble jazzy rolling]

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR"
exec python3 "$SCRIPT_DIR/analyze_idl.py" "$@"
