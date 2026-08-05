#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TIAGO_VARIANT="${ROBONIX_TIAGO_VARIANT:-lite}"
RBNX_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --tiago-variant)
      [[ $# -ge 2 ]] || { echo "[webots/boot] --tiago-variant requires a value" >&2; exit 2; }
      TIAGO_VARIANT="$2"
      shift 2
      ;;
    --help|-h)
      echo "Usage: $0 [--tiago-variant lite|full] [rbnx boot options]"
      echo "Example: $0 --tiago-variant full --no-update-check"
      exit 0
      ;;
    *)
      RBNX_ARGS+=("$1")
      shift
      ;;
  esac
done

case "$TIAGO_VARIANT" in
  lite)
    export ROBONIX_SOMA_ROBOT_YAML="$SCRIPT_DIR/soma.yaml"
    ;;
  full)
    export ROBONIX_SOMA_ROBOT_YAML="$SCRIPT_DIR/soma.full.yaml"
    ;;
  *)
    echo "[webots/boot] unsupported TIAGo variant: $TIAGO_VARIANT (choose lite or full)" >&2
    exit 2
    ;;
esac

export ROBONIX_TIAGO_VARIANT="$TIAGO_VARIANT"
echo "[webots/boot] using TIAGo variant: $ROBONIX_TIAGO_VARIANT"
echo "[webots/boot] using Soma robot YAML: $ROBONIX_SOMA_ROBOT_YAML"
exec rbnx boot -f "$SCRIPT_DIR/robonix_manifest.yaml" "${RBNX_ARGS[@]}"
