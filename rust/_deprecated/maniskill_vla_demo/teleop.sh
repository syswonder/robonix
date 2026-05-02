#!/usr/bin/env bash
# teleop.sh — ManiSkill3 keyboard teleoperation (SAPIEN GUI + matplotlib viewer)
#
# Usage:
#   ./teleop.sh                                          # ReplicaCAD + Fetch (default)
#   ./teleop.sh PickCube-v1                              # simpler scene for practice
#   ./teleop.sh ReplicaCADTidyHouseTrain_SceneManipulation-v1 pd_ee_delta_pose
#   RECORD=1 ./teleop.sh                                 # also record to ./teleop_demos/
#
# ── Keyboard controls ───────────────────────────────────────────────
#
#  End-effector (EE delta)    Mobile base           Head / torso
#   i  = +X (forward)         w = drive forward     v = pan left
#   k  = -X (backward)        s = drive backward    b = pan right
#   j  = +Y (left)            a = rotate CCW        n = tilt down
#   l  = -Y (right)           d = rotate CW         m = tilt up
#   u  = +Z (up)              z = torso up
#   o  = -Z (down)            x = torso down
#
#  Gripper : f = open    g = close
#  Rotation: 1~6 = EE rotation axes (roll/pitch/yaw ±)
#
#  Misc    : r = reset env    0 = switch to SAPIEN 3D viewer
#            close matplotlib window / Ctrl+C to quit
# ───────────────────────────────────────────────────────────────────

set -euo pipefail

PKG_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ENV_ID="${1:-ReplicaCADTidyHouseTrain_SceneManipulation-v1}"
CONTROL_MODE="${2:-pd_ee_delta_pose}"
RECORD="${RECORD:-0}"
# Shader presets: minimal | default | rt-fast | rt-med | rt
SHADER="${SHADER:-rt-fast}"
CAM_W="${CAM_W:-640}"
CAM_H="${CAM_H:-480}"
# DUAL=1 : use teleop_dual.py (SAPIEN 3-D viewer + matplotlib sensor panel simultaneously)
# DUAL=0 : use upstream demo_manual_control (matplotlib only, press 0 for SAPIEN viewer)
DUAL="${DUAL:-1}"

# Use the venv Python directly — Cursor's terminal hijacks the shell PATH so
# `source activate` + `python` may still resolve to Cursor's bundled interpreter.
PYTHON="$PKG_ROOT/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "[teleop] ERROR: venv not found at $PKG_ROOT/.venv"
    echo "[teleop]   Run:  cd $PKG_ROOT && ./run.sh setup"
    exit 1
fi

if [[ "$DUAL" == "1" ]]; then
    # ── Dual-window mode: SAPIEN viewer + matplotlib, both live ──────────────
    RECORD_ARG=""
    if [[ "$RECORD" == "1" ]]; then
        RECORD_ARG="--record-dir ./teleop_demos"
    fi
    MANISKILL_ENV_ID="$ENV_ID" \
    MANISKILL_CONTROL_MODE="$CONTROL_MODE" \
    SHADER="$SHADER" CAM_W="$CAM_W" CAM_H="$CAM_H" \
    MS_SKIP_ASSET_DOWNLOAD_PROMPT=1 \
    "$PYTHON" -m maniskill_vla_demo.teleop_dual \
        "$ENV_ID" --control "$CONTROL_MODE" \
        --shader "$SHADER" --cam-w "$CAM_W" --cam-h "$CAM_H" \
        $RECORD_ARG
else
    # ── Single-window fallback: upstream demo_manual_control ─────────────────
    echo ""
    echo "══════════════════════════════════════════════════════"
    echo "  ManiSkill3 Keyboard Teleoperation (matplotlib only)"
    echo "  env    : $ENV_ID"
    echo "  shader : $SHADER  (cam ${CAM_W}x${CAM_H})"
    echo "══════════════════════════════════════════════════════"
    echo ""
    echo "  EE delta : i/k=X  j/l=Y  u/o=Z  1-6=rotation"
    echo "  Base     : w/s=fwd/back  a/d=rotate  z/x=torso"
    echo "  Gripper  : f=open  g=close"
    echo "  Reset    : r    Switch to SAPIEN: 0    Quit: close window"
    echo ""

    RECORD_ARG=""
    if [[ "$RECORD" == "1" ]]; then
        RECORD_ARG="--record-dir ./teleop_demos/{env_id}"
    fi
    SENSOR_CFG="@{\"shader_pack\": \"${SHADER}\", \"width\": ${CAM_W}, \"height\": ${CAM_H}}"
    MS_SKIP_ASSET_DOWNLOAD_PROMPT=1 \
    "$PYTHON" -m mani_skill.examples.demo_manual_control \
        -e "$ENV_ID" \
        -c "$CONTROL_MODE" \
        $RECORD_ARG \
        robot_uids fetch \
        sensor_configs "$SENSOR_CFG"
fi
