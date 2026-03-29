#!/usr/bin/env bash
# Force-stop all example-related processes.

set -euo pipefail

echo "[stop] terminating example processes..."

# Graceful stop first.
pkill -TERM -f rerun 2>/dev/null || true
pkill -TERM -f ros2 2>/dev/null || true
pkill -TERM -f rclcpp 2>/dev/null || true
pkill -TERM -f robonix 2>/dev/null || true
pkill -TERM -f rbnx 2>/dev/null || true
pkill -TERM -f cargo 2>/dev/null || true
pkill -TERM -f python 2>/dev/null || true

sleep 1

# Force kill stragglers.
pkill -KILL -f rerun 2>/dev/null || true
pkill -KILL -f ros2 2>/dev/null || true
pkill -KILL -f rclcpp 2>/dev/null || true
pkill -KILL -f robonix 2>/dev/null || true
pkill -KILL -f rbnx 2>/dev/null || true
pkill -KILL -f cargo 2>/dev/null || true
pkill -KILL -f python 2>/dev/null || true

echo "[stop] done."

free -h