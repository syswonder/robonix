#!/usr/bin/env bash
set -euo pipefail

echo "=== show .gitmodules ==="
sed -n '1,200p' .gitmodules || true
echo

echo "=== parsed submodule urls ==="
git config -f .gitmodules --get-regexp '^submodule\..*\.url$' || true
echo

echo "=== gitlinks in tree ==="
git ls-tree -r HEAD | awk '$1=="160000" {print $4}' || true
echo

echo "=== try fix .gitmodules to forks urls (idempotent) ==="
git config -f .gitmodules submodule.robonix/driver/realsense_ros2/src/realsense-ros.url "https://github.com/enkerewpo/realsense-ros.git"
git config -f .gitmodules submodule.robonix/driver/mid360/src/livox_ros_driver2.url "https://github.com/enkerewpo/livox_ros_driver2.git"
git config -f .gitmodules submodule.robonix/driver/ranger_ros2/src/ranger_ros2.url "https://github.com/enkerewpo/ranger_ros2"
git config -f .gitmodules submodule.robonix/driver/ranger_ros2/src/ugv_sdk.url "https://github.com/enkerewpo/ugv_sdk.git"

git add .gitmodules
if ! git diff --cached --quiet; then
  git commit -m "fix: .gitmodules -> forks" || true
  # git push origin HEAD    # if you want to auto push, uncomment this
fi

echo "=== sync and update submodules ==="
git submodule sync --recursive
git submodule update --init --recursive --force

echo "=== final submodule status ==="
git submodule status --recursive || true