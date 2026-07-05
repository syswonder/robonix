#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

DEMO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REPO_ROOT="$(cd "$DEMO_ROOT/../.." && pwd)"
STATE_FILE="${HOME}/.robonix/processes.json"

python3 - "$DEMO_ROOT" "$REPO_ROOT" <<'PY'
import os
import signal
import sys
import time

demo_root = sys.argv[1]
repo_root = sys.argv[2]
needles = [
    f"rbnx boot",
    f"robonix-remote-liaison-demo",
    f"--manifest {demo_root}/robonix_manifest.yaml",
    "python3 -m audio_macos_bridge.main",
    "python -m audio_macos_bridge.main",
    "python3 -m speech_service.service",
    "python -m speech_service.service",
    "python3 -m voiceprint_service.service",
    "python -m voiceprint_service.service",
    "uv run --active python -m voiceprint_service.service",
    "python3 -m remote_demo_skill.service",
    "python -m remote_demo_skill.service",
    f"rbnx start -p {demo_root}",
    f"rbnx start -p {repo_root}/examples/webots/primitives/audio_macos_bridge",
    f"rbnx start -p {repo_root}/services/voiceprint",
    f"rbnx start -p {repo_root}/services/speech",
]
system_ports = {
    "robonix-atlas": "127.0.0.1:50051",
    "robonix-executor": "127.0.0.1:51161",
    "robonix-pilot": "127.0.0.1:51071",
    "robonix-liaison": "127.0.0.1:51081",
}

def matches():
    own = {os.getpid(), os.getppid()}
    found = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in own:
            continue
        try:
            raw = open(f"/proc/{pid}/cmdline", "rb").read()
        except OSError:
            continue
        cmd = raw.replace(b"\0", b" ").decode("utf-8", "ignore").strip()
        if cmd and any(needle in cmd for needle in needles):
            found.append((pid, cmd))
            continue
        if cmd and any(name in cmd and port in cmd for name, port in system_ports.items()):
            found.append((pid, cmd))
    return found

for sig in (signal.SIGTERM, signal.SIGKILL):
    targets = matches()
    if not targets:
        break
    for pid, _cmd in targets:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    time.sleep(0.5)
PY

if [[ -f "$STATE_FILE" ]]; then
  tmp="$(mktemp)"
  python3 - "$STATE_FILE" >"$tmp" <<'PY'
import json
import sys
from pathlib import Path

state_path = Path(sys.argv[1])
remove = {
    "com.robonix.example.audio_macos_bridge",
    "com.robonix.example.voiceprint_service",
    "com.robonix.example.speech_service",
    "com.robonix.demo.status_skill",
    "com.robonix.demo.notes_skill",
    "com.robonix.demo.summary_skill",
}

try:
    data = json.loads(state_path.read_text())
except Exception:
    data = []

kept = [
    item for item in data
    if item.get("package_name") not in remove and item.get("std_name") not in remove
]
print(json.dumps(kept, ensure_ascii=False, indent=2))
PY
  mv "$tmp" "$STATE_FILE"
fi

echo "[clean_demo_state] cleaned remote_liaison_demo stale package processes"
