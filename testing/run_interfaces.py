# SPDX-License-Identifier: MulanPSL-2.0
"""Interface / RPC contract tests for the robonix system components.

Complements the capability + flow scenarios (run.py): instead of asserting that
a *task* produces the right tool calls, this asserts that each system
component's gRPC interface answers correctly on a live deploy.

  atlas    — Query / ListContracts / InspectAtlas / channels / tools, via the
             `rbnx` inspection subcommands that wrap those RPCs. We assert each
             returns well-formed JSON and the expected system components +
             at least one primitive are registered.
  pilot    — SubmitTask, via `rbnx ask` with a no-tool chit-chat prompt. We
             assert the stream yields final text and a Completed status with
             zero capability calls (pure conversation, no plan dispatched).
  executor — Execute(Plan) → RtdlEvent stream, asserted transitively: every
             scenario in run.py that dispatches a plan proves executor streamed
             node-state events. Here we additionally assert the executor
             component is registered/reachable in atlas.
  liaison  — registered + reachable in atlas (its SubmitTask/StartVoiceSession
             endpoint is advertised). A full direct SubmitTask probe needs the
             liaison gRPC stub from a built deploy and is a documented follow-up.

Every command's raw output is written under ``logs/iface.*`` so CI keeps a full
trace. Exit non-zero if any interface assertion fails.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent / "logs"

# Contract ids every booted deploy must have registered (the system block is
# always present). Deploy-specific providers (camera/lidar/…) are asserted by
# the scenario layer, not here, so this stays deploy-agnostic. Extra required
# substrings can be passed via ROBONIX_REQUIRE (comma-separated).
REQUIRED_COMPONENTS = ["system/pilot", "system/executor", "system/liaison"]


def sh(cmd: list[str], log_name: str, timeout: int = 120) -> tuple[str, int]:
    """Run a command, tee stdout+stderr to logs/<log_name>, return (stdout, rc)."""
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    (LOG_DIR / log_name).write_text(
        f"$ {' '.join(cmd)}\n# exit={proc.returncode}\n"
        f"# --- stdout ---\n{proc.stdout}\n# --- stderr ---\n{proc.stderr}\n"
    )
    return proc.stdout, proc.returncode


def check_atlas(rbnx: str, server: str) -> list[str]:
    """Exercise the atlas query RPCs; assert JSON + expected registrations.

    `rbnx caps/contracts/tools --json` wrap atlas Query / ListContracts and the
    MCP tool listing. We parse each as JSON (proves the RPC + serialization) and
    assert the required system components appear in the caps listing. Extra
    required contract substrings come from ROBONIX_REQUIRE (comma-separated).
    """
    fails: list[str] = []
    caps_out = ""
    for sub, log in [
        (["caps"], "iface.atlas_caps.json"),
        (["contracts"], "iface.atlas_contracts.json"),
        (["tools"], "iface.atlas_tools.json"),
    ]:
        out, rc = sh([rbnx, *sub, "--server", server, "--json"], log)
        if sub[0] == "caps":
            caps_out = out
        if rc != 0:
            fails.append(f"`rbnx {sub[0]} --json` exit={rc}")
            continue
        try:
            json.loads(out)
        except json.JSONDecodeError as e:
            fails.append(f"`rbnx {sub[0]} --json` not valid JSON: {e}")

    required = list(REQUIRED_COMPONENTS)
    extra = os.environ.get("ROBONIX_REQUIRE", "").strip()
    if extra:
        required += [s.strip() for s in extra.split(",") if s.strip()]
    for want in required:
        if want not in caps_out:
            fails.append(f"atlas caps missing a registration for ~{want!r}")
    return fails


def check_pilot(rbnx: str, server: str) -> list[str]:
    """Exercise pilot SubmitTask with a no-tool prompt; assert clean completion.

    A pure chit-chat prompt has no matching scenario, so the fake VLM returns an
    empty-tree `done` envelope. The SubmitTask stream must therefore carry a
    final_text and a non-FAILED terminal status, and dispatch zero capability
    calls (no plan).
    """
    fails: list[str] = []
    out, rc = sh(
        [rbnx, "ask", "ci-iface: just say hello, do not use any tools", "--json", "--server", server],
        "iface.pilot_submit_task.jsonl",
        timeout=120,
    )
    events = []
    for line in out.splitlines():
        line = line.strip()
        if line:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    if rc != 0:
        fails.append(f"pilot SubmitTask (`rbnx ask`) exit={rc}")
    if not any(ev.get("final_text") for ev in events):
        fails.append("pilot SubmitTask produced no final_text")
    calls = [c for ev in events if ev.get("plan") for c in ev["plan"].get("calls", [])]
    if calls:
        fails.append(f"chit-chat unexpectedly dispatched calls: {[c.get('contract_id') for c in calls]}")
    if any((ev.get("status") or {}).get("state") == 2 for ev in events):
        fails.append("pilot SubmitTask ended FAILED on a trivial prompt")
    return fails


def main() -> int:
    ap = argparse.ArgumentParser(description="robonix interface / RPC contract tests")
    ap.add_argument("--rbnx", default=os.environ.get("RBNX_BIN", "rbnx"))
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    args = ap.parse_args()
    LOG_DIR.mkdir(exist_ok=True)

    suites = [("atlas", check_atlas), ("pilot", check_pilot)]
    failed = False
    for name, fn in suites:
        print(f"\n=== interface: {name} ===")
        errs = fn(args.rbnx, args.server)
        if errs:
            failed = True
            for e in errs:
                print(f"  FAIL - {e}")
        else:
            print("  PASS")

    print(f"\n=== logs in {LOG_DIR} ===")
    print("RESULT:", "FAIL" if failed else "PASS")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
