# SPDX-License-Identifier: MulanPSL-2.0
"""Run the robonix CI capability + flow scenarios against a live deploy.

Assumes the stack is already booted (real Webots sim + real atlas/executor/
pilot/components) with pilot pointed at the fake VLM (``VLM_BASE_URL`` →
fake_vlm/server.py). Two scenario families live under ``scenarios/``:

  cap/   single-capability tests — "tool X is dispatched and succeeds".
  flow/  multi-step task flows, including **exception injection**: a step
         calls a capability with bad args so the leaf genuinely fails, the
         next scripted round recovers, and we assert the task still finishes.
         This exercises pilot's replan-on-failure path deterministically.

For each scenario this submits the task with ``rbnx ask <task> --json``, writes
the full PilotEvent stream to ``logs/<name>.jsonl`` (every run is recorded), and
asserts the scripted capabilities were dispatched with the expected args, the
expected leaves succeeded (or failed, for injected faults), the turn took at
least ``expect_min_rounds`` planning rounds, and the task reached ``done``.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

STATE_FAILED = 2  # PilotEvent status.state (ask.rs)
LOG_DIR = Path(__file__).resolve().parent / "logs"


def run_ask(rbnx: str, task: str, server: str, timeout: int, log_path: Path) -> tuple[list[dict], int]:
    """Submit one task via ``rbnx ask --json``; tee stdout to a log file.

    Returns (parsed_events, exit_code). rbnx exits non-zero iff pilot emitted a
    FAILED status. The raw stdout (one PilotEvent JSON per line) plus stderr are
    persisted to log_path so every CI run keeps a full trace.
    """
    cmd = [rbnx, "ask", task, "--json", "--server", server]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    log_path.write_text(
        f"$ {' '.join(cmd)}\n# exit={proc.returncode}\n# --- stdout ---\n"
        f"{proc.stdout}\n# --- stderr ---\n{proc.stderr}\n"
    )
    events = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return events, proc.returncode


def collect_calls(events: list[dict]) -> list[dict]:
    """Flatten every dispatched capability call across all EVT_PLAN events."""
    return [c for ev in events if ev.get("plan") for c in ev["plan"].get("calls", [])]


def collect_leaf_results(events: list[dict]) -> list[dict]:
    """Flatten every leaf_result across all EVT_BATCH_RESULT events."""
    out = []
    for ev in events:
        batch = ev.get("batch_result")
        if batch:
            out.extend(r["leaf_result"] for r in batch.get("results", []) if r.get("leaf_result"))
    return out


def count_rounds(events: list[dict]) -> int:
    """Number of planning rounds = number of EVT_PLAN events."""
    return sum(1 for ev in events if ev.get("plan"))


def final_failed(events: list[dict]) -> bool:
    """True if pilot emitted a FAILED status anywhere in the stream."""
    return any((ev.get("status") or {}).get("state") == STATE_FAILED for ev in events)


def check_scenario(scenario: dict, events: list[dict], exit_code: int) -> list[str]:
    """Return a list of failure strings ([] means the scenario passed).

    Beyond the cap-level checks, flow scenarios may declare:
      expect_min_rounds   — at least this many planning rounds (multi-step).
      expect_leaf_failure — contract substrings whose leaf is EXPECTED to fail
                            (the injected fault); such a failure is not counted
                            against the run, and its absence IS a failure.
      expect_final_done   — require the turn to end without a FAILED status
                            (default true; the whole point of fault recovery).
    """
    fails: list[str] = []
    calls = collect_calls(events)
    contracts = [c.get("contract_id", "") for c in calls]
    args_blob = " ".join(c.get("args_json", "") for c in calls)
    leaves = collect_leaf_results(events)
    expect_fail = scenario.get("expect_leaf_failure", [])

    if scenario.get("expect_final_done", True) and final_failed(events):
        fails.append("pilot reported FAILED but expect_final_done")
    if exit_code != 0 and not expect_fail:
        fails.append(f"rbnx ask exit={exit_code}")

    for want in scenario.get("expect_contracts", []):
        if not any(want in c for c in contracts):
            fails.append(f"expected contract ~{want!r}, dispatched: {contracts}")
    for want in scenario.get("expect_args", []):
        if want not in args_blob:
            fails.append(f"expected arg ~{want!r} in: {args_blob}")

    rounds = count_rounds(events)
    if rounds < scenario.get("expect_min_rounds", 0):
        fails.append(f"expected >= {scenario['expect_min_rounds']} rounds, got {rounds}")

    # Injected faults: each expect_leaf_failure matcher must have a failed leaf.
    for want in expect_fail:
        if not any((not lr.get("success", True)) and want in lr.get("contract_id", "") for lr in leaves):
            fails.append(f"expected an INJECTED failure on ~{want!r}, none observed")

    # Unexpected failures: a failed leaf whose contract is not an expected fault.
    if not scenario.get("allow_leaf_failure", False):
        for lr in leaves:
            cid = lr.get("contract_id", "")
            if not lr.get("success", False) and not any(w in cid for w in expect_fail):
                fails.append(f"unexpected leaf failure {cid}: {lr.get('error', '')[:160]}")
    return fails


def main() -> int:
    ap = argparse.ArgumentParser(description="robonix CI scenario runner")
    ap.add_argument("--rbnx", default=os.environ.get("RBNX_BIN", "rbnx"))
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    ap.add_argument("--scenarios", type=Path, default=Path(__file__).resolve().parent / "scenarios")
    ap.add_argument("--timeout", type=int, default=300, help="per-scenario seconds")
    ap.add_argument("--only", help="run only the scenario with this name")
    ap.add_argument("--summary-json", type=Path, help="write a machine-readable result summary here")
    args = ap.parse_args()

    paths = sorted([*args.scenarios.rglob("*.yaml"), *args.scenarios.rglob("*.yml")])
    if args.only:
        paths = [p for p in paths if p.stem == args.only]
    if not paths:
        print("no scenarios found", file=sys.stderr)
        return 2

    LOG_DIR.mkdir(exist_ok=True)
    all_contracts: set[str] = set()
    failed: list[str] = []
    results: list[dict] = []  # per-scenario records for --summary-json

    for path in paths:
        scenario = yaml.safe_load(path.read_text())
        scenario.setdefault("name", path.stem)
        family = path.parent.name
        name = scenario["name"]
        print(f"\n=== {family}/{name} ===\n  task: {scenario['task']!r}")
        log_path = LOG_DIR / f"{family}.{name}.jsonl"
        try:
            events, code = run_ask(args.rbnx, scenario["task"], args.server, args.timeout, log_path)
        except subprocess.TimeoutExpired:
            print(f"  TIMEOUT after {args.timeout}s (log: {log_path.name})")
            failed.append(name)
            continue

        for c in collect_calls(events):
            all_contracts.add(c.get("contract_id", ""))

        errs = check_scenario(scenario, events, code)
        rounds = count_rounds(events)
        dispatched = sorted({c.get("contract_id", "") for c in collect_calls(events)})
        results.append({
            "name": name, "family": family, "passed": not errs, "rounds": rounds,
            "dispatched": dispatched, "failures": errs, "log": log_path.name,
        })
        if errs:
            print(f"  FAIL [{rounds} rounds] (log: {log_path.name}):")
            for e in errs:
                print(f"    - {e}")
            failed.append(name)
        else:
            print(f"  PASS [{rounds} rounds] dispatched: {dispatched}")

    print("\n=== coverage (capabilities exercised) ===")
    for c in sorted(all_contracts):
        print(f"  {c}")

    passed = len(paths) - len(failed)
    if args.summary_json:
        args.summary_json.write_text(json.dumps({
            "suite": "scenarios",
            "total": len(paths),
            "passed": passed,
            "failed": len(failed),
            "rate": round(passed / len(paths), 3) if paths else 0,
            "coverage": sorted(all_contracts),
            "log_dir": str(LOG_DIR),
            "scenarios": results,
        }, indent=2))

    print(f"\n=== logs in {LOG_DIR} ===")
    if failed:
        print(f"RESULT: FAIL — {len(failed)}/{len(paths)}: {failed}")
        return 1
    print(f"RESULT: PASS — {len(paths)}/{len(paths)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
