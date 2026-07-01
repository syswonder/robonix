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


def parse_args_json(call: dict) -> object:
    try:
        return json.loads(call.get("args_json", "{}"))
    except json.JSONDecodeError:
        return {}


def parse_leaf_output(leaf: dict) -> object:
    output = leaf.get("output", "")
    if isinstance(output, (dict, list)):
        return output
    if not isinstance(output, str):
        return output
    text = output.strip()
    if not text.startswith(("{", "[")):
        return output
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return output


def contains_subset(actual: object, expected: object) -> bool:
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            return False
        return all(k in actual and contains_subset(actual[k], v) for k, v in expected.items())
    if isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) < len(expected):
            return False
        return all(contains_subset(a, e) for a, e in zip(actual, expected))
    return actual == expected


def json_path_values(value: object, path: str) -> list[object]:
    if not path.startswith("$."):
        return []
    current = [value]
    for token in path[2:].split("."):
        next_values: list[object] = []
        is_array = token.endswith("[]")
        key = token[:-2] if is_array else token
        for item in current:
            if not isinstance(item, dict) or key not in item:
                continue
            child = item[key]
            if is_array:
                if isinstance(child, list):
                    next_values.extend(child)
            else:
                next_values.append(child)
        current = next_values
    return current


def check_output_clause(leaf: dict, clause: dict) -> list[str]:
    errs: list[str] = []
    raw = str(leaf.get("output", ""))
    err_text = str(leaf.get("error", ""))
    parsed = parse_leaf_output(leaf)
    if "text_equals" in clause and raw != clause["text_equals"]:
        errs.append(f"output text expected exactly {clause['text_equals']!r}, got {raw!r}")
    if "text_regex" in clause and not __import__("re").search(str(clause["text_regex"]), raw):
        errs.append(f"output text did not match regex {clause['text_regex']!r}: {raw!r}")
    if "error_regex" in clause and not __import__("re").search(str(clause["error_regex"]), err_text):
        errs.append(f"error text did not match regex {clause['error_regex']!r}: {err_text!r}")
    for line in clause.get("text_lines", []):
        if line not in raw.splitlines():
            errs.append(f"output missing exact line {line!r}")
    if "json" in clause and not contains_subset(parsed, clause["json"]):
        errs.append(f"output JSON did not contain subset {clause['json']!r}, got {parsed!r}")
    for cond in clause.get("jsonpath", []):
        values = json_path_values(parsed, str(cond.get("path", "")))
        if cond.get("exists") and not values:
            errs.append(f"jsonpath {cond.get('path')!r} did not exist")
            continue
        if "min_length" in cond:
            if len(values) != 1 or not hasattr(values[0], "__len__") or len(values[0]) < int(cond["min_length"]):
                errs.append(f"jsonpath {cond.get('path')!r} length < {cond['min_length']}: {values!r}")
        if "equals" in cond and cond["equals"] not in values:
            errs.append(f"jsonpath {cond.get('path')!r} missing exact value {cond['equals']!r}: {values!r}")
        if "prefix" in cond and not any(str(v).startswith(str(cond["prefix"])) for v in values):
            errs.append(f"jsonpath {cond.get('path')!r} missing prefix {cond['prefix']!r}: {values!r}")
    return errs


def validate_new_assertion_shape(scenario: dict) -> list[str]:
    errs: list[str] = []
    old_fields = tuple(f"expect_{suffix}" for suffix in ("contracts", "args", "outputs", "leaf_failure"))
    for field in old_fields:
        if field in scenario:
            errs.append(f"{field} is removed; use expect_leaves entries with contract/success/args/output")
    for idx, want in enumerate(scenario.get("expect_leaves", [])):
        contract = want.get("contract")
        if not isinstance(contract, str) or not contract.startswith("robonix/"):
            errs.append(f"expect_leaves[{idx}].contract must be a full robonix/... contract id")
    return errs


def check_expected_leaf(want: dict, calls: list[dict], leaves: list[dict]) -> str | None:
    contract = want["contract"]
    expected_success = want.get("success", True)
    candidates = [lr for lr in leaves if lr.get("contract_id") == contract and lr.get("success", False) == expected_success]
    if not candidates:
        return f"expected leaf contract={contract!r} success={expected_success}, observed {[(lr.get('contract_id'), lr.get('success')) for lr in leaves]}"
    call_args_by_id = {c.get("call_id"): parse_args_json(c) for c in calls}
    reasons: list[str] = []
    for leaf in candidates:
        leaf_reasons: list[str] = []
        if "args" in want:
            args = call_args_by_id.get(leaf.get("call_id"), {})
            if not contains_subset(args, want["args"]):
                leaf_reasons.append(f"args did not contain subset {want['args']!r}, got {args!r}")
        output_clause = want.get("output")
        if isinstance(output_clause, dict):
            leaf_reasons.extend(check_output_clause(leaf, output_clause))
        if not leaf_reasons:
            return None
        reasons.extend(leaf_reasons)
    return f"expected leaf {contract!r} did not satisfy assertions: {'; '.join(reasons)}"


def count_rounds(events: list[dict]) -> int:
    """Number of planning rounds = number of EVT_PLAN events."""
    return sum(1 for ev in events if ev.get("plan"))


def final_failed(events: list[dict]) -> bool:
    """True if pilot emitted a FAILED status anywhere in the stream."""
    return any((ev.get("status") or {}).get("state") == STATE_FAILED for ev in events)


def check_scenario(scenario: dict, events: list[dict], exit_code: int) -> list[str]:
    """Return a list of failure strings ([] means the scenario passed).

    Flow scenarios may declare:
      expect_min_rounds   — at least this many planning rounds (multi-step).
      expect_leaves       — exact leaf contracts with success/args/output checks.
      expect_final_done   — require the turn to end without a FAILED status
                            (default true; the whole point of fault recovery).
    """
    fails: list[str] = []
    fails.extend(validate_new_assertion_shape(scenario))
    calls = collect_calls(events)
    leaves = collect_leaf_results(events)

    if scenario.get("expect_final_done", True) and final_failed(events):
        fails.append("pilot reported FAILED but expect_final_done")
    if exit_code != 0:
        fails.append(f"rbnx ask exit={exit_code}")

    for want in scenario.get("expect_leaves", []):
        if not isinstance(want, dict) or "contract" not in want:
            fails.append(f"invalid expect_leaves entry: {want!r}")
            continue
        err = check_expected_leaf(want, calls, leaves)
        if err:
            fails.append(err)

    rounds = count_rounds(events)
    if rounds < scenario.get("expect_min_rounds", 0):
        fails.append(f"expected >= {scenario['expect_min_rounds']} rounds, got {rounds}")

    # Unexpected failures: a failed leaf whose contract is not an expected fault.
    if not scenario.get("allow_leaf_failure", False):
        expected_failures = {
            want.get("contract")
            for want in scenario.get("expect_leaves", [])
            if isinstance(want, dict) and want.get("success") is False
        }
        for lr in leaves:
            cid = lr.get("contract_id", "")
            if not lr.get("success", False) and cid not in expected_failures:
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
        except subprocess.TimeoutExpired as exc:
            print(f"  TIMEOUT after {args.timeout}s (log: {log_path.name})")
            stdout = exc.stdout or ""
            stderr = exc.stderr or ""
            if isinstance(stdout, bytes):
                stdout = stdout.decode("utf-8", errors="replace")
            if isinstance(stderr, bytes):
                stderr = stderr.decode("utf-8", errors="replace")
            log_path.write_text(
                f"$ {args.rbnx} ask {scenario['task']} --json --server {args.server}\n"
                f"# timeout={args.timeout}s\n# --- stdout ---\n{stdout}\n"
                f"# --- stderr ---\n{stderr}\n"
            )
            results.append({
                "name": name,
                "family": family,
                "passed": False,
                "rounds": 0,
                "dispatched": [],
                "failures": [f"timeout after {args.timeout}s"],
                "log": log_path.name,
            })
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
