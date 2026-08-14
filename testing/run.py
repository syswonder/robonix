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
asserts each RTDL ``do`` node produced its declared executor leaf and the task
reached ``done``. Assertions live on the RTDL leaf that planned the call, so the
scenario file mirrors what the fake VLM actually returns.
"""

from __future__ import annotations

import argparse
import copy
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


def collect_plan_rounds(events: list[dict]) -> list[list[dict]]:
    """Return dispatched capability calls grouped by EVT_PLAN order."""
    return [ev["plan"].get("calls", []) for ev in events if ev.get("plan")]


def collect_calls(events: list[dict]) -> list[dict]:
    """Flatten every dispatched capability call across all EVT_PLAN events."""
    return [c for calls in collect_plan_rounds(events) for c in calls]


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


def select_json_values(value: object, path: str) -> list[object]:
    if path == "$":
        return [value]
    if not path.startswith("$."):
        return []
    return json_path_values(value, path)


def _values_for_check(context: object, path: str | None) -> list[object]:
    return select_json_values(context, path or "$")


def _check_scalar(value: object, op: str, expected: object = None) -> bool:
    if op == "exists":
        return value is not None
    if op == "eq":
        return value == expected
    if op == "ne":
        return value != expected
    if op == "in":
        return value in (expected if isinstance(expected, list) else [expected])
    if op == "not_in":
        return value not in (expected if isinstance(expected, list) else [expected])
    if op == "starts_with":
        return str(value).startswith(str(expected))
    if op == "contains":
        return str(expected) in str(value)
    if op == "regex":
        return __import__("re").search(str(expected), str(value)) is not None
    if op in {"gt", "gte", "lt", "lte"}:
        try:
            lhs = float(value)
            rhs = float(expected)
        except (TypeError, ValueError):
            return False
        return {"gt": lhs > rhs, "gte": lhs >= rhs, "lt": lhs < rhs, "lte": lhs <= rhs}[op]
    if op == "min_length":
        return hasattr(value, "__len__") and len(value) >= int(expected)
    return False


def _check_values(values: list[object], check: dict) -> bool:
    op = str(check.get("op", "exists"))
    expected = check.get("value")
    if op == "exists":
        return bool(values)
    return any(_check_scalar(value, op, expected) for value in values)


def check_expr(context: object, expr: object) -> bool:
    if not isinstance(expr, dict):
        return False
    if "all" in expr:
        items = expr.get("all") or []
        return isinstance(items, list) and all(check_expr(context, item) for item in items)
    if "any" in expr:
        items = expr.get("any") or []
        return isinstance(items, list) and any(check_expr(context, item) for item in items)
    if "not" in expr:
        return not check_expr(context, expr.get("not"))
    return _check_values(_values_for_check(context, expr.get("path", "$")), expr)


def filter_selected_values(values: list[object], where: object) -> list[object]:
    if where is None:
        return values
    return [value for value in values if check_expr(value, where)]


def check_structured_clause(parsed: object, clause: dict) -> list[str]:
    errs: list[str] = []
    for idx, check in enumerate(clause.get("checks", []) or []):
        if not isinstance(check, dict):
            errs.append(f"checks[{idx}] must be a mapping")
            continue
        selected = select_json_values(parsed, str(check.get("select", "$")))
        selected = filter_selected_values(selected, check.get("where"))
        assertion = check.get("assert", {"op": "exists"})
        if not isinstance(assertion, dict):
            errs.append(f"checks[{idx}].assert must be a mapping")
            continue
        if not _check_values(selected, assertion):
            errs.append(f"checks[{idx}] failed: select={check.get('select', '$')!r} where={check.get('where')!r} assert={assertion!r} values={selected!r}")
    return errs


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
    errs.extend(check_structured_clause(parsed, clause))
    return errs


def iter_rtdl_do_nodes(node: object, path: str = "rtdl"):
    if not isinstance(node, dict):
        return
    if node.get("op") == "do":
        yield path, node
    for idx, child in enumerate(node.get("children", []) or []):
        yield from iter_rtdl_do_nodes(child, f"{path}.children[{idx}]")


def iter_expected_leaves(scenario: dict):
    for step_idx, step in enumerate(scenario.get("steps", [])):
        for node_path, node in iter_rtdl_do_nodes(step.get("rtdl")):
            expect = node.get("expect")
            if expect is not None:
                yield step_idx, node_path, node, expect


REMOVED_FILTER_KEYS = {"label_not", "label_in", "id_prefix", "id_contains"}
CHECK_OPS = {"exists", "eq", "ne", "in", "not_in", "starts_with", "contains", "regex", "gt", "gte", "lt", "lte", "min_length"}


def validate_check_expr(expr: object, path: str) -> list[str]:
    errs: list[str] = []
    if not isinstance(expr, dict):
        return [f"{path} must be a mapping"]
    removed = sorted(REMOVED_FILTER_KEYS & set(expr))
    if removed:
        errs.append(f"{path} uses removed field-specific filter(s) {removed}; use path/op/value predicates")
    logical = [key for key in ("all", "any", "not") if key in expr]
    if logical:
        if len(logical) != 1:
            errs.append(f"{path} must use only one logical operator")
            return errs
        key = logical[0]
        if key in {"all", "any"}:
            items = expr.get(key)
            if not isinstance(items, list) or not items:
                errs.append(f"{path}.{key} must be a non-empty list")
            else:
                for idx, item in enumerate(items):
                    errs.extend(validate_check_expr(item, f"{path}.{key}[{idx}]"))
        else:
            errs.extend(validate_check_expr(expr.get("not"), f"{path}.not"))
        return errs
    op = str(expr.get("op", ""))
    if not op:
        errs.append(f"{path}.op is required")
    elif op not in CHECK_OPS:
        errs.append(f"{path}.op {op!r} is not supported")
    check_path = expr.get("path", "$")
    if not isinstance(check_path, str) or not check_path.startswith("$"):
        errs.append(f"{path}.path must be a JSON selector starting with $")
    return errs


def validate_output_assertions(output: object, path: str) -> list[str]:
    errs: list[str] = []
    if not isinstance(output, dict):
        return errs
    if "jsonpath" in output:
        errs.append(f"{path}.jsonpath is removed; use {path}.checks")
    checks = output.get("checks")
    if checks is None:
        return errs
    if not isinstance(checks, list) or not checks:
        return [f"{path}.checks must be a non-empty list"]
    for idx, check in enumerate(checks):
        cpath = f"{path}.checks[{idx}]"
        if not isinstance(check, dict):
            errs.append(f"{cpath} must be a mapping")
            continue
        select = check.get("select", "$")
        if not isinstance(select, str) or not select.startswith("$"):
            errs.append(f"{cpath}.select must be a JSON selector starting with $")
        if "where" in check:
            errs.extend(validate_check_expr(check.get("where"), f"{cpath}.where"))
        assertion = check.get("assert", {"op": "exists"})
        errs.extend(validate_check_expr(assertion, f"{cpath}.assert"))
    return errs


def validate_capture_assertions(captures: object, path: str) -> list[str]:
    errs: list[str] = []
    if captures is None:
        return errs
    if not isinstance(captures, dict):
        return [f"{path} must be a mapping"]
    for name, spec in captures.items():
        cpath = f"{path}.{name}"
        if not isinstance(spec, dict):
            errs.append(f"{cpath} must be a mapping")
            continue
        removed = sorted(REMOVED_FILTER_KEYS & set(spec))
        if removed:
            errs.append(f"{cpath} uses removed field-specific filter(s) {removed}; use where path/op/value predicates")
        if "jsonpath" in spec or "field" in spec:
            errs.append(f"{cpath} jsonpath/field selector is removed; use select + path")
        if "select" in spec:
            select = spec.get("select")
            if not isinstance(select, str) or not select.startswith("$"):
                errs.append(f"{cpath}.select must be a JSON selector starting with $")
            extract_path = spec.get("path", "$")
            if not isinstance(extract_path, str) or not extract_path.startswith("$"):
                errs.append(f"{cpath}.path must be a JSON selector starting with $")
            if "where" in spec:
                errs.extend(validate_check_expr(spec.get("where"), f"{cpath}.where"))
        elif "regex" not in spec:
            errs.append(f"{cpath} must define either select/path or regex")
    return errs


def validate_new_assertion_shape(scenario: dict) -> list[str]:
    errs: list[str] = []
    for field in scenario:
        if field.startswith("expect_") and field != "expect_final_done":
            errs.append(f"{field} is removed; put checks on RTDL do nodes as steps[].rtdl...expect")
    for step_idx, step in enumerate(scenario.get("steps", [])):
        if "caps" in step:
            errs.append(f"steps[{step_idx}].caps is removed; use steps[{step_idx}].rtdl")
        if "expect" in step:
            errs.append(f"steps[{step_idx}].expect is removed; put checks on RTDL do nodes")
        rtdl = step.get("rtdl")
        if rtdl is None:
            if step.get("status") != "done":
                errs.append(f"steps[{step_idx}].rtdl is required unless the step only marks done")
            continue
        if not isinstance(rtdl, dict):
            errs.append(f"steps[{step_idx}].rtdl must be a mapping")
            continue
        for node_path, node in iter_rtdl_do_nodes(rtdl):
            expect = node.get("expect")
            if not isinstance(expect, dict):
                errs.append(f"steps[{step_idx}].{node_path}.expect is required on every RTDL do node")
                continue
            contract = expect.get("contract")
            if not isinstance(contract, str) or not contract.startswith("robonix/"):
                errs.append(f"steps[{step_idx}].{node_path}.expect.contract must be a full robonix/... contract id")
            errs.extend(validate_output_assertions(expect.get("output"), f"steps[{step_idx}].{node_path}.expect.output"))
            errs.extend(validate_capture_assertions(expect.get("capture"), f"steps[{step_idx}].{node_path}.expect.capture"))
    return errs


def leaf_satisfies_expected(want: dict, calls: list[dict], leaf: dict) -> list[str]:
    reasons: list[str] = []
    if leaf.get("contract_id") != want.get("contract"):
        reasons.append(f"contract {leaf.get('contract_id')!r} != {want.get('contract')!r}")
    if leaf.get("success", False) != want.get("success", True):
        reasons.append(f"success {leaf.get('success', False)!r} != {want.get('success', True)!r}")
    call_args_by_id = {c.get("call_id"): parse_args_json(c) for c in calls}
    if "args" in want:
        args = call_args_by_id.get(leaf.get("call_id"), {})
        if not contains_subset(args, want["args"]):
            reasons.append(f"args did not contain subset {want['args']!r}, got {args!r}")
    output_clause = want.get("output")
    if isinstance(output_clause, dict):
        reasons.extend(check_output_clause(leaf, output_clause))
    return reasons


def find_expected_leaf_index(want: dict, calls: list[dict], leaves: list[dict], used: set[int]) -> tuple[int | None, str | None]:
    contract = want["contract"]
    expected_success = want.get("success", True)
    candidates = [
        (idx, lr)
        for idx, lr in enumerate(leaves)
        if idx not in used
        and lr.get("contract_id") == contract
        and lr.get("success", False) == expected_success
    ]
    if not candidates:
        observed = [(lr.get("contract_id"), lr.get("success")) for lr in leaves]
        return None, f"expected leaf contract={contract!r} success={expected_success}, observed {observed}"
    reasons: list[str] = []
    for idx, leaf in candidates:
        leaf_reasons = leaf_satisfies_expected(want, calls, leaf)
        if not leaf_reasons:
            return idx, None
        reasons.extend(leaf_reasons)
    return None, f"expected leaf {contract!r} did not satisfy assertions: {'; '.join(reasons)}"


def count_rounds(events: list[dict]) -> int:
    """Number of planning rounds = number of EVT_PLAN events."""
    return sum(1 for ev in events if ev.get("plan"))


def summarize_rtdl_steps(scenario: dict) -> list[dict]:
    """Return the scenario's planned RTDL trees for reports.

    A scenario step is one VLM planning round. Keeping that tree in the summary
    lets the HTML report show the actual sequence/parallel/do structure instead
    of only a flattened contract list.
    """
    out: list[dict] = []
    for idx, step in enumerate(scenario.get("steps", [])):
        rtdl = step.get("rtdl")
        if rtdl is None:
            continue
        out.append(
            {
                "index": idx,
                "status": step.get("status", ""),
                "description": step.get("rtdl_description", ""),
                "content": step.get("content", ""),
                "rtdl": copy.deepcopy(rtdl),
            }
        )
    return out


def _short_text(value: object, limit: int = 240) -> str:
    text = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False)
    text = text.replace("\r", "")
    return text if len(text) <= limit else text[: limit - 1] + "…"


def summarize_plan_rounds(events: list[dict]) -> list[dict]:
    """Return executed plan rounds with leaf result excerpts for the report."""
    leaves_by_call_id = {lr.get("call_id"): lr for lr in collect_leaf_results(events)}
    rounds: list[dict] = []
    for idx, calls in enumerate(collect_plan_rounds(events)):
        round_calls = []
        for call in calls:
            call_id = call.get("call_id", "")
            leaf = leaves_by_call_id.get(call_id, {})
            round_calls.append(
                {
                    "call_id": call_id,
                    "contract": call.get("contract_id", ""),
                    "provider": call.get("provider_id", ""),
                    "args": parse_args_json(call),
                    "leaf_result": {
                        "success": bool(leaf.get("success", False)) if leaf else None,
                        "output": _short_text(leaf.get("output", "")) if leaf else "",
                        "error": _short_text(leaf.get("error", "")) if leaf else "",
                    },
                }
            )
        rounds.append({"index": idx, "calls": round_calls})
    return rounds


def final_failed(events: list[dict]) -> bool:
    """True if pilot emitted a FAILED status anywhere in the stream."""
    return any((ev.get("status") or {}).get("state") == STATE_FAILED for ev in events)


def check_scenario(scenario: dict, events: list[dict], exit_code: int) -> list[str]:
    """Return a list of failure strings ([] means the scenario passed).

    Leaf assertions live on RTDL ``do`` nodes. The runner scopes each assertion
    to that planning round's call_ids, so a later or unrelated leaf cannot
    satisfy an earlier node by accident.
    """
    fails: list[str] = []
    fails.extend(validate_new_assertion_shape(scenario))
    plan_rounds = collect_plan_rounds(events)
    all_calls = [c for calls in plan_rounds for c in calls]
    leaves = collect_leaf_results(events)

    if scenario.get("expect_final_done", True) and final_failed(events):
        fails.append("pilot reported FAILED but expect_final_done")
    if exit_code != 0:
        fails.append(f"rbnx ask exit={exit_code}")

    expected_by_step: dict[int, list[tuple[str, dict, dict]]] = {}
    expected_failure_specs: list[tuple[int, dict]] = []
    for step_idx, node_path, node, want in iter_expected_leaves(scenario):
        expected_by_step.setdefault(step_idx, []).append((node_path, node, want))
        if isinstance(want, dict) and want.get("success") is False:
            expected_failure_specs.append((step_idx, want))

    cursor = 0
    for step_idx, expected_nodes in sorted(expected_by_step.items()):
        invalid = [
            (node_path, want)
            for node_path, _node, want in expected_nodes
            if not isinstance(want, dict) or "contract" not in want
        ]
        for node_path, want in invalid:
            fails.append(f"invalid steps[{step_idx}].{node_path}.expect entry: {want!r}")
        if invalid:
            continue

        matched_round = None
        observed: list[str] = []
        for round_idx in range(cursor, len(plan_rounds)):
            step_calls = plan_rounds[round_idx]
            step_call_ids = {c.get("call_id") for c in step_calls}
            step_leaves = [lr for lr in leaves if lr.get("call_id") in step_call_ids]
            round_errs: list[str] = []
            if len(step_calls) != len(expected_nodes):
                round_errs.append(f"planned {len(step_calls)} call(s), expected {len(expected_nodes)}")
            if len(step_leaves) != len(step_calls):
                round_errs.append(f"produced {len(step_leaves)} leaf result(s) for {len(step_calls)} call(s)")
            used_leaf_indexes: set[int] = set()
            for node_path, _node, want in expected_nodes:
                match_idx, err = find_expected_leaf_index(want, step_calls, step_leaves, used_leaf_indexes)
                if err:
                    round_errs.append(f"{node_path}: {err}")
                elif match_idx is not None:
                    used_leaf_indexes.add(match_idx)
            if not round_errs:
                matched_round = round_idx
                cursor = round_idx + 1
                break
            if len(observed) < 3:
                contracts = [(c.get("contract_id"), c.get("call_id")) for c in step_calls]
                observed.append(f"round {round_idx}: calls={contracts}; {'; '.join(round_errs)}")
        if matched_round is None:
            expected_contracts = [want["contract"] for _node_path, _node, want in expected_nodes]
            detail = " | ".join(observed) if observed else "no later plan rounds"
            fails.append(
                f"steps[{step_idx}] did not match any plan round from {cursor}; "
                f"expected contracts={expected_contracts}; observed {detail}"
            )

    # Unexpected failures: a failed leaf must be explicitly expected, with matching
    # contract and args/output assertions. Contract-only allow-lists are too loose.
    if not scenario.get("allow_leaf_failure", False):
        for lr in leaves:
            if lr.get("success", False):
                continue
            matched = False
            for _step_idx, want in expected_failure_specs:
                if not leaf_satisfies_expected(want, all_calls, lr):
                    matched = True
                    break
            if not matched:
                cid = lr.get("contract_id", "")
                fails.append(f"unexpected leaf failure {cid}: {lr.get('error', '')[:160]}")
    return fails


def main() -> int:
    ap = argparse.ArgumentParser(description="robonix CI scenario runner")
    ap.add_argument("--rbnx", default=os.environ.get("RBNX_BIN", "rbnx"))
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    ap.add_argument("--scenarios", type=Path, default=Path(__file__).resolve().parent / "scenarios")
    ap.add_argument("--timeout", type=int, default=300, help="per-scenario seconds")
    ap.add_argument("--only", help="run only the scenario with this name")
    ap.add_argument(
        "--first",
        action="append",
        default=[],
        metavar="NAME",
        help="run the named scenario before the remaining sorted scenarios; repeatable",
    )
    ap.add_argument("--summary-json", type=Path, help="write a machine-readable result summary here")
    args = ap.parse_args()

    paths = sorted([*args.scenarios.rglob("*.yaml"), *args.scenarios.rglob("*.yml")])
    if args.only:
        paths = [p for p in paths if p.stem == args.only]
    elif args.first:
        priority = {name: index for index, name in enumerate(args.first)}
        paths.sort(key=lambda path: (priority.get(path.stem, len(priority)), str(path)))
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
            events = []
            for line in stdout.splitlines():
                line = line.strip()
                if not line:
                    continue
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
            observed_failures = check_scenario(scenario, events, 124) if events else []
            failures = [f"timeout after {args.timeout}s"]
            failures.extend(f"before timeout: {failure}" for failure in observed_failures)
            log_path.write_text(
                f"$ {args.rbnx} ask {scenario['task']} --json --server {args.server}\n"
                f"# timeout={args.timeout}s\n# --- stdout ---\n{stdout}\n"
                f"# --- stderr ---\n{stderr}\n"
            )
            calls = collect_calls(events)
            results.append({
                "name": name,
                "family": family,
                "passed": False,
                "rounds": len(collect_plan_rounds(events)),
                "dispatched": calls,
                "failures": failures,
                "log": log_path.name,
                "rtdl_steps": summarize_rtdl_steps(scenario),
                "observed_rounds": summarize_plan_rounds(events),
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
            "rtdl_steps": summarize_rtdl_steps(scenario),
            "observed_rounds": summarize_plan_rounds(events),
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
