# SPDX-License-Identifier: MulanPSL-2.0
"""Deterministic fake VLM for robonix CI.

Pilot talks to an OpenAI-compatible ``/v1/chat/completions`` endpoint in
streaming mode and expects one RTDL envelope as assistant content. This server
serves scenario-defined RTDL trees so planning is deterministic while every leaf
capability call still executes through the live Robonix deployment.

Scenario model:

* one ``steps[]`` entry = one VLM planning round;
* ``steps[].rtdl`` is the RTDL tree returned for that round;
* test-only keys such as ``id``, ``expect``, ``capture``, and ``once`` are never
  sent to Pilot;
* variables captured from earlier leaf outputs can be referenced in later args
  with ``{var: name}`` or string interpolation like ``"$name"``.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import yaml

_CAP_LINE = re.compile(r"^\s*-\s*capability_name:\s*(\S+)\s*$", re.MULTILINE)
_VAR_TOKEN = re.compile(r"\$([A-Za-z_][A-Za-z0-9_]*)")
_TEST_ONLY_NODE_KEYS = {"id", "expect", "capture", "once"}


def advertised_caps(messages: list[dict]) -> list[str]:
    seen: dict[str, None] = {}
    for m in messages:
        if m.get("role") != "system":
            continue
        content = m.get("content")
        if not isinstance(content, str):
            continue
        for name in _CAP_LINE.findall(content):
            seen.setdefault(name, None)
    return list(seen)


def resolve_cap(cap: str, caps: list[str]) -> str | None:
    return cap if cap in caps else None


def _json_values_from_text(text: str) -> list[Any]:
    values: list[Any] = []
    decoder = json.JSONDecoder()
    i = 0
    while i < len(text):
        if text[i] not in "[{":
            i += 1
            continue
        try:
            value, end = decoder.raw_decode(text[i:])
        except json.JSONDecodeError:
            i += 1
            continue
        values.append(value)
        i += max(1, end)
    return values


def _walk_json(value: Any):
    yield value
    if isinstance(value, dict):
        for child in value.values():
            if isinstance(child, str) and child.strip().startswith(("{", "[")):
                try:
                    child = json.loads(child)
                except json.JSONDecodeError:
                    pass
            yield from _walk_json(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk_json(child)


def _parse_maybe_json(value: Any) -> Any:
    if isinstance(value, str) and value.strip().startswith(("{", "[")):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            return value
    return value


def _json_path_values(value: Any, path: str) -> list[Any]:
    value = _parse_maybe_json(value)
    if path == "$":
        return [value]
    if not path.startswith("$."):
        return []
    current = [value]
    for token in path[2:].split("."):
        next_values: list[Any] = []
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


def _check_scalar(value: Any, op: str, expected: Any = None) -> bool:
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
        return re.search(str(expected), str(value)) is not None
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


def _check_values(values: list[Any], check: dict) -> bool:
    op = str(check.get("op", "exists"))
    expected = check.get("value")
    if op == "exists":
        return bool(values)
    return any(_check_scalar(value, op, expected) for value in values)


def _check_expr(context: Any, expr: Any) -> bool:
    if not isinstance(expr, dict):
        return False
    if "all" in expr:
        items = expr.get("all") or []
        return isinstance(items, list) and all(_check_expr(context, item) for item in items)
    if "any" in expr:
        items = expr.get("any") or []
        return isinstance(items, list) and any(_check_expr(context, item) for item in items)
    if "not" in expr:
        return not _check_expr(context, expr.get("not"))
    return _check_values(_json_path_values(context, str(expr.get("path", "$"))), expr)


def _matches_where(value: Any, where: Any) -> bool:
    if not where:
        return True
    return _check_expr(value, where)


def _extract_path(value: Any, path: str, where: dict | None = None) -> Any:
    value = _parse_maybe_json(value)
    if path.startswith("$."):
        path = path[2:]
    if not path:
        return value
    cur: Any = value
    for token in path.split("."):
        if token.endswith("[]"):
            key = token[:-2]
            if not isinstance(cur, dict) or not isinstance(cur.get(key), list):
                return None
            chosen = None
            for item in cur[key]:
                if _matches_where(item, where):
                    chosen = item
                    break
            if chosen is None:
                return None
            cur = chosen
            continue
        if isinstance(cur, dict):
            if token in cur:
                cur = cur[token]
            elif token == "id" and "object_id" in cur:
                cur = cur["object_id"]
            elif token == "label" and "cls" in cur:
                cur = cur["cls"]
            else:
                return None
        else:
            return None
    return cur


def _apply_transform(value: Any, transform: str) -> Any:
    if not transform:
        return value
    if transform == "sin_half":
        return math.sin(float(value) / 2.0)
    if transform == "cos_half":
        return math.cos(float(value) / 2.0)
    if transform == "float":
        return float(value)
    if transform == "int":
        return int(value)
    if transform == "str":
        return str(value)
    raise ValueError(f"unknown variable transform {transform!r}")


def _leaf_results_from_messages(messages: list[dict]) -> list[dict]:
    parts: list[str] = []
    for m in messages:
        if m.get("role") != "user":
            continue
        content = m.get("content")
        if isinstance(content, str):
            parts.append(content)
        elif isinstance(content, list):
            parts.extend(p.get("text", "") for p in content if p.get("type") == "text")
    leaves: list[dict] = []
    seen: set[tuple[str, str]] = set()

    def append_leaf(leaf: dict, container: dict) -> None:
        # `_walk_json` visits both `{call_id, leaf_result: {...}}` and the
        # nested leaf dict. Without de-duplication the same completed call is
        # counted twice, so a scenario with repeated contracts can skip its
        # next real step. Prefer the executor's stable call id. Older payloads
        # without one fall back to the canonical leaf content, which also
        # removes replayed copies of the same historical result.
        call_id = container.get("call_id") or leaf.get("call_id")
        if call_id is not None and str(call_id):
            key = ("call_id", str(call_id))
        else:
            key = (
                "leaf",
                json.dumps(
                    leaf,
                    sort_keys=True,
                    separators=(",", ":"),
                    default=str,
                ),
            )
        if key in seen:
            return
        seen.add(key)
        leaves.append(leaf)

    def collect(node: Any) -> None:
        if isinstance(node, list):
            for item in node:
                collect(item)
            return
        if not isinstance(node, dict):
            return
        leaf = node.get("leaf_result")
        if isinstance(leaf, dict):
            append_leaf(leaf, node)
            # The nested dict is the same result, not a second call. Continue
            # through siblings only in case the envelope also carries results.
            for key, value in node.items():
                if key != "leaf_result":
                    collect(value)
            return
        if {"contract_id", "success", "output"}.issubset(node.keys()):
            append_leaf(node, node)
            return
        for value in node.values():
            collect(value)

    for value in _json_values_from_text("\n".join(parts)):
        collect(value)
    return leaves


def _message_texts(messages: list[dict], roles: set[str] | None = None) -> list[str]:
    parts: list[str] = []
    for m in messages:
        if roles is not None and m.get("role") not in roles:
            continue
        content = m.get("content")
        if isinstance(content, str):
            parts.append(content)
        elif isinstance(content, list):
            parts.extend(p.get("text", "") for p in content if p.get("type") == "text")
    return parts


def _iter_do_nodes(node: Any):
    if not isinstance(node, dict):
        return
    if node.get("op") == "do":
        yield node
    for child in node.get("children", []) or []:
        yield from _iter_do_nodes(child)


def _leaf_matches_contract_success(leaf: dict, expect: dict) -> bool:
    contract = expect.get("contract")
    if contract and leaf.get("contract_id") != contract:
        return False
    return leaf.get("success", False) == expect.get("success", True)


def _leaf_matches_expect(leaf: dict, expect: dict) -> bool:
    return not _leaf_expect_errors(leaf, expect)


def _contains_subset(actual: Any, expected: Any) -> bool:
    if isinstance(expected, dict):
        if not isinstance(actual, dict):
            return False
        return all(k in actual and _contains_subset(actual[k], v) for k, v in expected.items())
    if isinstance(expected, list):
        if not isinstance(actual, list) or len(actual) < len(expected):
            return False
        return all(_contains_subset(a, e) for a, e in zip(actual, expected))
    return actual == expected


def _structured_check_errors(parsed: Any, clause: dict) -> list[str]:
    errors: list[str] = []
    for idx, check in enumerate(clause.get("checks", []) or []):
        if not isinstance(check, dict):
            errors.append(f"checks[{idx}]")
            continue
        selected = _json_path_values(parsed, str(check.get("select", "$")))
        where = check.get("where")
        if where is not None:
            selected = [value for value in selected if _matches_where(value, where)]
        assertion = check.get("assert", {"op": "exists"})
        if not isinstance(assertion, dict) or not _check_values(selected, assertion):
            errors.append(f"checks[{idx}]")
    return errors


def _leaf_expect_errors(leaf: dict, expect: dict) -> list[str]:
    errors: list[str] = []
    contract = expect.get("contract")
    if contract and leaf.get("contract_id") != contract:
        errors.append("contract")
    if leaf.get("success", False) != expect.get("success", True):
        errors.append("success")

    clause = expect.get("output")
    if not isinstance(clause, dict):
        return errors

    raw = str(leaf.get("output", ""))
    err_text = str(leaf.get("error", ""))
    parsed = _parse_maybe_json(raw.strip())
    if "text_equals" in clause and raw != clause["text_equals"]:
        errors.append("text_equals")
    if "text_regex" in clause and not re.search(str(clause["text_regex"]), raw):
        errors.append("text_regex")
    if "error_regex" in clause and not re.search(str(clause["error_regex"]), err_text):
        errors.append("error_regex")
    for line in clause.get("text_lines", []) or []:
        if line not in raw.splitlines():
            errors.append("text_lines")
    if "json" in clause and not _contains_subset(parsed, clause["json"]):
        errors.append("json")
    errors.extend(_structured_check_errors(parsed, clause))
    return errors


def _step_expected_do_nodes(step: dict) -> list[dict]:
    return [node for node in _iter_do_nodes(step.get("rtdl")) if isinstance(node.get("expect"), dict)]


def _step_expected_contracts(step: dict) -> set[str]:
    contracts: set[str] = set()
    for node in _step_expected_do_nodes(step):
        contract = node.get("expect", {}).get("contract")
        if isinstance(contract, str):
            contracts.add(contract)
    return contracts


def _planned_contracts_from_messages(messages: list[dict]) -> set[str]:
    planned: set[str] = set()
    for text in _message_texts(messages):
        for value in _json_values_from_text(text):
            for node in _walk_json(value):
                if not isinstance(node, dict):
                    continue
                rtdl = node.get("rtdl")
                if isinstance(rtdl, dict):
                    for do_node in _iter_do_nodes(rtdl):
                        cap = do_node.get("cap")
                        if isinstance(cap, str):
                            planned.add(cap)
                contract_id = node.get("contract_id")
                if isinstance(contract_id, str):
                    planned.add(contract_id)
    return planned


def _step_is_complete(step: dict, leaves: list[dict], consumed: set[int]) -> bool:
    nodes = _step_expected_do_nodes(step)
    if not nodes:
        return step.get("status") == "done"
    local_used: set[int] = set()
    for node in nodes:
        expect = node["expect"]
        match_idx = None
        for idx, leaf in enumerate(leaves):
            if idx in consumed or idx in local_used:
                continue
            require_full_expect = bool(step.get("retry_delay_s") or expect.get("capture"))
            matcher = _leaf_matches_expect if require_full_expect else _leaf_matches_contract_success
            if matcher(leaf, expect):
                match_idx = idx
                break
        if match_idx is None:
            return False
        local_used.add(match_idx)
    consumed.update(local_used)
    return True


def next_step_index_from_history(scenario: dict, messages: list[dict]) -> int:
    leaves = _leaf_results_from_messages(messages)
    consumed: set[int] = set()
    steps = scenario.get("steps", [])
    last_done_idx = len(steps)
    for idx, step in enumerate(steps):
        if step.get("status") == "done" and step.get("rtdl") is None:
            last_done_idx = idx
            break
        if not _step_is_complete(step, leaves, consumed):
            return idx
    return last_done_idx


def _capture_one(leaf: dict, spec: Any) -> Any:
    if not isinstance(spec, dict):
        return None
    source = str(spec.get("source", "output"))
    text = str(leaf.get("error" if source == "error" else "output", ""))
    parsed = _parse_maybe_json(text)
    if "select" in spec:
        selected = _json_path_values(parsed, str(spec.get("select", "$")))
        where = spec.get("where")
        if where is not None:
            selected = [value for value in selected if _matches_where(value, where)]
        if not selected:
            return None
        context = selected[0]
        value = _extract_path(context, str(spec.get("path", "$")))
    elif "regex" in spec:
        m = re.search(str(spec["regex"]), text, re.MULTILINE | re.DOTALL)
        value = m.group(int(spec.get("group", 1))) if m else None
    else:
        value = parsed
    if value is None:
        return None
    try:
        return _apply_transform(value, str(spec.get("transform", "")))
    except Exception:
        return value


def capture_vars_from_history(scenario: dict, messages: list[dict], before_step_index: int) -> dict[str, Any]:
    leaves = _leaf_results_from_messages(messages)
    consumed: set[int] = set()
    vars_: dict[str, Any] = {}
    for step in scenario.get("steps", [])[:before_step_index]:
        for node in _iter_do_nodes(step.get("rtdl")):
            expect = node.get("expect")
            if not isinstance(expect, dict):
                continue
            captures = expect.get("capture") or {}
            if not isinstance(captures, dict):
                continue
            match_idx = None
            for idx, leaf in enumerate(leaves):
                if idx in consumed:
                    continue
                if _leaf_matches_expect(leaf, expect):
                    match_idx = idx
                    break
            if match_idx is None:
                continue
            consumed.add(match_idx)
            leaf = leaves[match_idx]
            for name, cap in captures.items():
                value = _capture_one(leaf, cap)
                if value is not None:
                    vars_[str(name)] = value
    return vars_


def resolve_arg_refs(value: Any, unresolved: list[str], vars_: dict[str, Any]) -> Any:
    if isinstance(value, dict):
        if set(value.keys()) >= {"var"}:
            name = str(value.get("var", ""))
            if name not in vars_:
                unresolved.append(f"var ${name}")
                return value.get("default")
            try:
                return _apply_transform(vars_[name], str(value.get("transform", "")))
            except Exception as exc:  # noqa: BLE001
                unresolved.append(f"var ${name}: {exc}")
                return value.get("default")
        return {k: resolve_arg_refs(v, unresolved, vars_) for k, v in value.items()}
    if isinstance(value, list):
        return [resolve_arg_refs(v, unresolved, vars_) for v in value]
    if isinstance(value, str):
        exact = _VAR_TOKEN.fullmatch(value)
        if exact:
            name = exact.group(1)
            if name not in vars_:
                unresolved.append(f"var ${name}")
                return value
            return vars_[name]

        def repl(match: re.Match[str]) -> str:
            name = match.group(1)
            if name not in vars_:
                unresolved.append(f"var ${name}")
                return match.group(0)
            return str(vars_[name])

        return _VAR_TOKEN.sub(repl, value)
    return value


def _compile_rtdl_node(node: Any, caps: list[str], vars_: dict[str, Any], unresolved: list[str], once_seen: set[str]) -> dict | None:
    if not isinstance(node, dict):
        unresolved.append(f"invalid rtdl node {node!r}")
        return None
    op = str(node.get("op", ""))
    if op == "do":
        wanted = str(node.get("cap", ""))
        once_key = str(node.get("id") or wanted)
        if node.get("once") and once_key in once_seen:
            return None
        cap = resolve_cap(wanted, caps)
        if cap is None:
            unresolved.append(wanted or "<missing cap>")
            return None
        local_unresolved: list[str] = []
        args = resolve_arg_refs(node.get("args", {}), local_unresolved, vars_)
        if local_unresolved:
            unresolved.extend(local_unresolved)
            return None
        out = {
            "op": "do",
            "op_id": int(node.get("op_id", 0) or 0),
            "description": node.get("description") or node.get("id") or wanted,
            "cap": cap,
            "args": args,
        }
        if node.get("once"):
            once_seen.add(once_key)
        return out
    if op in {"sequence", "parallel"}:
        children = []
        for child in node.get("children", []) or []:
            compiled = _compile_rtdl_node(child, caps, vars_, unresolved, once_seen)
            if compiled is not None:
                children.append(compiled)
        out = {
            "op": op,
            "op_id": int(node.get("op_id", 0) or 0),
            "description": node.get("description") or node.get("id") or op,
            "children": children,
        }
        for key, value in node.items():
            if key not in out and key not in _TEST_ONLY_NODE_KEYS and key != "children":
                out[key] = value
        return out
    unresolved.append(f"unknown op {op!r}")
    return None


def build_envelope(step: dict, caps: list[str], vars_: dict[str, Any], unresolved: list[str], once_seen: set[str]) -> dict:
    rtdl = step.get("rtdl") or {"op": "sequence", "description": "wait", "children": []}
    compiled = _compile_rtdl_node(rtdl, caps, vars_, unresolved, once_seen)
    if compiled is None:
        compiled = {"op": "sequence", "op_id": 0, "description": "unresolved", "children": []}

    task_update = step.get("task_update")
    if not task_update and step.get("status"):
        task_update = {
            "goal": step.get("goal", "ci scenario"),
            "success_criterion": step.get("success_criterion", "scripted steps complete"),
            "status": step.get("status"),
        }
    return {
        "content": step.get("content", ""),
        "rtdl_description": step.get("rtdl_description", compiled.get("description", "ci")),
        "rtdl": compiled,
        "task_update": task_update,
    }


def terminal_envelope(note: str) -> dict:
    return {
        "content": note,
        "rtdl_description": "ci-done",
        "rtdl": {"op": "sequence", "op_id": 0, "description": "wait", "children": []},
        "task_update": {"goal": "ci scenario", "success_criterion": "scripted steps complete", "status": "done"},
    }


def wait_envelope(note: str = "waiting for prior RTDL leaf result") -> dict:
    return {
        "content": note,
        "rtdl_description": "ci-wait",
        "rtdl": {"op": "sequence", "op_id": 0, "description": "wait", "children": []},
        "task_update": {"goal": "ci scenario", "success_criterion": "scripted steps complete", "status": "in_progress"},
    }


class Handler(BaseHTTPRequestHandler):
    scenarios: dict[str, dict] = {}
    timeline_starts: dict[str, float] = {}
    once_seen: dict[str, set[str]] = {}
    pending_steps: dict[str, int] = {}

    def log_message(self, *_args):
        """Suppress the default per-request access log."""

    def _emit(self, line: str):
        print(f"[fake-vlm] {line}", file=sys.stderr, flush=True)

    def do_GET(self):  # noqa: N802
        if self.path.rstrip("/").endswith("/models"):
            body = json.dumps({"object": "list", "data": [{"id": "fake-vlm", "object": "model"}]}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def do_POST(self):  # noqa: N802
        if not self.path.rstrip("/").endswith("/chat/completions"):
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            self.send_error(400, "bad json")
            return

        messages = req.get("messages", [])
        caps = advertised_caps(messages)
        user_text = self._all_user_text(messages)
        request_round = max(0, sum(1 for m in messages if m.get("role") == "user") - 1)

        scenario = self._match_scenario(user_text)
        if scenario is None:
            self._emit(f"no scenario matched user text {user_text[:80]!r}; ending turn")
            envelope = terminal_envelope("no scripted scenario; nothing to do")
        else:
            name = scenario["name"]
            leaves = _leaf_results_from_messages(messages)
            planned_contracts = _planned_contracts_from_messages(messages)
            if request_round == 0 and not leaves and not planned_contracts:
                self.timeline_starts[name] = time.monotonic()
                self.once_seen[name] = set()
                self.pending_steps.pop(name, None)
            steps = scenario.get("steps", [])
            step_index = next_step_index_from_history(scenario, messages)
            if step_index < len(steps):
                pending = self.pending_steps.get(name)
                step = steps[step_index]
                expected_contracts = _step_expected_contracts(step)
                step_already_planned = bool(expected_contracts & planned_contracts)
                step_has_leaf = any(leaf.get("contract_id") in expected_contracts for leaf in leaves)
                if (pending == step_index or step_already_planned) and not step_has_leaf:
                    envelope = wait_envelope()
                    self._emit(f"scenario {name!r} request {request_round} step {step_index}: wait for result")
                else:
                    unresolved: list[str] = []
                    if step_has_leaf and step.get("retry_delay_s"):
                        retry_delay = max(0.0, float(step["retry_delay_s"]))
                        self._emit(
                            f"scenario {name!r} request {request_round} step {step_index}: "
                            f"retry after {retry_delay:.3f}s"
                        )
                        time.sleep(retry_delay)
                    self._apply_timeline_delay(scenario, step, step_index)
                    vars_ = capture_vars_from_history(scenario, messages, step_index)
                    once_seen = self.once_seen.setdefault(name, set())
                    envelope = build_envelope(step, caps, vars_, unresolved, once_seen)
                    if unresolved:
                        self._emit(
                            f"scenario {name!r} request {request_round} step {step_index}: "
                            f"UNRESOLVED {unresolved} (advertised: {caps})"
                        )
                    else:
                        called = [n.get("cap") for n in _iter_do_nodes(step.get("rtdl"))]
                        if called:
                            self.pending_steps[name] = step_index
                        self._emit(f"scenario {name!r} request {request_round} step {step_index}: caps {called}")
            else:
                self.pending_steps.pop(name, None)
                envelope = terminal_envelope("scripted steps complete")

        self._stream_envelope(req.get("model", "fake-vlm"), envelope)

    def _all_user_text(self, messages: list[dict]) -> str:
        parts: list[str] = []
        for m in messages:
            if m.get("role") != "user":
                continue
            content = m.get("content")
            if isinstance(content, str):
                parts.append(content)
            elif isinstance(content, list):
                parts.extend(p.get("text", "") for p in content if p.get("type") == "text")
        return "\n".join(parts)

    def _match_scenario(self, user_text: str) -> dict | None:
        hits = [s for t, s in self.scenarios.items() if t and t in user_text]
        if not hits:
            return None
        return max(hits, key=lambda s: len(s["task"]))

    def _apply_timeline_delay(self, scenario: dict, step: dict, round_index: int) -> None:
        delay = 0.0
        if "time_s" in step:
            start = self.timeline_starts.setdefault(scenario["name"], time.monotonic())
            delay = max(0.0, start + float(step["time_s"]) - time.monotonic())
        elif "delay_s" in step:
            delay = max(0.0, float(step["delay_s"]))
        if delay <= 0.0:
            return
        self._emit(f"scenario {scenario['name']!r} round {round_index}: timeline sleep {delay:.3f}s")
        time.sleep(delay)

    def _stream_envelope(self, model: str, envelope: dict):
        payload = json.dumps(envelope, ensure_ascii=False)
        created = int(time.time())
        base = {"id": "fake-cmpl", "object": "chat.completion.chunk", "created": created, "model": model}

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        def chunk(delta: dict, finish):
            obj = dict(base, choices=[{"index": 0, "delta": delta, "finish_reason": finish}])
            self.wfile.write(f"data: {json.dumps(obj)}\n\n".encode())
            self.wfile.flush()

        chunk({"role": "assistant", "content": payload}, None)
        chunk({}, "stop")
        self.wfile.write(b"data: [DONE]\n\n")
        self.wfile.flush()


def load_scenarios(scenario_dir: Path) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for path in sorted([*scenario_dir.rglob("*.yaml"), *scenario_dir.rglob("*.yml")]):
        data = yaml.safe_load(path.read_text())
        data.setdefault("name", path.stem)
        out[data["task"].strip()] = data
    return out


def main():
    ap = argparse.ArgumentParser(description="Deterministic fake VLM (RTDL) for robonix CI")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=18080)
    ap.add_argument("--scenarios", type=Path, default=Path(__file__).resolve().parent.parent / "scenarios")
    args = ap.parse_args()

    Handler.scenarios = load_scenarios(args.scenarios)
    print(
        f"[fake-vlm] {len(Handler.scenarios)} scenario(s) on http://{args.host}:{args.port}/v1",
        file=sys.stderr,
        flush=True,
    )
    ThreadingHTTPServer((args.host, args.port), Handler).serve_forever()


if __name__ == "__main__":
    main()
