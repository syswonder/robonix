# SPDX-License-Identifier: MulanPSL-2.0
"""Offline CI harness simulation for scenario YAML.

This does not replace the Webots integration suite. It validates the scenario
spec itself before we spend GPU/Webots time:

* parse every scenario;
* reject old/ambiguous assertion shapes through testing.run's validator;
* require every RTDL do node to define a leaf expectation;
* verify variables are captured before later steps reference them;
* synthesize PilotEvent plan/result streams and feed them to run.check_scenario.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

import run as scenario_run  # noqa: E402
from fake_vlm import server as fake_vlm_server  # noqa: E402

TRANSFORMS = {"", "float", "int", "str", "sin_half", "cos_half"}
VAR_REF = re.compile(r"\$([A-Za-z_][A-Za-z0-9_]*)")


def iter_do_nodes(node: Any, path: str = "rtdl"):
    if not isinstance(node, dict):
        return
    if node.get("op") == "do":
        yield path, node
    for idx, child in enumerate(node.get("children", []) or []):
        yield from iter_do_nodes(child, f"{path}.children[{idx}]")


def vars_in_value(value: Any) -> set[str]:
    out: set[str] = set()
    if isinstance(value, dict):
        if set(value.keys()) >= {"var"}:
            out.add(str(value.get("var", "")))
        for child in value.values():
            out.update(vars_in_value(child))
    elif isinstance(value, list):
        for child in value:
            out.update(vars_in_value(child))
    elif isinstance(value, str):
        out.update(VAR_REF.findall(value))
    return {v for v in out if v}


def validate_scenario_shape(scenario: dict) -> list[str]:
    errors = scenario_run.validate_new_assertion_shape(scenario)
    defined: set[str] = set()
    for step_idx, step in enumerate(scenario.get("steps", [])):
        rtdl = step.get("rtdl")
        for node_path, node in iter_do_nodes(rtdl):
            missing = sorted(vars_in_value(node.get("args", {})) - defined)
            if missing:
                errors.append(f"steps[{step_idx}].{node_path} references undefined vars: {missing}")
            expect = node.get("expect") or {}
            captures = expect.get("capture") or {}
            if captures and not isinstance(captures, dict):
                errors.append(f"steps[{step_idx}].{node_path}.expect.capture must be a mapping")
                continue
            for name, spec in captures.items():
                if not isinstance(spec, dict):
                    errors.append(f"capture {name!r} must be a mapping")
                    continue
                transform = str(spec.get("transform", ""))
                if transform not in TRANSFORMS:
                    errors.append(f"capture {name!r} has unknown transform {transform!r}")
                defined.add(str(name))
    return errors


def value_for_capture(name: str, spec: dict) -> Any:
    transform = str(spec.get("transform", ""))
    if transform in {"float", "sin_half", "cos_half"} or name.endswith(("_x", "_y", "_yaw")):
        return 0.5 if not name.endswith("_yaw") else 0.25
    if transform == "int":
        return 1
    if "id" in name:
        return "scene.object.simulated_001"
    return f"simulated-{name}"


def _set_path(root: dict, path: str, value: Any) -> None:
    if path == "$":
        if isinstance(value, dict):
            root.update(value)
        return
    if not path.startswith("$."):
        return
    cur: Any = root
    tokens = path[2:].split(".")
    for token in tokens[:-1]:
        if token.endswith("[]"):
            key = token[:-2]
            cur.setdefault(key, [{}])
            cur = cur[key][0]
        else:
            cur = cur.setdefault(token, {})
    last = tokens[-1]
    if last.endswith("[]"):
        cur.setdefault(last[:-2], [value])
    else:
        cur[last] = value


def _ensure_array(root: dict, path: str) -> list:
    if not path.startswith("$.") or not path.endswith("[]"):
        return []
    cur: Any = root
    tokens = path[2:-2].split(".")
    for token in tokens[:-1]:
        cur = cur.setdefault(token, {})
    return cur.setdefault(tokens[-1], [])


def _value_for_predicate(path: str, op: str, expected: Any) -> Any:
    key = path.rsplit(".", 1)[-1]
    if op == "starts_with":
        return f"{expected}simulated_001"
    if op == "contains":
        return f"simulated-{expected}-value"
    if op == "regex":
        return synthetic_text_for_regex(str(expected))
    if op == "ne":
        if expected == "robot":
            return "table"
        return "simulated"
    if op in {"gt", "gte"}:
        return float(expected or 0) + 1.0
    if op in {"lt", "lte"}:
        return float(expected or 1) - 1.0
    if op in {"in", "eq"}:
        if isinstance(expected, list):
            return expected[0] if expected else "simulated"
        return expected
    if key == "id":
        return "scene.object.simulated_001"
    if key == "label":
        return "table"
    return "simulated"


def _apply_predicate_to_item(item: dict, expr: Any) -> None:
    if not isinstance(expr, dict):
        return
    if "all" in expr or "any" in expr:
        for child in expr.get("all") or expr.get("any") or []:
            _apply_predicate_to_item(item, child)
        return
    if "not" in expr:
        return
    path = str(expr.get("path", "$"))
    if not path.startswith("$."):
        return
    key = path[2:]
    if "." in key or key.endswith("[]"):
        return
    item[key] = _value_for_predicate(path, str(expr.get("op", "exists")), expr.get("value"))


def _sample_item_for_where(where: Any) -> dict:
    item = {"id": "scene.object.simulated_001", "label": "table"}
    _apply_predicate_to_item(item, where)
    return item


def _satisfy_check(root: dict, check: dict) -> None:
    select = str(check.get("select", "$"))
    assertion = check.get("assert", {"op": "exists"})
    if select.endswith("[]"):
        array = _ensure_array(root, select)
        where = check.get("where")
        if not array or (where is not None and not any(scenario_run.check_expr(item, where) for item in array)):
            array.append(_sample_item_for_where(where))
        return
    if isinstance(assertion, dict) and assertion.get("op") == "min_length":
        current = scenario_run.select_json_values(root, select)
        if not current or not hasattr(current[0], "__len__") or len(current[0]) < int(assertion.get("value", 1)):
            _set_path(root, select, [{}])
    elif isinstance(assertion, dict) and assertion.get("op") == "regex":
        values = scenario_run.select_json_values(root, select)
        current = values[0] if values and isinstance(values[0], str) else ""
        fragment = synthetic_text_for_regex(str(assertion.get("value", "")))
        _set_path(root, select, f"{current} {fragment}".strip())
    elif isinstance(assertion, dict) and assertion.get("op") not in {None, "exists"}:
        _set_path(root, select, _value_for_predicate(select, str(assertion.get("op")), assertion.get("value")))
    else:
        _set_path(root, select, "simulated")


def _satisfy_capture(root: dict, name: str, spec: dict) -> None:
    if "select" not in spec:
        return
    select = str(spec.get("select", "$"))
    value = value_for_capture(name, spec)
    if select.endswith("[]"):
        array = _ensure_array(root, select)
        item = _sample_item_for_where(spec.get("where"))
        path = str(spec.get("path", "$"))
        if path.startswith("$.") and "." not in path[2:] and not path.endswith("[]"):
            item[path[2:]] = value
        if not array:
            array.append(item)
        else:
            array[0].update(item)
    else:
        path = str(spec.get("path", "$"))
        if path == "$":
            _set_path(root, select, value)
        elif select == "$":
            _set_path(root, path, value)
        else:
            container: dict[str, Any] = {}
            _set_path(container, path, value)
            _set_path(root, select, container)


def synthetic_text_for_regex(pattern: str) -> str:
    if "No such file" in pattern:
        return "No such file"
    if "hit 5" in pattern:
        return "hit 5.0s ceiling"
    if "hit 8" in pattern:
        return "hit 8.0s ceiling"
    if "wrote" in pattern:
        return "wrote 28 bytes to /tmp/ci_probe.txt"
    if "ci_probe" in pattern:
        return "file ci_probe.txt"
    if "users_json" in pattern:
        return '{"users_json": "[]", "count": 0}'
    if "spoke" in pattern:
        return '{"ok": true, "detail": "spoke 35 chars"}'
    if "label" in pattern:
        return json.dumps({"objects": [{"id": "scene.object.simulated_001", "label": "table"}]})
    if "approach pose" in pattern:
        return json.dumps({"reachable": True, "x": 0.5, "y": 0.0, "yaw": 0.25, "reason": "approach pose for simulated object"})
    sample = pattern.replace("^", "").replace("$", "").replace("\\n?", "")
    sample = re.sub(r"\\s(?:[+*?]|\{\d+(?:,\d*)?\})?", " ", sample)
    sample = re.sub(r"\\d(?:[+*?]|\{\d+(?:,\d*)?\})?", "1", sample)
    sample = re.sub(r"\.\*\??", "sample", sample)
    sample = re.sub(r"\\([\\\"'/:.,_ -])", r"\1", sample)
    try:
        if re.search(pattern, sample):
            return sample
    except re.error:
        pass
    return pattern.replace("^", "").replace("$", "")


def output_for_expect(expect: dict) -> tuple[str, str]:
    output_clause = expect.get("output") or {}
    if not expect.get("success", True):
        pattern = str(output_clause.get("error_regex", "expected failure"))
        return "", synthetic_text_for_regex(pattern)
    if "text_equals" in output_clause:
        return str(output_clause["text_equals"]), ""
    if "text_regex" in output_clause:
        text = synthetic_text_for_regex(str(output_clause["text_regex"]))
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            value = {}
        if isinstance(value, dict):
            for check in output_clause.get("checks", []) or []:
                if isinstance(check, dict):
                    _satisfy_check(value, check)
            for name, spec in (expect.get("capture") or {}).items():
                if isinstance(spec, dict):
                    _satisfy_capture(value, str(name), spec)
            if value:
                text = json.dumps(value)
        return text, ""
    value: dict[str, Any] = {}
    if isinstance(output_clause.get("json"), dict):
        value.update(output_clause["json"])
    for check in output_clause.get("checks", []) or []:
        if isinstance(check, dict):
            _satisfy_check(value, check)
    for name, spec in (expect.get("capture") or {}).items():
        if isinstance(spec, dict):
            _satisfy_capture(value, str(name), spec)
    return json.dumps(value or {"ok": True}), ""


def resolve_args(value: Any, vars_: dict[str, Any]) -> Any:
    if isinstance(value, dict):
        if set(value.keys()) >= {"var"}:
            name = str(value.get("var"))
            val = vars_[name]
            transform = str(value.get("transform", ""))
            if transform == "sin_half":
                import math
                return math.sin(float(val) / 2.0)
            if transform == "cos_half":
                import math
                return math.cos(float(val) / 2.0)
            return val
        return {k: resolve_args(v, vars_) for k, v in value.items()}
    if isinstance(value, list):
        return [resolve_args(v, vars_) for v in value]
    if isinstance(value, str):
        exact = VAR_REF.fullmatch(value)
        if exact:
            return vars_[exact.group(1)]
        return VAR_REF.sub(lambda m: str(vars_[m.group(1)]), value)
    return value


def synthesize_events(scenario: dict) -> list[dict]:
    events: list[dict] = []
    vars_: dict[str, Any] = {}
    for step_idx, step in enumerate(scenario.get("steps", [])):
        calls = []
        leaves = []
        do_nodes = list(iter_do_nodes(step.get("rtdl")))
        for leaf_idx, (_path, node) in enumerate(do_nodes):
            expect = node["expect"]
            args = resolve_args(node.get("args", {}), vars_)
            call_id = f"{step_idx + 1}:{leaf_idx}"
            calls.append({
                "call_id": call_id,
                "contract_id": expect["contract"],
                "provider_id": str(expect["contract"]).split("/")[1] if "/" in str(expect["contract"]) else "sim",
                "args_json": json.dumps(args),
            })
            output, error = output_for_expect(expect)
            leaves.append({
                "call_id": call_id,
                "contract_id": expect["contract"],
                "success": expect.get("success", True),
                "output": output,
                "error": error,
            })
            for name, spec in (expect.get("capture") or {}).items():
                if isinstance(spec, dict):
                    vars_[str(name)] = value_for_capture(str(name), spec)
        if calls:
            events.append({"plan": {"round": step_idx, "calls": calls}})
            events.append({"batch_result": {"round": step_idx, "results": [{"leaf_result": leaf} for leaf in leaves]}})
    return events



def all_caps_for_scenario(scenario: dict) -> list[str]:
    caps: list[str] = []
    for step in scenario.get("steps", []):
        for _path, node in iter_do_nodes(step.get("rtdl")):
            cap = str(node.get("cap", ""))
            if cap and cap not in caps:
                caps.append(cap)
    return caps


def find_test_only_keys(value: object) -> set[str]:
    out: set[str] = set()
    if isinstance(value, dict):
        for key, child in value.items():
            if key in {"expect", "capture", "once"}:
                out.add(key)
            out.update(find_test_only_keys(child))
    elif isinstance(value, list):
        for child in value:
            out.update(find_test_only_keys(child))
    return out


def simulate_fake_vlm_compile(scenario: dict) -> list[str]:
    errors: list[str] = []
    messages = [
        {"role": "system", "content": "\n".join(f"- capability_name: {cap}" for cap in all_caps_for_scenario(scenario))},
        {"role": "user", "content": scenario["task"]},
    ]
    once_seen: set[str] = set()
    for step_idx, step in enumerate(scenario.get("steps", [])):
        unresolved: list[str] = []
        vars_ = fake_vlm_server.capture_vars_from_history(scenario, messages, step_idx)
        envelope = fake_vlm_server.build_envelope(step, all_caps_for_scenario(scenario), vars_, unresolved, once_seen)
        if unresolved:
            errors.append(f"step {step_idx} unresolved in fake VLM compiler: {unresolved}")
        leaked = sorted(find_test_only_keys(envelope))
        if leaked:
            errors.append(f"step {step_idx} leaked test-only key(s) into RTDL envelope: {leaked}")
        leaves = []
        for leaf_idx, (_path, node) in enumerate(iter_do_nodes(step.get("rtdl"))):
            expect = node["expect"]
            output, error = output_for_expect(expect)
            leaves.append({
                "leaf_result": {
                    "call_id": f"{step_idx + 1}:{leaf_idx}",
                    "contract_id": expect["contract"],
                    "success": expect.get("success", True),
                    "output": output,
                    "error": error,
                }
            })
        if leaves:
            messages.append({"role": "user", "content": json.dumps({"batch_result": {"results": leaves}})})
        else:
            messages.append({"role": "user", "content": "{}"})
    return errors


def main() -> int:
    ap = argparse.ArgumentParser(description="simulate the Robonix scenario CI harness offline")
    ap.add_argument("--scenarios", type=Path, default=ROOT / "scenarios")
    args = ap.parse_args()
    failures: list[str] = []
    paths = sorted([*args.scenarios.rglob("*.yaml"), *args.scenarios.rglob("*.yml")])
    for path in paths:
        scenario = yaml.safe_load(path.read_text())
        scenario.setdefault("name", path.stem)
        shape_errors = validate_scenario_shape(scenario)
        if shape_errors:
            failures.extend(f"{path}: {err}" for err in shape_errors)
            continue
        compile_errors = simulate_fake_vlm_compile(scenario)
        if compile_errors:
            failures.extend(f"{path}: {err}" for err in compile_errors)
            continue
        events = synthesize_events(scenario)
        run_errors = scenario_run.check_scenario(scenario, events, 0)
        if run_errors:
            failures.extend(f"{path}: {err}" for err in run_errors)
    if failures:
        print("scenario simulation failed:", file=sys.stderr)
        for err in failures:
            print(f"  - {err}", file=sys.stderr)
        return 1
    print(f"scenario simulation passed: {len(paths)} scenario(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
