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


def assign_path(root: dict, path: str, value: Any, where: dict | None = None) -> None:
    if path.startswith("$."):
        path = path[2:]
    cur: Any = root
    tokens = path.split(".") if path else []
    for token in tokens[:-1]:
        if token.endswith("[]"):
            key = token[:-2]
            item = dict(where or {})
            if "label_not" in item:
                item["label"] = "table"
                item.pop("label_not", None)
            if "id_prefix" in item:
                item["id"] = f"{item.pop('id_prefix')}simulated_001"
            cur.setdefault(key, [item])
            cur = cur[key][0]
        else:
            cur = cur.setdefault(token, {})
    if not tokens:
        return
    last = tokens[-1]
    if last.endswith("[]"):
        key = last[:-2]
        cur.setdefault(key, [value])
    elif isinstance(cur, dict):
        cur[last] = value


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
    return pattern.replace("^", "").replace("$", "").replace("\\n?", "")


def output_for_expect(expect: dict) -> tuple[str, str]:
    output_clause = expect.get("output") or {}
    if not expect.get("success", True):
        pattern = str(output_clause.get("error_regex", "expected failure"))
        return "", synthetic_text_for_regex(pattern)
    if "text_equals" in output_clause:
        return str(output_clause["text_equals"]), ""
    if "text_regex" in output_clause:
        return synthetic_text_for_regex(str(output_clause["text_regex"])), ""
    value: dict[str, Any] = {}
    if isinstance(output_clause.get("json"), dict):
        value.update(output_clause["json"])
    for cond in output_clause.get("jsonpath", []) or []:
        path = str(cond.get("path", ""))
        if cond.get("min_length"):
            assign_path(value, path, [{}])
        elif cond.get("exists"):
            prefix = str(cond.get("prefix", "value"))
            assign_path(value, path, f"{prefix}simulated")
        elif "equals" in cond:
            assign_path(value, path, cond["equals"])
    for name, spec in (expect.get("capture") or {}).items():
        if isinstance(spec, dict):
            assign_path(value, str(spec.get("jsonpath") or spec.get("field") or ""), value_for_capture(str(name), spec), spec.get("where"))
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
