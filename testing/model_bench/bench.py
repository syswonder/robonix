# SPDX-License-Identifier: MulanPSL-2.0
"""Pilot model microbench: score models as Robonix's planner on a booted
Robonix whose world is mocked.

Every case is a task given to the real Pilot with `rbnx ask`. Pilot plans
with the model under test (reached through proxy.py), Executor runs the
plans, and the world's providers (world/mock_provider.py) answer from the
office floor in fixtures/scene.yaml. A case is scored only from what
happened: the plans Pilot dispatched, when the providers ran each call,
Pilot's final answer and whether the task failed.

A result file holds what was recorded and each case's call checks; scoring
the board (weights, levels, speed, routes) is Robonix Hub's.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import re
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent

STATE_FAILED = 2  # PilotEvent status.state
SCORING_VERSION = 2
OUT_OF_CREDIT = re.compile(r"credit balance|insufficient|余额不足", re.I)  # i18n-ok


def cases_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_env(path: str) -> dict:
    env = {}
    for line in Path(path).read_text().splitlines():
        m = re.match(r"\s*(?:export\s+)?([A-Za-z_]\w*)\s*=\s*(.*)", line)
        if m:
            env[m.group(1)] = m.group(2).strip().strip("'\"")
    return env


def probe_formats(base: str, key: str, model: str) -> dict:
    """Which response_format modes the endpoint accepts and honours."""
    msgs = [{"role": "user", "content": 'Reply in JSON: {"answer": <the integer 2+3>}'}]
    schema = {"type": "object", "properties": {"answer": {"type": "integer"}}, "required": ["answer"], "additionalProperties": False}
    modes = {
        "json_schema": {"type": "json_schema", "json_schema": {"name": "probe", "schema": schema, "strict": True}},
        "json_object": {"type": "json_object"},
    }
    out = {}
    for mode, fmt in modes.items():
        req = urllib.request.Request(
            base.rstrip("/") + "/chat/completions",
            data=json.dumps({"model": model, "messages": msgs, "response_format": fmt, "max_tokens": 64}).encode(),
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {key}"},
        )
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                text = (json.loads(r.read())["choices"][0]["message"].get("content") or "").strip()
            try:
                out[mode] = "ok" if json.loads(text).get("answer") == 5 else "accepted_not_enforced"
            except (json.JSONDecodeError, AttributeError):
                out[mode] = "accepted_not_enforced"
        except (urllib.error.HTTPError, OSError, KeyError, ValueError):
            out[mode] = "rejected"
    return out


def probe_vision(base: str, key: str, model: str, images: Path) -> str:
    """`image` if the model names and counts the two cats in a picture (a
    blind model can guess a count alone), `text` if it cannot read it,
    `error:<code>` if the endpoint did not answer. A reasoning model needs
    the output room to think before it answers."""
    data = (images / "two_cats.jpg").read_bytes()
    msgs = [{"role": "user", "content": [
        {"type": "text", "text": "Which animal is in the picture, and how many of them? Answer like: 3 dogs."},
        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(data).decode()}},
    ]}]
    req = urllib.request.Request(base.rstrip("/") + "/chat/completions",
                                 data=json.dumps({"model": model, "messages": msgs, "max_tokens": 4000}).encode(),
                                 headers={"Content-Type": "application/json", "Authorization": f"Bearer {key}"})
    for _ in range(3):
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                text = (json.loads(r.read())["choices"][0]["message"].get("content") or "").lower()
            seen = re.search(r"\b2\b|two|两|二", text) and re.search(r"cat|猫|kitten", text)  # i18n-ok
            return "image" if seen else "text"
        except urllib.error.HTTPError as e:
            if e.code == 429:
                time.sleep(10)
                continue
            # An endpoint that refuses image input answers 400.
            return "text" if e.code == 400 else f"error:{e.code}"
        except (OSError, KeyError, ValueError):
            return "error:network"
    return "error:429"


def fetch_images(urls: dict[str, str], directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for name, url in urls.items():
        dest = directory / name
        if not dest.exists():
            with urllib.request.urlopen(url, timeout=60) as r:
                dest.write_bytes(r.read())


# ── Running a case ─────────────────────────────────────────────────────────


def ask(rbnx: str, server: str, task: str, session: str) -> subprocess.Popen:
    return subprocess.Popen([rbnx, "ask", task, "--json", "--server", server, "--session", session],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)


def events_of(proc: subprocess.Popen, timeout: float) -> list[dict]:
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        out, _ = proc.communicate()
    events = []
    for line in out.splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return events


def lines_of(path: Path, tag: str) -> list[dict]:
    if not path.exists():
        return []
    out = []
    for line in path.read_text().splitlines():
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if rec.get("case") == tag:
            out.append(rec)
    return out


def run_case(a, world: Path, case: dict, tag: str) -> dict:
    w = case.get("world", {})
    (world / "state.json").write_text(json.dumps({
        "case": tag, "image": w.get("image"), "fail": w.get("fail", {}), "nav_s": w.get("nav_s", 2),
        "call_s": 0.8, "images_dir": str(Path(a.images).resolve()),
    }))
    session = str(uuid.uuid4())
    started = time.time()
    first = ask(a.rbnx, a.server, case["task"], session)
    turns: list[list[dict]] = []
    if steer := w.get("steer"):
        time.sleep(steer["after_s"])
        turns.append(events_of(ask(a.rbnx, a.server, steer["task"], session), a.timeout))
    turns.insert(0, events_of(first, a.timeout))
    events = [event for turn in turns for event in turn]
    elapsed = time.time() - started

    plans = [c for e in events if e.get("plan") for c in e["plan"].get("calls", [])]
    planned_calls = []
    for c in plans:
        try:
            args = json.loads(c.get("args_json") or "{}")
        except json.JSONDecodeError:
            args = {}
        planned_calls.append({"contract": c["contract_id"], "provider": c.get("provider_id"), "args": args, "call_id": c.get("call_id")})
    # A dispatched plan can contain leaves that cancellation prevents from running.
    calls = []
    ran = lines_of(world / "calls.jsonl", tag)
    completed = {
        leaf["call_id"]: leaf
        for event in events for result in (event.get("batch_result") or {}).get("results", [])
        if (leaf := result.get("leaf_result"))
    }
    for c in planned_calls:
        leaf = completed.get(c["call_id"])
        if leaf is None:
            continue
        for r in ran:
            execution_id = r.get("execution_id")
            if (execution_id and r["provider"] == c["provider"] and r["contract"] == c["contract"]
                    and any(execution_id in (leaf.get(field) or "") for field in ("output", "error"))):
                calls.append({**c, "args": r["args"], "start": r["start"], "end": r["end"], "ok": r["ok"], "executed": True})
                break
        else:
            # Executor builtins have leaf results but no mock-provider timing log.
            if c["contract"].startswith("robonix/system/executor/builtin/"):
                calls.append({**c, "ok": leaf["success"], "executed": True})
    calls.sort(key=lambda c: c.get("start", float("inf")))
    vlm = [{k: v for k, v in r.items() if k != "case"} for r in lines_of(world / "vlm.jsonl", tag)]
    finals = [e["final_text"] for e in events if e.get("final_text")]
    return {
        "calls": calls,
        "planned_calls": planned_calls,
        "finished": all(any(e.get("event_kind") == 4 for e in turn) for turn in turns),
        "final": finals[-1] if finals else "",
        "failed": any((e.get("status") or {}).get("state") == STATE_FAILED for e in events),
        "vlm": vlm,
        "elapsed_s": round(elapsed, 1),
    }


# ── Scoring a case ─────────────────────────────────────────────────────────


def get_path(value, dotted: str):
    for part in dotted.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def matches(actual, want) -> bool:
    if isinstance(want, dict) and "approx" in want:
        try:
            return abs(float(actual) - float(want["approx"])) <= want.get("tol", 0.05)
        except (TypeError, ValueError):
            return False
    if isinstance(want, dict) and "contains" in want:
        return isinstance(actual, str) and any(s.lower() in actual.lower() for s in want["contains"])
    if isinstance(want, dict) and "one_of" in want:
        return str(actual) in {str(x) for x in want["one_of"]}
    return str(actual) == str(want)


def fits(call: dict, want: dict) -> bool:
    return all(matches(get_path(call["args"], k), v) for k, v in want.items())


def is_(contract: str, entry) -> bool:
    return contract in entry if isinstance(entry, list) else contract == entry


def check(case: dict, run: dict) -> list[str]:
    exp, fails = case.get("expect", {}), []
    calls = sorted(
        (c for c in run["calls"] if c.get("executed") is True
         or (c.get("start") is not None and c.get("end") is not None)),
        key=lambda c: c.get("start") if c.get("start") is not None else float("inf"),
    )
    names = [c["contract"] for c in calls]
    of = lambda entry: [c for c in calls if is_(c["contract"], entry)]  # noqa: E731
    if run["failed"]:
        fails.append("Pilot reported the task FAILED")
    if not run.get("finished", True):
        fails.append("Pilot did not finish every submitted turn")
    # Pilot's own words when no reply of the model gave a valid plan.
    if run["final"].startswith("I couldn't produce a valid robot plan"):
        fails.append("Pilot got no valid plan from the model")
    if exp.get("no_calls_at_all") and calls:
        fails.append(f"expected no calls, got {names}")
    if exp.get("answered") and not run["final"].strip():
        fails.append("no final answer")
    for entry in exp.get("calls", []):
        if not of(entry):
            fails.append(f"missing call {entry}")
    for entry in exp.get("no_calls", []):
        if of(entry):
            fails.append(f"forbidden call {entry}")
    for cid, n in exp.get("count", {}).items():
        if len(of(cid)) != n:
            fails.append(f"{cid} called {len(of(cid))} times, expected {n}")
    for cid, n in exp.get("at_least", {}).items():
        if len(of(cid)) < n:
            fails.append(f"{cid} called {len(of(cid))} times, expected at least {n}")
    for cid, n in exp.get("at_most", {}).items():
        if len(of(cid)) > n:
            fails.append(f"{cid} called {len(of(cid))} times, expected at most {n}")
    for order in exp.get("orders", []):
        if any(c.get("start") is None for c in calls if any(is_(c["contract"], entry) for entry in order)):
            fails.append(f"order: missing execution timing for {order}")
            continue
        it = iter(names)
        if not all(any(is_(n, want) for n in it) for want in order):
            fails.append(f"order: wanted {order}, ran {names}")
    if "overlap" in exp:
        # Run side by side: each entry's first call overlaps every other's.
        firsts = [next(iter(of(e)), None) for e in exp["overlap"]]
        if any(c is None or c.get("start") is None or c.get("end") is None for c in firsts):
            fails.append("overlap: a call is missing")
        elif max(c["start"] for c in firsts) >= min(c["end"] for c in firsts):
            fails.append("overlap: the calls ran one after another")
    for cid, want in exp.get("args", {}).items():
        if of(cid) and not any(fits(c, want) for c in of(cid)):
            fails.append(f"no {cid} call with {want}")
        if not of(cid):
            fails.append(f"missing call {cid}")
    for cid, wants in exp.get("args_seq", {}).items():
        got = of(cid)
        for i, want in enumerate(wants):
            if i >= len(got) or not fits(got[i], want):
                fails.append(f"{cid} #{i + 1} does not have {want}")
    for cid, bad in exp.get("args_not", {}).items():
        if any(fits(c, bad) for c in of(cid)):
            fails.append(f"{cid} called with {bad}")
    return fails


def check_deploy(a) -> None:
    """Refuse to run on anything but this bench's proxy and world: tasks sent
    to another Robonix on the same ports would drive someone else's robot."""
    vlm_url = yaml.safe_load((Path(a.deploy) / "robonix_manifest.yaml").read_text())["system"]["pilot"]["vlm"]["upstream"]
    try:
        with urllib.request.urlopen(vlm_url.rstrip("/") + "/models", timeout=10) as r:
            ours = json.loads(r.read())["data"][0].get("owned_by") == "robonix-model-bench"
    except (OSError, ValueError, KeyError, IndexError):
        ours = False
    if not ours:
        sys.exit(f"{vlm_url} is not the bench's proxy; start proxy.py for this deployment")
    probe = "import json; from robonix_api import ATLAS; print(json.dumps(sorted(p.id for p in ATLAS.query())))"
    # Atlas stubs come from a world package's codegen.
    env = {**os.environ, "ROBONIX_ATLAS": a.server, "PYTHONPATH": str(Path(a.deploy) / "packages/scene/rbnx-build/codegen/proto_gen")}
    try:
        out = subprocess.run([str(HERE / ".venv/bin/python"), "-c", probe], capture_output=True, text=True, timeout=30, env=env).stdout
        providers = set(json.loads(out))
    except (OSError, ValueError, subprocess.SubprocessError):
        providers = set()
    missing = set(json.loads((HERE / "fixtures/catalog.json").read_text())) - providers
    if missing:
        sys.exit(f"Atlas at {a.server} is not a bench deployment (missing {sorted(missing)})")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--env-file", required=True, help="KEY=VALUE file with VLM_BASE_URL and VLM_API_KEY")
    ap.add_argument("--model", action="append", required=True, help="model id, or id=vendor/name to label it; repeatable")
    ap.add_argument("--deploy", default=str(HERE / ".deploy"), help="the booted bench deployment (world/make_deploy.py)")
    ap.add_argument("--server", default="127.0.0.1:52051", help="its Atlas")
    ap.add_argument("--rbnx", default=os.environ.get("RBNX_BIN", "rbnx"))
    ap.add_argument("--cases", default=str(HERE / "cases.yaml"))
    ap.add_argument("--images", default=str(HERE / ".images"))
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--timeout", type=float, default=300, help="seconds per case")
    ap.add_argument("--only", help="regex over case ids")
    ap.add_argument("--out", default=str(HERE / "results"))
    a = ap.parse_args()
    git = lambda *args: subprocess.run(["git", *args], capture_output=True, text=True, cwd=HERE).stdout.strip()  # noqa: E731

    env = read_env(a.env_file)
    base, key = env["VLM_BASE_URL"], env["VLM_API_KEY"]
    host = urllib.parse.urlparse(base).netloc
    spec = yaml.safe_load(Path(a.cases).read_text())
    fetch_images(spec.get("images", {}), Path(a.images))
    cases = [c for c in spec["cases"] if not a.only or re.search(a.only, c["id"])]
    world = Path(a.deploy) / "world"
    check_deploy(a)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    for model_spec in a.model:
        model, _, label = model_spec.partition("=")
        label = label or model
        print(f"== {label} ({model} @ {host})", flush=True)
        # Results are written after every case, and a rerun resumes: a run
        # stopped halfway is not paid for twice.
        path = out / (re.sub(r"[^A-Za-z0-9._-]", "_", f"{label}@{host}") + ".json")
        previous = json.loads(path.read_text()) if path.exists() else None
        if previous is not None and previous.get("meta", {}).get("scoring_version") != SCORING_VERSION:
            sys.exit(f"{path}: results use an older scoring version; preserve them and choose a new --out directory")
        results = previous["results"] if previous is not None else []
        done = {(r["id"], r["rep"]) for r in results}
        (world / "target.json").write_text(json.dumps({"env_file": str(Path(a.env_file).resolve()), "model": model}))
        vision = probe_vision(base, key, model, Path(a.images))
        if vision.startswith("error"):
            sys.exit(f"{model}: the endpoint did not answer the vision probe ({vision}); not running")
        meta = {"model": label, "endpoint_model": model, "host": host, "scoring_version": SCORING_VERSION, "formats": probe_formats(base, key, model), "modalities": ["text", "image"] if vision == "image" else ["text"],
                "robonix_commit": git("rev-parse", "HEAD") or None, "cases_sha256": cases_sha256(Path(a.cases)),
                "date": time.strftime("%Y-%m-%d")}
        for case in cases:
            for rep in range(a.repeat):
                if (case["id"], rep) in done:
                    continue
                # A relay that hangs, rate-limits or errors says nothing
                # about the model: run the case again, up to twice.
                for attempt in range(3):
                    run = run_case(a, world, case, f"{label}@{host}|{case['id']}|{rep}|{attempt}")
                    route_failed = any(v["status"] in (0, 429, 500, 502, 503, 504) for v in run["vlm"]) or (not run["vlm"] and run["failed"])
                    if not route_failed and run["elapsed_s"] < a.timeout:
                        break
                if any(v["status"] == 402 or OUT_OF_CREDIT.search(v.get("error", "")) for v in run["vlm"]):
                    sys.exit(f"{model}: the account behind {host} is out of credit; stopped before {case['id']} was recorded")
                fails = check(case, run)
                results.append({
                    "id": case["id"], "rep": rep, "fails": fails, "pilot_failed": run["failed"], "elapsed_s": run["elapsed_s"],
                    "attempts": attempt + 1, "calls": [{k: c.get(k) for k in ("contract", "args", "start", "end", "ok", "executed")} for c in run["calls"]],
                    "planned_calls": run["planned_calls"], "finished": run["finished"],
                    "final": run["final"], "vlm": run["vlm"],
                })
                print(f"   {'FAIL' if fails else 'JUDGE' if 'judge' in case else 'PASS'} {case['id']:<28} {run['elapsed_s']:>6.1f}s {'; '.join(fails)[:160]}", flush=True)
                path.write_text(json.dumps({"meta": meta, "results": results}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
