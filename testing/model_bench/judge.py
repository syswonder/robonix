# SPDX-License-Identifier: MulanPSL-2.0
"""Judge Pilot's final answers in bench results.

What Pilot dispatched is checked by bench.py. What its answer says is judged
here, the way tau2-bench judges its nl_assertions: a fixed judge model gets
the task, the case's ground truth (`judge.facts` in cases.yaml) and Pilot's
answer, and decides each assertion yes or no. A case passes when its calls
check out and every assertion holds.

The judge's instructions are in cases.yaml (`judge`). Verdicts are stored
in the result files with the judge model, so a rerun only judges answers it
has not seen. The judge's token usage is printed, and the judged files are
packed into one archive to submit on Robonix Hub.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

import yaml

from bench import HERE, OUT_OF_CREDIT, logs_for_results, read_env

if __package__:
    from .bundle import scrub, write_bundle
else:
    from bundle import scrub, write_bundle


def judge_key(spec: dict, model: str, case: dict) -> str:
    """What a verdict depends on besides the answer: a change to any of it judges the answer again."""
    return hashlib.sha256(json.dumps([model, spec["judge"], case["judge"]], ensure_ascii=False).encode()).hexdigest()[:16]


def ask_judge(env: dict, spec: dict, model: str, case: dict, answer: str) -> tuple[list[dict], dict]:
    j = spec["judge"]
    user = j["user"].rstrip().format(task=case["task"], facts=case["judge"]["facts"], answer=answer or j["empty_answer"],
                                     asserts="\n".join(f"{i + 1}. {a}" for i, a in enumerate(case["judge"]["assert"])))
    body = {"model": model, "temperature": 0, "response_format": {"type": "json_object"},
            "messages": [{"role": "system", "content": j["system"].rstrip()}, {"role": "user", "content": user}]}
    req = urllib.request.Request(f"{env['VLM_BASE_URL'].rstrip('/')}/chat/completions", data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json", "Authorization": f"Bearer {env['VLM_API_KEY']}"})
    for attempt in range(3):
        try:
            with urllib.request.urlopen(req, timeout=120) as r:
                reply = json.loads(r.read())
            break
        except urllib.error.HTTPError as e:
            text = e.read().decode(errors="replace")
            if e.code == 402 or OUT_OF_CREDIT.search(text):
                sys.exit(f"the judge's account is out of credit: {text[:200]}")
            if e.code not in (429, 500, 502, 503, 504) or attempt == 2:
                raise
        except (urllib.error.URLError, TimeoutError):
            if attempt == 2:
                raise
    verdicts = json.loads(reply["choices"][0]["message"]["content"])["verdicts"]
    if len(verdicts) != len(case["judge"]["assert"]):
        raise ValueError(f"{case['id']}: {len(verdicts)} verdicts for {len(case['judge']['assert'])} assertions")
    return [{"assert": a, "met": v["met"] is True, "evidence": v.get("evidence", ""), "reason": v.get("reason", "")}
            for a, v in zip(case["judge"]["assert"], verdicts)], reply.get("usage") or {}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("files", nargs="+", help="bench result files")
    ap.add_argument("--env-file", required=True, help="KEY=VALUE file with VLM_BASE_URL and VLM_API_KEY for the judge")
    ap.add_argument("--model", required=True, help="the judge model the board names")
    ap.add_argument("--cases", default=str(HERE / "cases.yaml"))
    a = ap.parse_args()
    env = read_env(a.env_file)
    spec = yaml.safe_load(Path(a.cases).read_text())
    cases = {c["id"]: c for c in spec["cases"]}
    used = {"prompt_tokens": 0, "completion_tokens": 0, "calls": 0}
    for f in map(Path, a.files):
        data = json.loads(f.read_text())
        provenance = data["meta"].get("provenance")
        if provenance is not None:
            provenance["judge_model"] = a.model
            data["meta"]["provenance"] = scrub(provenance, [env["VLM_API_KEY"]])
        data["meta"]["judge_model"] = a.model
        for r in data["results"]:
            case = cases[r["id"]]
            if "judge" not in case:
                continue
            key = judge_key(spec, a.model, case)
            if r.get("verdicts") and r.get("judge_key") == key:
                continue
            if r["final"].strip():
                r["verdicts"], usage = ask_judge(env, spec, a.model, case, r["final"])
                used["calls"] += 1
                for k in ("prompt_tokens", "completion_tokens"):
                    used[k] += usage.get(k, 0)
            else:
                r["verdicts"] = [{"assert": x, "met": False, "reason": "no answer"} for x in case["judge"]["assert"]]
            r["judge_key"] = key
            data["meta"]["judge_model"] = a.model
            f.write_text(json.dumps(data, ensure_ascii=False, indent=1))
        f.write_text(json.dumps(data, ensure_ascii=False, indent=1))
        rs = data["results"]
        passed = sum(not r["fails"] and all(v["met"] for v in r.get("verdicts") or []) for r in rs)
        print(f"{f.name}: {passed}/{len(rs)} cases pass", flush=True)
    print(f"judge {a.model}: {used['calls']} calls, {used['prompt_tokens']} prompt + {used['completion_tokens']} completion tokens")
    # What is uploaded to Robonix Hub: the judged files, as one archive.
    archive = Path(a.files[0]).parent / f"model-bench-{time.strftime('%Y%m%d-%H%M%S')}.zip"
    files = list(dict.fromkeys(map(Path, a.files)))
    secrets = [value for key, value in env.items() if key.upper().endswith(("_KEY", "_TOKEN")) or "SECRET" in key.upper()]
    write_bundle(archive.resolve(), files, logs_for_results(files), secrets)
    print(f"submit {archive} on Robonix Hub")


if __name__ == "__main__":
    main()
