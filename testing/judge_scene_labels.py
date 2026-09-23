#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Judge whether an observed label names the same kind of object as a truth label.

The benchmark matches predictions to ground truth by geometry and then asks
whether the label is right. Comparing the strings decides that a monitor called
"tv", a couch called "sofa" or a potted plant called "vase" is wrong, which is
a statement about vocabulary, not about recognition. `evaluate_webots_scene.py`
accepts a judgments file instead (`--label-judgments-file`); this writes one.

    export VLM_API_KEY=... VLM_BASE_URL=https://host/v1
    python3 testing/judge_scene_labels.py --evaluation out/evaluation.json \
        --output out/label_judgments.json

Talks to any OpenAI-compatible chat endpoint (the same `VLM_API_KEY` /
`VLM_BASE_URL` the scene service's VLM backend uses), with the standard library
only. Every (expected, observed) pair that appears anywhere in the evaluation --
matched targets, duplicates, ghosts -- is judged once. The verdict is strict on
purpose: two names are equivalent only when a person pointing at the object
would accept either, so "shelf" for a cabinet is not, and neither is "keyboard"
for a monitor. Pairs go to the model as bare category names; nothing about the
scene leaves the machine.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_MODEL = "anthropic/claude-opus-5"

SYSTEM = """You judge object-category labels produced by a robot's perception \
system against ground-truth labels from a simulated indoor scene.

For each pair decide whether the observed label names the SAME KIND of object \
as the expected label -- such that a person pointing at the real object would \
accept either word for it. Synonyms and near-synonyms are equivalent \
(couch/sofa, monitor/tv/screen, potted_plant/plant/houseplant, lamp/light). \
A more general or more specific word for the same object is equivalent when \
the object plainly is one (table/desk). Different objects that merely sit \
together, look alike, or share a function are NOT equivalent (cabinet/shelf, \
keyboard/monitor, window/mirror, chair/stool, table/cabinet).

Answer with one JSON object per line and nothing else:
{"expected_label": ..., "observed_label": ..., "equivalent": true|false, "reason": "<one sentence>"}"""


def _pairs_from_evaluation(evaluation: dict) -> list[tuple[str, str]]:
    """Every distinct (expected, observed) pair the evaluation compared."""
    world = evaluation if "tp" in evaluation else (
        (evaluation.get("worlds") or {}).get(evaluation.get("world_id") or "") or
        next(iter((evaluation.get("worlds") or {}).values()), {}))
    pairs: set[tuple[str, str]] = set()
    for entry in world.get("per_target") or ():
        a, b = entry.get("expected_label"), entry.get("observed_label")
        if a and b:
            pairs.add((str(a).strip().lower(), str(b).strip().lower()))
    for kind in ("duplicates", "ghosts"):
        for entry in world.get(kind) or ():
            a, b = entry.get("nearest_truth_label"), entry.get("observed_label")
            if a and b:
                pairs.add((str(a).strip().lower(), str(b).strip().lower()))
    return sorted(p for p in pairs if p[0] != p[1])


def _chat(base_url: str, api_key: str, model: str, system: str, user: str) -> str:
    """One chat completion over the OpenAI-compatible wire format."""
    body = json.dumps({
        "model": model,
        "temperature": 0,
        "messages": [{"role": "system", "content": system},
                     {"role": "user", "content": user}],
    }).encode("utf-8")
    req = urllib.request.Request(
        base_url.rstrip("/") + "/chat/completions", data=body, method="POST",
        headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=180) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        raise SystemExit(f"judge endpoint returned {e.code}: {e.read().decode('utf-8', 'replace')[:300]}")
    try:
        return payload["choices"][0]["message"]["content"]
    except (KeyError, IndexError, TypeError):
        raise SystemExit(f"unexpected reply shape from the judge endpoint: {str(payload)[:300]}")


def _judge(pairs: list[tuple[str, str]], model: str) -> list[dict]:
    """Ask the model once for the whole list; parse one JSON object per line."""
    api_key = os.environ.get("VLM_API_KEY", "").strip()
    base_url = os.environ.get("VLM_BASE_URL", "").strip()
    if not api_key or not base_url:
        raise SystemExit("set VLM_API_KEY and VLM_BASE_URL (an OpenAI-compatible endpoint)")
    listing = "\n".join(f"expected={a}  observed={b}" for a, b in pairs)
    text = _chat(base_url, api_key, model, SYSTEM, listing)

    wanted: dict[tuple[str, str], dict | None] = {p: None for p in pairs}
    for line in text.splitlines():
        line = line.strip().rstrip(",")
        if not line.startswith("{"):
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        key = (str(row.get("expected_label", "")).strip().lower(),
               str(row.get("observed_label", "")).strip().lower())
        if key in wanted and isinstance(row.get("equivalent"), bool) and row.get("reason"):
            wanted[key] = {"expected_label": key[0], "observed_label": key[1],
                           "equivalent": row["equivalent"], "reason": str(row["reason"]).strip()}
    missing = [p for p, v in wanted.items() if v is None]
    if missing:
        raise SystemExit(f"the judge returned no verdict for {len(missing)} pair(s): {missing[:5]}")
    return [wanted[p] for p in pairs]  # type: ignore[misc]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--evaluation", type=Path, required=True,
                    help="evaluation.json from evaluate_webots_scene.py")
    ap.add_argument("--output", type=Path, required=True,
                    help="where to write the judgments file the evaluator reads")
    ap.add_argument("--model", default=os.environ.get("VLM_JUDGE_MODEL", DEFAULT_MODEL))
    ap.add_argument("--extra-pair", action="append", default=[], metavar="EXPECTED:OBSERVED",
                    help="judge this pair too (repeatable)")
    args = ap.parse_args()

    evaluation = json.loads(args.evaluation.read_text(encoding="utf-8"))
    pairs = _pairs_from_evaluation(evaluation)
    for spec in args.extra_pair:
        a, _, b = spec.partition(":")
        if a and b:
            pairs.append((a.strip().lower(), b.strip().lower()))
    pairs = sorted(set(pairs))
    if not pairs:
        print("no label pairs to judge", file=sys.stderr)
        return 1

    judgments = _judge(pairs, args.model)
    payload = {
        "schema_version": 1,
        "judge": {"provider": "openai-compatible", "model": args.model},
        "judgments": judgments,
    }
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                           encoding="utf-8")
    equivalent = sum(1 for j in judgments if j["equivalent"])
    print(f"judged {len(judgments)} pairs: {equivalent} equivalent, "
          f"{len(judgments) - equivalent} distinct -> {args.output}")
    for j in judgments:
        mark = "=" if j["equivalent"] else "x"
        print(f"  {mark} {j['expected_label']:<14} {j['observed_label']:<14} {j['reason']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
