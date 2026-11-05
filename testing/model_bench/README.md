<!-- SPDX-License-Identifier: MulanPSL-2.0 -->
# Pilot model microbench

Runs language models as Robonix's planner on a running Robonix whose world is mocked. Atlas, Executor, Pilot and Soma run as deployed (`rbnx boot`); Pilot plans with the model under test; Executor runs the plans; the world's providers answer from a fixed office floor. Executed calls are checked by program, and Pilot's answer is judged by a fixed judge model. Following SkVM (SOSP '26, arXiv 2604.03088), cases are grouped by capability (`dim`) and level, so a model gets a profile rather than a single number.

This directory holds the bench only. A result file records each case's calls, answer, verdicts and model requests; Robonix Hub scores and ranks the files submitted to it.

## Pieces

| Path | Role |
|---|---|
| `world/make_deploy.py` | Writes the deployment: Atlas, Soma (the Webots TIAGo's `soma.yaml`), Executor, Pilot with its VLM pointed at the proxy, and one mock package per provider |
| `world/mock_provider.py` | One provider, booted by `rbnx boot` like any package. It registers the contracts the Webots deployment had for it (`fixtures/catalog.json`: same ids, argument schemas and descriptions) and answers from the world. Every call is logged with its arguments and when it ran |
| `world/scene.py`, `fixtures/scene.yaml` | The office floor (open office, meeting room, pantry; 22 objects, several of each class; `on_top_of`, `under`, `near`), answered in Scene's shapes (`list_objects`, `find`, `list_regions`, `list_relations`, `goal_near`, `goal_region`, ...) |
| `proxy.py` | Pilot's VLM endpoint. Forwards each request, streamed, to the model under test; records the `response_format` Pilot sent, the status, the latency, whether the reply was empty, the token usage and whether it was Pilot's corrective retry |
| `bench.py` | Gives each case to Pilot with `rbnx ask`, then checks executed calls from its events, the world's call log and the proxy's record |
| `judge.py` | Judges Pilot's final answers against each case's assertions |
| `cases.yaml` | The cases and the judge's instructions |

The camera returns the case's image: COCO val2017, fetched at run time into `.images/` and not redistributed. The world can inject a failure into any contract, make navigation take a while, and send a second instruction into the same session while the first task runs (`rbnx ask --session`).

## Checking a case

A case passes when Pilot gave a non-whitespace final answer, did not report the task failed, and finished every submitted turn. Pilot must also get a valid plan from the model (its answer is not its own "I couldn't produce a valid robot plan"), every `expect` check must hold, and every `judge` assertion must hold. The same completion checks apply to `judge.py`'s summary of saved results; older files without `finished` remain eligible.

`expect` is checked against plan leaves matched to the mock providers' execution log, ordered by their actual start times. Each mock response carries an opaque `_bench_execution_id`; an injected failure includes the same identifier in its error. The executor's leaf result links that invocation to its plan call, so Pilot's own context reads cannot be mistaken for planned executions. Executor builtin leaf results establish an attempt without mock-provider timing; they cannot satisfy ordering or overlap checks. Unexecuted plan leaves do not count. Provider calls that returned an error still count as attempts, so failure-handling cases can check them. Each submitted turn must reach Pilot's final-text event, including a turn with an empty answer.

| Key | Holds when |
|---|---|
| `calls`, `no_calls`, `no_calls_at_all` | each listed contract ran (a list entry: any one of them), none of these ran, nothing ran |
| `count`, `at_least`, `at_most` | how many times a contract ran |
| `orders` | these contracts started in this order; others may come between |
| `overlap` | these calls ran side by side in the world (their run times overlap) |
| `args`, `args_seq`, `args_not` | some call, the i-th call, no call of a contract has these argument values (`approx` with `tol`, `contains`, `one_of`, or exact) |
| `answered` | Pilot gave a final answer (required for every case) |

`judge` covers what the answer says, which no keyword list checks reliably. As in tau2-bench's `nl_assertions`, the case gives the ground truth (`facts`: what the scene, the camera frame or an injected failure really was) and yes/no statements about the answer (`assert`). `judge.py` asks a fixed judge model, at temperature 0, to decide each statement from the task, the facts and Pilot's answer. The judge model is recorded with the results.

A case is run again, up to twice, when the route to the model failed (a timeout, a rate limit, a server error): that says nothing about the model.

Results record `meta.scoring_version: 2`, executed `calls`, diagnostic `planned_calls`, and whether every submitted turn emitted a final-text event. That event does not establish physical task completion. The runner refuses to append to older result files; keep those files and use a new output directory. Scores produced by the earlier planned-call checks must be revalidated before comparison or publication.

The scoring regression tests need no service stack or model API:

```sh
python3 -m unittest discover -s testing -p 'test_model_bench.py' -v
```

## Running

```sh
# once: the world's venv and the deployment
uv venv testing/model_bench/.venv
uv pip install -p testing/model_bench/.venv -e pylib/robonix-api pyyaml grpcio-tools
python3 testing/model_bench/world/make_deploy.py          # --slot N for a second one on other ports

# boot it, and the proxy
(cd testing/model_bench/.deploy && rbnx boot) &
python3 testing/model_bench/proxy.py --world testing/model_bench/.deploy/world &

# a KEY=VALUE file with VLM_BASE_URL and VLM_API_KEY (OpenAI-compatible)
python3 testing/model_bench/bench.py --env-file api.env --model qwen3.8-max=qwen/qwen3.8-max
# then judge the answers with the judge model the board names
python3 testing/model_bench/judge.py --env-file judge.env --model openai/gpt-5.4 testing/model_bench/results/*.json
```

`--model id=vendor/name` names a model the way the board does, so runs over different endpoints line up. A result file is named after the model and the endpoint's host, and records the Robonix commit, the cases' hash and which `response_format` modes the endpoint honours; `judge.py` packs the files it judged into `results/model-bench-<time>.zip`, which is what is submitted on Robonix Hub.

## Several models at once

Each model needs its own deployment, since the proxy routes a deployment's Pilot to one model at a time. `make_deploy.py --slot N` writes `.deploy-N` with every port shifted by `100 * N`: Atlas on `52051 + 100 * N`, the proxy on `52442 + 100 * N`. Boot each slot and its proxy, then run one `bench.py` per slot with `--deploy testing/model_bench/.deploy-N --server 127.0.0.1:<its Atlas>`. A slow model's cases can be split across two slots with `--only` and separate `--out` directories, and both files submitted.

Keep one request in flight per provider and slot. Results are written after every case and a rerun skips the cases a file already has, so a stopped run resumes without paying again.

## Cost

A case takes about five Pilot rounds of 10–30k prompt tokens. Before a run, multiply that by the cases and the provider's prices, and check the account balance with a one-token request; stop at the first `402` or balance error.

- Claude caches a prompt only up to an explicit `cache_control` breakpoint. Pilot adds them for Claude models; without them every round is billed at the full input price.
- Some models fall into repeating one sentence under Pilot's `json_schema` until they hit the output limit (Gemini 3.1 Pro and Grok 4.7 did, at 65k and 128k tokens per request). Watch the proxy log (`world/vlm.jsonl`, `usage.completion_tokens`) and stop such a run early.
- `usage` is recorded only for answers that arrive. A request Pilot abandons on a timeout is still billed, so the provider's own usage page is the figure to trust.
