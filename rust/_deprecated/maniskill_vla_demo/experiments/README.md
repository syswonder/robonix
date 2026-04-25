# Pilot-orchestrated LIBERO SR eval

## On-disk layout (server)

```
~/robonix_eval/
  LIBERO/                # cloned repo + .venv-libero (py3.10, numpy 1.22)
  runs/                  # one subdir per eval invocation, with videos + report.md
    pilot_20260420_<HHMMSS>/
      summary.json
      report.md
      videos/task_XX/episode_YYYY.mp4
      plots/task_XX_episode_YYYY.png
      resource.csv
      pilot_prompts.log
~/models/openvla-libero-spatial/   # 15GB OpenVLA fine-tune weights
~/robonix_upstream/rust/examples/packages/maniskill_vla_demo/
  experiments/           # THIS directory — eval harness code
  maniskill_vla_demo/vla_node.py
  skills/{pick,place}/SKILL.md
```



End-to-end test of the Robonix planning layer:

```
LIBERO task description
      │
      ▼
Pilot (VLM reads skills/*/SKILL.md, emits JSON plan)
      │   plan = [{"skill":"pick","args":{...}}, {"skill":"place","args":{...}}]
      ▼
Executor (walks the plan, calls each node via the vla_node MCP)
      │
      ▼
vla_node (pick/place tools → OpenVLA-LIBERO closed-loop policy)
      │
      ▼
libero_env_node sidecar (gRPC bridge to LIBERO sim, records one mp4 per episode)
```

One `episode` = one complete plan execution from a fixed LIBERO init state.
The resulting mp4 video is _one continuous recording_ that spans every skill
node in the plan; plot annotations mark which span corresponds to which skill.

## Files

| file | purpose |
|---|---|
| `libero_env_node.py` | LIBERO gRPC sidecar. Speaks the same proto as ManiSkill `env_node`. Records per-episode mp4 when `LIBERO_RECORD_DIR` is set. |
| `mini_pilot_eval.py` | The eval driver. Talks to VLM, builds plan, walks it, records artifacts. |
| `resource_monitor.py` | nvidia-smi + /proc sampler. Runs for the whole eval. |
| `pilot_report_gen.py` | Post-hoc markdown generator with embedded plots and links to videos. |

## Dependencies

The LIBERO sidecar needs its **own** Python 3.10 venv (LIBERO pins
`numpy==1.22` + `gym==0.25` which conflict with the OpenVLA-transformers venv).
Point `LIBERO_VENV_PY` at the right interpreter:

```
export LIBERO_VENV_PY=/tmp/LIBERO/.venv-libero/bin/python3
```

`mini_pilot_eval.py` itself runs inside the maniskill_vla_demo .venv
(`transformers==4.40.1` + torch + mcp + httpx).

## VLM credentials

Put an `.env`-style file anywhere (default: `~/robonix_upstream/rust/examples/.env`)
with:

```
VLM_API_KEY=sk-...
VLM_BASE_URL=https://api.ofox.ai/v1
VLM_MODEL=gpt-5.4-mini
```

Pilot uses an OpenAI-compatible `chat/completions` endpoint.

## Running

```bash
# vla_node (+ atlas) already running, MCP on some port 4XXXX
# LIBERO env runs as a sidecar, port 50062
# python inside the OpenVLA venv:

python experiments/mini_pilot_eval.py \
  --mcp-url http://127.0.0.1:<VLA_MCP_PORT>/mcp \
  --pkg-root $(pwd) \
  --suite libero_spatial \
  --tasks 2 --episodes 2 \
  --out-dir /tmp/robonix_pilot_eval

python experiments/pilot_report_gen.py --eval-dir /tmp/robonix_pilot_eval
# → /tmp/robonix_pilot_eval/report.md
```
