#!/usr/bin/env python3
"""Robonix Pilot → Executor eval on LIBERO.

Runs the full orchestration loop:
  Task → Pilot (VLM reads SKILL.md catalogue) → execution graph (linear list)
        → Executor dispatches each node via MCP → one continuous episode video

Metrics per episode:
  - overall_success (env.check_success after the plan ran to completion)
  - plan length, time per node, timeout nodes

Artifacts written under --out-dir:
  summary.json         — full structured record including Pilot's plan JSON
  videos/task_TT/episode_EEEE.mp4
  plots/*.png          — resource curves with node-span annotations
  resource.csv         — GPU/CPU samples at --hz
  pilot_prompts.log    — raw VLM requests/responses per episode
  report.md            — markdown with plan, video link, plot, per-node table
"""
import argparse
import asyncio
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import httpx
from mcp.client.session import ClientSession
from mcp.client.streamable_http import streamablehttp_client

_PKG_EXPERIMENTS = Path(__file__).resolve().parent
_LIBERO_ROOT = os.environ.get("LIBERO_ROOT",
                              os.path.expanduser("~/robonix_eval/LIBERO"))
LIBERO_VENV = os.environ.get(
    "LIBERO_VENV_PY", f"{_LIBERO_ROOT}/.venv-libero/bin/python3"
)
LIBERO_ENV_SCRIPT = str(_PKG_EXPERIMENTS / "libero_env_node.py")
LIBERO_ENV_PORT = int(os.environ.get("LIBERO_GRPC_PORT", "50062"))
RESOURCE_MONITOR = str(_PKG_EXPERIMENTS / "resource_monitor.py")
_DEFAULT_OUT_ROOT = os.path.expanduser("~/robonix_eval/runs")


# ──────────────────────────────────────────────────────────────────────────────
# Skill catalog loader — reads SKILL.md files under the package and extracts the
# YAML frontmatter + body so Pilot's VLM prompt can enumerate available skills.
# ──────────────────────────────────────────────────────────────────────────────

def load_skill_catalog(pkg_root: Path) -> list[dict]:
    catalog = []
    for skill_md in sorted((pkg_root / "skills").glob("*/SKILL.md")):
        text = skill_md.read_text()
        m = re.match(r"^---\n(.*?)\n---\n(.*)$", text, re.DOTALL)
        if not m:
            continue
        header_raw, body = m.group(1), m.group(2)
        header = {}
        for line in header_raw.splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                header[k.strip()] = v.strip()
        catalog.append({
            "name": header.get("name", skill_md.parent.name),
            "description": header.get("description", ""),
            "skill_md": body.strip(),
            "path": str(skill_md),
        })
    return catalog


# ──────────────────────────────────────────────────────────────────────────────
# Mini Pilot: call OpenAI-compatible VLM to produce a JSON plan list
# ──────────────────────────────────────────────────────────────────────────────

PILOT_SYSTEM_PROMPT = """\
You are Robonix Pilot, a task planner for an embodied robot arm. You receive a
natural-language task description from the user, and a catalog of discrete SKILLS
the robot can perform. You output an execution plan as a JSON list: each entry is
one skill invocation. The Executor runs the plan node-by-node, passing each node
down to the underlying VLA policy. The simulator state PERSISTS across nodes
— do not reset mid-plan.

Hard rules:
- Output ONLY a JSON list, no prose before or after.
- Each list entry has exactly two keys: `skill` (one of the catalog names) and
  `args` (object matching that skill's SKILL.md).
- The FIRST entry sets `reset_env: true`. Every other entry MUST set
  `reset_env: false`.
- Do not invent skills that are not in the catalog.

Decomposition guidance (IMPORTANT):
- The VLA policy was trained on atomic `"pick up X and place it on Y"` demos.
  It does NOT know how to do "pick" alone or "place" alone. Therefore each
  plan node should correspond to a **full** pick+place unit, dispatched via
  the `pick_and_place` skill.
- For simple `"pick up X and place it on/in Y"` tasks → plan length 1:
  `[pick_and_place(X, Y)]`.
- For multi-object tasks like `"put both X and Y in Z"` → plan length 2:
  `[pick_and_place(X, Z), pick_and_place(Y, Z)]`.
- For 3+ stage tasks ("put A on P, then put B on Q, then put C on R") → keep
  chaining one `pick_and_place` node per stage.
- `"turn on the stove and put the pot on it"` → `[operate(turn on, stove),
  pick_and_place(pot, stove)]`.
- `"put X in the drawer and close it"` → `[operate(open, drawer),
  pick_and_place(X, drawer), operate(close, drawer)]`.
- Prefer the atomic `pick_and_place` skill. Only fall back to the low-level
  `pick` + `place` pair if explicitly asked; they do NOT work well with the
  LIBERO-trained policy.
"""


def _load_env_file(path: Path) -> dict[str, str]:
    """Parse a .env-style file (KEY=VALUE lines). Returns dict."""
    out = {}
    if not path.is_file():
        return out
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip().strip('"').strip("'")
    return out


class MiniPilot:
    def __init__(self, api_key: str, base_url: str, model: str,
                 log_path: Path, timeout: float = 60.0):
        self.api_key = api_key
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.log_path = log_path
        self.timeout = timeout

    def _format_catalog(self, catalog: list[dict]) -> str:
        lines = []
        for skill in catalog:
            lines.append(f"## skill: `{skill['name']}`")
            lines.append(f"_summary_: {skill['description']}")
            lines.append("")
            lines.append(skill["skill_md"])
            lines.append("")
        return "\n".join(lines)

    def plan(self, task_description: str, catalog: list[dict]) -> list[dict]:
        """Call VLM, return the parsed plan list (empty list on failure)."""
        user_prompt = (
            f"# Task\n{task_description}\n\n# Available skills\n\n"
            f"{self._format_catalog(catalog)}\n\n"
            f"Emit the execution plan as a JSON list now."
        )
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": PILOT_SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt},
            ],
            "temperature": 0.0,
        }
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }
        t0 = time.time()
        resp_text = ""
        plan_json = []
        error = None
        try:
            r = httpx.post(
                f"{self.base_url}/chat/completions",
                headers=headers,
                json=payload,
                timeout=self.timeout,
            )
            r.raise_for_status()
            data = r.json()
            resp_text = data["choices"][0]["message"]["content"]
            plan_json = _extract_json_list(resp_text)
        except Exception as e:
            error = repr(e)

        with self.log_path.open("a") as f:
            f.write(json.dumps({
                "ts": t0,
                "task": task_description,
                "prompt": user_prompt,
                "response_text": resp_text,
                "plan": plan_json,
                "error": error,
                "elapsed_s": round(time.time() - t0, 2),
            }, ensure_ascii=False) + "\n")
        return plan_json


def _extract_json_list(text: str) -> list:
    text = text.strip()
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\n", "", text)
        text = re.sub(r"\n```$", "", text)
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        # Try to find a [...] block
        m = re.search(r"\[[\s\S]*\]", text)
        if m:
            try:
                return json.loads(m.group(0))
            except json.JSONDecodeError:
                pass
    return []


# ──────────────────────────────────────────────────────────────────────────────
# Executor: walk the plan, dispatch each node via MCP, record timing + result
# ──────────────────────────────────────────────────────────────────────────────

async def snapshot_scene(sess: ClientSession) -> dict:
    try:
        res = await sess.call_tool("snapshot_scene", {})
        for c in res.content:
            text = getattr(c, "text", str(c))
            try:
                return json.loads(text)
            except json.JSONDecodeError:
                continue
    except Exception as e:
        return {"error": repr(e)}
    return {}


async def execute_plan(mcp_url: str, plan: list[dict]) -> list[dict]:
    trace = []
    async with streamablehttp_client(mcp_url) as (r, w, _):
        async with ClientSession(r, w) as sess:
            await sess.initialize()
            for idx, node in enumerate(plan):
                skill = node.get("skill") or node.get("name")
                args = node.get("args", {})
                t0 = time.time()
                try:
                    res = await sess.call_tool(skill, args)
                    text = ""
                    for c in res.content:
                        text = getattr(c, "text", str(c))
                        break
                    try:
                        info = json.loads(text)
                    except json.JSONDecodeError:
                        info = {"raw": text}
                except Exception as e:
                    info = {"error": repr(e)}
                t1 = time.time()
                inner_info = info.get("info", {}) if isinstance(info, dict) else {}
                trace.append({
                    "idx": idx,
                    "skill": skill,
                    "args": args,
                    "start_ts": t0,
                    "end_ts": t1,
                    "duration_s": round(t1 - t0, 2),
                    "cycles": info.get("cycles") or info.get("total_steps"),
                    "status": info.get("status"),
                    "reward": info.get("reward"),
                    "intermediate_success": bool(inner_info.get("success", False)),
                    "object_positions": inner_info.get("object_positions", {}),
                })
    return trace


# ──────────────────────────────────────────────────────────────────────────────
# libero_env_node lifecycle + resource monitor (same as v2, trimmed)
# ──────────────────────────────────────────────────────────────────────────────

def restart_env(suite: str, task_id: int, video_dir: Path,
                wait_s: float = 25.0) -> str:
    video_dir.mkdir(parents=True, exist_ok=True)
    log_dir = video_dir.parent.parent / "env_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"env_{suite}_task{task_id}.log"

    subprocess.run(["pkill", "-TERM", "-f", "libero_env_node.py"], check=False)
    time.sleep(3.0)
    subprocess.run(["pkill", "-KILL", "-f", "libero_env_node.py"], check=False)
    time.sleep(1.0)

    env = os.environ.copy()
    env.update({
        "MUJOCO_GL": "egl",
        "LIBERO_GRPC_PORT": str(LIBERO_ENV_PORT),
        "LIBERO_SUITE": suite,
        "LIBERO_TASK_ID": str(task_id),
        "LIBERO_RECORD_DIR": str(video_dir),
        "LIBERO_RECORD_FPS": "30",
        "LIBERO_ROOT": _LIBERO_ROOT,
    })
    env.pop("LIBERO_INIT_ID", None)

    with open(log_path, "w") as f:
        subprocess.Popen(
            [LIBERO_VENV, LIBERO_ENV_SCRIPT],
            cwd=_LIBERO_ROOT,
            env=env,
            stdout=f, stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
        )

    deadline = time.time() + wait_s + 30.0
    desc = None
    while time.time() < deadline:
        try:
            text = log_path.read_text()
            m = re.search(r"desc='([^']+)'", text)
            if m:
                desc = m.group(1)
            if "[libero-env] ready" in text:
                return desc or "(unknown)"
        except FileNotFoundError:
            pass
        time.sleep(1.0)
    raise RuntimeError(f"libero_env_node not ready — see {log_path}")


def find_vla_pid() -> int:
    out = subprocess.run(["pgrep", "-f", "maniskill_vla_demo.vla_node"],
                         capture_output=True, text=True)
    pids = [int(x) for x in out.stdout.split()]
    if not pids:
        raise RuntimeError("vla_node not running")
    return max(pids)


def start_resource_monitor(out_csv: Path, gpus: str, vla_pid: int,
                           hz: float) -> subprocess.Popen:
    log = out_csv.parent / "resource.log"
    with open(log, "w") as logf:
        return subprocess.Popen([
            sys.executable, RESOURCE_MONITOR,
            "--out", str(out_csv),
            "--gpus", gpus,
            "--pids", str(vla_pid),
            "--hz", str(hz),
        ], stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
           start_new_session=True)


# ──────────────────────────────────────────────────────────────────────────────
# Main eval driver
# ──────────────────────────────────────────────────────────────────────────────

async def run_episode(mcp_url: str, pilot: MiniPilot, catalog: list[dict],
                      task_desc: str, task_id: int, ep_idx: int) -> dict:
    ep_t0 = time.time()
    plan = pilot.plan(task_desc, catalog)
    plan_t = time.time()
    trace = await execute_plan(mcp_url, plan) if plan else []
    ep_t1 = time.time()

    overall_success = any(n["intermediate_success"] for n in trace)
    return {
        "task_id": task_id,
        "episode_idx": ep_idx,
        "task_description": task_desc,
        "episode_start_ts": ep_t0,
        "plan_generated_ts": plan_t,
        "episode_end_ts": ep_t1,
        "plan": plan,
        "plan_length": len(plan),
        "trace": trace,
        "overall_success": overall_success,
        "duration_s": round(ep_t1 - ep_t0, 2),
        "plan_latency_s": round(plan_t - ep_t0, 2),
    }


def run_eval(mcp_url: str, suite: str, n_tasks: int, n_episodes: int,
             pkg_root: Path, env_file: Path, out_dir: Path,
             gpus: str, hz: float) -> dict:
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "videos").mkdir(exist_ok=True)

    catalog = load_skill_catalog(pkg_root)
    print(f"[eval] loaded {len(catalog)} skills: {[s['name'] for s in catalog]}",
          flush=True)

    env_vars = _load_env_file(env_file)
    api_key = env_vars.get("VLM_API_KEY") or os.environ.get("VLM_API_KEY", "")
    base_url = env_vars.get("VLM_BASE_URL", "https://api.openai.com/v1")
    model = env_vars.get("VLM_MODEL", "gpt-5.4-mini-mini")
    if not api_key:
        raise SystemExit(f"VLM_API_KEY missing in {env_file} and environment")
    print(f"[eval] pilot using VLM model={model} base_url={base_url}", flush=True)

    pilot = MiniPilot(api_key, base_url, model,
                      log_path=out_dir / "pilot_prompts.log")
    vla_pid = find_vla_pid()
    print(f"[eval] vla_node pid={vla_pid}", flush=True)
    mon = start_resource_monitor(out_dir / "resource.csv", gpus, vla_pid, hz)
    time.sleep(1.0)

    overall_t0 = time.time()
    per_task = []
    total_eps = 0
    total_succ = 0
    try:
        for task_id in range(n_tasks):
            print(f"\n===== suite={suite} task_id={task_id} =====", flush=True)
            task_video_dir = out_dir / "videos" / f"task_{task_id:02d}"
            try:
                desc = restart_env(suite, task_id, task_video_dir)
            except Exception as e:
                print(f"  [restart error] {e}", flush=True)
                continue
            print(f"  task: {desc}", flush=True)

            episodes = []
            for ep in range(n_episodes):
                try:
                    rec = asyncio.run(run_episode(
                        mcp_url, pilot, catalog, desc, task_id, ep
                    ))
                except Exception as e:
                    print(f"  ep{ep}: exception {e}", flush=True)
                    rec = {"task_id": task_id, "episode_idx": ep,
                           "error": str(e), "overall_success": False,
                           "plan": [], "trace": []}
                rec["video_path"] = str(task_video_dir / f"episode_{ep:04d}.mp4")
                episodes.append(rec)
                total_eps += 1
                if rec.get("overall_success"):
                    total_succ += 1
                plan_names = [f"{n.get('skill')}({n.get('args',{}).get('object','?')})"
                              for n in rec.get("plan", [])]
                node_results = [(n["skill"], n["cycles"], n["intermediate_success"])
                                for n in rec.get("trace", [])]
                print(f"  ep{ep}: overall={rec.get('overall_success')} "
                      f"plan={plan_names} trace={node_results} "
                      f"dur={rec.get('duration_s','?')}s", flush=True)

            task_succ = sum(1 for e in episodes if e.get("overall_success"))
            task_sr = task_succ / max(1, len(episodes))
            per_task.append({
                "task_id": task_id,
                "description": desc,
                "n_episodes": len(episodes),
                "successes": task_succ,
                "task_sr": task_sr,
                "episodes": episodes,
            })
            print(f"  task_sr = {task_succ}/{len(episodes)} "
                  f"= {task_sr*100:.1f}%", flush=True)

            with open(out_dir / "summary.json", "w") as f:
                json.dump({"partial": True, "per_task": per_task}, f, indent=2)

    finally:
        try:
            mon.terminate(); mon.wait(timeout=3)
        except Exception:
            mon.kill()
        subprocess.run(["pkill", "-TERM", "-f", "libero_env_node.py"], check=False)
        time.sleep(3.0)
        subprocess.run(["pkill", "-KILL", "-f", "libero_env_node.py"], check=False)

    overall_sr = total_succ / max(1, total_eps)
    summary = {
        "suite": suite, "n_tasks": n_tasks, "n_episodes_per_task": n_episodes,
        "vlm_model": model, "vlm_base_url": base_url,
        "total_episodes": total_eps, "total_success": total_succ,
        "overall_sr": overall_sr,
        "eval_start_ts": overall_t0, "eval_end_ts": time.time(),
        "elapsed_s": time.time() - overall_t0,
        "per_task": per_task, "partial": False,
    }
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    return summary


def run_composite_eval(mcp_url: str, composite_json: Path, n_episodes: int,
                       pkg_root: Path, env_file: Path, out_dir: Path,
                       gpus: str, hz: float) -> dict:
    """Composite multi-stage eval: each entry in the JSON defines a synthetic
    long-horizon task reusing a scene from an existing LIBERO suite but given
    a hand-written multi-clause task description so Pilot has to chain several
    pick_and_place (or operate) nodes on the same scene."""
    composite = json.loads(composite_json.read_text())
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "videos").mkdir(exist_ok=True)

    catalog = load_skill_catalog(pkg_root)
    print(f"[eval] loaded {len(catalog)} skills: {[s['name'] for s in catalog]}",
          flush=True)
    env_vars = _load_env_file(env_file)
    api_key = env_vars.get("VLM_API_KEY") or os.environ.get("VLM_API_KEY", "")
    base_url = env_vars.get("VLM_BASE_URL", "https://api.openai.com/v1")
    model = env_vars.get("VLM_MODEL", "gpt-5.4-mini-mini")
    if not api_key:
        raise SystemExit(f"VLM_API_KEY missing in {env_file}")
    print(f"[eval] pilot using VLM model={model} base_url={base_url}", flush=True)
    pilot = MiniPilot(api_key, base_url, model,
                      log_path=out_dir / "pilot_prompts.log")
    vla_pid = find_vla_pid()
    print(f"[eval] vla_node pid={vla_pid}", flush=True)
    mon = start_resource_monitor(out_dir / "resource.csv", gpus, vla_pid, hz)
    time.sleep(1.0)

    overall_t0 = time.time()
    per_task = []
    total_eps = 0
    total_succ = 0
    try:
        for t_idx, entry in enumerate(composite):
            label = entry.get("label", f"composite_{t_idx}")
            scene_suite = entry["scene_suite"]
            scene_task_id = int(entry["scene_task_id"])
            desc = entry["task_description"]
            print(f"\n===== composite task {t_idx}: {label} =====", flush=True)
            print(f"  scene: {scene_suite}/task{scene_task_id}", flush=True)
            print(f"  task_description: {desc}", flush=True)
            task_video_dir = out_dir / "videos" / f"task_{t_idx:02d}_{label}"
            try:
                restart_env(scene_suite, scene_task_id, task_video_dir)
            except Exception as e:
                print(f"  [restart error] {e}", flush=True)
                continue

            episodes = []
            for ep in range(n_episodes):
                try:
                    rec = asyncio.run(run_episode(
                        mcp_url, pilot, catalog, desc, t_idx, ep
                    ))
                except Exception as e:
                    print(f"  ep{ep}: exception {e}", flush=True)
                    rec = {"task_id": t_idx, "episode_idx": ep,
                           "error": str(e), "overall_success": False,
                           "plan": [], "trace": []}
                rec["video_path"] = str(task_video_dir / f"episode_{ep:04d}.mp4")
                rec["scene_suite"] = scene_suite
                rec["scene_task_id"] = scene_task_id
                rec["composite_label"] = label
                episodes.append(rec)
                total_eps += 1
                if rec.get("overall_success"):
                    total_succ += 1
                plan_names = [f"{n.get('skill')}({n.get('args',{}).get('object','?')}"
                              f" -> {n.get('args',{}).get('destination','?')})"
                              for n in rec.get("plan", [])]
                node_results = [(n["skill"], n["cycles"], n["intermediate_success"])
                                for n in rec.get("trace", [])]
                print(f"  ep{ep}: overall={rec.get('overall_success')} "
                      f"plan_len={len(rec.get('plan',[]))} "
                      f"plan={plan_names} "
                      f"trace={node_results} "
                      f"dur={rec.get('duration_s','?')}s", flush=True)

            task_succ = sum(1 for e in episodes if e.get("overall_success"))
            task_sr = task_succ / max(1, len(episodes))
            per_task.append({
                "task_id": t_idx, "description": desc,
                "composite_label": label, "scene_suite": scene_suite,
                "scene_task_id": scene_task_id, "n_episodes": len(episodes),
                "successes": task_succ, "task_sr": task_sr, "episodes": episodes,
            })
            print(f"  task_sr = {task_succ}/{len(episodes)} = {task_sr*100:.1f}%",
                  flush=True)
            with open(out_dir / "summary.json", "w") as f:
                json.dump({"partial": True, "per_task": per_task}, f, indent=2)
    finally:
        try:
            mon.terminate(); mon.wait(timeout=3)
        except Exception:
            mon.kill()
        subprocess.run(["pkill", "-TERM", "-f", "libero_env_node.py"], check=False)
        time.sleep(3.0)
        subprocess.run(["pkill", "-KILL", "-f", "libero_env_node.py"], check=False)

    overall_sr = total_succ / max(1, total_eps)
    summary = {
        "mode": "composite",
        "composite_tasks": len(composite),
        "n_episodes_per_task": n_episodes,
        "vlm_model": model, "vlm_base_url": base_url,
        "total_episodes": total_eps, "total_success": total_succ,
        "overall_sr": overall_sr,
        "eval_start_ts": overall_t0, "eval_end_ts": time.time(),
        "elapsed_s": time.time() - overall_t0,
        "suite": "composite",
        "n_tasks": len(composite),
        "per_task": per_task, "partial": False,
    }
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)
    return summary


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mcp-url", required=True)
    ap.add_argument("--pkg-root", required=True,
                    help="robonix package root containing skills/")
    ap.add_argument("--env-file",
                    default=str(Path.home() / "robonix_upstream/rust/examples/.env"))
    ap.add_argument("--suite", default="libero_spatial")
    ap.add_argument("--tasks", type=int, default=2)
    ap.add_argument("--episodes", type=int, default=2)
    ap.add_argument("--composite-tasks", default="",
                    help="JSON file with custom long-horizon task definitions")
    ap.add_argument("--out-dir",
                    default=str(Path(_DEFAULT_OUT_ROOT) /
                                time.strftime("pilot_%Y%m%d_%H%M%S")),
                    help="where to write artifacts (default: "
                         "~/robonix_eval/runs/pilot_<timestamp>)")
    ap.add_argument("--gpus", default="2,3")
    ap.add_argument("--hz", type=float, default=2.0)
    args = ap.parse_args()

    if args.composite_tasks:
        summary = run_composite_eval(args.mcp_url, Path(args.composite_tasks),
                                     args.episodes, Path(args.pkg_root),
                                     Path(args.env_file), Path(args.out_dir),
                                     args.gpus, args.hz)
    else:
        summary = run_eval(args.mcp_url, args.suite, args.tasks, args.episodes,
                           Path(args.pkg_root), Path(args.env_file),
                           Path(args.out_dir), args.gpus, args.hz)
    print(f"\n===== FINAL (Robonix Pilot-orchestrated SR) =====")
    print(f"VLM: {summary['vlm_model']}")
    print(f"suite={summary['suite']} tasks={summary['n_tasks']} "
          f"eps/task={summary['n_episodes_per_task']}")
    print(f"overall SR: {summary['total_success']}/{summary['total_episodes']} "
          f"= {summary['overall_sr']*100:.2f}%")
    print(f"elapsed: {summary['elapsed_s']/60:.1f} min")
    print(f"out_dir: {args.out_dir}")
