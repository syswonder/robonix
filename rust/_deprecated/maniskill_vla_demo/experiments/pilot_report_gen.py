#!/usr/bin/env python3
"""Generate markdown report + PNG plots for mini_pilot_eval.py output."""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_resource_csv(path: Path) -> dict:
    gpu_ts = defaultdict(list); gpu_util = defaultdict(list); gpu_mem = defaultdict(list)
    pid_ts = defaultdict(list); pid_rss = defaultdict(list)
    if not path.is_file():
        return {"gpu_ts": {}, "gpu_util": {}, "gpu_mem": {},
                "pid_ts": {}, "pid_rss": {}}
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                ts = float(row["ts"])
            except (TypeError, ValueError):
                continue
            if row.get("gpu_idx"):
                idx = int(row["gpu_idx"])
                gpu_ts[idx].append(ts)
                gpu_util[idx].append(float(row["gpu_util_pct"] or 0))
                gpu_mem[idx].append(float(row["gpu_mem_used_mb"] or 0))
            if row.get("pid"):
                pid = int(row["pid"])
                pid_ts[pid].append(ts)
                pid_rss[pid].append(float(row["rss_mb"] or 0))
    return {"gpu_ts": dict(gpu_ts), "gpu_util": dict(gpu_util),
            "gpu_mem": dict(gpu_mem),
            "pid_ts": dict(pid_ts), "pid_rss": dict(pid_rss)}


def slice_range(ts_list, vals_list, t_min, t_max):
    return [(t, v) for t, v in zip(ts_list, vals_list) if t_min <= t <= t_max]


def _find_object_key(positions: dict, phrase: str) -> str | None:
    """Loosely map a natural-language phrase ('alphabet soup', 'the basket') to
    the closest `*_pos` key in an object_positions snapshot."""
    if not positions:
        return None
    phrase_l = phrase.lower()
    tokens = [tok for tok in phrase_l.replace(",", " ").split()
              if tok not in ("the", "a", "an", "of", "on", "in", "into", "onto",
                             "at", "to", "and", "it", "them")]
    best, best_overlap = None, 0
    for k in positions.keys():
        k_tokens = k.lower().replace("_pos", "").replace("_", " ").split()
        overlap = sum(1 for t in tokens if any(t in kt or kt in t for kt in k_tokens))
        if overlap > best_overlap:
            best_overlap = overlap; best = k
    return best


def judge_stage_success(node: dict, prev_positions: dict, xy_thresh: float = 0.12,
                        z_thresh: float = 0.05) -> dict:
    """Heuristic per-stage success: target object ends up near destination."""
    args = node.get("args", {}) or {}
    positions = node.get("object_positions", {}) or {}
    obj_phrase = args.get("object", "")
    dest_phrase = args.get("destination", args.get("target", ""))
    if not positions or not obj_phrase or not dest_phrase:
        return {"verdict": "unknown", "reason": "no position data"}
    obj_key = _find_object_key(positions, obj_phrase)
    dest_key = _find_object_key(positions, dest_phrase)
    if obj_key is None or dest_key is None:
        return {"verdict": "unknown", "obj_key": obj_key, "dest_key": dest_key,
                "reason": "could not map phrase to position key"}
    ox, oy, oz = positions[obj_key][:3]
    dx, dy, dz = positions[dest_key][:3]
    xy_dist = ((ox-dx)**2 + (oy-dy)**2) ** 0.5
    moved = None
    if prev_positions.get(obj_key):
        px, py, pz = prev_positions[obj_key][:3]
        moved = ((ox-px)**2 + (oy-py)**2 + (oz-pz)**2) ** 0.5
    verdict = "success" if xy_dist < xy_thresh else "failed"
    return {
        "verdict": verdict,
        "obj_key": obj_key, "dest_key": dest_key,
        "xy_dist": round(xy_dist, 3),
        "obj_xyz": [round(v, 3) for v in (ox, oy, oz)],
        "dest_xyz": [round(v, 3) for v in (dx, dy, dz)],
        "moved_dist": round(moved, 3) if moved is not None else None,
    }


def plot_episode(rec: dict, resource: dict, out_png: Path) -> None:
    if "episode_start_ts" not in rec or "episode_end_ts" not in rec:
        return
    t0 = rec["episode_start_ts"] - 1.0
    t1 = rec["episode_end_ts"] + 1.0

    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True,
                             gridspec_kw={"height_ratios": [1, 1, 1]})

    # Only plot GPUs that actually did work this episode.
    active_gpus = []
    for idx, ts in resource["gpu_ts"].items():
        seg = slice_range(ts, resource["gpu_util"][idx], t0, t1)
        if seg and max(v for _, v in seg) > 1.0:
            active_gpus.append(idx)
    if not active_gpus:
        active_gpus = list(resource["gpu_ts"].keys())

    # GPU util
    for idx in active_gpus:
        ts = resource["gpu_ts"][idx]
        seg = slice_range(ts, resource["gpu_util"][idx], t0, t1)
        if not seg: continue
        axes[0].plot([s[0]-t0 for s in seg], [s[1] for s in seg],
                     label=f"GPU{idx} util %", linewidth=1.4)
    axes[0].set_ylabel("GPU util (%)"); axes[0].set_ylim(-5, 110)
    axes[0].legend(loc="lower right", fontsize=8); axes[0].grid(True, alpha=0.3)

    # GPU mem (fixed 0..32GB per-card for RTX 5090)
    for idx in active_gpus:
        ts = resource["gpu_ts"][idx]
        seg = slice_range(ts, resource["gpu_mem"][idx], t0, t1)
        if not seg: continue
        axes[1].plot([s[0]-t0 for s in seg], [s[1]/1024.0 for s in seg],
                     label=f"GPU{idx} mem (GB)", linewidth=1.4)
    axes[1].set_ylabel("GPU mem (GB)"); axes[1].set_ylim(0, 32)
    axes[1].legend(loc="lower right", fontsize=8); axes[1].grid(True, alpha=0.3)

    # vla RSS — force an absolute 0-baseline so the y axis doesn't microzoom
    # into microscopic +1.9000 + 0.0007 fractional territory.
    rss_max_gb = 0.0
    for pid, ts in resource["pid_ts"].items():
        seg = slice_range(ts, resource["pid_rss"][pid], t0, t1)
        if not seg: continue
        vals = [s[1] / 1024.0 for s in seg]
        rss_max_gb = max(rss_max_gb, max(vals))
        axes[2].plot([s[0]-t0 for s in seg], vals,
                     label=f"vla pid={pid} RSS (GB)", linewidth=1.4)
    if rss_max_gb > 0:
        axes[2].set_ylim(0, max(4.0, rss_max_gb * 1.25))
    axes[2].yaxis.get_major_formatter().set_useOffset(False)
    axes[2].set_ylabel("vla_node RSS (GB)")
    axes[2].set_xlabel(f"seconds since episode start (t0={t0:.1f})")
    axes[2].legend(loc="lower right", fontsize=8); axes[2].grid(True, alpha=0.3)

    # Pilot planning span (before the trace starts)
    plan_t = rec.get("plan_generated_ts")
    if plan_t:
        pt0 = rec["episode_start_ts"] - t0
        pt1 = plan_t - t0
        for ax in axes:
            ax.axvspan(pt0, pt1, alpha=0.15, color="#7f7f7f")
        axes[0].text((pt0+pt1)/2, 102, "Pilot plan\n(VLM call)",
                     ha="center", va="top", fontsize=7,
                     bbox=dict(facecolor="#7f7f7f", alpha=0.25, edgecolor="none"))

    # Per-node spans — stagger label y so overlapping short nodes stay readable.
    colors = ["#ff7f0e", "#1f77b4", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
    trace = rec.get("trace", [])
    label_rows = [95, 70, 45, 20]  # stagger heights within the GPU util panel
    for n in trace:
        n0 = n["start_ts"] - t0
        n1 = n["end_ts"] - t0
        c = colors[n["idx"] % len(colors)]
        label = f"node{n['idx']}: {n['skill']}"
        ok = "SUCCESS" if n.get("intermediate_success") else (
            "timeout" if n.get("status") == "timeout" else "done")
        for ax in axes:
            ax.axvspan(n0, n1, alpha=0.15, color=c)
        y_label = label_rows[n["idx"] % len(label_rows)]
        # For very narrow spans, draw the label with a leader line pointing at it
        # so it doesn't clip off to the right margin.
        width = max(0.01, n1 - n0)
        axes[0].annotate(
            f"{label}\n{ok}, {n.get('cycles','?')} cyc ({width:.1f}s)",
            xy=((n0+n1)/2, 50),
            xytext=((n0+n1)/2, y_label),
            ha="center", va="center", fontsize=7.5,
            arrowprops=dict(arrowstyle="-", color=c, lw=0.7, alpha=0.6),
            bbox=dict(facecolor=c, alpha=0.35, edgecolor="none", pad=1.5),
        )

    status_emoji = "✓" if rec.get("overall_success") else "✗"
    title = (f"task{rec['task_id']} ep{rec['episode_idx']}   "
             f"{status_emoji}   "
             f"plan_len={rec.get('plan_length','?')}   "
             f"total {rec.get('duration_s','?')}s "
             f"(plan {rec.get('plan_latency_s','?')}s)")
    fig.suptitle(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110, bbox_inches="tight")
    plt.close(fig)


def generate_markdown(summary: dict, resource: dict, out_dir: Path,
                      report_path: Path) -> None:
    md = []
    md.append("# Robonix Pilot-orchestrated SR Evaluation\n")
    md.append(f"- VLM: `{summary.get('vlm_model','?')}` @ `{summary.get('vlm_base_url','?')}`")
    md.append(f"- Suite: `{summary['suite']}`  ")
    md.append(f"- Tasks: **{summary['n_tasks']}**  Episodes/task: **{summary['n_episodes_per_task']}**  ")
    md.append(f"- Elapsed: **{summary['elapsed_s']/60:.1f} min**\n")

    md.append("## Headline\n")
    md.append(f"- Overall SR: `{summary['total_success']}/{summary['total_episodes']}` = "
              f"**{summary['overall_sr']*100:.2f}%**\n")

    md.append("## Per-task SR\n")
    md.append("| task_id | instruction | SR | successes |")
    md.append("|---|---|---|---|")
    for t in summary["per_task"]:
        md.append(f"| {t['task_id']} | {t['description']} | "
                  f"**{t['task_sr']*100:.0f}%** | "
                  f"{t['successes']}/{t['n_episodes']} |")
    md.append("")

    md.append("## Per-episode detail\n")
    md.append("Each episode: Pilot's VLM-generated plan + one continuous video of the "
              "executor running every plan node end-to-end, with the resource curve "
              "annotated by which skill was executing.\n")
    for t in summary["per_task"]:
        md.append(f"### task_{t['task_id']}: {t['description']}\n")
        md.append(f"Task SR: **{t['task_sr']*100:.0f}%** ({t['successes']}/{t['n_episodes']})\n")
        for rec in t["episodes"]:
            ep = rec["episode_idx"]
            status = "✅" if rec.get("overall_success") else "❌"
            rel_video = f"videos/task_{t['task_id']:02d}/episode_{ep:04d}.mp4"
            rel_plot = f"plots/task_{t['task_id']:02d}_episode_{ep:04d}.png"
            plot_path = out_dir / rel_plot
            plot_path.parent.mkdir(parents=True, exist_ok=True)
            try:
                plot_episode(rec, resource, plot_path)
            except Exception as e:
                print(f"[report] plot failed task{t['task_id']} ep{ep}: {e}")

            md.append(f"#### ep{ep}  {status}   total {rec.get('duration_s','?')}s  "
                      f"(Pilot plan latency {rec.get('plan_latency_s','?')}s)\n")
            md.append("**Input (task description):**\n")
            md.append(f"> {rec.get('task_description','?')}\n")

            # Pilot plan
            md.append("**Pilot plan (VLM output):**\n")
            md.append("```json")
            md.append(json.dumps(rec.get("plan", []), indent=2, ensure_ascii=False))
            md.append("```")
            md.append("")

            # Node execution trace with per-stage success judged from object
            # position deltas (the env's task-level success predicate latches
            # after the first stage and is misleading for later ones).
            md.append("**Executor trace (per-stage success = target object is near "
                      "destination at stage end):**\n")
            md.append("| node | skill | args | cycles | env_latch | stage_success | "
                      "obj@end | dest | xy_dist | duration |")
            md.append("|---|---|---|---|---|---|---|---|---|---|")
            prev_pos = {}
            for n in rec.get("trace", []):
                args_str = json.dumps(n.get("args", {}), ensure_ascii=False)
                env_latch = "✓" if n.get("intermediate_success") else "✗"
                verdict = judge_stage_success(n, prev_pos)
                v = verdict["verdict"]
                stage_mark = {"success": "✅", "failed": "❌", "unknown": "—"}[v]
                obj_xyz = verdict.get("obj_xyz", "—")
                dest_xyz = verdict.get("dest_xyz", "—")
                xy_dist = verdict.get("xy_dist", "—")
                md.append(f"| {n['idx']} | `{n['skill']}` | `{args_str}` | "
                          f"{n.get('cycles','?')} | {env_latch} | "
                          f"{stage_mark} {v} | {obj_xyz} | {dest_xyz} | "
                          f"{xy_dist} | {n.get('duration_s','?')}s |")
                if n.get("object_positions"):
                    prev_pos = n["object_positions"]
            md.append("")

            abs_vid = out_dir / rel_video
            if abs_vid.is_file():
                md.append(f"**Video** (one continuous recording spanning all plan nodes): "
                          f"[{rel_video}](./{rel_video})\n")
            else:
                md.append(f"**Video**: _{rel_video}_ (missing)\n")
            md.append(f"**Resource curve** (grey = Pilot VLM call, colored spans = executor nodes):\n")
            md.append(f"![resource]({rel_plot})\n")

    report_path.write_text("\n".join(md))
    print(f"[report] wrote {report_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--eval-dir", required=True)
    args = ap.parse_args()

    out_dir = Path(args.eval_dir)
    summary = json.loads((out_dir / "summary.json").read_text())
    resource = load_resource_csv(out_dir / "resource.csv")
    generate_markdown(summary, resource, out_dir, out_dir / "report.md")


if __name__ == "__main__":
    main()
