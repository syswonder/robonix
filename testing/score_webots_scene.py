#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Score the objects Scene found against the Webots world, and draw them on the map.

Truth comes from the checked-in WBT (scene_quality_ground_truth), in the
robot's initial frame. The simulator's pose of the robot and Scene's own
estimate of it, read at the same moment, give the transform into the map
Scene's objects are in. Recall counts only the truth objects the map has
looked at: at least half of a ring just outside the object's footprint is
observed ground. A room the robot never entered is then not a miss, while an
object it drove past and did not report is. The scoring is
evaluate_webots_scene.py's.

    score_webots_scene.py --world-id office --state-file state.json \\
        --robot-world-pose X Y YAW --out-dir sim-logs

writes scene-score.json (the evaluation, plus the alignment and scope),
scene-objects.png (the SLAM map with every truth object and every detection,
named) and scene-score.png (the scores, overall and per class), in Arial.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import subprocess
import sys
from pathlib import Path

from PIL import Image

from scene_quality_ground_truth import (
    initial_robot_pose,
    load_semantic_inventory,
    transform_semantic_inventory_planar,
)

REPO = Path(__file__).resolve().parents[1]
BENCHMARK = REPO / "testing" / "fixtures" / "webots_scene_benchmark.json"
RING_OFFSET_M = 0.2
RING_KNOWN_FRACTION = 0.5
UNKNOWN = 128  # the grey Scene's /api/state renders unobserved cells with

GREEN, ORANGE, RED, GREY, FAINT = (22, 150, 60), (230, 130, 0), (210, 30, 30), (110, 110, 110), (190, 190, 190)


def alignment(robot_world: tuple[float, float, float], robot_map: dict,
              initial: tuple[float, float, float]) -> tuple[float, float, float]:
    """T(map <- initial robot frame) as (x, y, yaw), from one simultaneous pose pair."""
    x0, y0, yaw0 = initial
    wx, wy, wyaw = robot_world
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    ix, iy = c * (wx - x0) - s * (wy - y0), s * (wx - x0) + c * (wy - y0)
    yaw = float(robot_map["yaw"]) - (wyaw - yaw0)
    c, s = math.cos(yaw), math.sin(yaw)
    return float(robot_map["x"]) - (c * ix - s * iy), float(robot_map["y"]) - (s * ix + c * iy), yaw


class Grid:
    """Scene's occupancy PNG (rows top-down, y up) with map coordinates."""

    def __init__(self, occupancy: dict):
        self.image = Image.open(io.BytesIO(base64.b64decode(occupancy["png_b64"]))).convert("L")
        self.res = float(occupancy["resolution"])
        self.ox, self.oy = float(occupancy["origin_x"]), float(occupancy["origin_y"])

    def pixel(self, x: float, y: float) -> tuple[float, float]:
        return (x - self.ox) / self.res, self.image.height - (y - self.oy) / self.res

    def known(self, x: float, y: float) -> bool:
        px, py = self.pixel(x, y)
        if not (0 <= px < self.image.width and 0 <= py < self.image.height):
            return False
        return self.image.getpixel((int(px), int(py))) != UNKNOWN


def corners(cx: float, cy: float, sx: float, sy: float, yaw: float) -> list[tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2))]


def looked_at(truth, grid: Grid) -> bool:
    pts = corners(truth.center_m[0], truth.center_m[1], truth.size_m[0] + 2 * RING_OFFSET_M,
                  truth.size_m[1] + 2 * RING_OFFSET_M, truth.yaw_rad)
    ring = [(a[0] + (b[0] - a[0]) * k / 6, a[1] + (b[1] - a[1]) * k / 6)
            for a, b in zip(pts, pts[1:] + pts[:1]) for k in range(6)]
    return sum(grid.known(x, y) for x, y in ring) >= RING_KNOWN_FRACTION * len(ring)


def _arial() -> None:
    """Draw in Arial; fontconfig names the file, which matplotlib may not have indexed."""
    import matplotlib
    from matplotlib import font_manager

    found = subprocess.run(["fc-match", "-f", "%{file}", "Arial"], capture_output=True, text=True).stdout
    if found and "arial" in Path(found).name.lower():
        font_manager.fontManager.addfont(found)
    else:
        print(f"WARN Arial not installed; fontconfig offers {found or 'nothing'}", file=sys.stderr)
    matplotlib.rcParams["font.family"] = "Arial"


def draw_map(grid: Grid, truths, objects: list[dict], result: dict, out: Path) -> None:
    """The SLAM map with every truth box and every detection, named."""
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Polygon

    w, h = grid.image.size
    extent = (grid.ox, grid.ox + w * grid.res, grid.oy, grid.oy + h * grid.res)
    fig, ax = plt.subplots(figsize=(max(6.0, w * grid.res * 0.9), max(6.0, h * grid.res * 0.9)))
    ax.imshow(grid.image, cmap="gray", vmin=0, vmax=255, extent=extent, interpolation="nearest")
    rgb = {k: tuple(v / 255 for v in c) for k, c in
           {"ok": GREEN, "label": ORANGE, "fp": RED, "miss": GREY, "unseen": FAINT}.items()}

    def label(x: float, y: float, name: str, colour) -> None:
        ax.annotate(name, (x, y), xytext=(4, 3), textcoords="offset points", fontsize=7, color=colour,
                    bbox={"boxstyle": "round,pad=0.15", "fc": "white", "ec": "none", "alpha": 0.75})

    targets = {t["identity"]: t for t in result["per_target"]}
    for truth in truths:
        entry = targets.get(truth.identity)
        kind = ("unseen" if entry is None else "miss" if not entry["matched"]
                else "ok" if entry.get("label_correct") else "label")
        ax.add_patch(Polygon(corners(truth.center_m[0], truth.center_m[1], *truth.size_m[:2], truth.yaw_rad),
                             closed=True, fill=False, ec=rgb[kind], lw=0.6 if kind == "unseen" else 1.4))
        if kind == "miss":
            label(truth.center_m[0], truth.center_m[1], truth.label, rgb["miss"])
    by_id = {str(o.get("id")): o for o in objects}
    for entry in targets.values():
        obj = by_id.get(entry.get("object_id", ""))
        if obj is None:
            continue
        kind = "ok" if entry["label_correct"] else "label"
        ax.plot(obj["pose"]["x"], obj["pose"]["y"], "o", ms=5, color=rgb[kind])
        label(obj["pose"]["x"], obj["pose"]["y"], entry["observed_label"] if kind == "ok"
              else f'{entry["observed_label"]} ({entry["expected_label"]})', rgb[kind])
    for group, tag in (("duplicates", "duplicate"), ("ghosts", "ghost")):
        for fp in result[group]:
            ax.plot(fp["center_m"][0], fp["center_m"][1], "o", ms=5, color=rgb["fp"])
            label(fp["center_m"][0], fp["center_m"][1], f'{fp["observed_label"]} ({tag})', rgb["fp"])
    ax.plot(result["robot_map"]["x"], result["robot_map"]["y"], "o", ms=9, mfc="none", mec="#005ad8", mew=2)
    ax.legend(handles=[Line2D([], [], marker="o", ls="", color=rgb["ok"], label="Detected, correct name"),
                       Line2D([], [], marker="o", ls="", color=rgb["label"], label="Detected, wrong name (truth)"),
                       Line2D([], [], marker="o", ls="", color=rgb["fp"], label="Ghost or duplicate"),
                       Line2D([], [], color=rgb["miss"], label="Missed"),
                       Line2D([], [], color=rgb["unseen"], label="Not looked at"),
                       Line2D([], [], marker="o", ls="", mfc="none", mec="#005ad8", mew=2, label="Robot")],
              loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8, frameon=False)
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def draw_chart(result: dict, out: Path) -> None:
    """Overall scores, and per class what was looked at, found and named right."""
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    fig, (left, right) = plt.subplots(1, 2, figsize=(11, 4.2), gridspec_kw={"width_ratios": [1, 2.4]})
    names = ["Precision", "Recall", "F1", "Label accuracy"]
    values = [result["precision"], result["recall"], result["f1"], result["label_accuracy"]]
    bars = left.bar(names, values, color=["#2f6db5", "#2f9e5b", "#7a4fb5", "#d98a1c"])
    left.bar_label(bars, fmt="%.2f", fontsize=9)
    left.set_ylim(0, 1.08)
    left.set_title(f'TP {result["tp"]}   FP {result["fp"]}   FN {result["fn"]}', fontsize=10)
    left.tick_params(axis="x", labelrotation=20, labelsize=9)

    classes = sorted(result["per_class"], key=lambda c: -result["per_class"][c]["visible_truth_count"])
    per = [result["per_class"][c] for c in classes]
    x = range(len(classes))
    right.bar(x, [p["visible_truth_count"] for p in per], color=tuple(v / 255 for v in FAINT), label="Looked at")
    right.bar(x, [p["detected_count"] for p in per], color=tuple(v / 255 for v in ORANGE), label="Detected")
    right.bar(x, [p["correct_label_count"] for p in per], color=tuple(v / 255 for v in GREEN),
              label="Detected, correct name")
    right.set_xticks(list(x), classes, rotation=35, ha="right", fontsize=9)
    right.set_ylabel("Objects")
    right.yaxis.set_major_locator(MaxNLocator(integer=True))
    right.legend(fontsize=8, frameon=False)
    right.set_title(f'{result["visible_truth_count"]} of {result["truth_count"]} objects looked at; '
                    f'{result["duplicate_fp_count"]} duplicates, {result["ghost_fp_count"]} ghosts', fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--world-id", required=True)
    ap.add_argument("--state-file", type=Path, required=True, help="Scene /api/state")
    ap.add_argument("--robot-world-pose", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"),
                    help="the simulator's pose of the robot when the state was read")
    ap.add_argument("--out-dir", type=Path, required=True)
    a = ap.parse_args()

    state = json.loads(a.state_file.read_text(encoding="utf-8"))
    truths, _ = load_semantic_inventory(BENCHMARK, world_id=a.world_id, repository_root=REPO)
    tx, ty, yaw = alignment(tuple(a.robot_world_pose), state["robot"],
                            initial_robot_pose(BENCHMARK, world_id=a.world_id, repository_root=REPO))
    truths = transform_semantic_inventory_planar(truths, translation_m=(tx, ty, 0.0), yaw_rad=yaw)
    grid = Grid(state["occupancy"])
    visible = {t.identity for t in truths if looked_at(t, grid)}

    a.out_dir.mkdir(parents=True, exist_ok=True)
    visibility = a.out_dir / "scene-visibility.json"
    visibility.write_text(json.dumps({
        "visible_truth_ids": sorted(visible),
        "truth_alignment": {"target_frame": "map", "translation_m": [tx, ty, 0.0], "yaw_rad": yaw},
    }), encoding="utf-8")
    run = subprocess.run(
        [sys.executable, str(REPO / "testing" / "evaluate_webots_scene.py"), "--world-id", a.world_id,
         "--state-file", str(a.state_file), "--visibility-file", str(visibility)],
        capture_output=True, text=True, check=True)
    result = json.loads(run.stdout)
    result["robot_map"] = state["robot"]
    result["scope"] = {"method": "known ground on a ring around the footprint",
                       "ring_offset_m": RING_OFFSET_M, "known_fraction": RING_KNOWN_FRACTION}
    (a.out_dir / "scene-score.json").write_text(json.dumps(result, indent=2, sort_keys=True), encoding="utf-8")
    _arial()
    draw_map(grid, truths, list(state.get("objects") or ()), result, a.out_dir / "scene-objects.png")
    draw_chart(result, a.out_dir / "scene-score.png")
    print(f'scene score ({a.world_id}): precision {result["precision"]:.3f} recall {result["recall"]:.3f} '
          f'f1 {result["f1"]:.3f} label accuracy {result["label_accuracy"]:.3f} '
          f'(tp {result["tp"]} fp {result["fp"]} fn {result["fn"]}, '
          f'{result["visible_truth_count"]} of {result["truth_count"]} objects looked at)')
    return 0


if __name__ == "__main__":
    sys.exit(main())
