#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Export the Scene semantic map as a point cloud for offline inspection.

The debug UI renders the same data with three.js from a CDN, which needs
WebGL and network access. Neither is available on a headless host, so a
screenshot of that page cannot be taken as part of a run and the map cannot
be looked at from a chosen angle. This reads the same endpoint and writes
formats that do not need a browser:

  * a coloured PLY, one colour per object instance, openable in MeshLab,
    CloudCompare, Open3D or Blender;
  * a rerun recording (`.rrd`) with per-object entity paths, class labels
    and oriented boxes, when `rerun-sdk` is installed.

This is for a snapshot taken away from the robot. For watching a running
deployment, Scene publishes `visualization_msgs/MarkerArray` and
`sensor_msgs/PointCloud2`; open those in RViz or Foxglove rather than
exporting.

Usage:

    python3 testing/export_scene_pointcloud.py --out scene-map
    python3 testing/export_scene_pointcloud.py --state-file snapshot.json --out map

`--scene-url` defaults to the debug UI on localhost. Exporting from a saved
`/api/objects3d` snapshot needs no running deployment.
"""
from __future__ import annotations

import argparse
import json
import sys
import urllib.request
from pathlib import Path

_DEFAULT_URL = "http://127.0.0.1:50107/api/objects3d"


def load_snapshot(url: str | None, state_file: Path | None) -> dict:
    """Read one `/api/objects3d` payload from a file or the running service."""
    if state_file is not None:
        return json.loads(state_file.read_text(encoding="utf-8"))
    with urllib.request.urlopen(url, timeout=30) as response:  # noqa: S310
        return json.loads(response.read().decode("utf-8"))


def _instance_colour(obj: dict, index: int) -> tuple[int, int, int]:
    """Return the object's own colour, or a stable one derived from its id.

    The registry supplies `inst_color` for objects the perception layer has
    coloured. Objects without one still need to be distinguishable from their
    neighbours, and a hash of the identifier gives the same colour on every
    export, which a random palette would not.
    """
    raw = obj.get("inst_color")
    if isinstance(raw, (list, tuple)) and len(raw) >= 3:
        return tuple(int(max(0.0, min(1.0, float(c))) * 255) for c in raw[:3])
    digest = abs(hash(str(obj.get("id", index))))
    return ((digest >> 16) & 0xFF, (digest >> 8) & 0xFF, digest & 0xFF)


def write_ply(snapshot: dict, path: Path) -> int:
    """Write every object's points into one coloured PLY. Returns the count."""
    rows: list[str] = []
    for index, obj in enumerate(snapshot.get("objects") or []):
        red, green, blue = _instance_colour(obj, index)
        for point in obj.get("points") or []:
            if len(point) < 3:
                continue
            rows.append(
                f"{point[0]:.4f} {point[1]:.4f} {point[2]:.4f} "
                f"{red} {green} {blue}"
            )
    header = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(rows)}",
        "property float x",
        "property float y",
        "property float z",
        "property uchar red",
        "property uchar green",
        "property uchar blue",
        "end_header",
    ]
    path.write_text("\n".join(header + rows) + "\n", encoding="utf-8")
    return len(rows)


def write_rerun(snapshot: dict, path: Path) -> bool:
    """Log objects to a rerun recording. Returns False when rerun is absent."""
    try:
        import rerun as rr
    except ImportError:
        return False
    rr.init("robonix-scene", spawn=False)
    rr.save(str(path))
    for index, obj in enumerate(snapshot.get("objects") or []):
        points = [p[:3] for p in (obj.get("points") or []) if len(p) >= 3]
        if not points:
            continue
        label = str(obj.get("cls", "object"))
        entity = f"scene/{label}/{obj.get('id', index)}"
        colour = _instance_colour(obj, index)
        rr.log(entity, rr.Points3D(points, colors=[colour] * len(points)))
        centre = obj.get("center")
        corners = obj.get("bbox_corners") or []
        if centre and len(corners) >= 8:
            spans = [
                max(c[axis] for c in corners) - min(c[axis] for c in corners)
                for axis in (0, 1, 2)
            ]
            rr.log(
                f"{entity}/box",
                rr.Boxes3D(
                    centers=[centre[:3]],
                    half_sizes=[[s / 2.0 for s in spans]],
                    labels=[label],
                    colors=[colour],
                ),
            )
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-url", default=_DEFAULT_URL)
    parser.add_argument("--state-file", type=Path)
    parser.add_argument("--out", type=Path, required=True,
                        help="output stem; .ply, .rrd and .png are appended")
    arguments = parser.parse_args()

    snapshot = load_snapshot(arguments.scene_url, arguments.state_file)
    objects = snapshot.get("objects") or []
    if not objects:
        print("the snapshot holds no objects; nothing to export", file=sys.stderr)
        return 1

    stem = arguments.out
    stem.parent.mkdir(parents=True, exist_ok=True)
    written = write_ply(snapshot, stem.with_suffix(".ply"))
    print(f"{stem.with_suffix('.ply')}: {len(objects)} objects, {written} points")

    if write_rerun(snapshot, stem.with_suffix(".rrd")):
        print(f"{stem.with_suffix('.rrd')}: rerun recording")
    else:
        print("rerun-sdk not installed; skipped the .rrd export")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
