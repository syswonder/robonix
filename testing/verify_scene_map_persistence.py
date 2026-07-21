#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Verify scene-driven SLAM map persistence against a running Webots deploy.

Runtime verifier for the map persistence path. It uses scene's public HTTP
endpoints for save/load, then reaches into the Webots mapping container only as
a verifier to inspect the provider artifact and compare it with the live /map.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str


def http_json(method: str, url: str, payload: dict[str, Any] | None = None,
              timeout: float = 60.0) -> dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data,
                                 headers={"Content-Type": "application/json"},
                                 method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            body = resp.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code} {url}: {body}") from exc
    return json.loads(body)


def docker_python(container: str, code: str, *, env: dict[str, str] | None = None,
                  timeout: float = 60.0) -> dict[str, Any]:
    cmd = ["docker", "exec", "-i"]
    for k, v in (env or {}).items():
        cmd += ["-e", f"{k}={v}"]
    cmd += [container, "python3", "-"]
    proc = subprocess.run(cmd, input=code, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=timeout, check=False)
    if proc.returncode != 0:
        raise RuntimeError(
            f"docker python failed rc={proc.returncode} in {container}\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    return json.loads(proc.stdout)


def mapping_artifact(
    container: str,
    map_id: str,
    maps_dir: str,
    timeout: float,
) -> dict[str, Any]:
    code = r"""
import json, os, sqlite3, struct
mid = os.environ["VERIFY_MAP_ID"]
base = os.path.join(os.environ["VERIFY_MAPS_DIR"], mid)
png = os.path.join(base, "occupancy.png")
meta = os.path.join(base, "meta.yaml")
db = os.path.join(base, "rtabmap.db")
def read_meta(path):
    out = {}
    if not os.path.isfile(path):
        return out
    for line in open(path, encoding="utf-8"):
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        out[k.strip()] = v.strip()
    return out
def png_size(path):
    with open(path, "rb") as fh:
        sig = fh.read(24)
    if sig[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    return struct.unpack(">II", sig[16:24])
res = {
    "base": base,
    "exists": os.path.isdir(base),
    "db_exists": os.path.isfile(db),
    "preview_exists": os.path.isfile(png),
    "meta_exists": os.path.isfile(meta),
    "files": sorted(os.listdir(base)) if os.path.isdir(base) else [],
}
if os.path.isfile(png):
    res["preview_width"], res["preview_height"] = png_size(png)
if os.path.isfile(meta):
    res["meta"] = read_meta(meta)
if os.path.isfile(db):
    res["artifact_size"] = os.path.getsize(db)
    con = sqlite3.connect("file:" + db + "?mode=ro", uri=True, timeout=30.0)
    res["quick_check"] = con.execute("PRAGMA quick_check").fetchone()[0]
    counts = {}
    for table in ["Node", "Data", "Link", "Word"]:
        try:
            counts[table] = con.execute("SELECT COUNT(*) FROM " + table).fetchone()[0]
        except sqlite3.Error as exc:
            counts[table] = "error:" + str(exc)
    con.close()
    res["counts"] = counts
print(json.dumps(res, ensure_ascii=False))
"""
    return docker_python(
        container,
        code,
        env={"VERIFY_MAP_ID": map_id, "VERIFY_MAPS_DIR": maps_dir},
        timeout=timeout,
    )


def live_map(container: str, timeout: float) -> dict[str, Any]:
    code = r"""
import json, time
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
rclpy.init()
node = rclpy.create_node("verify_scene_map_persistence")
box = {}
def cb(msg):
    data = list(msg.data)
    box["msg"] = {
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "known": sum(1 for v in data if v >= 0),
        "occupied": sum(1 for v in data if v > 50),
        "free": sum(1 for v in data if v == 0),
        "unknown": sum(1 for v in data if v < 0),
        "frame_id": msg.header.frame_id,
        "origin_x": msg.info.origin.position.x,
        "origin_y": msg.info.origin.position.y,
    }
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(OccupancyGrid, "/map", cb, qos)
end = time.time() + 20.0
while time.time() < end and "msg" not in box:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node(); rclpy.shutdown()
if "msg" not in box:
    raise SystemExit("no /map received")
print(json.dumps(box["msg"], ensure_ascii=False))
"""
    shell = (
        "set -eo pipefail\n"
        "source /opt/ros/humble/setup.bash\n"
        "export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}\n"
        "python3 - <<'PY'\n" + code + "\nPY\n"
    )
    proc = subprocess.run(["docker", "exec", "-i", container, "bash"],
                          input=shell, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, timeout=timeout, check=False)
    if proc.returncode != 0:
        raise RuntimeError(
            f"live /map probe failed rc={proc.returncode}\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    return json.loads(proc.stdout)


def origin_meta(meta: dict[str, str]) -> tuple[float, float] | None:
    raw = meta.get("origin")
    if not raw:
        return None
    vals = raw.strip().strip("[]").split(",")
    if len(vals) < 2:
        return None
    try:
        return float(vals[0]), float(vals[1])
    except ValueError:
        return None


def check(name: str, ok: bool, detail: str, results: list[CheckResult]) -> None:
    results.append(CheckResult(name, ok, detail))
    print(("PASS" if ok else "FAIL"), name, detail)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scene-url", default="http://127.0.0.1:50107")
    ap.add_argument("--map-id", default=f"verify_scene_map_{int(time.time())}")
    ap.add_argument("--note", default="scene map persistence verifier")
    ap.add_argument("--mapping-container", default="robonix_mapping")
    ap.add_argument("--maps-dir", default="/mapping/maps")
    ap.add_argument("--sim-container", default="robonix_tiago_sim")
    ap.add_argument("--min-artifact-bytes", type=int, default=1_000_000)
    ap.add_argument("--min-nodes", type=int, default=1)
    ap.add_argument("--min-known-cells", type=int, default=1)
    ap.add_argument(
        "--dimension-tolerance-cells",
        type=int,
        default=2,
        help="maximum save/load occupancy-grid width or height drift in cells",
    )
    ap.add_argument("--origin-tolerance", type=float, default=0.05)
    ap.add_argument("--timeout", type=float, default=420.0)
    ap.add_argument("--skip-save", action="store_true")
    ap.add_argument("--skip-load", action="store_true")
    ap.add_argument("--delete-after", action="store_true")
    args = ap.parse_args()

    scene = args.scene_url.rstrip("/")
    results: list[CheckResult] = []
    print(f"map_id={args.map_id}")

    if not args.skip_save:
        save = http_json("POST", scene + "/api/maps/save",
                         {"map_id": args.map_id, "note": args.note},
                         timeout=args.timeout)
        print("save_response", json.dumps(save, ensure_ascii=False, sort_keys=True))
        check("scene_save_ok", bool(save.get("ok")), str(save.get("detail", "")), results)
        validation = save.get("validation") if isinstance(save.get("validation"), dict) else {}
        check("scene_validation_spatial_ok", bool(validation.get("spatial_ok")),
              str(validation.get("artifact_detail", "")), results)
        check("scene_validation_has_preview", bool(validation.get("has_preview")),
              f"validation={validation}", results)
        check("scene_semantic_counts_reported",
              "object_count" in validation and "room_count" in validation,
              f"objects={validation.get('object_count')} rooms={validation.get('room_count')}", results)

    artifact = mapping_artifact(
        args.mapping_container,
        args.map_id,
        args.maps_dir,
        timeout=60.0,
    )
    print("artifact", json.dumps(artifact, ensure_ascii=False, sort_keys=True))
    check("artifact_directory_exists", bool(artifact.get("exists")), str(artifact.get("base")), results)
    check("artifact_sqlite_exists", bool(artifact.get("db_exists")), str(artifact.get("files")), results)
    check("artifact_quick_check_ok", artifact.get("quick_check") == "ok", str(artifact.get("quick_check")), results)
    check("artifact_size_nontrivial", int(artifact.get("artifact_size") or 0) >= args.min_artifact_bytes,
          str(artifact.get("artifact_size")), results)
    counts = artifact.get("counts") if isinstance(artifact.get("counts"), dict) else {}
    check("artifact_nodes_nontrivial", int(counts.get("Node") or 0) >= args.min_nodes,
          str(counts), results)
    check("preview_exists", bool(artifact.get("preview_exists")), str(artifact.get("files")), results)

    live = None
    if not args.skip_load:
        load = http_json("POST", scene + "/api/maps/load",
                         {"map_id": args.map_id, "mode": "localization"},
                         timeout=args.timeout)
        print("load_response", json.dumps(load, ensure_ascii=False, sort_keys=True))
        check("scene_load_ok", bool(load.get("ok")), str(load.get("detail", "")), results)

        live = live_map(args.sim_container, timeout=45.0)
        print("live_map", json.dumps(live, ensure_ascii=False, sort_keys=True))
        check("live_map_known_cells", int(live.get("known") or 0) >= args.min_known_cells,
              str(live), results)

    meta = artifact.get("meta") if isinstance(artifact.get("meta"), dict) else {}
    expected_w = int(meta.get("width") or artifact.get("preview_width") or 0)
    expected_h = int(meta.get("height") or artifact.get("preview_height") or 0)
    check("preview_matches_meta",
          expected_w == int(artifact.get("preview_width") or -1)
          and expected_h == int(artifact.get("preview_height") or -1),
          f"meta={expected_w}x{expected_h} preview={artifact.get('preview_width')}x{artifact.get('preview_height')}",
          results)
    if live is not None:
        live_w = int(live.get("width") or -1)
        live_h = int(live.get("height") or -1)
        width_delta = abs(expected_w - live_w)
        height_delta = abs(expected_h - live_h)
        check("loaded_map_dimensions_match_saved_meta",
              width_delta <= args.dimension_tolerance_cells
              and height_delta <= args.dimension_tolerance_cells,
              f"meta={expected_w}x{expected_h} live={live_w}x{live_h} "
              f"delta={width_delta}x{height_delta} "
              f"tolerance={args.dimension_tolerance_cells} cells",
              results)
        meta_origin = origin_meta(meta)
        if meta_origin is not None:
            dx = abs(float(live.get("origin_x") or 0.0) - meta_origin[0])
            dy = abs(float(live.get("origin_y") or 0.0) - meta_origin[1])
            check("loaded_map_origin_matches_saved_meta",
                  dx <= args.origin_tolerance and dy <= args.origin_tolerance,
                  f"dx={dx:.4f} dy={dy:.4f} meta={meta_origin} live=({live.get('origin_x')},{live.get('origin_y')})",
                  results)

    if args.delete_after:
        deleted = http_json("POST", scene + "/api/maps/delete",
                            {"map_id": args.map_id}, timeout=60.0)
        print("delete_response", json.dumps(deleted, ensure_ascii=False, sort_keys=True))
        check("delete_after_ok", bool(deleted.get("ok")), str(deleted), results)

    failed = [r for r in results if not r.ok]
    print(f"summary: {len(results) - len(failed)}/{len(results)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
