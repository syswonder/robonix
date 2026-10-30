# SPDX-License-Identifier: MulanPSL-2.0
"""One provider of the bench's world, booted by `rbnx boot` like any package.

It registers the contracts the recorded Webots deployment had for this
provider (fixtures/catalog.json), with the same argument schemas and
descriptions, and answers them from the office floor (fixtures/scene.yaml)
and the current case's world state. Every call is appended to the call log,
which the bench scores from.
"""

from __future__ import annotations

import argparse
import base64
import json
import os
import threading
import time
import uuid
from pathlib import Path

from robonix_api import Ok, Primitive, Service, Skill

import scene

HERE = Path(__file__).resolve().parent
CATALOG = json.loads((HERE.parent / "fixtures/catalog.json").read_text())
STATE = Path(os.environ["BENCH_WORLD_DIR"]) / "state.json"
CALLS = Path(os.environ["BENCH_WORLD_DIR"]) / "calls.jsonl"
_lock = threading.Lock()


def state() -> dict:
    try:
        return json.loads(STATE.read_text())
    except (OSError, json.JSONDecodeError):
        return {}


def image(name: str | None) -> dict:
    """A camera frame as sensor_msgs/Image JSON, which Pilot passes to the model."""
    path = Path(state().get("images_dir", HERE.parent / ".images")) / (name or "living_room.jpg")
    return {"width": 640, "height": 480, "encoding": "jpeg", "data": base64.b64encode(path.read_bytes()).decode()}


def answer(cid: str, args: dict) -> dict:
    """What the provider behind `cid` returns, in its contract's response shape."""
    st = state()
    # Every call takes a moment, so calls dispatched side by side overlap in
    # the call log and calls in sequence do not.
    time.sleep(float(st.get("call_s", 0.8)))
    if (fail := st.get("fail", {}).get(cid)) is not None:
        raise RuntimeError(fail)
    leaf = cid.rsplit("/", 1)[-1]
    if cid.startswith("robonix/system/scene/"):
        return {
            "list_objects": scene.list_objects,
            "find": lambda: scene.find(**args),
            "list_regions": lambda: {"regions": [scene.region(r) for r in scene.SCENE["regions"]], "map_id": ".live", "stamp_unix": time.time()},
            "list_relations": lambda: {"edges": scene.edges(args.get("relation", ""))},
            "goal_near": lambda: scene.goal_near(**args),
            "goal_region": lambda: scene.goal_region(**args),
            "goal_room": lambda: scene.goal_region(args.get("room_id") or args.get("region_id", "")),
            "get_object_context": lambda: scene.object_context(**args),
            "get_robot_context": scene.robot_context,
            "get_scene_graph": scene.scene_graph,
        }.get(leaf, lambda: {"ok": True, "detail": ""})()
    if cid == "robonix/primitive/camera/snapshot":
        return image(st.get("image"))
    if cid == "robonix/primitive/camera/depth_snapshot":
        return {"width": 64, "height": 48, "encoding": "16UC1", "data": base64.b64encode(bytes(64 * 48 * 2)).decode()}
    if cid == "robonix/primitive/lidar/snapshot":
        return {"angle_min": -1.57, "angle_max": 1.57, "angle_increment": 0.0873, "range_min": 0.05, "range_max": 10.0,
                "ranges": [round(2.0 + 0.1 * (i % 7), 2) for i in range(37)]}
    if cid == "robonix/service/navigation/navigate":
        # The run is accepted; the robot then drives for a while.
        time.sleep(float(st.get("nav_s", 2.0)))
        return {"accepted": True, "run_id": f"nav-{int(time.time() * 1000)}", "detail": ""}
    if cid == "robonix/service/navigation/navigate/status":
        return {"known": True, "state": "SUCCEEDED", "detail": "distance_remaining=0.08"}
    if cid.startswith("robonix/service/navigation/"):
        return {"ok": True, "detail": ""}
    if cid == "robonix/service/map/get_pose":
        return {"ok": True, "x": 0.0, "y": 0.0, "theta": 0.0, "frame_id": "map", "detail": ""}
    if cid == "robonix/service/map/save_map":
        return {"ok": True, "map_id": args.get("map_id", ""), "detail": "saved"}
    if cid == "robonix/service/map/list_maps":
        return {"maps": []}
    if cid.startswith("robonix/service/map/"):
        return {"ok": True, "detail": ""}
    if cid == "robonix/service/speech/list_speakers":
        return {"speakers": [{"provider_id": "audio_driver", "namespace": "robonix/primitive/audio", "description": "host speaker"}]}
    if cid == "robonix/service/speech/speak":
        return {"ok": True, "detail": ""}
    if cid in ("robonix/service/memory/search", "robonix/service/memory/hybrid_search"):
        return {"results": {"data": "[]"}}
    if cid.startswith("robonix/service/memory/"):
        return {"confirmation": {"data": "saved"}}
    if cid == "robonix/skill/explore/explore":
        return {"run_id": f"exp-{int(time.time() * 1000)}", "state": "RUNNING"}
    if cid == "robonix/skill/explore/status":
        return {"run_id": args.get("run_id", ""), "state": "SUCCEEDED", "frontiers_left": 0, "elapsed_s": 60.0}
    if cid.startswith("robonix/skill/explore/"):
        return {"ok": True}
    if cid == "robonix/service/voiceprint/list":
        return {"speakers": []}
    return {"ok": True, "detail": ""}


def handler(provider_id: str, cid: str, schema: dict):
    class Input:
        @staticmethod
        def json_schema() -> dict:
            return schema

        @staticmethod
        def from_dict(d: dict) -> dict:
            return {k: v for k, v in d.items() if v is not None}

    def call(req: Input) -> dict:
        start = time.time()
        execution_id = uuid.uuid4().hex
        record = {"provider": provider_id, "contract": cid, "args": req, "case": state().get("case"), "start": start,
                  "execution_id": execution_id}
        try:
            out = answer(cid, req)
            record.update(ok=True)
            # The executor result identifies this invocation even when Pilot's
            # own context reads use the same provider, contract and arguments.
            return {**out, "_bench_execution_id": execution_id}
        except Exception as e:  # the leaf fails the way a real provider's would
            record.update(ok=False, error=str(e))
            raise RuntimeError(f"{e} [bench_execution_id={execution_id}]") from e
        finally:
            record["end"] = time.time()
            with _lock, CALLS.open("a") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")

    call.__annotations__ = {"req": Input, "return": dict}
    return call


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--provider", required=True)
    a = ap.parse_args()
    spec = CATALOG[a.provider]
    kind = spec["namespace"].split("/")[1]
    cls = {"primitive": Primitive, "skill": Skill}.get(kind, Service)
    provider = cls(id=a.provider, namespace=spec["namespace"])
    for c in spec["contracts"]:
        provider.mcp(c["contract_id"], description=c["description"])(handler(a.provider, c["contract_id"], c["input_schema"]))

    @provider.on_init
    def _init(_cfg):
        return Ok()

    if cls is Skill:

        @provider.on_activate
        def _activate():
            return Ok()

    provider.run()


if __name__ == "__main__":
    main()
