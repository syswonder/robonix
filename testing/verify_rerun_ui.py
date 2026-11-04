# SPDX-License-Identifier: MulanPSL-2.0
"""Keep real Scene viewers open across a supplied lifecycle command."""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import signal
import sys
import time
from pathlib import Path
from itertools import islice
from urllib.error import HTTPError, URLError
from urllib.request import urlopen
from urllib.parse import urlsplit


def bound_evidence(value, audit: dict, *, string_bytes=4096, items=256, depth=0, _budget=None):
    if _budget is None:
        _budget = [512 * 1024, 2048]
    if _budget[0] <= 0 or _budget[1] <= 0:
        audit["capacity_exhausted"] = True
        audit["total_budget_exhaustions"] = audit.get("total_budget_exhaustions", 0) + 1
        return None
    _budget[1] -= 1
    if isinstance(value, str):
        allowed = min(string_bytes, _budget[0])
        prefix = value[:allowed].encode("utf-8")[:allowed].decode("utf-8", "ignore")
        _budget[0] -= len(prefix.encode("utf-8"))
        if len(prefix) < len(value):
            audit["truncated_strings"] = audit.get("truncated_strings", 0) + 1
            audit["omitted_characters"] = audit.get("omitted_characters", 0) + len(value) - len(prefix)
        return prefix
    if isinstance(value, (dict, list, tuple)):
        if depth >= 8:
            audit["capacity_exhausted"] = True
            audit["depth_truncations"] = audit.get("depth_truncations", 0) + 1
            return "<depth limit>"
        if len(value) > items:
            audit["capacity_exhausted"] = True
            audit["omitted_items"] = audit.get("omitted_items", 0) + len(value) - items
        if isinstance(value, dict):
            return {bound_evidence(str(k), audit, string_bytes=string_bytes, _budget=_budget):
                    bound_evidence(v, audit, string_bytes=string_bytes,
                                   items=items, depth=depth + 1, _budget=_budget)
                    for k, v in islice(value.items(), items)}
        return [bound_evidence(v, audit, string_bytes=string_bytes, items=items,
                               depth=depth + 1, _budget=_budget)
                for v in value[:items]]
    return value


def retain_evidence(collection: list, value, audit: dict, name: str, *, limit=256):
    counts = audit.setdefault("collection_counts", {})
    counts[name] = counts.get(name, 0) + 1
    if len(collection) >= limit:
        audit["capacity_exhausted"] = True
        audit["omitted_items"] = audit.get("omitted_items", 0) + 1
        return
    collection.append(bound_evidence(value, audit))


def write_event(stream, event: dict, audit: dict, *, limit=64 * 1024 * 1024):
    before = audit.get("truncated_strings", 0)
    event = bound_evidence(event, audit)
    if audit.get("truncated_strings", 0) != before and not (
        event.get("kind") == "console" and event.get("level") in ("debug", "info", "log")
    ):
        audit["capacity_exhausted"] = True
    line = json.dumps(event) + "\n"
    size = len(line.encode("utf-8"))
    used = audit.get("log_bytes", 0)
    if used + size > limit:
        audit["capacity_exhausted"] = True
        audit["omitted_events"] = audit.get("omitted_events", 0) + 1
        return
    stream.write(line)
    stream.flush()
    audit["log_bytes"] = used + size


async def stop_owned_child(child, grace=5):
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            pass
        if sig == signal.SIGTERM:
            try:
                await asyncio.wait_for(child.wait(), grace)
            except asyncio.TimeoutError:
                pass
    await asyncio.wait_for(child.wait(), grace)


async def run_lifecycle(command, timeout, probe, summary, *, grace=5):
    child = await asyncio.create_subprocess_exec(*command, start_new_session=True)
    summary["child_pid"] = child.pid

    async def monitor():
        while child.returncode is None:
            if not await asyncio.to_thread(probe):
                summary["outage_seen"] = True
            try:
                await asyncio.wait_for(child.wait(), 0.25)
            except asyncio.TimeoutError:
                pass
        return child.returncode

    try:
        return await asyncio.wait_for(monitor(), timeout)
    except asyncio.TimeoutError as error:
        summary["lifecycle_timed_out"] = True
        raise RuntimeError(f"Lifecycle command exceeded {timeout:g}s") from error
    finally:
        await asyncio.shield(stop_owned_child(child, grace))
        summary["child_exit"] = child.returncode


# Observe the production instance without changing its renderer, data, or reconnect behaviour.
OBSERVER = """
const originalStart = WebViewer.prototype.start;
window.__sceneUITest = {token: crypto.randomUUID(), events: [], starts: 0, omittedEvents: 0};
WebViewer.prototype.start = function (...args) {
  const state = window.__sceneUITest;
  state.viewer = this;
  state.source = args[0];
  state.starts++;
  this.on('recording_open', event => {
    if (state.events.length < 256) state.events.push(event);
    else state.omittedEvents++;
  });
  return originalStart.apply(this, args);
};
"""
STATE = """() => {
  const s = window.__sceneUITest;
  if (!s?.viewer?.ready) return null;
  const id = s.viewer.get_active_recording_id();
  const range = id && s.viewer.get_time_range(id, 'scene');
  return {token: s.token, starts: s.starts, id, range, events: s.events, source: s.source,
          omitted_events: s.omittedEvents};
}"""
EXPORT = """async () => {
  const stream = window.__sceneUITest.viewer.save_recording();
  const reader = stream.getReader();
  const chunks = [];
  let size = 0;
  for (;;) {
    const {value, done} = await reader.read();
    if (done) break;
    size += value.byteLength;
    if (size > 64 * 1024 * 1024) {
      await reader.cancel();
      throw new Error('Viewer recording exceeds 64 MiB evidence bound');
    }
    chunks.push(value);
  }
  const a = document.createElement('a');
  const url = URL.createObjectURL(new Blob(chunks));
  a.href = url;
  a.download = 'viewer.rrd';
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}"""


def validate_pixels(path: Path, object_color: tuple | None = None) -> dict:
    from PIL import Image

    image = Image.open(path).convert("RGB")
    # Only the Rerun toolbar is excluded: geometry may be near any viewport edge.
    image = image.crop((0, 56, image.width, image.height))
    orange = floor = semantic = 0
    for (r, g, b) in image.get_flattened_data():
        orange += r > 150 and 55 < g < 210 and b < 115 and r > g * 1.2
        floor += 65 < r < 150 and r < g < b and b - r < 55
        if object_color:
            semantic += all(abs(actual - expected) <= 25
                            for actual, expected in zip((r, g, b), object_color))
    if orange < 8 or floor < 100:
        raise RuntimeError(f"Scene geometry not visible: robot_pixels={orange}, floor_pixels={floor}")
    if object_color and semantic < 8:
        raise RuntimeError(f"Scene semantic object not visible: object_pixels={semantic}")
    return {"robot_pixels": orange, "floor_pixels": floor, "object_pixels": semantic}


def validate_recording(path: Path, view: str, recording_id: str, objects: list) -> dict:
    from rerun.experimental import RrdReader

    recording = RrdReader(path)
    stores = recording.recordings()
    app_id = "robonix-scene" if view == "3d" else "robonix-scene-2d"
    if len(stores) != 1 or stores[0].recording_id != recording_id or stores[0].application_id != app_id:
        raise RuntimeError(f"Viewer exported the wrong Scene recording: {stores}")
    requirements = (
        (("/map/floor", "Mesh3D:vertex_positions"),
         ("/map/floor", "Mesh3D:triangle_indices"),
         ("/map/objects/sem_pcd/", "Points3D:positions"),
         ("/map/robot/heading", "Arrows3D:vectors"))
        if view == "3d" else
        (("/map2d/grid", "Image:buffer"),
         ("/map2d/objects/marker/", "Points2D:positions"),
         ("/map2d/robot/heading", "Arrows2D:vectors"))
    )
    found = set()
    candidates = {o["id"]: o for o in objects if o.get("settled")
                  and str(o.get("label", "")).strip().lower() != "robot"
                  and not o.get("is_robot") and not o.get("attributes", {}).get("is_robot")}
    matching_objects = {}
    object_prefix = "/map/objects/sem_pcd/" if view == "3d" else "/map2d/objects/marker/"
    object_component = "Points3D" if view == "3d" else "Points2D"
    for chunk in recording.stream():
        batch = chunk.to_record_batch()
        for prefix, component in requirements:
            if chunk.entity_path.startswith(prefix) and component in batch.schema.names:
                if any(batch.column(component).to_pylist()):
                    found.add((prefix, component))
        if chunk.entity_path.startswith(object_prefix):
            oid = chunk.entity_path.removeprefix(object_prefix)
            positions, colors = object_component + ":positions", object_component + ":colors"
            if oid in candidates and positions in batch.schema.names and colors in batch.schema.names:
                if any(batch.column(positions).to_pylist()):
                    rgba = next((row[0] for row in batch.column(colors).to_pylist() if row), None)
                    if rgba is not None:
                        matching_objects[oid] = (rgba >> 24, (rgba >> 16) & 255, (rgba >> 8) & 255)
    for prefix, component in requirements:
        if (prefix, component) not in found:
            raise RuntimeError(f"Viewer recording missing geometry: {prefix} {component}")
    if not matching_objects:
        raise RuntimeError("Viewer geometry does not match any settled nonrobot /api/state object")
    oid = sorted(matching_objects)[0]
    return {"geometry_columns": sorted(found), "bytes": path.stat().st_size,
            "object": candidates[oid], "object_color": matching_objects[oid]}


def recovered(before: dict, after: dict) -> bool:
    if before["page_token"] != after["page_token"]:
        raise RuntimeError("Scene page document restarted during lifecycle")
    return bool(after["id"] and after["id"] != before["id"]
                and any(e["recording_id"] == after["id"] for e in after["events"]))


def expected_outage_error(text: str, phase: str, source: str, location_url: str) -> bool:
    if phase != "lifecycle":
        return False
    origin = urlsplit(source.removeprefix("rerun+"))
    location = urlsplit(location_url)
    if location.scheme not in ("http", "https") or (
        location.scheme, location.hostname, location.port
    ) != (origin.scheme, origin.hostname, origin.port):
        return False
    if text in {
        "Failed to load resource: net::ERR_CONNECTION_REFUSED",
        "Failed to load resource: net::ERR_INCOMPLETE_CHUNKED_ENCODING",
    }:
        return True
    return text.strip() == (
        "ERROR re_viewer::app::logic: Data source failed: js api error: "
        f"TypeError: network error\n- Source: {source}"
    )


def scene_up(base_url: str) -> bool:
    try:
        with urlopen(base_url + "/api/state", timeout=1):
            return True
    except HTTPError as error:
        error.close()
        return True
    except (OSError, URLError):
        return False


def read_api(base_url: str, path: str):
    with urlopen(base_url + path, timeout=5) as response:
        raw = response.read(4 * 1024 * 1024 + 1)
        if len(raw) > 4 * 1024 * 1024:
            raise RuntimeError("Scene API evidence exceeds 4 MiB")
        return response.status, raw


async def run_evidence_worker(*args):
    child = await asyncio.create_subprocess_exec(
        sys.executable, str(Path(__file__).resolve()), *args,
        stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
        start_new_session=True)

    async def read_pipe(pipe, limit):
        data = bytearray()
        while chunk := await pipe.read(min(65536, limit + 1 - len(data))):
            data.extend(chunk)
            if len(data) > limit:
                raise RuntimeError("Evidence worker output capacity exhausted")
        return bytes(data)

    readers = [asyncio.create_task(read_pipe(child.stdout, 4 * 1024 * 1024)),
               asyncio.create_task(read_pipe(child.stderr, 64 * 1024))]
    try:
        stdout, stderr, _ = await asyncio.gather(*readers, child.wait())
        if child.returncode:
            raise RuntimeError(stderr.decode("utf-8", "replace")[:4096])
        return stdout
    finally:
        await asyncio.shield(stop_owned_child(child))
        for reader in readers:
            reader.cancel()
        await asyncio.gather(*readers, return_exceptions=True)


async def recording_proof(recording, scene, view, recording_id):
    raw = await run_evidence_worker("--recording-proof", str(recording), str(scene), view, recording_id)
    return json.loads(raw)


async def api_body(base_url, path, timeout=10):
    try:
        return await asyncio.wait_for(run_evidence_worker("--api-proof", base_url, path), timeout)
    except asyncio.TimeoutError as error:
        raise RuntimeError("Scene API evidence exceeded deadline") from error


async def observe_host(route):
    response = await route.fetch()
    body = await response.text()
    marker = 'import { WebViewer } from "/assets/rerun/index.js";'
    if body.count(marker) != 1:
        raise RuntimeError("Scene Rerun host import changed; observer cannot attach")
    await route.fulfill(response=response, body=body.replace(marker, marker + OBSERVER, 1))


async def wait_state(page, timeout: float, before: dict | None = None):
    from playwright.async_api import Error

    async def poll():
        origin = await page.evaluate("location.origin")
        while True:
            try:
                handle = await page.locator("iframe#v").element_handle()
                frame = await handle.content_frame() if handle else None
                state = await frame.evaluate(STATE) if frame else None
            except Error as error:
                if "Execution context was destroyed" not in str(error) and "Frame was detached" not in str(error):
                    raise
                await asyncio.sleep(0.25)
                continue
            if state:
                if state.get("omitted_events"):
                    raise RuntimeError("Viewer recording event evidence capacity exhausted")
                state["page_token"] = await page.evaluate("window.__sceneUITestPageToken")
            if state and state["id"]:
                if state["source"] != "rerun+" + origin + "/proxy":
                    raise RuntimeError(f"Viewer did not open Scene's real gRPC source: {state['source']}")
                if before is None or recovered(before, state):
                    return frame, state
            await asyncio.sleep(0.25)

    try:
        return await asyncio.wait_for(poll(), timeout)
    except asyncio.TimeoutError as error:
        raise RuntimeError("Viewer state exceeded phase deadline") from error


async def capture(page, frame, output: Path, label: str, view: str) -> dict:
    canvas = frame.locator("#host > canvas")
    await canvas.wait_for(state="visible")
    origin = urlsplit(page.url)._replace(path="", query="", fragment="").geturl()
    raw_scene = await api_body(origin, "/api/state")
    scene_path = output / f"{label}-{view}-state.json"
    scene_path.write_bytes(raw_scene)
    async with page.expect_download(timeout=60000) as pending:
        await frame.evaluate(EXPORT)
    download = await pending.value
    recording = output / f"{label}-{view}.rrd"
    await download.save_as(recording)
    state = await frame.evaluate(STATE)
    proof = await recording_proof(recording, scene_path, view, state["id"])
    await page.screenshot(path=str(output / f"{label}-{view}-unfocused.png"))
    if view == "3d":
        focus = await page.request.post("/api/viewer/focus", data={"object_id": proof["object"]["id"]})
        if focus.status != 200 or not (await focus.json()).get("ok"):
            raise RuntimeError("Scene's existing object focus action failed")
        await asyncio.sleep(1)
    await page.screenshot(path=str(output / f"{label}-{view}-page.png"))
    pixels = output / f"{label}-{view}-canvas.png"
    await canvas.screenshot(path=str(pixels))
    return {**proof, **validate_pixels(pixels, proof["object_color"])}


async def wait_proof(page, frame, output: Path, label: str, view: str, timeout: float) -> dict:
    deadline = time.monotonic() + timeout
    while True:
        try:
            return await asyncio.wait_for(capture(page, frame, output, label, view),
                                          max(0, deadline - time.monotonic()))
        except asyncio.TimeoutError as error:
            raise RuntimeError("Viewer proof exceeded phase deadline") from error
        except RuntimeError:
            if time.monotonic() >= deadline:
                raise
            await asyncio.sleep(min(1, max(0, deadline - time.monotonic())))


async def run(args) -> int:
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    events = (output / "browser.jsonl").open("w", encoding="utf-8")
    summary = {"passed": False, "command": args.command, "views": {}, "outage_seen": False}
    errors = []
    expected_errors = []
    responses = []
    pages = []
    audit = {}
    child_exit = None
    phase = "before"

    def record(kind, **fields):
        write_event(events, {"time": time.time(), "phase": phase, "kind": kind, **fields}, audit)

    def console(message):
        record("console", level=message.type, text=message.text, location=message.location)
        if message.type == "error":
            # The old viewer loses its stream during shutdown; a pass still requires an observed outage and recovery.
            expected = expected_outage_error(message.text, phase, "rerun+" + args.base_url + "/proxy",
                                             message.location.get("url", ""))
            if expected:
                retain_evidence(expected_errors, message.text, audit, "expected_outage_errors")
            else:
                retain_evidence(errors, message.text, audit, "browser_errors")

    def pageerror(error):
        record("pageerror", text=str(error))
        retain_evidence(errors, str(error), audit, "browser_errors")

    def response_received(response):
        entry = {"url": response.url, "status": response.status,
                 "frame": response.request.frame.url, "phase": phase}
        if "MessageProxyService" in response.url or "/assets/rerun/" in response.url:
            retain_evidence(responses, entry, audit, "responses")
        record("response", **entry)

    def check_network(view, after=False):
        entries = [r for r in responses if ("view=2d" in r["frame"]) == (view == "2d")
                   and r["status"] == 200 and (not after or r["phase"] != "before")]
        if not any("MessageProxyService" in r["url"] for r in entries):
            raise RuntimeError(f"No successful real gRPC-web response for {view} {'after restart' if after else ''}")
        if not after and not any(".wasm" in r["url"] for r in entries):
            raise RuntimeError(f"No successful real WASM load for {view}")
        if not after:
            for paths in (("/assets/rerun/index.js",),
                          ("/assets/rerun/re_viewer.js", "/assets/rerun/re_viewer")):
                if not any(urlsplit(r["url"]).path in paths for r in entries):
                    raise RuntimeError(f"No successful real JavaScript load for {view}: {paths}")

    try:
        from playwright.async_api import async_playwright

        async with async_playwright() as pw:
            browser = await pw.chromium.launch(headless=True, args=["--enable-unsafe-swiftshader"])
            try:
                context = await browser.new_context(base_url=args.base_url,
                                                    viewport={"width": 1440, "height": 1000},
                                                    accept_downloads=True)
                await context.add_init_script("window.__sceneUITestPageToken = crypto.randomUUID()")
                await context.route("**/rerun?*", observe_host)
                context.on("console", console)
                context.on("response", response_received)
                context.on("requestfailed", lambda request: record("requestfailed", url=request.url,
                                                                     failure=request.failure))
                for view, suffix in (("3d", "/"), ("2d", "/2d")):
                    page = await context.new_page()
                    page.on("pageerror", pageerror)
                    pages.append((view, page))
                    await page.goto(args.base_url + suffix, wait_until="domcontentloaded",
                                    timeout=int(args.timeout * 1000))
                    iframe = page.locator("iframe#v")
                    await iframe.wait_for(timeout=int(args.timeout * 1000))
                    frame, state = await wait_state(page, args.timeout)
                    proof = await wait_proof(page, frame, output, "before", view, args.timeout)
                    check_network(view)
                    summary["views"][view] = bound_evidence({"before": state, "before_proof": proof}, audit)
                    record("ready", view=view, state=state)
                if errors or audit.get("capacity_exhausted"):
                    raise RuntimeError(f"Browser errors before lifecycle: {errors}")
                phase = "lifecycle"
                # Inherit stdout/stderr unchanged so the existing lifecycle report parser still works.
                child_exit = await run_lifecycle(args.command, args.lifecycle_timeout,
                                                 lambda: scene_up(args.base_url), summary)
                phase = "after"
                if child_exit:
                    raise RuntimeError(f"Lifecycle command exited {child_exit}")
                if not summary["outage_seen"]:
                    raise RuntimeError("No Scene outage observed during lifecycle; restart not proven")
                for view, page in pages:
                    frame, state = await wait_state(page, args.timeout, summary["views"][view]["before"])
                    proof = await wait_proof(page, frame, output, "after", view, args.timeout)
                    check_network(view, after=True)
                    summary["views"][view].update(bound_evidence({"after": state, "after_proof": proof}, audit))
                if errors or audit.get("capacity_exhausted"):
                    raise RuntimeError(f"Browser errors: {errors}")
            finally:
                summary["browser_errors"] = errors
                summary["expected_outage_errors"] = expected_errors
                for view, page in pages:
                    try:
                        await page.screenshot(path=str(output / f"final-{view}.png"), timeout=10000)
                        iframe = await page.locator("iframe#v").element_handle()
                        frame = await iframe.content_frame() if iframe else None
                        final_state = await asyncio.wait_for(frame.evaluate(STATE), 5) if frame else None
                        if final_state:
                            final_state["page_token"] = await asyncio.wait_for(page.evaluate("window.__sceneUITestPageToken"), 5)
                        summary.setdefault("final_states", {})[view] = bound_evidence(final_state, audit)
                    except Exception as error:
                        record("artifact_error", view=view, text=str(error))
                try:
                    raw = await api_body(args.base_url, "/api/viewer")
                    summary["final_viewer_api"] = bound_evidence({"status": 200, "body": json.loads(raw)}, audit)
                except Exception as error:
                    record("artifact_error", text=str(error))
                await asyncio.wait_for(browser.close(), 10)
        if errors or audit.get("capacity_exhausted"):
            raise RuntimeError(f"Browser errors during finalization: {errors}")
        summary["passed"] = True
    except Exception as error:
        summary["passed"] = False
        summary["error"] = bound_evidence(str(error), audit)
        print(f"Rerun UI: FAIL: {summary['error']}", file=sys.stderr)
    finally:
        events.close()
        before = audit.get("truncated_strings", 0)
        summary = bound_evidence(summary, audit)
        if audit.get("truncated_strings", 0) != before:
            audit["capacity_exhausted"] = True
        if audit.get("capacity_exhausted"):
            summary["passed"] = False
            summary["capacity_failure"] = True
        summary["artifact_bounds"] = audit
        summary["raw_log_complete"] = not (audit.get("truncated_strings") or audit.get("omitted_events"))
        encoded = json.dumps(summary, indent=2) + "\n"
        if len(encoded.encode("utf-8")) > 4 * 1024 * 1024:
            summary = {"passed": False, "capacity_failure": True, "error": "Summary exceeds 4 MiB",
                       "artifact_bounds": audit}
            encoded = json.dumps(summary, indent=2) + "\n"
        (output / "summary.json").write_text(encoded)
    if summary.get("lifecycle_timed_out"):
        return 124
    if child_exit:
        return child_exit if child_exit > 0 else 128 - child_exit
    if summary["passed"]:
        print(f"Rerun UI: PASS; evidence: {output}", file=sys.stderr)
        return 0
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--lifecycle-timeout", type=float, default=1800)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.command[:1] == ["--"]:
        args.command = args.command[1:]
    if not args.command:
        parser.error("a lifecycle command after -- is required")
    if any(not math.isfinite(t) or t <= 0 for t in (args.timeout, args.lifecycle_timeout)):
        parser.error("timeouts must be finite and positive")
    args.base_url = args.base_url.rstrip("/")
    return asyncio.run(run(args))


if __name__ == "__main__":
    if sys.argv[1:2] == ["--api-proof"]:
        _, raw = read_api(*sys.argv[2:])
        sys.stdout.buffer.write(raw)
        sys.exit(0)
    if sys.argv[1:2] == ["--recording-proof"]:
        recording, scene, view, recording_id = sys.argv[2:]
        proof = validate_recording(Path(recording), view, recording_id,
                                   json.loads(Path(scene).read_bytes()).get("objects", []))
        print(json.dumps(proof))
        sys.exit(0)
    sys.exit(main())
