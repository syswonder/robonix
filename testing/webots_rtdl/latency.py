# SPDX-License-Identifier: MulanPSL-2.0
"""Compare live model streaming with complete-plan submission in Webots."""

import asyncio
import base64
import json
import os
import statistics
import sys
import time
import uuid
from pathlib import Path

OUT = Path(os.environ["RTDL_TEST_OUT"]).resolve()
sys.path.insert(0, str(OUT / "proto"))
import executor_pb2
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc as rpc
from google.protobuf.json_format import MessageToDict
from run import prompt


def append(path, value):
    with path.open("a") as log:
        log.write(json.dumps(value, ensure_ascii=False) + "\n")


def records(path):
    return (
        [json.loads(line) for line in path.read_text().splitlines()]
        if path.exists()
        else []
    )


def model_timing(model_events):
    request = next((e for e in model_events if e["kind"] == "request"), None)
    if request is None:
        return {}
    eof = next(
        (e for e in model_events if e["id"] == request["id"] and e["kind"] == "eof"),
        None,
    )
    pending = b""
    first_text = None
    for chunk in records(OUT / f"model-{request['id']}.chunks.jsonl"):
        pending += base64.b64decode(chunk["base64"])
        lines = pending.split(b"\n")
        pending = lines.pop()
        for line in lines:
            if not line.startswith(b"data: {"):
                continue
            data = json.loads(line[6:])
            if any(c.get("delta", {}).get("content") for c in data.get("choices", [])):
                first_text = chunk["ns"]
                break
        if first_text is not None:
            break
    return {
        "request_ns": request["ns"],
        "first_text_ns": first_text,
        "eof_ns": eof["ns"] if eof else None,
    }


async def relay():
    """Forward real upstream SSE immediately, recording bytes and monotonic times."""
    from aiohttp import ClientSession, ClientTimeout, web

    # Separate Pilot processes otherwise use different cache routing keys.
    cache_key = uuid.uuid4().hex

    async def forward(request):
        body = await request.read()
        tag = uuid.uuid4().hex
        stream = request.method == "POST"
        if stream:
            payload = json.loads(body)
            payload["prompt_cache_key"] = cache_key
            body = json.dumps(payload, ensure_ascii=False).encode()
            (OUT / f"model-{tag}.request.json").write_bytes(body)
            append(
                OUT / "model-events.jsonl",
                {"id": tag, "kind": "request", "ns": time.monotonic_ns()},
            )
        url = os.environ["VLM_BASE_URL"].rstrip("/") + "/" + request.match_info["path"]
        headers = {
            "Authorization": "Bearer " + os.environ["VLM_API_KEY"],
            "Content-Type": "application/json",
        }
        async with (
            ClientSession(timeout=ClientTimeout(total=240)) as session,
            session.request(
                request.method, url, data=body or None, headers=headers
            ) as upstream,
        ):
            response = web.StreamResponse(
                status=upstream.status,
                headers={
                    "Content-Type": upstream.headers.get(
                        "Content-Type", "application/json"
                    )
                },
            )
            await response.prepare(request)
            with (OUT / f"model-{tag}.chunks.jsonl").open("w") as log:
                async for chunk in upstream.content.iter_any():
                    stamp = time.monotonic_ns()
                    if stream:
                        log.write(
                            json.dumps(
                                {
                                    "ns": stamp,
                                    "base64": base64.b64encode(chunk).decode(),
                                }
                            )
                            + "\n"
                        )
                        log.flush()
                    await response.write(chunk)
            if stream:
                append(
                    OUT / "model-events.jsonl",
                    {
                        "id": tag,
                        "kind": "eof",
                        "ns": time.monotonic_ns(),
                        "status": upstream.status,
                    },
                )
            await response.write_eof()
            return response

    app = web.Application(client_max_size=64 * 1024 * 1024)
    app.router.add_route("*", "/v1/{path:.*}", forward)
    runner = web.AppRunner(app, handler_cancellation=True)
    await runner.setup()
    await web.TCPSite(runner, "127.0.0.1", 51180).start()
    print("Live upstream relay ready on 127.0.0.1:51180", flush=True)
    await asyncio.Event().wait()


async def trial(n, pair, mode, warmup=False):
    tag = f"{'warmup' if warmup else 'trial'}-{pair:02d}-n{n}-{mode}"
    event_path = OUT / f"{tag}.jsonl"
    if event_path.exists():
        raise RuntimeError(f"Refusing to overwrite {event_path}")
    task = pilot_pb2.Task(
        task_id=uuid.uuid4().hex,
        session_id=uuid.uuid4().hex,
        text=prompt(n),
        source=2,
        timestamp_ms=int(time.time() * 1000),
    )
    (OUT / f"{tag}.task.json").write_text(
        json.dumps(
            MessageToDict(task, preserving_proto_field_name=True),
            ensure_ascii=False,
            indent=2,
        )
    )
    events, error = [], None
    abort_ns, abort_error = None, None
    port = 51071 if mode == "streaming" else 51072
    async with grpc.aio.insecure_channel(
        f"127.0.0.1:{port}",
        options=[
            ("grpc.max_receive_message_length", 64 * 1024 * 1024),
            ("grpc.enable_http_proxy", 0),
        ],
    ) as channel:
        stub = rpc.RobonixSystemPilotStub(channel)
        start = time.monotonic_ns()
        try:
            await asyncio.wait_for(channel.channel_ready(), 15)
            start = time.monotonic_ns()
            async for event in stub.SubmitTask(task, timeout=420):
                data = MessageToDict(event, preserving_proto_field_name=True)
                events.append(data)
                append(event_path, {"ns": time.monotonic_ns(), "event": data})
                if (
                    event.HasField("node_state")
                    and event.node_state.node_index == 0
                    and event.node_state.state == 2
                    and abort_ns is None
                ):
                    # All plan work has finished; the next model summary is outside this measurement.
                    abort_ns = time.monotonic_ns()
                    stop = pilot_pb2.Task(
                        task_id=uuid.uuid4().hex,
                        session_id=task.session_id,
                        context_json='{"abort_turn":true}',
                        source=2,
                        timestamp_ms=int(time.time() * 1000),
                    )
                    try:
                        async for response in stub.SubmitTask(stop, timeout=20):
                            append(
                                OUT / f"{tag}.abort.jsonl",
                                {
                                    "ns": time.monotonic_ns(),
                                    "event": MessageToDict(
                                        response, preserving_proto_field_name=True
                                    ),
                                },
                            )
                    except grpc.RpcError as exc:
                        abort_error = str(exc)
        except (grpc.RpcError, asyncio.TimeoutError) as exc:
            error = str(exc)
    end = time.monotonic_ns()
    cleanup_error = None
    try:
        async with grpc.aio.insecure_channel(
            "127.0.0.1:50061", options=[("grpc.enable_http_proxy", 0)]
        ) as channel:
            active = await rpc.RobonixSystemExecutorListActivePlansStub(
                channel
            ).ListActivePlans(executor_pb2.ListActivePlans_Request(), timeout=5)
            if not active.success or json.loads(active.plans_json)["count"]:
                cleanup_error = "Executor still has active plans"
    except (grpc.RpcError, ValueError, KeyError) as exc:
        cleanup_error = str(exc)
    handlers = [
        e
        for e in records(Path(os.environ["RTDL_HANDLER_LOG"]))
        if start <= e["ns"] <= end
    ]
    plans = [e["plan"] for e in events if "plan" in e]
    leaves = [
        (p["plan_id"], x["op_id"])
        for p in plans
        for x in p.get("nodes", [])
        if x.get("node_kind") == 2
    ]
    terminal = [
        e["node_state"]
        for e in events
        if e.get("node_state", {}).get("node_kind") == 2
        and e["node_state"].get("state") in (2, 3, 4, 5)
    ]
    identities = [(e["plan_id"], e["op_id"]) for e in terminal]
    expected_handlers = ["lidar"] * n
    if n > 1:
        expected_handlers[-1] = "camera"
    nodes_succeeded = (
        len(plans) == 1
        and len(leaves) == n
        and len(identities) == n
        and set(identities) == set(leaves)
        and len(set(identities)) == n
        and all(e.get("state") == 2 for e in terminal)
        and [h["handler"] for h in handlers] == expected_handlers
    )
    execution_pass = (
        error is None
        and cleanup_error is None
        and abort_error is None
        and nodes_succeeded
    )
    task_done = any(e.get("task_state", {}).get("status") == "done" for e in events)
    model_events = [
        e for e in records(OUT / "model-events.jsonl") if start <= e["ns"] <= end
    ]
    timing = model_timing(model_events)
    first_action = handlers[0]["ns"] if handlers else None
    overlap = (
        first_action < timing["eof_ns"]
        if first_action and timing.get("eof_ns")
        else None
    )
    result = {
        "tag": tag,
        "n": n,
        "pair": pair,
        "mode": mode,
        "warmup": warmup,
        "start_ns": start,
        "end_ns": end,
        "pass_": execution_pass
        and abort_ns is not None
        and timing.get("eof_ns") is not None,
        "nodes_succeeded": nodes_succeeded,
        "task_done": task_done,
        "overlap": overlap,
        "error": error,
        "cleanup_error": cleanup_error,
        "abort_ns": abort_ns,
        "abort_error": abort_error,
        "plans": len(plans),
        "do_count": len(leaves),
        "terminal_count": len(terminal),
        "handlers": handlers,
        "model_events": model_events,
        "model_timing": timing,
        "first_action_s": (first_action - start) / 1e9 if first_action else None,
    }
    append(OUT / "latency-results.jsonl", result)
    print(
        json.dumps(
            {k: v for k, v in result.items() if k not in ("handlers", "model_events")},
            ensure_ascii=False,
        ),
        flush=True,
    )
    if cleanup_error or abort_error:
        raise RuntimeError(
            "Executor inspection failed; trial recorded, stopping before the next trial"
        )
    return result


def paired_statistics(rows):
    pairs = []
    for pair in sorted({r["pair"] for r in rows}):
        arms = {r["mode"]: r for r in rows if r["pair"] == pair}
        if len(arms) != 2 or not all(r["pass_"] for r in arms.values()):
            continue
        if any(
            not r["model_timing"].get("eof_ns")
            or not r["model_timing"].get("first_text_ns")
            for r in arms.values()
        ):
            continue
        streaming, buffered = (arms[m] for m in ("streaming", "buffered"))
        s, b = (r["first_action_s"] for r in (streaming, buffered))
        sf, bf = (
            (r["model_timing"]["first_text_ns"] - r["start_ns"]) / 1e9
            for r in (streaming, buffered)
        )
        pairs.append(
            {
                "pair": pair,
                "streaming": s,
                "buffered": b,
                "saving_s": b - s,
                "saving_percent": 100 * (b - s) / b,
                "streaming_first_text_s": sf,
                "buffered_first_text_s": bf,
                "saving_after_first_text_s": (b - bf) - (s - sf),
            }
        )
    stats = {}
    for key in (k for k in pairs[0] if k != "pair") if pairs else ():
        values = [p[key] for p in pairs]
        stats[key] = {
            "median": statistics.median(values),
            "quartiles": statistics.quantiles(values, n=4, method="inclusive")
            if len(values) > 1
            else None,
        }
    return {"count": len(pairs), "pairs": pairs, "stats": stats}


def summarize():
    rows = [r for r in records(OUT / "latency-results.jsonl") if not r["warmup"]]
    result = []
    for n in (1, 5, 10):
        group = [r for r in rows if r["n"] == n]
        result.append(
            {
                "n": n,
                "trials": len(group),
                "failed_trials": sum(not r["pass_"] for r in group),
                "rpc_errors": sum(r["error"] is not None for r in group),
                "task_done": sum(r["task_done"] for r in group),
                "successful_plans": sum(r["nodes_succeeded"] for r in group),
                "complete_plan_pairs": paired_statistics(group),
            }
        )
    (OUT / "latency-summary.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


async def run():
    for mode in ("streaming", "buffered"):
        existing = next(
            (
                r
                for r in records(OUT / "latency-results.jsonl")
                if r["warmup"] and r["mode"] == mode
            ),
            None,
        )
        result = existing or await trial(5, 0, mode, warmup=True)
        if not result.get("nodes_succeeded"):
            print(
                f"Warmup {mode} failed; retained in results, no automatic retry",
                flush=True,
            )
    for pair in range(3):
        for index, n in enumerate((1, 5, 10)):
            modes = (
                ("streaming", "buffered")
                if (pair + index) % 2 == 0
                else ("buffered", "streaming")
            )
            for mode in modes:
                await trial(n, pair, mode)
    summarize()


if __name__ == "__main__":
    if sys.argv[1:] == ["summarize"]:
        summarize()
    else:
        asyncio.run(relay() if sys.argv[1:] == ["relay"] else run())
