"""Real Atlas/Executor verification checks used by smoke_mcp."""
import asyncio
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
from unittest.mock import patch

import grpc
import atlas_pb2 as pb
import atlas_pb2_grpc
import pilot_pb2
import robonix_contracts_pb2_grpc as contracts

from vlm_verifier.directory import CameraDirectory
from vlm_verifier.camera import CameraPool


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


async def ready(endpoint):
    async with grpc.aio.insecure_channel(endpoint) as channel:
        await asyncio.wait_for(channel.channel_ready(), 10)


async def check(entry, mcp_port, model):
    """Assert one terminal result per node, original output, and fail-closed errors."""
    root = Path(__file__).resolve().parents[4]
    atlas_bin = os.environ.get("VLM_TEST_ATLAS_BIN", str(Path.home()/".cargo/bin/robonix-atlas"))
    executor_bin = os.environ.get("VLM_TEST_EXECUTOR_BIN", str(root/"target/debug/robonix-executor"))
    atlas_endpoint = f"127.0.0.1:{free_port()}"
    executor_endpoint = f"127.0.0.1:{free_port()}"
    processes = []
    with tempfile.TemporaryDirectory(prefix="vlm-executor-") as work:
        with open(Path(work)/"process.log", "w+") as log:
            try:
                processes.append(subprocess.Popen([
                    atlas_bin, "--listen", atlas_endpoint, "--capabilities", str(root/"capabilities"),
                ], stdout=log, stderr=log))
                await ready(atlas_endpoint)
                async with grpc.aio.insecure_channel(atlas_endpoint) as atlas_channel:
                    atlas = atlas_pb2_grpc.AtlasStub(atlas_channel)
                    await atlas.RegisterPrimitive(pb.RegisterRequest(
                        id="wrist", namespace="robonix/primitive/camera"))
                    await atlas.RegisterService(pb.RegisterRequest(
                        id="vlm_verifier", namespace="robonix/service/verifier"))
                    await atlas.DeclareCapability(pb.DeclareCapabilityRequest(
                        provider_id="wrist", contract_id="robonix/primitive/camera/rgb",
                        transport=pb.TRANSPORT_ROS2, endpoint="/verifier_test/rgb",
                        params=pb.TransportParams(ros2=pb.Ros2Params())))
                    await atlas.DeclareCapability(pb.DeclareCapabilityRequest(
                        provider_id="vlm_verifier", contract_id="robonix/service/verifier/verify",
                        transport=pb.TRANSPORT_MCP, endpoint=f"http://127.0.0.1:{mcp_port}/mcp",
                        params=pb.TransportParams(mcp=pb.McpParams())))
                    for provider in ("wrist", "vlm_verifier"):
                        await atlas.DeclareCapability(pb.DeclareCapabilityRequest(
                            provider_id=provider, contract_id="robonix/lifecycle/driver",
                            transport=pb.TRANSPORT_GRPC, endpoint=f"127.0.0.1:{free_port()}",
                            params=pb.TransportParams(grpc=pb.GrpcParams(
                                service_name="robonix.contracts.RobonixLifecycleDriver",
                                method="/robonix.contracts.RobonixLifecycleDriver/Driver"))))
                        await atlas.SetLifecycleState(pb.SetLifecycleStateRequest(
                            id=provider, state=pb.STATE_INACTIVE))
                        await atlas.SetLifecycleState(pb.SetLifecycleStateRequest(
                            id=provider, state=pb.STATE_ACTIVE))
                    config = {"verification": [{
                        "target_contract_id": "robonix/system/executor/builtin/run_command",
                        "target_provider_id": "executor",
                        "verifier_provider_id": "vlm_verifier",
                        "verifier_args": {"camera_provider_id": "wrist"},
                    }]}
                    processes.append(subprocess.Popen([
                        executor_bin, "--atlas", atlas_endpoint, "--listen", executor_endpoint,
                        "--id", "executor", "--config-json", json.dumps(config),
                    ], stdout=log, stderr=log))
                    await ready(executor_endpoint)
                    with patch.dict(os.environ, {"ROBONIX_ATLAS": atlas_endpoint}), patch.object(entry, "ATLAS", CameraDirectory()):
                        async with grpc.aio.insecure_channel(executor_endpoint) as channel:
                            stub = contracts.RobonixSystemExecutorExecuteStub(channel)
                            for mode in ("passed", "rejected", "malformed", "missing_camera"):
                                model.verdict = (
                                    '{"passed":"true","detail":"invalid"}' if mode == "malformed"
                                    else {"passed": mode == "passed", "detail": "visual evidence"})
                                if mode == "missing_camera":
                                    entry.CAMERAS.close()
                                    entry.CAMERAS = CameraPool(lambda: entry.ATLAS, entry.service.id)
                                    await atlas.Unregister(pb.UnregisterRequest(id="wrist"))
                                plan = pilot_pb2.Plan(
                                    plan_id=mode, session_id="vlm-smoke", root_index=0,
                                    nodes=[pilot_pb2.RtdlNode(
                                        node_kind=2, op_id="pick", description="Pick up the red object",
                                        call=pilot_pb2.CapabilityCall(
                                            call_id=mode+":pick", provider_id="executor",
                                            contract_id="robonix/system/executor/builtin/run_command",
                                            args_json=json.dumps({"command": "printf original-marker"}),
                                        ))])
                                terminal = []
                                async for event in stub.Execute(plan, timeout=20):
                                    if event.HasField("node_state"):
                                        ns = event.node_state
                                        if ns.state in (2, 3, 4):
                                            terminal.append(ns)
                                assert len(terminal) == 1, terminal
                                result = terminal[0].leaf_result
                                assert result.success is (mode == "passed"), result
                                assert "original-marker" in result.output, result
                                if mode == "rejected":
                                    assert result.error.startswith("result verification failed:"), result
                                if mode in ("malformed", "missing_camera"):
                                    assert result.error.startswith("result verification unavailable:"), result
                    print("PASS: real Atlas + Executor + VLM Verifier; pass/reject/unavailable, one terminal, original output")
            except BaseException:
                log.flush()
                log.seek(0)
                print(log.read()[-6000:])
                raise
            finally:
                for process in reversed(processes):
                    process.terminate()
                for process in reversed(processes):
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
