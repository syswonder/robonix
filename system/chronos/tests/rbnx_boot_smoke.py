# SPDX-License-Identifier: MulanPSL-2.0
"""Opt-in real rbnx boot/shutdown smoke with a late, stationary ROS camera.

Run after building the workspace, from a sourced ROS environment, with a Python
interpreter providing grpcio, grpcio-tools and PyYAML. No Minecraft is started.
"""
from pathlib import Path
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

import grpc
import grpc_tools
import yaml


def port():
    """Choose an unused loopback port without touching existing deployments."""
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        return reservation.getsockname()[1]


def wait(predicate, timeout=15):
    """Bound readiness waits and keep transient state checks separate from assertions."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(0.05)
    raise TimeoutError("smoke readiness condition did not complete")


def run():
    """Boot without a camera, activate it later, verify streams and shut down owned processes."""
    root = Path(__file__).resolve().parents[3]
    artifacts = Path(tempfile.mkdtemp(prefix="chronos-rbnx-boot-"))
    print(f"Artifacts: {artifacts}", flush=True)
    stubs = artifacts / "stubs"
    stubs.mkdir()
    generated = max(root.glob("target/debug/build/robonix-chronos-*/out/robonix_contracts.proto"), key=lambda path: path.stat().st_mtime_ns).parent
    includes = [f"-I{generated}", f"-I{Path(grpc_tools.__file__).parent / '_proto'}"]
    subprocess.run([sys.executable, "-m", "grpc_tools.protoc", *includes,
                    f"--python_out={stubs}", f"--grpc_python_out={stubs}",
                    *map(str, generated.glob("*.proto"))], check=True)
    subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{root / 'system/atlas/proto'}",
                    f"--python_out={stubs}", f"--grpc_python_out={stubs}",
                    str(root / "system/atlas/proto/atlas.proto")], check=True)
    sys.path.insert(0, str(stubs))
    import atlas_pb2 as a
    import atlas_pb2_grpc as ag
    import robonix_contracts_pb2_grpc as contracts
    from google.protobuf.empty_pb2 import Empty

    atlas_address = f"127.0.0.1:{port()}"
    chronos_address = f"127.0.0.1:{port()}"
    config = yaml.safe_load((root / "system/chronos/examples/minecraft-camera.yaml").read_text())["chronos"]
    config.update(listen=chronos_address, provider_id="chronos-boot-test")
    manifest = artifacts / "robonix_manifest.yaml"
    manifest.write_text(yaml.safe_dump(dict(manifestVersion=1, name="chronos-boot-test",
        system=dict(atlas=dict(listen=atlas_address), chronos=config))))
    environment = os.environ.copy()
    environment["PATH"] = f"{root / 'target/debug'}:{environment['PATH']}"
    environment["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    environment["SCRIBE_LOG_DIR"] = str(artifacts / "logs")
    rbnx = str(root / "target/debug/rbnx")
    boot_log = (artifacts / "boot.log").open("w")
    shutdown_log = (artifacts / "shutdown.log").open("w")
    boot = subprocess.Popen([rbnx, "boot", "-f", str(manifest)], cwd=artifacts,
                            env=environment, stdout=boot_log, stderr=boot_log)
    publisher = None
    streams = []
    channel = grpc.insecure_channel(atlas_address)
    connection = None
    failures = []
    cache = {}
    lock = threading.Lock()
    threads = []
    try:
        grpc.channel_ready_future(channel).result(timeout=10)
        atlas = ag.AtlasStub(channel)

        def active():
            assert boot.poll() is None, "rbnx exited during boot"
            providers = atlas.Query(a.QueryRequest(id="chronos-boot-test"), timeout=2).providers
            return providers and providers[0].state == a.STATE_ACTIVE

        wait(active)
        wait(lambda: "component(s) up" in (artifacts / "boot.log").read_text())
        connection = grpc.insecure_channel(chronos_address)
        quality = contracts.RobonixSystemChronosQualityStub(connection)
        models = contracts.RobonixSystemChronosConversionModelsStub(connection)

        def consume(stream, name, stopping):
            """Keep latest snapshots and expose unexpected stream failures to the main thread."""
            try:
                for snapshot in stream:
                    with lock:
                        cache[name] = snapshot
                if not stopping.is_set():
                    failures.append(f"{name} stream ended unexpectedly")
            except grpc.RpcError as error:
                if not stopping.is_set() or error.code() not in (grpc.StatusCode.CANCELLED, grpc.StatusCode.UNAVAILABLE):
                    failures.append(str(error))

        stopping = threading.Event()
        for stream, name in [(quality.Quality(Empty(), timeout=60), "quality"),
                             (models.ConversionModels(Empty(), timeout=60), "models")]:
            streams.append(stream)
            thread = threading.Thread(target=consume, args=(stream, name, stopping), daemon=True)
            thread.start()
            threads.append(thread)
        wait(lambda: "models" in cache and "quality" in cache)
        assert not cache["quality"].reference.verified
        assert all(not domain.has_model for domain in cache["models"].domains)

        driver = contracts.RobonixLifecycleDriverStub(connection)
        import lifecycle_pb2
        stopping.set()
        for command in [2, 1]:
            reply = driver.Driver(lifecycle_pb2.Driver_Request(command=command, config_json="{}"), timeout=3)
            assert reply.ok, reply.error
        for stream in streams:
            stream.cancel()
        streams.clear()
        for thread in threads:
            thread.join(timeout=2)
        threads.clear()
        with lock:
            cache.clear()
        stopping = threading.Event()
        for stream, name in [(quality.Quality(Empty(), timeout=60), "quality"),
                             (models.ConversionModels(Empty(), timeout=60), "models")]:
            streams.append(stream)
            thread = threading.Thread(target=consume, args=(stream, name, stopping), daemon=True)
            thread.start()
            threads.append(thread)
        wait(lambda: "models" in cache and "quality" in cache)
        assert not failures, failures

        topic = f"/chronos_boot_smoke_{os.getpid()}"
        atlas.RegisterPrimitive(a.RegisterRequest(id="minecraft_mod", namespace="robonix/primitive/camera"), timeout=2)
        atlas.DeclareCapability(a.DeclareCapabilityRequest(provider_id="minecraft_mod", contract_id="robonix/lifecycle/driver",
            transport=a.TRANSPORT_GRPC, endpoint="127.0.0.1:1", params=a.TransportParams(grpc=a.GrpcParams(
                proto_file="capabilities/lifecycle/driver.v1.toml", service_name="robonix.contracts.RobonixLifecycleDriver",
                method="/robonix.contracts.RobonixLifecycleDriver/Driver"))), timeout=2)
        atlas.DeclareCapability(a.DeclareCapabilityRequest(provider_id="minecraft_mod", contract_id="robonix/primitive/camera/rgb",
            transport=a.TRANSPORT_ROS2, endpoint=topic, params=a.TransportParams(ros2=a.Ros2Params(qos_profile="reliable"))), timeout=2)
        atlas.SetLifecycleState(a.SetLifecycleStateRequest(id="minecraft_mod", state=a.STATE_INACTIVE), timeout=2)
        time.sleep(0.5)
        inspected = json.loads(atlas.InspectAtlas(a.InspectAtlasRequest(), timeout=2).json)
        assert not any(edge["consumer_id"] == "chronos-boot-test" for edge in inspected["channels"].values())
        atlas.SetLifecycleState(a.SetLifecycleStateRequest(id="minecraft_mod", state=a.STATE_ACTIVE), timeout=2)
        publisher = subprocess.Popen(["python3", str(root / "system/chronos/tests/collector_publisher.py"), topic], env=environment,
                                     stdout=boot_log, stderr=boot_log)

        def fitted():
            assert boot.poll() is None and publisher.poll() is None
            assert not failures, failures
            with lock:
                domains = cache["models"].domains
                return any(domain.has_model and domain.state == 1 and domain.model.generation == domain.generation for domain in domains)

        wait(fitted)
        state_path = artifacts / "rbnx-boot/state.json"
        state = json.loads(state_path.read_text())
        owned_pids = [component["pid"] for component in state["components"]]
        chronos_pid = next(component["pid"] for component in state["components"] if component["name"] == "chronos")
        helper_pids = []
        for task in Path(f"/proc/{chronos_pid}/task").iterdir():
            helper_pids.extend(map(int, (task / "children").read_text().split()))
        assert len(set(helper_pids)) == 1
        (artifacts / "state-before-shutdown.json").write_text(json.dumps(state, indent=2))
        assert not failures, failures
        stopping.set()
        result = subprocess.run([rbnx, "shutdown", "-f", str(manifest)], cwd=artifacts, env=environment,
                                stdout=shutdown_log, stderr=shutdown_log, timeout=30)
        assert result.returncode == 0
        assert boot.wait(timeout=15) == 0
        assert not state_path.exists()
        assert all(not Path(f"/proc/{pid}").exists() for pid in owned_pids + helper_pids)
        print("PASS: rbnx boot -> quality before device -> waiting deactivate/reactivate -> late ACTIVE camera -> model -> rbnx shutdown", flush=True)
    finally:
        if "stopping" in locals():
            stopping.set()
        for stream in streams:
            stream.cancel()
        for thread in threads:
            thread.join(timeout=2)
        if publisher is not None:
            publisher.terminate()
            try:
                publisher.wait(timeout=3)
            except subprocess.TimeoutExpired:
                publisher.kill()
                publisher.wait()
        if boot.poll() is None:
            boot.send_signal(signal.SIGTERM)
            try:
                boot.wait(timeout=30)
            except subprocess.TimeoutExpired:
                boot.kill()
                boot.wait()
        if connection is not None:
            connection.close()
        channel.close()
        boot_log.close()
        shutdown_log.close()


if __name__ == "__main__":
    run()
