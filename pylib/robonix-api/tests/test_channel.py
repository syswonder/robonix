# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import asyncio
import concurrent.futures
import json
import os
from pathlib import Path
from types import SimpleNamespace
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

from robonix_api import Channel, Transport
from robonix_api.atlas import _Atlas

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).with_name("mcp_server.py")
ECHO = "robonix/service/map/get_mode"
NAV = "robonix/service/navigation/navigate"
SPEED_LIMIT = "robonix/service/navigation/get_speed_limit"


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_port(process, port):
    for _ in range(100):
        if process.poll() is not None:
            raise RuntimeError(f"test process exited: {process.returncode}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                return
        except OSError:
            time.sleep(0.05)
    raise RuntimeError("test server did not start")


class McpDecodeTest(unittest.TestCase):
    def decode(self, text, structured=None):
        from robonix_api.channel_mcp import _decode
        return _decode(SimpleNamespace(
            content=[SimpleNamespace(type="text", text=text)],
            isError=False, structuredContent=structured,
        ))

    def test_non_finite_floats_from_python_servers(self):
        # FastMCP writes LaserScan ranges beyond range_max as Infinity in text
        # and as null in its wrapped structured output.
        text = '{"header": {"frame_id": "hokuyo"}, "ranges": [0.5, Infinity, NaN]}'
        for structured in (None, {"result": {"header": {"frame_id": "hokuyo"}, "ranges": [0.5, None, None]}}):
            with self.subTest(structured=structured is not None):
                value = self.decode(text, structured)
                self.assertEqual(value["header"], {"frame_id": "hokuyo"})
                self.assertEqual(value["ranges"][:2], [0.5, float("inf")])
                self.assertNotEqual(value["ranges"][2], value["ranges"][2])


class ChannelLimitTest(unittest.TestCase):
    def limit(self, value):
        from robonix_api.channel import _channel_limit
        env = {} if value is None else {"ROBONIX_ATLAS_CHANNEL_LIMIT": value}
        with mock.patch.dict(os.environ, env, clear=False):
            if value is None:
                os.environ.pop("ROBONIX_ATLAS_CHANNEL_LIMIT", None)
            return _channel_limit()

    def test_default_and_override(self):
        self.assertEqual(self.limit(None), 1024)
        self.assertEqual(self.limit(" 64 "), 64)

    def test_invalid_values_fall_back(self):
        for value in ("", "abc", "2", "-5"):
            with self.subTest(value=value), self.assertLogs("robonix_api.channel", "WARNING"):
                self.assertEqual(self.limit(value), 1024)


@unittest.skipUnless((ROOT / "target/debug/robonix-atlas").exists(), "build Atlas and codegen first")
class ChannelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.work = Path(cls.temp.name)
        cls.control = cls.work / "control.json"
        cls.control.write_text("{}")
        legacy = cls.work / "contracts"
        legacy.mkdir()
        (legacy / "driver.toml").write_text(
            '[contract]\nid="robonix/skill/channel_test/driver"\nversion="1"\nkind="skill"\n'
            'idl="lifecycle/srv/Driver.srv"\n[mode]\ntype="rpc"\n')
        subprocess.run([str(ROOT / "target/debug/robonix-codegen"), "--lang", "mcp",
                        "-I", str(ROOT / "capabilities/lib"), "-o", str(cls.work / "types")],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc",
                        f"-I{ROOT / 'system/atlas/proto'}", f"--python_out={cls.work / 'types'}",
                        f"--grpc_python_out={cls.work / 'types'}", "atlas.proto"], check=True)
        sys.path.insert(0, str(cls.work / "types"))
        atlas_port, mcp_port = free_port(), free_port()
        cls.log = (cls.work / "servers.log").open("w")
        cls.atlas_process = subprocess.Popen([
            str(ROOT / "target/debug/robonix-atlas"), "--listen", f"127.0.0.1:{atlas_port}",
            "--capabilities", f"{ROOT / 'capabilities'},{legacy}",
        ], stdout=cls.log, stderr=cls.log)
        cls.mcp_process = subprocess.Popen([
            sys.executable, str(FIXTURE), "--port", str(mcp_port), "--control", str(cls.control),
        ], stdout=cls.log, stderr=cls.log, env={**os.environ,
            "PYTHONPATH": os.pathsep.join(str(p) for p in (
                cls.work / "types", ROOT / "pylib/robonix-api", os.environ.get("PYTHONPATH", ""),
            ) if p),
        })
        wait_port(cls.atlas_process, atlas_port)
        wait_port(cls.mcp_process, mcp_port)
        cls.atlas_address = f"127.0.0.1:{atlas_port}"
        cls.atlas = _Atlas(cls.atlas_address)
        cls.endpoint = f"http://127.0.0.1:{mcp_port}/mcp"
        cls.atlas.register_service("channel-test", "robonix/service")
        cls.atlas.declare_capability("channel-test", ECHO, Transport.MCP, cls.endpoint)
        for cid in (NAV, NAV + "/status", NAV + "/cancel", SPEED_LIMIT):
            cls.atlas.declare_capability("channel-test", cid, Transport.MCP, cls.endpoint + "/")

    @classmethod
    def tearDownClass(cls):
        cls.control.write_text('{"finish": true}')
        for ch in list(cls.atlas._channel_client.channels.values()):
            try:
                ch.close()
            except RuntimeError:
                pass
        cls.mcp_process.terminate()
        cls.atlas_process.terminate()
        cls.mcp_process.wait(timeout=10)
        cls.atlas_process.wait(timeout=10)
        cls.atlas._channel_client.stop_portal()
        cls.atlas._channel.close()
        cls.log.close()
        sys.path.remove(str(cls.work / "types"))
        cls.temp.cleanup()

    def setUp(self):
        self.configure()

    def configure(self, **values):
        pending = self.control.with_suffix(".pending")
        pending.write_text(json.dumps(values))
        pending.replace(self.control)

    def connect(self, cid=ECHO, provider="channel-test", transport=Transport.MCP):
        return self.atlas.connect_channel(consumer_id="test-consumer", provider_id=provider,
                                         contract_id=cid, transport=transport)

    def channel_exists(self, channel):
        return channel.channel_id in self.atlas.inspect()["channels"]

    @classmethod
    def start_grpc(cls):
        if hasattr(cls, "grpc_process"):
            return
        proto = cls.work / "proto"
        subprocess.run([str(ROOT / "target/debug/robonix-codegen"), "--lang", "proto",
                        "-I", str(ROOT / "capabilities/lib"), "--contracts", str(ROOT / "capabilities"),
                        "-o", str(proto)], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{proto}",
                        f"--python_out={cls.work / 'types'}", f"--grpc_python_out={cls.work / 'types'}",
                        *map(str, proto.glob("*.proto"))], check=True)
        port = free_port()
        cls.grpc_process = subprocess.Popen([
            sys.executable, str(FIXTURE.with_name("grpc_server.py")), "--port", str(port),
            "--control", str(cls.control), "--types", str(cls.work / "types"), "--atlas", cls.atlas._endpoint,
        ], stdout=cls.log, stderr=cls.log)
        wait_port(cls.grpc_process, port)
        cls.addClassCleanup(cls.grpc_process.wait, timeout=10)
        cls.addClassCleanup(cls.grpc_process.terminate)
        cls.atlas.register_service("grpc-test", "robonix/service")
        from robonix_api.atlas_types import GrpcParams
        import robonix_contracts_pb2 as contracts
        from robonix_api.lifecycle import contract_id_to_pascal
        for cid in (ECHO, NAV, NAV + "/status", NAV + "/cancel",
                    "robonix/service/speech/tts_stream", "robonix/service/speech/wake_word",
                    "robonix/service/speech/asr_stream", "robonix/primitive/audio/mic", "robonix/primitive/audio/speaker"):
            service = contracts.DESCRIPTOR.services_by_name[contract_id_to_pascal(cid)]
            method = service.methods[0]
            cls.atlas.declare_capability("grpc-test", cid, Transport.GRPC, f"127.0.0.1:{port}",
                params=GrpcParams(service_name=service.full_name, method=f"/{service.full_name}/{method.name}"))

    def grpc_channel(self, cid=ECHO):
        self.start_grpc()
        return self.connect(cid, provider="grpc-test", transport=Transport.GRPC)

    def test_grpc_calls_and_execution_lifetime(self):
        from google.protobuf.message import Message
        with self.grpc_channel() as channel:
            from map_pb2 import GetMode_Request
            with self.assertRaises(TypeError):
                channel.call(42)
            self.assertIsInstance(channel.call(GetMode_Request()), Message)
            self.assertEqual(channel.raw_response["mode"], "mapping")
            with self.assertRaisesRegex(RuntimeError, "only one"):
                channel.call({})
        with self.grpc_channel(NAV) as channel:
            self.assertTrue(channel.call({}).accepted)
            self.assertEqual(channel.run_id, "grpc-run")
            self.assertEqual(channel.status().state, "RUNNING")
            self.assertTrue(channel.cancel().accepted)
            self.assertEqual(channel.status().state, "CANCELED")
        channel = self.grpc_channel(NAV)
        channel.call({})
        self.configure(status_error=True)
        with self.assertRaisesRegex(RuntimeError, "downstream chassis busy"):
            channel.status(timeout=0.2)
        self.assertTrue(self.channel_exists(channel))
        self.configure(finish=True)
        channel.status()
        channel.close()
        channel = self.grpc_channel()
        self.configure(delay=0.2)
        with self.assertRaisesRegex(RuntimeError, "retained"):
            channel.call({}, timeout=0.01)
        self.assertIs(self.atlas.get_channel(channel.channel_id), channel)
        channel.close(timeout=2)
        self.assertFalse(self.channel_exists(channel))
        self.configure(grpc_error=True)
        channel = self.grpc_channel()
        with self.assertRaisesRegex(RuntimeError, "downstream chassis busy"):
            channel.call({})
        with self.assertRaisesRegex(RuntimeError, "unconfirmed"):
            channel.close(timeout=0)
        self.assertTrue(self.channel_exists(channel))

    def test_grpc_setup_binding_failure_releases_allocation(self):
        from robonix_api.atlas_types import GrpcParams
        with self.grpc_channel() as channel:
            endpoint = channel.endpoint.replace("127.0.0.1", "localhost")
        self.atlas.register_service("grpc-bad-binding", "robonix/service")
        self.atlas.declare_capability("grpc-bad-binding", ECHO, Transport.GRPC, endpoint,
            params=GrpcParams(service_name="robonix.contracts.Missing", method="Missing"))
        before = set(self.atlas.inspect()["channels"])
        with self.assertRaises(KeyError):
            self.connect(provider="grpc-bad-binding", transport=Transport.GRPC)
        self.assertEqual(set(self.atlas.inspect()["channels"]), before)

    def test_grpc_stream_shapes_and_close(self):
        with self.grpc_channel("robonix/service/speech/tts_stream") as channel:
            rows = list(channel.call({"text": "hello"}))
            self.assertEqual([x.chunk.sequence for x in rows], [0, 1, 2])
            self.assertEqual(rows[0].chunk.data, b"\x01\x02")
        with self.grpc_channel("robonix/service/speech/wake_word") as channel:
            from audio_pb2 import AudioChunk
            self.assertEqual(channel.call(iter([AudioChunk(sequence=4), {"sequence": 7}])).keyword, "[4, 7]")
        with self.grpc_channel("robonix/service/speech/asr_stream") as channel:
            rows = list(channel.call(iter([{"chunk": {"sequence": 4}}, {"chunk": {"sequence": 7}}])))
            self.assertEqual([x.text for x in rows], ["4", "7"])
        with self.grpc_channel("robonix/primitive/audio/mic") as channel:
            self.assertEqual(len(list(channel.call({}))), 3)
        with self.grpc_channel("robonix/primitive/audio/speaker") as channel:
            channel.call(iter([{"sequence": 1}]))
        self.configure(stream_count=8, stream_delay=0.05)
        channel = self.grpc_channel("robonix/service/speech/tts_stream")
        rows = channel.call({})
        next(rows)
        with self.assertRaisesRegex(RuntimeError, "unconfirmed"):
            channel.close(timeout=0.01)
        channel.close(timeout=2)
        self.assertFalse(self.channel_exists(channel))

    def test_grpc_close_retains_blocked_request_iterator(self):
        import threading
        entered, release = threading.Event(), threading.Event()
        def source():
            yield {"sequence": 1}
            entered.set()
            release.wait(3)
            yield {"sequence": 2}
        channel = self.grpc_channel("robonix/service/speech/wake_word")
        with concurrent.futures.ThreadPoolExecutor() as pool:
            future = pool.submit(channel.call, source())
            try:
                self.assertTrue(entered.wait(2))
                with self.assertRaisesRegex(RuntimeError, "iterator is still blocked"):
                    channel.close(timeout=0.01)
                self.assertIs(self.atlas.get_channel(channel.channel_id), channel)
            finally:
                release.set()
            future.result(timeout=2)
            channel.close()

    def test_grpc_early_response_retains_blocked_source(self):
        import threading
        entered, release = threading.Event(), threading.Event()
        def source():
            yield {"sequence": 1}
            entered.set()
            release.wait(3)
            yield {"sequence": 2}
        self.configure(early_return=True)
        channel = self.grpc_channel("robonix/service/speech/wake_word")
        try:
            self.assertEqual(channel.call(source()).keyword, "early")
            self.assertTrue(entered.is_set())
            with self.assertRaisesRegex(RuntimeError, "iterator is still blocked"):
                channel.close(timeout=0.01)
            self.assertIs(self.atlas.get_channel(channel.channel_id), channel)
        finally:
            release.set()
        for _ in range(100):
            if not channel._execution.protocol.source_active.is_set():
                break
            time.sleep(0.01)
        channel.close()
        self.assertFalse(self.channel_exists(channel))

    def test_executor_skill_activation(self):
        self.start_grpc()
        port = free_port()
        process = subprocess.Popen([
            sys.executable, str(FIXTURE.with_name("grpc_server.py")), "--port", str(port),
            "--control", str(self.control), "--types", str(self.work / "types"), "--atlas", self.atlas._endpoint,
        ], stdout=self.log, stderr=self.log)
        self.addCleanup(process.wait, timeout=10)
        self.addCleanup(process.terminate)
        wait_port(process, port)
        endpoint = f"127.0.0.1:{port}"
        subprocess.run(["cargo", "test", "-p", "robonix-executor", "dispatch::tests::skill_activation",
                        "--", "--ignored"], cwd=ROOT, check=True, env={**os.environ,
            "ROBONIX_TEST_ATLAS": self.atlas._endpoint, "ROBONIX_TEST_GRPC": endpoint,
            "ROBONIX_TEST_CONTROL": str(self.control)})

    def test_rust_binding_generation(self):
        project = self.work / "rust-bindings"
        contracts = project / "contracts"
        contracts.mkdir(parents=True)
        for name, idl in (
            ("uuid", "unique_identifier_msgs/msg/UUID.msg"),
            ("rgbd", "camera/msg/RGBD.msg"),
            ("uint32", "common_interfaces/std_msgs/msg/UInt32.msg"),
        ):
            (contracts / f"{name}.toml").write_text(
                f'[contract]\nid="robonix/primitive/{name}"\nversion="1"\nkind="primitive"\n'
                f'idl=" {idl} "\n[mode]\ntype=" topic_out "\n'
            )
        env = self.prepare_rust_bindings(project, contracts)
        (project / "src").mkdir()
        (project / "src/main.rs").write_text(
            '#![allow(dead_code)]\n'
            'include!(concat!(env!("OUT_DIR"), "/contract_proto_modules.rs"));\nfn main() {}\n'
        )
        subprocess.run(["cargo", "check", "--offline", "--manifest-path", str(project / "Cargo.toml")],
                       cwd=ROOT, check=True, env=env)

    def prepare_rust_bindings(self, project, contracts):
        project.mkdir(exist_ok=True)
        proto = project / "proto"
        subprocess.run([
            str(ROOT / "target/debug/robonix-codegen"), "--lang", "proto",
            "-I", str(ROOT / "capabilities/lib"), "--contracts", str(contracts), "-o", str(proto),
        ], check=True, stdout=subprocess.DEVNULL)
        (project / "Cargo.toml").write_text(
            '[package]\nname="robonix-channel-codegen-test"\nversion="0.0.0"\nedition="2024"\n'
            '[workspace]\n[dependencies]\n'
            f'robonix-atlas={{path={json.dumps(str(ROOT / "system/atlas"))}}}\n'
            'anyhow="1"\nprost="0.14"\ntonic="0.14"\ntonic-prost="0.14"\ntokio-stream="0.1"\n'
            '[dev-dependencies]\ntokio={version="1",features=["macros","rt","time"]}\n'
            '[build-dependencies]\ntonic-prost-build="0.14"\nprotoc-bin-vendored="3.2"\n'
        )
        (project / "build.rs").write_text('''use std::path::PathBuf;
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let proto = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?).join("proto");
    let files: Vec<_> = std::fs::read_dir(&proto)?.map(|entry| entry.map(|entry| entry.path()))
        .collect::<Result<Vec<_>, _>>()?.into_iter()
        .filter(|path| path.extension().is_some_and(|extension| extension == "proto")).collect();
    unsafe { std::env::set_var("PROTOC", protoc_bin_vendored::protoc_bin_path()?); }
    tonic_prost_build::configure().compile_protos(&files, &[proto.clone()])?;
    std::fs::copy(proto.join("contract_proto_modules.rs"),
        PathBuf::from(std::env::var("OUT_DIR")?).join("contract_proto_modules.rs"))?;
    Ok(())
}
''')
        # Keep the compiler fixture on the workspace's resolved dependency versions.
        (project / "Cargo.lock").write_bytes((ROOT / "Cargo.lock").read_bytes())
        return {**os.environ, "CARGO_TARGET_DIR": os.environ.get("CARGO_TARGET_DIR", str(ROOT / "target"))}

    def test_rust_grpc_channels(self):
        self.start_grpc()
        project = self.work / "rust-grpc"
        env = self.prepare_rust_bindings(project, ROOT / "capabilities")
        (project / "tests").mkdir()
        (project / "tests/channel_grpc.rs").write_bytes(Path(__file__).with_name("channel_grpc.rs").read_bytes())
        subprocess.run(["cargo", "test", "--offline", "--manifest-path", str(project / "Cargo.toml"),
                        "--test", "channel_grpc"], cwd=ROOT, check=True,
                       env={**env, "ROBONIX_TEST_ATLAS": self.atlas._endpoint,
                            "ROBONIX_TEST_CONTROL": str(self.control)})

    def test_scene_verifier_consumer(self):
        sys.path.insert(0, str(ROOT / "services/verifiers/scene_verifier"))
        self.addCleanup(sys.path.remove, str(ROOT / "services/verifiers/scene_verifier"))
        from scene_verifier.scene_client import SCENE_CONTRACT, fetch_robot_context

        before = set(self.atlas.inspect()["channels"])
        self.atlas.declare_capability("channel-test", SCENE_CONTRACT, Transport.MCP, self.endpoint)
        with mock.patch("robonix_api.ATLAS", self.atlas):
            for text_only in (False, True):
                self.configure(text_only=text_only)
                result = asyncio.run(fetch_robot_context("verifier", "channel-test", 1))
                self.assertTrue(result.pose_known)
                self.assertEqual((result.map_id, result.x, result.y), ("floor-3", 1.0, 2.0))
                self.assertEqual(set(self.atlas.inspect()["channels"]), before)

            valid = {"pose_known": True, "stale": False, "map_id": "floor-3",
                     "x": 0.0, "y": 0.0, "yaw": 0.0, "reason": "current"}
            for text_only in (False, True):
                for malformed in (
                    {key: value for key, value in valid.items() if key != "stale"},
                    {**valid, "pose_known": "false"},
                    {**valid, "x": "0.0"},
                ):
                    self.configure(text_only=text_only, scene_response=malformed)
                    with self.assertRaises(ValueError):
                        asyncio.run(fetch_robot_context("verifier", "channel-test", 1))
                    self.assertEqual(set(self.atlas.inspect()["channels"]), before)

            self.configure(text_only=True, scene_text=json.dumps(valid).replace(
                '"stale": false', '"stale": true, "stale": false'))
            with self.assertRaisesRegex(RuntimeError, "Duplicate JSON key"):
                asyncio.run(fetch_robot_context("verifier", "channel-test", 1))
            self.assertEqual(set(self.atlas.inspect()["channels"]), before)
            # NaN reads as null, which the pose check then rejects.
            self.configure(text_only=True, scene_text=json.dumps(valid).replace('"x": 0.0', '"x": NaN'))
            with self.assertRaises(ValueError):
                asyncio.run(fetch_robot_context("verifier", "channel-test", 1))
            self.assertEqual(set(self.atlas.inspect()["channels"]), before)

            self.configure(scene_response={"result": valid})
            self.assertTrue(asyncio.run(fetch_robot_context("verifier", "channel-test", 1)).pose_known)

    def test_process_exits_after_mcp_call(self):
        script = (
            "import sys\n"
            "from robonix_api import Transport\n"
            "from robonix_api.atlas import _Atlas\n"
            f"atlas = _Atlas({self.atlas_address!r})\n"
            "with atlas.connect_channel(consumer_id='exit-test', provider_id='channel-test',\n"
            f"        contract_id={ECHO!r}, transport=Transport.MCP) as channel:\n"
            "    channel.call({})\n"
        )
        env = {**os.environ, "PYTHONPATH": os.pathsep.join(str(p) for p in (
            self.work / "types", ROOT / "pylib/robonix-api", os.environ.get("PYTHONPATH", ""),
        ) if p)}
        subprocess.run([sys.executable, "-c", script], env=env, check=True, timeout=30)

    def test_nav_stress_consumer(self):
        sys.path.insert(0, str(ROOT / "testing"))
        self.addCleanup(sys.path.remove, str(ROOT / "testing"))
        import nav_stress
        from nav_stress import wait_terminal

        before = set(self.atlas.inspect()["channels"])
        with mock.patch("robonix_api.ATLAS", self.atlas):
            world = SimpleNamespace(costmap=object(), map=None, collisions=[], count=0)
            self.configure(finish=True)
            trace = self.control.with_suffix(".trace")
            start = len(trace.read_text().splitlines()) if trace.exists() else 0
            with mock.patch.object(nav_stress, "World", return_value=world), \
                 mock.patch.object(nav_stress, "candidates", return_value=[(1.0, 2.0), (4.0, 5.0)]), \
                 mock.patch.object(self.atlas, "connect_capability", side_effect=AssertionError("legacy MCP call")), \
                 mock.patch.object(sys, "argv", ["nav_stress.py", "--goals", "2", "--explore-s", "0"]):
                self.assertEqual(nav_stress.main(), 0)
            calls = [json.loads(line)["name"] for line in trace.read_text().splitlines()[start:]]
            self.assertEqual(calls, ["navigate", "status", "navigate", "status"])
            self.assertEqual(set(self.atlas.inspect()["channels"]), before)

        for legacy in (False, True):
            self.configure(legacy=legacy, finish=True)
            with self.connect(NAV) as channel:
                self.assertTrue(channel.call({}).accepted)
                self.assertTrue(self.channel_exists(channel))
                self.assertEqual(wait_terminal(channel, 1)["state"], "SUCCEEDED")
            self.assertFalse(self.channel_exists(channel))

        self.configure(ignore_cancel=True)
        channel = self.connect(NAV)
        channel.call({})
        self.assertEqual(wait_terminal(channel, 0)["state"], "TIMEOUT_LOCAL")
        with self.assertRaisesRegex(RuntimeError, "Timed out"):
            channel.close(timeout=0.5)
        self.assertIs(self.atlas.get_channel(channel.channel_id), channel)
        self.assertTrue(self.channel_exists(channel))
        self.configure(finish=True)
        wait_terminal(channel, 1)
        channel.close()
        self.assertFalse(self.channel_exists(channel))

    def test_sync_types_text_and_legacy_interface(self):
        from map_mcp import GetMode_Request, GetMode_Response
        for text_only in (False, True):
            self.configure(text_only=text_only)
            with self.connect() as channel:
                self.assertIsInstance(channel, Channel)
                self.assertIsNone(channel.raw_response)
                response = channel.call(GetMode_Request())
                self.assertIsInstance(response, GetMode_Response)
                self.assertEqual(response.mode, "mapping")
                raw = channel.raw_response
                raw["mode"] = "changed"
                self.assertEqual(channel.raw_response["mode"], "mapping")
                with self.assertRaisesRegex(RuntimeError, "only one"):
                    channel.call({})
            self.assertFalse(self.channel_exists(channel))
            channel.close()
        with self.atlas.connect_capability(consumer_id="old", provider_id="channel-test",
                                            contract_id=ECHO, transport="mcp") as old:
            self.assertEqual(old.endpoint, self.endpoint)
        self.assertFalse(self.channel_exists(old))

    def test_generated_response_does_not_depend_on_tool_catalog(self):
        with mock.patch("mcp.client.session.ClientSession.list_tools", side_effect=RuntimeError("catalog unavailable")):
            with self.connect() as channel:
                self.assertEqual(channel.call({}).mode, "mapping")

    def test_contract_tool_through_legacy_mcp_url(self):
        with self.connect(SPEED_LIMIT) as channel:
            response = channel.call({})
            self.assertTrue(response.available)
            self.assertEqual(response.max_linear_speed_mps, 0.26)
        self.assertFalse(self.channel_exists(channel))

    def test_async_lifetime_cancel_and_legacy_run_id(self):
        for legacy in (False, True):
            self.configure(legacy=legacy, cancel_delay=0.1)
            channel = self.connect(NAV)
            channel.call({})
            self.assertTrue(self.channel_exists(channel))
            self.assertEqual(channel.run_id, "" if legacy else "run-1")
            self.assertEqual(channel.status().state, "RUNNING")
            self.assertTrue(channel.cancel().accepted)
            self.assertTrue(self.channel_exists(channel))
            channel.close()
            self.assertFalse(self.channel_exists(channel))
            self.assertIsNone(self.atlas.get_channel(channel.channel_id))

    def test_uncertain_close_and_invalid_status_can_recover(self):
        self.configure(ignore_cancel=True, invalid_status=True)
        channel = self.connect(NAV)
        channel.call({})
        with self.assertRaisesRegex(RuntimeError, "Invalid capability state"):
            channel.status()
        self.configure(ignore_cancel=True)
        with self.assertRaisesRegex(RuntimeError, "Timed out"):
            channel.close(timeout=0.15)
        self.assertTrue(self.channel_exists(channel))
        self.assertIs(self.atlas.get_channel(channel.channel_id), channel)
        self.configure(finish=True)
        self.assertEqual(channel.status().state, "SUCCEEDED")
        channel.close()
        self.assertFalse(self.channel_exists(channel))

    def test_rejected_start_does_not_cancel_another_run(self):
        self.configure(reject=True)
        channel = self.connect(NAV)
        self.assertFalse(channel.call({}).accepted)
        self.assertIsNone(channel.run_id)
        with self.assertRaisesRegex(RuntimeError, "no confirmed async execution"):
            channel.cancel()
        channel.close()
        self.assertFalse(self.channel_exists(channel))

    def test_setup_failure_and_rejected_transport_do_not_allocate(self):
        before = len(self.atlas.inspect()["channels"])
        with self.assertRaisesRegex(ValueError, "Specify"):
            self.connect(transport=Transport.UNSPECIFIED)
        self.atlas.register_service("offline", "robonix/service")
        self.atlas.declare_capability("offline", ECHO, Transport.MCP,
                                      f"http://127.0.0.1:{free_port()}/mcp")
        with self.assertRaises(Exception):
            self.connect(provider="offline")
        self.assertEqual(len(self.atlas.inspect()["channels"]), before)
        self.atlas.register_service("partial", "robonix/service")
        partial_endpoint = f"http://127.0.0.1:{free_port()}/mcp"
        for cid in (NAV, NAV + "/status"):
            self.atlas.declare_capability("partial", cid, Transport.MCP, partial_endpoint)
        with self.assertRaisesRegex(ValueError, "both /status and /cancel"):
            self.connect(NAV, provider="partial")
        import map_mcp
        with mock.patch.object(map_mcp, "_SRV_TYPES", {}), self.assertRaisesRegex(RuntimeError, "regenerate"):
            self.connect()
        self.assertEqual(len(self.atlas.inspect()["channels"]), before)
        with mock.patch.object(self.atlas._wire_stub, "DisconnectCapability", side_effect=RuntimeError("Atlas unavailable")):
            with self.assertRaisesRegex(RuntimeError, "cleanup failed"):
                self.connect(provider="offline")
        retained = [ch for ch in self.atlas._channel_client.channels.values() if ch.provider_id == "offline"]
        self.assertEqual(len(retained), 1)
        self.assertTrue(self.channel_exists(retained[0]))
        retained[0].close()
        self.assertEqual(len(self.atlas.inspect()["channels"]), before)

    def test_concurrent_start_and_timeout_retain_execution(self):
        self.configure(delay=0.2)
        channel = self.connect()
        with concurrent.futures.ThreadPoolExecutor() as pool:
            call = pool.submit(channel.call, {})
            time.sleep(0.05)
            with self.assertRaisesRegex(RuntimeError, "only one"):
                channel.call({})
            call.result(timeout=5)
        channel.close()
        channel = self.connect()
        with self.assertRaisesRegex(RuntimeError, "retained"):
            channel.call({}, timeout=0.02)
        with self.assertRaisesRegex(RuntimeError, "unconfirmed"):
            channel.close()
        self.assertTrue(self.channel_exists(channel))
        from robonix_api import Service
        provider = Service(id="cleanup-consumer", namespace="robonix/service", pkg_root=self.work)
        idle = self.connect()
        provider._channels = [channel, idle]
        with self.assertLogs("robonix_api.capability", level="ERROR"):
            provider._teardown()
        self.assertEqual(provider._channels, [channel])
        self.assertFalse(self.channel_exists(idle))

    def test_tool_error_and_status_failure_preserve_causes(self):
        self.configure(tool_error=True)
        channel = self.connect()
        with self.assertRaisesRegex(RuntimeError, "FAILED_PRECONDITION: chassis busy"):
            channel.call({})
        channel.close()
        self.assertFalse(self.channel_exists(channel))
        channel = self.connect(NAV)
        with self.assertRaisesRegex(RuntimeError, "MCP tool error"):
            channel.call({"goal": "invalid"})
        channel.close()
        self.assertFalse(self.channel_exists(channel))
        self.configure(status_error=True)
        channel = self.connect(NAV)
        channel.call({})
        with self.assertRaisesRegex(RuntimeError, "status unavailable"):
            channel.status()
        self.assertTrue(self.channel_exists(channel))
        self.configure(finish=True)
        channel.status()
        channel.close()
        self.configure(malformed_status=True)
        channel = self.connect(NAV)
        channel.call({})
        before = len(self.atlas.inspect()["channels"])
        for _ in range(3):
            with self.assertRaisesRegex(RuntimeError, "failed"):
                channel.status()
        self.assertEqual(len(self.atlas.inspect()["channels"]), before)
        self.configure(finish=True)
        channel.status()
        channel.close()

    @unittest.skipUnless((ROOT / "target/debug/robonix-executor").exists(), "build Executor first")
    def test_executor_cancellation_race_and_uncertain_termination(self):
        import grpc
        sources = sorted((ROOT / "target/debug/build").glob("robonix-executor-*/out/robonix_contracts.proto"),
                         key=lambda path: path.stat().st_mtime, reverse=True)
        proto = sources[0].parent
        subprocess.run([sys.executable, "-m", "grpc_tools.protoc", f"-I{proto}",
                        f"--python_out={self.work / 'types'}", f"--grpc_python_out={self.work / 'types'}",
                        *map(str, proto.glob("*.proto"))], check=True, stderr=subprocess.DEVNULL)
        import executor_pb2
        import pilot_pb2
        import robonix_contracts_pb2_grpc as contracts
        port = free_port()
        process = subprocess.Popen([
            str(ROOT / "target/debug/robonix-executor"), "--listen", f"127.0.0.1:{port}",
            "--atlas", self.atlas._endpoint, "--id", "channel-executor",
        ], stdout=self.log, stderr=self.log)
        try:
            wait_port(process, port)
            with grpc.insecure_channel(f"127.0.0.1:{port}", options=[("grpc.enable_http_proxy", 0)]) as wire:
                execute = contracts.RobonixSystemExecutorExecuteStub(wire)
                control = contracts.RobonixSystemExecutorControlPlanStub(wire)

                def plan(identifier, args_json="{}"):
                    return pilot_pb2.Plan(plan_id=identifier, session_id="channel-test", root_index=0,
                        nodes=[pilot_pb2.RtdlNode(node_kind=2, op_id="nav", description="test navigation", call=pilot_pb2.CapabilityCall(
                            call_id=identifier, provider_id="channel-test", contract_id=NAV, args_json=args_json))])

                def held():
                    return [c for c in self.atlas.inspect()["channels"].values()
                            if c["consumer_id"] == "channel-executor" and c["contract_id"] == NAV]

                with concurrent.futures.ThreadPoolExecutor() as pool:
                    for start_delay in (0, 0.3):
                        self.configure(start_delay=start_delay, cancel_delay=0.1)
                        identifier = f"cancel-{start_delay}"
                        events = pool.submit(list, execute.Execute(plan(identifier), timeout=15))
                        for _ in range(100):
                            if held():
                                break
                            time.sleep(0.02)
                        self.assertTrue(held())
                        # Cancel while the start request can still be awaiting its response.
                        response = control.ControlPlan(executor_pb2.ControlPlan_Request(
                            action="cancel", plan_id=identifier, wait_ms=6000), timeout=12)
                        self.assertTrue(response.completed, response)
                        rows = events.result(timeout=12)
                        terminals = [e.node_state for e in rows if e.HasField("node_state")]
                        self.assertEqual(terminals[-1].state, 4)
                        self.assertFalse(held())
                    self.configure(reject=True)
                    rows = list(execute.Execute(plan("rejected-start"), timeout=10))
                    terminals = [e.node_state for e in rows if e.HasField("node_state")]
                    self.assertEqual(terminals[-1].state, 3)
                    self.assertFalse(held())
                    self.configure()
                    rows = list(execute.Execute(plan("tool-error", '{"goal":"invalid"}'), timeout=10))
                    errors = [e.node_state.leaf_result.error for e in rows if e.HasField("node_state")]
                    self.assertTrue(any("MCP tool error" in e for e in errors), errors)
                    self.assertFalse(held())
                    self.configure(finish=True)
                    rows = list(execute.Execute(plan("wrapped-result"), timeout=10))
                    terminals = [e.node_state for e in rows if e.HasField("node_state")]
                    self.assertEqual(terminals[-1].state, 2)
                    self.assertFalse(held())
                    self.configure(invalid_status=True)
                    rows = list(execute.Execute(plan("invalid-status"), timeout=10))
                    errors = [e.node_state.leaf_result.error for e in rows if e.HasField("node_state")]
                    self.assertTrue(any("unknown capability state" in e for e in errors), errors)
                    self.assertTrue(held(), "invalid status must retain the main allocation")
        finally:
            process.terminate()
            process.wait(timeout=10)
