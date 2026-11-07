# SPDX-License-Identifier: MulanPSL-2.0
from concurrent.futures import ThreadPoolExecutor
from contextlib import redirect_stderr
import io
import subprocess
import sys
from types import SimpleNamespace
import unittest

import grpc
import atlas_pb2
import lifecycle_pb2

from runtime import close_runtime


class RuntimeCleanupTest(unittest.TestCase):
    def test_failed_shutdown_still_disconnects_and_reaps_process(self):
        disconnected = []

        def shutdown(request, context):
            context.abort(grpc.StatusCode.UNAVAILABLE, "provider already exited")

        def disconnect(request, context):
            disconnected.append(request.channel_id)
            context.abort(grpc.StatusCode.UNAVAILABLE, "binding already closed")

        server = grpc.server(ThreadPoolExecutor(max_workers=1))
        server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler("test", {
            "Shutdown": grpc.unary_unary_rpc_method_handler(
                shutdown, request_deserializer=lifecycle_pb2.Driver_Request.FromString),
            "Disconnect": grpc.unary_unary_rpc_method_handler(
                disconnect, request_deserializer=atlas_pb2.DisconnectCapabilityRequest.FromString),
        }),))
        port = server.add_insecure_port("127.0.0.1:0")
        server.start()
        channel = grpc.insecure_channel(f"127.0.0.1:{port}", options=[("grpc.enable_http_proxy", 0)])
        driver = SimpleNamespace(Driver=channel.unary_unary(
            "/test/Shutdown", request_serializer=lifecycle_pb2.Driver_Request.SerializeToString))
        atlas = SimpleNamespace(DisconnectCapability=channel.unary_unary(
            "/test/Disconnect", request_serializer=atlas_pb2.DisconnectCapabilityRequest.SerializeToString))
        process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        errors = io.StringIO()
        try:
            with redirect_stderr(errors):
                close_runtime(process, [channel], driver, atlas,
                              [SimpleNamespace(channel_id="one"), SimpleNamespace(channel_id="two")])
            self.assertIsNotNone(process.poll())
            self.assertEqual(disconnected, ["one", "two"])
            self.assertIn("Provider cleanup", errors.getvalue())
            self.assertIn("Binding cleanup", errors.getvalue())
            with self.assertRaises(ValueError):
                driver.Driver(lifecycle_pb2.Driver_Request(command=3), timeout=1)
        finally:
            channel.close()
            server.stop(0).wait()
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
