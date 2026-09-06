"""Real ROS + MCP HTTP + fake model HTTP integration; no robot movement.

Run from package root with generated PYTHONPATH and ROS environment.
Uses an isolated ROS domain by default and a fake Atlas directory only.
"""
import asyncio
import base64
import io
import json
import os
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("ROS_DOMAIN_ID", "193")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

import rclpy
import uvicorn
from mcp import ClientSession
from mcp.client.streamable_http import streamablehttp_client
from PIL import Image as PillowImage
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image

from vlm_verifier import camera
from vlm_verifier import service as entry


class ModelHandler(BaseHTTPRequestHandler):
    verdict = {"passed": True, "detail": "target visibly held"}
    calls = 0

    def log_message(self, *args):
        pass

    def do_POST(self):
        """Validate actual model wire payload and return the chosen verdict."""
        data = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        assert self.path == "/v1/chat/completions"
        image_url = data["messages"][1]["content"][1]["image_url"]["url"]
        image = PillowImage.open(io.BytesIO(base64.b64decode(image_url.split(",")[1])))
        # The shared latest frame is red.
        red, green, blue = image.getpixel((0, 0))
        assert red > 240 and blue < 10
        type(self).calls += 1
        content = self.verdict if isinstance(self.verdict, str) else json.dumps(self.verdict)
        response = json.dumps({"choices": [{"message": {"content": content}}]}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)


async def main():
    """Exercise passed, rejected, malformed model, and missing-camera MCP results."""
    model = ThreadingHTTPServer(("127.0.0.1", 0), ModelHandler)
    model_thread = threading.Thread(target=model.serve_forever, daemon=True)
    model_thread.start()
    entry.init({"vlm": {"base_url": f"http://127.0.0.1:{model.server_port}/v1",
                        "api_key": "test-secret", "model": "fake-vision"}})
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node("verifier_test_camera", context=context)
    publisher = node.create_publisher(Image, "/verifier_test/rgb", 10)
    stopped = threading.Event()
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    def publish():
        """Continuously publish the frame retained by the verifier's shared cache."""
        msg = Image(height=4, width=4, encoding="rgb8", step=12,
                    data=[255, 0, 0] * 16)
        publisher.publish(msg)

    node.create_timer(0.05, publish)

    def spin():
        while not stopped.is_set():
            executor.spin_once(timeout_sec=0.05)

    ros_thread = threading.Thread(target=spin, daemon=True)
    ros_thread.start()
    channels = []
    atlas = Mock()

    def find(**kwargs):
        assert kwargs["provider_id"] in ("wrist", "missing")
        assert kwargs["contract_id"] == camera.RGB_CONTRACT
        if kwargs["provider_id"] == "missing":
            return []
        return [SimpleNamespace(provider_id="wrist", contract_id=camera.RGB_CONTRACT)]

    def connect(**kwargs):
        assert kwargs["provider_id"] == "wrist"
        channel = Mock(endpoint="/verifier_test/rgb")
        channels.append(channel)
        return channel

    atlas.find_capability.side_effect = find
    atlas.connect_capability.side_effect = connect
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    config = uvicorn.Config(entry.service._mcp_app.streamable_http_app(), log_level="error")
    server = uvicorn.Server(config)
    server_task = asyncio.create_task(server.serve(sockets=[sock]))
    try:
        while not server.started:
            if server_task.done():
                await server_task
                raise RuntimeError("MCP server did not start")
            await asyncio.sleep(0.02)
        with patch.object(entry, "ATLAS", atlas):
            async with streamablehttp_client(f"http://127.0.0.1:{port}/mcp") as (read, write, _):
                async with ClientSession(read, write) as client:
                    await client.initialize()

                    async def call(provider="wrist"):
                        """Call."""
                        payload = dict(
                            target_provider_id="pick", target_contract_id="robonix/skill/pick/pick",
                            target_description="Pick the red object",
                            target_args={"object_name": "red object"}, target_output={"success": True},
                            verifier_args={"camera_provider_id": provider},
                        )
                        return await client.call_tool("verify", {
                            "call_id": "smoke", "args_json": json.dumps(payload),
                        })

                    for passed in (True, False):
                        ModelHandler.verdict = {"passed": passed, "detail": "visual evidence"}
                        response = await call()
                        assert not response.isError, response
                        body = json.loads(response.content[0].text)
                        assert body["passed"] is passed, body
                    ModelHandler.verdict = '{"passed":"true","detail":"invalid"}'
                    assert (await call()).isError
                    before = ModelHandler.calls
                    assert (await call("missing")).isError
                    assert ModelHandler.calls == before
                    assert len(channels) == 1
                    assert channels[0].close.call_count == 0
                    print("PASS: shared latest ROS frame, MCP pass/reject/errors, HTTP VLM, channel reuse")
                    if os.environ.get("VLM_TEST_EXECUTOR_BIN"):
                        from executor_checks import check
                        entry.CAMERAS.close()
                        entry.CAMERAS = camera.CameraPool(lambda: entry.ATLAS, entry.service.id)
                        await check(entry, port, ModelHandler)
                    entry.shutdown()
                    assert channels[0].close.call_count == 1
    finally:
        server.should_exit = True
        await server_task
        sock.close()
        stopped.set()
        ros_thread.join(timeout=2)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
        model.shutdown()
        model.server_close()
        model_thread.join(timeout=2)


if __name__ == "__main__":
    asyncio.run(main())
