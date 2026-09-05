# SPDX-License-Identifier: MulanPSL-2.0
"""Request-local RGB observation through an exact Atlas camera provider."""
import asyncio
import threading
import time
import uuid

RGB_CONTRACT = "robonix/primitive/camera/rgb"


def resolve_camera(atlas, consumer_id, provider_id):
    """Require exactly one matching ROS RGB capability, with no topic fallback."""
    caps = atlas.find_capability(
        contract_id=RGB_CONTRACT, transport="ros2", provider_id=provider_id,
    )
    caps = [cap for cap in caps if cap.provider_id == provider_id and cap.contract_id == RGB_CONTRACT]
    if len(caps) != 1:
        raise RuntimeError("configured camera RGB capability missing or ambiguous")
    return atlas.connect_capability(
        consumer_id=consumer_id, provider_id=provider_id,
        contract_id=RGB_CONTRACT, transport="ros2",
    )


def encode_jpeg(message):
    """Validate dimensions/stride and encode standard 8-bit ROS images as JPEG."""
    import io
    from PIL import Image
    modes = {"rgb8": ("RGB", "RGB", 3), "bgr8": ("RGB", "BGR", 3),
             "rgba8": ("RGBA", "RGBA", 4), "bgra8": ("RGBA", "BGRA", 4),
             "mono8": ("L", "L", 1)}
    if message.encoding not in modes:
        raise ValueError("unsupported camera image encoding")
    mode, raw_mode, channels = modes[message.encoding]
    if message.width <= 0 or message.height <= 0 or message.step < message.width * channels:
        raise ValueError("camera returned invalid image dimensions or stride")
    if len(message.data) != message.height * message.step:
        raise ValueError("camera returned invalid image data length")
    frame = Image.frombytes(mode, (message.width, message.height), bytes(message.data),
                            "raw", raw_mode, message.step, 1).convert("RGB")
    output = io.BytesIO()
    frame.save(output, format="JPEG", quality=90)
    return output.getvalue()


def capture_sync(atlas, consumer_id, provider_id, stop):
    """Wait up to five seconds for a newly stamped frame; clean up on all exits."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image

    deadline = time.monotonic() + 5
    channel = None
    context = Context()
    node = executor = None
    try:
        channel = resolve_camera(atlas, consumer_id, provider_id)
        if stop.is_set() or time.monotonic() >= deadline:
            raise TimeoutError("camera observation timed out")
        rclpy.init(context=context)
        node = rclpy.create_node("vlm_verifier_" + uuid.uuid4().hex, context=context)
        started_ns = node.get_clock().now().nanoseconds
        frames = []

        def receive(message):
            """Ignore queued pre-request frames and retain only one new image."""
            stamp = message.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns >= started_ns and not frames:
                frames.append(message)

        node.create_subscription(
            Image, channel.endpoint, receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        while not stop.is_set() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.05, max(0, deadline - time.monotonic())))
            if frames:
                return encode_jpeg(frames[0])
        raise TimeoutError("camera observation timed out")
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        if channel is not None:
            channel.close()


async def capture(atlas, consumer_id, provider_id):
    """Run ROS work off the MCP event loop and signal cleanup on cancellation."""
    stop = threading.Event()
    try:
        return await asyncio.wait_for(
            asyncio.to_thread(capture_sync, atlas, consumer_id, provider_id, stop), 5,
        )
    finally:
        stop.set()
