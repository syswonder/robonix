# SPDX-License-Identifier: MulanPSL-2.0
"""Shared RGB subscriptions resolved through exact Atlas camera providers."""
import asyncio
from dataclasses import dataclass
import logging
import os
from pathlib import Path
import re
import threading
import time
from typing import Any, Callable

RGB_CONTRACT = "robonix/primitive/camera/rgb"

log = logging.getLogger("vlm_verifier.camera")


def save_verification_frame(call_id, jpeg, log_dir=None):
    """Persist the exact JPEG sent to the VLM and return its private file path.

    The deployment's Scribe directory keeps the diagnostic beside component
    logs. The sanitized call id cannot escape the frame directory, while a
    nanosecond timestamp prevents calls from overwriting frames across boots.
    """
    root = Path(log_dir or os.environ.get("SCRIBE_LOG_DIR", "rbnx-build/data"))
    frame_dir = root / "vlm_verifier-frames"
    frame_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    safe_call_id = re.sub(r"[^A-Za-z0-9._-]+", "_", call_id.strip())[:80] or "call"
    path = frame_dir / f"{time.time_ns()}_{safe_call_id}.jpg"
    with path.open("xb") as output:
        os.chmod(path, 0o600)
        output.write(jpeg)
    return path


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


@dataclass
class _CameraEntry:
    """Resources and latest frame retained for one exact camera provider."""

    channel: Any
    subscription: Any
    message: Any = None


class CameraPool:
    """Own one ROS node and one long-lived subscription per camera provider."""

    def __init__(self, atlas: Callable[[], Any], consumer_id: str):
        self._atlas = atlas
        self._consumer_id = consumer_id
        self._condition = threading.Condition()
        self._entries: dict[str, _CameraEntry] = {}
        self._context = None
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._stop = threading.Event()
        self._closed = False

    def _ensure_ros(self):
        """Start the shared ROS context, node, executor, and spin thread once."""
        if self._node is not None:
            return
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor

        self._context = Context()
        rclpy.init(context=self._context)
        self._node = rclpy.create_node("vlm_verifier_camera_pool", context=self._context)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._spin, name="vlm-verifier-camera", daemon=True,
        )
        self._spin_thread.start()

    def _spin(self):
        """Dispatch callbacks until service shutdown without owning request state."""
        while not self._stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.1)
            except Exception:
                if not self._stop.is_set():
                    log.exception("shared camera ROS spin loop stopped")
                return

    def _ensure_camera(self, provider_id):
        """Resolve and subscribe once, closing partial resources on failure."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Image

        with self._condition:
            if self._closed:
                raise RuntimeError("camera pool is closed")
            entry = self._entries.get(provider_id)
            if entry is not None:
                return entry
            self._ensure_ros()
            channel = resolve_camera(self._atlas(), self._consumer_id, provider_id)
            entry = _CameraEntry(channel=channel, subscription=None)

            def receive(message):
                """Replace this provider's cached frame and wake initial readers."""
                with self._condition:
                    entry.message = message
                    self._condition.notify_all()

            try:
                entry.subscription = self._node.create_subscription(
                    Image, channel.endpoint, receive,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE),
                )
            except Exception:
                channel.close()
                raise
            self._entries[provider_id] = entry
            return entry

    def capture_sync(self, provider_id, stop):
        """Return the cached latest frame, waiting only for the first delivery."""
        deadline = time.monotonic() + 5
        entry = self._ensure_camera(provider_id)
        with self._condition:
            while entry.message is None and not stop.is_set() and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(timeout=min(0.1, remaining))
            if entry.message is None:
                raise TimeoutError("camera observation timed out")
            message = entry.message
        return encode_jpeg(message)

    async def capture(self, provider_id):
        """Read the shared latest-frame cache without blocking the MCP event loop."""
        stop = threading.Event()
        try:
            return await asyncio.wait_for(
                asyncio.to_thread(self.capture_sync, provider_id, stop), 5,
            )
        finally:
            stop.set()
            with self._condition:
                self._condition.notify_all()

    def close(self):
        """Stop ROS work and release all retained channels at service shutdown."""
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._stop.set()
            entries = list(self._entries.values())
            self._entries.clear()
            self._condition.notify_all()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2)
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        if self._context is not None and self._context.ok():
            self._context.shutdown()
        for entry in entries:
            entry.channel.close()
