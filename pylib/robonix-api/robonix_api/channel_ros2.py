# SPDX-License-Identifier: MulanPSL-2.0
"""ROS 2 communication using the process-wide backend and contract types."""
from __future__ import annotations

import importlib
import logging
import threading
import time

log = logging.getLogger(__name__)

# get the msg/srv type class
def binding(descriptor):
    name = descriptor.io_srv_type if descriptor.mode == "rpc" else descriptor.io_msg_type
    package, kind, cls = name.split("/")
    expected = "srv" if descriptor.mode == "rpc" else "msg"
    if kind != expected:
        raise ValueError(f"Invalid ROS 2 type: {name}")
    return getattr(importlib.import_module(f"{package}.{kind}"), cls)


class _Ros2:
    """Own only this channel's entities; other users keep the shared backend."""

    def __init__(self, execution):
        from .ros import RosBackend

        self.execution = execution
        self.backend = RosBackend.get()
        self.entity = None
        self.done = threading.Event()
        self.condition = threading.Condition(threading.RLock())
        self.callbacks = set()
        self.stopped = False
        self.result = self.error = None
        self.mode = execution.descriptor.mode

    def prepare(self, deadline):
        from .channel import _remaining
        execution = self.execution
        self.cls = binding(execution.descriptor)
        ch = execution.channel
        self.qos = ch.params.qos_profile or 10
        # Start the backend and validate QoS before returning a usable channel.
        self.backend.node
        self.backend.qos(self.qos)
        if self.mode == "rpc":
            self.entity = self.backend.create_client(self.cls, ch.endpoint, ch.params.qos_profile)
            if not self.entity.wait_for_service(timeout_sec=_remaining(deadline)):
                raise TimeoutError(f"ROS 2 service unavailable: {ch.endpoint}")
        elif self.mode == "topic_in":
            self.entity = self.backend.create_publisher(self.cls, ch.endpoint, self.qos)

    def _message(self, value, cls):
        if isinstance(value, dict):
            from rosidl_runtime_py.set_message import set_message_fields
            message = cls()
            set_message_fields(message, value)
            return message
        if not isinstance(value, cls):
            raise TypeError(f"Expected {cls.__name__} or dict")
        return value

    def _finished(self, future):
        try:
            from rosidl_runtime_py.convert import message_to_ordereddict
            self.result = future.result()
            self.execution.complete(self.result, message_to_ordereddict(self.result))
        except Exception as exc:
            self.error = exc
        finally:
            self.done.set()

    def call(self, request, timeout):
        if self.mode != "rpc":
            raise ValueError("Use publish() or subscribe() for a ROS 2 topic")
        message = self._message(request, self.cls.Request)
        self.execution.start()
        try:
            future = self.entity.call_async(message)
            future.add_done_callback(self._finished)
            if not self.done.wait(timeout):
                raise TimeoutError("ROS 2 response timed out")
            if self.error is not None:
                raise self.error
            return self.result
        except Exception as exc:
            raise self.execution._error(f"ROS 2 request failed: {exc}") from exc

    def publish(self, message):
        if self.mode != "topic_in":
            raise ValueError("publish() requires a ROS 2 topic_in contract")
        message = self._message(message, self.cls)
        with self.condition, self.execution.lock:
            if self.stopped or self.execution.closing or self.execution.channel._closed:
                raise RuntimeError("Channel is closing or closed")
            self.execution.started = True
            self.entity.publish(message)

    def subscribe(self, callback):
        if self.mode != "topic_out":
            raise ValueError("subscribe() requires a ROS 2 topic_out contract")
        if not callable(callback):
            raise TypeError("Expected a subscription callback")
        with self.condition:
            self.execution.start()
            self.entity = self.backend.create_subscription(
                self.cls, self.execution.channel.endpoint,
                lambda message: self._receive(callback, message), self.qos,
            )

    def _receive(self, callback, message):
        ident = threading.get_ident()
        with self.condition:
            if self.stopped:
                return
            self.callbacks.add(ident)
        try:
            callback(message)
        except Exception:
            log.exception("Channel %s subscription callback failed", self.execution.channel.channel_id)
        finally:
            with self.condition:
                self.callbacks.remove(ident)
                self.condition.notify_all()

    def finish(self, timeout):
        if self.mode == "rpc":
            self.done.wait(timeout)
            return
        deadline = time.monotonic() + timeout
        with self.condition:
            self.stopped = True
            if threading.get_ident() in self.callbacks:
                raise self.execution._error("Close the subscription outside its callback")
            self.close()
            if not self.condition.wait_for(lambda: not self.callbacks, timeout=max(0, deadline - time.monotonic())):
                raise self.execution._error("ROS 2 subscription callbacks still running")
            with self.execution.lock:
                self.execution.terminal = True

    def close(self):
        if self.entity is not None:
            kind = {"rpc": "client", "topic_in": "publisher", "topic_out": "subscription"}[self.mode]
            getattr(self.backend.node, f"destroy_{kind}")(self.entity)
            self.entity = None
