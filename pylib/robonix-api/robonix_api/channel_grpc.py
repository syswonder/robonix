# SPDX-License-Identifier: MulanPSL-2.0
"""Generated gRPC bindings and calls owned by an execution Channel."""
from __future__ import annotations

import importlib
import threading

from .atlas_types import GrpcParams


def binding(params, mode):
    from google.protobuf.message_factory import GetMessageClass

    if not isinstance(params, GrpcParams):
        raise ValueError("Missing gRPC transport parameters")
    # find python module for grpc
    module = importlib.import_module(params.proto_file.removesuffix(".proto").replace("/", ".") + "_pb2")
    service = module.DESCRIPTOR.pool.FindServiceByName(params.service_name)
    method = service.FindMethodByName(params.method.rsplit("/", 1)[-1])
    route = f"/{service.full_name}/{method.name}"
    if params.method not in (method.name, route):
        raise ValueError(f"gRPC method does not match service: {params.method}")
    expected = {
        "rpc": (False, False),
        "rpc_server_stream": (False, True), "topic_out": (False, True),
        "rpc_client_stream": (True, False), "topic_in": (True, False),
        "rpc_bidirectional_stream": (True, True),
    }
    if expected.get(mode) != (method.client_streaming, method.server_streaming):
        raise ValueError(f"gRPC method does not match contract mode: {mode}")
    # GetMessageClass is a function that returns a class that corresponds to the message type
    return GetMessageClass(method.input_type), GetMessageClass(method.output_type), method, route


def raw_response(message):
    from google.protobuf.json_format import MessageToDict
    return MessageToDict(message, preserving_proto_field_name=True,
                         always_print_fields_with_no_presence=True)


class _Grpc:
    """Retain the native call so a local timeout cannot abandon remote execution."""

    def __init__(self, execution):
        self.execution = execution # class _Execution in channel.py
        self.connection = None
        self.rpc = None
        self.done = threading.Event()
        self.read_lock = threading.Lock()
        self.stop_requests = threading.Event()
        self.draining = False
        self.error = None
        self.result = None
        self.source_active = threading.Event()
        self.source_lock = threading.Lock()

    def prepare(self, deadline):
        import grpc
        from .channel import _remaining
        execution = self.execution
        ch = execution.channel
        input_cls, output_cls, self.method, route = binding(ch.params, execution.descriptor.mode)
        execution.binding = (input_cls, output_cls)
        options = [("grpc.enable_http_proxy", 0)]
        if ch.endpoint.startswith("https://"):
            self.connection = grpc.secure_channel(ch.endpoint[8:], grpc.ssl_channel_credentials(), options=options)
        else:
            self.connection = grpc.insecure_channel(ch.endpoint.removeprefix("http://"), options=options)
        grpc.channel_ready_future(self.connection).result(timeout=_remaining(deadline))
        shape = ("stream" if self.method.client_streaming else "unary") + "_" + (
            "stream" if self.method.server_streaming else "unary")
        self.invoke = getattr(self.connection, shape)(
            route, request_serializer=input_cls.SerializeToString,
            response_deserializer=output_cls.FromString,
        )

    def _message(self, value):
        """Accept a dict or the bound protobuf request type for the same RPC. And return the protobuf."""
        from google.protobuf.json_format import ParseDict
        cls = self.execution.binding[0]
        if isinstance(value, dict):
            return ParseDict(value, cls()) # Change value to protobuf
        if not isinstance(value, cls):
            raise TypeError(f"Expected {cls.__name__} or dict")
        return value

    def _requests(self, source):
        """Convert requests as gRPC consumes the stream, without buffering it."""
        while True:
            with self.source_lock:
                if self.stop_requests.is_set():
                    return
                # Closing must detect a producer blocked inside next(source).
                self.source_active.set()
            try:
                value = next(source)
            except StopIteration:
                return
            except Exception as exc:
                self.error = exc
                raise
            finally:
                with self.source_lock:
                    self.source_active.clear()
            # A stop may arrive while next(source) is waiting for a value.
            if self.stop_requests.is_set():
                return
            try:
                # yield a protobuf of value
                yield self._message(value)
            except Exception as exc:
                self.error = exc
                raise

    def _finished(self, future=None):
        import grpc
        try:
            if self.method.server_streaming:
                if self.rpc.code() == grpc.StatusCode.OK:
                    with self.execution.lock:
                        self.execution.terminal = True
            else:
                self.result = future.result()
                self.execution.complete(self.result, raw_response(self.result))
        except Exception as exc:
            self.error = self.error or exc
        finally:
            self.done.set()

    def call(self, request, timeout):
        # gRPC expects an iterator for streaming input and one message otherwise.
        args = self._requests(iter(request)) if self.method.client_streaming else self._message(request)
        self.execution.start()
        try:
            if self.method.server_streaming:
                self.rpc = self.invoke(args, timeout=timeout)
                if not self.rpc.add_callback(self._finished):
                    self._finished()
                return self._responses()
            self.rpc = self.invoke.future(args, timeout=timeout if self.method.client_streaming else None)
            self.rpc.add_done_callback(self._finished)
            if not self.done.wait(timeout):
                raise TimeoutError("gRPC response timed out")
            if self.error is not None:
                raise self.error
            return self.result
        except Exception as exc:
            raise self.execution._error(f"gRPC request failed: {exc}") from exc

    def _responses(self):
        while True:
            try:
                with self.read_lock:
                    value = next(self.rpc)
            except StopIteration:
                return
            except Exception as exc:
                raise self.execution._error(f"gRPC stream failed: {self.error or exc}") from exc
            yield value

    def _drain(self):
        try:
            for _ in self._responses():
                pass
        except Exception as exc:
            self.error = exc

    def finish(self, timeout):
        self.stop_requests.set()
        if self.method.server_streaming and not self.draining:
            self.draining = True
            threading.Thread(target=self._drain, name="robonix-grpc-drain", daemon=True).start()
        self.done.wait(timeout)
        if not self.done.is_set() and self.source_active.is_set():
            raise self.execution._error("Request iterator is still blocked; stop its producer before closing")

    def close(self):
        with self.source_lock:
            self.stop_requests.set()
            if self.source_active.is_set():
                raise self.execution._error("Request iterator is still blocked; stop its producer before closing")
        if self.connection is not None:
            self.connection.close()
