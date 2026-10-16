# SPDX-License-Identifier: MulanPSL-2.0
"""Selected reference readings for the Webots primitive workers."""

import asyncio
import json
import logging
import threading
import time

import grpc

from .atlas import ATLAS

log = logging.getLogger(__name__)
_CONTRACT = "robonix/system/chronos/reference_time"


class _ReferenceClock:
    def __init__(self, config, consumer):
        config = config or {}
        self.kind = config.get("kind", "linux_system")
        if self.kind not in ("linux_system", "ros_time"):
            raise ValueError("reference kind must be linux_system or ros_time")
        self.provider = config.get("provider_id", "chronos")
        if not isinstance(self.provider, str) or not self.provider or self.provider.strip() != self.provider:
            raise ValueError("reference provider_id must be nonempty and whitespace-normalized")
        self.consumer = consumer
        self._lock = threading.Lock()
        self._reading = None
        self._received_at = 0.0
        self._failure = None
        self._closed = threading.Event()
        self._loop = None
        self._task = None
        self._thread = None
        if self.kind == "ros_time":
            from robonix_contracts_pb2_grpc import RobonixSystemChronosReferenceTimeStub
            self._stub_type = RobonixSystemChronosReferenceTimeStub
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def read(self):
        """Return (nanoseconds, generation, connection), or None while unavailable."""
        if self._closed.is_set():
            raise RuntimeError("reference subscription stopped")
        if self.kind == "linux_system":
            return time.monotonic_ns(), 0, 0
        with self._lock:
            if self._failure is not None:
                raise RuntimeError(f"reference subscription failed: {self._failure}")
            if time.monotonic() - self._received_at >= 5.0:
                return None
            return self._reading

    def close(self):
        self._closed.set()
        with self._lock:
            loop, task = self._loop, self._task
            if loop is not None and not loop.is_closed():
                loop.call_soon_threadsafe(task.cancel)
        if self._thread is not None:
            self._thread.join(timeout=8.0)
            if self._thread.is_alive():
                raise RuntimeError("reference subscription did not stop")
        with self._lock:
            if self._failure is not None:
                raise RuntimeError(f"reference subscription failed: {self._failure}")

    def _release(self):
        stub, wire = ATLAS._wire_stub, ATLAS._wire_pb
        response = stub.InspectAtlas(wire.InspectAtlasRequest(), timeout=2.0)
        for channel_id, channel in json.loads(response.json)["channels"].items():
            if (channel["consumer_id"] == self.consumer and channel["provider_id"] == self.provider
                    and channel["contract_id"] == _CONTRACT
                    and channel["transport"] == "TRANSPORT_GRPC"):
                stub.DisconnectCapability(wire.DisconnectCapabilityRequest(channel_id=channel_id), timeout=1.0)

    def _run(self):
        try:
            asyncio.run(self._follow())
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            with self._lock:
                self._failure = exc
            log.exception("Chronos reference subscription failed")

    async def _follow(self):
        from google.protobuf.empty_pb2 import Empty

        with self._lock:
            self._loop = asyncio.get_running_loop()
            self._task = asyncio.current_task()
        connection = 0
        try:
            while not self._closed.is_set():
                try:
                    await asyncio.to_thread(self._release)
                    stub, wire = ATLAS._wire_stub, ATLAS._wire_pb
                    connect = asyncio.create_task(asyncio.to_thread(
                        stub.ConnectCapability,
                        wire.ConnectCapabilityRequest(
                            consumer_id=self.consumer, provider_id=self.provider,
                            contract_id=_CONTRACT, transport=wire.TRANSPORT_GRPC),
                        timeout=2.0,
                    ))
                    try:
                        reply = await asyncio.shield(connect)
                    except asyncio.CancelledError:
                        # A lost Connect reply can still have allocated an Atlas channel.
                        await asyncio.gather(connect, return_exceptions=True)
                        raise
                    endpoint = reply.endpoint
                    channel = (grpc.aio.secure_channel(endpoint.removeprefix("https://"), grpc.ssl_channel_credentials())
                               if endpoint.startswith("https://") else
                               grpc.aio.insecure_channel(endpoint.removeprefix("http://")))
                    connection += 1
                    previous = None
                    async with channel:
                        call = self._stub_type(channel).ReferenceTime(Empty())
                        try:
                            # A completed read can race wait_for cancellation.
                            while not self._closed.is_set():
                                sample = await asyncio.wait_for(call.read(), timeout=5.0)
                                if sample is grpc.aio.EOF:
                                    raise OSError("Chronos reference stream ended")
                                if sample.available:
                                    if sample.timestamp_ns < 0 or sample.generation == 0:
                                        raise OSError("invalid Chronos reference reading")
                                    if (previous is not None and sample.generation == previous[1]
                                            and sample.timestamp_ns < previous[0]):
                                        raise OSError("Chronos reference moved backwards within a generation")
                                    previous = sample.timestamp_ns, sample.generation, connection
                                else:
                                    # Motion must notice an outage even if input recovers between polls.
                                    connection += 1
                                with self._lock:
                                    self._reading = previous if sample.available else None
                                    self._received_at = time.monotonic()
                        finally:
                            call.cancel()
                except (grpc.RpcError, OSError, asyncio.TimeoutError) as exc:
                    log.warning("Chronos reference unavailable: %s", exc)
                finally:
                    with self._lock:
                        self._reading = None
                await asyncio.sleep(1.0)
        finally:
            await asyncio.to_thread(self._release)
            with self._lock:
                self._reading = None
                self._loop = self._task = None
