# SPDX-License-Identifier: MulanPSL-2.0
"""Chronos-owned query/ROS collection; stdout is a bounded observation IPC protocol."""
from __future__ import annotations

import importlib
import json
import logging
import math
import sys
import threading
import time
import uuid

log = logging.getLogger("chronos.collector")
REFERENCE_RESOLUTION_NS = max(1, math.ceil(time.get_clock_info("time").resolution * 1_000_000_000))


def receive_time():
    """Bracket the callback's host reference read before extraction or lock contention."""
    before = time.monotonic_ns()
    reference = time.time_ns()
    after = time.monotonic_ns()
    if after < before:
        raise RuntimeError("monotonic clock regressed during callback sampling")
    span = after - before
    return dict(received_reference_ns=reference, received_steady_ns=before + span // 2,
                received_uncertainty_ns=(span + 1) // 2 + REFERENCE_RESOLUTION_NS)


def field(message, path):
    """Resolve a configured dotted message path without evaluating expressions."""
    for part in path.split("."):
        message = getattr(message, part)
    return message


def integer(value, name, minimum=0):
    """Reject lossy coercions and counters outside the RPC's integer range."""
    if type(value) is not int or not minimum <= value <= 2**64 - 1:
        raise ValueError(f"invalid {name}")
    return value


def timestamp(value):
    """Accept integer nanoseconds or a ROS Time value, preserving exact precision."""
    if type(value) is not int:
        nanos = integer(value.nanosec, "stamp.nanosec")
        if nanos >= 1_000_000_000:
            raise ValueError("invalid stamp.nanosec")
        value = integer(value.sec, "stamp.sec") * 1_000_000_000 + nanos
    if type(value) is not int or not 0 <= value <= 2**63 - 1:
        raise ValueError("invalid device timestamp")
    return value


class Source:
    """Keep one domain's capture identity; timestamp-only regressions fail closed."""

    def __init__(self, plan):
        """Validate identity mapping and create a unique timestamp-only collection session."""
        self.plan = plan
        paths = [plan.get(name) for name in ("session_id_field", "device_generation_field", "sequence_id_field")]
        if any(paths) and not all(paths):
            raise ValueError("configure all three capture identity field paths, or omit all three")
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.last = None
        self.failed = False
        self.last_steady = None

    def sample(self, message):
        """Deduplicate and extract observations; never guess a device reset from a stamp."""
        if self.failed:
            return None
        stamp = timestamp(field(message, self.plan["timestamp_field"]))
        session_path = self.plan.get("session_id_field")
        generation_path = self.plan.get("device_generation_field")
        sequence_path = self.plan.get("sequence_id_field")
        session = field(message, session_path) if session_path else self.session
        if not isinstance(session, str) or not session or session != session.strip() or len(session.encode()) > 128:
            raise ValueError("invalid session_id")
        generation = integer(field(message, generation_path), "device_generation", 1) if generation_path else 1
        sequence = integer(field(message, sequence_path), "sequence_id") if sequence_path else self.sequence + 1
        return self.record(stamp, session, generation, sequence, bool(sequence_path), time.monotonic_ns())

    def record(self, stamp, session, generation, sequence, native_sequence, steady):
        """Keep raw counters; permit only unambiguous declared wraps within one device generation."""
        period = self.plan.get("wrap_period_ns")
        if period is not None and stamp >= period:
            raise ValueError("device timestamp is outside the declared wrap period")
        identity = (session, generation)
        if self.last is not None:
            previous_identity, previous_sequence, previous_stamp = self.last
            if session == previous_identity[0] and generation < previous_identity[1]:
                raise ValueError("stale device generation")
            if identity == previous_identity:
                if native_sequence and sequence <= previous_sequence:
                    if sequence == previous_sequence and stamp != previous_stamp:
                        raise ValueError("capture identity reused with a different timestamp")
                    return None
                if period is not None:
                    elapsed = steady - self.last_steady
                    rate = 1_000_000 + self.plan.get("max_drift_ppm", 0)
                    if elapsed < 0 or elapsed * rate * 2 >= period * 1_000_000:
                        raise ValueError("counter wrap is ambiguous after sampling gap; restart collection")
                    if (stamp - previous_stamp) % period >= (period + 1) // 2:
                        raise ValueError("counter movement is ambiguous without an explicit identity change")
                if not native_sequence and stamp == previous_stamp:
                    return None
                if stamp < previous_stamp and (period is None or previous_stamp - stamp <= period // 2):
                    raise ValueError("device timestamp regressed without an explicit identity change; restart collection")
        self.sequence = sequence
        self.last = (identity, sequence, stamp)
        self.last_steady = steady
        return dict(domain_id=self.plan["domain_id"], session_id=session,
                    device_generation=generation, sequence_id=sequence,
                    device_timestamp_ns=stamp)


class QuerySource(Source):
    """Load one hardware adapter only when configured; expose the same bounded observation envelope."""

    def __init__(self, plan):
        """Create the configured adapter without importing unrelated SDKs or opening ROS subscriptions."""
        super().__init__(plan)
        module, factory = plan["query"]["adapter"].split(":")
        self.adapter = getattr(importlib.import_module(module), factory)(dict(plan["query"].get("options", {})))
        if not callable(getattr(self.adapter, "read", None)) or not callable(getattr(self.adapter, "close", None)):
            if callable(getattr(self.adapter, "close", None)):
                self.adapter.close()
            raise TypeError("query adapter must implement read() and close()")
        self.query_started = None

    def sample_query(self):
        """Validate the real query bracket and scale unsigned raw ticks before emitting a midpoint pair."""
        from chronos_query import QuerySample
        sample = self.adapter.read()
        if not isinstance(sample, QuerySample):
            raise TypeError("query adapter must return chronos_query.QuerySample")
        ticks = integer(sample.ticks, "query ticks")
        generation = integer(sample.device_generation, "device_generation", 1)
        extra_uncertainty = integer(sample.uncertainty_ns, "query uncertainty", 1)
        before = integer(sample.before_steady_ns, "query before steady")
        after = integer(sample.after_steady_ns, "query after steady")
        for reference in (sample.before_reference_ns, sample.after_reference_ns):
            if type(reference) is not int or not 0 <= reference <= 2**63 - 1:
                raise ValueError("invalid query reference time")
        elapsed = after - before
        if elapsed < 0 or elapsed > self.plan["query"]["timeout_ns"]:
            raise ValueError("query duration exceeded timeout or steady clock regressed")
        if after > time.monotonic_ns() or time.monotonic_ns() - before > 100_000_000:
            raise ValueError("query bracket is future-dated or stale")
        wall_elapsed = sample.after_reference_ns - sample.before_reference_ns
        if abs(wall_elapsed - elapsed) > 1_000_000:
            raise ValueError("host reference clock changed during query")
        stamp = timestamp(ticks * self.plan["query"]["tick_ns"])
        steady = before + elapsed // 2
        observation = self.record(stamp, self.session, generation, self.sequence + 1, False, steady)
        if observation is not None:
            uncertainty = integer((elapsed + 1) // 2 + abs(wall_elapsed - elapsed)
                                  + extra_uncertainty + REFERENCE_RESOLUTION_NS,
                                  "combined query uncertainty", 1)
            observation.update(received_reference_ns=(sample.before_reference_ns + sample.after_reference_ns) // 2,
                received_steady_ns=steady,
                received_uncertainty_ns=uncertainty)
        return observation



def run():
    """Run optional ROS subscriptions and independent query workers through bounded, acknowledged IPC."""
    plans = json.loads(sys.stdin.readline())
    condition = threading.Condition()
    pending = {}
    stopped = threading.Event()
    sources = []
    query_threads = []
    query_sources = []
    node = executor = rclpy = None

    def fail(source):
        """Keep a failed domain out of the pending queue and expose its failure without source switching."""
        with condition:
            source.failed = True
            pending.pop(source.plan["domain_id"], None)
            log.exception("collection stopped for domain %s", source.plan["domain_id"])
            condition.notify_all()

    def enqueue(source, observation):
        """Retain only the latest sample per domain while the parent processes an earlier observation."""
        with condition:
            if observation is not None and not source.failed:
                pending[source.plan["domain_id"]] = (observation["received_steady_ns"], observation)
                condition.notify()

    def poll(source):
        """Query at the configured cadence; close resources on success, stop or an adapter error."""
        try:
            while not stopped.is_set() and not source.failed:
                source.query_started = time.monotonic_ns()
                observation = source.sample_query()
                if time.monotonic_ns() - source.query_started > source.plan["query"]["timeout_ns"]:
                    raise TimeoutError("query adapter read exceeded timeout")
                source.query_started = None
                enqueue(source, observation)
                if stopped.wait(source.plan["sampling_interval_ns"] / 1_000_000_000):
                    break
        except Exception:
            fail(source)
        finally:
            try:
                source.adapter.close()
            except Exception:
                fail(source)

    def spin():
        """Make executor failure fatal rather than leaving an apparently healthy collector."""
        try:
            executor.spin()
        except Exception:
            log.exception("Chronos subscription executor failed")
        finally:
            stopped.set()
            with condition:
                condition.notify_all()

    def callback(source, message):
        """Capture callback time before lock/extraction and report domain failures."""
        received = receive_time()
        with condition:
            try:
                observation = source.sample(message)
                if observation is not None:
                    observation.update(received)
                    enqueue(source, observation)
            except Exception:
                fail(source)

    try:
        for plan in plans:
            if plan.get("query") is not None:
                source = QuerySource(plan)
                sources.append(source)
                query_sources.append(source)
                query_threads.append(threading.Thread(target=poll, args=(source,), daemon=True))
                continue
            if node is None:
                import rclpy
                from rclpy.executors import SingleThreadedExecutor
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
                from rosidl_runtime_py.utilities import get_message
                rclpy.init(args=[])
                node = rclpy.create_node("chronos_collector_" + uuid.uuid4().hex)
                executor = SingleThreadedExecutor()
                executor.add_node(node)
            source = Source(plan)
            sources.append(source)
            if plan["qos"] not in ("best_effort", "reliable", "latched"):
                raise ValueError(f"unsupported QoS profile: {plan['qos']}")
            qos = QoSProfile(depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT if plan["qos"] == "best_effort" else ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if plan["qos"] == "latched" else DurabilityPolicy.VOLATILE)
            msg_type = plan["msg_type"].removesuffix(".msg")
            parts = [part for part in msg_type.split("/") if part != "msg"]
            if len(parts) != 2:
                raise ValueError(f"unsupported message type: {msg_type}")
            node.create_subscription(get_message(f"{parts[0]}/msg/{parts[1]}"), plan["topic"],
                                     lambda message, source=source: callback(source, message), qos)
        if executor is not None:
            threading.Thread(target=spin, daemon=True).start()
        for thread in query_threads:
            thread.start()
        print(json.dumps({"ready": True}), flush=True)
        while not stopped.is_set():
            with condition:
                for source in sources:
                    if isinstance(source, QuerySource) and not source.failed and source.query_started is not None:
                        if time.monotonic_ns() - source.query_started > source.plan["query"]["timeout_ns"]:
                            # A blocked SDK call cannot be cancelled safely in a Python thread.
                            # Terminate the helper so Rust reaps it and releases device resources.
                            raise TimeoutError(f"query timed out for domain {source.plan['domain_id']}; stopping collector")
                if sources and all(source.failed for source in sources):
                    raise RuntimeError("all configured observation domains have failed")
                condition.wait_for(lambda: pending or stopped.is_set(), timeout=0.01)
                if not pending:
                    continue
                domain = next(iter(pending))
                received, observation = pending.pop(domain)
            if time.monotonic_ns() - received > 100_000_000:
                log.debug("discarded stale pending observation for domain %s", domain)
                continue
            print(json.dumps(observation), flush=True)
            if not sys.stdin.readline():
                break
        if stopped.is_set():
            raise RuntimeError("Chronos subscription executor stopped")
    finally:
        stopped.set()
        for source, thread in zip(query_sources, query_threads):
            if thread.ident is None:
                source.adapter.close()
            else:
                thread.join(timeout=0.1)
        if executor is not None:
            executor.shutdown(timeout_sec=1)
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, stream=sys.stderr)
    run()
