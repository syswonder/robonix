# SPDX-License-Identifier: MulanPSL-2.0
"""Capability — the one Python object a robonix package instantiates.

A Capability talks to atlas (RegisterCapability + DeclareInterface +
QueryCapabilities + Heartbeat), serves the lifecycle gRPC interface
(Driver(CMD_INIT/ACTIVATE/DEACTIVATE/SHUTDOWN)), and provides a thin
layer of helpers over rclpy / grpcio / FastMCP for the most common
patterns.

The conscious design split:
  - Layer 1 (always available): atlas declare/query, lifecycle, subprocess
    spawning, sentinel waits. NO middleware-server-startup magic.
  - Layer 2 (opt-in convenience): create_publisher / create_subscription /
    `@cap.mcp` / `@cap.grpc`. Wraps the corresponding lib so users can write
    one line instead of ten. Bypassable: write rclpy / grpcio / FastMCP
    directly and call `cap.declare_*` to register with atlas.
"""
from __future__ import annotations

import inspect
import json
import logging
import os
import signal
import sys
import threading
from pathlib import Path
from typing import Any, Callable

from .atlas import AtlasClient
from .atlas_types import (
    CapabilityRecord,
    CapabilityState,
    Channel,
    Transport,
)
from .codegen import ensure_proto_gen, find_pkg_root
from .lifecycle import (
    bind_user_handler,
    build_lifecycle_servicer,
    resolve_servicer,
)
from .ros import RosBackend, resolve_msg_type
from .spawn import SpawnRegistry
from .tool import mcp_contract

log = logging.getLogger("robonix_api.capability")


def _install_simple_logger() -> None:
    """FastMCP / mcp / uvicorn install a `rich`-based RichHandler on the root
    logger, which wraps log messages into narrow columns and inserts spurious
    whitespace — unreadable in `tail -f`, and `logging.basicConfig` is a no-op
    once that handler is in place. We forcibly replace the root handlers with
    one stderr StreamHandler in a plain `[time] LEVEL logger: message` shape.
    Idempotent — only runs once per process."""
    if getattr(_install_simple_logger, "_done", False):
        return
    root = logging.getLogger()
    for h in list(root.handlers):
        root.removeHandler(h)
    handler = logging.StreamHandler()
    handler.setFormatter(
        logging.Formatter(
            fmt="%(asctime)s %(levelname)-5s %(name)s: %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    root.addHandler(handler)
    if root.level == logging.NOTSET or root.level > logging.INFO:
        root.setLevel(logging.INFO)
    _install_simple_logger._done = True  # type: ignore[attr-defined]


_install_simple_logger()

# ── transport-ENUM ↔ contract.mode compatibility matrix ──────────────────────
_MODE_TRANSPORT_OK = {
    "rpc": {"grpc", "mcp", "ros2"},
    "topic_in": {"ros2", "grpc"},
    "topic_out": {"ros2", "grpc"},
}


class Capability:
    """One instance per package.

    Args:
        id:        capability_id (e.g. "com.robonix.ranger.mid360_lidar").
        namespace: atlas namespace (e.g. "primitive/lidar"). Framework derives
                   the `*/driver` contract id and gRPC servicer class from this.
        pkg_root:  package root dir; default is auto-detected via the caller's
                   __file__ walking up to a directory with `package_manifest.yaml`.
        atlas_endpoint:  defaults to env ROBONIX_ATLAS or "127.0.0.1:50051".
    """

    def __init__(
        self,
        id: str,
        namespace: str,
        *,
        pkg_root: Path | None = None,
        md_path: str | None = None,
        atlas_endpoint: str | None = None,
    ) -> None:
        self.id = id
        # Namespace is whatever the operator chose — atlas validates
        # contract_id starts with `<namespace>/`. Built-in robonix caps use
        # `robonix/...` but a third-party skill is free to pick any prefix
        # (e.g. `myorg/skill/pick_vla`). We normalise trailing slashes only.
        self.namespace = namespace.strip("/")
        if not self.namespace:
            raise ValueError("namespace must be non-empty")

        # Locate pkg_root from the caller's frame if not given.
        if pkg_root is None:
            caller_file = _caller_file(skip=1)
            if caller_file is not None:
                pkg_root = find_pkg_root(caller_file)
        self.pkg_root: Path = (pkg_root or Path.cwd()).resolve()

        # Add the package's codegen output to sys.path so atlas_pb2 / contracts
        # are importable when run() / declare_*() actually need them.
        ensure_proto_gen(self.pkg_root)

        self._atlas = AtlasClient(
            endpoint=atlas_endpoint
            or os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"),
        )
        # Ports are auto-allocated in run() (gRPC's add_insecure_port returns
        # the actually bound port; MCP uses a free-port pre-claim). Atlas
        # gets the chosen port via DeclareInterface so consumers find us
        # through QueryCapabilities — no port management required from the
        # operator. This used to be ROBONIX_DRIVER_PORT / ROBONIX_MCP_PORT
        # env vars per package; collisions became inevitable as soon as more
        # than one capability ran on the same host.
        self._driver_port: int = 0
        self._mcp_port: int = 0

        # md_path: explicit overrides; else <pkg_root>/CAPABILITY.md if exists.
        if md_path is None:
            cand = self.pkg_root / "CAPABILITY.md"
            md_path = str(cand) if cand.is_file() else ""
        self._md_path = md_path

        self._spawn = SpawnRegistry()

        # User-registered handlers (filled by decorators).
        self._on_init: Callable | None = None
        self._on_activate: Callable | None = None
        self._on_deactivate: Callable | None = None
        self._on_shutdown: Callable | None = None

        # Lifecycle state. Source of truth on the cap side; pushed to
        # atlas via SetCapabilityState whenever it transitions (see
        # _set_state below). Initial value is REGISTERED — it flips to
        # INACTIVE on Driver(CMD_INIT) success, ACTIVE on
        # CMD_ACTIVATE, back to INACTIVE on CMD_DEACTIVATE,
        # TERMINATED on CMD_SHUTDOWN.
        self._state: str = "registered"

        # Channels we opened via cap.connect(); force-closed on teardown
        # so atlas doesn't accumulate dangling consumer→provider edges.
        self._channels: list[Channel] = []

        # Layer 2 registries (populated by decorators / methods).
        self._mcp_app = None  # FastMCP app, lazy
        self._mcp_handlers: list[Callable] = []  # decorated funcs
        self._grpc_handlers: list[tuple[str, Callable]] = []  # (contract_id, fn)
        self._grpc_servicers: list[tuple[str, Any]] = (
            []
        )  # (contract_id, servicer instance)
        self._publishers: dict[str, Any] = {}  # contract_id → rclpy publisher

        self._driver_server = None
        self._mcp_server_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._stopping = threading.Event()

    # ── lifecycle decorators ─────────────────────────────────────────────
    def on_init(self, fn: Callable[[dict], Any]) -> Callable[[dict], Any]:
        """REGISTERED → INACTIVE. Parse config, validate dependencies,
        bind logical device. NO hot runtime resources yet."""
        if self._on_init is not None:
            raise RuntimeError("on_init handler already registered")
        self._on_init = fn
        return fn

    def on_activate(self, fn: Callable[[], Any]) -> Callable[[], Any]:
        """INACTIVE → ACTIVE. Acquire hot runtime resources
        (threads, models, ROS subs, hardware fds). After this returns
        Ok(), atlas marks the cap ACTIVE and consumers may call its
        data interfaces. Optional for primitives/services (framework
        auto-promotes); REQUIRED for skills.
        Takes no args — only on_init receives cfg."""
        self._on_activate = fn
        return fn

    def on_deactivate(self, fn: Callable[[], Any]) -> Callable[[], Any]:
        """ACTIVE → INACTIVE. Release hot resources but keep
        config / atlas registration. Optional for primitives/services;
        executor calls this on skills via its eviction policy.
        Takes no args."""
        self._on_deactivate = fn
        return fn

    def on_shutdown(self, fn: Callable[[], Any]) -> Callable[[], Any]:
        """any → TERMINATED. Last-chance cleanup before process exit
        (close ports, flush logs). Must return a Result like the other
        handlers (the value doesn't drive state — cap goes TERMINATED
        regardless — but the dispatcher still type-checks it)."""
        self._on_shutdown = fn
        return fn

    # ── lifecycle state ──────────────────────────────────────────────────
    @property
    def state(self) -> str:
        """Current lifecycle state — one of registered / inactive /
        active / error / terminated. The framework drives this off
        Driver cmd transitions; users typically don't write to it
        directly."""
        return self._state

    def _set_state(self, new_state: str | None, detail: str = "") -> None:
        """Update local state + push to atlas. Idempotent on no-change.
        `new_state=None` updates only state_detail without changing the
        state itself (used for Deferred — cap stays where it is, but
        state_detail explains why)."""
        if new_state is None:
            # Detail-only push; keep current state.
            try:
                self._atlas.set_capability_state(self.id, self._state, detail)
            except Exception:  # noqa: BLE001
                pass
            return
        new_state = new_state.lower()
        if new_state == self._state:
            return
        prev = self._state
        self._state = new_state
        log.info(
            "[%s] state %s → %s%s",
            self.id,
            prev,
            new_state,
            f" ({detail})" if detail else "",
        )
        try:
            self._atlas.set_capability_state(self.id, new_state, detail)
        except Exception:  # noqa: BLE001
            # set_capability_state already logs at debug; swallow to keep
            # the cap usable even if atlas is briefly unreachable.
            pass

    # ── Layer 1: raw atlas declares ──────────────────────────────────────
    def declare_ros2_topic(
        self, contract_id: str, topic: str, *, qos: str = "best_effort"
    ) -> None:
        """Declare a ROS 2 topic endpoint for a topic_in / topic_out contract.
        `qos` is one of the QoS preset strings (see dev guide §10.8)."""
        self._check_mode("ros2", contract_id)
        self._atlas.declare_ros2(self.id, _full_id(contract_id), topic, qos)

    def declare_ros2_service(
        self, contract_id: str, service: str
    ) -> None:
        """Declare a ROS 2 service endpoint for an rpc contract over
        the ROS 2 transport. ROS 2 service QoS is fixed (reliable,
        depth 10) — no `qos` param."""
        self._check_mode("ros2", contract_id)
        self._atlas.declare_ros2(self.id, _full_id(contract_id), service, "")

    def declare_grpc(
        self,
        contract_id: str,
        endpoint: str,
        service_name: str,
        method: str,
        proto_file: str = "robonix_contracts.proto",
    ) -> None:
        self._check_mode("grpc", contract_id)
        self._atlas.declare_grpc(
            self.id,
            _full_id(contract_id),
            endpoint,
            service_name,
            method,
            proto_file=proto_file,
        )

    def declare_mcp(
        self,
        contract_id: str,
        endpoint: str,
        description: str = "",
        input_schema_json: str = "{}",
    ) -> None:
        self._check_mode("mcp", contract_id)
        self._atlas.declare_mcp(
            self.id, _full_id(contract_id), endpoint, description, input_schema_json
        )

    # ── Layer 1: connect (discovery is on the atlas singleton — `from
    # robonix_api import atlas; atlas.get(...) / atlas.find(...)`) ─────
    def connect(
        self,
        provider: CapabilityRecord,
        contract_id: str,
        transport: Transport | str | int,
    ) -> Channel:
        """Open a consumer→provider channel given a resolved provider
        record (from `atlas.get(id)` or `atlas.find(...)`). The returned
        `Channel` is a context manager; auto-closes on teardown if not
        explicitly `with`-blocked."""
        if not isinstance(provider, CapabilityRecord):
            raise TypeError(
                "cap.connect(provider, ...) requires a CapabilityRecord; "
                "use atlas.get(id) or atlas.find(...) to resolve one"
            )
        full_contract = _full_id(contract_id)
        ch = self._atlas.connect(
            consumer_id=self.id,
            capability_id=provider.capability_id,
            contract_id=full_contract,
            transport=transport,
        )
        self._channels.append(ch)
        return ch

    # ── Layer 1: subprocess + ROS sentinel ───────────────────────────────
    def spawn(
        self,
        argv,
        *,
        env: dict | None = None,
        log: str | Path | None = None,
        cwd: Path | None = None,
    ):
        log_path: Path | None
        if log is None:
            log_path = None
        elif isinstance(log, Path):
            log_path = log
        else:
            log_path = self.pkg_root / "rbnx-build" / "data" / str(log)
        return self._spawn.spawn(argv, env=env, log_path=log_path, cwd=cwd)

    def wait_for_topic(
        self, topic: str, msg_type: str | type, timeout_s: float = 30.0
    ) -> bool:
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        return RosBackend.get().wait_for_topic(topic, cls, timeout_s)

    def resolve_host_ip(self, target_ip: str) -> str | None:
        """`ip route get <target>` → src field. Used by drivers (e.g. mid360)
        that need to bake the host's IP into a vendor config file."""
        import subprocess

        try:
            out = subprocess.run(
                ["ip", "-4", "route", "get", target_ip],
                capture_output=True,
                text=True,
                timeout=2,
                check=False,
            )
        except FileNotFoundError:
            return None
        toks = out.stdout.split()
        if "src" in toks:
            i = toks.index("src")
            if i + 1 < len(toks):
                return toks[i + 1]
        return None

    # ── Layer 2: ROS publisher / subscriber ──────────────────────────────
    def create_publisher(
        self,
        contract_id: str,
        *,
        topic: str,
        msg_type: type | str,
        qos: str | int = "best_effort",
        declare: bool = True,
    ):
        """Create an rclpy publisher AND (optionally) atlas-declare it.
        Returns the rclpy publisher; user calls `.publish(msg)` directly OR
        via `cap.emit(contract_id, msg)`."""
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        pub = RosBackend.get().create_publisher(cls, topic, qos)
        self._publishers[_full_id(contract_id)] = pub
        if declare:
            self.declare_ros2_topic(
                contract_id, topic, qos=qos if isinstance(qos, str) else "reliable"
            )
        return pub

    def create_subscription(
        self,
        contract_id: str,
        *,
        topic: str,
        msg_type: type | str,
        callback: Callable[[Any], None],
        qos: str | int = "best_effort",
        declare: bool = True,
    ):
        """Create an rclpy subscription. If `declare=True` we also tell atlas
        we consume this contract over ROS2 (atlas tracks consumer-side
        bindings too — useful for `rbnx channels` audits)."""
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        sub = RosBackend.get().create_subscription(cls, topic, callback, qos)
        if declare:
            try:
                self.declare_ros2_topic(
                    contract_id, topic, qos=qos if isinstance(qos, str) else "reliable"
                )
            except Exception:  # noqa: BLE001
                # Consumer-side declare is optional; don't fail if atlas refuses.
                pass
        return sub

    def create_subscription_from_channel(
        self,
        channel: Channel,
        *,
        msg_type: type | str,
        callback: Callable[[Any], None],
    ):
        """Create an rclpy subscription using the topic + qos atlas
        resolved via `cap.connect(...)`. Skips the `topic=` / `qos=`
        ceremony — those come from the channel.

        v0.1 still requires `msg_type` as a Python class (or a string
        like 'sensor_msgs/Image' resolved via rosidl). v0.2 will pull
        the message type from the contract's IDL automatically."""
        cls = msg_type if isinstance(msg_type, type) else resolve_msg_type(msg_type)
        from .atlas_types import Ros2Params

        qos = 0
        if isinstance(channel.params, Ros2Params) and channel.params.qos_profile:
            qos = channel.params.qos_profile  # rclpy resolves the named profile
            qos = qos if isinstance(qos, int) else 0
        return RosBackend.get().create_subscription(cls, channel.endpoint, callback, qos)

    def emit(self, contract_id: str, msg: Any) -> None:
        """Publish `msg` on the rclpy publisher created via create_publisher."""
        pub = self._publishers.get(_full_id(contract_id))
        if pub is None:
            raise RuntimeError(
                f"no publisher for contract {contract_id!r} — "
                f"call cap.create_publisher(...) first"
            )
        pub.publish(msg)

    # ── Layer 2: MCP tool decorator ──────────────────────────────────────
    def mcp(self, contract_id: str, *, name: str | None = None):
        """Register an MCP tool bound to this contract. Same surface as
        `@mcp_contract` but auto-declares to atlas + boots a FastMCP server
        on cap.run() at port `mcp_port`."""
        full = _full_id(contract_id)
        self._check_mode("mcp", contract_id)
        self._ensure_mcp_app()

        def decorator(fn):
            mcp_contract(
                self._mcp_app,  # pyright: ignore[reportArgumentType]
                contract_id=full,
                name=name,
            )(
                fn
            )  # pyright: ignore[reportArgumentType]
            self._mcp_handlers.append(fn)
            return fn

        return decorator

    def _ensure_mcp_app(self) -> None:
        if self._mcp_app is not None:
            return
        from mcp.server.fastmcp import FastMCP

        self._mcp_app = FastMCP(self.id)

    def use_mcp_app(self, app) -> None:
        """For tools that aren't typed against codegen MCP dataclasses (i.e.
        plain `@app.tool()` with primitive Python types): build your own
        FastMCP app, register tools on it, hand it over here. Capability
        will run the HTTP server on `mcp_port` during cap.run().

        You're then on the hook for one cap.declare_mcp(contract_id, ...)
        call per tool — the framework can't introspect bare FastMCP tools
        for contract metadata."""
        if self._mcp_app is not None and self._mcp_app is not app:
            raise RuntimeError(
                "MCP app already set; use_mcp_app conflicts with @cap.mcp decorators"
            )
        self._mcp_app = app
        self._user_owned_mcp = True

    @property
    def mcp_endpoint(self) -> str:
        """Convenience: the URL where the MCP HTTP server will live after
        cap.run() starts. Useful for declare_mcp(contract_id, endpoint=...)."""
        return f"http://127.0.0.1:{self._mcp_port}/mcp/"

    # ── Layer 2: gRPC ─────────────────────────────────────────────────────
    def attach_grpc_servicer(self, contract_id: str, servicer) -> None:
        """Attach an already-built Servicer instance for a contract. Use this
        when your gRPC handler is a whole class (e.g. one Servicer subclass
        with several methods sharing state via __init__) rather than a single
        function — `@cap.grpc` only handles one-method-per-class. The framework
        adds your servicer to the shared lifecycle gRPC server so atlas only
        sees one endpoint per cap, and atlas-declares the contract pointing
        at it. Call BEFORE cap.run() / cap.bootstrap().
        """
        full = _full_id(contract_id)
        self._check_mode("grpc", contract_id)
        self._grpc_servicers.append((full, servicer))

    def grpc(self, contract_id: str):
        """Bind a handler to the contract's generated gRPC Servicer.

        NOTE: For now this just stores the handler; full auto-binding to the
        right generated servicer + start_grpc_server happens in cap.run().
        """
        full = _full_id(contract_id)
        self._check_mode("grpc", contract_id)

        def decorator(fn):
            self._grpc_handlers.append((full, fn))
            return fn

        return decorator

    # ── mode/transport compatibility check (best-effort) ─────────────────
    def _check_mode(self, transport: str, contract_id: str) -> None:
        """Look up the contract TOML to check transport is allowed for its mode.
        Best-effort — silently passes if we can't find the TOML."""
        # TODO: scan capabilities/<full_path>.v1.toml for [mode] type. For
        # now we trust the user. Safer than failing on missing TOML.
        return

    # ── run / bootstrap ──────────────────────────────────────────────────
    def bootstrap(self) -> None:
        """Set up atlas registration + gRPC servers + MCP HTTP + heartbeat
        WITHOUT blocking. Useful when the package has its own asyncio main
        loop (e.g. scene) — call bootstrap() during startup and let the
        existing event loop manage the rest of the lifetime. Idempotent on
        re-entry; second call is a no-op.

        For the standard "register + block until SIGTERM" flow use `run()`."""
        if self._driver_server is not None:
            return  # already bootstrapped
        self._do_bootstrap()

    def run(self) -> None:
        """Block. Order:
        1. RegisterCapability
        2. build the gRPC server (driver lifecycle + every @cap.grpc handler
           share one server on driver_port — atlas declares each contract
           with its own service_name+method; consumers multiplex by service)
        3. atlas-declare every gRPC interface
        4. start FastMCP uvicorn (only if any @cap.mcp tool)
        5. start heartbeat
        6. install SIGTERM/SIGINT handlers
        7. signal.pause() until SIGTERM/SIGINT
        8. teardown: SIGTERM all spawn'd subprocesses, stop servers
        """
        self._do_bootstrap()

        # 6. signals
        signal.signal(signal.SIGTERM, lambda *_: self._teardown_and_exit())
        signal.signal(signal.SIGINT, lambda *_: self._teardown_and_exit())

        log.info("ready — awaiting Driver(CMD_INIT)")
        try:
            while not self._stopping.is_set():
                self._stopping.wait(60.0)
        finally:
            self._teardown()

    def _do_bootstrap(self) -> None:
        # 1. atlas register
        registered_ok = False
        try:
            new = self._atlas.register_capability(
                self.id, self.namespace, self._md_path or ""
            )
            log.info(
                "registered cap %s%s", self.id, "" if new else " (already existed)"
            )
            registered_ok = True
        except Exception as e:  # noqa: BLE001
            log.warning("RegisterCapability failed: %s", e)
        # Push the initial REGISTERED state explicitly, so atlas reflects
        # "process is alive but Driver(CMD_INIT) hasn't run yet" instead of
        # leaning on the legacy "first non-driver interface declare → init"
        # inference. No-op if register itself failed.
        if registered_ok:
            self._set_state("registered")

        # 2. gRPC server — build everything BEFORE start (gRPC python requires
        # all servicers added before server.start()).
        import grpc
        import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore
        import lifecycle_pb2  # type: ignore
        from concurrent import futures

        server = grpc.server(futures.ThreadPoolExecutor(max_workers=8))

        # 2a. driver lifecycle — only when codegen emitted a `<ns>/driver`
        # contract for this package. system/* services that expose only MCP
        # tools (memory, scene, speech) skip this — they have no hardware
        # init phase, so rbnx-boot doesn't need a Driver(CMD_INIT) target.
        driver_decl: tuple[str, str] | None = None  # (service_name, method)
        lifecycle_info = build_lifecycle_servicer(
            self.namespace,
            contracts_grpc,
            lifecycle_pb2.Driver_Response,
            on_init=self._on_init,
            on_activate=self._on_activate,
            on_deactivate=self._on_deactivate,
            on_shutdown=self._user_shutdown_then_teardown,
            on_state_change=self._set_state,
            log_tag=self.id,
        )
        if lifecycle_info is not None:
            lifecycle_inst, lifecycle_add_fn, driver_base, driver_method = (
                lifecycle_info
            )
            lifecycle_add_fn(lifecycle_inst, server)
            driver_decl = (driver_base, driver_method)

        # 2b. user @cap.grpc handlers
        user_grpc_decls: list[tuple[str, str, str]] = (
            []
        )  # (contract_id, service_name, method)
        for contract_id, fn in self._grpc_handlers:
            info = resolve_servicer(contract_id, contracts_grpc)
            if info is None:
                log.warning(
                    "@cap.grpc(%r): no generated Servicer found in robonix_contracts_pb2_grpc "
                    "(did codegen run for this contract?). Skipping.",
                    contract_id,
                )
                continue
            servicer_cls, method_name, add_fn, base = info
            DynServicer = bind_user_handler(servicer_cls, method_name, fn)
            add_fn(DynServicer(), server)
            user_grpc_decls.append((contract_id, base, method_name))
            log.info("wired @cap.grpc %s → %s.%s", contract_id, base, method_name)

        # 2b'. user-attached pre-built Servicer instances (cap.attach_grpc_servicer)
        for contract_id, servicer in self._grpc_servicers:
            info = resolve_servicer(contract_id, contracts_grpc)
            if info is None:
                log.warning(
                    "attach_grpc_servicer(%r): no generated Servicer found in "
                    "robonix_contracts_pb2_grpc — did codegen run? Skipping.",
                    contract_id,
                )
                continue
            servicer_cls, method_name, add_fn, base = info
            if not isinstance(servicer, servicer_cls):
                log.warning(
                    "attach_grpc_servicer(%r): servicer %r is not a %s; "
                    "atlas declare may still work but the server won't dispatch.",
                    contract_id,
                    type(servicer).__name__,
                    servicer_cls.__name__,
                )
            add_fn(servicer, server)
            user_grpc_decls.append((contract_id, base, method_name))
            log.info(
                "attached gRPC servicer for %s → %s.%s", contract_id, base, method_name
            )

        # 2c. bind on port 0 — OS picks a free port; grpc returns it.
        # Atlas DeclareInterface below carries the actual port out, so
        # consumers find us via QueryCapabilities regardless.
        self._driver_port = server.add_insecure_port("[::]:0")
        server.start()
        self._driver_server = server
        log.info("Lifecycle gRPC serving on 0.0.0.0:%d", self._driver_port)

        # 3. atlas-declare every gRPC interface
        endpoint = f"127.0.0.1:{self._driver_port}"
        if driver_decl is not None:
            driver_base, driver_method = driver_decl
            try:
                self._atlas.declare_grpc(
                    capability_id=self.id,
                    contract_id=f"{self.namespace}/driver",
                    endpoint=endpoint,
                    service_name=driver_base,
                    method=driver_method,
                )
            except Exception as e:  # noqa: BLE001
                log.warning("DeclareInterface(driver) failed: %s", e)
        for contract_id, service_name, method in user_grpc_decls:
            try:
                self._atlas.declare_grpc(
                    capability_id=self.id,
                    contract_id=contract_id,
                    endpoint=endpoint,
                    service_name=service_name,
                    method=method,
                )
            except Exception as e:  # noqa: BLE001
                log.warning("DeclareInterface(%s) failed: %s", contract_id, e)

        # 4. FastMCP server (whenever an MCP app exists, whether built via
        # @cap.mcp decorators or handed in via cap.use_mcp_app)
        if self._mcp_app is not None:
            self._start_mcp_server()
            if self._mcp_handlers:
                self._declare_mcp_handlers()

        # 5. heartbeat
        self._heartbeat_thread = self._atlas.start_heartbeat(self.id)

        # 6. State promotion. Caps WITH a Driver(CMD_INIT/CMD_ACTIVATE)
        # handshake rely on rbnx boot to drive them through
        # INACTIVE → ACTIVE via the lifecycle servicer's
        # on_state_change callback (wired above). Caps WITHOUT a driver
        # contract (system services like memory / scene that only expose
        # MCP tools or one-shot gRPC RPCs) are fully ready as soon as
        # gRPC + MCP are listening — promote to ACTIVE here so
        # `rbnx caps` shows them online instead of stranded at
        # INACTIVE forever.
        if driver_decl is None and registered_ok:
            self._set_state("active")

    def _start_mcp_server(self) -> None:
        # Pre-claim a free port via socket(0) → close → hand to uvicorn.
        # Tiny race window (someone could grab it between close and bind),
        # but in practice the OS won't immediately reassign and uvicorn
        # binds within a few ms. Same trick used by pytest-asyncio etc.
        import socket
        import uvicorn

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("0.0.0.0", 0))
        self._mcp_port = s.getsockname()[1]
        s.close()
        cfg = uvicorn.Config(
            self._mcp_app.streamable_http_app(),  # pyright: ignore[reportOptionalMemberAccess]
            host="0.0.0.0",
            port=self._mcp_port,
            log_level="warning",
        )
        server = uvicorn.Server(cfg)
        thread = threading.Thread(
            target=server.run,
            name="robonix-mcp",
            daemon=True,
        )
        thread.start()
        self._mcp_server_thread = thread
        log.info("MCP HTTP serving on 0.0.0.0:%d", self._mcp_port)

    def _declare_mcp_handlers(self) -> None:
        endpoint = f"http://127.0.0.1:{self._mcp_port}/mcp/"
        for fn in self._mcp_handlers:
            cid = getattr(fn, "_robonix_contract_id", None)
            if cid is None:
                continue
            description = (fn.__doc__ or "").strip()
            input_cls = getattr(fn, "_robonix_input_cls", None)
            schema_json = json.dumps(
                input_cls.json_schema()
                if input_cls is not None
                else {"type": "object", "properties": {}, "required": []}
            )
            try:
                self._atlas.declare_mcp(
                    self.id,
                    cid,
                    endpoint,
                    description=description,
                    input_schema_json=schema_json,
                )
            except Exception as e:  # noqa: BLE001
                log.warning("declare mcp %s failed: %s", cid, e)

    def _user_shutdown_then_teardown(self) -> None:
        """Called by Driver(CMD_SHUTDOWN). Run user's @cap.on_shutdown
        first (last chance for state to flush), then framework teardown."""
        if self._on_shutdown is not None:
            try:
                self._on_shutdown()
            except Exception:  # noqa: BLE001
                log.exception("[%s] on_shutdown raised", self.id)
        self._teardown()

    def _teardown(self) -> None:
        # Close any consumer→provider channels we opened so atlas drops
        # the bookkeeping (provider eviction also handles this, but
        # explicit close is cheaper + immediate).
        for ch in self._channels:
            ch.close()
        self._channels.clear()
        self._spawn.shutdown_all()
        if self._driver_server is not None:
            try:
                self._driver_server.stop(grace=2.0)
            except Exception:  # noqa: BLE001
                pass

    def _teardown_and_exit(self) -> None:
        self._stopping.set()
        # Best-effort terminal state push so atlas reflects the shutdown
        # in the brief window before heartbeat eviction would catch it.
        if self._on_shutdown is not None:
            try:
                self._on_shutdown()
            except Exception:  # noqa: BLE001
                log.exception("[%s] on_shutdown raised", self.id)
        try:
            self._set_state("terminated", "process signal teardown")
        except Exception:  # noqa: BLE001
            pass
        self._teardown()
        # Let signal.pause loop exit naturally; don't sys.exit here so atexit
        # hooks can run.


# ── helpers ──────────────────────────────────────────────────────────────
def _full_id(contract_id: str) -> str:
    """Pass-through; contract_id is whatever string atlas/the contract TOML
    says it is — `robonix/...` for built-ins, but a third-party skill could
    name its contracts under any prefix. We don't auto-prepend or strip."""
    return contract_id.strip()


def _caller_file(skip: int = 0) -> Path | None:
    """Walk up the stack looking for the first frame outside this package."""
    here = Path(__file__).parent.resolve()
    frame = inspect.currentframe()
    if frame is None:
        return None
    frame = frame.f_back  # caller of the helper
    for _ in range(skip):
        if frame is None:
            return None
        frame = frame.f_back
    while frame is not None:
        f = frame.f_globals.get("__file__")
        if f:
            p = Path(f).resolve()
            try:
                p.relative_to(here)
            except ValueError:
                return p
        frame = frame.f_back
    return None
