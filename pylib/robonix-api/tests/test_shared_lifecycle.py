# SPDX-License-Identifier: MulanPSL-2.0
"""Shared lifecycle selection and legacy generated-stub compatibility."""

from pathlib import Path
from types import SimpleNamespace

import pytest

from robonix_api.lifecycle import (
    CMD_ACTIVATE,
    CMD_DEACTIVATE,
    CMD_INIT,
    CMD_SHUTDOWN,
    SHARED_DRIVER_CONTRACT_ID,
    build_lifecycle_servicer,
    generated_grpc_metadata,
    lifecycle_contract_for_module,
)
from robonix_api.capability import _ProviderBase
from robonix_api.result import Deferred, Ok

ROOT = Path(__file__).resolve().parents[3]


class _Response:
    """Minimal stand-in for generated lifecycle_pb2.Driver_Response."""

    def __init__(self, *, ok: bool, state: str, error: str) -> None:
        self.ok = ok
        self.state = state
        self.error = error


class _RpcContext:
    """Minimal gRPC context that exposes post-RPC completion callbacks."""

    def __init__(self) -> None:
        self.callbacks: list[object] = []

    def add_callback(self, callback) -> bool:
        """Record a callback for execution after the response is finished."""
        self.callbacks.append(callback)
        return True

    def finish(self) -> None:
        """Simulate gRPC completing the response."""
        for callback in self.callbacks:
            callback()


def _grpc_module(*, shared: bool, legacy: bool) -> SimpleNamespace:
    """Build a fake generated gRPC module for contract selection tests."""
    attrs: dict[str, object] = {}
    if shared:
        attrs["RobonixLifecycleDriverServicer"] = type(
            "RobonixLifecycleDriverServicer", (), {"Driver": lambda *_: None}
        )
        attrs["add_RobonixLifecycleDriverServicer_to_server"] = lambda *_: None
    if legacy:
        attrs["RobonixSystemSceneDriverServicer"] = type(
            "RobonixSystemSceneDriverServicer", (), {"Driver": lambda *_: None}
        )
        attrs["add_RobonixSystemSceneDriverServicer_to_server"] = lambda *_: None
    return SimpleNamespace(**attrs)


def test_all_legacy_driver_contracts_remain_available_for_old_manifests() -> None:
    legacy_contracts = sorted(
        path
        for path in (ROOT / "capabilities").glob("**/driver.v1.toml")
        if path.parent.name != "lifecycle"
    )

    assert len(legacy_contracts) == 13
    for path in legacy_contracts:
        contents = path.read_text()
        assert "Legacy" in contents, path
        assert "idl" in contents and "lifecycle/srv/Driver.srv" in contents, path


def test_omitted_manifest_prefers_current_shared_stub() -> None:
    module = _grpc_module(shared=True, legacy=True)
    received: list[dict] = []

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        SHARED_DRIVER_CONTRACT_ID,
        allow_old_artifact_fallback=False,
    )
    built = build_lifecycle_servicer(
        "robonix/system/scene",
        module,
        _Response,
        on_init=lambda cfg: received.append(cfg) or Ok(),
        requested_contract_id=SHARED_DRIVER_CONTRACT_ID,
        allow_old_artifact_fallback=False,
    )

    assert selected == (SHARED_DRIVER_CONTRACT_ID, "RobonixLifecycleDriver")
    assert built is not None
    assert built[4] == SHARED_DRIVER_CONTRACT_ID
    response = built[0].Driver(
        SimpleNamespace(command=CMD_INIT, config_json='{"camera":"front"}'),
        None,
    )
    assert response.ok is True
    assert received == [{"camera": "front"}]


def test_generated_driver_metadata_uses_fully_qualified_grpc_route() -> None:
    assert generated_grpc_metadata("RobonixLifecycleDriver", "Driver") == (
        "robonix.contracts.RobonixLifecycleDriver",
        "/robonix.contracts.RobonixLifecycleDriver/Driver",
    )
    assert generated_grpc_metadata("RobonixSystemSceneDriver", "Driver") == (
        "robonix.contracts.RobonixSystemSceneDriver",
        "/robonix.contracts.RobonixSystemSceneDriver/Driver",
    )


def test_explicit_shared_driver_with_empty_handlers_reaches_active(caplog) -> None:
    module = _grpc_module(shared=True, legacy=True)
    transitions: list[tuple[str | None, str]] = []
    built = build_lifecycle_servicer(
        "robonix/system/scene",
        module,
        _Response,
        requested_contract_id=SHARED_DRIVER_CONTRACT_ID,
        on_state_change=lambda state, detail: transitions.append((state, detail)),
    )

    assert built is not None
    servicer = built[0]
    init_response = servicer.Driver(
        SimpleNamespace(command=CMD_INIT, config_json="{}"),
        None,
    )
    activate_response = servicer.Driver(
        SimpleNamespace(command=CMD_ACTIVATE, config_json="{}"),
        None,
    )
    deactivate_response = servicer.Driver(
        SimpleNamespace(command=CMD_DEACTIVATE, config_json="{}"),
        None,
    )
    shutdown_response = servicer.Driver(
        SimpleNamespace(command=CMD_SHUTDOWN, config_json="{}"),
        None,
    )

    assert init_response.ok is True
    assert init_response.state == "inactive"
    assert activate_response.ok is True
    assert activate_response.state == "active"
    assert deactivate_response.ok is True
    assert deactivate_response.state == "inactive"
    assert shutdown_response.ok is True
    assert shutdown_response.state == "terminated"
    assert transitions == [
        ("inactive", ""),
        ("active", ""),
        ("inactive", ""),
        ("terminated", ""),
    ]
    assert "no on_init handler" in caplog.text
    assert "no on_activate handler" in caplog.text
    assert "no on_deactivate handler" in caplog.text
    assert "no on_shutdown handler" in caplog.text


def test_legacy_namespace_driver_still_builds_when_shared_stub_is_absent() -> None:
    module = _grpc_module(shared=False, legacy=True)
    received: list[dict] = []

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        "robonix/system/scene/driver",
        allow_old_artifact_fallback=True,
    )
    built = build_lifecycle_servicer(
        "robonix/system/scene",
        module,
        _Response,
        on_init=lambda cfg: received.append(cfg) or Ok(),
        requested_contract_id="robonix/system/scene/driver",
        allow_old_artifact_fallback=True,
    )

    assert selected == (
        "robonix/system/scene/driver",
        "RobonixSystemSceneDriver",
    )
    assert built is not None
    assert built[4] == "robonix/system/scene/driver"
    response = built[0].Driver(
        SimpleNamespace(
            command=CMD_INIT,
            config_json='{"legacy_option":"still-delivered"}',
        ),
        None,
    )
    assert response.ok is True
    assert received == [{"legacy_option": "still-delivered"}]


def test_legacy_manifest_selection_wins_when_both_stubs_exist() -> None:
    module = _grpc_module(shared=True, legacy=True)

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        "robonix/system/scene/driver",
        allow_old_artifact_fallback=True,
    )

    assert selected == (
        "robonix/system/scene/driver",
        "RobonixSystemSceneDriver",
    )


def test_direct_current_provider_prefers_shared_when_both_stubs_exist() -> None:
    module = _grpc_module(shared=True, legacy=True)

    selected = lifecycle_contract_for_module("robonix/system/scene", module)

    assert selected == (SHARED_DRIVER_CONTRACT_ID, "RobonixLifecycleDriver")


def test_empty_legacy_launcher_selection_defaults_to_shared() -> None:
    module = _grpc_module(shared=True, legacy=True)

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        "",
    )

    assert selected == (SHARED_DRIVER_CONTRACT_ID, "RobonixLifecycleDriver")


def test_legacy_manifest_upgrades_to_shared_runtime_when_legacy_stub_is_absent(
    caplog,
) -> None:
    module = _grpc_module(shared=True, legacy=False)
    received: list[dict] = []

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        "robonix/system/scene/driver",
        allow_old_artifact_fallback=True,
    )
    built = build_lifecycle_servicer(
        "robonix/system/scene",
        module,
        _Response,
        on_init=lambda cfg: received.append(cfg) or Ok(),
        requested_contract_id="robonix/system/scene/driver",
        allow_old_artifact_fallback=True,
    )

    assert selected == (
        SHARED_DRIVER_CONTRACT_ID,
        "RobonixLifecycleDriver",
    )
    assert built is not None
    assert built[4] == SHARED_DRIVER_CONTRACT_ID
    response = built[0].Driver(
        SimpleNamespace(command=CMD_INIT, config_json='{"legacy":true}'),
        None,
    )
    assert response.ok is True
    assert received == [{"legacy": True}]
    assert "legacy manifest selected" in caplog.text
    assert "shared runtime Driver" in caplog.text
    assert "finish migration" in caplog.text


def test_omitted_manifest_without_any_driver_stub_fails_with_rebuild_guidance(
    caplog,
) -> None:
    module = _grpc_module(shared=False, legacy=False)

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        SHARED_DRIVER_CONTRACT_ID,
        allow_old_artifact_fallback=False,
    )

    assert selected is None
    with pytest.raises(RuntimeError, match=r"no usable lifecycle Driver.*rbnx build"):
        build_lifecycle_servicer(
            "robonix/system/scene",
            module,
            _Response,
            requested_contract_id=SHARED_DRIVER_CONTRACT_ID,
            allow_old_artifact_fallback=False,
        )
    assert SHARED_DRIVER_CONTRACT_ID in caplog.text
    assert "rbnx build" in caplog.text


def test_partial_generated_service_fails_closed(caplog) -> None:
    module = SimpleNamespace(
        RobonixLifecycleDriverServicer=type(
            "RobonixLifecycleDriverServicer", (), {"Driver": lambda *_: None}
        )
    )

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        SHARED_DRIVER_CONTRACT_ID,
        allow_old_artifact_fallback=False,
    )

    assert selected is None
    with pytest.raises(RuntimeError, match=r"no usable lifecycle Driver.*rbnx build"):
        build_lifecycle_servicer(
            "robonix/system/scene",
            module,
            _Response,
            requested_contract_id=SHARED_DRIVER_CONTRACT_ID,
            allow_old_artifact_fallback=False,
        )
    assert "incomplete; refusing lifecycle startup" in caplog.text


def test_explicit_shared_selection_does_not_downgrade_when_stub_is_missing(
    caplog,
) -> None:
    module = _grpc_module(shared=False, legacy=True)

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        SHARED_DRIVER_CONTRACT_ID,
        allow_old_artifact_fallback=True,
    )

    assert selected is None
    assert "unavailable in generated stubs" in caplog.text


def test_legacy_manifest_upgrade_requires_launcher_marker(caplog) -> None:
    module = _grpc_module(shared=True, legacy=False)

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        "robonix/system/scene/driver",
    )

    assert selected is None
    assert "did not mark this as a legacy-manifest upgrade" in caplog.text


def test_partial_legacy_stub_rejects_shared_runtime_upgrade(caplog) -> None:
    module = _grpc_module(shared=True, legacy=False)
    module.RobonixSystemSceneDriverServicer = type(
        "RobonixSystemSceneDriverServicer", (), {"Driver": lambda *_: None}
    )

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        "robonix/system/scene/driver",
        allow_old_artifact_fallback=True,
    )

    assert selected is None
    assert "incomplete; refusing shared-runtime upgrade" in caplog.text


def test_unrelated_legacy_driver_selection_is_rejected(caplog) -> None:
    module = _grpc_module(shared=True, legacy=True)

    selected = lifecycle_contract_for_module(
        "robonix/system/scene",
        module,
        "robonix/primitive/camera/driver",
        allow_old_artifact_fallback=True,
    )

    assert selected is None
    assert "unrelated to provider namespace" in caplog.text


def test_provider_base_has_no_implicit_active_promotion_hook() -> None:
    assert not hasattr(_ProviderBase, "_promote_ready_provider_without_driver")


def test_shared_driver_delivers_init_config_and_shutdown_to_handlers() -> None:
    module = _grpc_module(shared=True, legacy=False)
    received: list[object] = []
    built = build_lifecycle_servicer(
        "robonix/system/scene",
        module,
        _Response,
        on_init=lambda cfg: received.append(cfg) or Ok(),
        on_shutdown=lambda: received.append("shutdown") or Ok(),
    )

    assert built is not None
    servicer = built[0]
    init_response = servicer.Driver(
        SimpleNamespace(
            command=CMD_INIT,
            config_json='{"camera_provider_id":"front_camera","web_port":50107}',
        ),
        None,
    )
    shutdown_response = servicer.Driver(
        SimpleNamespace(command=CMD_SHUTDOWN, config_json=""),
        None,
    )

    assert init_response.ok is True
    assert shutdown_response.ok is True
    assert received == [
        {"camera_provider_id": "front_camera", "web_port": 50107},
        "shutdown",
    ]


def test_shutdown_tears_down_server_only_after_rpc_response_finishes() -> None:
    module = _grpc_module(shared=True, legacy=False)
    events: list[str] = []
    context = _RpcContext()
    built = build_lifecycle_servicer(
        "robonix/system/scene",
        module,
        _Response,
        on_init=lambda _cfg: Ok(),
        on_shutdown=lambda: events.append("resources_closed") or Ok(),
        on_shutdown_complete=lambda: events.append("server_stopped"),
    )

    assert built is not None
    response = built[0].Driver(
        SimpleNamespace(command=CMD_SHUTDOWN, config_json=""),
        context,
    )

    assert response.ok is True
    assert events == ["resources_closed"]
    assert len(context.callbacks) == 1
    context.finish()
    assert events == ["resources_closed", "server_stopped"]


def test_deferred_deactivate_keeps_the_provider_in_its_current_state() -> None:
    module = _grpc_module(shared=True, legacy=False)
    transitions: list[tuple[str | None, str]] = []
    reason = "deactivation is not implemented"
    built = build_lifecycle_servicer(
        "robonix/system/scene",
        module,
        _Response,
        on_init=lambda _cfg: Ok(),
        on_deactivate=lambda: Deferred(reason),
        on_state_change=lambda state, detail: transitions.append((state, detail)),
    )

    assert built is not None
    response = built[0].Driver(
        SimpleNamespace(command=CMD_DEACTIVATE, config_json=""),
        None,
    )

    assert response.ok is False
    assert response.state == "deferred"
    assert response.error == reason
    assert transitions == [(None, reason)]
