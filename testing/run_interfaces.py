# SPDX-License-Identifier: MulanPSL-2.0
"""Interface / RPC contract tests for the robonix system components.

Complements the capability + flow scenarios (run.py): instead of asserting that
a *task* produces the right tool calls, this asserts that each system
component's gRPC interface answers correctly on a live deploy.

  atlas    — Query / ListContracts / InspectAtlas / channels / tools, via the
             `rbnx` inspection subcommands that wrap those RPCs. We assert each
             returns well-formed JSON and the expected system components +
             at least one primitive are registered.
  pilot    — SubmitTask, via `rbnx ask` with a no-tool chit-chat prompt. We
             assert the stream yields final text and a Completed status with
             zero capability calls (pure conversation, no plan dispatched).
  executor — list/control RPCs are exercised directly, outside RTDL. We assert
             that cancel-all completes against an idle executor and does not
             create a self-referential control plan. Execute(Plan) → RtdlEvent
             remains asserted transitively by every dispatched scenario.
  liaison  — registered + reachable in atlas (its SubmitTask/StartVoiceSession
             endpoint is advertised). A full direct SubmitTask probe needs the
             liaison gRPC stub from a built deploy and is a documented follow-up.

Every command's raw output is written under ``logs/iface.*`` so CI keeps a full
trace. Exit non-zero if any interface assertion fails.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent / "logs"

# Contract ids every booted deploy must have registered (the system block is
# always present). Deploy-specific providers (camera/lidar/…) are asserted by
# the scenario layer, not here, so this stays deploy-agnostic. Extra required
# substrings can be passed via ROBONIX_REQUIRE (comma-separated).
REQUIRED_COMPONENTS = ["system/pilot", "system/executor", "system/liaison"]
GENERATED_PROTO_FILES = (
    "atlas_pb2.py",
    "atlas_pb2_grpc.py",
    "audio_pb2.py",
    "executor_pb2.py",
    "robonix_contracts_pb2.py",
    "robonix_contracts_pb2_grpc.py",
)
AUDIO_PROVIDER_NAME = "audio_driver"


def require_audio() -> bool:
    value = os.environ.get("ROBONIX_REQUIRE_AUDIO", "").strip().lower()
    return value in {"1", "true", "yes", "on"}


def _yaml_scalar(value: str) -> str:
    """Decode the simple quoted or unquoted scalars used by deploy entries."""
    value = value.strip()
    if value[:1] in {'"', "'"} and value[-1:] == value[:1]:
        return value[1:-1]
    return value.split(" #", 1)[0].strip()


def _audio_source_from_manifest(manifest: Path) -> tuple[str, str] | None:
    """Return the selected audio entry's ``(path|url, value)`` pair.

    The deployment schema writes list entries with ``- name:`` first. Parsing
    only that small stable surface keeps this harness stdlib-only; CI installs
    PyYAML for scenarios, but interface discovery should also work locally
    before that optional dependency is installed.
    """
    if not manifest.is_file():
        return None
    entries: list[dict[str, str]] = []
    current: dict[str, str] | None = None
    current_indent = -1
    for raw_line in manifest.read_text().splitlines():
        stripped = raw_line.lstrip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(raw_line) - len(stripped)
        item = re.match(r"-\s+name\s*:\s*(.+)$", stripped)
        if item:
            if current is not None:
                entries.append(current)
            current = {"name": _yaml_scalar(item.group(1))}
            current_indent = indent
            continue
        if current is not None and indent <= current_indent:
            entries.append(current)
            current = None
        if current is not None:
            source = re.match(r"(path|url)\s*:\s*(.+)$", stripped)
            if source:
                current[source.group(1)] = _yaml_scalar(source.group(2))
    if current is not None:
        entries.append(current)
    for entry in entries:
        if entry.get("name") != AUDIO_PROVIDER_NAME:
            continue
        for source_kind in ("url", "path"):
            if entry.get(source_kind):
                return source_kind, entry[source_kind]
    return None


def _remote_repo_name(url: str) -> str:
    """Mirror rbnx's URL-cache basename rule for one remote package."""
    name = url.rstrip("/").rsplit("/", 1)[-1].rsplit(":", 1)[-1]
    if name.endswith(".git"):
        name = name[:-4]
    if not name or name in {".", ".."}:
        raise ValueError(f"invalid package URL basename: {url!r}")
    return name


def _proto_dirs_for_package(package_root: Path) -> list[Path]:
    """Return current and pre-rbnx-build generated-proto layouts in order."""
    return [
        package_root / "rbnx-build/codegen/proto_gen",
        package_root / "proto_gen",
    ]


def generated_proto_dir(
    deployment_dir: Path | None = None,
    workspace: Path | None = None,
) -> Path:
    """Resolve one complete generated tree for the selected deployment.

    Remote packages are built under ``<deployment>/rbnx-boot/cache/<repo>``.
    Resolve the audio package from that deployment's manifest before trying
    the historical in-tree locations, so a stale legacy build cannot shadow
    the package that ``rbnx build`` and ``rbnx boot`` actually selected.
    """
    if workspace is None:
        configured_workspace = os.environ.get("GITHUB_WORKSPACE", "").strip()
        workspace = (
            Path(configured_workspace)
            if configured_workspace
            else Path(__file__).resolve().parents[1]
        )
    workspace = workspace.resolve()
    if deployment_dir is None:
        configured = os.environ.get("ROBONIX_DEPLOYMENT_DIR", "").strip()
        deployment_dir = (
            Path(configured) if configured else workspace / "examples/webots"
        )
    if not deployment_dir.is_absolute():
        deployment_dir = workspace / deployment_dir
    deployment_dir = deployment_dir.resolve()

    package_roots: list[Path] = []
    source = _audio_source_from_manifest(deployment_dir / "robonix_manifest.yaml")
    if source is not None:
        source_kind, value = source
        if source_kind == "url":
            package_roots.append(
                deployment_dir / "rbnx-boot/cache" / _remote_repo_name(value)
            )
        else:
            expanded = Path(os.path.expandvars(os.path.expanduser(value)))
            package_roots.append(
                expanded if expanded.is_absolute() else deployment_dir / expanded
            )

    # Keep the known external cache location as a deterministic fallback for
    # generated/older deployment manifests that omit the source entry.
    package_roots.append(deployment_dir / "rbnx-boot/cache/primitive-audio-driver-rbnx")
    # Intentional compatibility for old checkouts where audio lived in-tree.
    package_roots.append(deployment_dir / "primitives/audio_driver")

    candidates: list[Path] = []
    for root in package_roots:
        for candidate in _proto_dirs_for_package(root.resolve()):
            if candidate not in candidates:
                candidates.append(candidate)
    missing_by_candidate: list[str] = []
    for candidate in candidates:
        missing = [
            name for name in GENERATED_PROTO_FILES if not (candidate / name).is_file()
        ]
        if not missing:
            return candidate
        missing_by_candidate.append(f"{candidate} (missing {', '.join(missing)})")
    searched = "; ".join(missing_by_candidate)
    raise FileNotFoundError(
        f"generated interface proto files not found for deployment {deployment_dir}; "
        f"searched {searched}; run `rbnx build` in that deployment first"
    )


# Kept as a descriptive alias for the audio-specific callers below.
audio_proto_dir = generated_proto_dir


def sh(cmd: list[str], log_name: str, timeout: int = 120) -> tuple[str, int]:
    """Run a command, tee stdout+stderr to logs/<log_name>, return (stdout, rc)."""
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    (LOG_DIR / log_name).write_text(
        f"$ {' '.join(cmd)}\n# exit={proc.returncode}\n"
        f"# --- stdout ---\n{proc.stdout}\n# --- stderr ---\n{proc.stderr}\n"
    )
    return proc.stdout, proc.returncode


def check_atlas(rbnx: str, server: str) -> list[str]:
    """Exercise the atlas query RPCs; assert JSON + expected registrations.

    `rbnx caps/contracts/tools --json` wrap atlas Query / ListContracts and the
    MCP tool listing. We parse each as JSON (proves the RPC + serialization) and
    assert the required system components appear in the caps listing. Extra
    required contract substrings come from ROBONIX_REQUIRE (comma-separated).
    """
    fails: list[str] = []
    caps_out = ""
    for sub, log in [
        (["caps"], "iface.atlas_caps.json"),
        (["contracts"], "iface.atlas_contracts.json"),
        (["tools"], "iface.atlas_tools.json"),
    ]:
        out, rc = sh([rbnx, *sub, "--server", server, "--json"], log)
        if sub[0] == "caps":
            caps_out = out
        if rc != 0:
            fails.append(f"`rbnx {sub[0]} --json` exit={rc}")
            continue
        try:
            json.loads(out)
        except json.JSONDecodeError as e:
            fails.append(f"`rbnx {sub[0]} --json` not valid JSON: {e}")

    required = list(REQUIRED_COMPONENTS)
    extra = os.environ.get("ROBONIX_REQUIRE", "").strip()
    if extra:
        required += [s.strip() for s in extra.split(",") if s.strip()]
    for want in required:
        if want not in caps_out:
            fails.append(f"atlas caps missing a registration for ~{want!r}")
    return fails


def check_pilot(rbnx: str, server: str) -> list[str]:
    """Exercise pilot SubmitTask with a no-tool prompt; assert clean completion.

    A pure chit-chat prompt has no matching scenario, so the fake VLM returns an
    empty-tree `done` envelope. The SubmitTask stream must therefore carry a
    final_text and a non-FAILED terminal status, and dispatch zero capability
    calls (no plan).
    """
    fails: list[str] = []
    out, rc = sh(
        [rbnx, "ask", "ci-iface: just say hello, do not use any tools", "--json", "--server", server],
        "iface.pilot_submit_task.jsonl",
        timeout=120,
    )
    events = []
    for line in out.splitlines():
        line = line.strip()
        if line:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    if rc != 0:
        fails.append(f"pilot SubmitTask (`rbnx ask`) exit={rc}")
    if not any(ev.get("final_text") for ev in events):
        fails.append("pilot SubmitTask produced no final_text")
    calls = [c for ev in events if ev.get("plan") for c in ev["plan"].get("calls", [])]
    if calls:
        fails.append(f"chit-chat unexpectedly dispatched calls: {[c.get('contract_id') for c in calls]}")
    if any((ev.get("status") or {}).get("state") == 2 for ev in events):
        fails.append("pilot SubmitTask ended FAILED on a trivial prompt")
    return fails



def connect_contract(
    server: str,
    *,
    consumer_id: str,
    provider_id: str,
    contract_id: str,
):
    """Open an Atlas-managed gRPC channel to one provider contract."""
    import grpc  # type: ignore

    proto = str(generated_proto_dir())
    if proto not in sys.path:
        sys.path.insert(0, proto)
    import atlas_pb2  # type: ignore
    import atlas_pb2_grpc  # type: ignore

    atlas_channel = grpc.insecure_channel(server)
    atlas = atlas_pb2_grpc.AtlasStub(atlas_channel)
    req = atlas_pb2.ConnectCapabilityRequest(
        consumer_id=consumer_id,
        provider_id=provider_id,
        contract_id=contract_id,
        transport=atlas_pb2.TRANSPORT_GRPC,
    )
    resp = atlas.ConnectCapability(req, timeout=10)
    endpoint = resp.endpoint if resp.endpoint.startswith("http") else f"{resp.endpoint}"
    if endpoint.startswith("http://"):
        endpoint = endpoint[len("http://"):]
    elif endpoint.startswith("https://"):
        endpoint = endpoint[len("https://"):]
    return resp.channel_id, atlas, grpc.insecure_channel(endpoint)


def connect_audio_contract(server: str, contract_id: str):
    """Open an Atlas channel to audio_driver's gRPC contract."""
    return connect_contract(
        server,
        consumer_id="testing/run_interfaces/audio",
        provider_id="audio_driver",
        contract_id=contract_id,
    )


def executor_grpc_class(module, suffix: str):
    name = f"RobonixSystemExecutor{suffix}"
    cls = getattr(module, name, None)
    if cls is None:
        raise AttributeError(f"executor gRPC class not found: {name}")
    return cls


def check_executor(server: str) -> list[str]:
    """Exercise Executor's out-of-band plan-control authority directly.

    Plan control is deliberately not an RTDL capability exposed to the model.
    Calling cancel-all against an idle runtime must complete successfully and
    must leave the authoritative active-plan table empty.
    """
    fails: list[str] = []
    try:
        proto = str(generated_proto_dir())
        if proto not in sys.path:
            sys.path.insert(0, proto)
        import atlas_pb2  # type: ignore
        import executor_pb2  # type: ignore
        import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore
    except Exception as e:  # noqa: BLE001
        return [f"executor proto imports failed: {e}"]

    opened: list[tuple[object, str]] = []

    def open_contract(contract_id: str):
        channel_id, atlas, channel = connect_contract(
            server,
            consumer_id="testing/run_interfaces/executor",
            provider_id="executor",
            contract_id=contract_id,
        )
        opened.append((atlas, channel_id))
        return channel

    snapshots: dict[str, object] = {}
    try:
        list_ch = open_contract("robonix/system/executor/list_active_plans")
        list_stub = executor_grpc_class(
            contracts_grpc, "ListActivePlansStub"
        )(list_ch)

        def list_active(label: str) -> dict:
            response = list_stub.ListActivePlans(
                executor_pb2.ListActivePlans_Request(), timeout=10
            )
            if not response.success:
                raise RuntimeError(response.error or "ListActivePlans rejected")
            parsed = json.loads(response.plans_json)
            snapshots[label] = parsed
            return parsed

        before = list_active("before")
        if before.get("count") != 0 or before.get("plans") != []:
            fails.append(f"executor was not idle before control probe: {before}")

        control_ch = open_contract("robonix/system/executor/control_plan")
        control_stub = executor_grpc_class(
            contracts_grpc, "ControlPlanStub"
        )(control_ch)
        response = control_stub.ControlPlan(
            executor_pb2.ControlPlan_Request(
                action="cancel_all",
                wait_ms=1_000,
            ),
            timeout=5,
        )
        snapshots["control"] = {
            "success": response.success,
            "completed": response.completed,
            "message": response.message,
            "error": response.error,
        }
        if not response.success or not response.completed:
            fails.append(
                "executor cancel_all did not complete: "
                f"success={response.success}, completed={response.completed}, "
                f"error={response.error!r}"
            )

        after = list_active("after")
        if after.get("count") != 0 or after.get("plans") != []:
            fails.append(f"plan-control RPC created or retained a plan: {after}")
    except Exception as e:  # noqa: BLE001
        fails.append(f"executor plan-control RPC smoke failed: {e}")
    finally:
        (LOG_DIR / "iface.executor_plan_control.json").write_text(
            json.dumps(snapshots, indent=2, sort_keys=True) + "\n"
        )
        for atlas, channel_id in opened:
            try:
                atlas.DisconnectCapability(
                    atlas_pb2.DisconnectCapabilityRequest(channel_id=channel_id),
                    timeout=5,
                )
            except Exception:
                pass
    return fails


def audio_grpc_class(module, suffix: str):
    for prefix in ("RobonixPrimitiveAudio", "PrimitiveAudio"):
        name = f"{prefix}{suffix}"
        cls = getattr(module, name, None)
        if cls is not None:
            return cls
    raise AttributeError(f"audio gRPC class not found for suffix {suffix}")


def check_audio(server: str) -> list[str]:
    """Exercise audio list/select plus mic/speaker streams against ALSA null."""
    fails: list[str] = []
    try:
        proto = str(audio_proto_dir())
        if proto not in sys.path:
            sys.path.insert(0, proto)
        from google.protobuf import empty_pb2  # type: ignore
        import atlas_pb2  # type: ignore
        import audio_pb2  # type: ignore
        import robonix_contracts_pb2_grpc as contracts_grpc  # type: ignore
    except Exception as e:  # noqa: BLE001
        return [f"audio proto imports failed: {e}"]

    opened: list[tuple[object, str]] = []

    def open_contract(contract_id: str):
        channel_id, atlas, channel = connect_audio_contract(server, contract_id)
        opened.append((atlas, channel_id))
        return channel

    try:
        list_ch = open_contract("robonix/primitive/audio/list_devices")
        list_stub = audio_grpc_class(contracts_grpc, "ListDevicesStub")(list_ch)
        resp = list_stub.ListAudioDevices(audio_pb2.ListAudioDevices_Request(), timeout=10)
        devices = list(resp.devices)
        if not devices:
            fails.append("audio list_devices returned no devices")
        ids = {d.id for d in devices}
        expected_input = os.environ.get("AUDIO_MIC_DEVICE", "").strip()
        expected_output = os.environ.get("AUDIO_SPEAKER_DEVICE", "").strip()
        if expected_input and expected_input not in ids:
            fails.append(f"audio list_devices did not expose configured input {expected_input!r}; got {sorted(ids)}")
        if expected_output and expected_output not in ids:
            fails.append(f"audio list_devices did not expose configured output {expected_output!r}; got {sorted(ids)}")
        if expected_input and resp.current_input_id != expected_input:
            fails.append(
                f"audio current input is {resp.current_input_id!r}, expected {expected_input!r}"
            )
        if expected_output and resp.current_output_id != expected_output:
            fails.append(
                f"audio current output is {resp.current_output_id!r}, expected {expected_output!r}"
            )

        select_ch = open_contract("robonix/primitive/audio/select_device")
        select_stub = audio_grpc_class(contracts_grpc, "SelectDeviceStub")(select_ch)
        for kind, device_id in (("input", expected_input), ("output", expected_output)):
            if not device_id:
                continue
            sel = select_stub.SelectAudioDevice(
                audio_pb2.SelectAudioDevice_Request(kind=kind, id=device_id),
                timeout=10,
            )
            if not sel.ok:
                fails.append(f"audio select_device {kind}={device_id!r} rejected: {sel.error}")

        mic_ch = open_contract("robonix/primitive/audio/mic")
        mic_stub = audio_grpc_class(contracts_grpc, "MicStub")(mic_ch)
        stream = mic_stub.Mic(empty_pb2.Empty(), timeout=10)
        try:
            chunk = next(stream)
            if not chunk.data:
                fails.append("audio mic stream returned an empty chunk")
        finally:
            cancel = getattr(stream, "cancel", None)
            if cancel:
                cancel()

        speaker_ch = open_contract("robonix/primitive/audio/speaker")
        speaker_stub = audio_grpc_class(contracts_grpc, "SpeakerStub")(speaker_ch)
        silence = b"\0\0" * 1600
        chunks = iter([audio_pb2.AudioChunk(data=silence, duration_s=0.1)])
        speaker_stub.Speaker(chunks, timeout=10)
    except Exception as e:  # noqa: BLE001
        fails.append(f"audio RPC smoke failed: {e}")
    finally:
        for atlas, channel_id in opened:
            try:
                atlas.DisconnectCapability(
                    atlas_pb2.DisconnectCapabilityRequest(channel_id=channel_id),
                    timeout=5,
                )
            except Exception:
                pass
    return fails


def main() -> int:
    ap = argparse.ArgumentParser(description="robonix interface / RPC contract tests")
    ap.add_argument("--rbnx", default=os.environ.get("RBNX_BIN", "rbnx"))
    ap.add_argument("--server", default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"))
    args = ap.parse_args()
    LOG_DIR.mkdir(exist_ok=True)

    suites = [
        ("atlas", check_atlas),
        ("pilot", check_pilot),
        ("executor", lambda _rbnx, server: check_executor(server)),
    ]
    if require_audio():
        suites.append(("audio", lambda _rbnx, server: check_audio(server)))
    failed = False
    for name, fn in suites:
        print(f"\n=== interface: {name} ===")
        errs = fn(args.rbnx, args.server)
        if errs:
            failed = True
            for e in errs:
                print(f"  FAIL - {e}")
        else:
            print("  PASS")

    print(f"\n=== logs in {LOG_DIR} ===")
    print("RESULT:", "FAIL" if failed else "PASS")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
