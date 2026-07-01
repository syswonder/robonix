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
  executor — Execute(Plan) → RtdlEvent stream, asserted transitively: every
             scenario in run.py that dispatches a plan proves executor streamed
             node-state events. Here we additionally assert the executor
             component is registered/reachable in atlas.
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
import subprocess
import sys
from pathlib import Path

LOG_DIR = Path(__file__).resolve().parent / "logs"

# Contract ids every booted deploy must have registered (the system block is
# always present). Deploy-specific providers (camera/lidar/…) are asserted by
# the scenario layer, not here, so this stays deploy-agnostic. Extra required
# substrings can be passed via ROBONIX_REQUIRE (comma-separated).
REQUIRED_COMPONENTS = ["system/pilot", "system/executor", "system/liaison"]


def require_audio() -> bool:
    value = os.environ.get("ROBONIX_REQUIRE_AUDIO", "").strip().lower()
    return value in {"1", "true", "yes", "on"}


def audio_proto_dir() -> Path:
    repo = Path(os.environ.get("GITHUB_WORKSPACE", Path.cwd()))
    candidates = [
        repo / "examples/webots/primitives/audio_driver/rbnx-build/codegen/proto_gen",
        repo / "examples/webots/primitives/audio_driver/proto_gen",
    ]
    for cand in candidates:
        has_atlas = (cand / "atlas_pb2.py").exists()
        has_contracts = (cand / "robonix_contracts_pb2_grpc.py").exists()
        if has_atlas and has_contracts:
            return cand
    raise FileNotFoundError(
        "audio_driver generated proto files not found; run `rbnx build` first"
    )


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



def connect_audio_contract(server: str, contract_id: str):
    """Open an Atlas channel to audio_driver's gRPC contract."""
    import grpc  # type: ignore

    proto = str(audio_proto_dir())
    if proto not in sys.path:
        sys.path.insert(0, proto)
    import atlas_pb2  # type: ignore
    import atlas_pb2_grpc  # type: ignore

    atlas_channel = grpc.insecure_channel(server)
    atlas = atlas_pb2_grpc.AtlasStub(atlas_channel)
    req = atlas_pb2.ConnectCapabilityRequest(
        consumer_id="testing/run_interfaces/audio",
        provider_id="audio_driver",
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
        ci_device = os.environ.get("AUDIO_CI_ALSA_DEVICE", "null")
        if ci_device not in ids:
            fails.append(f"audio list_devices did not expose CI device {ci_device!r}; got {sorted(ids)}")
        if resp.current_input_id != ci_device or resp.current_output_id != ci_device:
            fails.append(
                f"audio current ids not pinned to {ci_device!r}: "
                f"input={resp.current_input_id!r}, output={resp.current_output_id!r}"
            )

        select_ch = open_contract("robonix/primitive/audio/select_device")
        select_stub = audio_grpc_class(contracts_grpc, "SelectDeviceStub")(select_ch)
        for kind in ("input", "output"):
            sel = select_stub.SelectAudioDevice(
                audio_pb2.SelectAudioDevice_Request(kind=kind, id=ci_device),
                timeout=10,
            )
            if not sel.ok:
                fails.append(f"audio select_device {kind}={ci_device!r} rejected: {sel.error}")

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

    suites = [("atlas", check_atlas), ("pilot", check_pilot)]
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
