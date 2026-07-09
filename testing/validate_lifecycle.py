#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Validate the rbnx boot / shutdown lifecycle end-to-end on a live deploy.

The full Webots CI suite (scenarios/cap/flow) proves that the stack can be
booted and exercised. It does NOT prove the lifecycle is clean: that
`rbnx shutdown` actually tears the stack down, that no zombie / orphaned
container / stale state survives, and that the stack can be booted AGAIN
after a clean shutdown (i.e. the bug fixed in #134 — Soma packages failing
to re-register after a restart, chassis_provider hanging at "Running",
goal_pose_relay etc. accumulating).

This validator runs three transitions, in order, after the scenario suite
has finished:

  1. `rbnx shutdown` of the first boot, then check:
       * `<manifest-dir>/rbnx-boot/state.json` is gone
       * no `rbnx start -p` wrappers are still alive (the four runtime
         processes the user cares about: atlas, pilot, soma, liaison;
         the rest are sibling `rbnx start -p <pkg>` workers)
       * no provider-owned docker container from this manifest is alive
       * the atlas / pilot / executor / liaison ports are free
  2. Re-`rbnx boot` the same manifest in a clean workdir. Wait for the same
     set of capabilities + providers that scenario waiting already proved
     on the first boot. This is the "issue #128" restart loop.
  3. By default, `rbnx shutdown` of the second boot (final cleanup; the
     report step is expected to capture the resulting clean state). In CI,
     `--leave-running` keeps the second boot alive so the workflow can run
     the scenario suite again against the post-shutdown boot.

Failure in any step fails the workflow (exit non-zero). Detailed checks
are streamed to stderr; the human-readable summary lands on stdout for
the report step to capture.

The script is intentionally a single file with stdlib only — it must run
on the same image the existing `testing/run.py` already does, no new
dependencies.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

# Mirrors the post-scenario wait set in .github/workflows/testing.yml
# (Boot Robonix deployment). Kept in sync manually; if a future scenario
# needs more providers, add them here AND in the workflow.
REQUIRED_CAPS = [
    "robonix/primitive/camera/snapshot",
    "robonix/primitive/lidar/snapshot",
    "robonix/primitive/audio/list_devices",
    "robonix/primitive/audio/select_device",
    "robonix/primitive/audio/mic",
    "robonix/primitive/audio/speaker",
    "robonix/system/scene/list_objects",
    "robonix/system/scene/goal_near",
    "robonix/service/navigation/navigate",
    "robonix/service/navigation/navigate/status",
    "robonix/service/navigation/navigate/cancel",
    "robonix/skill/explore/explore",
    "robonix/skill/explore/explore/status",
    "robonix/skill/explore/explore/cancel",
    "robonix/service/map/save_map",
    "robonix/service/memory/save",
    "robonix/service/memory/search",
    "robonix/service/speech/speak",
    "robonix/service/voiceprint/list",
]
REQUIRED_ACTIVE = [
    "scene", "tiago_camera", "tiago_lidar", "audio_driver",
    "mapping", "nav2", "memory", "speech", "voiceprint",
]

# Runtime processes that the user has called out by name in the issue.
# `pgrep -f` matches the wrapper command; the `rbnx start` form is
# `rbnx start -p <package>`, but during boot the long-lived children
# of atlas/pilot/soma/liaison ALSO appear under the same rbnx binary.
# We check the four user-facing services plus the `rbnx boot` parent
# itself, so the validator can also catch the original "zombie boot
# parent" failure mode (process_group_has_members would self-deadlock).
PROCESS_NAME_PATTERNS = [
    "rbnx boot",
    "robonix-atlas",
    "robonix-pilot",
    "robonix-soma",
    "robonix-liaison",
]

# Same per-run port layout the workflow exposes via env. Keep in sync with
# .github/workflows/testing.yml : "Configure run isolation".
PER_RUN_PORTS = {
    "atlas": int(os.environ.get("ATLAS_PORT", "50051")),
    "executor": int(os.environ.get("EXECUTOR_PORT", "50061")),
    "pilot": int(os.environ.get("PILOT_PORT", "50071")),
    "liaison": int(os.environ.get("LIAISON_PORT", "50081")),
}


# ── small helpers ───────────────────────────────────────────────────────────

def log(msg: str) -> None:
    print(f"[validate-lifecycle] {msg}", flush=True)


def err(msg: str) -> None:
    print(f"[validate-lifecycle][ERROR] {msg}", file=sys.stderr, flush=True)


def run(cmd: list[str], timeout: int = 60, check: bool = False) -> subprocess.CompletedProcess:
    """Run a subprocess; on failure, surface the full combined output."""
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and proc.returncode != 0:
        raise RuntimeError(
            f"command failed: {' '.join(cmd)}\n"
            f"  exit={proc.returncode}\n  stdout=\n{proc.stdout}\n"
            f"  stderr=\n{proc.stderr}"
        )
    return proc


def find_state_file(manifest_dir: Path) -> Path:
    return manifest_dir / "rbnx-boot" / "state.json"


def pgrep(pattern: str) -> list[int]:
    """Return PIDs matching `pattern` via pgrep, [] if pgrep is unavailable."""
    if shutil.which("pgrep") is None:
        return []
    try:
        out = subprocess.run(
            ["pgrep", "-f", pattern], capture_output=True, text=True, timeout=10
        )
    except subprocess.TimeoutExpired:
        return []
    return [int(x) for x in out.stdout.split() if x.strip().isdigit()]


def pid_cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ""
    return raw.replace(b"\0", b" ").decode(errors="replace").strip()


def _run_scope_tokens(manifest_dir: Path) -> list[str]:
    tokens = [str(manifest_dir.resolve())]
    tokens.extend(f":{port}" for port in PER_RUN_PORTS.values())
    return tokens


def scoped_pgrep(pattern: str, manifest_dir: Path) -> list[tuple[int, str]]:
    """Return matching PIDs that belong to this manifest/run.

    CI runners and developer machines may have unrelated Robonix processes alive.
    The lifecycle check must fail on this run's leftovers, not on a stale Atlas
    from another workspace or port. Package processes carry the manifest path in
    argv; system services carry the per-run ports.
    """
    scoped: list[tuple[int, str]] = []
    tokens = _run_scope_tokens(manifest_dir)
    for pid in pgrep(pattern):
        cmdline = pid_cmdline(pid)
        if any(token in cmdline for token in tokens):
            scoped.append((pid, cmdline))
    return scoped


def port_listening(port: int) -> bool:
    """True iff `port` is currently bound to LISTEN on the local host."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1.0)
        try:
            s.connect(("127.0.0.1", port))
        except (ConnectionRefusedError, socket.timeout, OSError):
            return False
        return True


def docker_running(prefix: str) -> list[str]:
    """Names of containers whose name starts with `prefix`."""
    if shutil.which("docker") is None:
        return []
    out = run(["docker", "ps", "--format", "{{.Names}}"], check=False)
    return [n for n in out.stdout.splitlines() if n.startswith(prefix)]


def _ci_container_prefix(sim_container: str | None) -> str | None:
    """Return the current run's container prefix from ci-<run>-sim."""
    if not sim_container:
        return None
    if sim_container.startswith("ci-") and sim_container.endswith("-sim"):
        return sim_container[:-3]
    return None


def _clean_failures(label: str, manifest_dir: Path, sim_container: str | None) -> list[str]:
    failures: list[str] = []

    sp = find_state_file(manifest_dir)
    if sp.exists():
        failures.append(f"{label}: state.json still present at {sp}")

    survivors: list[str] = []
    for pat in PROCESS_NAME_PATTERNS:
        matches = scoped_pgrep(pat, manifest_dir)
        if matches:
            pids = [pid for pid, _ in matches]
            survivors.append(f"{pat} ({len(pids)} pids: {','.join(map(str, pids))})")
    if survivors:
        failures.append(f"{label}: runtime processes still alive: {survivors}")

    bound: list[str] = []
    for name, port in PER_RUN_PORTS.items():
        if port_listening(port):
            bound.append(f"{name}={port}")
    if bound:
        failures.append(f"{label}: ports still bound: {bound}")

    prefix = _ci_container_prefix(sim_container)
    if prefix and sim_container:
        providers = docker_running(prefix)
        stale = [c for c in providers if c != sim_container]
        if stale:
            failures.append(f"{label}: leftover provider containers: {stale}")

    return failures


# ── shutdown-side checks ───────────────────────────────────────────────────

def assert_clean(label: str, manifest_dir: Path, sim_container: str | None) -> None:
    """Verify the host has nothing left over from a `rbnx boot` run."""
    clean_timeout_s = float(os.environ.get("ROBONIX_LIFECYCLE_CLEAN_TIMEOUT_S", "90"))
    log(f"{label}: waiting up to {clean_timeout_s:.0f}s for shutdown cleanup")
    deadline = time.monotonic() + clean_timeout_s
    failures: list[str] = []
    while True:
        failures = _clean_failures(label, manifest_dir, sim_container)
        if not failures:
            log(f"{label}: clean — state.json removed, no leftover processes/ports/containers")
            return
        if time.monotonic() >= deadline:
            for f in failures:
                err(f)
            raise SystemExit(2)
        time.sleep(0.5)


# ── boot-side checks (re-uses the workflow's existing wait pattern) ───────

def wait_for_boot(
    label: str,
    rbnx: str,
    server: str,
    manifest_yaml: Path,
    log_file: Path,
    timeout_s: int = 600,
) -> None:
    """Spawn `rbnx boot -f <manifest>` detached, then poll atlas for caps."""
    log(f"{label}: starting rbnx boot — log -> {log_file}")
    log_file.parent.mkdir(parents=True, exist_ok=True)
    boot_proc = subprocess.Popen(
        [rbnx, "boot", "-f", str(manifest_yaml)],
        stdout=log_file.open("wb"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    log(f"{label}: rbnx boot pid={boot_proc.pid}")
    deadline = time.monotonic() + timeout_s
    missing_since: float | None = None
    while True:
        if boot_proc.poll() is not None:
            raise RuntimeError(
                f"{label}: rbnx boot exited prematurely with code {boot_proc.returncode}; "
                f"see {log_file}"
            )
        caps = run([rbnx, "caps", "-v", "--server", server], check=False)
        out = caps.stdout
        ok = (
            caps.returncode == 0
            and all(c in out for c in REQUIRED_CAPS)
            and all(f"● {p} [ACTIVE]" in out for p in REQUIRED_ACTIVE)
        )
        if ok:
            log(f"{label}: all required providers ACTIVE")
            return
        if time.monotonic() >= deadline:
            err(f"{label}: timeout after {timeout_s}s waiting for providers")
            err("  last caps output:")
            for line in out.splitlines()[-40:]:
                err(f"    {line}")
            err(f"  rbnx boot log tail:")
            try:
                tail = log_file.read_text(errors="replace").splitlines()[-120:]
            except OSError as e:
                tail = [f"<read failed: {e}>"]
            for line in tail:
                err(f"    {line}")
            raise SystemExit(3)
        if missing_since is None:
            missing_since = time.monotonic()
        time.sleep(5)


# ── main ──────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--rbnx", default="rbnx", help="path to rbnx binary")
    ap.add_argument(
        "--manifest", required=True, type=Path,
        help="the generated manifest (e.g. robonix_manifest.ci.generated.yaml)",
    )
    ap.add_argument(
        "--server", required=True,
        help="atlas endpoint, e.g. 127.0.0.1:50051 (per-run ATLAS_PORT)",
    )
    ap.add_argument(
        "--sim-container", default=os.environ.get("ROBONIX_SIM_CONTAINER", ""),
        help="name of the long-running sim container; left running between "
             "the two boots so the second boot can reuse it",
    )
    ap.add_argument(
        "--log-dir", type=Path, default=None,
        help="where to write lifecycle logs (default: <manifest-dir>/rbnx-boot/logs)",
    )
    ap.add_argument(
        "--shutdown-timeout", type=int, default=240,
        help="seconds to wait for rbnx shutdown to finish (default 240)",
    )
    ap.add_argument(
        "--leave-running", action="store_true",
        help="after the second boot is healthy, leave it running instead of "
             "performing the final shutdown",
    )
    args = ap.parse_args()

    manifest_dir = args.manifest.parent
    state_file = find_state_file(manifest_dir)
    log_dir = args.log_dir or (manifest_dir / "rbnx-boot" / "logs")
    log_dir.mkdir(parents=True, exist_ok=True)

    # 1. Verify the first boot is still in scope — state.json must exist.
    if not state_file.exists():
        err(
            f"no boot state at {state_file}. This validator must run AFTER "
            f"`rbnx boot` and the scenario suite. Did the earlier steps run?"
        )
        return 4
    log(f"phase 1: state.json present at {state_file} — first boot still up")

    # 2. Shutdown the first boot.
    log("phase 1: rbnx shutdown — first boot")
    shutdown_log = log_dir / "lifecycle-shutdown-1.log"
    sd = subprocess.run(
        [args.rbnx, "shutdown", "-f", str(args.manifest)],
        capture_output=True, text=True, timeout=args.shutdown_timeout,
    )
    shutdown_log.write_text(
        f"$ {' '.join([args.rbnx, 'shutdown', '-f', str(args.manifest)])}\n"
        f"# exit={sd.returncode}\n"
        f"# --- stdout ---\n{sd.stdout}\n# --- stderr ---\n{sd.stderr}\n"
    )
    if sd.returncode != 0:
        err(f"first shutdown failed with exit {sd.returncode}; see {shutdown_log}")
        return 5
    log(f"phase 1: shutdown returned {sd.returncode}; log -> {shutdown_log}")

    # 3. Assert the first shutdown was actually clean.
    assert_clean("phase 1", manifest_dir, args.sim_container or None)

    # 4. Re-boot — issue #128 regression check.
    boot_log_2 = log_dir / "lifecycle-boot-2.log"
    wait_for_boot(
        "phase 2", args.rbnx, args.server, args.manifest, boot_log_2,
        timeout_s=600,
    )

    # 5. Verify the re-boot is actually clean too (state.json exists, ports bound).
    if not state_file.exists():
        err(
            f"phase 2: re-boot did not write state.json at {state_file}. "
            f"See {boot_log_2}."
        )
        return 6
    if not all(port_listening(p) for p in PER_RUN_PORTS.values()):
        err(
            f"phase 2: not all per-run ports are bound after re-boot: "
            f"{[(n, p) for n, p in PER_RUN_PORTS.items() if not port_listening(p)]}"
        )
        return 7
    log("phase 2: re-boot is healthy — state.json present, all ports bound")

    if args.leave_running:
        log("RESULT: PASS — boot → shutdown → re-boot healthy; leaving stack running")
        summary = {
            "suite": "lifecycle",
            "passed": True,
            "manifest": str(args.manifest),
            "left_running": True,
            "checks": [
                "first_shutdown_clean",
                "reboot_active_providers",
            ],
        }
        print(json.dumps(summary, indent=2))
        return 0

    # 6. Final shutdown.
    log("phase 3: rbnx shutdown — second boot (final cleanup)")
    shutdown_log_2 = log_dir / "lifecycle-shutdown-2.log"
    sd2 = subprocess.run(
        [args.rbnx, "shutdown", "-f", str(args.manifest)],
        capture_output=True, text=True, timeout=args.shutdown_timeout,
    )
    shutdown_log_2.write_text(
        f"$ {' '.join([args.rbnx, 'shutdown', '-f', str(args.manifest)])}\n"
        f"# exit={sd2.returncode}\n"
        f"# --- stdout ---\n{sd2.stdout}\n# --- stderr ---\n{sd2.stderr}\n"
    )
    if sd2.returncode != 0:
        err(
            f"second shutdown failed with exit {sd2.returncode}; "
            f"see {shutdown_log_2}"
        )
        return 8
    log(f"phase 3: shutdown returned {sd2.returncode}; log -> {shutdown_log_2}")

    # 7. Assert the second shutdown was clean too.
    assert_clean("phase 3", manifest_dir, args.sim_container or None)

    log("RESULT: PASS — boot → shutdown → re-boot → shutdown all clean")
    summary = {
        "suite": "lifecycle",
        "passed": True,
        "manifest": str(args.manifest),
        "checks": [
            "first_shutdown_clean",
            "reboot_active_providers",
            "second_shutdown_clean",
        ],
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
