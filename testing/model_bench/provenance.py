# SPDX-License-Identifier: MulanPSL-2.0
"""Source, binary and caller-declared serving facts for a benchmark run."""

import hashlib
import os
import platform
import shutil
import subprocess
from pathlib import Path

SERVING_FIELDS = ("framework", "version", "hardware", "precision", "max_context", "sampling")
SAMPLING_FIELDS = {
    "temperature", "top_p", "top_k", "min_p", "max_tokens", "max_completion_tokens",
    "seed", "stop", "frequency_penalty", "presence_penalty", "repetition_penalty", "mode",
}


def validate_serving(endpoint_kind: str, serving: dict | None) -> None:
    if endpoint_kind not in {"cloud", "local", "robot"}:
        raise ValueError("endpoint kind must be cloud, local or robot")
    if serving is None and endpoint_kind == "cloud":
        return
    if not isinstance(serving, dict):
        raise ValueError("local and robot endpoints require serving facts")
    missing = [key for key in SERVING_FIELDS if not serving.get(key)
               or isinstance(serving[key], str) and not serving[key].strip()]
    if missing:
        raise ValueError("serving facts require nonempty " + ", ".join(missing))
    if any(not isinstance(serving[key], str) for key in SERVING_FIELDS[:4]):
        raise ValueError("serving framework, version, hardware and precision must be strings")
    if (not isinstance(serving["max_context"], int)
            or isinstance(serving["max_context"], bool) or serving["max_context"] <= 0):
        raise ValueError("serving max_context must be a positive integer")
    if not isinstance(serving["sampling"], dict):
        raise ValueError("serving sampling must be an object")


def _run(command: list[str]) -> str | None:
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return result.stdout.strip() if result.returncode == 0 else None


def _git(repo: Path, *args: str) -> str | None:
    return _run(["git", "--no-optional-locks", "-C", str(repo), *args])


def _dirty(repo: Path, path: str = ".") -> bool | None:
    status = _git(repo, "status", "--porcelain", "--untracked-files=normal", "--", path)
    return bool(status) if status is not None else None


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _binary(path: str | None) -> dict:
    if path is None:
        return {"version": None, "sha256": None}
    try:
        digest = _sha256(Path(path))
    except OSError:
        digest = None
    version = _run([path, "--version"])
    return {"version": version[:4096] if version else None, "sha256": digest}


def collect(repo: Path, cases: Path, model: str, endpoint_kind: str, board_id: str,
            judge_model: str | None, rbnx: str, started: float, command: list[str],
            serving: dict | None = None) -> dict:
    validate_serving(endpoint_kind, serving)
    if serving is not None:
        serving = {key: serving[key] for key in SERVING_FIELDS}
        serving["sampling"] = {key: value for key, value in serving["sampling"].items()
                               if key in SAMPLING_FIELDS and (type(value) in (str, int, float, bool, type(None))
                               or key == "stop" and isinstance(value, list)
                               and all(isinstance(item, str) for item in value))}
    repo, cases = repo.resolve(), cases.resolve()
    cases_path = cases.relative_to(repo).as_posix() if cases.is_relative_to(repo) else None
    binary = shutil.which(rbnx)
    runtime = {}
    for name in ("atlas", "executor", "pilot", "soma", "vitals"):
        sibling = Path(binary).parent / f"robonix-{name}" if binary else None
        path = (str(sibling) if sibling and sibling.is_file() and os.access(sibling, os.X_OK)
                else shutil.which(f"robonix-{name}"))
        runtime[name] = _binary(path)
    return {
        "robonix": {"commit": _git(repo, "rev-parse", "HEAD") or None,
                    "dirty": _dirty(repo)},
        "bench": {"commit": _git(repo, "log", "-1", "--format=%H", "--", "testing/model_bench") or None,
                  "dirty": _dirty(repo, "testing/model_bench"),
                  "cases_commit": (_git(repo, "log", "-1", "--format=%H", "--", cases_path) or None)
                  if cases_path is not None else None,
                  "cases_dirty": _dirty(repo, cases_path) if cases_path is not None else None},
        "board": {"id": board_id, "cases_sha256": _sha256(cases)},
        "judge_model": judge_model,
        "planner": {"model": model, "endpoint_kind": endpoint_kind, "serving": serving},
        "system": {"os": platform.system(), "kernel": platform.release(),
                   "python": platform.python_version()},
        "versions": {"rbnx": _binary(binary), "runtime": runtime},
        "start": started,
        "end": None,
        "command": command,
    }
