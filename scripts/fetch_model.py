#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0

"""Fetch and install externally distributed robot render assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import tarfile
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import BinaryIO


BUFFER_BYTES = 1024 * 1024
PROGRESS_BYTES = 64 * 1024 * 1024


@dataclass(frozen=True)
class ExpectedFile:
    path: PurePosixPath
    size_bytes: int
    sha256: str


@dataclass(frozen=True)
class ModelArtifact:
    model_id: str
    version: str
    url: str
    size_bytes: int
    sha256: str
    install_roots: tuple[str, ...]
    files: tuple[ExpectedFile, ...]


def parse_args() -> argparse.Namespace:
    """Parse one manifest, destination, and optional local artifact override."""
    parser = argparse.ArgumentParser(
        description="Fetch and verify one external robot model artifact."
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--artifact-url",
        help="Override the manifest URL, for mirrors and local release testing.",
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        help="Artifact cache directory (default: XDG cache or ~/.cache).",
    )
    parser.add_argument(
        "--offline",
        action="store_true",
        help="Use an already verified cache entry without network access.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Reinstall even when all destination files already match.",
    )
    return parser.parse_args()


def safe_relative_path(raw_path: str, label: str) -> PurePosixPath:
    """Accept a normalized POSIX path that cannot leave the model directory."""
    value = str(raw_path or "").strip()
    path = PurePosixPath(value)
    if (
        not value
        or "\x00" in value
        or "\\" in value
        or path.is_absolute()
        or any(part in {"", ".", ".."} for part in path.parts)
    ):
        raise ValueError(f"invalid {label}: {raw_path!r}")
    return path


def valid_sha256(raw_value: str, label: str) -> str:
    """Normalize one hexadecimal SHA-256 digest or reject the manifest."""
    value = str(raw_value or "").strip().lower()
    if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
        raise ValueError(f"invalid SHA-256 for {label}")
    return value


def positive_size(raw_value: object, label: str) -> int:
    """Require a positive byte count before any network or disk operation."""
    try:
        value = int(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid byte size for {label}") from exc
    if value < 1:
        raise ValueError(f"byte size for {label} must be positive")
    return value


def load_manifest(path: Path, artifact_url: str | None = None) -> ModelArtifact:
    """Load and fully validate the versioned model artifact manifest."""
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("model artifact manifest must be a JSON object")
    if document.get("schemaVersion") != 1:
        raise ValueError("model artifact manifest schemaVersion must be 1")
    artifact = document.get("artifact")
    files = document.get("files")
    roots = document.get("installRoots")
    if not isinstance(artifact, dict) or not isinstance(files, dict):
        raise ValueError("manifest artifact and files must be objects")
    if not isinstance(roots, list) or not roots:
        raise ValueError("manifest installRoots must be a non-empty list")

    install_roots = tuple(
        safe_relative_path(root, "install root").as_posix() for root in roots
    )
    if any("/" in root for root in install_roots):
        raise ValueError("install roots must be top-level model directories")
    if len(set(install_roots)) != len(install_roots):
        raise ValueError("manifest installRoots contains duplicates")

    expected_files: list[ExpectedFile] = []
    for raw_path, metadata in sorted(files.items()):
        if not isinstance(metadata, dict):
            raise ValueError(f"file metadata must be an object: {raw_path}")
        relative_path = safe_relative_path(raw_path, "model file path")
        if relative_path.parts[0] not in install_roots:
            raise ValueError(f"model file is outside installRoots: {raw_path}")
        expected_files.append(
            ExpectedFile(
                path=relative_path,
                size_bytes=positive_size(metadata.get("bytes"), raw_path),
                sha256=valid_sha256(metadata.get("sha256", ""), raw_path),
            )
        )
    if not expected_files:
        raise ValueError("manifest files must not be empty")

    url = str(artifact_url or artifact.get("url") or "").strip()
    parsed_url = urllib.parse.urlparse(url)
    if parsed_url.scheme not in {"file", "http", "https"}:
        raise ValueError("artifact URL must use file, http, or https")
    return ModelArtifact(
        model_id=required_text(document.get("modelId"), "modelId"),
        version=required_text(document.get("version"), "version"),
        url=url,
        size_bytes=positive_size(artifact.get("bytes"), "artifact"),
        sha256=valid_sha256(artifact.get("sha256", ""), "artifact"),
        install_roots=install_roots,
        files=tuple(expected_files),
    )


def required_text(raw_value: object, label: str) -> str:
    """Require non-empty model identity fields used in operator diagnostics."""
    value = str(raw_value or "").strip()
    if not value:
        raise ValueError(f"manifest {label} must not be empty")
    return value


def default_cache_dir() -> Path:
    """Return a user-writable cache shared across Robonix model examples."""
    cache_home = Path(
        os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")
    ).expanduser()
    return cache_home / "robonix" / "model-artifacts"


def sha256_file(path: Path) -> str:
    """Hash one file with bounded memory use."""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(BUFFER_BYTES), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_matches(path: Path, size_bytes: int, sha256: str) -> bool:
    """Check size before paying the cost of hashing a potentially large file."""
    return (
        path.is_file()
        and path.stat().st_size == size_bytes
        and sha256_file(path) == sha256
    )


def installation_matches(output: Path, artifact: ModelArtifact) -> bool:
    """Return true only when every managed destination file is verified."""
    for expected in artifact.files:
        target = output.joinpath(*expected.path.parts)
        if not file_matches(target, expected.size_bytes, expected.sha256):
            return False
    return True


def cache_path(cache_dir: Path, artifact: ModelArtifact) -> Path:
    """Name an immutable cache entry by digest instead of a mutable URL basename."""
    suffixes = "".join(Path(urllib.parse.urlparse(artifact.url).path).suffixes)
    safe_suffix = suffixes if suffixes in {".tar", ".tar.gz", ".tgz"} else ".tar.gz"
    return cache_dir / f"{artifact.sha256}{safe_suffix}"


def _copy_response(
    response: BinaryIO,
    destination: BinaryIO,
    initial_bytes: int,
    expected_bytes: int,
) -> int:
    """Stream one response to disk while enforcing the declared artifact size."""
    received = initial_bytes
    next_progress = ((received // PROGRESS_BYTES) + 1) * PROGRESS_BYTES
    while True:
        chunk = response.read(BUFFER_BYTES)
        if not chunk:
            break
        destination.write(chunk)
        received += len(chunk)
        if received > expected_bytes:
            raise ValueError("artifact download exceeds declared byte size")
        if received >= next_progress:
            print(f"[model] downloaded {received / 1048576:.0f} MiB")
            next_progress += PROGRESS_BYTES
    return received


def download_artifact(
    artifact: ModelArtifact,
    destination: Path,
    offline: bool = False,
) -> Path:
    """Download with HTTP range resume, then atomically publish a verified cache file."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    if file_matches(destination, artifact.size_bytes, artifact.sha256):
        print(f"[model] using verified cache: {destination}")
        return destination
    if offline:
        raise FileNotFoundError(f"verified artifact is not cached: {destination}")

    partial = destination.with_name(f"{destination.name}.part")
    initial_bytes = partial.stat().st_size if partial.is_file() else 0
    if initial_bytes >= artifact.size_bytes:
        partial.unlink()
        initial_bytes = 0
    request = urllib.request.Request(artifact.url)
    if initial_bytes:
        request.add_header("Range", f"bytes={initial_bytes}-")
    try:
        response = urllib.request.urlopen(request, timeout=60)
    except urllib.error.URLError as exc:
        raise RuntimeError(f"download model artifact: {exc}") from exc

    response_status = getattr(response, "status", None)
    append = initial_bytes > 0 and response_status == 206
    if not append:
        initial_bytes = 0
    mode = "ab" if append else "wb"
    with response, partial.open(mode) as output:
        received = _copy_response(
            response,
            output,
            initial_bytes,
            artifact.size_bytes,
        )
        output.flush()
        os.fsync(output.fileno())
    if received != artifact.size_bytes:
        raise ValueError(
            f"artifact size mismatch: expected {artifact.size_bytes}, got {received}"
        )
    if sha256_file(partial) != artifact.sha256:
        partial.unlink(missing_ok=True)
        raise ValueError("artifact SHA-256 mismatch")
    os.replace(partial, destination)
    print(f"[model] cached verified artifact: {destination}")
    return destination


def _copy_member(source: BinaryIO, target: Path, expected: ExpectedFile) -> None:
    """Extract one declared regular file and verify its exact bytes."""
    target.parent.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256()
    written = 0
    with target.open("wb") as output:
        while True:
            chunk = source.read(BUFFER_BYTES)
            if not chunk:
                break
            output.write(chunk)
            digest.update(chunk)
            written += len(chunk)
            if written > expected.size_bytes:
                raise ValueError(f"archive member exceeds declared size: {expected.path}")
        output.flush()
        os.fsync(output.fileno())
    if written != expected.size_bytes or digest.hexdigest() != expected.sha256:
        raise ValueError(f"archive member failed verification: {expected.path}")


def extract_verified(archive_path: Path, staging: Path, artifact: ModelArtifact) -> None:
    """Extract only declared regular files and reject unsafe or unexpected members."""
    expected = {item.path.as_posix(): item for item in artifact.files}
    extracted: set[str] = set()
    with tarfile.open(archive_path, mode="r:*") as archive:
        for member in archive:
            if member.isdir():
                continue
            member_path = safe_relative_path(member.name, "archive member")
            wire_path = member_path.as_posix()
            if not member.isfile() or wire_path not in expected:
                raise ValueError(f"unexpected archive member: {member.name}")
            if wire_path in extracted:
                raise ValueError(f"duplicate archive member: {member.name}")
            expected_file = expected[wire_path]
            if member.size != expected_file.size_bytes:
                raise ValueError(f"archive member size mismatch: {member.name}")
            source = archive.extractfile(member)
            if source is None:
                raise ValueError(f"cannot read archive member: {member.name}")
            with source:
                _copy_member(
                    source,
                    staging.joinpath(*member_path.parts),
                    expected_file,
                )
            extracted.add(wire_path)
    missing = sorted(set(expected) - extracted)
    if missing:
        raise ValueError(f"artifact is missing declared files: {', '.join(missing)}")


def install_roots(staging: Path, output: Path, roots: tuple[str, ...]) -> None:
    """Replace dedicated asset roots atomically and roll back a failed installation."""
    output.mkdir(parents=True, exist_ok=True)
    transaction = f"{os.getpid()}-{time.time_ns()}"
    installed: list[tuple[Path, Path | None]] = []
    try:
        for root in roots:
            source = staging / root
            if not source.is_dir():
                raise ValueError(f"staged artifact has no install root: {root}")
            target = output / root
            backup = output / f".{root}.backup-{transaction}"
            if target.exists():
                os.replace(target, backup)
                active_backup: Path | None = backup
            else:
                active_backup = None
            try:
                os.replace(source, target)
            except Exception:
                if active_backup is not None:
                    os.replace(active_backup, target)
                raise
            installed.append((target, active_backup))
    except Exception:
        for target, backup in reversed(installed):
            if target.exists():
                shutil.rmtree(target)
            if backup is not None and backup.exists():
                os.replace(backup, target)
        raise
    for _target, backup in installed:
        if backup is not None:
            shutil.rmtree(backup)


def fetch_model(
    manifest_path: Path,
    output: Path,
    *,
    artifact_url: str | None = None,
    cache_dir: Path | None = None,
    offline: bool = False,
    force: bool = False,
) -> ModelArtifact:
    """Verify an existing model or download, validate, and atomically install it."""
    artifact = load_manifest(manifest_path, artifact_url)
    output = output.resolve()
    if not force and installation_matches(output, artifact):
        print(f"[model] {artifact.model_id} {artifact.version} is already verified")
        return artifact
    artifact_cache = cache_path((cache_dir or default_cache_dir()).resolve(), artifact)
    downloaded = download_artifact(artifact, artifact_cache, offline)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{output.name}.staging-", dir=output.parent
    ) as temporary:
        staging = Path(temporary)
        extract_verified(downloaded, staging, artifact)
        install_roots(staging, output, artifact.install_roots)
    if not installation_matches(output, artifact):
        raise RuntimeError("installed model failed final verification")
    print(
        f"[model] installed {artifact.model_id} {artifact.version} "
        f"({len(artifact.files)} files) into {output}"
    )
    return artifact


def main() -> int:
    """Run the model fetch transaction and print concise actionable failures."""
    args = parse_args()
    try:
        fetch_model(
            args.manifest,
            args.output,
            artifact_url=args.artifact_url,
            cache_dir=args.cache_dir,
            offline=args.offline,
            force=args.force,
        )
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError, tarfile.TarError) as exc:
        print(f"[model] error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
