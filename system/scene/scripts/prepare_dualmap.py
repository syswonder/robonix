#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Prepare the pinned DualMap source with an injectable inference implementation."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

PKG = Path(__file__).resolve().parents[1]
COMMIT = "157235ec49e6a1f439babbc571c4c02ad1f06aa9"
ARCHIVE_SHA256 = "4d7e0456565215e8e3de99ecf6fb80ce1dbeae825a1db9a34bfa68182766e592"
PATCH = PKG / "docker/dualmap-inference.patch"
ORIGINAL = {
    "utils/object_detector.py": "2d572df8b6a9a2b8c94cb0aeee5ff30927a968baac46b85c7981bc5a8cfcf463",
    "utils/pcd_utils.py": "6429d39537b8e5f81242d878a0aa51490cc4581434535f046468f0801200a212",
    "utils/tracker.py": "27d912632f6d69a91947b0b9df384ed5089534042e80089c18d3846191fb9adb",
}


def checksum(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(root):
    root = Path(root)
    saved = json.loads((root / ".rbnx-inference.json").read_text())
    if (saved["commit"] != COMMIT or saved["patch"] != checksum(PATCH)
            or any(checksum(root / name) != digest for name, digest in saved["files"].items())
            or set(saved["files"]) != set(ORIGINAL)):
        raise RuntimeError("Prepared DualMap source changed; rebuild the selected environment")


def fetch_archive(cache):
    cache.mkdir(parents=True, exist_ok=True)
    archive = cache / f"{COMMIT}.tar.gz"
    if archive.is_file() and checksum(archive) == ARCHIVE_SHA256:
        return archive
    official = f"https://codeload.github.com/Eku127/DualMap/tar.gz/{COMMIT}"
    github = f"https://github.com/Eku127/DualMap/archive/{COMMIT}.tar.gz"
    mirror = os.environ.get("RBNX_GH_MIRROR", "").rstrip("/")
    urls = [f"{mirror}/{github}", official] if mirror else [official]
    pending = archive.with_suffix(".part")
    for url in urls:
        try:
            subprocess.run(["curl", "-fL", "--connect-timeout", "15", "--max-time", "180",
                            "--retry", "1", "--speed-time", "30", "--speed-limit", "4096",
                            "-o", str(pending), url], check=True, timeout=400)
            if checksum(pending) != ARCHIVE_SHA256:
                raise ValueError("DualMap source archive checksum mismatch")
            pending.replace(archive)
            return archive
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            print(f"[scene/dualmap] {type(error).__name__}; trying next source", flush=True)
        finally:
            pending.unlink(missing_ok=True)
    raise RuntimeError("Could not download the verified DualMap source")


def extract(archive, destination):
    with tarfile.open(archive, "r:gz") as bundle:
        for member in bundle:
            parts = Path(member.name).parts
            if not parts or parts[0] != f"DualMap-{COMMIT}":
                raise ValueError("Unexpected DualMap archive prefix")
            relative = Path(*parts[1:])
            if not relative.parts or relative.parts[0] not in ("utils", "config", "LICENSE"):
                continue
            target = destination / relative
            if not target.resolve().is_relative_to(destination.resolve()):
                raise ValueError("Unsafe DualMap archive path")
            if member.isdir():
                continue
            if not member.isfile():
                raise ValueError("DualMap source must contain only regular files")
            target.parent.mkdir(parents=True, exist_ok=True)
            with bundle.extractfile(member) as source, target.open("wb") as output:
                shutil.copyfileobj(source, output)


def prepare(destination=None):
    cache = PKG / "rbnx-build/dualmap"
    key = checksum(PATCH)
    destination = Path(destination) if destination else cache / key
    if destination.exists():
        verify(destination)
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    archive = fetch_archive(cache)
    with tempfile.TemporaryDirectory(prefix=".dualmap-", dir=destination.parent) as temporary:
        staged = Path(temporary) / "source"
        staged.mkdir()
        extract(archive, staged)
        if any(checksum(staged / name) != digest for name, digest in ORIGINAL.items()):
            raise RuntimeError("DualMap source differs from the pinned version")
        subprocess.run(["git", "apply", "--check", str(PATCH)], cwd=staged, check=True, timeout=30)
        subprocess.run(["git", "apply", str(PATCH)], cwd=staged, check=True, timeout=30)
        for name in ORIGINAL:
            compile((staged / name).read_bytes(), str(staged / name), "exec")
        saved = dict(commit=COMMIT, patch=key,
                     files={name: checksum(staged / name) for name in ORIGINAL})
        (staged / ".rbnx-inference.json").write_text(json.dumps(saved, indent=2) + "\n")
        verify(staged)
        staged.rename(destination)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path)
    args = parser.parse_args()
    print(prepare(args.root))
