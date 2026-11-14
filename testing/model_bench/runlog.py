# SPDX-License-Identifier: MulanPSL-2.0
"""Capture benchmark output and the component logs appended during a run."""

import contextlib
import hashlib
import os
import shutil
import sys
import traceback
from pathlib import Path


class _Tee:
    def __init__(self, stream, log):
        self.stream, self.log = stream, log

    def write(self, text):
        written = self.stream.write(text)
        self.log.write(text)
        return written

    def flush(self):
        self.stream.flush()
        self.log.flush()

    def __getattr__(self, name):
        return getattr(self.stream, name)


def _files(root: Path):
    if root.is_symlink():
        raise ValueError(f"component log root must not be a symlink: {root}")
    if not root.exists():
        return
    if not root.is_dir():
        raise ValueError(f"component log source must be a directory: {root}")
    for path in sorted(root.iterdir()):
        if path.is_symlink():
            if path.is_dir():
                continue
            raise ValueError(f"component log file must not be a symlink: {path}")
        if path.is_dir():
            yield from _files(path)
        elif path.is_file():
            yield path


def _open(path: Path):
    # Refuse a file symlink introduced after directory enumeration.
    return os.fdopen(os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)), "rb")


def _prefix_hash(stream, size: int):
    digest = hashlib.sha256()
    while size and (chunk := stream.read(min(size, 1024 * 1024))):
        digest.update(chunk)
        size -= len(chunk)
    return digest.digest()


def _snapshot(directory: Path, sources: list[Path], offsets: dict, existing: set[Path]):
    for index, root in enumerate(sources):
        if root in existing and not root.exists():
            raise FileNotFoundError(f"component log directory disappeared: {root}")
        for path in _files(root):
            destination = directory / "components" / str(index) / path.relative_to(root)
            destination.parent.mkdir(parents=True, exist_ok=True)
            with _open(path) as source, destination.open("wb") as target:
                info = os.fstat(source.fileno())
                offset, digest = offsets.get((info.st_dev, info.st_ino), (0, None))
                unchanged = offset and info.st_size >= offset and _prefix_hash(source, offset) == digest
                source.seek(offset if unchanged else 0)
                shutil.copyfileobj(source, target)


@contextlib.contextmanager
def capture(directory: Path, sources: list[Path]):
    offsets = {}
    existing = {root for root in sources if root.exists()}
    for root in sources:
        for path in _files(root):
            with _open(path) as stream:
                info = os.fstat(stream.fileno())
                offsets[(info.st_dev, info.st_ino)] = (info.st_size, _prefix_hash(stream, info.st_size))
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / "bench.stdout.log").open("w", encoding="utf-8") as stdout, (
        directory / "bench.stderr.log"
    ).open("w", encoding="utf-8") as stderr:
        try:
            with contextlib.redirect_stdout(_Tee(sys.stdout, stdout)), contextlib.redirect_stderr(_Tee(sys.stderr, stderr)):
                yield
        except BaseException:
            traceback.print_exc(file=stderr)
            raise
        finally:
            _snapshot(directory, sources, offsets, existing)
