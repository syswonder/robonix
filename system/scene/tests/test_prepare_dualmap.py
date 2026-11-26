# SPDX-License-Identifier: MulanPSL-2.0
"""Pinned source preparation preserves old generations and rejects changed code."""
import hashlib
import importlib.util
import io
from pathlib import Path
import subprocess
import tarfile

import pytest


def module():
    source = Path(__file__).resolve().parents[1] / "scripts/prepare_dualmap.py"
    spec = importlib.util.spec_from_file_location("prepare_dualmap", source)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def setup_source(tmp_path, monkeypatch):
    prepare = module()
    monkeypatch.setattr(prepare, "PKG", tmp_path)
    original = b"value = 1\n"
    monkeypatch.setattr(prepare, "ORIGINAL", {"utils/object_detector.py": hashlib.sha256(original).hexdigest()})
    patch = tmp_path / "change.patch"
    patch.write_text("--- a/utils/object_detector.py\n+++ b/utils/object_detector.py\n@@ -1 +1 @@\n-value = 1\n+value = 2\n")
    monkeypatch.setattr(prepare, "PATCH", patch)
    archive = tmp_path / "rbnx-build/dualmap" / f"{prepare.COMMIT}.tar.gz"
    archive.parent.mkdir(parents=True)
    with tarfile.open(archive, "w:gz") as bundle:
        member = tarfile.TarInfo(f"DualMap-{prepare.COMMIT}/utils/object_detector.py")
        member.size = len(original)
        bundle.addfile(member, io.BytesIO(original))
    monkeypatch.setattr(prepare, "ARCHIVE_SHA256", prepare.checksum(archive))
    return prepare


def test_cached_source_is_patched_and_verified_before_reuse(tmp_path, monkeypatch):
    prepare = setup_source(tmp_path, monkeypatch)
    source = prepare.prepare()
    assert (source / "utils/object_detector.py").read_text() == "value = 2\n"
    assert prepare.prepare() == source
    (source / "utils/object_detector.py").write_text("value = 3\n")
    with pytest.raises(RuntimeError, match="changed"):
        prepare.prepare()


def test_failed_patch_does_not_change_the_previous_source(tmp_path, monkeypatch):
    prepare = setup_source(tmp_path, monkeypatch)
    source = prepare.prepare()
    prepare.PATCH.write_text("--- a/utils/object_detector.py\n+++ b/utils/object_detector.py\n@@ -1 +1 @@\n-value = 9\n+value = 3\n")
    with pytest.raises(subprocess.CalledProcessError):
        prepare.prepare()
    assert (source / "utils/object_detector.py").read_text() == "value = 2\n"
    assert not list(source.parent.glob(".dualmap-*"))


@pytest.mark.parametrize("name,link", [("utils/../../escape.py", False), ("utils/link.py", True)])
def test_source_archive_rejects_paths_and_links(tmp_path, name, link):
    prepare = module()
    archive = tmp_path / "unsafe.tar.gz"
    with tarfile.open(archive, "w:gz") as bundle:
        member = tarfile.TarInfo(f"DualMap-{prepare.COMMIT}/{name}")
        if link:
            member.type, member.linkname = tarfile.SYMTYPE, "/etc/passwd"
        bundle.addfile(member)
    destination = tmp_path / "source"
    destination.mkdir()
    with pytest.raises(ValueError):
        prepare.extract(archive, destination)
    assert not (tmp_path / "escape.py").exists()


def test_bad_mirror_archive_falls_back_to_the_verified_origin(tmp_path, monkeypatch):
    prepare = module()
    expected = b"verified archive"
    monkeypatch.setattr(prepare, "ARCHIVE_SHA256", hashlib.sha256(expected).hexdigest())
    monkeypatch.setenv("RBNX_GH_MIRROR", "https://example.invalid")
    sources = []

    def download(command, **kwargs):
        sources.append(command[-1])
        Path(command[command.index("-o") + 1]).write_bytes(b"corrupt" if len(sources) == 1 else expected)

    monkeypatch.setattr(prepare.subprocess, "run", download)
    archive = prepare.fetch_archive(tmp_path)
    assert archive.read_bytes() == expected
    assert len(sources) == 2 and sources[1].startswith("https://codeload.github.com/")
