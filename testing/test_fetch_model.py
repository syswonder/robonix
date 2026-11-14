# SPDX-License-Identifier: MulanPSL-2.0

from __future__ import annotations

import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "fetch_model.py"
SPEC = importlib.util.spec_from_file_location("fetch_model", SCRIPT)
assert SPEC and SPEC.loader
fetch_model = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = fetch_model
SPEC.loader.exec_module(fetch_model)


class FetchModelTest(unittest.TestCase):
    def setUp(self) -> None:
        """Create one isolated release artifact and matching manifest."""
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.archive = self.root / "robot-model.tar.gz"
        self.payload = b"solid robot\nendsolid robot\n"
        self._write_archive({"meshes/robot.stl": self.payload})
        self.manifest = self.root / "model-assets.json"
        self._write_manifest()
        self.output = self.root / "model"
        self.cache = self.root / "cache"

    def tearDown(self) -> None:
        """Release temporary artifacts after each transaction test."""
        self.temporary.cleanup()

    def _write_archive(self, files: dict[str, bytes]) -> None:
        """Build a deterministic-enough local tar fixture from named byte payloads."""
        with tarfile.open(self.archive, "w:gz") as archive:
            for name, data in files.items():
                info = tarfile.TarInfo(name)
                info.size = len(data)
                archive.addfile(info, io.BytesIO(data))

    def _write_manifest(self) -> None:
        """Record exact archive and extracted-file checksums in schema version 1."""
        document = {
            "schemaVersion": 1,
            "modelId": "fixture-robot",
            "version": "v1",
            "artifact": {
                "url": self.archive.as_uri(),
                "bytes": self.archive.stat().st_size,
                "sha256": hashlib.sha256(self.archive.read_bytes()).hexdigest(),
            },
            "installRoots": ["meshes"],
            "files": {
                "meshes/robot.stl": {
                    "bytes": len(self.payload),
                    "sha256": hashlib.sha256(self.payload).hexdigest(),
                }
            },
        }
        self.manifest.write_text(json.dumps(document), encoding="utf-8")

    def test_fetches_verifies_and_reuses_installation(self) -> None:
        """A second invocation succeeds without needing its removed source archive."""
        fetch_model.fetch_model(
            self.manifest,
            self.output,
            cache_dir=self.cache,
        )
        self.archive.unlink()
        fetch_model.fetch_model(
            self.manifest,
            self.output,
            cache_dir=self.cache,
            offline=True,
        )
        self.assertEqual(
            (self.output / "meshes/robot.stl").read_bytes(),
            self.payload,
        )

    def test_repairs_a_corrupt_installation_from_verified_cache(self) -> None:
        """Checksum drift triggers an atomic reinstall from the immutable cache."""
        fetch_model.fetch_model(
            self.manifest,
            self.output,
            cache_dir=self.cache,
        )
        (self.output / "meshes/robot.stl").write_bytes(b"corrupt")
        fetch_model.fetch_model(
            self.manifest,
            self.output,
            cache_dir=self.cache,
            offline=True,
        )
        self.assertEqual(
            (self.output / "meshes/robot.stl").read_bytes(),
            self.payload,
        )

    def test_rejects_unexpected_archive_member(self) -> None:
        """An archive cannot smuggle undeclared files into a managed model root."""
        self._write_archive(
            {
                "meshes/robot.stl": self.payload,
                "meshes/undeclared.stl": b"undeclared",
            }
        )
        self._write_manifest()
        with self.assertRaisesRegex(ValueError, "unexpected archive member"):
            fetch_model.fetch_model(
                self.manifest,
                self.output,
                cache_dir=self.cache,
            )

    def test_rejects_traversing_manifest_path(self) -> None:
        """Manifest paths are validated before any network access starts."""
        document = json.loads(self.manifest.read_text(encoding="utf-8"))
        document["files"]["../outside.stl"] = document["files"].pop(
            "meshes/robot.stl"
        )
        self.manifest.write_text(json.dumps(document), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "invalid model file path"):
            fetch_model.load_manifest(self.manifest)

    def test_rejects_non_object_manifest(self) -> None:
        """A valid JSON value still has to use the declared object schema."""
        self.manifest.write_text("[]", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "must be a JSON object"):
            fetch_model.load_manifest(self.manifest)

    def test_restarts_full_length_unverified_partial(self) -> None:
        """A stale full-length partial is replaced instead of issuing an empty range."""
        artifact = fetch_model.load_manifest(self.manifest)
        cached = fetch_model.cache_path(self.cache, artifact)
        cached.parent.mkdir(parents=True)
        partial = cached.with_name(f"{cached.name}.part")
        partial.write_bytes(b"x" * artifact.size_bytes)

        fetch_model.fetch_model(
            self.manifest,
            self.output,
            cache_dir=self.cache,
        )

        self.assertEqual(fetch_model.sha256_file(cached), artifact.sha256)


if __name__ == "__main__":
    unittest.main()
