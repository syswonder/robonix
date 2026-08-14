"""Tests for ImageStore — save, list, remove observation images."""

import os
import sys
import tempfile
import unittest
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_SVC = _HERE.parent
if str(_SVC) not in sys.path:
    sys.path.insert(0, str(_SVC))

from memory_service.storage.image_store import ImageStore


class TestImageStore:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.store = ImageStore(image_root=self.tmp)

    def teardown_method(self):
        import shutil
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_save_and_list(self):
        img = b"\xff\xd8\xff\xe0fake jpeg data"
        path = self.store.save(node_id=42, image_bytes=img)
        assert "42" in path
        assert path.endswith("frame_0001.jpg")

        paths = self.store.list(42)
        assert len(paths) == 1
        assert paths[0] == path

    def test_save_multiple_frames(self):
        for i in range(3):
            self.store.save(node_id=7, image_bytes=b"img" + bytes([i]))
        paths = self.store.list(7)
        assert len(paths) == 3
        assert paths[0].endswith("frame_0001.jpg")
        assert paths[1].endswith("frame_0002.jpg")
        assert paths[2].endswith("frame_0003.jpg")

    def test_save_batch(self):
        imgs = [b"a", b"b", b"c"]
        paths = self.store.save_batch(node_id=99, images=imgs)
        assert len(paths) == 3
        assert self.store.count(99) == 3

    def test_empty_node_no_images(self):
        assert self.store.list(999) == []
        assert self.store.count(999) == 0

    def test_remove_images(self):
        self.store.save(5, b"img1")
        self.store.save(5, b"img2")
        assert self.store.count(5) == 2
        assert self.store.remove(5) is True
        assert self.store.count(5) == 0
        assert not self.store._node_dir(5).exists()

    def test_remove_nonexistent(self):
        assert self.store.remove(12345) is False

    def test_images_persist_on_disk(self):
        self.store.save(10, b"persist test")
        node_dir = self.store._node_dir(10)
        assert node_dir.exists()
        files = list(node_dir.glob("frame_*.*"))
        assert len(files) == 1
        content = files[0].read_bytes()
        assert content == b"persist test"


if __name__ == "__main__":
    import traceback
    tests = [TestImageStore()]
    passed = failed = 0
    for obj in tests:
        for name in sorted(dir(obj)):
            if name.startswith("test_"):
                try:
                    if hasattr(obj, "setup_method"):
                        obj.setup_method()
                    getattr(obj, name)()
                    if hasattr(obj, "teardown_method"):
                        obj.teardown_method()
                    print(f"  PASS {type(obj).__name__}.{name}")
                    passed += 1
                except Exception:
                    print(f"  FAIL {type(obj).__name__}.{name}")
                    traceback.print_exc()
                    failed += 1
                    if hasattr(obj, "teardown_method"):
                        obj.teardown_method()
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
