# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for the epoch sidecar (scene_service.map_meta).

Pure Python + tmp dirs — always runs, no ROS / milvus needed.
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.map_meta import (  # noqa: E402
    MapMetaStore,
    MapSemanticMeta,
    make_meta,
)


def test_roundtrip_and_seq_progression():
    """write → read preserves fields; next_partition advances only after a
    commit, and hands the SAME token out again after a failed attempt."""
    with tempfile.TemporaryDirectory() as d:
        store = MapMetaStore(d)
        assert store.read("labx") is None

        part1, seq1 = store.next_partition("labx")
        assert (part1, seq1) == ("labx__s1", 1)
        # A failed save never called write() → token is reused, not burned.
        assert store.next_partition("labx") == ("labx__s1", 1)

        store.write(make_meta("labx", part1, seq1, generation=3, mode="mapping"))
        meta = store.read("labx")
        assert meta is not None
        assert meta.object_partition == "labx__s1"
        assert meta.save_seq == 1
        assert meta.mapping_generation == 3
        assert meta.mapping_mode == "mapping"
        assert meta.saved_at_unix > 0

        part2, seq2 = store.next_partition("labx")
        assert (part2, seq2) == ("labx__s2", 2)
        # Maps do not share sequences.
        assert store.next_partition("other") == ("other__s1", 1)
    print("  [PASS] test_roundtrip_and_seq_progression")


def test_corrupt_sidecar_treated_as_absent():
    """A malformed sidecar degrades to 'no snapshot' (restore nothing) and
    the sequence restarts — it must never crash a Load."""
    with tempfile.TemporaryDirectory() as d:
        store = MapMetaStore(d)
        path = os.path.join(d, "labx.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write("{not json")
        assert store.read("labx") is None
        assert store.next_partition("labx") == ("labx__s1", 1)
        # Parseable JSON but no usable partition → same treatment.
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"map_id": "labx", "object_partition": ""}, f)
        assert store.read("labx") is None
    print("  [PASS] test_corrupt_sidecar_treated_as_absent")


def test_delete_and_sanitization():
    with tempfile.TemporaryDirectory() as d:
        store = MapMetaStore(d)
        store.write(make_meta("labx", "labx__s1", 1))
        assert store.delete("labx") is True
        assert store.read("labx") is None
        assert store.delete("labx") is False

        # Hostile ids are sanitized the same way everywhere.
        part, _seq = store.next_partition('a/b "c')
        assert "/" not in part and '"' not in part
        store.write(make_meta('a/b "c', part, 1))
        assert store.read('a/b "c').object_partition == part
    print("  [PASS] test_delete_and_sanitization")


def test_unknown_keys_dropped():
    """A sidecar written by a NEWER scene still loads (forward-tolerant)."""
    m = MapSemanticMeta.from_json({
        "map_id": "m", "object_partition": "m__s1", "save_seq": 1,
        "saved_at_unix": 1.0, "written_by_future_scene": True,
    })
    assert m.object_partition == "m__s1"
    assert not hasattr(m, "written_by_future_scene")
    print("  [PASS] test_unknown_keys_dropped")


if __name__ == "__main__":
    print("Running map_meta unit tests...\n")
    test_roundtrip_and_seq_progression()
    test_corrupt_sidecar_treated_as_absent()
    test_delete_and_sanitization()
    test_unknown_keys_dropped()
    print("\nAll tests passed!")
