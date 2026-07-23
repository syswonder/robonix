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


def test_foreign_identity_sidecar_treated_as_corrupt():
    """A parseable sidecar whose identity belongs to ANOTHER map (wrong
    map_id, foreign partition token, or a seq that disagrees) must degrade
    to 'no snapshot' — trusting it would let Load restore another map's
    objects, exactly the mis-anchoring the sidecar exists to prevent."""
    with tempfile.TemporaryDirectory() as d:
        store = MapMetaStore(d)
        path = os.path.join(d, "map_a.json")
        # Identity names map_b (a copied / hand-renamed sidecar file).
        with open(path, "w", encoding="utf-8") as f:
            json.dump(make_meta("map_b", "map_b__s1", 1).to_json(), f)
        assert store.read("map_a") is None
        # Right map id, but the partition is another map's token.
        with open(path, "w", encoding="utf-8") as f:
            json.dump(make_meta("map_a", "map_b__s1", 1).to_json(), f)
        assert store.read("map_a") is None
        # Partition token disagrees with the recorded save_seq.
        with open(path, "w", encoding="utf-8") as f:
            json.dump(make_meta("map_a", "map_a__s2", 1).to_json(), f)
        assert store.read("map_a") is None
        # save_seq below the token floor (tokens start at __s1).
        with open(path, "w", encoding="utf-8") as f:
            json.dump(make_meta("map_a", "map_a__s0", 0).to_json(), f)
        assert store.read("map_a") is None
        # A consistent record still reads (validation is not over-eager).
        store.write(make_meta("map_a", "map_a__s1", 1))
        assert store.read("map_a").object_partition == "map_a__s1"
    print("  [PASS] test_foreign_identity_sidecar_treated_as_corrupt")


def test_numeric_fields_normalized_on_read():
    """A foreign/hand-edited sidecar may carry save_seq or
    mapping_generation as strings or floats. read() must hand back ints —
    the raw values would poison next_partition's arithmetic ("2" + 1) or
    mint a float token ("__s3.0") that no later read accepts — and a
    value that cannot be an int makes the record corrupt, not half-used."""
    with tempfile.TemporaryDirectory() as d:
        store = MapMetaStore(d)
        path = os.path.join(d, "map_a.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"map_id": "map_a", "object_partition": "map_a__s2",
                       "save_seq": "2", "saved_at_unix": 1.0,
                       "mapping_generation": "5"}, f)
        meta = store.read("map_a")
        assert meta is not None
        assert meta.save_seq == 2 and type(meta.save_seq) is int
        assert (meta.mapping_generation == 5
                and type(meta.mapping_generation) is int)
        assert store.next_partition("map_a") == ("map_a__s3", 3)
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"map_id": "map_a", "object_partition": "map_a__s2",
                       "save_seq": 2.0, "saved_at_unix": 1.0,
                       "mapping_generation": "yes"}, f)
        assert store.read("map_a") is None
    print("  [PASS] test_numeric_fields_normalized_on_read")


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
    test_foreign_identity_sidecar_treated_as_corrupt()
    test_numeric_fields_normalized_on_read()
    test_unknown_keys_dropped()
    print("\nAll tests passed!")
